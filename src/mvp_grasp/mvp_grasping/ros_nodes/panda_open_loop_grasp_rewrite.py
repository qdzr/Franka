#! /usr/bin/env python

from __future__ import division, print_function
import franka_control_wrappers

import rospy

import os
import time
import datetime

import geometry_msgs.msg as gmsg
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from franka_msgs.msg import FrankaState, Errors as FrankaErrors

from franka_control_wrappers.panda_commander import PandaCommander

import dougsm_helpers.tf_helpers as tfh
from dougsm_helpers.ros_control import ControlSwitcher
from tf import transformations as tft

from ggcnn.msg import Grasp
from ggcnn.srv import GraspPrediction

from mvp_grasping.panda_base_grasping_controller import Logger, Run, Experiment
import moveit_commander
# robot = moveit_commander.RobotCommander()

Run.log_properties = ['success', 'time', 'quality']
Experiment.log_properties = ['success_rate', 'mpph']

downmove_dis = 0.154

class PandaOpenLoopGraspController(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """
    def __init__(self):
        ggcnn_service_name = '/ggcnn_service'
        self.base_frame = 'panda_link0'
        self.camera_frame = 'camera_depth_optical_frame'
        # rospy.wait_for_service(ggcnn_service_name + '/predict')
        # rospy.loginfo("/ggcnn_service/predict Client start")
        # self.ggcnn_srv = rospy.ServiceProxy(ggcnn_service_name + '/predict', GraspPrediction)

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
        self.max_velo = 0.10
        self.curr_velo = Twist()
        self.best_grasp = Grasp()

        self.cs = ControlSwitcher({'moveit': 'position_joint_trajectory_controller',
                                   'velocity': 'cartesian_velocity_node_controller'})
        self.cs.switch_controller('moveit')
        
        #self.pc = PandaCommander(group_name='panda_arm_hand')
        self.pc = PandaCommander(group_name='panda_arm')

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__robot_state_callback, queue_size=1)

        # Centre and above the bin
        # self.pregrasp_pose = [(rospy.get_param('/grasp_entropy_node/histogram/bounds/x2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/x1'))/2 - 0.03,
        #                       (rospy.get_param('/grasp_entropy_node/histogram/bounds/y2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/y1'))/2 + 0.10,
        #                       rospy.get_param('/grasp_entropy_node/height/z1') + 0.05,
        #                       2**0.5/2, -2**0.5/2, 0, 0]
        
        self.place_pose = [-0.012284, -0.561789, 0.00293782, -2.87222, -0.020787, 2.22821, 0.796928]
        self.test_pose = [0.7106, -0.01391, -0.0807, 0.992, -0.1256, 0, 0]
        self.last_weight = 0
        self.__weight_increase_check()

        # self.experiment = Experiment()
        rospy.loginfo("panda_open_loop_grasp init end")

    def __recover_robot_from_error(self):
        rospy.logerr('Recovering')
        self.pc.recover()
        rospy.logerr('Done')
        self.ROBOT_ERROR_DETECTED = False

    def __weight_increase_check(self):
        #pass
        try:
            w = rospy.wait_for_message('/scales/weight', Int16, timeout=2).data
            increased = w > self.last_weight
            self.last_weight = w
            return increased
        except:
            rospy.loginfo("scales weight increase check, disable by defaults")
            rospy.loginfo("if grasp success, enter 1 to record")
            return raw_input('No weight. Success? [1/0]') == '1'

    def __robot_state_callback(self, msg):
        self.robot_state = msg
        if any(self.robot_state.cartesian_collision):
            if not self.ROBOT_ERROR_DETECTED:
                rospy.logerr('Detected Cartesian Collision')
            self.ROBOT_ERROR_DETECTED = True
        for s in FrankaErrors.__slots__:
            if getattr(msg.current_errors, s):
                self.stop()
                if not self.ROBOT_ERROR_DETECTED:
                    rospy.logerr('Robot Error Detected')
                self.ROBOT_ERROR_DETECTED = True

    def __execute_best_grasp(self):
            print("panda_open execute_best_grasp start")
            self.cs.switch_controller('moveit')
            # ret = self.ggcnn_srv.call()
            ret = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray, timeout=30)
            print("predict result")
            if not ret.data:
                return False
            data = list(ret.data)
            x, y, z, angle, width, _ = data
            self.last_image_pose = tfh.current_robot_pose(self.base_frame, self.camera_frame)
            camera_pose = self.last_image_pose
            cam_p = camera_pose.position
            camera_rot = tft.quaternion_matrix(tfh.quaternion_to_list(camera_pose.orientation))[0:3, 0:3]
            angle -= np.arcsin(camera_rot[0, 1])  # Correct for the rotation of the camera
            angle = (angle + np.pi / 2) % np.pi - np.pi / 2  # Wrap [-np.pi/2, np.pi/2]
            pos = np.dot(camera_rot, np.stack((x, y, z))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])

            self.best_grasp = gmsg.Pose()
            self.best_grasp.position.x = pos[0, 0]
            self.best_grasp.position.y = pos[0, 1]
            self.best_grasp.position.z = pos[0, 2]
            self.best_grasp.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0,  (angle % np.pi - np.pi / 2)))
            tfh.publish_pose_as_transform(self.best_grasp, 'panda_link0', 'X', 0.5)

            if raw_input('Continue?') == '0':
                rospy.loginfo("if enter 0, return false!")
                return False

            # Offset for initial pose.
            initial_offset = 0.10
            LINK_EE_OFFSET = 0.138

            # change the number 0.026 to 0.030
            # Add some limits, plus a starting offset.
            # best_grasp.pose.position.z = max(best_grasp.pose.position.z - 0.01, 0.026)  # 0.021 = collision with ground
            self.best_grasp.position.z = max(self.best_grasp.position.z, 0.035)  # 0.021 = collision with ground
            self.best_grasp.position.z += 0.05  # Offset from end efector position to

            #self.next_grasp.position.z = self.next_grasp.position.z - downmove_dis
            # self.pc.set_gripper(best_grasp.width, wait=False)
           
            # rospy.sleep(0.1)
            rospy.loginfo("panda is going to best grasp pose")
            self.pc.goto_pose(self.best_grasp, velocity=0.1)

            rospy.loginfo('move done')
            self.best_grasp.position.z = self.best_grasp.position.z - downmove_dis
            self.pc.goto_pose(self.best_grasp, velocity=0.1)
            self.pc.set_gripper(width, wait=False)
            #self.pc.grasp(0, force=2)
            
            rospy.sleep(0.1)
            # Reset the position
            self.best_grasp.position.z -= initial_offset + LINK_EE_OFFSET

            self.cs.switch_controller('velocity')
            v = Twist()
            v.linear.z = -0.05

            # Monitor robot state and descend
            while self.robot_state.O_T_EE[-2] > self.best_grasp.position.z and not any(self.robot_state.cartesian_contact) and not self.ROBOT_ERROR_DETECTED:
                self.curr_velo_pub.publish(v)
                rospy.sleep(0.01)
            v.linear.z = 0
            self.curr_velo_pub.publish(v)
            
            # Check for collision
            if self.ROBOT_ERROR_DETECTED:
                return False
            # close the fingers.
            rospy.loginfo('after 200ms, close the figger')
            rospy.sleep(0.2)
            self.pc.grasp(0, force=2)

            # Sometimes triggered by closing on something that pushes the robot
            # This is two sentense same as above,I don't know if it is useful.
            if self.ROBOT_ERROR_DETECTED:
                return False

            return True

    def stop(self):
        self.pc.stop()
        self.curr_velo = Twist()
        self.curr_velo_pub.publish(self.curr_velo)

    def go(self):
        raw_input('Press Enter to Start.')
        while not rospy.is_shutdown():
            self.cs.switch_controller('moveit')
            # self.pc.goto_pose(self.test_pose, velocity=0.1)
            self.pc.goto_named_pose('grip_ready', velocity=0.1)
            rospy.loginfo("panda will move to pregrasp_pose")
            # self.pc.goto_pose(self.pregrasp_pose, velocity=0.1)
            rospy.loginfo("panda gripper init")
            self.pc.set_gripper(0.08)

            self.cs.switch_controller('velocity')
            
            # run = self.experiment.new_run()
            # run.start()
            grasp_ret = self.__execute_best_grasp()
            # run.stop()
            
            if not grasp_ret or self.ROBOT_ERROR_DETECTED:
                rospy.logerr('Something went wrong, aborting this run')
                if self.ROBOT_ERROR_DETECTED:
                    self.__recover_robot_from_error()
                continue

            # Release Object
            self.cs.switch_controller('moveit')
            rospy.loginfo("panda will come back to ready pose")
            self.pc.goto_named_pose('grip_ready', velocity=0.1)
            self.pc.goto_joints(self.place_pose, group_name='panda_arm')
            self.pc.set_gripper(0.07)
            self.pc.goto_named_pose('drop_box', velocity=0.1)
            # self.pc.set_gripper(0.07)
            
            # Check success using the scales.
            rospy.sleep(1.0)
            grasp_success = self.__weight_increase_check()
            if not grasp_success:
                rospy.logerr("Failed Grasp")
            else:
                rospy.logerr("Successful Grasp")

            #run.success = grasp_success
            #run.quality = self.best_grasp.quality
            #run.save()


if __name__ == '__main__':
    rospy.init_node('panda_open_loop_grasp')
    p = gmsg.Pose()
    p.position.x = -0.216084427945
    p.position.y = 0.216234246393
    p.position.z = 0.589984428162
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.5
    p.orientation.w = 0.0
    rospy.loginfo("panda_open_loop_grasp node init")
    pg = PandaOpenLoopGraspController()
    pg.go()
