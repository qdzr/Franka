#! /usr/bin/env python

from __future__ import division, print_function

import numpy as np
import franka_control_wrappers

import rospy

import os
import time
import datetime

from tf import transformations as tft
import geometry_msgs.msg as gmsg
from std_msgs.msg import Int16
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

from franka_control_wrappers.panda_commander import PandaCommander

import dougsm_helpers.tf_helpers as tfh
from dougsm_helpers.ros_control import ControlSwitcher

from ggcnn.msg import Grasp
from ggcnn.srv import GraspPrediction

from mvp_grasping.panda_base_grasping_controller import Logger, Run, Experiment
import moveit_commander
robot = moveit_commander.RobotCommander()

Run.log_properties = ['success', 'time', 'quality']
Experiment.log_properties = ['success_rate', 'mpph']

downmove_dis = 0.201

class PandaOpenLoopGraspController(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """
    def __init__(self):
        self.base_frame = 'panda_link0'
        self.camera_frame = 'camera_depth_optical_frame'
        self.yolo_topic = '/yolov5/BoundingBoxes'
        cam_info_topic = '/camera/depth/camera_info'
        camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        self.cam_K = np.array(camera_info_msg.K).reshape((3, 3))

        ggcnn_service_name = '/ggcnn_service'
        rospy.wait_for_service(ggcnn_service_name + '/predict')
        rospy.loginfo("/ggcnn_service/predict Client start") 
        self.ggcnn_srv = rospy.ServiceProxy(ggcnn_service_name + '/predict', GraspPrediction)

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
        self.max_velo = 0.10
        self.curr_velo = Twist()
        self.best_grasp = Grasp()
        # ADD
        self.last_image_pose = None

        self.cs = ControlSwitcher({'moveit': 'position_joint_trajectory_controller',
                                   'velocity': 'cartesian_velocity_node_controller'})
        self.cs.switch_controller('moveit')
        
        #self.pc = PandaCommander(group_name='panda_arm_hand')
        self.pc = PandaCommander(group_name='panda_arm')
        self.pc.print_debug_info()

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__robot_state_callback, queue_size=1)

        # Centre and above the bin
        self.pregrasp_pose = [(rospy.get_param('/grasp_entropy_node/histogram/bounds/x2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/x1'))/2 - 0.02,
                              (rospy.get_param('/grasp_entropy_node/histogram/bounds/y2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/y1'))/2 + 0.03,
                              rospy.get_param('/grasp_entropy_node/height/z1') + 0.07,
                              2**0.5/2, -2**0.5/2, 0, 0]
        self.pre_joints = [-0.2489083656239928, -0.8864167918155068, -0.7377001426463456, -1.9069772342338767,
                           -0.6579447935356033, 1.3590755160914527, 1.289369003293001]

        self.pre_pose = [-0.0167990443619, -0.40043176435, 0.677801884091, -0.408974670494, 0.90964484478, -0.0681539530083, -0.0253182532707]
        
        self.place_pose = [-0.012284, -0.561789, 0.00293782, -2.87222, -0.020787, 2.22821, 0.796928]
        self.test_pose = [0.7106, -0.01391, -0.0807, 0.992, -0.1256, 0, 0]
        self.last_weight = 0
        self.__weight_increase_check()

        self.experiment = Experiment()
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

    def __execute_best_grasp(self, prepose):
            print("panda_open execute_best_grasp start")
            self.cs.switch_controller('moveit')
            ret = self.ggcnn_srv.call()
            print("predict result")
            print(ret)
            print("/ggcnn_service/predict server callback start")
            if not ret.success:
                return False
            best_grasp = ret.best_grasp
            self.best_grasp = best_grasp
            # self.next_grasp = best_grasp.pose
            tfh.publish_pose_as_transform(best_grasp.pose, 'panda_link0', 'X', 0.5)

            rospy.loginfo("if enter 0, return false!")
            if raw_input('Continue?') == '0':
                return False

            # Offset for initial pose.
            initial_offset = 0.11
            LINK_EE_OFFSET = 0.17  # 0.19

            # add
            # best_grasp.pose.position.x = prepose.position.x
            # best_grasp.pose.position.y = prepose.position.y

            # change the number 0.026 to 0.030
            # Add some limits, plus a starting offset.
            # best_grasp.pose.position.z = max(best_grasp.pose.position.z - 0.01, 0.026)  # 0.021 = collision with ground
            best_grasp.pose.position.z = max(best_grasp.pose.position.z, 0.038)  # 0.021 = collision with ground
            best_grasp.pose.position.z += initial_offset + LINK_EE_OFFSET  # Offset from end efector position to

            #self.next_grasp.position.z = self.next_grasp.position.z - downmove_dis
            # self.pc.set_gripper(best_grasp.width, wait=False)
           
            # rospy.sleep(0.1)
            rospy.loginfo("panda is going to best grasp pose")
            self.pc.goto_pose(best_grasp.pose, velocity=0.1)
            
            best_grasp.pose.position.z = best_grasp.pose.position.z - LINK_EE_OFFSET
            self.pc.goto_pose(best_grasp.pose, velocity=0.1)
            self.pc.set_gripper(best_grasp.width, wait=False)
            #self.pc.grasp(0, force=2)
            #rospy.loginfo("This is the current gripper figger pose")
            frigger_width = robot.get_current_state().joint_state.position[7]
           
            i = 0
            bias = 0.01
            while i < 4 and frigger_width < 1e-2:
                rospy.loginfo("open gripper and grasp again due to bias")
                self.pc.set_gripper(0.08)
                best_grasp.pose.position.z = best_grasp.pose.position.z -bias
                self.pc.goto_pose(best_grasp.pose, velocity=0.03)
                self.pc.grasp(0, force=2)
                # rospy.sleep(0.1)
               
                i = i + 1
                frigger_width = robot.get_current_state().joint_state.position[7]
                print(frigger_width)
                if frigger_width > 8 * 1e-3:
                    rospy.sleep(0.1)
                if self.ROBOT_ERROR_DETECTED:
                    break
            
            rospy.sleep(0.1)
            # Reset the position
            best_grasp.pose.position.z -= initial_offset + LINK_EE_OFFSET

            self.cs.switch_controller('velocity')
            v = Twist()
            v.linear.z = -0.05

            # Monitor robot state and descend
            while self.robot_state.O_T_EE[-2] > best_grasp.pose.position.z and not any(self.robot_state.cartesian_contact) and not self.ROBOT_ERROR_DETECTED:
                self.curr_velo_pub.publish(v)
                rospy.sleep(0.01)
            v.linear.z = 0
            self.curr_velo_pub.publish(v)
            
            # Check for collision
            if self.ROBOT_ERROR_DETECTED:
                # Tis code is added to try grasp again after collision by adding position.z
                # The original code:
                return False

            # close the fingers.
            rospy.sleep(0.2)
            print("sleep")
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

    def yolo_callback(self, boxes):
        rospy.loginfo("callback")
        objects = []
        boxs = boxes.bounding_boxes
        for box in boxs:
            objects.append(box.Class)
        print("Detected: ", objects)
        grasp_object = str(input("Please enter object: "))
        print(grasp_object)
        print(type(grasp_object))
        for box in boxs:
            if box.Class == grasp_object:
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                xcenter = int((xmin + xmax) / 2)
                ycenter = int((ymin + ymax) / 2)
                break
            else:
                continue
        return xcenter, ycenter

    def _yolo_process(self):
        boundingboxes = rospy.wait_for_message(self.yolo_topic, BoundingBoxes)
        u, v = self.yolo_callback(boundingboxes)
        zc = 0.62
        xc = (u - self.cam_K[0, 2]) / self.cam_K[0, 0] * zc
        yc = (v + 35 - self.cam_K[1, 2]) / self.cam_K[1, 1] * zc

        self.last_image_pose = tfh.current_robot_pose(self.base_frame, self.camera_frame)
        camera_pose = self.last_image_pose
        camera_rot = tft.quaternion_matrix(tfh.quaternion_to_list(camera_pose.orientation))[0:3, 0:3]
        cam_p = camera_pose.position
        pos = np.dot(camera_rot, np.stack((xc, yc, zc))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])

        pre_ret_grasp = gmsg.Pose()
        pre_ret_grasp.position.x = pos[0, 0]
        pre_ret_grasp.position.y = pos[0, 1]
        pre_ret_grasp.position.z = pos[0, 2]

        pre_ret_grasp.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, - 3 * np.pi / 4))

        return pre_ret_grasp

    def go(self):
        raw_input('Press Enter to Start.')
        while not rospy.is_shutdown():
            self.cs.switch_controller('moveit')
            # self.pc.goto_pose(self.test_pose, velocity=0.1)
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            rospy.loginfo("panda will move to pregrasp_pose")
            self.pc.goto_pose(self.pregrasp_pose, velocity=0.1)
            # self.pc.goto_pose(self.pre_pose, velocity=0.1)
            # self.pc.goto_joints(self.pre_joints)
            print(self.pregrasp_pose)

            rospy.loginfo("panda gripper init")
            self.pc.set_gripper(0.01)
            self.pc.set_gripper(0.08)


            # according to yolo topic, change pregrasp_pose
            rospy.loginfo("according to yolo topic, change pregrasp_pose")
            self.pre_ret_pose = self._yolo_process()
            print("pre_ret_pose: ")
            print(self.pre_ret_pose)

            self.pre_ret_pose.position.z = self.pre_ret_pose.position.z + 0.52

            self.pc.goto_pose(self.pre_ret_pose, velocity=0.1)
            raw_input('Press Enter to Start.')

            self.cs.switch_controller('velocity')
            
            run = self.experiment.new_run()
            run.start()
            grasp_ret = self.__execute_best_grasp(self.pre_ret_pose)
            run.stop()
            
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
    # p_pose = tfh.convert_pose(p, 'panda_link0', 'panda_link8')

    # print(p)
    rospy.loginfo("panda_open_loop_grasp node init")
    pg = PandaOpenLoopGraspController()
    pg.go()
