#! /usr/bin/env python

from __future__ import division, print_function
import franka_control_wrappers

import rospy

import os
import time
import datetime

import geometry_msgs.msg as gmsg
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors

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
        ggcnn_service_name = '/ggcnn_service'
        rospy.wait_for_service(ggcnn_service_name + '/predict')
        rospy.loginfo("/ggcnn_service/predict Client start") 
        self.ggcnn_srv = rospy.ServiceProxy(ggcnn_service_name + '/predict', GraspPrediction)
        print("test node 0")

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
        self.max_velo = 0.10
        self.curr_velo = Twist()
        self.best_grasp = Grasp()
        
        print("test node 01")
        # self.cs = ControlSwitcher({'moveit': 'position_joint_trajectory_controller',
        #                            'velocity': 'cartesian_velocity_node_controller'})
        # self.cs.switch_controller('moveit')
        print("test node 1")
        
        #self.pc = PandaCommander(group_name='panda_arm_hand')
        self.pc = PandaCommander(group_name='manipulator')
        self.pc.print_debug_info()
        print("test node 2")

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        # rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__robot_state_callback, queue_size=1)

        # Centre and above the bin
        # self.pregrasp_pose = [(rospy.get_param('/grasp_entropy_node/histogram/bounds/x2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/x1'))/2 - 0.02,
        #                       (rospy.get_param('/grasp_entropy_node/histogram/bounds/y2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/y1'))/2 + 0.03,
        #                       rospy.get_param('/grasp_entropy_node/height/z1') + 0.07,
        #                       2**0.5/2, -2**0.5/2, 0, 0]
        self.pre_joints = [-0.155125, -0.1780026, -1.20847, 0.0, -1.57079, 0.0]

        # self.pre_pose = [-0.155125, -0.1780026, -1.20847, 0.0, -1.57079, 0.0]
        
        # self.place_pose = [-0.012284, -0.561789, 0.00293782, -2.87222, -0.020787, 2.22821, 0.796928]
        # self.test_pose = [0.7106, -0.01391, -0.0807, 0.992, -0.1256, 0, 0]
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

    def __execute_best_grasp(self):
            print("redwallbot_open execute_best_grasp start")
            # self.cs.switch_controller('moveit')
            ret = self.ggcnn_srv.call()
            print("predict result")
            print(ret)
            print("/ggcnn_service/predict server callback start")
            if not ret.success:
                return False
            best_grasp = ret.best_grasp
            self.best_grasp = best_grasp
            # self.next_grasp = best_grasp.pose
            tfh.publish_pose_as_transform(best_grasp.pose, 'base_link', 'X', 0.5)

            rospy.loginfo("if enter 0, return false!")
            if raw_input('Continue?') == '0':
                return False

            # Offset for initial pose.
            initial_offset = 0.11
            LINK_EE_OFFSET = 0.19

            # change the number 0.026 to 0.030
            # Add some limits, plus a starting offset.
            # best_grasp.pose.position.z = max(best_grasp.pose.position.z - 0.01, 0.026)  # 0.021 = collision with ground
            best_grasp.pose.position.z = max(best_grasp.pose.position.z, 0.030)  # 0.021 = collision with ground
            # best_grasp.pose.position.z += initial_offset + LINK_EE_OFFSET  # Offset from end efector position to
            best_grasp.pose.position.z += initial_offset

            #self.next_grasp.position.z = self.next_grasp.position.z - downmove_dis
            # self.pc.set_gripper(best_grasp.width, wait=False)
           
            # rospy.sleep(0.1)
            rospy.loginfo("Robot is going to best grasp pose")
            self.pc.goto_pose(best_grasp.pose, velocity=0.1)
            
            # best_grasp.pose.position.z = best_grasp.pose.position.z - LINK_EE_OFFSET
            # self.pc.goto_pose(best_grasp.pose, velocity=0.1)

            rospy.loginfo("Grasp! if enter 0, return false!")
            if raw_input('Continue?') == '0':
                return False
            # self.pc.set_gripper(best_grasp.width, wait=False)
            #self.pc.grasp(0, force=2)
            #rospy.loginfo("This is the current gripper figger pose")
            frigger_width = robot.get_current_state().joint_state.position[7]
            #print(frigger_width)
            #self.test_width = best_grasp.width
            # if frigger_width < 1e-1:
            # rospy.loginfo("grasp again due to bias")
            # self.pc.set_gripper(0.08, wait=False)
            i = 0
            bias = 0.01
            while i < 4 and frigger_width < 1e-2:
                rospy.loginfo("open gripper and grasp again due to bias")
                self.pc.set_gripper(0.08)
                best_grasp.pose.position.z = best_grasp.pose.position.z -bias
                self.pc.goto_pose(best_grasp.pose, velocity=0.03)
                self.pc.grasp(0, force=2)
                # rospy.sleep(0.1)
                # self.pc.set_gripper(best_grasp.width, wait=False)
                # self.pc.set_gripper(self.test_width, wait=False)
                # print(best_grasp.width)
                # print(self.test_width)
                i = i + 1
                #robot = moveit_commander.RobotCommander()
                #global robot
                frigger_width = robot.get_current_state().joint_state.position[7]
                print(frigger_width)
                if frigger_width > 8 * 1e-3:
                    rospy.sleep(0.1)
                if self.ROBOT_ERROR_DETECTED:
                    break
            
            rospy.sleep(0.1)
            # Reset the position
            best_grasp.pose.position.z -= initial_offset + LINK_EE_OFFSET

            # self.cs.switch_controller('velocity')
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
                # rospy.loginfo("grasp again due to robot_error_detection")
                # bias = 0.015
                # self.pc.set_gripper(0.08, wait=False)
                # best_grasp.pose.position.z = best_grasp.pose.position.z + bias
                # self.pc.goto_pose(best_grasp.pose, velocity=0.03)
                # self.pc.set_gripper(best_grasp.width, wait=False)
                # if self.ROBOT_ERROR_DETECTED:
                #     return False
                # else:
                    # self.ROBOT_ERROR_DETECTED = False

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

    def go(self):
        raw_input('Press Enter to Start.')
        while not rospy.is_shutdown():
            # self.cs.switch_controller('moveit')
            # self.pc.goto_pose(self.test_pose, velocity=0.1)
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            rospy.loginfo("panda will move to pregrasp_pose")
            # self.pc.goto_pose(self.pregrasp_pose, velocity=0.1)
            # self.pc.goto_pose(self.pre_pose, velocity=0.1)
            self.pc.goto_joints(self.pre_joints)

            rospy.loginfo("Robot gripper init")
            # self.pc.set_gripper(0.01)
            # self.pc.set_gripper(0.08)

            # self.cs.switch_controller('velocity')
            
            run = self.experiment.new_run()
            run.start()
            grasp_ret = self.__execute_best_grasp()
            run.stop()
            
            if not grasp_ret or self.ROBOT_ERROR_DETECTED:
                rospy.logerr('Something went wrong, aborting this run')
                if self.ROBOT_ERROR_DETECTED:
                    self.__recover_robot_from_error()
                continue

            # Release Object
            # self.cs.switch_controller('moveit')
            rospy.loginfo("Robot will come back to place pose")
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            self.pc.goto_joints(self.place_pose, group_name='manipulator')
            # self.pc.set_gripper(0.07)
            # self.pc.goto_named_pose('drop_box', velocity=0.1)
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
