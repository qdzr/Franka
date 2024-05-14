#!/usr/bin/env python
# coding=utf-8
import franka_control_wrappers
import os
import time
import datetime
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
from panda_grasp import Panda_moveit, current_robot_pose, list_to_quaternion, quaternion_to_list
from dougsm_helpers.ros_control import ControlSwitcher
from ggcnn.msg import Grasp
# from mvp_grasping.panda_base_grasping_controller import Logger, Run, Experiment
import moveit_commander
import rospy
import numpy as np
from tf import transformations as tft
import geometry_msgs.msg as gmsg
# from sensor_msgs.msg import Image, CameraInfo
base_frame = 'panda_link0'
camera_frame = 'camera_color_optical_frame'
robot_cam_pose = None
# cam_obj_pose = None
confidence = None
camera_rot = None
object_rot = None
robot_matrix_3d = None
grasp_2d = True


# robot = moveit_commander.RobotCommander()


class find_object_2d_grasp(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """

    def __init__(self):
        rospy.loginfo("robot plan start")
        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist,
                                             queue_size=1)
        self.max_velo = 0.10
        self.curr_velo = Twist()
        self.best_grasp = Grasp()

        self.cs = ControlSwitcher({'moveit': 'position_joint_trajectory_controller',
                                   'velocity': 'cartesian_velocity_node_controller'})
        self.cs.switch_controller('moveit')
        self.pc = Panda_moveit(group_name='panda_arm')

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__robot_state_callback,
                         queue_size=1)

        self.place_pose = [-0.012284, -0.561789, 0.00293782, -2.87222, -0.020787, 2.22821, 0.796928]
        self.start_pose = [0.600918633073, 0.0798243177083, 0.699749050227,
                           -0.926278989853, 0.376706391303, 0.00243192964424, -0.00967540322904]

        rospy.loginfo("find object 2d to grasp init end")
        self.test_pose = gmsg.Pose()
        self.test_pose.position.x = 0.420897846403
        self.test_pose.position.y = -0.00813912899792
        self.test_pose.position.z = 0.660206857147
        self.test_pose.orientation.x = -0.936174235237
        self.test_pose.orientation.y = 0.327151150622
        self.test_pose.orientation.z = -0.112307056463
        self.test_pose.orientation.w = 0.0627459241222

    def __recover_robot_from_error(self):
        rospy.logerr('Recovering')
        self.pc.recover()
        rospy.logerr('Done')
        self.ROBOT_ERROR_DETECTED = False

    def __weight_increase_check(self):
        # pass
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
        global robot_obj_pose
        self.cs.switch_controller('moveit')
        # tfh.publish_pose_as_transform(robot_obj_pose, 'panda_link0', 'G', 0.5)

        # Offset for initial pose.
        initial_offset = 0.10
        LINK_EE_OFFSET = 0.138

        # change the number 0.026 to 0.030
        # Add some limits, plus a starting offset.
        robot_obj_pose.position.z = max(robot_obj_pose.position.z, 0.033)  # 0.021 = collision with ground
        robot_obj_pose.position.z = 0.0335
        robot_obj_pose.orientation = list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, -np.pi / 4))
        # robot_obj_pose.position.z += initial_offset + LINK_EE_OFFSET  # Offset from end efector position to
        print("robot_obj_pose: ", robot_obj_pose)
        # self.pc.set_gripper(best_grasp.width, wait=False)
        self.pc.goto_pose(robot_obj_pose, velocity=0.1)
        rospy.loginfo("panda is going to best grasp pose")

        # best_grasp.pose.position.z = best_grasp.pose.position.z - downmove_dis
        # self.pc.goto_pose(best_grasp.pose, velocity=0.05)

        self.pc.set_gripper(0.07, wait=False)
        # self.pc.grasp(0, force=2)

        rospy.sleep(0.1)
        # Reset the position
        robot_obj_pose.position.z -= initial_offset + LINK_EE_OFFSET

        self.cs.switch_controller('velocity')
        v = Twist()
        v.linear.z = -0.05

        # Monitor robot state and descend
        while self.robot_state.O_T_EE[-2] > robot_obj_pose.position.z and not any(
                self.robot_state.cartesian_contact) and not self.ROBOT_ERROR_DETECTED:
            self.curr_velo_pub.publish(v)
            rospy.sleep(0.01)
        v.linear.z = 0
        self.curr_velo_pub.publish(v)

        # Check for collision
        if self.ROBOT_ERROR_DETECTED:
            return False

        # close the fingers.
        rospy.sleep(0.2)
        rospy.loginfo('grasp object')
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

    def ready_pose(self):
        self.cs.switch_controller('moveit')
        self.pc.goto_pose(self.start_pose, velocity=0.1)
        self.cs.switch_controller('velocity')

    def go(self):
        raw_input('Press Enter to Start.')
        while not rospy.is_shutdown():
            self.cs.switch_controller('moveit')
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            rospy.loginfo("panda will move to pregrasp_pose")
            # self.pc.goto_pose(q, velocity=0.1)
            rospy.loginfo("panda gripper init")
            self.pc.set_gripper(0.08)
            self.cs.switch_controller('velocity')
            grasp_ret = self.__execute_best_grasp()

            if not grasp_ret or self.ROBOT_ERROR_DETECTED:
                rospy.logerr('Something went wrong, aborting this run')
                if self.ROBOT_ERROR_DETECTED:
                    self.__recover_robot_from_error()
                continue

            # Release Object
            self.cs.switch_controller('moveit')
            rospy.loginfo("panda will come back to ready pose")
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            self.pc.goto_joints(self.place_pose, group_name='panda_arm')
            self.pc.set_gripper(0.07)

            # Check success using the scales.
            rospy.sleep(1.0)
            grasp_success = self.__weight_increase_check()
            if not grasp_success:
                rospy.logerr("Failed Grasp")
            else:
                rospy.logerr("Successful Grasp")

    def move(self):
        # raw_input('Press Enter to Start.')
        self.cs.switch_controller('moveit')
        rospy.loginfo("panda will move to pregrasp_pose")
        # self.pc.goto_pose(q, velocity=0.1)
        rospy.loginfo("panda gripper init")
        self.pc.set_gripper(0.0)
        self.pc.set_gripper(0.08)
        self.cs.switch_controller('velocity')
        grasp_ret = self.__execute_best_grasp()

        if not grasp_ret or self.ROBOT_ERROR_DETECTED:
            rospy.logerr('Something went wrong, aborting this run')
            if self.ROBOT_ERROR_DETECTED:
                self.__recover_robot_from_error()

        # Release Object
        self.cs.switch_controller('moveit')
        rospy.loginfo("panda will come back to ready pose")
        # self.pc.goto_named_pose('grip_ready', velocity=0.1)
        self.pc.goto_joints(self.place_pose, group_name='panda_arm')
        self.pc.set_gripper(0.07)

        # Check success using the scales.
        rospy.sleep(1.0)
        grasp_success = self.__weight_increase_check()
        if not grasp_success:
            rospy.logerr("Failed Grasp")
        else:
            rospy.logerr("Successful Grasp")


if __name__ == '__main__':
    rospy.init_node('find_object_2d_grasp')
    rospy.loginfo('start the node to transform pose')
    global robot_obj_pose
    pg = find_object_2d_grasp()
    # pg.ready_pose()
    rospy.loginfo('waiting for the topic /objection_position_pose')
    pre = [0.492483570346, 0.0330769210778, 0.562724593622,
                     -0.918652267421, 0.394513626656, 0.0141818935886, -0.0153585103108]
    group = Panda_moveit(group_name='panda_arm')
    group.set_gripper(0.0)
    group.set_gripper(0.07)
    group.goto_pose(pre)
    raw_input("please enter to start")
    while not rospy.is_shutdown():
        robot_obj_pose = rospy.wait_for_message('/objection_position_pose', Pose, timeout=20)
        print(robot_obj_pose)
        print(type(robot_obj_pose))
        if robot_obj_pose:
            rospy.loginfo('robot_object pose publish to robot')
            pre[2] -= 0.3
            group.goto_pose(pre)
            robot_obj_pose.position.z = 0.04
            robot_obj_pose.orientation = list_to_quaternion(tft.quaternion_from_euler(np.pi, 0,  - np.pi / 4))
            print(robot_obj_pose)
            group.set_gripper(0.07)
            group.goto_pose(robot_obj_pose)
            robot_obj_pose.position.z -= 0.040
            group.goto_pose(robot_obj_pose)
            group.grasp(width=0, force=2)
            group.goto_pose(pre)
            break

        else:
            rospy.loginfo("nothing detected")

    pg.pc.close_moveit()
    rospy.loginfo("moveit done")
