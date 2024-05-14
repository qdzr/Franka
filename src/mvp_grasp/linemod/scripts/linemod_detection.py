#!/usr/bin/env python
#coding=utf-8
import franka_control_wrappers
import os
import time
import datetime
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
from franka_control_wrappers.panda_commander import PandaCommander
from dougsm_helpers.ros_control import ControlSwitcher
from ggcnn.msg import Grasp
#from mvp_grasping.panda_base_grasping_controller import Logger, Run, Experiment
import moveit_commander
import rospy
import numpy as np
from object_recognition_msgs.msg import RecognizedObjectArray
from tf import transformations as tft
import geometry_msgs.msg as gmsg
from ggcnn.srv import GraspPrediction, GraspPredictionResponse
import dougsm_helpers.tf_helpers as tfh
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

def linemod_callback():
    global robot_cam_pose, confidence, camera_rot, object_rot, robot_matrix_3d, cam_obj_pose
    robot_cam_pose = tfh.current_robot_pose(base_frame, camera_frame)
    robot_cam_pose_p = robot_cam_pose.position
    robot_cam_pose_rot = robot_cam_pose.orientation
    print('robot_cam_pose: ', robot_cam_pose)
    rospy.loginfo("object detected")
    # rospy.loginfo(rospy.get_caller_id() + ' heard %s', cam_obj_pose)
    # cam_obj_pose = msg.objects[0].pose.pose.pose
    confidence = msg.objects[0].confidence
    rospy.loginfo('confidence: %f', confidence)
    global robot_obj_pose
    robot_obj_pose = gmsg.Pose()
    camera_rot = tft.quaternion_matrix(tfh.quaternion_to_list(robot_cam_pose.orientation))[0:3, 0:3]
    pos = np.dot(camera_rot, np.stack((cam_obj_pose.position.x, cam_obj_pose.position.y, 
                 cam_obj_pose.position.z))).T + np.array([[robot_cam_pose.position.x, 
                 robot_cam_pose.position.y, robot_cam_pose.position.z]])
    print('pos shape:', type(pos))
    print(pos)
    robot_obj_pose.position.x = pos[0, 0]
    robot_obj_pose.position.y = pos[0, 1]
    robot_obj_pose.position.z = pos[0, 2]
    angle_cam = -np.arcsin(camera_rot[0, 1])
    angle = (angle_cam + np.pi/2) % np.pi - np.pi
    print('angle:', angle_cam-np.pi/2)
    ready_orientation = [0.924262, -0.381359, -0.0120597, 0.012619]
    # robot_obj_pose.orientation.x = ready_orientation[0]
    # robot_obj_pose.orientation.y = ready_orientation[1]
    # robot_obj_pose.orientation.z = ready_orientation[2]
    # robot_obj_pose.orientation.w = ready_orientation[3]
    robot_obj_pose.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, angle_cam- np.pi/2))
    print(robot_obj_pose)
    

class linemod_grasp(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """
    def __init__(self):
        rospy.loginfo("grasp start") 
        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
        # self.msg_sub = rospy.Subscriber('/recognized_object_array', RecognizedObjectArray, linemod_callback, queue_size=1)
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
        # rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__robot_state_callback, queue_size=1)
        
        self.place_pose = [-0.012284, -0.561789, 0.00293782, -2.87222, -0.020787, 2.22821, 0.796928]
        self.start_pose = [0.420897846403, -0.00813912899792, 0.660206857147,
                            -0.936174235237, 0.327151150622, -0.112307056463, 0.0627459241222]
        self.ready_pose = [0.459808189148, -0.298772024783, 0.545817317653,
                             -0.792093406959, 0.575342422524, -0.0256140412745, 0.202269751537]
        self.ready_pose1 = [0.48307912876, 0.0670036298594, 0.604352453705,
                             0.892269661958, -0.442148090349, 0.0892527950362, 0.0198457836357]
        self.pre_pose = [0.636556191571, 0.00653153850712, 0.600565816469, 
                          0.92163234798, -0.38794839485, -0.000923922460956, 0.00943421738002]
        self.test_pose = gmsg.Pose()
        self.test_pose.position.x = 0.420897846403
        self.test_pose.position.y = -0.00813912899792
        self.test_pose.position.z = 0.660206857147
        self.test_pose.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, -0.785266))
        #self.last_weight = 0
        #self.__weight_increase_check()
        
        #self.experiment = Experiment()
        rospy.loginfo("linemod_detection_grasp init end")

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
            global robot_obj_pose
            self.cs.switch_controller('moveit')
            # print('test')
            # self.pc.goto_pose(self.start_pose, velocity=0.1)
            #best_grasp = Grasp()
           
            #test_grasp.pose = self.test_pose
            ###self.best_grasp.pose = best_grasp
            # self.next_grasp = best_grasp.pose
            #robot_obj_pose =self.test_pose

            # tfh.publish_pose_as_transform(robot_obj_pose, 'panda_link0', 'G', 0.5)

            # if raw_input('Continue?') == '0':
            #      rospy.loginfo("if enter 0, return false!")
            #      return False

            # Offset for initial pose.
            initial_offset = 0.10
            LINK_EE_OFFSET = 0.138

            # change the number 0.026 to 0.030
            # Add some limits, plus a starting offset.
            robot_obj_pose.position.z = max(robot_obj_pose.position.z - 0.01, 0.026)  # 0.021 = collision with ground

            # best_grasp.pose.position.z = max(best_grasp.pose.position.z - 0.01, 0.030)  # 0.021 = collision with ground
            # robot_obj_pose.position.z += initial_offset + LINK_EE_OFFSET  # Offset from end efector position to
            robot_obj_pose.position.z += initial_offset - 0.12
            #self.next_grasp.position.z = self.next_grasp.position.z - downmove_dis
            # self.pc.set_gripper(best_grasp.width, wait=False)
           
            # rospy.sleep(0.1)
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            # print("pose p")
            # self.pc.goto_pose(p, velocity=0.1)
            # print(self.test_pose)
            #self.pc.goto_pose(best_grasp.pose, velocity=0.1)
            rospy.loginfo("panda is going to best grasp pose")
            self.pc.goto_pose(robot_obj_pose, velocity=0.15)
            
            
            # best_grasp.pose.position.z = best_grasp.pose.position.z - downmove_dis
            # self.pc.goto_pose(best_grasp.pose, velocity=0.05)

            self.pc.set_gripper(0.07, wait=False)
            #self.pc.grasp(0, force=2)
            #rospy.loginfo("This is the current gripper figger pose")
            # frigger_width = robot.get_current_state().joint_state.position[7]
            #print(frigger_width)
            #self.test_width = best_grasp.width
            # if frigger_width < 1e-1:
            # rospy.loginfo("grasp again due to bias")
            # self.pc.set_gripper(0.08, wait=False)
            rospy.sleep(0.1)
            # Reset the position
            robot_obj_pose.position.z -= initial_offset + LINK_EE_OFFSET

            self.cs.switch_controller('velocity')
            v = Twist()
            v.linear.z = -0.05

            # Monitor robot state and descend
            # while self.robot_state.O_T_EE[-2] > best_grasp.pose.position.z and not any(self.robot_state.cartesian_contact) and not self.ROBOT_ERROR_DETECTED:
            #     self.curr_velo_pub.publish(v)
            #     rospy.sleep(0.01)
            # v.linear.z = 0
            # self.curr_velo_pub.publish(v)
            
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

    def pose(self):
        self.cs.switch_controller('moveit')
        self.pc.goto_pose(self.start_pose, velocity=0.1)
        # self.pc.goto_pose(self.pre_pose, velocity=0.1)
        rospy.sleep(2.0)
        self.pc.goto_pose(self.test_pose, velocity=0.1)
        self.cs.switch_controller('velocity')


    def go(self):
        raw_input('Press Enter to Start.')
        while not rospy.is_shutdown():
            self.cs.switch_controller('moveit')
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            rospy.loginfo("panda will move to pregrasp_pose")
            # self.pc.goto_pose(q, velocity=0.1)
            print('start')

            self.pc.goto_pose(self.pre_pose, velocity=0.2)
            rospy.sleep(2.0)
            # self.pc.goto_pose(robot_obj_pose, velocity=0.1)
            rospy.loginfo("panda gripper init")
            self.pc.set_gripper(0.08)

            self.cs.switch_controller('velocity')
            global pos_get
            self.get_msg(pos_get)
            #run = self.experiment.new_run()
            #run.start()
            grasp_ret = self.__execute_best_grasp()
            #run.stop()
            
            # if not grasp_ret or self.ROBOT_ERROR_DETECTED:
            #     rospy.logerr('Something went wrong, aborting this run')
            #     if self.ROBOT_ERROR_DETECTED:
            #         self.__recover_robot_from_error()
            #     continue

            # Release Object
            self.cs.switch_controller('moveit')
            rospy.loginfo("panda will come back to ready pose")
            # self.pc.goto_named_pose('grip_ready', velocity=0.1)
            self.pc.goto_joints(self.place_pose, group_name='panda_arm')
            self.pc.set_gripper(0.07)
            pos_get = True
            # self.pc.goto_named_pose('drop_box', velocity=0.1)
            # self.pc.set_gripper(0.07)
            self.cs.switch_controller('velocity')
            
            # Check success using the scales.
            rospy.sleep(1.0)
            #grasp_success = self.__weight_increase_check()
            # if not grasp_success:
            #     rospy.logerr("Failed Grasp")
            # else:
            #     rospy.logerr("Successful Grasp")

            #run.success = grasp_success
            #run.quality = self.best_grasp.quality
            #run.save()

    def get_msg(self, pos_get):
        self.pose()
        while not rospy.is_shutdown() and pos_get:
            msg = rospy.wait_for_message('/recognized_object_array', RecognizedObjectArray, timeout=2)
            if len(msg.objects) > 0 and int(input('if right pose, enter 1: ')):
                cam_obj_pose = msg.objects[0].pose.pose.pose
                confidence = msg.objects[0].confidence
                rospy.loginfo('hears %s', cam_obj_pose)
                rospy.loginfo('confidence: %f', confidence)
                pos_get = False
            else:
                rospy.loginfo("nothing detected")
 
if __name__ == '__main__':
    rospy.init_node('linemod_trans')
    rospy.loginfo('start the node to transform pose')
    pg = linemod_grasp()
    pg.pose()
    rospy.loginfo('waiting for the topic /recognized_object_array')
    pos_get = True
    # msg = rospy.wait_for_message('/recognized_object_array', RecognizedObjectArray, timeout=2).objects[0].pose.pose.pose
    while not rospy.is_shutdown() and pos_get:
        msg = rospy.wait_for_message('/recognized_object_array', RecognizedObjectArray, timeout=2)
        if len(msg.objects) > 0 and int(input('if right pose, enter 1: ')):
            cam_obj_pose = msg.objects[0].pose.pose.pose
            confidence = msg.objects[0].confidence
            rospy.loginfo('hears %s', cam_obj_pose)
            rospy.loginfo('confidence: %f', confidence)
            pos_get = False
        else:
            rospy.loginfo("nothing detected")

    # rospy.Subscriber('/recognized_object_array', RecognizedObjectArray, linemod_callback, queue_size=1)
    linemod_callback()
    pg.go()
