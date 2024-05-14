#!/usr/bin/env python
import rospy
import sys
import tf
import actionlib
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryActionGoal
import tf2_geometry_msgs
import numpy as np
import geometry_msgs.msg as gmsg
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import moveit_commander
from moveit_commander.conversions import list_to_pose


class Panda_moveit(object):
    def __init__(self, group_name=None):
        moveit_commander.roscpp_initialize(sys.argv)
        self.reset_publisher = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal,
                                               queue_size=1)
        self.group_name = group_name
        self.robot = moveit_commander.robot.RobotCommander()
        self.group = moveit_commander.move_group.MoveGroupCommander(group_name)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.01)

    def remember_pose_set_name(self, pose_name):
        rospy.loginfo('remember pose named {}'.format(pose_name))
        self.group.remember_joint_values(pose_name)

    def print_debug_info(self):
        if self.group is not None:
            planning_frame = self.group.get_planning_frame()
            print("============ Reference frame: %s" % planning_frame)
            eef_link = self.group.get_end_effector_link()
            print("============ End effector: %s" % eef_link)
        else:
            print("============ No active planning group.")
        print("============ Robot Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def goto_joints(self, joint_values, group_name='panda_arm', wait=True):
        """
        Move to joint positions.
        :param joint_values:  Array of joint positions
        :param group_name:  Move group (use current if None)
        :param wait:  Wait for completion if True
        :return: Bool success
        """
        if not self.group:
            raise ValueError('No active Planning Group')
        joint_goal = self.group.get_current_joint_values()
        if len(joint_goal) != len(joint_values):
            raise IndexError('Expected %d Joint Values, got %d' % (len(joint_goal), len(joint_values)))
        for i, v in enumerate(joint_values):
            joint_goal[i] = v

        success = self.group.go(joint_goal, wait)
        self.group.stop()
        return success

    def goto_pose(self, pose, velocity=0.1, wait=True):
        """
         Move to pose
        :param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """
        if not self.group:
            raise ValueError('No active Planning Group')
        if type(pose) is list:
            pose = list_to_pose(pose)
        pose.position.z = max(pose.position.z, 0.0335)
        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_pose_target(pose)
        success = self.group.go(wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()
        return success

    def goto_pose_cartesian(self, pose, velocity=0.2, wait=True):
        """
        Move to pose following a cartesian trajectory.
        :param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """
        if not self.group:
            raise ValueError('No active Planning Group')

        if type(pose) is list:
            pose = list_to_pose(pose)

        self.group.set_max_velocity_scaling_factor(velocity)
        (plan, fraction) = self.group.compute_cartesian_path(
                                           [pose],   # waypoints to follow
                                           0.005,    # eef_step
                                           0.0)      # jump_threshold
        if fraction != 1.0:
            raise ValueError('Unable to plan entire path!')

        success = self.group.execute(plan, wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()
        return success

    def goto_named_pose(self, pose_name, velocity=0.1, wait=True):
        """
        Move to named pos
        :param pose: Name of named pose
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """
        if not self.group:
            raise ValueError('No active Planning Group')

        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_named_target(pose_name)
        success = self.group.go(wait=wait)
        self.group.stop()
        return success

    def shift_pose_by_DOF(self, IDEX, number, end_link_name='panda_link8'):
        """
        # shift_pose_target(DOF_Index, DOF_move_value, end_link_name)
        # [0, 1, 2, 3, 4, 5] --------- [x, y, z, rx, ry, rz]
        """
        self.group.shift_pose_target(IDEX, number, end_link_name)
        self.group.go(wait=True)

    def home_gripper(self):
        """
        Home and initialise the gripper
        :return: Bool success
        """
        client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)
        client.wait_for_server()
        client.send_goal(franka_gripper.msg.HomingGoal())
        return client.wait_for_result()

    def set_gripper(self, width, speed=0.1, wait=True):
        """
        Set gripper with.
        :param width: Width in metres
        :param speed: Move velocity (m/s)
        :param wait: Wait for completion if True
        :return: Bool success
        """
        client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()
        client.send_goal(franka_gripper.msg.MoveGoal(width, speed))
        if wait:
            return client.wait_for_result()
        else:
            return True

    def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
        """
        Wrapper around the franka_gripper/grasp action.
        http://docs.ros.org/kinetic/api/franka_gripper/html/action/Grasp.html
        :param width: Width (m) to grip
        :param e_inner: epsilon inner
        :param e_outer: epsilon outer
        :param speed: Move velocity (m/s)
        :param force: Force to apply (N)
        :return: Bool success
        """
        client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
                speed,
                force
            )
        )
        return client.wait_for_result()

    def stop(self):
        """
        Stop the current movement.
        """
        if not self.group:
            raise ValueError('No active Planning Group')
        if self.group:
            self.group.stop()

    def recover(self):
        """
        Call the error reset action server.
        """
        self.reset_publisher.publish(ErrorRecoveryActionGoal())
        rospy.sleep(3.0)

    def close_moveit(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def convert_pose(pose, from_frame, to_frame):
    """
    Convert a pose or transform between frames using tf.
        pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
        from_frame      -> A string that defines the original reference_frame of the robot
        to_frame        -> A string that defines the desired reference_frame of the robot to convert to
    """
    try:
        listener = tf.TransformListener()
        listener.waitForTransform(to_frame, from_frame, rospy.Time(), rospy.Duration(4.0))
        trans, rot = listener.lookupTransform(to_frame, from_frame, rospy.Time(0))
        # print('trans:', trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None

    spose = gmsg.PoseStamped()
    spose.pose = pose
    spose.header.stamp = rospy.Time().now
    spose.header.frame_id = from_frame
    spose.pose.position.x = trans[0]
    spose.pose.position.y = trans[1]
    spose.pose.position.z = trans[2]
    spose.pose.orientation.x = rot[0]
    spose.pose.orientation.y = rot[1]
    spose.pose.orientation.z = rot[2]
    spose.pose.orientation.w = rot[3]

    return spose.pose


def current_robot_pose(reference_frame, base_frame):
    """
    Get the current pose of the robot in the given reference frame
        reference_frame         -> A string that defines the reference_frame that the robots current pose will be defined in
    """
    # Create Pose
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Transforms robots current pose to the base reference frame
    return convert_pose(p, base_frame, reference_frame)


def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def list_to_quaternion(l):
    q = gmsg.Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]
    return q


if __name__ == '__main__':
    pose_target = Pose()
    pose_target.position.x = 0.30490281095
    pose_target.position.y = 0.000502838993857
    pose_target.position.z = 0.588340742141
    pose_target.orientation.x = 0.926107278239
    pose_target.orientation.y = -0.377146825889
    pose_target.orientation.z = -0.00217708474545
    pose_target.orientation.w = 0.00899117437009
    pre_pose = [0.492483570346, 0.0330769210778, 0.562724593622,
                     -0.918652267421, 0.394513626656, 0.0141818935886, -0.0153585103108]
    start_pre_pose = [0.600918633073, 0.0798243177083, 0.699749050227,
                           -0.926278989853, 0.376706391303, 0.00243192964424, -0.00967540322904]
    euler = tf.transformations.euler_from_quaternion([2**0.5/2, -2**0.5/2, 0, 0])
    print('euler: ', euler)
    q_test = list_to_quaternion(tf.transformations.quaternion_from_euler(np.pi, 0, 0))
    print("z angle = 0: ", q_test)
    rospy.init_node('move_group_grasp', anonymous=True)
    group_name = 'panda_arm'
    arm_group = Panda_moveit(group_name=group_name)
    # arm_group.print_debug_info()

    print("============ Printing current robot pose")
    robot_pose = current_robot_pose('panda_link0', 'panda_link8')
    print('robot_pose', robot_pose)
    camera_rot = tf.transformations.quaternion_matrix(quaternion_to_list(robot_pose.orientation))[0:3, 0:3]
    print('=====cam_angle: ', np.arcsin(camera_rot[0, 1]))
    robot_cam_pose = current_robot_pose('panda_link0', 'camera_depth_optical_frame')

    camera_rot = tf.transformations.quaternion_matrix(quaternion_to_list(robot_cam_pose.orientation))[0:3, 0:3]
    print('=====cam_angle: ', np.arcsin(camera_rot[0, 1]))

    arm_group.shift_pose_by_DOF(5, -np.pi/2, end_link_name='panda_link8')
    robot_cam_pose = current_robot_pose('panda_link0', 'camera_depth_optical_frame')

    camera_rot = tf.transformations.quaternion_matrix(quaternion_to_list(robot_cam_pose.orientation))[0:3, 0:3]
    print('=====cam_angle: ', np.arcsin(camera_rot[0, 1]))

    arm_group.goto_pose(robot_pose)
    print(robot_pose)

    # arm_group.goto_pose(start_pre_pose)
    rospy.sleep(4.0)
    robot_pose_cam = Pose()
    robot_pose_cam_pos = [0.550480624363, 0.0361430341771, 0.033]
    robot_pose_cam.position.x = robot_pose_cam_pos[0]
    robot_pose_cam.position.y = robot_pose_cam_pos[1]
    robot_pose_cam.position.z = robot_pose_cam_pos[2]
    robot_pose_cam.orientation = list_to_quaternion(tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 4))

    # robot_pose_cam = current_robot_pose('world', 'camera_link')
    rospy.loginfo('robot move done')
    arm_group.close_moveit()

    # print(arm_group.get_current_pose())  # from /panda_link0 to /panda_link8

