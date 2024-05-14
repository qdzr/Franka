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

pc = PandaCommander(group_name='manipulator')
