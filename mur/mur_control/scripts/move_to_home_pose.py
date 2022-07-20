#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
import moveit_commander
from math import pi, tau

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_to_home_node", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "UR_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

tau = 2.0 * pi
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0.0 * tau / 4
joint_goal[1] = -tau / 4
joint_goal[2] = tau / 4
joint_goal[3] = -tau / 4
joint_goal[4] = -tau / 4
joint_goal[5] = 0  

move_group.go(joint_goal, wait=True)

move_group.stop()