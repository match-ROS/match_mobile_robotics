#!/usr/bin/env python3
import rospy
from net_box_hardware_helper import NetBoxRosWrapper

if __name__== "__main__":
    rospy.init_node("force_torque_sensor")
    sensor=NetBoxRosWrapper()
    rospy.spin()