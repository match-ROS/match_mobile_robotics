#!/usr/bin/env python
from robot_teacher.JointStateHandler import JointStateCommander
import rospy
if __name__=="__main__":
    rospy.init_node("joint_state_loader")
    loader=JointStateCommander("testfile.yaml")
    rospy.spin()
