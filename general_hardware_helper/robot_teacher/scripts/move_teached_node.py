#!/usr/bin/env python
from robot_teacher.JointStateHandler import JointStateCommander
import rospy
if __name__=="__main__":
    try:
        rospy.init_node("joint_state_loader")
        loader=JointStateCommander()
        rospy.spin()
    except Exception as ex:
        rospy.logerr(ex)
