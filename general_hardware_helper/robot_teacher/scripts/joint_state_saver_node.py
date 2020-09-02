#!/usr/bin/env python
import rospy
from robot_teacher.JointStateHandler import JointStateSaver

if __name__=="__main__":
    rospy.init_node("joint_state_saver")
    try:
        joint_state_saver=JointStateSaver()
        rospy.spin()
   except Exception as ex:
        rospy.logerr(ex)
