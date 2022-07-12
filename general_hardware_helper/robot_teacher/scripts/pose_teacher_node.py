#!/usr/bin/env python3
from robot_teacher.JointTeacher import TeachedPoseHandler
import rospy
if __name__=="__main__":
    try:
        rospy.init_node("teached_pose_handler")
        loader=TeachedPoseHandler()
        rospy.spin()
    except Exception as ex:
        rospy.logerr(ex)
