#!/usr/bin/env python
from robot_teacher.PoseTeacher import PoseFileHandler
import rospy
if __name__=="__main__":
    rospy.init_node("pose_teacher")
    handler=PoseFileHandler()
    rospy.spin()
