#! /usr/bin/env python

import rospy
import actionlib
from ur_dashboard_msgs.msg import SetModeAction,SetModeGoal

def feedbackCallback(feedback):
    rospy.loginfo("Feedback:")
    rospy.loginfo(feedback)

def activeCallback():
    rospy.loginfo("Enabling ur!")

def doneCallback(state,result):
    if result.success: 
        rospy.loginfo("Done:")
        rospy.loginfo("State:")
        rospy.loginfo(state)
        rospy.loginfo("Result:")
        rospy.loginfo(result)
        rospy.signal_shutdown("Enabeling done")
    else:
        rospy.logwarn("Done:")
        rospy.logwarn("State:")
        rospy.logwarn(state)
        rospy.logwarn("Result:")
        rospy.logwarn(result)
        rospy.signal_shutdown("Enabeling done")


if __name__=="__main__":
    rospy.init_node("ur_enabler")
    client=actionlib.SimpleActionClient("ur_hardware_interface/set_mode",SetModeAction)
    client.wait_for_server()
    goal=SetModeGoal()
    goal.target_robot_mode=7
    goal.play_program=True
    client.send_goal(goal,feedback_cb=feedbackCallback,done_cb=doneCallback,active_cb=activeCallback)
    rospy.spin()