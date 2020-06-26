#! /usr/bin/env python

import rospy
import ur_dashboard_msgs.msg as ur
import actionlib

if __name__=="__main__":
    rospy.init_node("enable_ur")
    client=actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', ur.SetModeAction)
    rospy.loginfo("Waiting for set_mode action server!")
    client.wait_for_server()
    rospy.loginfo("Found set_mode action server!")
    goal_msg=ur.SetModeActionGoal()
    goal_msg.goal.target_robot_mode=7
    goal_msg.goal.play_program=True
    client.send_goal(goal=goal_msg.goal)
    
    
    rospy.loginfo("Waiting for robot to come up!")
    client.wait_for_result()
    result=ur.SetModeActionResult()
    result=client.get_result()
    rospy.loginfo("Succeeded: %d \t Message: %s",
                    result.success,
                    result.message)
    






