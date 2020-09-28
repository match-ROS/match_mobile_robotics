#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool
import actionlib
from control_msgs.msg import GripperCommandAction,GripperCommandGoal

client=actionlib.ActionClient("franka_gripper/gripper_action",GripperCommandAction)

def gripCallback(req):
    global client
    goal=GripperCommandGoal()
  
    if not req.data:
        goal.command.position=0.035
        goal.command.max_effort=20
        msg="Released"
    else :
        goal.command.position=0.0
        goal.command.max_effort=20
        msg="Gripped"
    
    client.send_goal(goal)
    return {"success":True,"message":msg}
        


if __name__=="__main__":
    rospy.init_node("grip_service_interface")
    rospy.Service("grip",SetBool,gripCallback)
    global client    
    client.wait_for_server()
    rospy.spin()
    