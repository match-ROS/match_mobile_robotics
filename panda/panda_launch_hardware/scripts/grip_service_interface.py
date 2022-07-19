#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool
import actionlib
from franka_gripper.msg import GraspAction,GraspGoal

client=actionlib.ActionClient("franka_gripper/grasp",GraspAction)

def gripCallback(req):
    global client
    goal=GraspGoal()
  
    if not req.data:
        goal.width=0.05        
        goal.force=10
        goal.speed=0.1
        goal.epsilon.outer=0.05
        goal.epsilon.inner=0.05      
        msg="Released"
    else :
        goal.width=0.00
        goal.force=10
        goal.speed=0.1
        goal.epsilon.outer=0.05
        goal.epsilon.inner=0.05        
        msg="Gripped"
    
    client.send_goal(goal)
    return {"success":True,"message":msg}
        


if __name__=="__main__":
    rospy.init_node("grip_service_interface")
    rospy.Service("grip",SetBool,gripCallback)
    global client    
    client.wait_for_server()
    rospy.spin()
    