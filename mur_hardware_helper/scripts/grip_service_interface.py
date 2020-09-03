#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool,SetBoolResponse
from std_msgs.msg import String



def gripCallback(req):
    global client
    goal=String()
  
    if not req.data:
        goal.data="set_tool_digital_out(0,False)"
        msg="Released"
    else:
        goal.data="set_tool_digital_out(0,True)"
        msg="Gripped"
    client.publish(goal)    
    return {"success":True,"message":msg}
        


if __name__=="__main__":    
    global client
    rospy.init_node("grip_service_interface")
    client=rospy.Publisher("ur_hardware_interface/script_command",String,queue_size=10)
    rospy.Service("grip",SetBool,gripCallback)
    rospy.spin()
    