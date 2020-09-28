#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool,SetBoolResponse
from std_msgs.msg import String
from ur_msgs.msg import IOStates 


def gripCallback(req):
    global client
    goal=String()
    io=rospy.wait_for_message("ur_hardware_interface/io_states",IOStates)

    states=io.digital_out_states
    
    if not req.data:
        if states[16].state:    #Check if gripper already released sicne double forcing leads to system crash
            goal.data="set_tool_digital_out(0,False)"           
            client.publish(goal)    
        msg="Released"
    else:
        if not states[16].state:  #Check if gripper already closed sicne double forcing leads to system crash
            goal.data="set_tool_digital_out(0,True)"
            client.publish(goal)    
        msg="Gripped"
  
    return {"success":True,"message":msg}
        


if __name__=="__main__":    
    global client
    rospy.init_node("grip_service_interface")
    client=rospy.Publisher("ur_hardware_interface/script_command",String,queue_size=10)
    rospy.Service("grip",SetBool,gripCallback)
    rospy.spin()
    