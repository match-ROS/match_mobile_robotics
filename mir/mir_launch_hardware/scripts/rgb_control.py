#!/usr/bin/env python3

# This node offers a number of services to control the RGB LEDs of the MIR robot.

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from mir_msgs.msg import LightCmd


class RGB_Control():
    
    def __init__(self):
        rospy.loginfo('RGB_Control: init')
        # create trigger service for rainbow effect
        rospy.Service('rainbow_start', Trigger, self.rainbow_effect)
        rospy.Service('rainbow_stop', Trigger, self.rainbow_effect)
        self.light_cmd_pub = rospy.Publisher('/new_light_cmd', LightCmd, queue_size=1)
        self.light_cmd = LightCmd()
        rospy.spin()
        
    def rainbow_effect(self, req):
                
        if req._connection_header['service'] == '/rainbow_start':
            rospy.loginfo('rainbow_start')
            self.light_cmd.color1 = "ffffff"
            self.light_cmd.color2 = "ffffff"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "rainbow"
            self.light_cmd_pub.publish(self.light_cmd)
            
        elif req._connection_header['service'] == '/rainbow_stop':
            rospy.loginfo('rainbow_stop')
            self.light_cmd.color1 = "ffff00"
            self.light_cmd.color2 = "ff0000"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "Wave"
            self.light_cmd_pub.publish(self.light_cmd)
        else:
            rospy.logwarn('Unknown service: ' + req._connection_header['service'])
            
        
            
        return TriggerResponse(success=True, message='OK')
        
    
    
if __name__ == '__main__':
    rospy.init_node('rgb_control')
    RGB_Control()
    
    
    
    

