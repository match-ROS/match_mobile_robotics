#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class ur_command_smoother():
    
    
    def __init__(self):
        rospy.init_node("ur_command_smoother_node")
        self.config()
        
        
        self.actual_twist = Twist()
        twist_command = Twist()
        self.target_twist = Twist()
        self.last_msg_timestamp = rospy.get_time()
        
        rospy.Subscriber("/ur/unsmooth_twist",Twist,self.unsmooth_twist_cb)
        self.smooth_twitst_pub = rospy.Publisher("/twist_controller/command",Twist,queue_size=1)
        self.run()
        rospy.spin()
        
        
        
    
        
    def run(self):
        Rate = rospy.Rate(self.rate)    
        
        while not rospy.is_shutdown():
            
            twist_command = self.target_twist
            
            # check velocity limits
            if abs(twist_command.linear.x) > self.max_vel_lin_x:
                twist_command.linear.x = twist_command.linear.x / abs(twist_command.linear.x) * self.max_vel_lin_x
            if abs(twist_command.linear.y) > self.max_vel_lin_y:
                    twist_command.linear.y = twist_command.linear.y / abs(twist_command.linear.y) * self.max_vel_lin_y
            if abs(twist_command.linear.z) > self.max_vel_lin_z:
                    twist_command.linear.z = twist_command.linear.z / abs(twist_command.linear.z) * self.max_vel_lin_z
            if abs(twist_command.angular.x) > self.max_vel_lin_x:
                    twist_command.angular.x = twist_command.angular.x / abs(twist_command.angular.x) * self.max_vel_ang_x
            if abs(twist_command.angular.y) > self.max_vel_lin_y:
                    twist_command.angular.y = twist_command.angular.y / abs(twist_command.angular.y) * self.max_vel_ang_y
            if abs(twist_command.angular.z) > self.max_vel_lin_z:
                twist_command.angular.z = twist_command.angular.z / abs(twist_command.angular.z) * self.max_vel_ang_z
                
                
            # compute target acceleration
            acc_lin_x = twist_command.linear.x  - self.actual_twist.linear.x
            acc_lin_y = twist_command.linear.y  - self.actual_twist.linear.y
            acc_lin_z = twist_command.linear.z  - self.actual_twist.linear.z
            acc_ang_x = twist_command.angular.x - self.actual_twist.angular.x
            acc_ang_y = twist_command.angular.y - self.actual_twist.angular.y
            acc_ang_z = twist_command.angular.z - self.actual_twist.angular.z
            
            # limit target acceleration
            if abs(acc_lin_x) > self.max_acc_lin_x:
                self.actual_twist.linear.x += acc_lin_x / abs(acc_lin_x) * self.max_acc_lin_x
            else:   
                 self.actual_twist.linear.x += acc_lin_x
                 
            if abs(acc_lin_y) > self.max_acc_lin_y:
                    self.actual_twist.linear.y += acc_lin_y / abs(acc_lin_y) * self.max_acc_lin_y
            else:   
                 self.actual_twist.linear.y += acc_lin_y
                 
            if abs(acc_lin_z) > self.max_acc_lin_z:
                    self.actual_twist.linear.z += acc_lin_z / abs(acc_lin_z) * self.max_acc_lin_z
            else:   
                 self.actual_twist.linear.z += acc_lin_z
                 
            if abs(acc_ang_x) > self.max_acc_ang_x:
                    self.actual_twist.angular.x += acc_ang_x / abs(acc_ang_x) * self.max_acc_ang_x
            else:   
                 self.actual_twist.angular.x += acc_ang_x
                
            if abs(acc_ang_y) > self.max_acc_ang_y:
                        self.actual_twist.angular.y += acc_ang_y / abs(acc_ang_y) * self.max_acc_ang_y
            else:   
                 self.actual_twist.angular.y += acc_ang_y
                 
            if abs(acc_ang_z) > self.max_acc_ang_z:
                        self.actual_twist.angular.z += acc_ang_z / abs(acc_ang_z) * self.max_acc_ang_z
            else:   
                 self.actual_twist.angular.z += acc_ang_z
                 

            self.smooth_twitst_pub.publish(self.actual_twist)
            
            
            # check for timestamp
            if abs(self.last_msg_timestamp-rospy.get_time()) >  self.timeout:
                self.target_twist = Twist()   
                 
            rospy.loginfo_throttle(0.2,self.actual_twist)
            Rate.sleep()
                
        
    def config(self):
        self.rate = rospy.get_param('~publish_rate',100)
        self.timeout = rospy.get_param('~timeout',0.1)
        
        self.max_vel_lin_x = rospy.get_param('~max_vel_lin_x',0.0)
        self.max_vel_lin_y = rospy.get_param("~max_vel_lin_y",0.0)
        self.max_vel_lin_z = rospy.get_param("~max_vel_lin_z",0.0)
        self.max_vel_ang_x = rospy.get_param("~max_vel_ang_x",0.0)
        self.max_vel_ang_y = rospy.get_param("~max_vel_ang_y",0.0)
        self.max_vel_ang_z = rospy.get_param("~max_vel_ang_z",0.0)
        
        self.max_acc_lin_x = rospy.get_param('~max_acc_lin_x',0.0)
        self.max_acc_lin_y = rospy.get_param('~max_acc_lin_y',0.0)
        self.max_acc_lin_z = rospy.get_param('~max_acc_lin_z',0.0)
        self.max_acc_ang_x = rospy.get_param('~max_acc_ang_x',0.0)
        self.max_acc_ang_y = rospy.get_param('~max_acc_ang_y',0.0)
        self.max_acc_ang_z = rospy.get_param('~max_acc_ang_z',0.0)
        
        
    def unsmooth_twist_cb(self,Twist):
        self.last_msg_timestamp = rospy.get_time()
        self.target_twist = Twist
        print("callback")
        
if __name__ == '__main__':
    exe = ur_command_smoother()
    exe.run()