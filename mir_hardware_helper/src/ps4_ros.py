#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class ps4_ros():

    def __init__(self):
        rospy.init_node('controller_ros_node', anonymous=False)
        sub_joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_vel_pub_miranda = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        self.cmd_vel_pub_mur = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()

        # Default values
        self.speed_vx = 0.1
        self.speed_w = 0.2
        self.state = 1
        self.maxStates = 2 


    def run(self):
        print("run")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 1:
                self.cmd_vel_pub_miranda.publish(self.twist_msg)
            elif self.state == 2:
                self.cmd_vel_pub_mur.publish(self.twist_msg)
            rate.sleep()

    def joy_callback(self,data):
        self.x = abs(data.axes[5] - 1) - abs(data.axes[2] - 1) #data.axes[1] + data.axes[4]
        self.w = data.axes[0] + data.axes[3]
        self.changeMode = data.buttons[10]
        
        
        if (data.buttons[3] == 1 and self.buttons_old[3] == 0):
            self.speed_vx *= 1.1

        if (data.buttons[0] == 1 and self.buttons_old[0] == 0):
            self.speed_vx *= 0.9

        if (data.buttons[1] == 1 and self.buttons_old[1] == 0):
            self.speed_w *= 1.1

        if (data.buttons[2] == 1 and self.buttons_old[2] == 0):
            self.speed_w *= 0.9

        if (self.changeMode == 1 and self.changeMode_old == 0):
            self.state += 1
            if self.state > self.maxStates:
                self.state = 1

        self.twist_msg.linear.x = self.x * self.speed_vx
        self.twist_msg.angular.z = self.w * self.speed_w





        print(self.x, self.w,self.speed_vx, self.speed_w)
        # save old state
        self.changeMode_old = data.buttons[10]
        self.buttons_old = data.buttons



if __name__ == '__main__':
    try:
      #rospy.loginfo("Starting Controller Node")
      PS4_ros = ps4_ros()
      PS4_ros.run()
      rospy.spin()

    except rospy.ROSInterruptException:
        print("das hat nicht geklaptt")
      #rospy.loginfo( "ps4_ros node terminated.")

