#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time



def main():
    rospy.init_node('simple_curve_publisher', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/virtual_leader/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)  # 10 Hz

    twist = Twist()

    # Publish linear velocity for 5 seconds
    twist.linear.x = 0.1
    twist.angular.z = 0.0
    start_time = time.time()
    while time.time() - start_time < 15:
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Publish linear and angular velocity for another 5 seconds
    twist.linear.x = 0.1
    twist.angular.z = 0.05
    start_time = time.time()
    while time.time() - start_time < 15:
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Publish linear velocity for 5 seconds
    twist.linear.x = 0.1
    twist.angular.z = 0.0
    start_time = time.time()
    while time.time() - start_time < 15:
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass