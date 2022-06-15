#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

global f_msg
f_msg = None
global b_msg
b_msg = None

global old_msg
old_msg = None

def check_msgs():
    global f_msg
    global b_msg 

    if(f_msg == None or b_msg == None):
        return 

    if(f_msg.header.stamp.nsecs == b_msg.header.stamp.nsecs):
        rospy.loginfo("f_scan and b_scan headers are equal! " + str(f_msg.header.stamp.nsecs) + " | " + str(b_msg.header.stamp.nsecs))

def f_scan_cb(msg):
    global f_msg

    f_msg = msg
    check_msgs()

def b_scan_cb(msg):
    global b_msg

    b_msg = msg
    check_msgs()

def scan_cb(msg):
    global old_msg

    if(old_msg == None):
        return

    if(old_msg.header.stamp.nsecs == msg.header.stamp.nsecs):
        rospy.loginfo("scan headers are equal! " + str(old_msg.header.stamp.nsecs) + " | " + str(msg.header.stamp.nsecs))

    old_msg = msg

def main():
    rospy.init_node('scan_checker', anonymous=True)
    rospy.Subscriber('/f_scan', LaserScan, f_scan_cb)
    rospy.Subscriber('/b_scan', LaserScan, b_scan_cb)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.loginfo('Scan checker is running!')
    rospy.spin()


if __name__ == '__main__':
    main()
