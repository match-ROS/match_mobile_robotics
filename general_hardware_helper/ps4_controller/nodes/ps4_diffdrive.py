#!/usr/bin/env python3
import rospy
from ps4_controller.PlayStationDiffDrive import PlayStationDiffDrive
from geometry_msgs.msg import Twist

if __name__=="__main__":
    rospy.init_node("ps4_diffdrive_controller")
    ps4=PlayStationDiffDrive(Twist)
    ps4.run()
    rospy.spin()