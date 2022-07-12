#!/usr/bin/env python3
import rospy
from ps4_controller.PlayStationDiffDriveServices import PlayStationDiffDriveServices
from geometry_msgs.msg import TwistStamped

if __name__=="__main__":
    rospy.init_node("ps4_diffdrive_services")
    ps4=PlayStationDiffDriveServices(TwistStamped)
    ps4.run()
    rospy.spin()