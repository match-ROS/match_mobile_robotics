#!/usr/bin/env python
import rospy
from watchdogs import ForceWatchDog

if __name__=="__main__":
    rospy.init_node("force_watchdog")
    dog=ForceWatchDog()
    rospy.spin()