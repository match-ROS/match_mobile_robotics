#!/usr/bin/env python3

from threading import local
import actionlib

import rospy
import mbf_msgs.msg as mbf_msgs 
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay the simple goal topic published by RViz, the make_plan service and dynamic reconfiguration
calls (note that some parameters have changed names; see http://wiki.ros.org/move_base_flex for details)
"""

# keep configured base local and global planners to send to MBF
global global_planner_name 
global_planner_name = None
global local_planner_name
local_planner_name = None


def simple_goal_cb(msg):
    rospy.loginfo("Relaying simple goal to move_base_flex/exe_path action server")
    mbf_mb_exe_path_ac.send_goal(mbf_msgs.ExePathGoal(path=msg, controller = "DWAPlannerROS"))
    
    # get status of action server
    while not rospy.is_shutdown():
        status_code = mbf_mb_exe_path_ac.get_state()
        

        rospy.sleep(1.0)
    

if __name__ == '__main__':
    rospy.init_node("move_base")

    # move_base_flex get_path and move_base action clients
    mbf_mb_exe_path_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
    mbf_mb_exe_path_ac.wait_for_server(rospy.Duration(20))

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_path_simple/path', Path, simple_goal_cb)

    rospy.spin()
