#!/usr/bin/env python

"""
Script to move the robot to a named pose using MoveIt.
The target pose and move group can be configured via ROS parameters.
"""

import sys
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException


def main():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)

    # Get parameters with defaults
    target_pose = rospy.get_param('~target_pose', 'start')
    move_group_name = rospy.get_param('~move_group', 'manipulator')

    try:
        # Create a MoveGroupCommander for the specified group
        move_group = moveit_commander.MoveGroupCommander(move_group_name)
        
        # Set planning parameters
        move_group.set_planning_time(10.0)
        move_group.set_num_planning_attempts(5)
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_max_acceleration_scaling_factor(0.3)

        rospy.loginfo("Moving robot to '%s' position using group '%s'..." % (target_pose, move_group_name))
        
        # Set the target to the named position
        move_group.set_named_target(target_pose)
        
        # Plan and execute the motion
        success = move_group.go(wait=True)
        
        # Ensure there is no residual movement
        move_group.stop()
        move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("Successfully moved to '%s' position!" % target_pose)
        else:
            rospy.logerr("Failed to move to '%s' position" % target_pose)
            return 1

    except MoveItCommanderException as e:
        rospy.logerr("MoveIt commander exception: %s" % str(e))
        return 1
    except rospy.ROSInterruptException:
        rospy.loginfo("Motion interrupted")
        return 1
    finally:
        moveit_commander.roscpp_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())

