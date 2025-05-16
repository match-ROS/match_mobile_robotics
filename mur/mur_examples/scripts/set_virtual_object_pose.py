#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def set_target_pose():
    # Initialize the ROS node
    rospy.init_node('set_virtual_object_pose')
    pose = rospy.get_param('~pose', [0.0, 0.0, 1.5, 0, 0, 0.3826834, 0.9238795])
    pose_topic = rospy.get_param('~pose_topic', '/virtual_object/set_pose')
    # Create a publisher for the target pose
    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)

    rospy.loginfo('Setting the target pose to: {}'.format(pose))

    rospy.sleep(1)

    # Create a PoseStamped message
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'  # Set the frame ID
    target_pose.pose.position.x = pose[0]  # Set the position (x-coordinate)
    target_pose.pose.position.y = pose[1]  # Set the position (y-coordinate)
    target_pose.pose.position.z = pose[2]  # Set the position (z-coordinate)
    target_pose.pose.orientation.x = pose[3]  # Set the orientation (x-coordinate)
    target_pose.pose.orientation.y = pose[4]  # Set the orientation (y-coordinate)
    target_pose.pose.orientation.z = pose[5]  # Set the orientation (z-coordinate)
    target_pose.pose.orientation.w = pose[6]  # Set the orientation (w-coordinate)

    # Publish the target pose
    pub.publish(target_pose)

    # Spin the ROS node to send the message
    rospy.sleep(1)

if __name__ == '__main__':
    set_target_pose()