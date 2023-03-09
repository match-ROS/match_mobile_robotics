#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class Mir_pose_simple():

    def __init__(self):
        # Get params
        self.localization_type = rospy.get_param("~localization_type", "amcl")
        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", "amcl_pose")
        self.groud_truth_topic = rospy.get_param("~groud_truth_topic", "groud_truth")
        
        # Subscribe to the localization topic based on the localization_type parameter
        if self.localization_type == "amcl":
            rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.amcl_pose_callback)
        elif self.localization_type == "ground_truth":
            rospy.Subscriber(self.groud_truth_topic, Odometry, self.groud_truth_callback)
        elif self.localization_type == "odom":
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        # Initialize the publishers
        self.pose_pub = rospy.Publisher('mir_pose_simple', Pose, queue_size=1)
        self.pose_stamped_pub = rospy.Publisher('mir_pose_stamped_simple', PoseStamped, queue_size=1)

    def odom_callback(self, msg):
        self.pose_pub.publish(msg.pose.pose)
        self.pose_stamped_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))
        
    def amcl_pose_callback(self, msg):
        self.pose_pub.publish(msg.pose.pose)
        self.pose_stamped_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))

    def groud_truth_callback(self, msg):
        self.pose_pub.publish(msg.pose.pose)
        self.pose_stamped_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))



if __name__ == '__main__':
    try:
        rospy.init_node('mir_pose_simple')
        rospy.loginfo('mir_pose_simple node started')
        Mir_pose_simple()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass