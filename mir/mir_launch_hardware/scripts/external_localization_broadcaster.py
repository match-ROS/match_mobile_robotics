#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class ExternalLocalizationBroadcaster:
    def __init__(self):
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mur620b/mir/')
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.localization_topic = rospy.get_param('~localization_topic', '/qualisys/mur620b/base_link/pose')
        self.sub = rospy.Subscriber(self.localization_topic, PoseStamped, self.callback)
        self.timestamp = rospy.Time.now()
        rospy.spin()

    def callback(self, msg = PoseStamped()):
        now = rospy.Time.now()
        if (now - self.timestamp) <= rospy.Duration(0.0):
            now = self.timestamp + rospy.Duration(0.001)
            # alter the timestamp to avoid tf2 buffer error
        self.timestamp = now
        pose = msg.pose
        self.pose_broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z), 
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            now,
            self.tf_prefix + '/base_footprint',
            self.tf_prefix + '/odom'   
        )
                                            

            
if __name__ == '__main__':
    rospy.init_node('external_localization_broadcaster')
    broadcaster = ExternalLocalizationBroadcaster()
