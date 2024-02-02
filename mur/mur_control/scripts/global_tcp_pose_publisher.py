#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf 
from tf.transformations import quaternion_from_euler

class GlobalTCPPosePublisher():

    def __init__(self):
        self.rate = rospy.get_param('rate', 100.0)
        self.UR_base_link_name = rospy.get_param('UR_base_link_name', 'mur620a/UR10_l/base_link')
        self.base_frame = rospy.get_param('base_frame', 'map')
        self.local_TCP_pose_topic = rospy.get_param('local_TCP_pose_topic', '/mur620a/UR10_l/tcp_pose')

        rospy.Subscriber(self.local_TCP_pose_topic, Pose, self.local_TCP_pose_callback)

    def tcp_pose_publisher(self):
        rospy.init_node('global_tcp_pose_publisher', anonymous=True)
        
        tf_listener = tf.TransformListener()    
        
        pub = rospy.Publisher('global_tcp_pose', PoseStamped, queue_size=10)
        
        rate = rospy.Rate(self.rate)  # 10 Hz
        
        tf_listener.waitForTransform(self.base_frame, self.UR_base_link_name, rospy.Time(), rospy.Duration(4.0))

        while not rospy.is_shutdown():
            try:
                trans, rot = tf_listener.lookupTransform(self.base_frame, self.UR_base_link_name, rospy.Time(0))
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.base_frame
                pose.pose.position.x =  trans[0]
                pose.pose.position.y =  trans[1]
                pose.pose.position.z =  trans[2]
                q = quaternion_from_euler(rot[0], rot[1], rot[2])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                pub.publish(pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    

                

if __name__ == '__main__':
    try:
        GlobalTCPPosePublisher().tcp_pose_publisher()
    except rospy.ROSInterruptException:
        pass
