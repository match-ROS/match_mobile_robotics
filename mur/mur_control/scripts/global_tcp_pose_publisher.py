#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf 
from tf.transformations import quaternion_from_euler

class GlobalTCPPosePublisher():

    def __init__(self):
        rospy.init_node('global_tcp_pose_publisher', anonymous=True)
        self.rate = rospy.get_param('rate', 100.0)
        self.UR_base_link_name = rospy.get_param('~UR_base_link_name', 'mur620a/UR10_l/base_link_inertia')
        self.base_frame = rospy.get_param('~base_frame', 'map')
        self.local_TCP_pose_topic = rospy.get_param('~local_TCP_pose_topic', '/mur620a/UR10_l/ur_calibrated_pose')

        self.local_TCP_pose = Pose()

        rospy.Subscriber(self.local_TCP_pose_topic, PoseStamped, self.local_TCP_pose_callback)

    def tcp_pose_publisher(self):
        
        
        tf_listener = tf.TransformListener()    
        
        pub = rospy.Publisher('global_tcp_pose', PoseStamped, queue_size=10)
        
        rate = rospy.Rate(self.rate)  # 10 Hz
        
        trans = None
        while trans is None:
            try:
                # get transform between map and UR base link
                trans, rot = tf_listener.lookupTransform(self.base_frame, self.UR_base_link_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            try:
                # get transform between map and UR base link
                trans, rot = tf_listener.lookupTransform(self.base_frame, self.UR_base_link_name, rospy.Time(0))

                # transform local TCP pose to map frame and add to the UR base link pose
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.base_frame

                R = tf.transformations.quaternion_matrix([rot[0], rot[1], rot[2], rot[3]])
                pose.pose.position.x = trans[0] + R[0,0]*self.local_TCP_pose.position.x + R[0,1]*self.local_TCP_pose.position.y + R[0,2]*self.local_TCP_pose.position.z
                pose.pose.position.y = trans[1] + R[1,0]*self.local_TCP_pose.position.x + R[1,1]*self.local_TCP_pose.position.y + R[1,2]*self.local_TCP_pose.position.z
                pose.pose.position.z = trans[2] + R[2,0]*self.local_TCP_pose.position.x + R[2,1]*self.local_TCP_pose.position.y + R[2,2]*self.local_TCP_pose.position.z

                R_local = tf.transformations.quaternion_matrix([self.local_TCP_pose.orientation.x, self.local_TCP_pose.orientation.y, self.local_TCP_pose.orientation.z, self.local_TCP_pose.orientation.w])
                R_combined = tf.transformations.concatenate_matrices(R, R_local)
                q = tf.transformations.quaternion_from_matrix(R_combined)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                pub.publish(pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()

    def local_TCP_pose_callback(self, data):
        self.local_TCP_pose = data.pose

                

if __name__ == '__main__':
    try:
        GlobalTCPPosePublisher().tcp_pose_publisher()
    except rospy.ROSInterruptException:
        pass
