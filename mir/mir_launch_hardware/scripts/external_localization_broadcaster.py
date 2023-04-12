#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose

class ExternalLocalizationBroadcaster:
    def __init__(self):
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mur620b/mir/')
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.localization_topic = rospy.get_param('~localization_topic', '/qualisys/mur620b/base_link/pose')
        self.mocap_offset = rospy.get_param('~mocap_offset', [52.19662100595798, 43.254582705671346, -0.8770744555474559, -0.00509510875183926, -0.006591832597023015, -1.2673565795989212])
        self.sub = rospy.Subscriber(self.localization_topic, PoseStamped, self.callback)
        self.timestamp = rospy.Time.now()
        rospy.spin()

    def callback(self, msg = PoseStamped()):
        now = rospy.Time.now()
        if (now - self.timestamp) <= rospy.Duration(0.0):
            now = self.timestamp + rospy.Duration(0.001)
            # alter the timestamp to avoid tf2 buffer error
        self.timestamp = now
        pose = self.transform_pose_to_mir_map(msg.pose)
        self.pose_broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z), 
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            now,
            self.tf_prefix + '/base_footprint',
            self.tf_prefix + '/odom'   
        )
              
              
    def transform_pose_to_mir_map(self, pose = Pose()):
        # initialize transformation matrix
        T = transformations.identity_matrix()
        # construct transformation matrix from mocap_offset
        q = transformations.quaternion_from_euler(self.mocap_offset[3], self.mocap_offset[4], self.mocap_offset[5])
        R = transformations.quaternion_matrix(q)
        T[0:3,0:3] = R[0:3,0:3]
        T[0,3] = self.mocap_offset[0]
        T[1,3] = self.mocap_offset[1]
        T[2,3] = self.mocap_offset[2]
        
        # apply transformation to position
        pose_out = Pose()
        pose_out.position.x = T[0,0]*pose.position.x + T[0,1]*pose.position.y + T[0,2]*pose.position.z + T[0,3]
        pose_out.position.y = T[1,0]*pose.position.x + T[1,1]*pose.position.y + T[1,2]*pose.position.z + T[1,3]
        pose_out.position.z = T[2,3] # z is not transformed
        
        # apply transformation to orientation
        orientation = transformations.quaternion_multiply(transformations.quaternion_from_matrix(T), [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        
        pose_out.orientation.x = orientation[0]
        pose_out.orientation.y = orientation[1]
        pose_out.orientation.z = orientation[2]
        pose_out.orientation.w = orientation[3]
         
        return pose_out

            
if __name__ == '__main__':
    rospy.init_node('external_localization_broadcaster')
    broadcaster = ExternalLocalizationBroadcaster()
