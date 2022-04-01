#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import math

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2

import tf

import roslib; roslib.load_manifest('laser_assembler')


class keyence_transform():
    def __init__(self):
        rospy.init_node('keyence_transform_node')
        self.transform = geometry_msgs.msg.TransformStamped()
        self.transform.header.frame_id = "map"
        self.transform.child_frame_id = "sensor_optical_frame"
        self.br = tf2_ros.TransformBroadcaster() 
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("map", self.transform.child_frame_id, rospy.Time(), rospy.Duration(4.0))
         
        rospy.sleep(1)  
        self.cloud_pub = rospy.Publisher("/cloud_out",PointCloud2, queue_size = 10)
        self.cloud_out = PointCloud2()
        rospy.loginfo("Transform publisher running")
        rospy.Subscriber("/profiles",PointCloud2, self.pointcloud_cb)
        rospy.spin()

        

    def pose_cb(self):
        # calculate transformation from world to EE
        self.transform.header.stamp = rospy.Time.now()
        self.cloud_out.header.frame_id = "map"
        self.cloud_out.header.stamp = self.transform.header.stamp
        
        
        try:
            (trans,rot) = self.listener.lookupTransform( 'map',self.transform.child_frame_id,  rospy.Time(0))
            #print(trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        self.transform.transform.translation.x = trans[0]
        self.transform.transform.translation.y = trans[1]
        self.transform.transform.translation.z = trans[2]
        self.transform.transform.rotation.x = rot[0]
        self.transform.transform.rotation.y = rot[1]
        self.transform.transform.rotation.z = rot[2]
        self.transform.transform.rotation.w = rot[3]

        #self.br.sendTransform(self.transform)
        

    def pointcloud_cb(self,cloud):
        self.pose_cb()
        self.cloud_out = do_transform_cloud(cloud, self.transform)
        self.cloud_pub.publish(self.cloud_out)
        

        
        
                
                





if __name__=="__main__":
    keyence_transform()