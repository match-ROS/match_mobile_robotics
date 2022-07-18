#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
 
        rospy.init_node('static_tf2_broadcaster')

        target_frame=rospy.get_param('~target_frame')
        source_frame=rospy.get_param('~source_frame')
        x=rospy.get_param('~x')
        y=rospy.get_param('~y')
        z=rospy.get_param('~z')
        yaw=rospy.get_param('~yaw')

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = source_frame
        static_transformStamped.child_frame_id = target_frame

        static_transformStamped.transform.translation.x = float(x)
        static_transformStamped.transform.translation.y = float(y)
        static_transformStamped.transform.translation.z = float(z)

        quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()