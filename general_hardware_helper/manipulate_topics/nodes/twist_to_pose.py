#!/usr/bin/env python
import rospy
import tf.transformations as tf
import numpy as np
from geometry_msgs.msg import PoseStamped,Quaternion
from geometry_msgs.msg import TwistStamped


if __name__=="__main__":
    rospy.init_node("twist_dummie")
    pub=rospy.Publisher("topic_out",PoseStamped,queue_size=10)
    
    while not rospy.is_shutdown():
        vel=rospy.wait_for_message("/virtual_master/master_velocity",TwistStamped)
        pos=rospy.wait_for_message("/virtual_master/master_pose",PoseStamped)      
        y=vel.twist.linear.y
        x=vel.twist.linear.x

        pose=PoseStamped()
        pose.header.frame_id="map"
        pose.pose.position=pos.pose.position
        pose.pose.orientation=Quaternion(*tf.quaternion_from_euler(0,0,np.arctan2(y,x)))
        pub.publish(pose)
