#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

global pub
def poseCallback(msg):
    global pub
    pose_new=msg
    pose_new.header.frame_id=msg.header.frame_id.split("/")[-1]
    pub.publish(pose_new)

if __name__=="__main__":
    rospy.init_node("tf_prefix_remover")
    global pub
    sub=rospy.Subscriber("topic_in",PoseStamped,poseCallback)
    pub=rospy.Publisher("topic_out",PoseStamped,queue_size=10)
    rospy.spin()