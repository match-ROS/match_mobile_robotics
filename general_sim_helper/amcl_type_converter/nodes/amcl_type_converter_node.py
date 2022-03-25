#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose

class AmclTypeConverter():
    def __init__(self):
        self.amcl_input_topic = rospy.get_param("~amcl_input_topic","amcl_pose")
        self.amcl_output_topic = rospy.get_param("~amcl_output_topic", "amcl_pose_converted")

        rospy.loginfo("amcl_input_topic:" + self.amcl_input_topic)
        rospy.loginfo("amcl_output_topic:" + self.amcl_output_topic)

        rospy.Subscriber(self.amcl_input_topic, PoseWithCovarianceStamped, self.amcl_pose_cb)
        self.converted_amcl_publisher = rospy.Publisher(self.amcl_output_topic, Pose, queue_size=10)

    
    def amcl_pose_cb(self,data):
        rospy.logerr("msg received")
        self.converted_amcl_publisher.publish(data.pose.pose)


if __name__=="__main__":
    rospy.init_node("amcl_type_converter_node")
    
    amcl_type_converter = AmclTypeConverter()
    
    rospy.spin()