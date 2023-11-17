#!/usr/bin/env python3
import rospy
import tf2_ros, tf2_geometry_msgs
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped

class MocapTransformPublisher:
    
    def callback(self, data):
        self.transformation.header.stamp = rospy.Time.now()
        point_transformed = tf2_geometry_msgs.do_transform_pose(data, self.transformation)
        self.publisher.publish(point_transformed)
        rospy.loginfo(point_transformed)
    
    def run(self):
        rospy.init_node('mocap_transform_publisher')
        rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        rospy.loginfo("Starting mocap transform publisher")
        rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.qualisys_topic = rospy.get_param("~localization_topic", "/qualisys/mur620c/pose")
        rospy.loginfo(self.qualisys_topic)
        self.transformed_topic = self.qualisys_topic + "_transformed"
        self.publisher = rospy.Publisher(self.transformed_topic, PoseStamped, queue_size=10)
        self.trans = rospy.get_param("~transformation_mocap", "42.3678045642562 44.91618935433426 0 0 0 1.2801445960189657 1 ")
        t = self.trans.split(" ")
        self.transformation = TransformStamped()
        self.transformation.child_frame_id = "mocap"
        self.transformation.header.frame_id = "map"
        self.transformation.transform.translation.x = float(t[0])
        self.transformation.transform.translation.y = float(t[1])
        self.transformation.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, float(t[5]))
        self.transformation.transform.rotation.x = q[0]
        self.transformation.transform.rotation.y = q[1]
        self.transformation.transform.rotation.z = q[2]
        self.transformation.transform.rotation.w = q[3]
        self.subscriber = rospy.Subscriber(self.qualisys_topic, PoseStamped, self.callback)
        rospy.spin()
        

if __name__ == '__main__':
    mtp = MocapTransformPublisher()
    mtp.run()