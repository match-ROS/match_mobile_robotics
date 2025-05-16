#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from tf import TransformListener
from tf.transformations import quaternion_matrix, quaternion_from_matrix, concatenate_matrices

import numpy as np

# Listens to Pose messages and calculates the kinematic chain to publish the total Pose
# in: world_T_base, base_T_ee; out: world_T_ee
def pose_to_matrix(pose:Pose):
    """Convert Pose to transformation matrix"""
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    matrix = quaternion_matrix(rot)
    matrix[0:3, 3] = trans
    return matrix

def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert transformation matrix to Pose"""
    pose = Pose()
    translation = matrix[0:3, 3]
    rotation = quaternion_from_matrix(matrix)
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
    return pose
class TotalPose:
    def __init__(self):
        # msg_class_base,_,_=rostopic.get_topic_class("world_T_base", blocking=True)
        self.world_T_base = np.eye(4)
        self.base_T_ee = np.eye(4)
        self.world_T_ee = PoseStamped()
        self.world_T_ee.header.frame_id = rospy.get_param("~world_frame", "map")

        # In case there is a static transform between the 2 topics:
        self.static_tf_included = rospy.get_param("~static_tf_included", False)
        static_tf_parent = rospy.get_param("~static_tf_parent", "mur620/base_footprint")
        static_tf_child = rospy.get_param("~static_tf_child", "mur620/UR10_l/base_link")
        self.static_T = None
        if self.static_tf_included:
            tf_listener = TransformListener()

            while not rospy.is_shutdown():
                try:
                    tf_listener.waitForTransform(static_tf_parent, static_tf_child, rospy.Time(0), rospy.Duration(5.0))
                    static_trans, static_rot = tf_listener.lookupTransform(static_tf_parent, static_tf_child, rospy.Time(0))
                    break
                except Exception as e:
                    rospy.logwarn(f"Could not get static transform: {e}. Retrying...")
                    rospy.sleep(1.0)
            self.static_T = quaternion_matrix([static_rot[0], static_rot[1], static_rot[2], static_rot[3]])
            self.static_T[0:3, 3] = static_trans

        self.world_T_base_sub = rospy.Subscriber("world_T_base", PoseStamped, self.world_T_base_callback)
        self.base_T_ee_sub = rospy.Subscriber("base_T_ee", PoseStamped, self.base_T_ee_callback)
        self.world_T_ee_pub = rospy.Publisher("world_T_ee", PoseStamped, queue_size=1)

    def world_T_base_callback(self, msg: PoseStamped):
        self.world_T_base = pose_to_matrix(msg.pose)
        self.world_T_ee.header.stamp = msg.header.stamp
        self.update_world_T_ee()

    def base_T_ee_callback(self, msg: PoseStamped):
        base_T_ee = pose_to_matrix(msg.pose)
        if self.static_tf_included:
            base_T_ee = concatenate_matrices(self.static_T, base_T_ee)
        self.base_T_ee = base_T_ee

        self.world_T_ee.header.stamp = msg.header.stamp
        self.update_world_T_ee()

    def update_world_T_ee(self):
        
        # self.world_T_ee.pose = transform_pose_by_pose(self.world_T_base, self.base_T_ee, (False, False))

        world_T_ee = concatenate_matrices(self.world_T_base, self.base_T_ee)    
        self.world_T_ee.pose = matrix_to_pose(world_T_ee)
        self.world_T_ee_pub.publish(self.world_T_ee)

if __name__ == "__main__":
    rospy.init_node("total_pose")
    total_pose = TotalPose()
    rospy.spin()