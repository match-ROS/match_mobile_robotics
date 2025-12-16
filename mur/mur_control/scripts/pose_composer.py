#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
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
        self.base_T_ee_raw = np.eye(4)
        self.base_T_ee = np.eye(4)
        self.world_T_ee = PoseStamped()
        self.world_T_ee.header.frame_id = rospy.get_param("~world_frame", "map")

        # In case there is a static transform between the 2 topics:
        self.static_tf_included = rospy.get_param("~static_tf_included", False)
        static_tf_parent = rospy.get_param("~static_tf_parent", "mur620/base_footprint")
        static_tf_child = rospy.get_param("~static_tf_child", "mur620/UR10_l/base_link")
        self.mocap_mode = rospy.get_param("~mocap_mode", False)
        self.static_T = None
        self.tf_listener = TransformListener()

        self.static_T = np.eye(4)
        self.lift_pre_T = np.eye(4)
        self.lift_post_T = np.eye(4)
        self.dynamic_lift_T = np.eye(4)
        self.lift_transforms_ready = False

        self.lift_enabled = rospy.get_param("~lift_enabled", False)
        self.lift_joint_name = rospy.get_param("~lift_joint_name", "")
        self.lift_joint_states_topic = rospy.get_param("~lift_joint_states_topic", "/joint_states")
        self.lift_bottom_frame = rospy.get_param("~lift_bottom_frame", "")
        self.lift_top_frame = rospy.get_param("~lift_top_frame", "")
        axis = rospy.get_param("~lift_axis", "z").lower()
        axis_map = {"x": 0, "y": 1, "z": 2}
        if axis not in axis_map:
            rospy.logwarn(f"Invalid lift axis '{axis}'. Falling back to 'z'.")
            axis = "z"
        self.lift_axis_index = axis_map[axis]

        if self.lift_enabled:
            missing = []
            if not self.lift_joint_name:
                missing.append("lift_joint_name")
            if not self.lift_bottom_frame:
                missing.append("lift_bottom_frame")
            if not self.lift_top_frame:
                missing.append("lift_top_frame")
            if not self.lift_joint_states_topic:
                missing.append("lift_joint_states_topic")
            if missing:
                rospy.logwarn(
                    "Lift integration requested but missing parameters: %s. Disabling lift integration." % ", ".join(missing)
                )
                self.lift_enabled = False

        if self.static_tf_included:
            if self.lift_enabled:
                self.lift_pre_T = self._lookup_transform(static_tf_parent, self.lift_bottom_frame)
                self.lift_post_T = self._lookup_transform(self.lift_top_frame, static_tf_child)
                self.lift_transforms_ready = True
            else:
                self.static_T = self._lookup_transform(static_tf_parent, static_tf_child)
        elif self.lift_enabled:
            rospy.logwarn("Lift integration requires static transforms. Disabling lift.")
            self.lift_enabled = False

        self.world_T_base_sub = rospy.Subscriber("world_T_base", PoseStamped, self.world_T_base_callback)
        self.base_T_ee_sub = rospy.Subscriber("base_T_ee", PoseStamped, self.base_T_ee_callback)
        self.world_T_ee_pub = rospy.Publisher("world_T_ee", PoseStamped, queue_size=1)
        self.lift_joint_sub = None
        if self.lift_enabled:
            self.lift_joint_sub = rospy.Subscriber(
                self.lift_joint_states_topic, JointState, self.lift_joint_callback, queue_size=1
            )

    def _lookup_transform(self, parent: str, child: str) -> np.ndarray:
        if not parent or not child:
            rospy.logwarn("Cannot lookup transform with empty frame ids. Using identity transform.")
            return np.eye(4)

        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(5.0))
                trans, rot = self.tf_listener.lookupTransform(parent, child, rospy.Time(0))
                matrix = quaternion_matrix([rot[0], rot[1], rot[2], rot[3]])
                matrix[0:3, 3] = trans
                return matrix
            except Exception as e:
                rospy.logwarn(f"Could not get transform from {parent} to {child}: {e}. Retrying...")
                rospy.sleep(1.0)
        return np.eye(4)

    def _apply_static_chain(self, base_T_ee: np.ndarray) -> np.ndarray:
        if not self.static_tf_included:
            return base_T_ee
        if self.lift_enabled and self.lift_transforms_ready:
            return concatenate_matrices(self.lift_pre_T, self.dynamic_lift_T, self.lift_post_T, base_T_ee)
        return concatenate_matrices(self.static_T, base_T_ee)

    def world_T_base_callback(self, msg: PoseStamped):
        self.world_T_base = pose_to_matrix(msg.pose)
        self.world_T_ee.header.stamp = msg.header.stamp
        self.update_world_T_ee()

    def base_T_ee_callback(self, msg: PoseStamped):
        base_T_ee = pose_to_matrix(msg.pose)
        self.base_T_ee_raw = base_T_ee
        self.base_T_ee = self._apply_static_chain(base_T_ee)

        self.world_T_ee.header.stamp = msg.header.stamp
        if not self.mocap_mode:
            self.update_world_T_ee()

    def lift_joint_callback(self, msg: JointState):
        try:
            lift_index = msg.name.index(self.lift_joint_name)
        except ValueError:
            rospy.logwarn_throttle(5.0, f"Lift joint '{self.lift_joint_name}' not found in JointState message. Waiting...")
            return

        if lift_index >= len(msg.position):
            return

        translation_value = msg.position[lift_index]
        translation = [0.0, 0.0, 0.0]
        translation[self.lift_axis_index] = translation_value
        self.dynamic_lift_T[0:3, 3] = translation

        self.base_T_ee = self._apply_static_chain(self.base_T_ee_raw)
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