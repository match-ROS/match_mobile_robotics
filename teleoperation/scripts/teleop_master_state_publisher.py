#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
class PipelineDebugMasterBridge:
    """
    - Subscribes to cartesian_velocity_controller/PipelineDebug
    - Extracts:
        - current_pose            -> PoseStamped
        - cartesian_cmd_{lin,ang} -> TwistStamped
    - Republishes to:
        - ~target_pose_topic
        - ~feedforward_twist_topic
    """

    def __init__(self):
        self.pipeline_debug_topic = rospy.get_param(
            "~pipeline_debug_topic",
            "cartesian_velocity_controller_r/pipeline_debug",
        )

        self.target_pose_topic = rospy.get_param("~target_pose_topic", "target_pose_teleop")
        self.feedforward_twist_topic = rospy.get_param("~feedforward_twist_topic", "feedforward_twist_teleop")

        # Optional override (if empty, use msg.header.frame_id)
        self.frame_id_override = rospy.get_param("~frame_id_override", "")

        # Import message at runtime (keeps teleoperation package lightweight)
        try:
            from cartesian_velocity_controller.msg import PipelineDebug  # noqa: F401
            self._PipelineDebug = PipelineDebug
        except Exception as ex:
            rospy.logerr(
                "teleop_master_state_publisher: cannot import cartesian_velocity_controller/PipelineDebug. "
                "Is the package built and sourced? Error: %s",
                str(ex),
            )
            raise

        self.pub_pose = rospy.Publisher(self.target_pose_topic, PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher(self.feedforward_twist_topic, TwistStamped, queue_size=1)

        self.sub = rospy.Subscriber(
            self.pipeline_debug_topic,
            self._PipelineDebug,
            self._cb,
            queue_size=1,
            tcp_nodelay=True,
        )

    def spin(self):
        rospy.spin()

    def _cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        frame_id = self.frame_id_override if self.frame_id_override else msg.header.frame_id

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose = msg.current_pose
        self.pub_pose.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = stamp
        twist_msg.header.frame_id = frame_id
        twist_msg.twist.linear = msg.cartesian_cmd_linear
        twist_msg.twist.angular = msg.cartesian_cmd_angular
        self.pub_twist.publish(twist_msg)


if __name__ == "__main__":
    rospy.init_node("teleop_master_state_publisher")
    node = PipelineDebugMasterBridge()
    node.spin()

