#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import WrenchStamped


class FakeWrenchHoldPublisher:
    def __init__(self):
        self.wrench_topic = rospy.get_param("~wrench_topic", "wrench")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "wrench_setpoint")
        self.publish_rate = float(rospy.get_param("~publish_rate", 250.0))

        # If non-empty, force header.frame_id to this value. Otherwise, use the last received setpoint frame_id.
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.use_msg_frame_id = bool(rospy.get_param("~use_msg_frame_id", False))

        # Held setpoint (default: all zeros).
        self._held = WrenchStamped()
        self._held.header.frame_id = self.frame_id

        self.pub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=1)
        self.sub = rospy.Subscriber(self.setpoint_topic, WrenchStamped, self._cb, queue_size=1)

        rospy.loginfo("fake_wrench_hold_publisher publishing to '%s' (rate=%.1f Hz), setpoint '%s'",
                      rospy.resolve_name(self.wrench_topic), self.publish_rate, rospy.resolve_name(self.setpoint_topic))

    def _cb(self, msg: WrenchStamped):
        self._held = msg

    def spin(self):
        r = rospy.Rate(self.publish_rate if self.publish_rate > 0.0 else 250.0)
        while not rospy.is_shutdown():
            out = WrenchStamped()
            out.header.stamp = rospy.Time.now()

            if self.use_msg_frame_id and self._held.header.frame_id:
                out.header.frame_id = self._held.header.frame_id
            else:
                out.header.frame_id = self.frame_id

            out.wrench = self._held.wrench
            self.pub.publish(out)
            r.sleep()


def main():
    rospy.init_node("fake_wrench_hold_publisher")
    node = FakeWrenchHoldPublisher()
    node.spin()


if __name__ == "__main__":
    main()

