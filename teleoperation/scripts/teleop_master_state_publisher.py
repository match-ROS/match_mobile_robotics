#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
import tf2_ros


def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    # (w,x,y,z)
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_axis_angle(q):
    # q assumed normalized, returns (axis_x,axis_y,axis_z, angle)
    qw, qx, qy, qz = q
    qw = max(-1.0, min(1.0, qw))
    angle = 2.0 * math.acos(qw)
    s = math.sqrt(max(0.0, 1.0 - qw * qw))
    if s < 1e-12 or abs(angle) < 1e-12:
        return (0.0, 0.0, 0.0, 0.0)
    return (qx / s, qy / s, qz / s, angle)


class MasterStatePublisher:
    def __init__(self):
        self.master_global_frame = rospy.get_param("~master_global_frame", "world")
        self.master_ee_frame = rospy.get_param("~master_ee_frame", "tool0")

        self.target_pose_topic = rospy.get_param("~target_pose_topic", "target_pose")
        self.feedforward_twist_topic = rospy.get_param("~feedforward_twist_topic", "feedforward_twist")

        self.rate_hz = float(rospy.get_param("~rate", 250.0))
        self.k_twist = float(rospy.get_param("~k_twist", 1.0))
        self.twist_ema_alpha = float(rospy.get_param("~twist_ema_alpha", 0.0))  # 0 disables filtering

        self.pub_pose = rospy.Publisher(self.target_pose_topic, PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher(self.feedforward_twist_topic, TwistStamped, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.prev_t = None
        self.prev_pos = None  # (x,y,z)
        self.prev_quat = None  # (w,x,y,z)
        self.prev_twist = None  # (vx,vy,vz, wx,wy,wz)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            r.sleep()

    def step(self):
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.master_global_frame,
                self.master_ee_frame,
                rospy.Time(0),
                rospy.Duration(0.02),
            )
        except Exception as ex:
            rospy.logwarn_throttle(1.0, "teleop_master_state_publisher: TF lookup failed: %s", str(ex))
            return

        now = rospy.Time.now()
        pos = (
            tfm.transform.translation.x,
            tfm.transform.translation.y,
            tfm.transform.translation.z,
        )
        quat = quat_normalize(
            (
                tfm.transform.rotation.w,
                tfm.transform.rotation.x,
                tfm.transform.rotation.y,
                tfm.transform.rotation.z,
            )
        )

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.master_global_frame
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        self.pub_pose.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = self.master_global_frame

        if self.prev_t is not None:
            dt = (now - self.prev_t).to_sec()
            if dt > 1e-4:
                vx = (pos[0] - self.prev_pos[0]) / dt
                vy = (pos[1] - self.prev_pos[1]) / dt
                vz = (pos[2] - self.prev_pos[2]) / dt

                # orientation delta: q_delta = q_curr * q_prev^{-1}
                q_prev_inv = quat_conj(self.prev_quat)
                q_delta = quat_mul(quat, q_prev_inv)

                # shortest path
                if q_delta[0] < 0.0:
                    q_delta = (-q_delta[0], -q_delta[1], -q_delta[2], -q_delta[3])

                ax, ay, az, angle = quat_to_axis_angle(quat_normalize(q_delta))
                wx = (angle * ax) / dt
                wy = (angle * ay) / dt
                wz = (angle * az) / dt

                raw = (self.k_twist * vx, self.k_twist * vy, self.k_twist * vz,
                       self.k_twist * wx, self.k_twist * wy, self.k_twist * wz)

                if self.prev_twist is not None and self.twist_ema_alpha > 0.0:
                    a = max(0.0, min(1.0, self.twist_ema_alpha))
                    filt = tuple(a * raw[i] + (1.0 - a) * self.prev_twist[i] for i in range(6))
                else:
                    filt = raw

                twist.twist.linear.x = filt[0]
                twist.twist.linear.y = filt[1]
                twist.twist.linear.z = filt[2]
                twist.twist.angular.x = filt[3]
                twist.twist.angular.y = filt[4]
                twist.twist.angular.z = filt[5]

                self.prev_twist = filt

        self.prev_t = now
        self.prev_pos = pos
        self.prev_quat = quat

        self.pub_twist.publish(twist)


if __name__ == "__main__":
    rospy.init_node("teleop_master_state_publisher")
    node = MasterStatePublisher()
    node.spin()

