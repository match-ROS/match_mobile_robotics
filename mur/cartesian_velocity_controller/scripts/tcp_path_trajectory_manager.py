#!/usr/bin/env python3
"""Publish constant-speed TCP pose setpoints along a simple 3D polyline."""

import csv
import math
import os
from typing import List, Optional, Sequence, Tuple

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker


Point3 = Tuple[float, float, float]


def _as_float(value, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be numeric, got {value!r}")


def _as_bool(value, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("true", "1", "yes", "y", "on"):
            return True
        if lowered in ("false", "0", "no", "n", "off"):
            return False
    raise ValueError(f"{name} must be boolean, got {value!r}")


def _point_from_value(value, index: int) -> Point3:
    if isinstance(value, dict):
        return (
            _as_float(value.get("x"), f"points[{index}].x"),
            _as_float(value.get("y"), f"points[{index}].y"),
            _as_float(value.get("z"), f"points[{index}].z"),
        )

    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (
            _as_float(value[0], f"points[{index}][0]"),
            _as_float(value[1], f"points[{index}][1]"),
            _as_float(value[2], f"points[{index}][2]"),
        )

    raise ValueError(f"points[{index}] must be [x, y, z] or {{x, y, z}}")


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _quaternion_from_value(value) -> Quaternion:
    if isinstance(value, dict):
        return Quaternion(
            x=_as_float(value.get("x", 0.0), "orientation.x"),
            y=_as_float(value.get("y", 0.0), "orientation.y"),
            z=_as_float(value.get("z", 0.0), "orientation.z"),
            w=_as_float(value.get("w", 1.0), "orientation.w"),
        )

    if isinstance(value, (list, tuple)) and len(value) >= 4:
        return Quaternion(
            x=_as_float(value[0], "orientation[0]"),
            y=_as_float(value[1], "orientation[1]"),
            z=_as_float(value[2], "orientation[2]"),
            w=_as_float(value[3], "orientation[3]"),
        )

    raise ValueError("orientation must be [x, y, z, w] or {x, y, z, w}")


def _distance(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _rotate_vector(q: Quaternion, v: Point3) -> Point3:
    # Quaternion-vector multiplication implemented explicitly to keep the node dependency-light.
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return v
    x, y, z, w = x / n, y / n, z / n, w / n

    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])

    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def _transform_point(transform, point: Point3) -> Point3:
    rotated = _rotate_vector(transform.transform.rotation, point)
    translation = transform.transform.translation
    return (
        rotated[0] + translation.x,
        rotated[1] + translation.y,
        rotated[2] + translation.z,
    )


class PolylinePath:
    def __init__(self, points: Sequence[Point3]):
        if len(points) < 2:
            raise ValueError("path must contain at least two points")

        self.points = list(points)
        self.segments: List[Tuple[Point3, Point3, float]] = []
        self.total_length = 0.0

        for i in range(len(self.points) - 1):
            segment_length = _distance(self.points[i], self.points[i + 1])
            if segment_length <= 1e-9:
                rospy.logwarn("Skipping zero-length segment at index %d", i)
                continue
            self.segments.append((self.points[i], self.points[i + 1], segment_length))
            self.total_length += segment_length

        if self.total_length <= 1e-9:
            raise ValueError("path total length is zero")

    def sample(self, distance_along_path: float) -> Point3:
        s = min(max(distance_along_path, 0.0), self.total_length)
        remaining = s

        for a, b, segment_length in self.segments:
            if remaining <= segment_length:
                t = remaining / segment_length
                return (
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                )
            remaining -= segment_length

        return self.points[-1]

    def tangent(self, distance_along_path: float) -> Point3:
        s = min(max(distance_along_path, 0.0), self.total_length)
        remaining = s

        for a, b, segment_length in self.segments:
            if remaining <= segment_length:
                return (
                    (b[0] - a[0]) / segment_length,
                    (b[1] - a[1]) / segment_length,
                    (b[2] - a[2]) / segment_length,
                )
            remaining -= segment_length

        a, b, segment_length = self.segments[-1]
        return (
            (b[0] - a[0]) / segment_length,
            (b[1] - a[1]) / segment_length,
            (b[2] - a[2]) / segment_length,
        )


def _load_csv(path_file: str) -> List[Point3]:
    points: List[Point3] = []
    with open(path_file, newline="") as handle:
        reader = csv.reader(handle)
        for line_number, row in enumerate(reader, start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 3:
                raise ValueError(f"{path_file}:{line_number}: expected x,y,z")
            points.append((_as_float(row[0], "x"), _as_float(row[1], "y"), _as_float(row[2], "z")))
    return points


def _load_yaml(path_file: str):
    with open(path_file) as handle:
        data = yaml.safe_load(handle)
    if data is None:
        raise ValueError(f"{path_file} is empty")
    return data


class TcpPathTrajectoryManager:
    def __init__(self):
        self.path_file = rospy.get_param("~path_file", "")
        if not self.path_file:
            raise ValueError("~path_file is required")
        self.path_file = os.path.expanduser(self.path_file)

        path_data = self._load_path_data(self.path_file)
        self.frame_id = str(rospy.get_param("~frame_id", path_data.get("frame_id", "map")))
        self.speed = float(rospy.get_param("~speed", path_data.get("speed", 0.03)))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.loop = _as_bool(rospy.get_param("~loop", False), "loop")
        self.hold_final_pose = _as_bool(rospy.get_param("~hold_final_pose", True), "hold_final_pose")
        self.start_paused = _as_bool(rospy.get_param("~start_paused", False), "start_paused")
        self.marker_scale = float(rospy.get_param("~marker_scale", 0.02))
        self.base_control_enabled = _as_bool(rospy.get_param("~base_control/enabled", False), "base_control/enabled")
        self.base_frame = str(rospy.get_param("~base_control/base_frame", "base_link"))
        self.base_cmd_vel_topic = str(rospy.get_param("~base_control/cmd_vel_topic", "cmd_vel"))
        self.base_preferred_x = float(rospy.get_param("~base_control/preferred_tcp_x", 0.65))
        self.base_preferred_y = float(rospy.get_param("~base_control/preferred_tcp_y", 0.0))
        self.base_kx = float(rospy.get_param("~base_control/kx", 0.35))
        self.base_ky = float(rospy.get_param("~base_control/ky", 0.8))
        self.base_k_heading = float(rospy.get_param("~base_control/k_heading", 0.4))
        self.base_max_linear = abs(float(rospy.get_param("~base_control/max_linear_velocity", 0.08)))
        self.base_max_angular = abs(float(rospy.get_param("~base_control/max_angular_velocity", 0.25)))
        self.base_x_deadband = abs(float(rospy.get_param("~base_control/x_deadband", 0.05)))
        self.base_y_deadband = abs(float(rospy.get_param("~base_control/y_deadband", 0.04)))
        self.base_heading_deadband = abs(float(rospy.get_param("~base_control/heading_deadband", 0.10)))
        self.base_allow_reverse = _as_bool(rospy.get_param("~base_control/allow_reverse", False), "base_control/allow_reverse")
        self.base_align_to_path = _as_bool(rospy.get_param("~base_control/align_to_path", True), "base_control/align_to_path")
        self.base_tf_timeout = float(rospy.get_param("~base_control/tf_timeout", 0.05))

        if self.speed <= 0.0:
            raise ValueError("speed must be > 0")
        if self.rate_hz <= 0.0:
            raise ValueError("rate must be > 0")

        points = [_point_from_value(p, i) for i, p in enumerate(path_data["points"])]
        self.path = PolylinePath(points)
        self.orientation = self._load_orientation(path_data)

        target_topic = rospy.get_param("~target_pose_topic", "target_pose")
        self.target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=1)
        self.progress_pub = rospy.Publisher("~progress", Float64, queue_size=1, latch=True)
        self.path_marker_pub = rospy.Publisher("~path_marker", Marker, queue_size=1, latch=True)
        self.current_marker_pub = rospy.Publisher("~current_marker", Marker, queue_size=1)
        self.base_cmd_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        if self.base_control_enabled:
            self.base_cmd_pub = rospy.Publisher(self.base_cmd_vel_topic, Twist, queue_size=1)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.paused = self.start_paused
        self.stopped = False
        self.done = False
        self.has_started = not self.start_paused
        self.distance_offset = 0.0
        self.last_time: Optional[rospy.Time] = None

        rospy.Service("~pause", Trigger, self._pause_cb)
        rospy.Service("~resume", Trigger, self._resume_cb)
        rospy.Service("~restart", Trigger, self._restart_cb)
        rospy.Service("~stop", Trigger, self._stop_cb)

        rospy.loginfo(
            "TCP path loaded: %d points, %.3f m total, %.3f m/s, frame '%s'",
            len(self.path.points),
            self.path.total_length,
            self.speed,
            self.frame_id,
        )
        if self.base_control_enabled:
            rospy.loginfo(
                "Base C-light enabled: topic '%s', base_frame '%s', preferred TCP [%.2f, %.2f]",
                self.base_cmd_vel_topic,
                self.base_frame,
                self.base_preferred_x,
                self.base_preferred_y,
            )

    def _load_path_data(self, path_file: str):
        if path_file.lower().endswith(".csv"):
            return {"points": _load_csv(path_file)}

        data = _load_yaml(path_file)
        if isinstance(data, list):
            return {"points": data}
        if isinstance(data, dict) and "points" in data:
            return data
        raise ValueError("path file must contain a 'points' list")

    def _load_orientation(self, path_data) -> Quaternion:
        if "orientation" in path_data:
            return _quaternion_from_value(path_data["orientation"])

        rpy = path_data.get("orientation_rpy", rospy.get_param("~orientation_rpy", [0.0, 0.0, 0.0]))
        if not isinstance(rpy, (list, tuple)) or len(rpy) < 3:
            raise ValueError("orientation_rpy must be [roll, pitch, yaw]")
        return _quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

    def _pause_cb(self, _req):
        self.paused = True
        self._publish_zero_base()
        return TriggerResponse(success=True, message="paused")

    def _resume_cb(self, _req):
        self.paused = False
        self.stopped = False
        self.has_started = True
        self.last_time = rospy.Time.now()
        return TriggerResponse(success=True, message="resumed")

    def _restart_cb(self, _req):
        self.distance_offset = 0.0
        self.done = False
        self.stopped = False
        self.paused = self.start_paused
        self.has_started = not self.start_paused
        self.last_time = rospy.Time.now()
        self._publish_zero_base()
        return TriggerResponse(success=True, message="restarted")

    def _stop_cb(self, _req):
        self.stopped = True
        self.paused = True
        self._publish_zero_base()
        return TriggerResponse(success=True, message="stopped")

    def run(self):
        self._publish_path_marker()
        self.last_time = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(0.0, (now - self.last_time).to_sec()) if self.last_time else 0.0
            self.last_time = now

            if self.has_started and not self.paused and not self.stopped and not self.done:
                self.distance_offset += self.speed * dt
                if self.distance_offset >= self.path.total_length:
                    if self.loop:
                        self.distance_offset = math.fmod(self.distance_offset, self.path.total_length)
                    else:
                        self.distance_offset = self.path.total_length
                        self.done = True

            active_motion = self.has_started and not self.paused and not self.stopped and not self.done

            if self.has_started and not self.stopped and (self.hold_final_pose or not self.done):
                point = self.path.sample(self.distance_offset)
                self._publish_pose(point, now)
                self._publish_current_marker(point, now)
                if active_motion:
                    tangent = self.path.tangent(self.distance_offset)
                    self._publish_base_command(point, tangent)
                else:
                    self._publish_zero_base()
            else:
                self._publish_zero_base()

            progress = min(self.distance_offset / self.path.total_length, 1.0)
            self.progress_pub.publish(Float64(data=progress))
            rate.sleep()

    def _publish_pose(self, point: Point3, stamp: rospy.Time):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = point[2]
        msg.pose.orientation = self.orientation
        self.target_pub.publish(msg)

    def _publish_base_command(self, point: Point3, tangent: Point3):
        if not self.base_control_enabled or self.base_cmd_pub is None or self.tf_buffer is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.frame_id,
                rospy.Time(0),
                rospy.Duration(self.base_tf_timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Base C-light TF failed (%s -> %s): %s",
                self.frame_id,
                self.base_frame,
                exc,
            )
            self._publish_zero_base()
            return

        target_in_base = _transform_point(transform, point)
        tangent_in_base = _rotate_vector(transform.transform.rotation, tangent)

        x_error = target_in_base[0] - self.base_preferred_x
        y_error = target_in_base[1] - self.base_preferred_y

        linear = 0.0 if abs(x_error) < self.base_x_deadband else self.base_kx * x_error
        if not self.base_allow_reverse:
            linear = max(0.0, linear)
        linear = _clamp(linear, -self.base_max_linear, self.base_max_linear)

        lateral_term = 0.0 if abs(y_error) < self.base_y_deadband else self.base_ky * y_error
        heading_term = 0.0
        if self.base_align_to_path:
            heading = _normalize_angle(math.atan2(tangent_in_base[1], tangent_in_base[0]))
            if abs(heading) >= self.base_heading_deadband:
                heading_term = self.base_k_heading * heading

        angular = _clamp(lateral_term + heading_term, -self.base_max_angular, self.base_max_angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.base_cmd_pub.publish(cmd)

    def _publish_zero_base(self):
        if self.base_control_enabled and self.base_cmd_pub is not None:
            self.base_cmd_pub.publish(Twist())

    def _publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tcp_print_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.path.points]
        self.path_marker_pub.publish(marker)

    def _publish_current_marker(self, point: Point3, stamp: rospy.Time):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "tcp_print_path"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale * 4.0
        marker.scale.y = self.marker_scale * 4.0
        marker.scale.z = self.marker_scale * 4.0
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.current_marker_pub.publish(marker)


def main():
    rospy.init_node("tcp_path_trajectory_manager")
    try:
        TcpPathTrajectoryManager().run()
    except Exception as exc:
        rospy.logfatal("tcp_path_trajectory_manager failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
