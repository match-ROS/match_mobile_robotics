#!/usr/bin/env python3
"""Publish constant-speed TCP pose setpoints along a simple 3D polyline."""

import csv
import math
import os
from typing import List, Optional, Sequence, Tuple

import rospy
import tf2_ros
import yaml
from cartesian_velocity_controller.msg import CartesianTrajectorySetpoint, EndEffectorState
from cartesian_velocity_controller.srv import ValidatePoses
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Float64, Header
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker


Point3 = Tuple[float, float, float]
QuaternionTuple = Tuple[float, float, float, float]


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


def _quaternion_tuple(q: Quaternion) -> QuaternionTuple:
    return (q.x, q.y, q.z, q.w)


def _quaternion_from_tuple(q: QuaternionTuple) -> Quaternion:
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def _quat_normalize(q: QuaternionTuple) -> QuaternionTuple:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _quat_slerp(a: QuaternionTuple, b: QuaternionTuple, t: float) -> QuaternionTuple:
    qa = _quat_normalize(a)
    qb = _quat_normalize(b)
    dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]
    if dot < 0.0:
        qb = (-qb[0], -qb[1], -qb[2], -qb[3])
        dot = -dot
    dot = _clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        return _quat_normalize((
            qa[0] + t * (qb[0] - qa[0]),
            qa[1] + t * (qb[1] - qa[1]),
            qa[2] + t * (qb[2] - qa[2]),
            qa[3] + t * (qb[3] - qa[3]),
        ))
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * _clamp(t, 0.0, 1.0)
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return (
        s0 * qa[0] + s1 * qb[0],
        s0 * qa[1] + s1 * qb[1],
        s0 * qa[2] + s1 * qb[2],
        s0 * qa[3] + s1 * qb[3],
    )


def _quat_inverse(q: QuaternionTuple) -> QuaternionTuple:
    qn = _quat_normalize(q)
    return (-qn[0], -qn[1], -qn[2], qn[3])


def _quat_multiply(a: QuaternionTuple, b: QuaternionTuple) -> QuaternionTuple:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_to_axis_angle(q: QuaternionTuple) -> Point3:
    qn = _quat_normalize(q)
    if qn[3] < 0.0:
        qn = (-qn[0], -qn[1], -qn[2], -qn[3])
    angle = 2.0 * math.acos(_clamp(qn[3], -1.0, 1.0))
    s = math.sqrt(max(0.0, 1.0 - qn[3] * qn[3]))
    if s < 1e-9 or angle < 1e-9:
        return (0.0, 0.0, 0.0)
    return (qn[0] / s * angle, qn[1] / s * angle, qn[2] / s * angle)


def _distance(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(v: Point3, s: float) -> Point3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _norm(v: Point3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v: Point3) -> Point3:
    n = _norm(v)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


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


class PathSegment:
    def __init__(self):
        self.length = 0.0

    def sample(self, u: float) -> Point3:
        raise NotImplementedError

    def tangent(self, u: float) -> Point3:
        raise NotImplementedError


class LineSegment(PathSegment):
    def __init__(self, a: Point3, b: Point3):
        super().__init__()
        self.a = a
        self.b = b
        self.delta = _sub(b, a)
        self.length = _norm(self.delta)

    def sample(self, u: float) -> Point3:
        return _add(self.a, _scale(self.delta, _clamp(u, 0.0, 1.0)))

    def tangent(self, _u: float) -> Point3:
        return _unit(self.delta)


class CubicBezierSegment(PathSegment):
    def __init__(self, p0: Point3, p1: Point3, p2: Point3, p3: Point3, samples: int = 24):
        super().__init__()
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self._length_lut: List[Tuple[float, float]] = [(0.0, 0.0)]
        prev = self.sample(0.0)
        total = 0.0
        n = max(4, samples)
        for i in range(1, n + 1):
            u = float(i) / float(n)
            cur = self.sample(u)
            total += _distance(prev, cur)
            self._length_lut.append((u, total))
            prev = cur
        self.length = total

    def sample(self, u: float) -> Point3:
        t = _clamp(u, 0.0, 1.0)
        omt = 1.0 - t
        return (
            omt ** 3 * self.p0[0] + 3.0 * omt * omt * t * self.p1[0] + 3.0 * omt * t * t * self.p2[0] + t ** 3 * self.p3[0],
            omt ** 3 * self.p0[1] + 3.0 * omt * omt * t * self.p1[1] + 3.0 * omt * t * t * self.p2[1] + t ** 3 * self.p3[1],
            omt ** 3 * self.p0[2] + 3.0 * omt * omt * t * self.p1[2] + 3.0 * omt * t * t * self.p2[2] + t ** 3 * self.p3[2],
        )

    def tangent(self, u: float) -> Point3:
        t = _clamp(u, 0.0, 1.0)
        omt = 1.0 - t
        d = (
            3.0 * omt * omt * (self.p1[0] - self.p0[0]) + 6.0 * omt * t * (self.p2[0] - self.p1[0]) + 3.0 * t * t * (self.p3[0] - self.p2[0]),
            3.0 * omt * omt * (self.p1[1] - self.p0[1]) + 6.0 * omt * t * (self.p2[1] - self.p1[1]) + 3.0 * t * t * (self.p3[1] - self.p2[1]),
            3.0 * omt * omt * (self.p1[2] - self.p0[2]) + 6.0 * omt * t * (self.p2[2] - self.p1[2]) + 3.0 * t * t * (self.p3[2] - self.p2[2]),
        )
        return _unit(d)

    def u_at_distance(self, distance_along_segment: float) -> float:
        if self.length <= 1e-12:
            return 0.0
        s = _clamp(distance_along_segment, 0.0, self.length)
        for i in range(1, len(self._length_lut)):
            u0, s0 = self._length_lut[i - 1]
            u1, s1 = self._length_lut[i]
            if s <= s1:
                span = max(1e-12, s1 - s0)
                return u0 + (u1 - u0) * ((s - s0) / span)
        return 1.0


class SmoothedPath:
    def __init__(self, points: Sequence[Point3], blend_tolerance: float = 0.0):
        if len(points) < 2:
            raise ValueError("path must contain at least two points")

        self.points = list(points)
        self.segments: List[PathSegment] = []
        self.total_length = 0.0
        self.waypoint_s: List[float] = []
        self._build(max(0.0, blend_tolerance))

        if self.total_length <= 1e-9:
            raise ValueError("path total length is zero")

    def _append_segment(self, segment: PathSegment):
        if segment.length <= 1e-9:
            return
        self.segments.append(segment)
        self.total_length += segment.length

    def _build(self, blend_tolerance: float):
        if blend_tolerance <= 1e-9 or len(self.points) < 3:
            self.waypoint_s = [0.0]
            for i in range(len(self.points) - 1):
                seg = LineSegment(self.points[i], self.points[i + 1])
                self._append_segment(seg)
                self.waypoint_s.append(self.total_length)
            return

        self.waypoint_s = [0.0]
        current = self.points[0]
        for i in range(1, len(self.points) - 1):
            prev_pt = self.points[i - 1]
            corner = self.points[i]
            next_pt = self.points[i + 1]
            len_prev = _distance(prev_pt, corner)
            len_next = _distance(corner, next_pt)
            if len_prev <= 1e-9 or len_next <= 1e-9:
                self._append_segment(LineSegment(current, corner))
                self.waypoint_s.append(self.total_length)
                current = corner
                continue

            d = min(blend_tolerance, 0.45 * len_prev, 0.45 * len_next)
            if d <= 1e-9:
                self._append_segment(LineSegment(current, corner))
                self.waypoint_s.append(self.total_length)
                current = corner
                continue

            incoming = _unit(_sub(corner, prev_pt))
            outgoing = _unit(_sub(next_pt, corner))
            entry = _sub(corner, _scale(incoming, d))
            exit_pt = _add(corner, _scale(outgoing, d))

            self._append_segment(LineSegment(current, entry))

            handle = 0.55 * d
            c1 = _add(entry, _scale(incoming, handle))
            c2 = _sub(exit_pt, _scale(outgoing, handle))
            before_blend_s = self.total_length
            bezier = CubicBezierSegment(entry, c1, c2, exit_pt)
            self._append_segment(bezier)
            self.waypoint_s.append(before_blend_s + 0.5 * bezier.length)
            current = exit_pt

        self._append_segment(LineSegment(current, self.points[-1]))
        self.waypoint_s.append(self.total_length)

    def _locate(self, distance_along_path: float) -> Tuple[PathSegment, float]:
        s = min(max(distance_along_path, 0.0), self.total_length)
        remaining = s
        for segment in self.segments:
            if remaining <= segment.length:
                if isinstance(segment, CubicBezierSegment):
                    return segment, segment.u_at_distance(remaining)
                return segment, remaining / max(1e-12, segment.length)
            remaining -= segment.length
        return self.segments[-1], 1.0

    def sample(self, distance_along_path: float) -> Point3:
        segment, u = self._locate(distance_along_path)
        return segment.sample(u)

    def tangent(self, distance_along_path: float) -> Point3:
        segment, u = self._locate(distance_along_path)
        return segment.tangent(u)

    def marker_points(self, samples_per_curve: int = 12) -> List[Point3]:
        pts: List[Point3] = []
        for segment in self.segments:
            n = samples_per_curve if isinstance(segment, CubicBezierSegment) else 1
            for i in range(n + 1):
                if pts and i == 0:
                    continue
                pts.append(segment.sample(float(i) / float(max(1, n))))
        return pts


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


def _orientation_from_waypoint(value, inherited: QuaternionTuple, index: int) -> QuaternionTuple:
    if not isinstance(value, dict):
        return inherited
    if "orientation" in value:
        return _quaternion_tuple(_quaternion_from_value(value["orientation"]))
    if "orientation_rpy" in value:
        rpy = value["orientation_rpy"]
        if not isinstance(rpy, (list, tuple)) or len(rpy) < 3:
            raise ValueError(f"waypoints[{index}].orientation_rpy must be [roll, pitch, yaw]")
        return _quaternion_tuple(_quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2])))
    return inherited


def _point_from_waypoint(value, index: int) -> Point3:
    if isinstance(value, dict):
        if "position" in value:
            return _point_from_value(value["position"], index)
        return _point_from_value(value, index)
    return _point_from_value(value, index)


class OrientationProfile:
    def __init__(self, s_values: Sequence[float], orientations: Sequence[QuaternionTuple]):
        if len(s_values) != len(orientations) or not s_values:
            raise ValueError("orientation profile requires matching s/orientation entries")
        self.s_values = list(s_values)
        self.orientations = [_quat_normalize(q) for q in orientations]

    def sample(self, s: float) -> QuaternionTuple:
        if s <= self.s_values[0]:
            return self.orientations[0]
        if s >= self.s_values[-1]:
            return self.orientations[-1]
        for i in range(1, len(self.s_values)):
            s0 = self.s_values[i - 1]
            s1 = self.s_values[i]
            if s <= s1:
                t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
                return _quat_slerp(self.orientations[i - 1], self.orientations[i], t)
        return self.orientations[-1]

    def angular_velocity(self, s: float, s_dot: float, total_length: float) -> Point3:
        if abs(s_dot) <= 1e-9 or total_length <= 1e-9:
            return (0.0, 0.0, 0.0)
        ds = max(1e-4, min(0.01, total_length * 1e-3))
        s0 = _clamp(s, 0.0, total_length)
        s1 = _clamp(s0 + (ds if s_dot >= 0.0 else -ds), 0.0, total_length)
        actual_ds = s1 - s0
        if abs(actual_ds) <= 1e-9:
            return (0.0, 0.0, 0.0)
        q0 = self.sample(s0)
        q1 = self.sample(s1)
        dq = _quat_multiply(q1, _quat_inverse(q0))
        axis_angle = _quat_to_axis_angle(dq)
        scale = s_dot / actual_ds
        return (axis_angle[0] * scale, axis_angle[1] * scale, axis_angle[2] * scale)


class ScalarMotionProfile:
    def __init__(self, max_velocity: float, max_acceleration: float, max_jerk: float, response_tau: float = 0.08):
        self.max_velocity = abs(max_velocity)
        self.max_acceleration = abs(max_acceleration)
        self.max_jerk = abs(max_jerk)
        self.response_tau = max(1e-3, response_tau)
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0

    def reset(self):
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0

    def update(self, desired_velocity: float, remaining_distance: float, dt: float, loop: bool) -> Tuple[float, float, float]:
        if dt <= 1e-9:
            return self.velocity, self.acceleration, self.jerk

        target_v = _clamp(desired_velocity, 0.0, self.max_velocity)
        if not loop and self.max_acceleration > 1e-9:
            braking_v = math.sqrt(max(0.0, 2.0 * self.max_acceleration * max(0.0, remaining_distance)))
            target_v = min(target_v, braking_v)

        target_acc = (target_v - self.velocity) / self.response_tau
        target_acc = _clamp(target_acc, -self.max_acceleration, self.max_acceleration)

        if self.max_jerk > 1e-9:
            self.jerk = _clamp((target_acc - self.acceleration) / dt, -self.max_jerk, self.max_jerk)
            self.acceleration += self.jerk * dt
        else:
            self.acceleration = target_acc
            self.jerk = 0.0
        self.acceleration = _clamp(self.acceleration, -self.max_acceleration, self.max_acceleration)

        self.velocity += self.acceleration * dt
        if self.velocity < 0.0:
            self.velocity = 0.0
            if self.acceleration < 0.0:
                self.acceleration = 0.0
        self.velocity = min(self.velocity, self.max_velocity)
        return self.velocity, self.acceleration, self.jerk


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
        self.preposition_enabled = _as_bool(rospy.get_param("~preposition/enabled", True), "preposition/enabled")
        self.require_start_reached = _as_bool(
            rospy.get_param("~preposition/require_start_reached",
                            rospy.get_param("~preposition/require_start_reached_on_resume", True)),
            "preposition/require_start_reached",
        )
        self.start_position_tolerance = abs(float(rospy.get_param("~preposition/position_tolerance", 0.01)))
        self.preposition_dwell_s = max(0.0, float(rospy.get_param("~preposition/dwell_s", 1.0)))
        self.ee_state_topic = str(rospy.get_param("~preposition/ee_state_topic", "end_effector_state"))
        self.tracking_guard_enabled = _as_bool(
            rospy.get_param("~tracking_guard/enabled", True), "tracking_guard/enabled")
        self.tracking_guard_slowdown_error = abs(float(rospy.get_param("~tracking_guard/slowdown_error", 0.03)))
        self.tracking_guard_stop_error = abs(float(rospy.get_param("~tracking_guard/stop_error", 0.08)))
        self.tracking_guard_resume_hysteresis = abs(float(
            rospy.get_param("~tracking_guard/resume_hysteresis", 0.05)))
        self.tracking_guard_stopped = False
        self.tracking_guard_last_error: Optional[float] = None
        self.speed_scale = 1.0

        if self.tracking_guard_stop_error <= self.tracking_guard_slowdown_error:
            raise ValueError("tracking_guard/stop_error must be greater than tracking_guard/slowdown_error")

        if self.speed <= 0.0:
            raise ValueError("speed must be > 0")
        if self.rate_hz <= 0.0:
            raise ValueError("rate must be > 0")

        points, orientations = self._load_waypoints(path_data)
        self.waypoint_points = points
        self.waypoint_orientations = orientations
        self.blend_tolerance = abs(float(rospy.get_param("~blend_tolerance", path_data.get("blend_tolerance", 0.0))))
        self.path = SmoothedPath(points, self.blend_tolerance)
        self.orientation_profile = OrientationProfile(self.path.waypoint_s, orientations)

        self.max_linear_velocity = abs(float(rospy.get_param(
            "~max_linear_velocity", path_data.get("max_linear_velocity", self.speed))))
        if self.max_linear_velocity <= 1e-9:
            self.max_linear_velocity = self.speed
        self.max_linear_acceleration = abs(float(rospy.get_param(
            "~max_linear_acceleration", path_data.get("max_linear_acceleration", 0.5))))
        self.max_linear_jerk = abs(float(rospy.get_param(
            "~max_linear_jerk", path_data.get("max_linear_jerk", 5.0))))
        self.motion_profile = ScalarMotionProfile(
            max_velocity=self.max_linear_velocity,
            max_acceleration=self.max_linear_acceleration,
            max_jerk=self.max_linear_jerk,
        )

        target_topic = rospy.get_param("~target_pose_topic", "target_pose")
        target_state_topic = rospy.get_param("~target_state_topic", "target_state")
        self.publish_target_pose = _as_bool(rospy.get_param("~publish_target_pose", True), "publish_target_pose")
        self.validate_waypoints = _as_bool(rospy.get_param("~validate_waypoints", True), "validate_waypoints")
        self.validation_timeout = max(0.0, float(rospy.get_param("~validation_timeout", 2.0)))
        self.validation_service = str(rospy.get_param(
            "~validation_service", self._infer_controller_service(target_state_topic, "validate_poses")))
        self.target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=1)
        self.target_state_pub = rospy.Publisher(target_state_topic, CartesianTrajectorySetpoint, queue_size=1)
        self.progress_pub = rospy.Publisher("~progress", Float64, queue_size=1, latch=True)
        self.speed_scale_pub = rospy.Publisher("~speed_scale", Float64, queue_size=1, latch=True)
        self.path_marker_pub = rospy.Publisher("~path_marker", Marker, queue_size=1, latch=True)
        self.current_marker_pub = rospy.Publisher("~current_marker", Marker, queue_size=1)
        self.base_cmd_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        if self.base_control_enabled:
            self.base_cmd_pub = rospy.Publisher(self.base_cmd_vel_topic, Twist, queue_size=1)
        if self.base_control_enabled or self.require_start_reached or self.tracking_guard_enabled:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.paused = False
        self.stopped = False
        self.done = False
        self.has_started = not self.start_paused
        self.state = "tracking" if self.has_started else "idle"
        self.distance_offset = 0.0
        self.last_time: Optional[rospy.Time] = None
        self.current_tcp: Optional[Point3] = None
        self.current_tcp_frame = ""
        self.last_start_error: Optional[float] = None
        self.dwell_start_time: Optional[rospy.Time] = None
        self.last_profile_state = (0.0, 0.0, 0.0)

        if self.require_start_reached or self.tracking_guard_enabled:
            self.ee_state_sub = rospy.Subscriber(self.ee_state_topic, EndEffectorState, self._ee_state_cb, queue_size=10)

        if self.validate_waypoints:
            self._validate_waypoints_or_raise()

        rospy.Service("~start", Trigger, self._start_cb)
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
        if self.blend_tolerance > 0.0:
            rospy.loginfo("Path smoothing enabled: blend_tolerance=%.3f m", self.blend_tolerance)
        if self.base_control_enabled:
            rospy.loginfo(
                "Base C-light enabled: topic '%s', base_frame '%s', preferred TCP [%.2f, %.2f]",
                self.base_cmd_vel_topic,
                self.base_frame,
                self.base_preferred_x,
                self.base_preferred_y,
            )
        if self.preposition_enabled:
            rospy.loginfo(
                "Preposition enabled: start service moves to first point, waits %.2f s, start tolerance %.3f m",
                self.preposition_dwell_s,
                self.start_position_tolerance,
            )
        if self.tracking_guard_enabled:
            rospy.loginfo(
                "Tracking guard enabled: slowdown %.3f m, stop %.3f m, resume hysteresis %.3f m",
                self.tracking_guard_slowdown_error,
                self.tracking_guard_stop_error,
                self.tracking_guard_resume_hysteresis,
            )

    def _load_path_data(self, path_file: str):
        if path_file.lower().endswith(".csv"):
            return {"points": _load_csv(path_file)}

        data = _load_yaml(path_file)
        if isinstance(data, list):
            return {"points": data}
        if isinstance(data, dict) and ("points" in data or "waypoints" in data):
            return data
        raise ValueError("path file must contain a 'points' or 'waypoints' list")

    @staticmethod
    def _infer_controller_service(target_state_topic: str, service_name: str) -> str:
        topic = target_state_topic.rstrip("/")
        suffix = "/target_state"
        if topic.endswith(suffix):
            return topic[: -len(suffix)] + "/" + service_name
        return service_name

    def _load_waypoints(self, path_data) -> Tuple[List[Point3], List[QuaternionTuple]]:
        default_orientation = _quaternion_tuple(self._load_orientation(path_data))
        points: List[Point3] = []
        orientations: List[QuaternionTuple] = []

        if "waypoints" in path_data:
            inherited = default_orientation
            for i, waypoint in enumerate(path_data["waypoints"]):
                point = _point_from_waypoint(waypoint, i)
                inherited = _orientation_from_waypoint(waypoint, inherited, i)
                points.append(point)
                orientations.append(inherited)
        else:
            points = [_point_from_value(p, i) for i, p in enumerate(path_data["points"])]
            orientations = [default_orientation for _ in points]

        if len(points) != len(orientations):
            raise ValueError("internal waypoint/orientation mismatch")
        return points, orientations

    def _validate_waypoints_or_raise(self):
        if not self.validation_service:
            rospy.logwarn("Waypoint validation requested but validation_service is empty; skipping")
            return

        try:
            rospy.wait_for_service(self.validation_service, timeout=self.validation_timeout)
        except rospy.ROSException as exc:
            raise RuntimeError(f"Waypoint validation service '{self.validation_service}' not available: {exc}")

        poses = []
        for point, quat in zip(self.waypoint_points, self.waypoint_orientations):
            pose = PoseStamped().pose
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            pose.orientation = _quaternion_from_tuple(quat)
            poses.append(pose)

        proxy = rospy.ServiceProxy(self.validation_service, ValidatePoses)
        header = Header(stamp=rospy.Time(0), frame_id=self.frame_id)
        resp = proxy(header, poses)
        if not resp.success:
            invalid = [str(i) for i, ok in enumerate(resp.valid) if not ok]
            raise RuntimeError(
                f"Waypoint validation failed via '{self.validation_service}': {resp.message}; invalid indexes: {', '.join(invalid)}")
        rospy.loginfo("Waypoint validation OK: %d waypoint(s) reachable", len(poses))

    def _load_orientation(self, path_data) -> Quaternion:
        if "orientation" in path_data:
            return _quaternion_from_value(path_data["orientation"])

        rpy = path_data.get("orientation_rpy", rospy.get_param("~orientation_rpy", [0.0, 0.0, 0.0]))
        if not isinstance(rpy, (list, tuple)) or len(rpy) < 3:
            raise ValueError("orientation_rpy must be [roll, pitch, yaw]")
        return _quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

    def _start_cb(self, _req):
        self.distance_offset = 0.0
        self.motion_profile.reset()
        self.last_profile_state = (0.0, 0.0, 0.0)
        self.speed_scale = 1.0
        self.tracking_guard_stopped = False
        self.tracking_guard_last_error = None
        self.done = False
        self.stopped = False
        self.paused = False
        self.has_started = False
        self.dwell_start_time = None
        self.last_time = rospy.Time.now()
        self.state = "preposition" if self.preposition_enabled else "tracking"
        if self.state == "tracking":
            self.has_started = True
        self._publish_zero_base()
        return TriggerResponse(success=True, message=f"started {self.state}")

    def _pause_cb(self, _req):
        self.paused = True
        self.speed_scale = 0.0
        self._publish_zero_base()
        return TriggerResponse(success=True, message="paused")

    def _resume_cb(self, _req):
        if self.state == "idle":
            return self._start_cb(_req)
        self.paused = False
        self.stopped = False
        self.last_time = rospy.Time.now()
        return TriggerResponse(success=True, message="resumed")

    def _restart_cb(self, _req):
        self.distance_offset = 0.0
        self.motion_profile.reset()
        self.last_profile_state = (0.0, 0.0, 0.0)
        self.speed_scale = 1.0
        self.tracking_guard_stopped = False
        self.tracking_guard_last_error = None
        self.done = False
        self.stopped = False
        self.paused = False
        self.has_started = False
        self.state = "idle"
        self.dwell_start_time = None
        self.last_time = rospy.Time.now()
        self._publish_zero_base()
        return TriggerResponse(success=True, message="reset to idle")

    def _stop_cb(self, _req):
        self.stopped = True
        self.paused = True
        self.state = "stopped"
        self.speed_scale = 0.0
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

            nominal_point = self.path.sample(self.distance_offset)
            self.speed_scale = self._tracking_guard_speed_scale(nominal_point)

            if self.state == "tracking" and not self.paused and not self.stopped and not self.done:
                remaining = max(0.0, self.path.total_length - self.distance_offset)
                v, a, j = self.motion_profile.update(self.speed * self.speed_scale, remaining, dt, self.loop)
                self.last_profile_state = (v, a, j)
                self.distance_offset += v * dt
                if self.distance_offset >= self.path.total_length:
                    if self.loop:
                        self.distance_offset = math.fmod(self.distance_offset, self.path.total_length)
                    else:
                        self.distance_offset = self.path.total_length
                        self.done = True
                        self.state = "done"
                        self.motion_profile.reset()
                        self.last_profile_state = (0.0, 0.0, 0.0)
            elif self.paused or self.stopped or self.done:
                self.motion_profile.reset()
                self.last_profile_state = (0.0, 0.0, 0.0)
                if self.paused or self.stopped:
                    self.speed_scale = 0.0

            active_motion = self.state == "tracking" and not self.paused and not self.stopped and not self.done

            if self.state == "idle":
                point = self.path.sample(0.0)
                self._publish_current_marker(point, now)
                self._publish_zero_base()
            elif self.state == "preposition" and not self.paused and not self.stopped:
                point = self.path.sample(0.0)
                self._publish_pose(point, now, active=False)
                self._publish_current_marker(point, now)
                self._publish_zero_base()
                if self._start_reached():
                    self.state = "dwell"
                    self.dwell_start_time = now
                    rospy.loginfo("Start point reached; waiting %.2f s before path following", self.preposition_dwell_s)
            elif self.state == "dwell" and not self.paused and not self.stopped:
                point = self.path.sample(0.0)
                self._publish_pose(point, now, active=False)
                self._publish_current_marker(point, now)
                self._publish_zero_base()
                if self.dwell_start_time is None:
                    self.dwell_start_time = now
                if (now - self.dwell_start_time).to_sec() >= self.preposition_dwell_s:
                    self.state = "tracking"
                    self.has_started = True
                    self.last_time = now
                    rospy.loginfo("Starting path following")
            elif self.state in ("tracking", "done") and not self.stopped and (self.hold_final_pose or not self.done):
                point = self.path.sample(self.distance_offset)
                self._publish_pose(point, now, active=active_motion)
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
            self.speed_scale_pub.publish(Float64(data=self.speed_scale))
            rate.sleep()

    def _tracking_guard_speed_scale(self, target_point: Point3) -> float:
        if not self.tracking_guard_enabled:
            self.tracking_guard_last_error = None
            self.tracking_guard_stopped = False
            return 1.0
        if self.state != "tracking" or self.paused or self.stopped or self.done:
            return 0.0

        tcp = self._current_tcp_in_path_frame()
        if tcp is None:
            self.tracking_guard_last_error = None
            return 1.0

        error = _distance(tcp, target_point)
        self.tracking_guard_last_error = error
        resume_error = max(self.tracking_guard_slowdown_error,
                           self.tracking_guard_stop_error - self.tracking_guard_resume_hysteresis)

        if self.tracking_guard_stopped:
            if error <= resume_error:
                self.tracking_guard_stopped = False
            else:
                return 0.0

        if error >= self.tracking_guard_stop_error:
            self.tracking_guard_stopped = True
            rospy.logwarn_throttle(
                1.0,
                "Tracking guard stop: TCP error %.3f m >= %.3f m",
                error,
                self.tracking_guard_stop_error,
            )
            return 0.0

        if error <= self.tracking_guard_slowdown_error:
            return 1.0

        span = self.tracking_guard_stop_error - self.tracking_guard_slowdown_error
        return _clamp((self.tracking_guard_stop_error - error) / span, 0.0, 1.0)

    def _publish_pose(self, point: Point3, stamp: rospy.Time, active: bool = True):
        orientation = _quaternion_from_tuple(self.orientation_profile.sample(self.distance_offset))
        v_scalar, a_scalar, j_scalar = self.last_profile_state if active else (0.0, 0.0, 0.0)
        tangent = self.path.tangent(self.distance_offset)
        angular_velocity = self.orientation_profile.angular_velocity(
            self.distance_offset, v_scalar, self.path.total_length) if active else (0.0, 0.0, 0.0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = point[0]
        pose_msg.pose.position.y = point[1]
        pose_msg.pose.position.z = point[2]
        pose_msg.pose.orientation = orientation

        state_msg = CartesianTrajectorySetpoint()
        state_msg.header = pose_msg.header
        state_msg.pose = pose_msg.pose
        state_msg.velocity.linear = Vector3(
            x=tangent[0] * v_scalar,
            y=tangent[1] * v_scalar,
            z=tangent[2] * v_scalar,
        )
        state_msg.velocity.angular = Vector3(
            x=angular_velocity[0],
            y=angular_velocity[1],
            z=angular_velocity[2],
        )
        state_msg.acceleration.linear = Vector3(
            x=tangent[0] * a_scalar,
            y=tangent[1] * a_scalar,
            z=tangent[2] * a_scalar,
        )
        state_msg.jerk.linear = Vector3(
            x=tangent[0] * j_scalar,
            y=tangent[1] * j_scalar,
            z=tangent[2] * j_scalar,
        )
        state_msg.path_s = self.distance_offset
        state_msg.path_progress = min(self.distance_offset / self.path.total_length, 1.0)
        state_msg.active = active
        if self.publish_target_pose:
            self.target_pub.publish(pose_msg)
        self.target_state_pub.publish(state_msg)

    def _ee_state_cb(self, msg: EndEffectorState):
        self.current_tcp = (msg.position.x, msg.position.y, msg.position.z)
        self.current_tcp_frame = msg.header.frame_id

    def _current_tcp_in_path_frame(self) -> Optional[Point3]:
        if self.current_tcp is None:
            return None
        if not self.current_tcp_frame or self.current_tcp_frame == self.frame_id:
            return self.current_tcp
        if self.tf_buffer is None:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.current_tcp_frame,
                rospy.Time(0),
                rospy.Duration(self.base_tf_timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "TCP feedback TF failed (%s -> %s): %s",
                self.current_tcp_frame,
                self.frame_id,
                exc,
            )
            return None
        return _transform_point(transform, self.current_tcp)

    def _start_reached(self) -> bool:
        if not self.require_start_reached:
            return True
        tcp = self._current_tcp_in_path_frame()
        if tcp is None:
            self.last_start_error = None
            return False
        self.last_start_error = _distance(tcp, self.path.sample(0.0))
        return self.last_start_error <= self.start_position_tolerance

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
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.path.marker_points()]
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
