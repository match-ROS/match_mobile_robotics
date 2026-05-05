#!/usr/bin/env python3
"""Terminal dashboard for whole-body print demo debugging."""

import math
import sys
import threading
from typing import Iterable, List, Optional, Sequence

import rospy
from cartesian_velocity_controller.msg import EndEffectorState
from cartesian_velocity_controller.msg import JointVelocityFeedback
from cartesian_velocity_controller.msg import PipelineDebug
from cartesian_velocity_controller.msg import WholeBodyPrintDebug
from visualization_msgs.msg import Marker


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def vector_norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def fmt_float(value: Optional[float], width: int = 7, precision: int = 3, suffix: str = "") -> str:
    if value is None:
        return " " * max(0, width - 2) + "--"
    try:
        if math.isnan(value) or math.isinf(value):
            return " " * max(0, width - 2) + "--"
    except TypeError:
        return " " * max(0, width - 2) + "--"
    return f"{value:{width}.{precision}f}{suffix}"


def fmt_bool(value: Optional[bool]) -> str:
    if value is None:
        return "--"
    return "yes" if value else "no"


def fmt_vec3(values: Sequence[Optional[float]], precision: int = 3) -> str:
    return "[" + ", ".join(fmt_float(v, width=7, precision=precision).strip() for v in values) + "]"


def as_bool(value) -> bool:
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
    raise ValueError(f"Cannot parse boolean from {value!r}")


def age_string(stamp: Optional[rospy.Time], now: rospy.Time) -> str:
    if stamp is None or stamp.is_zero():
        return "--"
    age = max(0.0, (now - stamp).to_sec())
    return f"{age:4.2f}s"


class TopicCache:
    def __init__(self, label: str) -> None:
        self.label = label
        self.msg = None
        self.stamp = None

    def update(self, msg) -> None:
        self.msg = msg
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None) if header is not None else None
        if stamp is not None and not stamp.is_zero():
            self.stamp = stamp
        else:
            self.stamp = rospy.Time.now()


class WholeBodyPrintMonitor:
    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.mur_ns = rospy.get_param("~mur_ns", "mur620b").strip("/")
        self.arm = rospy.get_param("~arm", "right").strip().lower()
        self.refresh_hz = max(0.5, float(rospy.get_param("~refresh_hz", 5.0)))
        self.stale_timeout = max(0.1, float(rospy.get_param("~stale_timeout", 1.0)))
        self.max_joint_rows = max(1, int(rospy.get_param("~max_joint_rows", 6)))
        self.clear_screen = as_bool(rospy.get_param("~clear_screen", True))

        if self.arm not in ("left", "right"):
            rospy.logwarn("Invalid arm '%s', falling back to 'right'", self.arm)
            self.arm = "right"
        self.arm_suffix = "l" if self.arm == "left" else "r"

        root = f"/{self.mur_ns}"
        self.topics = {
            "whole_body": rospy.get_param("~whole_body_debug_topic", f"{root}/whole_body_print_controller/debug"),
            "pipeline": rospy.get_param(
                "~pipeline_debug_topic",
                f"{root}/cartesian_velocity_controller_{self.arm_suffix}/pipeline_debug",
            ),
            "joint_feedback": rospy.get_param(
                "~joint_feedback_topic",
                f"{root}/cartesian_velocity_controller_{self.arm_suffix}/joint_velocity_feedback",
            ),
            "end_effector": rospy.get_param(
                "~end_effector_state_topic",
                f"{root}/cartesian_velocity_controller_{self.arm_suffix}/end_effector_state",
            ),
            "path_marker": rospy.get_param(
                "~path_marker_topic",
                f"{root}/whole_body_print_controller/path_marker",
            ),
            "current_marker": rospy.get_param(
                "~current_marker_topic",
                f"{root}/whole_body_print_controller/current_marker",
            ),
        }

        self.whole_body = TopicCache("whole_body")
        self.pipeline = TopicCache("pipeline")
        self.joint_feedback = TopicCache("joint_feedback")
        self.end_effector = TopicCache("end_effector")
        self.path_marker = TopicCache("path_marker")
        self.current_marker = TopicCache("current_marker")

        rospy.Subscriber(self.topics["whole_body"], WholeBodyPrintDebug, self._whole_body_cb, queue_size=20)
        rospy.Subscriber(self.topics["pipeline"], PipelineDebug, self._pipeline_cb, queue_size=20)
        rospy.Subscriber(self.topics["joint_feedback"], JointVelocityFeedback, self._joint_feedback_cb, queue_size=20)
        rospy.Subscriber(self.topics["end_effector"], EndEffectorState, self._end_effector_cb, queue_size=20)
        rospy.Subscriber(self.topics["path_marker"], Marker, self._path_marker_cb, queue_size=5)
        rospy.Subscriber(self.topics["current_marker"], Marker, self._current_marker_cb, queue_size=20)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.refresh_hz), self._render_timer_cb)

    def _whole_body_cb(self, msg: WholeBodyPrintDebug) -> None:
        with self.lock:
            self.whole_body.update(msg)

    def _pipeline_cb(self, msg: PipelineDebug) -> None:
        with self.lock:
            self.pipeline.update(msg)

    def _joint_feedback_cb(self, msg: JointVelocityFeedback) -> None:
        with self.lock:
            self.joint_feedback.update(msg)

    def _end_effector_cb(self, msg: EndEffectorState) -> None:
        with self.lock:
            self.end_effector.update(msg)

    def _path_marker_cb(self, msg: Marker) -> None:
        with self.lock:
            self.path_marker.update(msg)

    def _current_marker_cb(self, msg: Marker) -> None:
        with self.lock:
            self.current_marker.update(msg)

    def _render_timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            whole_body = self.whole_body.msg
            pipeline = self.pipeline.msg
            joint_feedback = self.joint_feedback.msg
            end_effector = self.end_effector.msg
            path_marker = self.path_marker.msg
            current_marker = self.current_marker.msg
            whole_body_stamp = self.whole_body.stamp
            pipeline_stamp = self.pipeline.stamp
            joint_feedback_stamp = self.joint_feedback.stamp
            end_effector_stamp = self.end_effector.stamp
            path_marker_stamp = self.path_marker.stamp
            current_marker_stamp = self.current_marker.stamp

        lines = self._build_lines(
            now,
            whole_body,
            whole_body_stamp,
            pipeline,
            pipeline_stamp,
            joint_feedback,
            joint_feedback_stamp,
            end_effector,
            end_effector_stamp,
            path_marker,
            path_marker_stamp,
            current_marker,
            current_marker_stamp,
        )
        self._draw(lines)

    def _topic_status(self, label: str, stamp: Optional[rospy.Time], now: rospy.Time) -> str:
        if stamp is None:
            return f"{label}: missing"
        age = max(0.0, (now - stamp).to_sec())
        state = "STALE" if age > self.stale_timeout else "ok"
        return f"{label}: {state} age={age:4.2f}s"

    def _build_lines(
        self,
        now: rospy.Time,
        whole_body: Optional[WholeBodyPrintDebug],
        whole_body_stamp: Optional[rospy.Time],
        pipeline: Optional[PipelineDebug],
        pipeline_stamp: Optional[rospy.Time],
        joint_feedback: Optional[JointVelocityFeedback],
        joint_feedback_stamp: Optional[rospy.Time],
        end_effector: Optional[EndEffectorState],
        end_effector_stamp: Optional[rospy.Time],
        path_marker: Optional[Marker],
        path_marker_stamp: Optional[rospy.Time],
        current_marker: Optional[Marker],
        current_marker_stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = []
        lines.append(
            f"Whole-Body Print Monitor  mur_ns=/{self.mur_ns}  arm={self.arm}  refresh={self.refresh_hz:.1f} Hz  "
            f"time={now.to_sec():.3f}"
        )
        lines.append(
            " | ".join(
                [
                    self._topic_status("whole_body", whole_body_stamp, now),
                    self._topic_status("pipeline", pipeline_stamp, now),
                    self._topic_status("joint_fb", joint_feedback_stamp, now),
                    self._topic_status("ee", end_effector_stamp, now),
                ]
            )
        )
        lines.append("")
        lines.extend(self._whole_body_lines(now, whole_body, whole_body_stamp))
        lines.append("")
        lines.extend(self._pipeline_lines(now, pipeline, pipeline_stamp))
        lines.append("")
        lines.extend(self._joint_feedback_lines(now, joint_feedback, joint_feedback_stamp))
        lines.append("")
        lines.extend(self._end_effector_lines(now, end_effector, end_effector_stamp))
        lines.append("")
        lines.extend(self._marker_lines(now, path_marker, path_marker_stamp, current_marker, current_marker_stamp))
        lines.append("")
        lines.append("Topics:")
        for key in ("whole_body", "pipeline", "joint_feedback", "end_effector", "path_marker", "current_marker"):
            lines.append(f"  {key:13s} {self.topics[key]}")
        return lines

    def _marker_lines(
        self,
        now: rospy.Time,
        path_marker: Optional[Marker],
        path_stamp: Optional[rospy.Time],
        current_marker: Optional[Marker],
        current_stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = ["[RViz markers]"]
        if path_marker is None:
            lines.append("  path_marker    no data")
        else:
            lines.append(
                "  "
                f"path_marker    frame={path_marker.header.frame_id or '-'}  age={age_string(path_stamp, now)}  "
                f"points={len(path_marker.points)}"
            )
        if current_marker is None:
            lines.append("  current_marker no data")
        else:
            p = current_marker.pose.position
            lines.append(
                "  "
                f"current_marker frame={current_marker.header.frame_id or '-'}  age={age_string(current_stamp, now)}  "
                f"pos={fmt_vec3([p.x, p.y, p.z])}"
            )
        return lines

    def _whole_body_lines(
        self,
        now: rospy.Time,
        msg: Optional[WholeBodyPrintDebug],
        stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = ["[Whole-body]"]
        if msg is None:
            lines.append("  no data")
            return lines

        tcp_error_norm = vector_norm3(msg.tcp_error.x, msg.tcp_error.y, msg.tcp_error.z)
        base_target_dx = msg.target_in_base.x - msg.preferred_tcp_x
        base_target_dy = msg.target_in_base.y - msg.preferred_tcp_y
        progress = 100.0 * clamp(msg.path_progress, 0.0, 1.0)

        lines.append(
            "  "
            f"state={msg.state:<12s} age={age_string(stamp, now)} "
            f"path={fmt_float(msg.path_s, 7, 3)}/{fmt_float(msg.path_length, 7, 3)} m "
            f"progress={progress:6.2f}%"
        )
        lines.append(
            "  "
            f"path_frame={msg.path_frame}  base_frame={msg.base_frame}  arm_scale={msg.arm_tracking_scale:5.2f}"
        )
        lines.append(f"  target_world    {fmt_vec3([msg.target_position.x, msg.target_position.y, msg.target_position.z])}")
        lines.append(
            f"  arm_target      {fmt_vec3([msg.arm_target_position.x, msg.arm_target_position.y, msg.arm_target_position.z])}"
        )
        lines.append(
            f"  tcp_current     {fmt_vec3([msg.current_tcp_position.x, msg.current_tcp_position.y, msg.current_tcp_position.z])}"
        )
        lines.append(f"  tcp_error       {fmt_vec3([msg.tcp_error.x, msg.tcp_error.y, msg.tcp_error.z])}  |e|={tcp_error_norm:6.3f}")
        lines.append(
            "  "
            f"target_in_base   {fmt_vec3([msg.target_in_base.x, msg.target_in_base.y, msg.target_in_base.z])}  "
            f"preferred=[{msg.preferred_tcp_x:6.3f}, {msg.preferred_tcp_y:6.3f}]  "
            f"delta=[{base_target_dx:6.3f}, {base_target_dy:6.3f}]"
        )
        lines.append(
            "  "
            f"base_cmd         lin.x={msg.base_command.linear.x:7.3f}  ang.z={msg.base_command.angular.z:7.3f}   "
            f"nominal lin.x={msg.base_nominal_command.linear.x:7.3f}  ang.z={msg.base_nominal_command.angular.z:7.3f}"
        )
        lines.append(
            "  "
            f"base_flags       enabled={fmt_bool(msg.base_enabled):>3s}  tf_ok={fmt_bool(msg.base_tf_ok):>3s}  "
            f"in_zone={fmt_bool(msg.base_in_tracking_zone):>3s}  "
            f"sat_lin={fmt_bool(msg.base_linear_saturated):>3s}  sat_ang={fmt_bool(msg.base_angular_saturated):>3s}"
        )
        lines.append(
            "  "
            f"avoidance        enabled={fmt_bool(msg.avoidance_enabled):>3s}  active={fmt_bool(msg.avoidance_active):>3s}  "
            f"min_dist={fmt_float(msg.avoidance_min_distance, 7, 3)}  "
            f"omega={fmt_float(msg.avoidance_omega, 7, 3)}  speed_scale={fmt_float(msg.avoidance_speed_scale, 6, 2)}"
        )
        lines.append(
            "  "
            f"lifter           enabled={fmt_bool(msg.lifter_enabled):>3s}  have_state={fmt_bool(msg.lifter_have_state):>3s}  "
            f"pos={fmt_float(msg.lifter_position, 7, 3)}  tgt={fmt_float(msg.lifter_target, 7, 3)}  "
            f"vel_cmd={fmt_float(msg.lifter_velocity_command, 7, 3)}"
        )
        return lines

    def _pipeline_lines(
        self,
        now: rospy.Time,
        msg: Optional[PipelineDebug],
        stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = ["[Arm pipeline]"]
        if msg is None:
            lines.append("  no data")
            return lines

        current = msg.current_pose.position
        waypoint = msg.active_waypoint.position
        target_raw = msg.target_raw.position
        target_filtered = msg.target_filtered.position
        cmd_lin = msg.cartesian_cmd_linear
        cmd_ang = msg.cartesian_cmd_angular
        pid_pos = msg.pid_position_error
        pid_ori = msg.pid_orientation_error
        desired_lin = msg.v_desired_linear
        filtered_lin = msg.v_filtered_linear

        lines.append(
            "  "
            f"age={age_string(stamp, now)}  waypoint={msg.active_waypoint_index + 1}/{msg.total_waypoints}  "
            f"dist_wp={msg.distance_waypoint_to_current:6.3f}  "
            f"dist_raw={msg.distance_target_raw_to_current:6.3f}  "
            f"dist_filt={msg.distance_target_filtered_to_current:6.3f}"
        )
        lines.append(f"  current_pose    {fmt_vec3([current.x, current.y, current.z])}")
        lines.append(f"  active_waypoint {fmt_vec3([waypoint.x, waypoint.y, waypoint.z])}")
        lines.append(f"  target_raw      {fmt_vec3([target_raw.x, target_raw.y, target_raw.z])}")
        lines.append(f"  target_filtered {fmt_vec3([target_filtered.x, target_filtered.y, target_filtered.z])}")
        lines.append(
            "  "
            f"v_desired_lin    {fmt_vec3([desired_lin.x, desired_lin.y, desired_lin.z])}  "
            f"v_filtered_lin   {fmt_vec3([filtered_lin.x, filtered_lin.y, filtered_lin.z])}"
        )
        lines.append(
            "  "
            f"cart_cmd_lin     {fmt_vec3([cmd_lin.x, cmd_lin.y, cmd_lin.z])}  "
            f"cart_cmd_ang     {fmt_vec3([cmd_ang.x, cmd_ang.y, cmd_ang.z])}"
        )
        lines.append(
            "  "
            f"pid_err_pos      {fmt_vec3([pid_pos.x, pid_pos.y, pid_pos.z])}  "
            f"|e|={msg.pid_position_error_norm:6.3f}"
        )
        lines.append(
            "  "
            f"pid_err_ori      {fmt_vec3([pid_ori.x, pid_ori.y, pid_ori.z])}  "
            f"|e|={msg.pid_orientation_error_norm:6.3f}"
        )
        lines.append(
            "  "
            f"jacobian         sigma_min={msg.jacobian_min_singular_value:8.5f}  "
            f"damping={msg.jacobian_damping_factor:8.5f}  "
            f"safety_scale={msg.safety_scaling_factor:6.3f}"
        )
        lines.append(
            "  "
            f"safety_reason    {msg.safety_limiting_reason or '-'}"
        )
        lines.append(
            "  "
            f"closest_obs      id={msg.closest_obstacle_id or '-'}  link={msg.closest_link_name or '-'}  "
            f"dist={fmt_float(msg.closest_obstacle_distance, 7, 3)}  active_poi={msg.active_poi_count}"
        )
        if msg.joint_velocity_after_limiter:
            preview = list(msg.joint_velocity_after_limiter[: min(6, len(msg.joint_velocity_after_limiter))])
            lines.append(f"  qdot_after_lim  {fmt_vec3(preview, precision=3)}{' ...' if len(msg.joint_velocity_after_limiter) > 6 else ''}")
        return lines

    def _joint_feedback_lines(
        self,
        now: rospy.Time,
        msg: Optional[JointVelocityFeedback],
        stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = ["[Joint feedback]"]
        if msg is None:
            lines.append("  no data")
            return lines

        lines.append(f"  age={age_string(stamp, now)}")
        lines.append("  joint                      cmd        act        err        pos")

        count = min(
            len(msg.joint_names),
            len(msg.commanded_velocity),
            len(msg.actual_velocity),
            len(msg.current_position),
        )
        if count == 0:
            lines.append("  no joint rows")
            return lines

        for idx in range(min(count, self.max_joint_rows)):
            cmd = msg.commanded_velocity[idx]
            act = msg.actual_velocity[idx]
            pos = msg.current_position[idx]
            err = cmd - act
            name = msg.joint_names[idx]
            lines.append(f"  {name:22.22s} {cmd:9.4f} {act:9.4f} {err:9.4f} {pos:10.4f}")

        if count > self.max_joint_rows:
            lines.append(f"  ... {count - self.max_joint_rows} more joints")
        return lines

    def _end_effector_lines(
        self,
        now: rospy.Time,
        msg: Optional[EndEffectorState],
        stamp: Optional[rospy.Time],
    ) -> List[str]:
        lines = ["[End effector]"]
        if msg is None:
            lines.append("  no data")
            return lines

        pos = msg.position
        lin = msg.linear_velocity
        acc = msg.linear_acceleration
        rpy = msg.orientation_rpy
        ang = msg.angular_velocity
        lines.append(f"  frame={msg.header.frame_id or '-'}  age={age_string(stamp, now)}")
        lines.append(f"  position       {fmt_vec3([pos.x, pos.y, pos.z])}")
        lines.append(
            "  "
            f"linear vel      {fmt_vec3([lin.x, lin.y, lin.z])}  |v|={vector_norm3(lin.x, lin.y, lin.z):6.3f}"
        )
        lines.append(
            "  "
            f"linear acc      {fmt_vec3([acc.x, acc.y, acc.z])}  |a|={vector_norm3(acc.x, acc.y, acc.z):6.3f}"
        )
        lines.append(f"  orientation    rpy={fmt_vec3([rpy.x, rpy.y, rpy.z])}")
        lines.append(
            "  "
            f"angular vel     {fmt_vec3([ang.x, ang.y, ang.z])}  |w|={vector_norm3(ang.x, ang.y, ang.z):6.3f}"
        )
        return lines

    def _draw(self, lines: Iterable[str]) -> None:
        output = "\n".join(lines) + "\n"
        if self.clear_screen and sys.stdout.isatty():
            sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(output)
        sys.stdout.flush()


def main() -> None:
    rospy.init_node("whole_body_print_monitor")
    WholeBodyPrintMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
