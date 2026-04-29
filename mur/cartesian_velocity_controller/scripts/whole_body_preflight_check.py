#!/usr/bin/env python3
"""Preflight checks for the mur620b whole-body print MVP."""

import sys
from typing import Iterable, List, Sequence, Tuple

import rosgraph
import rospy
import tf2_ros
from sensor_msgs.msg import JointState, LaserScan


def _normalize_topic(topic: str, mur_ns: str) -> str:
    topic = str(topic).strip()
    if not topic:
        return topic
    if topic.startswith("/"):
        return topic
    return f"/{mur_ns}/{topic}"


def _topic_type_exists(topic_types: Sequence[Tuple[str, str]], topic: str) -> bool:
    return any(name == topic for name, _type in topic_types)


def _service_exists(services: Iterable[str], service: str) -> bool:
    return service in set(services)


def _wait_for_msg(topic: str, msg_type, timeout: float) -> Tuple[bool, str]:
    try:
        rospy.wait_for_message(topic, msg_type, timeout=timeout)
        return True, "ok"
    except Exception as exc:
        return False, str(exc)


class PreflightCheck:
    def __init__(self) -> None:
        self.mur_ns = rospy.get_param("~mur_ns", "mur620b").strip("/")
        self.arm = rospy.get_param("~arm", "right")
        self.path_frame = rospy.get_param("~path_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", f"{self.mur_ns}/base_link")
        self.timeout = float(rospy.get_param("~timeout", 2.0))
        self.require_scan = bool(rospy.get_param("~require_scan", False))
        self.require_cmd_vel_subscriber = bool(rospy.get_param("~require_cmd_vel_subscriber", True))

        suffix = "r" if self.arm == "right" else "l"
        controller_ns = f"/{self.mur_ns}/cartesian_velocity_controller_{suffix}"
        self.ee_state_topic = rospy.get_param("~ee_state_topic", f"{controller_ns}/end_effector_state")
        self.validate_service = rospy.get_param("~validation_service", f"{controller_ns}/validate_poses")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", f"/{self.mur_ns}/joint_states")
        self.cmd_vel_topic = _normalize_topic(rospy.get_param("~cmd_vel_topic", "cmd_vel"), self.mur_ns)
        self.speed_scale_topic = rospy.get_param(
            "~speed_scale_topic", f"/{self.mur_ns}/tcp_path_trajectory_manager/speed_scale")

        raw_scan_topics = rospy.get_param("~scan_topics", ["mir/f_scan", "mir/scan"])
        self.scan_topics = [_normalize_topic(t, self.mur_ns) for t in raw_scan_topics]

        self.master = rosgraph.Master(rospy.get_name())
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.failures: List[str] = []
        self.warnings: List[str] = []

    def run(self) -> bool:
        topic_types = self.master.getPublishedTopics("/")
        pubs, subs, services = self.master.getSystemState()
        service_names = [name for name, _providers in services]
        sub_topics = {name for name, _nodes in subs}

        self._check_service(service_names, self.validate_service)
        self._check_topic_message(self.joint_states_topic, JointState, "joint state")
        self._check_topic_message(self.ee_state_topic, self._ee_msg_type(), "end-effector feedback")
        self._check_topic_presence(topic_types, self.speed_scale_topic, required=False, label="tracking guard speed_scale")
        self._check_cmd_vel(sub_topics)
        self._check_scans(topic_types)
        self._check_tf()

        for warning in self.warnings:
            rospy.logwarn("%s", warning)
        for failure in self.failures:
            rospy.logerr("%s", failure)

        if self.failures:
            rospy.logerr("Preflight failed: %d failure(s), %d warning(s)", len(self.failures), len(self.warnings))
            return False

        rospy.loginfo("Preflight OK: %d warning(s)", len(self.warnings))
        return True

    def _ee_msg_type(self):
        from cartesian_velocity_controller.msg import EndEffectorState

        return EndEffectorState

    def _check_service(self, services: Sequence[str], service: str) -> None:
        if _service_exists(services, service):
            rospy.loginfo("service OK: %s", service)
        else:
            self.failures.append(f"missing service: {service}")

    def _check_topic_presence(self, topic_types: Sequence[Tuple[str, str]], topic: str, required: bool, label: str) -> None:
        if _topic_type_exists(topic_types, topic):
            rospy.loginfo("topic OK: %s (%s)", topic, label)
            return
        message = f"missing topic: {topic} ({label})"
        if required:
            self.failures.append(message)
        else:
            self.warnings.append(message)

    def _check_topic_message(self, topic: str, msg_type, label: str) -> None:
        ok, detail = _wait_for_msg(topic, msg_type, self.timeout)
        if ok:
            rospy.loginfo("message OK: %s (%s)", topic, label)
        else:
            self.failures.append(f"no message on {topic} ({label}) within {self.timeout:.1f}s: {detail}")

    def _check_cmd_vel(self, sub_topics: Sequence[str]) -> None:
        if self.cmd_vel_topic in sub_topics:
            rospy.loginfo("cmd_vel subscriber OK: %s", self.cmd_vel_topic)
            return
        message = f"no subscriber currently registered on {self.cmd_vel_topic}"
        if self.require_cmd_vel_subscriber:
            self.failures.append(message)
        else:
            self.warnings.append(message)

    def _check_scans(self, topic_types: Sequence[Tuple[str, str]]) -> None:
        available = [topic for topic in self.scan_topics if _topic_type_exists(topic_types, topic)]
        if available:
            rospy.loginfo("scan topic OK: %s", ", ".join(available))
            ok, detail = _wait_for_msg(available[0], LaserScan, self.timeout)
            if not ok:
                message = f"scan topic exists but no message on {available[0]} within {self.timeout:.1f}s: {detail}"
                if self.require_scan:
                    self.failures.append(message)
                else:
                    self.warnings.append(message)
            return

        message = f"no configured scan topic is published: {', '.join(self.scan_topics)}"
        if self.require_scan:
            self.failures.append(message)
        else:
            self.warnings.append(message)

    def _check_tf(self) -> None:
        try:
            self.tf_buffer.lookup_transform(self.path_frame, self.base_frame, rospy.Time(0), rospy.Duration(self.timeout))
            rospy.loginfo("TF OK: %s -> %s", self.path_frame, self.base_frame)
        except Exception as exc:
            self.failures.append(f"missing TF {self.path_frame} -> {self.base_frame}: {exc}")


def main() -> int:
    rospy.init_node("whole_body_preflight_check")
    ok = PreflightCheck().run()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
