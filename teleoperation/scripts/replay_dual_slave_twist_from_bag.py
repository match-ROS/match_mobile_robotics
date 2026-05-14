#!/usr/bin/env python3
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rosbag
import rospy
import tf2_ros
import yaml

from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _get_str(name: str, default: str) -> str:
    value = rospy.get_param(name, default)
    return str(value) if value is not None else str(default)


def _get_bool(name: str, default: bool) -> bool:
    try:
        return bool(rospy.get_param(name, default))
    except Exception:
        return bool(default)


def _get_int(name: str, default: int) -> int:
    try:
        return int(rospy.get_param(name, default))
    except Exception:
        return int(default)


def _get_float(name: str, default: float) -> float:
    try:
        return float(rospy.get_param(name, default))
    except Exception:
        return float(default)


def _get_optional_float(name: str) -> Optional[float]:
    if not rospy.has_param(name):
        return None
    value = rospy.get_param(name)
    if value is None:
        return None
    text = str(value).strip()
    if not text or text.lower() in ("manifest", "none"):
        return None
    return float(text)


def _resolve_ns(ns: str) -> str:
    ns = str(ns).strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _norm3(v) -> float:
    return math.sqrt(float(v.x) * float(v.x) + float(v.y) * float(v.y) + float(v.z) * float(v.z))


def _quat_angle_xyzw(q1: List[float], q2: List[float]) -> float:
    dot = abs(sum(float(a) * float(b) for a, b in zip(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _zero_twist() -> Twist:
    return Twist()


def _twist_values(msg) -> Tuple[float, float, float, float, float, float]:
    src = msg.twist if hasattr(msg, "twist") else msg
    return (
        float(src.linear.x),
        float(src.linear.y),
        float(src.linear.z),
        float(src.angular.x),
        float(src.angular.y),
        float(src.angular.z),
    )


def _twist_from_values(values: Tuple[float, float, float, float, float, float]) -> Twist:
    out = Twist()
    out.linear.x = values[0]
    out.linear.y = values[1]
    out.linear.z = values[2]
    out.angular.x = values[3]
    out.angular.y = values[4]
    out.angular.z = values[5]
    return out


@dataclass
class TwistSample:
    rel_t: float
    values: Tuple[float, float, float, float, float, float]


@dataclass
class ScaledReplay:
    output_dt_s: float
    duration_s: float
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]]

    @property
    def tick_count(self) -> int:
        if not self.commands_by_side:
            return 0
        return len(next(iter(self.commands_by_side.values())))


def _resample_time_scaled_twists(
    samples: List[TwistSample],
    source_duration_s: float,
    speed_scale: float,
    output_rate_hz: float,
) -> Tuple[float, List[Tuple[float, float, float, float, float, float]]]:
    if source_duration_s <= 0.0:
        raise ReplayAbort("Invalid source duration for replay")
    if speed_scale <= 0.0:
        raise ReplayAbort("speed_scale must be > 0")
    if output_rate_hz <= 0.0:
        raise ReplayAbort("replay_rate_hz must be > 0")
    if not samples:
        raise ReplayAbort("No twist samples available for resampling")

    eps = 1e-12
    output_dt_s = 1.0 / output_rate_hz
    output_duration_s = source_duration_s / speed_scale
    tick_count = max(1, int(math.ceil(output_duration_s / output_dt_s - eps)))

    clean: List[TwistSample] = []
    for sample in sorted(samples, key=lambda item: item.rel_t):
        rel_t = min(max(float(sample.rel_t), 0.0), source_duration_s)
        item = TwistSample(rel_t=rel_t, values=sample.values)
        if clean and abs(rel_t - clean[-1].rel_t) <= eps:
            clean[-1] = item
        else:
            clean.append(item)

    last_sample_end_s = min(source_duration_s, clean[-1].rel_t + output_dt_s)
    values: List[Tuple[float, float, float, float, float, float]] = []
    source_index = 0
    for tick in range(tick_count):
        source_a = speed_scale * tick * output_dt_s
        source_b = min(source_duration_s, speed_scale * (tick + 1) * output_dt_s)
        acc = [0.0] * 6

        while source_index < len(clean):
            next_t = clean[source_index + 1].rel_t if source_index + 1 < len(clean) else last_sample_end_s
            if next_t > source_a + eps:
                break
            source_index += 1

        idx = source_index
        while idx < len(clean):
            start_t = clean[idx].rel_t
            end_t = clean[idx + 1].rel_t if idx + 1 < len(clean) else last_sample_end_s
            if start_t >= source_b - eps:
                break
            overlap = min(source_b, end_t) - max(source_a, start_t)
            if overlap > eps:
                for component in range(6):
                    acc[component] += overlap * clean[idx].values[component]
            idx += 1

        values.append(tuple(component / output_dt_s for component in acc))

    return output_dt_s, values


@dataclass
class ControllerManagerServices:
    list_controllers: str
    load_controller: str
    switch_controller: str


class ControllerManager:
    def __init__(
        self,
        controller_manager_ns: str,
        *,
        wait_services_timeout_s: float,
        auto_load: bool,
        strictness: int,
        start_asap: bool,
        switch_timeout_s: float,
        verify_timeout_s: float,
        verify_period_s: float,
    ):
        cm = _resolve_ns(controller_manager_ns)
        self.services = ControllerManagerServices(
            list_controllers=f"{cm}/list_controllers",
            load_controller=f"{cm}/load_controller",
            switch_controller=f"{cm}/switch_controller",
        )
        self.wait_services_timeout_s = wait_services_timeout_s
        self.auto_load = auto_load
        self.strictness = strictness
        self.start_asap = start_asap
        self.switch_timeout_s = switch_timeout_s
        self.verify_timeout_s = verify_timeout_s
        self.verify_period_s = verify_period_s
        self.list_proxy = None
        self.load_proxy = None
        self.switch_proxy = None

    def connect(self) -> None:
        self._wait_service(self.services.list_controllers)
        self._wait_service(self.services.switch_controller)
        if self.auto_load:
            self._wait_service(self.services.load_controller)
        self.list_proxy = rospy.ServiceProxy(self.services.list_controllers, ListControllers)
        self.switch_proxy = rospy.ServiceProxy(self.services.switch_controller, SwitchController)
        self.load_proxy = (
            rospy.ServiceProxy(self.services.load_controller, LoadController)
            if self.auto_load
            else None
        )

    def _wait_service(self, name: str) -> None:
        resolved = rospy.resolve_name(name)
        timeout = float(self.wait_services_timeout_s)
        rospy.loginfo("Waiting for service '%s' (timeout=%.3fs)", resolved, timeout)
        if timeout <= 0.0:
            rospy.wait_for_service(name)
        else:
            rospy.wait_for_service(name, timeout=timeout)

    def list_states(self) -> Optional[Dict[str, str]]:
        try:
            resp = self.list_proxy(ListControllersRequest())
        except rospy.ServiceException as exc:
            rospy.logwarn("ListControllers failed on %s: %s", self.services.list_controllers, exc)
            return None
        return {controller.name: controller.state for controller in resp.controller}

    def ensure_loaded(self, controllers: List[str]) -> None:
        if not self.auto_load or self.load_proxy is None:
            return
        states = self.list_states()
        for name in controllers:
            if states is not None and name in states:
                continue
            try:
                resp = self.load_proxy(LoadControllerRequest(name=name))
                if not resp.ok:
                    rospy.logwarn("LoadController('%s') returned ok=false", name)
            except rospy.ServiceException as exc:
                rospy.logwarn("LoadController('%s') failed: %s", name, exc)

    def ensure_states(self, must_run: List[str], must_stop: List[str]) -> bool:
        states = self.list_states()
        if states is None:
            return self._switch(list(must_run), list(must_stop))

        start = [name for name in must_run if states.get(name) != "running"]
        stop = [name for name in must_stop if states.get(name) == "running"]
        if start or stop:
            ok = self._switch(start, stop)
            if not ok and int(self.strictness) == 2:
                rospy.logwarn("STRICT controller switch failed; retrying BEST_EFFORT.")
                old = self.strictness
                self.strictness = 1
                ok = self._switch(start, stop)
                self.strictness = old
            if not ok:
                return False

        if self.verify_timeout_s <= 0.0:
            return True

        start_time = rospy.Time.now()
        last_states = None
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() >= self.verify_timeout_s:
                break
            last_states = self.list_states()
            if last_states is None:
                rospy.sleep(self.verify_period_s)
                continue
            running_ok = all(last_states.get(name) == "running" for name in must_run)
            stopped_ok = all(last_states.get(name) != "running" for name in must_stop)
            if running_ok and stopped_ok:
                return True
            rospy.sleep(self.verify_period_s)

        rospy.logerr(
            "Controller verification failed. must_run=%s must_stop=%s last_states=%s",
            must_run,
            must_stop,
            last_states,
        )
        return False

    def _switch(self, start: List[str], stop: List[str]) -> bool:
        req = SwitchControllerRequest()
        req.start_controllers = list(start)
        req.stop_controllers = list(stop)
        req.strictness = int(self.strictness)
        req.start_asap = bool(self.start_asap)
        req.timeout = float(self.switch_timeout_s)
        try:
            resp = self.switch_proxy(req)
            return bool(resp.ok)
        except rospy.ServiceException as exc:
            rospy.logerr("SwitchController start=%s stop=%s failed: %s", start, stop, exc)
            return False


class ReplayAbort(RuntimeError):
    pass


class DualSlaveTwistReplay:
    def __init__(self):
        self.status_pub = rospy.Publisher(
            _get_str("~status_topic", "/teleop/dual_slave_replay/status"),
            String,
            queue_size=1,
            latch=True,
        )
        self.manifest_path = os.path.abspath(_get_str("~manifest", ""))
        if not self.manifest_path:
            raise ReplayAbort("Missing required param '~manifest'")
        with open(self.manifest_path, "r") as f:
            self.manifest = yaml.safe_load(f)
        if not isinstance(self.manifest, dict):
            raise ReplayAbort(f"Invalid manifest: {self.manifest_path}")

        bag_default = str(self.manifest.get("bag", ""))
        bag_param = _get_str("~bag", "").strip()
        self.bag_path = os.path.abspath(bag_param or bag_default)
        if not os.path.isfile(self.bag_path):
            raise ReplayAbort(f"Bag not found: {self.bag_path}")

        self.robot_ns = _get_str("~robot_ns", "mur620d")
        self.dry_run = _get_bool("~dry_run", False)
        self.home_only = _get_bool("~home_only", False)
        self.execute_home = _get_bool("~execute_home", True)
        self.start_delay_s = _get_float("~start_delay_s", 2.0)
        self.speed_scale = _get_float("~speed_scale", 1.0)
        self.time_scale = _get_float("~time_scale", 1.0)
        self.replay_rate_hz = _get_float("~replay_rate_hz", 500.0)
        self.publish_zero_rate_hz = _get_float("~publish_zero_rate_hz", 20.0)
        self.zero_before_s = _get_float("~zero_before_s", 0.5)
        self.zero_after_s = _get_float("~zero_after_s", 0.5)
        if self.speed_scale <= 0.0:
            raise ReplayAbort("speed_scale must be > 0")
        if self.replay_rate_hz <= 0.0:
            raise ReplayAbort("replay_rate_hz must be > 0")
        if abs(self.time_scale - 1.0) > 1e-9:
            raise ReplayAbort(
                "time_scale is deprecated for trajectory replay. Use speed_scale as the "
                "area-preserving trajectory speed factor: 2.0 halves duration, 0.5 doubles it."
            )
        self.planning_time = _get_float("~planning_time", 5.0)
        self.num_planning_attempts = _get_int("~num_planning_attempts", 5)
        self.max_velocity_scaling_factor = _get_float("~max_velocity_scaling_factor", 0.1)
        self.max_acceleration_scaling_factor = _get_float("~max_acceleration_scaling_factor", 0.1)
        self.goal_joint_tolerance = _get_float("~goal_joint_tolerance", 0.001)
        self.tcp_position_tolerance_m = _get_float("~tcp_position_tolerance_m", 0.03)
        self.tcp_orientation_tolerance_rad = _get_float("~tcp_orientation_tolerance_rad", 0.15)
        self.tcp_verify_timeout_s = _get_float("~tcp_verify_timeout_s", 5.0)
        self.force_abort_threshold_n = _get_float("~force_abort_threshold_n", 150.0)
        self.torque_abort_threshold_nm = _get_float("~torque_abort_threshold_nm", 40.0)
        self.wrench_timeout_s = _get_float("~wrench_timeout_s", 0.5)
        self.zero_ft_delay_s = _get_float("~ur10e_zero_loadcell_delay_s", 0.5)
        self.zero_ft_wait_timeout_s = _get_float("~ur10e_zero_loadcell_wait_timeout_s", 10.0)
        self.moveit_ns = _get_str("~moveit_ns", f"/{self.robot_ns}")
        self.robot_description_param = _get_str(
            "~robot_description_param", f"/{self.robot_ns}/robot_description"
        )

        self.twist_controller = _get_str("~twist_controller", "twist_controller")
        self.arm_controller = _get_str("~arm_controller", "arm_controller")
        self.force_torque_controller = _get_str(
            "~force_torque_controller", "force_torque_sensor_controller"
        )
        self.controller_auto_load = _get_bool("~auto_load_controllers", True)
        self.controller_strictness = _get_int("~strictness", 2)
        self.controller_start_asap = _get_bool("~start_asap", False)
        self.controller_switch_timeout_s = _get_float("~switch_timeout_s", 0.0)
        self.controller_wait_services_timeout_s = _get_float("~wait_services_timeout_s", 15.0)
        self.controller_verify_timeout_s = _get_float("~controller_state_verify_timeout_s", 5.0)
        self.controller_verify_period_s = _get_float("~controller_state_verify_period_s", 0.1)

        self.window = self.manifest.get("window", {})
        self.replay_start_s = self._window_time("start_offset_s", "auto_start_time", "replay_start_time")
        self.replay_end_s = self._window_time("end_offset_s", "auto_end_time", "replay_end_time")
        if self.replay_end_s <= self.replay_start_s:
            raise ReplayAbort("Invalid replay window: end is not after start")

        self.arms = self._load_arms()
        self.pubs = {
            side: rospy.Publisher(arm["publish_command_topic"], Twist, queue_size=1)
            for side, arm in self.arms.items()
        }
        self.last_wrench: Dict[str, Tuple[rospy.Time, float, float]] = {}
        self.wrench_subs = [
            rospy.Subscriber(arm["wrench_topic"], WrenchStamped, self._wrench_cb, callback_args=side)
            for side, arm in self.arms.items()
        ]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _publish_status(self, text: str) -> None:
        rospy.loginfo("%s", text)
        self.status_pub.publish(String(data=text))

    def _window_time(self, offset_param: str, auto_key: str, replay_key: str) -> float:
        override_offset = _get_optional_float("~" + offset_param)
        if override_offset is None and replay_key in self.window:
            return float(self.window[replay_key])
        offset = override_offset
        if offset is None:
            offset = float(self.window.get(offset_param, 0.0))
        auto_time = self.window.get(auto_key)
        if auto_time is not None:
            return float(auto_time) + offset
        return float(self.window[replay_key])

    def _load_arms(self) -> Dict[str, Dict]:
        manifest_arms = self.manifest.get("arms", {})
        out = {}
        for side, default_arm_ns in [("left", "UR10_l"), ("right", "UR10_r")]:
            arm = dict(manifest_arms.get(side, {}))
            if not arm:
                raise ReplayAbort(f"Manifest missing arm entry: {side}")
            arm_ns = str(arm.get("arm_ns", default_arm_ns))
            move_group_param = f"~move_group_name_{'l' if side == 'left' else 'r'}"
            arm["move_group_name"] = _get_str(move_group_param, arm.get("move_group_name", ""))
            if not arm["move_group_name"]:
                raise ReplayAbort(f"Missing MoveIt group for {side}")
            arm["controller_manager_ns"] = _get_str(
                f"~{side}_controller_manager_ns",
                f"/{self.robot_ns}/{arm_ns}/controller_manager",
            )
            arm["publish_command_topic"] = _get_str(
                f"~{side}_command_topic",
                arm.get("command_topic", f"/{self.robot_ns}/{arm_ns}/twist_controller/command_collision_free"),
            )
            arm["wrench_topic"] = _get_str(
                f"~{side}_wrench_topic",
                f"/{self.robot_ns}/{arm_ns}/wrench",
            )
            arm["zero_ftsensor_enabled"] = _get_bool(
                f"~do_ur10e_zero_loadcell_slave_{side}",
                True,
            )
            arm["zero_ftsensor_service"] = _get_str(
                f"~{side}_zero_ftsensor_service",
                f"/{self.robot_ns}/{arm_ns}/ur_hardware_interface/zero_ftsensor",
            )
            out[side] = arm
        return out

    def _wrench_cb(self, msg: WrenchStamped, side: str) -> None:
        self.last_wrench[side] = (
            rospy.Time.now(),
            _norm3(msg.wrench.force),
            _norm3(msg.wrench.torque),
        )

    def _check_safety(self) -> None:
        now = rospy.Time.now()
        for side, sample in self.last_wrench.items():
            stamp, force_norm, torque_norm = sample
            if self.wrench_timeout_s > 0.0 and (now - stamp).to_sec() > self.wrench_timeout_s:
                continue
            if self.force_abort_threshold_n > 0.0 and force_norm >= self.force_abort_threshold_n:
                raise ReplayAbort(
                    f"{side} force abort: {force_norm:.3f} N >= {self.force_abort_threshold_n:.3f} N"
                )
            if self.torque_abort_threshold_nm > 0.0 and torque_norm >= self.torque_abort_threshold_nm:
                raise ReplayAbort(
                    f"{side} torque abort: {torque_norm:.3f} Nm >= {self.torque_abort_threshold_nm:.3f} Nm"
                )

    def _publish_zero_for(self, duration_s: float) -> None:
        if self.dry_run:
            return
        rate_hz = max(1.0, self.publish_zero_rate_hz)
        rate = rospy.Rate(rate_hz)
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, duration_s))
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            for pub in self.pubs.values():
                pub.publish(_zero_twist())
            rate.sleep()
        for pub in self.pubs.values():
            pub.publish(_zero_twist())

    def _controller_manager(self, arm: Dict) -> ControllerManager:
        cm = ControllerManager(
            arm["controller_manager_ns"],
            wait_services_timeout_s=self.controller_wait_services_timeout_s,
            auto_load=self.controller_auto_load,
            strictness=self.controller_strictness,
            start_asap=self.controller_start_asap,
            switch_timeout_s=self.controller_switch_timeout_s,
            verify_timeout_s=self.controller_verify_timeout_s,
            verify_period_s=self.controller_verify_period_s,
        )
        cm.connect()
        cm.ensure_loaded([self.arm_controller, self.twist_controller, self.force_torque_controller])
        return cm

    def _ensure_home_controllers(self, controllers: Dict[str, ControllerManager]) -> None:
        for side, cm in controllers.items():
            self._publish_status(f"{side}: enabling MoveIt controller state")
            ok = cm.ensure_states(
                must_run=[self.arm_controller, self.force_torque_controller],
                must_stop=[self.twist_controller],
            )
            if not ok:
                raise ReplayAbort(f"{side}: cannot enable MoveIt controller state")

    def _ensure_twist_controllers(self, controllers: Dict[str, ControllerManager]) -> None:
        for side, cm in controllers.items():
            self._publish_status(f"{side}: enabling twist controller state")
            ok = cm.ensure_states(
                must_run=[self.twist_controller, self.force_torque_controller],
                must_stop=[self.arm_controller],
            )
            if not ok:
                raise ReplayAbort(f"{side}: cannot enable twist controller state")

    def _joint_target_for_group(self, group, arm: Dict) -> Dict[str, float]:
        joint_state = arm.get("joint_state", {})
        names = [str(name) for name in joint_state.get("name", [])]
        positions = [float(value) for value in joint_state.get("position", [])]
        if len(names) != len(positions):
            raise ReplayAbort("Manifest joint_state name/position length mismatch")
        exact = {name: positions[i] for i, name in enumerate(names)}
        by_base = {name.split("/")[-1]: positions[i] for i, name in enumerate(names)}
        target = {}
        missing = []
        for joint in group.get_active_joints():
            if joint in exact:
                target[joint] = exact[joint]
                continue
            base = joint.split("/")[-1]
            if base in by_base:
                target[joint] = by_base[base]
                continue
            missing.append(joint)
        if missing:
            raise ReplayAbort(f"Missing joint values for MoveIt joints: {missing}")
        return target

    def _moveit_to_initial_joint_state(self) -> None:
        if self.dry_run:
            return
        try:
            import moveit_commander
        except Exception as exc:
            raise ReplayAbort(f"Cannot import moveit_commander: {exc}")

        moveit_commander.roscpp_initialize(sys.argv)
        for side, arm in self.arms.items():
            self._publish_status(f"{side}: planning to recorded initial joint state")
            group = moveit_commander.MoveGroupCommander(
                arm["move_group_name"],
                robot_description=str(self.robot_description_param),
                ns=str(_resolve_ns(self.moveit_ns)),
            )
            group.set_planning_time(float(self.planning_time))
            group.set_num_planning_attempts(int(self.num_planning_attempts))
            group.set_max_velocity_scaling_factor(float(self.max_velocity_scaling_factor))
            group.set_max_acceleration_scaling_factor(float(self.max_acceleration_scaling_factor))
            group.set_goal_joint_tolerance(float(self.goal_joint_tolerance))
            group.set_start_state_to_current_state()
            target = self._joint_target_for_group(group, arm)
            group.set_joint_value_target(target)
            try:
                plan_res = group.plan()
            except Exception as exc:
                raise ReplayAbort(f"{side}: planning failed: {exc}")
            success = True
            plan = plan_res
            if isinstance(plan_res, tuple):
                success = bool(plan_res[0])
                plan = plan_res[1] if len(plan_res) >= 2 else None
            if not success or plan is None:
                raise ReplayAbort(f"{side}: MoveIt did not find a plan")
            points = getattr(getattr(plan, "joint_trajectory", None), "points", [])
            if not points:
                raise ReplayAbort(f"{side}: MoveIt plan has no trajectory points")
            if self.execute_home:
                self._publish_status(f"{side}: executing initial joint state")
                ok = bool(group.execute(plan, wait=True))
                group.stop()
                group.clear_pose_targets()
                if not ok:
                    raise ReplayAbort(f"{side}: MoveIt execution failed")
            else:
                self._publish_status(f"{side}: plan completed, execute_home=false")

    def _verify_tcp_pose(self) -> None:
        if self.dry_run:
            return
        for side, arm in self.arms.items():
            expected = arm.get("tcp_pose_base", {})
            base_frame = str(arm.get("base_frame", expected.get("frame_id", "")))
            tcp_frame = str(arm.get("tcp_frame", expected.get("child_frame_id", "")))
            if not base_frame or not tcp_frame:
                raise ReplayAbort(f"{side}: missing base/tcp frame in manifest")
            expected_pos = [float(x) for x in expected.get("position", [])]
            expected_q = [float(x) for x in expected.get("orientation_xyzw", [])]
            if len(expected_pos) != 3 or len(expected_q) != 4:
                raise ReplayAbort(f"{side}: missing tcp_pose_base in manifest")

            deadline = rospy.Time.now() + rospy.Duration(self.tcp_verify_timeout_s)
            transform = None
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        base_frame,
                        tcp_frame,
                        rospy.Time(0),
                        rospy.Duration(0.2),
                    )
                    break
                except Exception:
                    rospy.sleep(0.05)
            if transform is None:
                raise ReplayAbort(f"{side}: TF lookup failed for {base_frame} -> {tcp_frame}")

            actual_pos = [
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ]
            actual_q = [
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ]
            pos_err = math.sqrt(sum((a - b) * (a - b) for a, b in zip(actual_pos, expected_pos)))
            ori_err = _quat_angle_xyzw(actual_q, expected_q)
            rospy.loginfo("%s: TCP verification pos_err=%.4f m ori_err=%.4f rad", side, pos_err, ori_err)
            if pos_err > self.tcp_position_tolerance_m:
                raise ReplayAbort(
                    f"{side}: TCP position error {pos_err:.4f} m > {self.tcp_position_tolerance_m:.4f} m"
                )
            if ori_err > self.tcp_orientation_tolerance_rad:
                raise ReplayAbort(
                    f"{side}: TCP orientation error {ori_err:.4f} rad > {self.tcp_orientation_tolerance_rad:.4f} rad"
                )

    def _zero_ft_sensors(self) -> None:
        if self.dry_run:
            return

        for side, arm in self.arms.items():
            if not arm.get("zero_ftsensor_enabled", True):
                rospy.loginfo("%s: FT sensor zero disabled", side)
                continue

            service_name = str(arm["zero_ftsensor_service"])
            resolved_service = rospy.resolve_name(service_name)
            self._publish_status(f"{side}: zeroing FT sensor {resolved_service}")

            try:
                if self.zero_ft_wait_timeout_s <= 0.0:
                    rospy.wait_for_service(service_name)
                else:
                    rospy.wait_for_service(service_name, timeout=self.zero_ft_wait_timeout_s)
            except rospy.ROSException as exc:
                raise ReplayAbort(f"{side}: FT zero service not available ({resolved_service}): {exc}")

            if self.zero_ft_delay_s > 0.0:
                rospy.sleep(self.zero_ft_delay_s)

            try:
                response = rospy.ServiceProxy(service_name, Trigger)()
            except rospy.ServiceException as exc:
                raise ReplayAbort(f"{side}: FT zero service call failed ({resolved_service}): {exc}")

            if not response.success:
                raise ReplayAbort(
                    f"{side}: FT zero request returned success=false ({resolved_service}): {response.message}"
                )
            rospy.loginfo("%s: FT sensor zeroed successfully: %s", side, response.message)

    def _load_samples(self) -> Dict[str, List[TwistSample]]:
        source_topics = {
            self.arms["left"]["command_topic"]: "left",
            self.arms["right"]["command_topic"]: "right",
        }
        samples: Dict[str, List[TwistSample]] = {side: [] for side in self.arms}
        with rosbag.Bag(self.bag_path, "r") as bag:
            for topic, msg, stamp in bag.read_messages(
                topics=list(source_topics.keys()),
                start_time=rospy.Time.from_sec(self.replay_start_s),
                end_time=rospy.Time.from_sec(self.replay_end_s),
            ):
                side = source_topics[topic]
                rel_t = stamp.to_sec() - self.replay_start_s
                samples[side].append(TwistSample(rel_t=rel_t, values=_twist_values(msg)))

        missing = [side for side, side_samples in samples.items() if not side_samples]
        if missing:
            raise ReplayAbort(f"No twist messages found in replay window for arm(s): {missing}")
        for side_samples in samples.values():
            side_samples.sort(key=lambda item: item.rel_t)
        return samples

    def _build_scaled_replay(self, samples: Dict[str, List[TwistSample]]) -> ScaledReplay:
        source_duration_s = self.replay_end_s - self.replay_start_s
        output_dt_s = 1.0 / self.replay_rate_hz
        commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]] = {}

        for side, side_samples in samples.items():
            side_dt_s, commands = _resample_time_scaled_twists(
                side_samples,
                source_duration_s,
                self.speed_scale,
                self.replay_rate_hz,
            )
            output_dt_s = side_dt_s
            commands_by_side[side] = commands

        counts = {side: len(commands) for side, commands in commands_by_side.items()}
        if len(set(counts.values())) != 1:
            raise ReplayAbort(f"Internal replay resampling mismatch: {counts}")

        tick_count = next(iter(counts.values()))
        return ScaledReplay(
            output_dt_s=output_dt_s,
            duration_s=tick_count * output_dt_s,
            commands_by_side=commands_by_side,
        )

    def _run_replay(self, replay: ScaledReplay) -> None:
        self._publish_status("publishing zero before replay")
        self._publish_zero_for(self.zero_before_s)

        if self.start_delay_s > 0.0:
            self._publish_status(f"waiting start delay {self.start_delay_s:.3f}s")
            end_delay = rospy.Time.now() + rospy.Duration(self.start_delay_s)
            rate = rospy.Rate(100.0)
            while not rospy.is_shutdown() and rospy.Time.now() < end_delay:
                self._check_safety()
                rate.sleep()

        self._publish_status("replaying")
        wall_start = rospy.Time.now()
        for tick in range(replay.tick_count):
            self._check_safety()
            target_time = wall_start + rospy.Duration(tick * replay.output_dt_s)
            while not rospy.is_shutdown() and rospy.Time.now() < target_time:
                self._check_safety()
                rospy.sleep(min(0.002, max(0.0, (target_time - rospy.Time.now()).to_sec())))
            if rospy.is_shutdown():
                raise ReplayAbort("ROS shutdown")
            for side, commands in replay.commands_by_side.items():
                self.pubs[side].publish(_twist_from_values(commands[tick]))

        replay_end = wall_start + rospy.Duration(replay.duration_s)
        while not rospy.is_shutdown() and rospy.Time.now() < replay_end:
            self._check_safety()
            rospy.sleep(min(0.002, max(0.0, (replay_end - rospy.Time.now()).to_sec())))

        self._publish_status("publishing zero after replay")
        self._publish_zero_for(self.zero_after_s)
        self._publish_status("complete")

    def run(self) -> None:
        samples = self._load_samples()
        replay = self._build_scaled_replay(samples)
        self._publish_status(
            "loaded %s twist samples, window %.6f -> %.6f, source duration %.3fs, "
            "speed_scale %.3f, replay duration %.3fs, rate %.1f Hz, ticks %d"
            % (
                {side: len(side_samples) for side, side_samples in samples.items()},
                self.replay_start_s,
                self.replay_end_s,
                self.replay_end_s - self.replay_start_s,
                self.speed_scale,
                replay.duration_s,
                self.replay_rate_hz,
                replay.tick_count,
            )
        )
        if self.dry_run:
            self._publish_status("dry_run_complete")
            return

        controllers = {
            side: self._controller_manager(arm)
            for side, arm in self.arms.items()
        }
        try:
            self._publish_zero_for(self.zero_before_s)
            self._ensure_home_controllers(controllers)
            self._moveit_to_initial_joint_state()
            self._ensure_twist_controllers(controllers)
            self._verify_tcp_pose()
            self._zero_ft_sensors()
            if self.home_only:
                self._publish_status("home_only_complete")
                self._publish_zero_for(self.zero_after_s)
                return
            self._run_replay(replay)
        except Exception:
            self._publish_zero_for(self.zero_after_s)
            raise


def main() -> int:
    rospy.init_node("replay_dual_slave_twist_from_bag", anonymous=False)
    try:
        replay = DualSlaveTwistReplay()
        replay.run()
        return 0
    except ReplayAbort as exc:
        try:
            replay._publish_status(f"aborted: {exc}")
        except Exception:
            pass
        rospy.logerr("%s", exc)
        return 2
    except Exception as exc:
        try:
            replay._publish_status(f"failed: {exc}")
        except Exception:
            pass
        rospy.logerr("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
