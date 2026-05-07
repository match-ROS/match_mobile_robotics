#!/usr/bin/env python3
import argparse
import math
import os
import sys
import warnings
from datetime import datetime
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rosbag
import rospy
import tf2_ros
import yaml

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState


DEFAULT_TOPICS = {
    "master_wrench_left": "/teleop_debug/mur620b_UR10_l/master_wrench_filtered",
    "master_wrench_right": "/teleop_debug/mur620b_UR10_r/master_wrench_filtered",
    "slave_command_left": "/mur620d/UR10_l/twist_controller/command_collision_free",
    "slave_command_right": "/mur620d/UR10_r/twist_controller/command_collision_free",
    "slave_joint_left": "/mur620d/UR10_l/joint_states",
    "slave_joint_right": "/mur620d/UR10_r/joint_states",
    "slave_global_tcp_left": "/mur620d/UR10_l/global_tcp_pose",
    "slave_global_tcp_right": "/mur620d/UR10_r/global_tcp_pose",
    "tf": "/tf",
    "tf_static": "/tf_static",
}

DEFAULT_FRAMES = {
    "left": {
        "arm_ns": "UR10_l",
        "move_group_name": "UR_arm_l",
        "base_frame": "mur620d/UR10_l/base_link_inertia",
        "tcp_frame": "mur620d/UR10_l/tool0",
    },
    "right": {
        "arm_ns": "UR10_r",
        "move_group_name": "UR_arm_r",
        "base_frame": "mur620d/UR10_r/base_link_inertia",
        "tcp_frame": "mur620d/UR10_r/tool0",
    },
}


def _stamp_to_sec(stamp) -> float:
    return float(stamp.to_sec())


def _init_rospy_time_for_offline_tf() -> None:
    # tf2_ros.Buffer with a timeout may query rospy's clock even when we only
    # analyze a bag offline. Initialize just the time subsystem, without
    # registering a ROS node or requiring a running roscore.
    try:
        rospy.rostime.set_rostime_initialized(True)
    except Exception:
        pass


def _vec3_norm(v) -> float:
    return math.sqrt(float(v.x) * float(v.x) + float(v.y) * float(v.y) + float(v.z) * float(v.z))


def _twist_parts(msg) -> Tuple[float, float]:
    twist = msg.twist if isinstance(msg, TwistStamped) else msg
    return _vec3_norm(twist.linear), _vec3_norm(twist.angular)


def _wrench_parts(msg: WrenchStamped) -> Tuple[float, float]:
    return _vec3_norm(msg.wrench.force), _vec3_norm(msg.wrench.torque)


def _is_wrench_active(
    msg: WrenchStamped,
    force_threshold_n: float,
    torque_threshold_nm: float,
) -> bool:
    force_norm, torque_norm = _wrench_parts(msg)
    return force_norm > force_threshold_n or torque_norm > torque_threshold_nm


def _is_twist_active(
    msg,
    linear_threshold_mps: float,
    angular_threshold_radps: float,
) -> bool:
    linear_norm, angular_norm = _twist_parts(msg)
    return linear_norm > linear_threshold_mps or angular_norm > angular_threshold_radps


def _pose_to_dict(pose_msg: PoseStamped, nearest_delta_s: float) -> Dict:
    q = pose_msg.pose.orientation
    p = pose_msg.pose.position
    return {
        "stamp": _stamp_to_sec(pose_msg.header.stamp),
        "nearest_delta_s": float(nearest_delta_s),
        "frame_id": pose_msg.header.frame_id,
        "position": [float(p.x), float(p.y), float(p.z)],
        "orientation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


def _transform_to_pose_dict(
    transform: TransformStamped,
    requested_time_s: float,
    lookup_time_s: float,
) -> Dict:
    tr = transform.transform.translation
    q = transform.transform.rotation
    return {
        "stamp": float(lookup_time_s),
        "requested_stamp": float(requested_time_s),
        "nearest_delta_s": float(lookup_time_s - requested_time_s),
        "frame_id": transform.header.frame_id,
        "child_frame_id": transform.child_frame_id,
        "position": [float(tr.x), float(tr.y), float(tr.z)],
        "orientation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


def _joint_state_to_dict(msg: JointState, nearest_delta_s: float) -> Dict:
    return {
        "stamp": _stamp_to_sec(msg.header.stamp),
        "nearest_delta_s": float(nearest_delta_s),
        "name": [str(x) for x in msg.name],
        "position": [float(x) for x in msg.position],
        "velocity": [float(x) for x in msg.velocity],
        "effort": [float(x) for x in msg.effort],
    }


def _nearest(samples: Sequence[Tuple[float, object]], target_time_s: float) -> Tuple[float, object]:
    if not samples:
        raise RuntimeError("No samples available")
    return min(samples, key=lambda item: abs(item[0] - target_time_s))


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _default_output_dir(bag_path: str) -> str:
    bag_dir = os.path.dirname(os.path.abspath(bag_path)) or "."
    base = os.path.basename(bag_path)
    if base.endswith(".bag"):
        base = base[:-4]
    return os.path.join(bag_dir, base + "_replay_analysis")


def _collect_activity(
    bag_path: str,
    topics: Dict[str, str],
    args,
) -> Tuple[Dict[str, List[Tuple[float, float, float]]], List[float]]:
    series: Dict[str, List[Tuple[float, float, float]]] = {
        "master_wrench_left": [],
        "master_wrench_right": [],
        "slave_command_left": [],
        "slave_command_right": [],
    }
    active_times: List[float] = []
    activity_topics = [
        topics["master_wrench_left"],
        topics["master_wrench_right"],
        topics["slave_command_left"],
        topics["slave_command_right"],
    ]
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=activity_topics):
            ts = _stamp_to_sec(t)
            if topic == topics["master_wrench_left"]:
                f, tau = _wrench_parts(msg)
                series["master_wrench_left"].append((ts, f, tau))
                if _is_wrench_active(msg, args.master_force_threshold_n, args.master_torque_threshold_nm):
                    active_times.append(ts)
            elif topic == topics["master_wrench_right"]:
                f, tau = _wrench_parts(msg)
                series["master_wrench_right"].append((ts, f, tau))
                if _is_wrench_active(msg, args.master_force_threshold_n, args.master_torque_threshold_nm):
                    active_times.append(ts)
            elif topic == topics["slave_command_left"]:
                lin, ang = _twist_parts(msg)
                series["slave_command_left"].append((ts, lin, ang))
                if _is_twist_active(msg, args.twist_linear_threshold_mps, args.twist_angular_threshold_radps):
                    active_times.append(ts)
            elif topic == topics["slave_command_right"]:
                lin, ang = _twist_parts(msg)
                series["slave_command_right"].append((ts, lin, ang))
                if _is_twist_active(msg, args.twist_linear_threshold_mps, args.twist_angular_threshold_radps):
                    active_times.append(ts)
    return series, active_times


def _collect_start_state(
    bag_path: str,
    topics: Dict[str, str],
    sample_time_s: float,
    bag_start_s: float,
    bag_end_s: float,
) -> Tuple[Dict[str, Tuple[float, JointState]], Dict[str, Tuple[float, PoseStamped]], tf2_ros.Buffer]:
    joint_samples = {"left": [], "right": []}
    global_pose_samples = {"left": [], "right": []}
    cache_time_s = max(10.0, bag_end_s - bag_start_s + 10.0)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time_s))
    read_topics = [
        topics["slave_joint_left"],
        topics["slave_joint_right"],
        topics["slave_global_tcp_left"],
        topics["slave_global_tcp_right"],
        topics["tf"],
        topics["tf_static"],
    ]
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=read_topics):
            ts = _stamp_to_sec(t)
            if topic == topics["slave_joint_left"]:
                joint_samples["left"].append((ts, msg))
            elif topic == topics["slave_joint_right"]:
                joint_samples["right"].append((ts, msg))
            elif topic == topics["slave_global_tcp_left"]:
                global_pose_samples["left"].append((ts, msg))
            elif topic == topics["slave_global_tcp_right"]:
                global_pose_samples["right"].append((ts, msg))
            elif topic == topics["tf_static"]:
                for transform in msg.transforms:
                    try:
                        with warnings.catch_warnings():
                            warnings.filterwarnings("ignore", message="translation should be of type Vector3")
                            warnings.filterwarnings("ignore", message="rotation should be of type Quaternion")
                            tf_buffer.set_transform_static(transform, "analyze_dual_slave_replay_window")
                    except Exception:
                        pass
            elif topic == topics["tf"]:
                for transform in msg.transforms:
                    try:
                        with warnings.catch_warnings():
                            warnings.filterwarnings("ignore", message="translation should be of type Vector3")
                            warnings.filterwarnings("ignore", message="rotation should be of type Quaternion")
                            tf_buffer.set_transform(transform, "analyze_dual_slave_replay_window")
                    except Exception:
                        pass

    nearest_joints = {
        side: _nearest(samples, sample_time_s)
        for side, samples in joint_samples.items()
    }
    nearest_global = {
        side: _nearest(samples, sample_time_s)
        for side, samples in global_pose_samples.items()
    }
    return nearest_joints, nearest_global, tf_buffer


def _lookup_base_tcp(
    tf_buffer: tf2_ros.Buffer,
    base_frame: str,
    tcp_frame: str,
    sample_time_s: float,
    search_radius_s: float = 0.5,
    search_step_s: float = 0.01,
) -> Tuple[TransformStamped, float]:
    offsets = [0.0]
    steps = int(math.ceil(max(0.0, search_radius_s) / max(1e-6, search_step_s)))
    for i in range(1, steps + 1):
        delta = i * search_step_s
        offsets.extend([delta, -delta])

    last_error: Optional[Exception] = None
    for offset in offsets:
        lookup_time_s = sample_time_s + offset
        if lookup_time_s < 0.0:
            continue
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                tcp_frame,
                rospy.Time.from_sec(lookup_time_s),
                rospy.Duration(0.0),
            )
            return transform, lookup_time_s
        except Exception as exc:
            last_error = exc

    raise RuntimeError(
        "Cannot lookup transform %s -> %s near %.6f within +/- %.3fs: %s"
        % (base_frame, tcp_frame, sample_time_s, search_radius_s, last_error)
    )


def _make_plot(
    plot_path: str,
    series: Dict[str, List[Tuple[float, float, float]]],
    auto_start_s: float,
    auto_end_s: float,
    replay_start_s: float,
    replay_end_s: float,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def rel_x(samples: Iterable[Tuple[float, float, float]]) -> List[float]:
        return [s[0] - replay_start_s for s in samples]

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    axes[0].set_title("Master filtered wrench norm")
    for key, label in [
        ("master_wrench_left", "left force"),
        ("master_wrench_right", "right force"),
    ]:
        samples = series[key]
        axes[0].plot(rel_x(samples), [s[1] for s in samples], label=label)
    for key, label in [
        ("master_wrench_left", "left torque"),
        ("master_wrench_right", "right torque"),
    ]:
        samples = series[key]
        axes[0].plot(rel_x(samples), [s[2] for s in samples], linestyle="--", label=label)
    axes[0].set_ylabel("N / Nm")
    axes[0].legend(loc="upper right")

    axes[1].set_title("Slave command linear norm")
    for key, label in [
        ("slave_command_left", "left linear"),
        ("slave_command_right", "right linear"),
    ]:
        samples = series[key]
        axes[1].plot(rel_x(samples), [s[1] for s in samples], label=label)
    axes[1].set_ylabel("m/s")
    axes[1].legend(loc="upper right")

    axes[2].set_title("Slave command angular norm")
    for key, label in [
        ("slave_command_left", "left angular"),
        ("slave_command_right", "right angular"),
    ]:
        samples = series[key]
        axes[2].plot(rel_x(samples), [s[2] for s in samples], label=label)
    axes[2].set_ylabel("rad/s")
    axes[2].set_xlabel("time from replay start [s]")
    axes[2].legend(loc="upper right")

    markers = [
        (auto_start_s - replay_start_s, "auto start", "tab:green"),
        (auto_end_s - replay_start_s, "auto end", "tab:red"),
        (0.0, "replay start", "black"),
        (replay_end_s - replay_start_s, "replay end", "black"),
    ]
    for axis in axes:
        for x, label, color in markers:
            axis.axvline(x, color=color, linestyle=":", linewidth=1.2)
            ymax = axis.get_ylim()[1]
            axis.text(x, ymax, label, rotation=90, va="top", ha="right", color=color, fontsize=8)
        axis.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def _topic_arg(parser: argparse.ArgumentParser, name: str, default_key: str) -> None:
    parser.add_argument(
        "--" + name.replace("_", "-"),
        default=DEFAULT_TOPICS[default_key],
        help=f"default: {DEFAULT_TOPICS[default_key]}",
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze a dual real teleoperation bag and create a replay manifest."
    )
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--manifest", default=None)
    parser.add_argument("--plot", default=None)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--end-offset-s", type=float, default=0.0)
    parser.add_argument("--home-sample-offset-s", type=float, default=-0.2)
    parser.add_argument("--master-force-threshold-n", type=float, default=1e-6)
    parser.add_argument("--master-torque-threshold-nm", type=float, default=1e-6)
    parser.add_argument("--twist-linear-threshold-mps", type=float, default=1e-5)
    parser.add_argument("--twist-angular-threshold-radps", type=float, default=1e-5)
    parser.add_argument("--robot-ns", default="mur620d")
    _topic_arg(parser, "master_wrench_left", "master_wrench_left")
    _topic_arg(parser, "master_wrench_right", "master_wrench_right")
    _topic_arg(parser, "slave_command_left", "slave_command_left")
    _topic_arg(parser, "slave_command_right", "slave_command_right")
    _topic_arg(parser, "slave_joint_left", "slave_joint_left")
    _topic_arg(parser, "slave_joint_right", "slave_joint_right")
    _topic_arg(parser, "slave_global_tcp_left", "slave_global_tcp_left")
    _topic_arg(parser, "slave_global_tcp_right", "slave_global_tcp_right")
    return parser


def main() -> int:
    _init_rospy_time_for_offline_tf()

    parser = _build_parser()
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        parser.error(f"Bag not found: {bag_path}")

    output_dir = os.path.abspath(args.output_dir or _default_output_dir(bag_path))
    os.makedirs(output_dir, exist_ok=True)
    manifest_path = os.path.abspath(args.manifest or os.path.join(output_dir, "replay_manifest.yaml"))
    plot_path = os.path.abspath(args.plot or os.path.join(output_dir, "replay_window.png"))

    topics = {
        "master_wrench_left": args.master_wrench_left,
        "master_wrench_right": args.master_wrench_right,
        "slave_command_left": args.slave_command_left,
        "slave_command_right": args.slave_command_right,
        "slave_joint_left": args.slave_joint_left,
        "slave_joint_right": args.slave_joint_right,
        "slave_global_tcp_left": args.slave_global_tcp_left,
        "slave_global_tcp_right": args.slave_global_tcp_right,
        "tf": DEFAULT_TOPICS["tf"],
        "tf_static": DEFAULT_TOPICS["tf_static"],
    }

    with rosbag.Bag(bag_path, "r") as bag:
        bag_start_s = float(bag.get_start_time())
        bag_end_s = float(bag.get_end_time())

    series, active_times = _collect_activity(bag_path, topics, args)
    if not active_times:
        raise RuntimeError(
            "No active sample found. Lower the wrench/twist thresholds or check the input topics."
        )

    auto_start_s = min(active_times)
    auto_end_s = max(active_times)
    replay_start_s = _clamp(auto_start_s + args.start_offset_s, bag_start_s, bag_end_s)
    replay_end_s = _clamp(auto_end_s + args.end_offset_s, replay_start_s, bag_end_s)
    if replay_end_s <= replay_start_s:
        raise RuntimeError("Invalid replay window: end is not after start")

    sample_time_s = _clamp(replay_start_s + args.home_sample_offset_s, bag_start_s, replay_start_s)
    nearest_joints, nearest_global, tf_buffer = _collect_start_state(
        bag_path,
        topics,
        sample_time_s,
        bag_start_s,
        bag_end_s,
    )

    arms: Dict[str, Dict] = {}
    for side in ["left", "right"]:
        frames = DEFAULT_FRAMES[side].copy()
        frames["base_frame"] = frames["base_frame"].replace("mur620d", args.robot_ns, 1)
        frames["tcp_frame"] = frames["tcp_frame"].replace("mur620d", args.robot_ns, 1)
        tf_msg, tf_lookup_time_s = _lookup_base_tcp(
            tf_buffer,
            frames["base_frame"],
            frames["tcp_frame"],
            sample_time_s,
        )

        joint_t, joint_msg = nearest_joints[side]
        global_t, global_msg = nearest_global[side]
        arms[side] = {
            "arm_ns": frames["arm_ns"],
            "move_group_name": frames["move_group_name"],
            "base_frame": frames["base_frame"],
            "tcp_frame": frames["tcp_frame"],
            "command_topic": topics[f"slave_command_{side}"],
            "joint_state_topic": topics[f"slave_joint_{side}"],
            "global_tcp_pose_topic": topics[f"slave_global_tcp_{side}"],
            "sample_time": float(sample_time_s),
            "joint_state": _joint_state_to_dict(joint_msg, joint_t - sample_time_s),
            "tcp_pose_base": _transform_to_pose_dict(tf_msg, sample_time_s, tf_lookup_time_s),
            "global_tcp_pose": _pose_to_dict(global_msg, global_t - sample_time_s),
        }

    manifest = {
        "version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "bag": bag_path,
        "output_dir": output_dir,
        "plot": plot_path,
        "topics": topics,
        "thresholds": {
            "master_force_threshold_n": float(args.master_force_threshold_n),
            "master_torque_threshold_nm": float(args.master_torque_threshold_nm),
            "twist_linear_threshold_mps": float(args.twist_linear_threshold_mps),
            "twist_angular_threshold_radps": float(args.twist_angular_threshold_radps),
        },
        "window": {
            "bag_start_time": float(bag_start_s),
            "bag_end_time": float(bag_end_s),
            "auto_start_time": float(auto_start_s),
            "auto_end_time": float(auto_end_s),
            "start_offset_s": float(args.start_offset_s),
            "end_offset_s": float(args.end_offset_s),
            "home_sample_offset_s": float(args.home_sample_offset_s),
            "home_sample_time": float(sample_time_s),
            "replay_start_time": float(replay_start_s),
            "replay_end_time": float(replay_end_s),
            "duration_s": float(replay_end_s - replay_start_s),
        },
        "arms": arms,
    }

    with open(manifest_path, "w") as f:
        yaml.safe_dump(manifest, f, default_flow_style=False, sort_keys=False)

    try:
        _make_plot(plot_path, series, auto_start_s, auto_end_s, replay_start_s, replay_end_s)
    except Exception as exc:
        print(f"WARNING: manifest written, but plot generation failed: {exc}", file=sys.stderr)

    print(f"Manifest: {manifest_path}")
    print(f"Plot: {plot_path}")
    print(f"Replay window: {replay_start_s:.6f} -> {replay_end_s:.6f} ({replay_end_s - replay_start_s:.3f} s)")
    print(f"Home sample: {sample_time_s:.6f}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
