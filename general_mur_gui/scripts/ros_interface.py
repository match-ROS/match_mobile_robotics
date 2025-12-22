import os
import subprocess
import yaml
import threading
import shlex
from typing import Optional, Iterable
import signal
import rospy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import QTimer
import rospy
import tf.transformations as tf_trans
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float32, Int32, Int16, Bool
from sensor_msgs.msg import BatteryState
from rosgraph_msgs.msg import Log
import rosnode
import time
import rospkg

from dynamixel_workbench_msgs.msg import DynamixelStateList

import rospy
from geometry_msgs.msg import PoseStamped


DEBUG_CONFIG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config", "rosconsole_debug.config")
)

GUI_CACHE_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config", "gui_persistence.yaml")
)

DEFAULT_SPRAY_DISTANCE = 0.52
DEFAULT_PATH_INDEX = 0
DEFAULT_COMPONENT_NAME = "rectangleRoundedCorners"
COMPONENT_TRANSFORM_KEYS = ("tx", "ty", "tz", "rx", "ry", "rz")
ROSCORE_HOST = "roscore"


def _deploy_gui_cache_to_robot(robot: str, workspace: Optional[str]):
    """Copy the GUI cache (driver nodes, spray distance, …) onto the robot's workspace."""
    robot = (robot or "").strip()
    workspace = (workspace or "").strip()
    if not (robot and workspace):
        return
    if not os.path.exists(GUI_CACHE_PATH):
        return

    remote_path = f"{robot}:~/{workspace}/src/match_additive_manufacturing/print_gui/config/gui_persistence.yaml"
    try:
        subprocess.check_call([
            "scp",
            GUI_CACHE_PATH,
            remote_path,
        ])
    except subprocess.CalledProcessError as exc:
        print(f"Failed to copy GUI cache to {robot}: {exc}")


def _is_debug_enabled(gui) -> bool:
    return bool(getattr(gui, "is_debug_enabled", lambda: False)())


def _build_debug_env(gui, base_env=None):
    env = os.environ.copy() if base_env is None else base_env.copy()
    if _is_debug_enabled(gui):
        env["ROSCONSOLE_CONFIG_FILE"] = DEBUG_CONFIG_PATH
    else:
        env.pop("ROSCONSOLE_CONFIG_FILE", None)
    return env


def _remote_debug_prefix(gui, workspace: Optional[str] = None) -> str:
    workspace = (workspace or "").strip()
    if not (_is_debug_enabled(gui) and workspace):
        return ""
    remote_config = f"~/{workspace}/src/match_additive_manufacturing/print_gui/config/rosconsole_debug.config"
    return f"ROSCONSOLE_CONFIG_FILE={remote_config}; "


def _popen_with_debug(command, gui, shell=False, **kwargs):
    env = _build_debug_env(gui, kwargs.pop("env", None))
    return subprocess.Popen(command, shell=shell, env=env, **kwargs)


def _load_gui_cache_payload() -> dict:
    try:
        with open(GUI_CACHE_PATH, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except FileNotFoundError:
        return {}
    except (yaml.YAMLError, OSError) as exc:
        print(f"Failed to read GUI cache: {exc}")
        return {}
    return data if isinstance(data, dict) else {}


def _save_gui_cache_payload(payload: dict):
    os.makedirs(os.path.dirname(GUI_CACHE_PATH), exist_ok=True)
    with open(GUI_CACHE_PATH, "w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, default_flow_style=False, sort_keys=True)


def _load_servo_targets_cache(default_left: float = 0.0, default_right: float = 0.0):
    data = _load_gui_cache_payload()
    entry = data.get("servo_targets") if isinstance(data, dict) else None
    left = default_left
    right = default_right
    if isinstance(entry, dict):
        l_val = entry.get("left")
        r_val = entry.get("right")
        if isinstance(l_val, (int, float)):
            left = float(l_val)
        if isinstance(r_val, (int, float)):
            right = float(r_val)
    return (left, right)


def _save_servo_targets_cache(left: float, right: float):
    payload = _load_gui_cache_payload()
    payload["servo_targets"] = {
        "left": float(left),
        "right": float(right),
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    _save_gui_cache_payload(payload)


def _run_remote_commands(
    gui,
    description: str,
    commands,
    *,
    use_workspace_debug: bool = False,
    target_robots=None,
    allocate_tty: bool = False,
):
    """Run commands over SSH for each target robot."""

    selected_robots = target_robots if target_robots is not None else gui.get_selected_robots()
    if isinstance(selected_robots, str):
        selected_robots = [selected_robots]
    if not selected_robots:
        print(f"No robots selected. Skipping {description}.")
        return

    cmd_list = list(commands)
    if not cmd_list:
        print(f"No commands provided for {description}.")
        return

    workspace = (gui.get_workspace_name() or "").strip()
    debug_prefix = _remote_debug_prefix(gui, workspace) if use_workspace_debug else ""
    env_prefix = [
        "source ~/.bashrc",
        f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/",
        "source /opt/ros/noetic/setup.bash",
    ]
    if workspace:
        env_prefix.append(f"source ~/{workspace}/devel/setup.bash")

    for robot in selected_robots:
        per_robot_cmds = []
        for cmd in cmd_list:
            resolved_cmd = cmd(robot) if callable(cmd) else cmd
            if not isinstance(resolved_cmd, str):
                raise ValueError("Commands must resolve to strings")
            resolved_cmd = resolved_cmd.strip()
            if debug_prefix:
                resolved_cmd = f"{debug_prefix}{resolved_cmd}"
            per_robot_cmds.append(resolved_cmd)

        remote_cmd = "; ".join(env_prefix + per_robot_cmds)
        ssh_cmd = ["ssh"]
        if allocate_tty:
            ssh_cmd += ["-t", "-t"]
        ssh_cmd.extend([robot, remote_cmd])
        print(f"{description} on {robot} with command: {remote_cmd}")
        stdin_target = None if allocate_tty else subprocess.DEVNULL
        _popen_with_debug(ssh_cmd, gui, stdin=stdin_target)


def _kill_ros_nodes(node_names) -> bool:
    """Kill the provided ROS nodes via rosnode kill, returns True if any ran."""
    nodes = sorted({n.strip() for n in node_names if n and n.strip()})
    if not nodes:
        return False

    killed_any = False
    for node in nodes:
        try:
            subprocess.check_call(
                ["rosnode", "kill", node],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            killed_any = True
        except (subprocess.CalledProcessError, FileNotFoundError):
            print(f"Failed to kill node {node}. It may already be stopped.")
    return killed_any


def _collect_driver_node_names(robot: str, launch_args) -> set:
    """Use roslaunch introspection to list nodes defined for the driver launch."""
    cli_args = [arg for arg in launch_args if arg]
    cmd = [
        "roslaunch",
        "--nodes",
        "mur_launch_hardware",
        f"{robot}.launch",
    ] + cli_args

    try:
        output = subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"Failed to introspect nodes for {robot}: {exc}")
        return set()

    nodes = set()
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        nodes.add(stripped)
    return nodes


def _load_driver_node_cache() -> dict:
    """Load the persisted driver nodes from disk (per robot)."""
    data = _load_gui_cache_payload()

    robots_section = data.get("robots") if isinstance(data, dict) else None
    raw = robots_section if isinstance(robots_section, dict) else {}

    # legacy support: old payload stored under top-level driver_nodes/robots
    if not raw:
        legacy = data.get("driver_nodes") if isinstance(data, dict) else None
        if isinstance(legacy, dict) and isinstance(legacy.get("robots"), dict):
            raw = legacy.get("robots", {})
        elif isinstance(data, dict) and isinstance(data.get("robots"), dict):
            raw = data.get("robots", {})

    normalized = {}
    for robot, nodes in raw.items():
        if not isinstance(robot, str):
            continue
        node_list = None
        if isinstance(nodes, dict) and isinstance(nodes.get("driver_nodes"), (list, tuple, set)):
            node_list = nodes.get("driver_nodes")
        elif isinstance(nodes, (list, tuple, set)):
            node_list = nodes

        if node_list:
            normalized[robot] = set(str(n).strip() for n in node_list if str(n).strip())
    return normalized


def _save_driver_node_cache(node_map: dict):
    """Persist the provided per-robot node map to disk as YAML."""
    payload = _load_gui_cache_payload()
    robots_section = payload.setdefault("robots", {})
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

    for robot, nodes in node_map.items():
        if not robot:
            continue
        normalized = sorted({str(n).strip() for n in nodes if str(n).strip()})
        robot_entry = robots_section.setdefault(robot, {})
        robot_entry["driver_nodes"] = normalized
        robot_entry["updated_at"] = timestamp

    _save_gui_cache_payload(payload)


def _clamp_spray_distance(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _extract_spray_distance(entry) -> Optional[float]:
    if isinstance(entry, dict):
        block = entry.get("spray_distance")
        if isinstance(block, dict):
            val = block.get("value")
            if isinstance(val, (int, float)):
                return float(val)
        if isinstance(block, (int, float)):
            return float(block)
        value = entry.get("value")
        if isinstance(value, (int, float)):
            return float(value)
    elif isinstance(entry, (int, float)):
        return float(entry)
    return None


def _load_spray_distance_cache(robot: Optional[str] = None, default: float = DEFAULT_SPRAY_DISTANCE) -> float:
    data = _load_gui_cache_payload()

    if robot:
        robots_section = data.get("robots") if isinstance(data, dict) else None
        if isinstance(robots_section, dict):
            robot_entry = robots_section.get(robot)
            value = _extract_spray_distance(robot_entry)
            if value is not None:
                return _clamp_spray_distance(value)

    # fallback to first available robot entry
    robots_section = data.get("robots") if isinstance(data, dict) else None
    if isinstance(robots_section, dict):
        for entry in robots_section.values():
            value = _extract_spray_distance(entry)
            if value is not None:
                return _clamp_spray_distance(value)

    # fallback to legacy top-level spray_distance
    value = _extract_spray_distance(data if isinstance(data, dict) else None)
    if value is not None:
        return _clamp_spray_distance(value)

    return default


def _save_spray_distance_cache(value: float, robots: Optional[Iterable[str]] = None):
    payload = _load_gui_cache_payload()
    robots_section = payload.setdefault("robots", {})
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    clamped = float(_clamp_spray_distance(value))

    robot_list = [r for r in (robots or []) if r]
    for robot in robot_list:
        entry = robots_section.setdefault(robot, {})
        entry["spray_distance"] = {
            "value": clamped,
            "updated_at": timestamp,
        }

    payload["spray_distance"] = {
        "value": clamped,
        "updated_at": timestamp,
    }

    _save_gui_cache_payload(payload)


def _normalize_path_index(value) -> Optional[int]:
    try:
        normalized = int(value)
    except (TypeError, ValueError):
        return None
    return max(0, normalized)


def _load_path_index_cache(default: int = DEFAULT_PATH_INDEX) -> int:
    data = _load_gui_cache_payload()
    if isinstance(data, dict):
        entry = data.get("path_index")
        if isinstance(entry, dict):
            for key in ("value", "default"):
                normalized = _normalize_path_index(entry.get(key))
                if normalized is not None:
                    return normalized
        else:
            normalized = _normalize_path_index(entry)
            if normalized is not None:
                return normalized
    normalized_default = _normalize_path_index(default)
    return normalized_default if normalized_default is not None else DEFAULT_PATH_INDEX


def _save_path_index_cache(value: int):
    normalized = _normalize_path_index(value)
    if normalized is None:
        return
    payload = _load_gui_cache_payload()
    payload["path_index"] = {
        "value": normalized,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    _save_gui_cache_payload(payload)


def _format_ros_list_arg(values):
    """Format Python list as double-quoted string literal for roslaunch args."""
    return '"' + str(values).replace("'", '"') + '"'


def _load_component_choice(default: str = DEFAULT_COMPONENT_NAME) -> str:
    data = _load_gui_cache_payload()
    name = None
    if isinstance(data, dict):
        entry = data.get("component")
        if isinstance(entry, dict):
            raw = entry.get("name") or entry.get("value")
            if isinstance(raw, str) and raw.strip():
                name = raw.strip()
        elif isinstance(entry, str) and entry.strip():
            name = entry.strip()
        elif isinstance(data.get("component_name"), str):
            candidate = data.get("component_name", "")
            if candidate.strip():
                name = candidate.strip()

    return name if name else default


def _save_component_choice(component_name: str):
    normalized = (component_name or "").strip()
    if not normalized:
        return
    payload = _load_gui_cache_payload()
    payload["component"] = {
        "name": normalized,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    payload["component_name"] = normalized
    _save_gui_cache_payload(payload)


def _default_component_transform():
    return {key: 0.0 for key in COMPONENT_TRANSFORM_KEYS}


def _coerce_float(value, fallback: float = 0.0) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            return fallback
    return fallback


def _normalize_component_transform(block) -> dict:
    normalized = _default_component_transform()
    if isinstance(block, dict):
        for key in COMPONENT_TRANSFORM_KEYS:
            normalized[key] = _coerce_float(block.get(key), normalized[key])
    return normalized


def _load_component_transform_entry(component_name: Optional[str]) -> dict:
    name = (component_name or DEFAULT_COMPONENT_NAME).strip() or DEFAULT_COMPONENT_NAME
    payload = _load_gui_cache_payload()
    transforms = payload.get("component_transforms") if isinstance(payload, dict) else None
    entry = {}
    if isinstance(transforms, dict):
        entry = transforms.get(name, {})
    return _normalize_component_transform(entry)


def _save_component_transform_entry(component_name: str, transform: dict):
    name = (component_name or "").strip()
    if not name:
        return
    payload = _load_gui_cache_payload()
    transforms = payload.setdefault("component_transforms", {})
    normalized = _normalize_component_transform(transform)
    stored = dict(normalized)
    stored["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    transforms[name] = stored
    _save_gui_cache_payload(payload)


class _DriverControlSession:
    """Maintains a lightweight SSH shell per robot for reliable cleanup commands."""

    def __init__(self, gui, robot: str, workspace: Optional[str]):
        self.robot = robot
        self.workspace = (workspace or "").strip()
        self.process = _popen_with_debug(
            ["ssh", "-T", robot, "bash", "-l"],
            gui,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if self.process is not None and self.process.stdin is not None:
            self._bootstrap_environment()
        else:
            self.process = None

    def _bootstrap_environment(self):
        env_cmds = [
            "source ~/.bashrc",
            f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/",
            "source /opt/ros/noetic/setup.bash",
        ]
        if self.workspace:
            env_cmds.append(f"source ~/{self.workspace}/devel/setup.bash")
        env_cmds.append("echo '[driver-control ready]' > /dev/null")
        self.send(" && ".join(env_cmds))

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def send(self, command: str) -> bool:
        if not self.is_alive() or not command:
            return False
        proc = self.process
        if proc is None or proc.stdin is None:
            return False
        try:
            proc.stdin.write(command.strip() + "\n")
            proc.stdin.flush()
            return True
        except Exception:
            return False

    def close(self):
        proc = self.process
        if not proc:
            return
        try:
            self.send("exit")
            if proc.stdin is not None:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()


def _ensure_driver_control_session(gui, robot: str, workspace: Optional[str]):
    if gui is None:
        return None
    sessions = getattr(gui, "_driver_control_sessions", {})
    session = sessions.get(robot)
    if session is not None and session.is_alive():
        return session
    session = _DriverControlSession(gui, robot, workspace)
    if session.is_alive():
        sessions[robot] = session
        gui._driver_control_sessions = sessions
        return session
    return None


def _driver_kill_commands(robot: str):
    return [
        f"pkill -f 'mur_launch_hardware {robot}.launch' || true",
        "pkill -f ur_robot_driver || true",
        "pkill -f roslaunch || true",
    ]


def _driver_kill_script(robot: str) -> str:
    return "; ".join(_driver_kill_commands(robot))


def _stop_non_driver_commands():
    return [
        "pkill -f mir_trajectory_follower || true",
        "pkill -f increment_path_index || true",
        "pkill -f path_index_advancer || true",
        "pkill -f complete_ur_trajectory_follower_ff_only || true",
        "pkill -f 'ur_direction_controller|orthogonal_error_correction|move_ur_to_start_pose|ur_vel_induced_by_mir|world_twist_in_mir|twist_combiner|ur_yaw_controller' || true",
        "pkill -f keyence_scanner_ljx8000 || true",
        "pkill -f profile_orthogonal_controller || true",
        "pkill -f strand-center-app || true",
        "pkill -f parse_mir_path || true",
        "pkill -f parse_ur_path || true",
        "pkill -f target_broadcaster || true",
        "pkill -f move_mir_to_start_pose || true",
        "pkill -f move_ur_to_start_pose || true",
    ]


def _stop_non_driver_script(_robot: str) -> str:
    return "; ".join(_stop_non_driver_commands())

class ROSInterface:
    def __init__(self, gui):
        self.gui = gui
        self.updated_poses = {}
        self.virtual_object_pose = None
        self.battery_states = {}
        self.active_battery_subs = set()
        self.current_index = _load_path_index_cache(DEFAULT_PATH_INDEX)
        self._cached_path_index = self.current_index
        self._cached_spray_distance = _load_spray_distance_cache()
        self._cached_servo_targets = _load_servo_targets_cache()
        self._cached_component_name = _load_component_choice(DEFAULT_COMPONENT_NAME)
        self._component_transform_cache = {}
        self._servo_state_lock = threading.Lock()
        self._latest_servo_positions = {}
        self._start_condition_topic = rospy.get_param("~start_condition_topic", "/start_condition")
        self._start_signal_pub = None
        self._start_signal_active = False
        self._start_signal_timer = QTimer(self.gui)
        self._start_signal_timer.setSingleShot(True)
        self._start_signal_timer.setInterval(10000)
        self._start_signal_timer.timeout.connect(self._deactivate_start_signal)

        # Rosbag config
        self.rosbag_topics = [
            # "/tf",
            "/ur_path_transformed",
            "/mir_path_transformed",
            "/profiles"
            "/path_index",
            "/laser_profile_offset_cmd_vel",
            "/orthogonal_error",
            "/orthogonal_twist",
            "/ur_error_world"
            "/mur620c/UR10_r/twist_controller/command_collision_free",
            "/mur620c/UR10_r/twist_controller/controller_input",
            "/ur_twist_direction_world",
            "/servo_target_pos_left",
            "/servo_target_pos_right",
            "/mur620c/UR10_r/ur_calibrated_pose",
            "/mur620c/UR10_r/global_tcp_pose",
            "/mur620c/mir_pose_simple"
        ]
        self.rosbag_enabled = {t: True for t in self.rosbag_topics}
        self.rosbag_process = None
        self.rosbag_dir = os.path.expanduser("~/rosbags")  # fixer Ordner
        os.makedirs(self.rosbag_dir, exist_ok=True)


        if not rospy.core.is_initialized():
            rospy.init_node("additive_manufacturing_gui", anonymous=True, disable_signals=True)

        # Subscriptions and publishers used by the GUI
        rospy.Subscriber('/path_index', Int32, self._path_idx_cb, queue_size=10)
        self._path_index_pub = rospy.Publisher('/path_index', Int32, queue_size=10, latch=False)
        self._init_dynamixel_publishers()
        self._init_servo_state_listener()
        # Receive medians from robot-side node (published by profiles_median_node on the mur)
        self._last_med_base = float('nan')
        self._last_med_map = float('nan')

        def _median_base_cb(msg: Float32):
            try:
                self._last_med_base = float(msg.data)
            except Exception:
                self._last_med_base = float('nan')
            try:
                self.gui.medians.emit(self._last_med_base, self._last_med_map)
            except Exception:
                pass

        def _median_map_cb(msg: Float32):
            try:
                self._last_med_map = float(msg.data)
            except Exception:
                self._last_med_map = float('nan')
            try:
                self.gui.medians.emit(self._last_med_base, self._last_med_map)
            except Exception:
                pass

        try:
            rospy.Subscriber('/profiles/median_base', Float32, _median_base_cb, queue_size=1)
            rospy.Subscriber('/profiles/median_map', Float32, _median_map_cb, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"Failed to subscribe to median topics: {e}")

        self._rosout_sub = None
        self._subscribe_rosout_logs()

    def get_cached_spray_distance(self) -> float:
        current = getattr(self, "_cached_spray_distance", DEFAULT_SPRAY_DISTANCE)
        robots = getattr(self.gui, "get_selected_robots", lambda: [])()
        target_robot = None
        if isinstance(robots, str):
            target_robot = robots
        elif isinstance(robots, (list, tuple)) and robots:
            target_robot = robots[0]

        resolved = _load_spray_distance_cache(target_robot, current)
        self._cached_spray_distance = resolved
        return resolved

    def persist_spray_distance(self, value: float, target_robots=None):
        clamped = _clamp_spray_distance(value)
        self._cached_spray_distance = clamped

        robots = target_robots if target_robots is not None else (
            getattr(self.gui, "get_selected_robots", lambda: [])()
        )
        if isinstance(robots, str):
            robots = [robots]
        elif robots is None:
            robots = []
        robots = [r for r in robots if r]
        _save_spray_distance_cache(clamped, robots)
        workspace = (getattr(self.gui, "get_workspace_name", lambda: "")() or "").strip()

        if not robots or not workspace:
            return

        def _run_deploy(targets, ws):
            for robot in targets:
                try:
                    _deploy_gui_cache_to_robot(robot, ws)
                except Exception as exc:
                    print(f"Failed to deploy GUI cache to {robot}: {exc}")

        threading.Thread(target=_run_deploy, args=(list(robots), workspace), daemon=True).start()

    def get_cached_servo_targets(self):
        targets = getattr(self, "_cached_servo_targets", (0.0, 0.0))
        if isinstance(targets, (list, tuple)) and len(targets) == 2:
            return targets
        return (0.0, 0.0)

    def persist_servo_targets(self, left_percent: float, right_percent: float):
        left = float(left_percent)
        right = float(right_percent)
        self._cached_servo_targets = (left, right)
        _save_servo_targets_cache(left, right)

        self._subscribe_rosout_logs()

    def get_cached_path_index(self) -> int:
        cached = getattr(self, "_cached_path_index", None)
        if isinstance(cached, int):
            return cached
        value = _load_path_index_cache(self.current_index)
        self._cached_path_index = value
        return value

    def persist_path_index(self, value: int):
        normalized = _normalize_path_index(value)
        if normalized is None:
            return
        self._cached_path_index = normalized
        self.current_index = normalized
        _save_path_index_cache(normalized)

    def get_cached_component_name(self) -> str:
        cached = getattr(self, "_cached_component_name", None)
        if isinstance(cached, str) and cached.strip():
            return cached.strip()
        resolved = _load_component_choice(DEFAULT_COMPONENT_NAME)
        self._cached_component_name = resolved
        return resolved

    def persist_component_choice(self, component_name: str):
        normalized = (component_name or "").strip()
        if not normalized:
            return
        self._cached_component_name = normalized
        _save_component_choice(normalized)

    def get_component_transform(self, component_name: Optional[str] = None) -> dict:
        name = (component_name or self.get_cached_component_name() or DEFAULT_COMPONENT_NAME).strip()
        if not name:
            name = DEFAULT_COMPONENT_NAME
        cache = getattr(self, "_component_transform_cache", {}) or {}
        if name not in cache:
            cache[name] = _load_component_transform_entry(name)
            self._component_transform_cache = cache
        entry = cache.get(name, _default_component_transform())
        return dict(entry)

    def persist_component_transform(self, component_name: Optional[str], transform: dict):
        name = (component_name or self.get_cached_component_name() or "").strip()
        if not name:
            return
        normalized = _normalize_component_transform(transform)
        cache = getattr(self, "_component_transform_cache", {}) or {}
        cache[name] = normalized
        self._component_transform_cache = cache
        _save_component_transform_entry(name, normalized)

    def _subscribe_rosout_logs(self):
        """Ensure we have a single /rosout subscription feeding the GUI."""
        if getattr(self, "_rosout_sub", None) is not None:
            return
        try:
            self._rosout_sub = rospy.Subscriber('/rosout', Log, self._rosout_cb, queue_size=50)
        except Exception as e:
            self._rosout_sub = None
            rospy.logwarn(f"Failed to subscribe to /rosout: {e}")

    def shutdown(self):
        """Shut down background ROS helpers so the launcher terminal recovers."""
        if getattr(self, "_is_shutting_down", False):
            return
        self._is_shutting_down = True

        try:
            if hasattr(self, "_start_signal_timer"):
                self._start_signal_timer.stop()
        except Exception:
            pass
        self._deactivate_start_signal()

        # Stop an active rosbag recording first so it flushes cleanly.
        if self.rosbag_process and self.rosbag_process.poll() is None:
            try:
                self.rosbag_process.send_signal(signal.SIGINT)
                self.rosbag_process.wait(timeout=4)
            except Exception:
                try:
                    self.rosbag_process.terminate()
                    self.rosbag_process.wait(timeout=2)
                except Exception:
                    self.rosbag_process.kill()
            finally:
                self.rosbag_process = None

        # Tear down persistent SSH cleanup sessions to avoid dangling TTYs.
        sessions = getattr(self.gui, "_driver_control_sessions", {}) or {}
        for robot, session in list(sessions.items()):
            try:
                session.close()
            except Exception as exc:
                print(f"Failed to close driver cleanup session for {robot}: {exc}")
        self.gui._driver_control_sessions = {}

        try:
            rospy.signal_shutdown("GUI closed")
        except Exception as exc:
            print(f"Failed to signal rospy shutdown: {exc}")
        
    def _launch_process(self, command, shell=False, **kwargs):
        return _popen_with_debug(command, self.gui, shell=shell, **kwargs)

    def _launch_in_terminal(self, title: str, command: str):
        cmd = ["terminator", f"--title={title}", "-x", f"{command}; exec bash"]
        return self._launch_process(cmd)

    def _debug_prefix_for_workspace(self, workspace: Optional[str] = None) -> str:
        workspace = workspace or self.gui.get_workspace_name()
        return _remote_debug_prefix(self.gui, workspace)

    def init_override_velocity_slider(self):
        self.velocity_override_manual_pub = rospy.Publisher('/velocity_override_manual', Float32, queue_size=10, latch=True)
        self.velocity_override_pub = rospy.Publisher('/velocity_override', Float32, queue_size=10, latch=True)
        
        def _handle_override_change(value: int):
            self.gui.override_value_label.setText(f"{value}%")
            if not self.gui.laser_override_radio.isChecked():
                self.velocity_override_pub.publish(value / 100.0)
            else:
                self.velocity_override_manual_pub.publish(value / 100.0)
        
        self.gui.override_slider.valueChanged.connect(_handle_override_change)
        # publish to /velocity_override as well:
        # ensure the initial slider value is sent once
        self.velocity_override_pub.publish(self.gui.override_slider.value() / 100.0)

    def init_nozzle_override_slider(self):
        """Wire the nozzle height override slider to /nozzle_height_override."""
        self.nozzle_height_override_pub = rospy.Publisher('/nozzle_height_override', Float32, queue_size=10, latch=True)
        slider = getattr(self.gui, 'nozzle_override_slider', None)
        label = getattr(self.gui, 'nozzle_override_value_label', None)
        if slider is None or label is None:
            return

        def _handle_change(raw_value: int):
            meters = raw_value / 1000.0  # slider uses millimeters
            label.setText(f"{raw_value:.1f} mm")
            self.nozzle_height_override_pub.publish(Float32(meters))

        slider.valueChanged.connect(_handle_change)
        _handle_change(slider.value())

    def trigger_start_signal(self):
        publisher = self._ensure_start_signal_publisher()
        if publisher is None:
            rospy.logerr("Start signal publisher unavailable; cannot trigger start condition.")
            return

        try:
            publisher.publish(Bool(data=True))
        except Exception as exc:
            rospy.logerr(f"Failed to publish start signal: {exc}")
            return

        self._start_signal_active = True
        if hasattr(self.gui, "update_start_signal_visual"):
            self.gui.update_start_signal_visual(True)
        if hasattr(self, "_start_signal_timer"):
            self._start_signal_timer.start()

    def _ensure_start_signal_publisher(self):
        if self._start_signal_pub is not None:
            return self._start_signal_pub
        try:
            self._start_signal_pub = rospy.Publisher(
                self._start_condition_topic,
                Bool,
                queue_size=1,
                latch=True,
            )
        except Exception as exc:
            rospy.logerr(f"Failed to create start signal publisher: {exc}")
            self._start_signal_pub = None
        return self._start_signal_pub

    def _deactivate_start_signal(self):
        publisher = getattr(self, "_start_signal_pub", None)
        if publisher is not None:
            try:
                publisher.unregister()
            except Exception as exc:
                rospy.logwarn(f"Failed to unregister start signal publisher: {exc}")
            finally:
                self._start_signal_pub = None

        if self._start_signal_active:
            self._start_signal_active = False
            if hasattr(self.gui, "update_start_signal_visual"):
                self.gui.update_start_signal_visual(False)
    
    def _path_idx_cb(self, msg: Int32):
        self.current_index = msg.data
        self._cached_path_index = self.current_index
        self.gui.path_idx.emit(self.current_index)  # Update the GUI with the new index

    def publish_path_index(self, value: int):
        """Publish the provided index on /path_index so downstream nodes follow the change."""
        try:
            normalized = int(value)
        except (TypeError, ValueError):
            rospy.logwarn(f"Ignoring invalid path index value: {value}")
            return

        publisher = getattr(self, "_path_index_pub", None)
        if publisher is None:
            try:
                publisher = rospy.Publisher('/path_index', Int32, queue_size=10, latch=True)
                self._path_index_pub = publisher
            except Exception as exc:
                rospy.logerr(f"Failed to create path index publisher: {exc}")
                return

        try:
            publisher.publish(Int32(data=normalized))
            self.current_index = normalized
            self._cached_path_index = normalized
            rospy.loginfo(f"Published path index {normalized}")
        except Exception as exc:
            rospy.logerr(f"Failed to publish path index {normalized}: {exc}")

    def start_roscore(self):
        """Starts roscore on the roscore PC."""
        workspace = self.gui.get_workspace_name()
        debug_prefix = self._debug_prefix_for_workspace(workspace)
        command = (
            "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; "
            f"{debug_prefix}roscore; exec bash'"
        )
        self._launch_in_terminal("Roscore Terminal", command)

    def start_mocap(self):
        """Starts the motion capture system on the roscore PC."""
        workspace = self.gui.get_workspace_name()
        debug_prefix = self._debug_prefix_for_workspace(workspace)
        command = (
            "ssh -t -t roscore 'source ~/.bashrc; source /opt/ros/noetic/setup.bash; "
            "source ~/catkin_ws/devel/setup.bash; "
            f"{debug_prefix}roslaunch launch_mocap mocap_launch.launch; exec bash'"
        )
        self._launch_in_terminal("Mocap", command)

    def start_sync(self):
        """Starts file synchronization between workspace and selected robots."""
        selected_robots = self.gui.get_selected_robots()
        self.workspace_name = self.gui.get_workspace_name()
        self.gui.btn_sync.setStyleSheet("background-color: lightgreen;")  # Mark sync as active
        
        for robot in selected_robots:
            command = f"while inotifywait -r -e modify,create,delete,move ~/{self.workspace_name}/src; do \n" \
                      f"rsync --delete -avzhe ssh ~/{self.workspace_name}/src rosmatch@{robot}:~/{self.workspace_name}/ \n" \
                      "done"
            self._launch_in_terminal(f"Sync to {robot}", command)

    def prime_driver_cleanup_sessions(self):
        """Manually open/refresh the persistent SSH session used for driver cleanup."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robots selected. Skipping driver cleanup session setup.")
            return

        workspace = self.gui.get_workspace_name()
        ready = []
        failed = []

        for robot in selected_robots:
            session = _ensure_driver_control_session(self.gui, robot, workspace)
            if session and session.is_alive():
                ready.append(robot)
            else:
                failed.append(robot)

        if ready:
            print(f"Driver cleanup channel ready for: {', '.join(ready)}")
        if failed:
            print(f"Failed to open driver cleanup channel for: {', '.join(failed)}")

    def update_button_status(self):
        # --- Parser + Keyence status ---
        node_cache = self._get_rosnode_list()
        mir = self.is_ros_node_running_fast("/retrieve_and_publish_mir_path", node_cache)
        ur = self.is_ros_node_running_fast("/retrieve_and_publish_ur_path", node_cache)
        key = self.is_ros_node_running_fast("/keyence_ljx_profile_node", node_cache)
        flow = self.is_ros_node_running_fast("/flow_serial_bridge", node_cache)
        tgt = self.is_ros_node_running_fast("/target_broadcaster", node_cache)
        laser = self.is_ros_node_running_fast("/laser_profile_controller", node_cache)
        mocap = self.is_ros_node_running_fast("/qualisys", node_cache)

        # --- Button coloring ---
        self.gui.btn_parse_mir.setStyleSheet("background-color: lightgreen;" if mir else "background-color: lightgray;") if hasattr(self.gui,"btn_parse_mir") else None
        self.gui.btn_parse_ur.setStyleSheet("background-color: lightgreen;" if ur else "background-color: lightgray;") if hasattr(self.gui,"btn_parse_ur") else None
        self.gui.btn_keyence.setStyleSheet("background-color: lightgreen;" if key else "background-color: lightgray;") if hasattr(self.gui,"btn_keyence") else None
        if hasattr(self.gui, "btn_flow_sensor"):
            self.gui.btn_flow_sensor.setStyleSheet("background-color: lightgreen;" if flow else "background-color: lightgray;")
        self.gui.btn_target_broadcaster.setStyleSheet("background-color: lightgreen;" if tgt else "background-color: lightgray;") if hasattr(self.gui,"btn_target_broadcaster") else None
        self.gui.btn_laser_ctrl.setStyleSheet("background-color: lightgreen;" if laser else "background-color: lightgray;") if hasattr(self.gui,"btn_laser_ctrl") else None
        self.gui.btn_mocap.setStyleSheet("background-color: lightgreen;" if mocap else "background-color: lightgray;") if hasattr(self.gui,"btn_mocap") else None

        # --- Driver status (UR hardware interface nodes) ---
        driver_nodes = self._driver_node_names(
            robots=self.gui.get_selected_robots(),
            urs=self.gui.get_selected_urs(),
        )
        driver_total = len(driver_nodes)
        driver_active = sum(1 for node in driver_nodes if node in node_cache)
        if hasattr(self.gui, "btn_launch_drivers"):
            if driver_total == 0 or driver_active == 0:
                driver_color = "background-color: lightgray;"
            elif driver_active == driver_total:
                driver_color = "background-color: lightgreen;"
            else:
                driver_color = "background-color: orange;"
            self.gui.btn_launch_drivers.setStyleSheet(driver_color)

        # --- Auto-start target broadcaster ---
        if mir and ur and not tgt: 
            print("Auto-starting /target_broadcaster…"); _popen_with_debug(f"roslaunch print_hw target_broadcaster.launch initial_path_index:={self.gui.idx_spin.value()}", self.gui, shell=True)

    def _get_rosnode_list(self):
        """Return the current ROS node names as a set for quick membership checks."""
        try:
            output = subprocess.check_output(["rosnode", "list"], text=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            return set()
        return {line.strip() for line in output.splitlines() if line.strip()}

    def is_ros_node_running(self, node_name, node_cache=None):
        """Checks if a specific ROS node is running by using `rosnode list`."""
        nodes = node_cache if node_cache is not None else self._get_rosnode_list()
        return node_name in nodes

    def is_ros_node_running_fast(self, name, node_cache=None):
        if node_cache is None:
            node_cache = self._get_rosnode_list()
        return name in node_cache

    def _driver_node_names(self, robots=None, urs=None):
        """Return the UR driver node names to monitor for the given robot/UR selection."""
        robot_names = robots if robots else list(getattr(self.gui, "robots", {}).keys())
        if not robot_names:
            return []
        ur_prefixes = urs if urs else ["UR10_l", "UR10_r"]
        names = []
        for robot in robot_names:
            for ur in ur_prefixes:
                names.append(f"/{robot}/{ur}/ur_hardware_interface")
        return names
        
    def make_ur_callback(self, robot):
        def callback(msg):
            if robot not in self.battery_states:
                self.battery_states[robot] = {}
            self.battery_states[robot]["ur"] = msg.data
            self.update_battery_display(robot)
        return callback

    def make_mir_callback(self, robot):
        def callback(msg):
            if robot not in self.battery_states:
                self.battery_states[robot] = {}
            self.battery_states[robot]["mir"] = msg.percentage * 100  # falls 0.0–1.0
            self.update_battery_display(robot)
        return callback

    def update_battery_display(self, robot):
        mir_label, ur_label = self.gui.battery_labels.get(robot, (None, None))
        if not mir_label or not ur_label:
            return

        state = self.battery_states.get(robot, {})

        if "mir" in state:
            val = state["mir"]
            mir_label.setText(f"MiR: {val:.0f}%")
            mir_label.setStyleSheet(f"background-color: {self.get_color(val)}; padding: 2px;")

        if "ur" in state:
            val = state["ur"]
            ur_label.setText(f"UR: {val:.0f}%")
            ur_label.setStyleSheet(f"background-color: {self.get_color(val)}; padding: 2px;")


    def check_and_subscribe_battery(self):
        if not rospy.core.is_initialized():
            rospy.init_node("battery_status_gui", anonymous=True, disable_signals=True)

        for robot in self.gui.get_selected_robots():
            if robot in self.active_battery_subs:
                continue  # bereits abonniert

            rospy.Subscriber(f"/{robot}/bms_status/SOC", Float32, self.make_ur_callback(robot), queue_size=1)
            rospy.Subscriber(f"/{robot}/battery_state", BatteryState, self.make_mir_callback(robot))
            self.active_battery_subs.add(robot)
            print(f"🔌 Subscribed to battery topics for {robot}")

    def get_color(self, val):
        if val >= 50:
            return "lightgreen"
        elif val >= 20:
            return "orange"
        else:
            return "red"

    def launch_keyence_scanner(self):
        """Launches the Keyence scanner on the robots PC."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping Keyence scanner launch.")
            return

        _run_remote_commands(
            self.gui,
            "Launching Keyence scanner",
            ["roslaunch laser_scanner_tools keyence_scanner_ljx8000.launch"],
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def launch_flow_sensor_bridge(self):
        """Launches the foam volume flow sensor bridge on the robots."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping flow sensor bridge launch.")
            return

        _run_remote_commands(
            self.gui,
            "Launching flow sensor bridge",
            ["roslaunch foam_volume_flow_sensor flow_serial_bridge.launch"],
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def stop_flow_sensor_bridge(self):
        """Stops the foam volume flow sensor bridge on the robots."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping flow sensor bridge stop.")
            return

        stop_cmds = [
            "rosnode kill /flow_serial_bridge || true",
            "pkill -f flow_serial_bridge || true",
        ]

        _run_remote_commands(
            self.gui,
            "Stopping flow sensor bridge",
            stop_cmds,
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def launch_laser_orthogonal_controller(self):
        """Launches the Keyence scanner on the robots PC."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping orthogonal controller launch.")
            return

        _run_remote_commands(
            self.gui,
            "Launching orthogonal controller",
            ["roslaunch laser_scanner_tools profile_orthogonal_controller.launch"],
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def _orth_pid_config_path(self) -> str:
        try:
            pkg_path = rospkg.RosPack().get_path("ur_trajectory_follower")
            return os.path.join(pkg_path, "config", "pid_twist_controller_direction_orthogonal.yaml")
        except Exception as exc:
            print(f"Falling back to relative orthogonal PID path: {exc}")
            return os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "..",
                    "ur_trajectory_follower",
                    "config",
                    "pid_twist_controller_direction_orthogonal.yaml",
                )
            )

    def _remote_orth_pid_config_path(self, workspace: Optional[str]) -> str:
        workspace = (workspace or "").strip()
        if workspace:
            return f"~/{workspace}/src/match_additive_manufacturing/ur_trajectory_follower/config/pid_twist_controller_direction_orthogonal.yaml"
        return self._orth_pid_config_path()

    def load_orthogonal_pid_config(self) -> dict:
        path = self._orth_pid_config_path()
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return data if isinstance(data, dict) else {}

    def _push_orth_pid_config_to_robots(self, local_path: str):
        workspace = (self.gui.get_workspace_name() or "").strip()
        robots = self.gui.get_selected_robots()
        if not workspace or not robots:
            return
        remote_path = self._remote_orth_pid_config_path(workspace)
        for robot in robots:
            try:
                subprocess.check_call(["scp", local_path, f"{robot}:{remote_path}"])
            except subprocess.CalledProcessError as exc:
                print(f"Failed to copy PID config to {robot}: {exc}")

    def save_orthogonal_pid_config(self, payload: dict):
        path = self._orth_pid_config_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(payload or {}, stream, default_flow_style=False, sort_keys=False)
        self._push_orth_pid_config_to_robots(path)

    def start_orthogonal_pid_controller(self):
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping orthogonal PID start.")
            return

        workspace = (self.gui.get_workspace_name() or "").strip()
        if workspace:
            cfg_arg = (
                f"$HOME/{workspace}/src/match_additive_manufacturing/ur_trajectory_follower/config/"
                "pid_twist_controller_direction_orthogonal.yaml"
            )
        else:
            cfg_arg = "$(rospack find ur_trajectory_follower)/config/pid_twist_controller_direction_orthogonal.yaml"

        def _launch_cmd(_robot):
            return (
                "roslaunch additive_manufacturing_helpers pid_twist_controller.launch "
                "node_name:=pid_twist_controller_orthogonal "
                "input_twist_topic:=/orthogonal_error "
                "output_twist_topic:=/orthogonal_twist "
                f"pid_values_path:={cfg_arg}"
            )

        _run_remote_commands(
            self.gui,
            "Starting orthogonal PID controller",
            [_launch_cmd],
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def stop_orthogonal_pid_controller(self):
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping orthogonal PID stop.")
            return

        stop_cmds = [
            "rosnode kill /pid_twist_controller_orthogonal/pid_twist_controller_orthogonal || true",
            "rosnode kill /pid_twist_controller_orthogonal || true",
            "pkill -f pid_twist_controller_orthogonal || true",
        ]

        _run_remote_commands(
            self.gui,
            "Stopping orthogonal PID controller",
            stop_cmds,
            use_workspace_debug=True,
            target_robots=selected_robots,
        )

    def launch_strand_center_app(self):
        """Launch the DepthAI strand-center app on the selected robots over SSH."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping strand-center launch.")
            return

        workspace = self.gui.get_workspace_name()
        script_rel = "src/match_additive_manufacturing/camera_oak/strand-center-app/main.py"

        for robot in selected_robots:
            remote_script = f"~/{workspace}/{script_rel}"
            debug_prefix = self._debug_prefix_for_workspace(workspace)
            command = (
                f"ssh -t -t {robot} '"
                "source ~/.bashrc; "
                f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/; "
                "source /opt/ros/noetic/setup.bash; "
                f"source ~/{workspace}/devel/setup.bash; "
                f"{debug_prefix}python3 {remote_script}; "
                "exec bash'"
            )

            self._launch_in_terminal(f"StrandCenter {robot}", command)

    def start_local_rosbag(self, fname, topics):
        cmd = ["rosbag", "record", "-O", fname, "--lz4"] + topics
        print("Starting LOCAL rosbag:", " ".join(cmd))
        proc = subprocess.Popen(cmd)
        self._local_rosbag_proc = proc
        return proc

    def stop_local_rosbag(self):
        proc = getattr(self, "_local_rosbag_proc", None)
        if not proc or proc.poll() is not None:
            return
        print("Stopping LOCAL rosbag…")
        try:
            proc.send_signal(signal.SIGINT); proc.wait(timeout=4)
        except:
            proc.terminate()
        self._local_rosbag_proc = None

    def start_remote_rosbag(self, gui, robot, fname, topics):
        pidfile = f"/home/rosmatch/rosbags/rosbag_{robot}.pid"
        rosbag_cmd = f"rosbag record -O {fname} --lz4 {' '.join(topics)}"

        remote_cmd = (
            f"ssh -t -t {robot} '"
            "source ~/.bashrc; "
            f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/; "
            "source /opt/ros/noetic/setup.bash; "
            "source ~/catkin_ws/devel/setup.bash; "
            "mkdir -p ~/rosbags; "
            f"nohup {rosbag_cmd} > /home/rosmatch/rosbags/rosbag_{robot}.log 2>&1 & "
            "echo $! > " + pidfile + "; "
            "exec bash'"
        )

        print("Starting REMOTE rosbag:", remote_cmd)
        proc = _popen_with_debug([
            "terminator",
            f"--title=Rosbag {robot}",
            "-x",
            f"{remote_cmd}; exec bash"
        ], gui)

        if not hasattr(gui, "_remote_rosbag_procs"):
            gui._remote_rosbag_procs = {}
        gui._remote_rosbag_procs[robot] = proc
        return proc


    def stop_remote_rosbag(self, gui, robot):
        pidfile = f"/home/rosmatch/rosbags/rosbag_{robot}.pid"
        stop_cmd = (
            f"ssh -t -t {robot} '"
            f"if [ -f {pidfile} ]; then "
            f"  pid=$(cat {pidfile}); "
            f"  echo Stopping rosbag pid $pid; "
            f"  kill -2 $pid || true; "
            f"  sleep 2; "
            f"  kill $pid || true; "
            f"  rm {pidfile}; "
            "fi; "
            "exit'"
        )
        print(f"Stopping REMOTE rosbag on {robot}…")
        subprocess.Popen(stop_cmd, shell=True)

        # mark process as stopped in GUI
        if hasattr(gui, "_remote_rosbag_procs"):
            gui._remote_rosbag_procs[robot] = None



    def toggle_rosbag_record(self, gui):
        # rebuild topic selection
        local_topics  = [t for t, s in gui.topic_settings.items() if s["local"]]
        remote_topics = [t for t, s in gui.topic_settings.items() if s["remote"]]

        # check running state
        local_running  = hasattr(self, "_local_rosbag_proc") and self._local_rosbag_proc and self._local_rosbag_proc.poll() is None
        remote_running = hasattr(gui, "_remote_rosbag_procs") and any(
            p and p.poll() is None for p in gui._remote_rosbag_procs.values()
        )

        # --- STOP ---
        if local_running or remote_running:
            self.stop_local_rosbag()
            if hasattr(gui, "_remote_rosbag_procs"):
                for robot in gui._remote_rosbag_procs:
                    self.stop_remote_rosbag(gui, robot)

            gui.btn_rosbag_record.setStyleSheet("background-color: lightgray;")
            return

        # --- START ---
        ts = time.strftime("%Y%m%d_%H%M%S")
        local_fname  = f"{self.rosbag_dir}/record_{ts}_GUI-PC"
        remote_fname = f"~/rosbags/record_{ts}_MuR"

        robot_list = gui.get_selected_robots() or ["mur620c"]

        if local_topics:
            self.start_local_rosbag(local_fname, local_topics)

        if remote_topics:
            for robot in robot_list:
                self.start_remote_rosbag(gui, robot, remote_fname, remote_topics)

        gui.btn_rosbag_record.setStyleSheet("background-color: red; color: white;")


    # -------------------- Dynamixel Driver & Servo Targets --------------------
    def start_dynamixel_driver(self):
        """Start the Dynamixel servo driver on the selected robots over SSH (new terminal per robot)."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping Dynamixel start.")
            return
        workspace = self.gui.get_workspace_name()
        for robot in selected_robots:
            debug_prefix = self._debug_prefix_for_workspace(workspace)
            command = (
                f"ssh -t -t {robot} '"
                f"source ~/.bashrc; "
                f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/; "
                f"source /opt/ros/noetic/setup.bash; "
                f"source ~/{workspace}/devel/setup.bash; "
                f"{debug_prefix}roslaunch dynamixel_match dynamixel_motors.launch; exec bash'"
            )
            self._launch_in_terminal(f"Dynamixel {robot}", command)

    def stop_dynamixel_driver(self):
        """Stop the Dynamixel servo driver on the selected robots over SSH."""
        selected_robots = self.gui.get_selected_robots()
        if not selected_robots:
            print("No robot selected. Skipping Dynamixel stop.")
            return
        workspace = self.gui.get_workspace_name()
        for robot in selected_robots:
            # Kill by rosnode and fall back to process patterns
            remote_cmd = (
                "source ~/.bashrc; "
                F"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/; "
                "source /opt/ros/noetic/setup.bash; "
                f"source ~/{workspace}/devel/setup.bash; "
                "rosnode kill /servo_driver || true; "
                "pkill -f dynamixel_motors.launch || true; "
                "pkill -f servo_driver.py || true; "
                "pkill -f dynamixel_workbench_controllers || true;"
            )
            ssh_cmd = ["ssh", robot, remote_cmd]
            _popen_with_debug(ssh_cmd, self.gui, stdin=subprocess.DEVNULL)

    def _init_dynamixel_publishers(self):
        """Initialize publishers for left/right servo target topics (Int16)."""
        try:
            self.servo_left_pub = rospy.Publisher('/servo_target_pos_left', Int16, queue_size=10)
            self.servo_right_pub = rospy.Publisher('/servo_target_pos_right', Int16, queue_size=10)
        except Exception as e:
            rospy.logwarn(f"Failed to create Dynamixel target publishers: {e}")

    def publish_servo_targets(self, left_value: int, right_value: int):
        """Publish target positions for both servos as Int16 (0..4095).
        Values are forwarded as std_msgs/Int16 on /servo_target_pos_left and /servo_target_pos_right.
        """
        if not hasattr(self, 'servo_left_pub'):
            self._init_dynamixel_publishers()
        try:
            # Coerce to servo range 0..4095
            left_int = max(min(int(round(left_value)), 4095), 0)
            right_int = max(min(int(round(right_value)), 4095), 0)
            self.servo_left_pub.publish(Int16(left_int))
            self.servo_right_pub.publish(Int16(right_int))
            rospy.loginfo(f"Published servo targets L={left_int}, R={right_int}")
        except Exception as e:
            rospy.logerr(f"Failed to publish servo targets: {e}")

    def _init_servo_state_listener(self):
        """Subscribe to the Dynamixel state topic to cache latest raw positions."""
        try:
            rospy.Subscriber(
                '/dynamixel_workbench/dynamixel_state',
                DynamixelStateList,
                self._servo_state_cb,
                queue_size=1,
            )
        except Exception as exc:
            rospy.logwarn(f"Failed to subscribe to dynamixel state topic: {exc}")

    def _servo_state_cb(self, msg: DynamixelStateList):
        entries = getattr(msg, 'dynamixel_state', []) if msg is not None else []
        if not entries:
            return
        with self._servo_state_lock:
            store = self._latest_servo_positions
            for state in entries:
                try:
                    pos_val = int(round(getattr(state, 'present_position', 0)))
                except Exception:
                    continue
                keys = set()
                name = getattr(state, 'name', '')
                if isinstance(name, str) and name.strip():
                    normalized = name.strip().lower()
                    keys.add(normalized)
                    if normalized.startswith('servo_'):
                        keys.add(normalized.replace('servo_', '', 1))
                    keys.add(f"servo_{normalized}")
                try:
                    motor_id = int(getattr(state, 'id', -1))
                    if motor_id >= 0:
                        keys.add(str(motor_id))
                        keys.add(f"id_{motor_id}")
                except Exception:
                    pass
                for key in keys:
                    store[key] = pos_val

    def get_latest_servo_position(self, which: str):
        """Return the most recent raw position for the requested servo."""
        if not which:
            return None
        lookup_keys = []
        normalized = str(which).strip().lower()
        if normalized:
            lookup_keys.extend([
                normalized,
                f"servo_{normalized}",
                normalized.replace('servo_', '', 1) if normalized.startswith('servo_') else '',
            ])
            if normalized in ('left', 'right'):
                lookup_keys.append('1' if normalized == 'left' else '2')
        with self._servo_state_lock:
            snapshot = dict(self._latest_servo_positions)
        for key in lookup_keys:
            if key and key in snapshot:
                return snapshot[key]
        return None

    def _rosout_cb(self, msg: Log):
        """
        Callback for /rosout (rosgraph_msgs/Log).
        Forwards a compact text line to the Qt GUI via signal.
        """
        try:
            text = getattr(msg, "msg", "")

            # --- Blacklist bestimmter Meldungen ---
            blacklist = [
                "The complete state of the robot is not yet known",
                "Battery:",
                "Unable to transform object from frame",
                "tf_static: sent 17 transforms",
                "tf_static: updated transform",
                # hier ggf. weitere Phrasen ergänzen
            ]
            for phrase in blacklist:
                if phrase in text:
                    return  # einfach ignorieren

            # Map level to readable string (if constants exist)
            level_map = {
                getattr(Log, "DEBUG", 1): "DEBUG",
                getattr(Log, "INFO", 2): "INFO",
                getattr(Log, "WARN", 4): "WARN",
                getattr(Log, "ERROR", 8): "ERROR",
                getattr(Log, "FATAL", 16): "FATAL",
            }
            level = level_map.get(msg.level, str(msg.level))
            node_name = getattr(msg, "name", "?")

            if hasattr(self.gui, "ros_log_signal"):
                self.gui.ros_log_signal.emit(level, node_name, text)
        except Exception as e:
            rospy.logwarn(f"Error while forwarding /rosout message: {e}")



def launch_ros(gui, package, launch_file):
    selected_robots = gui.get_selected_robots()
    robot_names_str = "[" + ",".join(f"'{r}'" for r in selected_robots) + "]"

    command = f"roslaunch {package} {launch_file} robot_names:={robot_names_str}"
    print(f"Executing: {command}")
    _popen_with_debug(command, gui, shell=True)

def start_status_update(gui):
    threading.Thread(target=update_status, args=(gui,), daemon=True).start()

def update_status(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    active_counts = {"force_torque_sensor_controller": 0, "twist_controller": 0, "arm_controller": 0, "admittance": 0}
    total_count = len(selected_robots) * len(selected_urs)
    
    for robot in selected_robots:
        for ur in selected_urs:
            service_name = f"/{robot}/{ur}/controller_manager/list_controllers"
            try:
                output = subprocess.check_output(f"rosservice call {service_name}", shell=True).decode()
                controllers = yaml.safe_load(output).get("controller", [])
                for controller in controllers:
                    if controller.get("state") == "running":
                        active_counts[controller["name"]] = active_counts.get(controller["name"], 0) + 1
            except Exception as e:
                rospy.logwarn(f"Error checking controllers for {robot}/{ur}: {e}")
    
    status_text = """
    Force/Torque Sensor: {}/{} {}
    Twist Controller: {}/{} {}
    Arm Controller: {}/{} {}
    Admittance Controller: {}/{} {}
    """.format(
        active_counts["force_torque_sensor_controller"], total_count, get_status_symbol(active_counts["force_torque_sensor_controller"], total_count),
        active_counts["twist_controller"], total_count, get_status_symbol(active_counts["twist_controller"], total_count),
        active_counts["arm_controller"], total_count, get_status_symbol(active_counts["arm_controller"], total_count),
        active_counts["admittance"], total_count, get_status_symbol(active_counts["admittance"], total_count),
    )
    
    gui.status_label.setText(status_text)

def get_status_symbol(active, total):
    if active == total:
        return "✅"
    elif active > 0:
        return "⚠️"
    return "❌"

def open_rviz(gui):
    command = "roslaunch print_gui launch_rviz.launch"
    _popen_with_debug(command, gui, shell=True)

def turn_on_arm_controllers(gui):
    """Turns on all arm controllers for the selected robots."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    ur_prefixes_str = _format_ros_list_arg(selected_urs)

    for robot in selected_robots:
        robot_list_str = _format_ros_list_arg([robot])
        _run_remote_commands(
            gui,
            f"Turning on arm controllers",
            [
                (
                    f"roslaunch print_gui turn_on_all_arm_controllers.launch "
                    f"robot_names:={robot_list_str} UR_prefixes:={ur_prefixes_str}"
                )
            ],
            use_workspace_debug=True,
            target_robots=[robot],
        )

def turn_on_twist_controllers(gui):
    """Turns on all twist controllers for the selected robots."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    ur_prefixes_str = _format_ros_list_arg(selected_urs)

    for robot in selected_robots:
        robot_list_str = _format_ros_list_arg([robot])
        _run_remote_commands(
            gui,
            "Turning on twist controllers",
            [
                (
                    f"roslaunch print_gui turn_on_all_twist_controllers.launch "
                    f"robot_names:={robot_list_str} UR_prefixes:={ur_prefixes_str}"
                )
            ],
            use_workspace_debug=True,
            target_robots=[robot],
        )

def enable_all_urs(gui):
    """Enables all UR robots for the selected configurations."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping launch.")
        return

    ur_prefixes_str = _format_ros_list_arg(selected_urs)

    for robot in selected_robots:
        robot_list_str = _format_ros_list_arg([robot])
        _run_remote_commands(
            gui,
            "Enabling URs",
            [
                (
                    f"roslaunch print_gui enable_all_URs.launch "
                    f"robot_names:={robot_list_str} UR_prefixes:={ur_prefixes_str}"
                )
            ],
            use_workspace_debug=True,
            target_robots=[robot],
        )


def launch_drivers(gui):
    """SSH into the selected robots, start drivers, and keep a cleanup channel per robot."""
    selected_robots = gui.get_selected_robots()
    workspace_name = gui.get_workspace_name()

    if not selected_robots:
        print("No robots selected. Skipping driver launch.")
        return

    if not hasattr(gui, "_driver_processes"):
        gui._driver_processes = []
    gui._driver_processes = [p for p in gui._driver_processes if p and p.poll() is None]
    gui._driver_robots = list(set(selected_robots))

    selected_urs = gui.get_selected_urs()
    try:
        offset_values = gui.get_tcp_offset_sixd()
    except AttributeError:
        offset_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tcp_offset_literal = "[" + ",".join(f"{value:.6f}" for value in offset_values) + "]"

    launch_arg_pairs = [
        ("launch_ur_l", "true" if "UR10_l" in selected_urs else "false"),
        ("launch_ur_r", "true" if "UR10_r" in selected_urs else "false"),
        ("tcp_offset", tcp_offset_literal),
    ]
    launch_cli_args = [f"{name}:={value}" for name, value in launch_arg_pairs]
    launch_suffix = "".join(f" {arg}" for arg in launch_cli_args)
    persist_updates = {}

    for robot in selected_robots:
        workspace = workspace_name

        debug_prefix = _remote_debug_prefix(gui, workspace)
        command = (
            f"ssh -t -t {robot} '"
            "source ~/.bashrc; "
            f"export ROS_MASTER_URI=http://{ROSCORE_HOST}:11311/; "
            "source /opt/ros/noetic/setup.bash; "
            f"source ~/{workspace}/devel/setup.bash; "
            f"{debug_prefix}roslaunch mur_launch_hardware {robot}.launch{launch_suffix}; "
            "exec bash'"
        )

        proc = _popen_with_debug([
            "terminator",
            f"--title=Driver {robot}",
            "-x",
            f"{command}; exec bash"
        ], gui)
        if proc is not None:
            gui._driver_processes.append(proc)

        _ensure_driver_control_session(gui, robot, workspace)

        nodes = _collect_driver_node_names(robot, launch_cli_args)
        if nodes:
            node_map = getattr(gui, "_driver_nodes_by_robot", {})
            node_map[robot] = nodes
            gui._driver_nodes_by_robot = node_map
            persist_updates[robot] = nodes

    if persist_updates:
        cache = _load_driver_node_cache()
        cache.update({robot: set(nodes) for robot, nodes in persist_updates.items()})
        _save_driver_node_cache(cache)
        for robot in selected_robots:
            _deploy_gui_cache_to_robot(robot, workspace_name)

def quit_drivers(gui=None):
    """Terminates running driver sessions and uses the per-robot SSH channel for clean stops."""
    print("Stopping all driver sessions...")
    workspace_name = gui.get_workspace_name() if gui is not None else None
    processes = []
    if gui is not None and hasattr(gui, "_driver_processes"):
        processes = [p for p in gui._driver_processes if p and p.poll() is None]

    for proc in processes:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        except Exception as exc:
            rospy.logwarn(f"Failed to terminate driver terminal: {exc}")

    if gui is not None:
        gui._driver_processes = [p for p in getattr(gui, "_driver_processes", []) if p and p.poll() is None]

    # Ask the robots themselves to stop the driver launch files.
    robots = []
    if gui is not None:
        robots = gui.get_selected_robots()
        if not robots:
            robots = getattr(gui, "_driver_robots", [])

    if gui is not None and robots:
        node_map = getattr(gui, "_driver_nodes_by_robot", {})
        file_cache = _load_driver_node_cache()
        for robot, nodes in file_cache.items():
            node_map.setdefault(robot, set()).update(nodes)

        target_nodes = set()
        for robot in robots:
            target_nodes.update(node_map.get(robot, set()))

        if target_nodes and _kill_ros_nodes(target_nodes):
            print("Killed driver nodes derived from launch introspection.")
            # Keep cache entries so other operators can still reference the
            # launch configuration even after this GUI session shuts it down.
            combined_cache = {robot: set(nodes) for robot, nodes in node_map.items() if nodes}
            if file_cache:
                for robot, nodes in file_cache.items():
                    combined_cache.setdefault(robot, set()).update(nodes)
                _save_driver_node_cache(combined_cache)
                if robots and workspace_name:
                    for robot in robots:
                        _deploy_gui_cache_to_robot(robot, workspace_name)

            if combined_cache:
                gui._driver_nodes_by_robot = combined_cache
            elif hasattr(gui, "_driver_nodes_by_robot"):
                delattr(gui, "_driver_nodes_by_robot")

    robots_needing_fallback = robots
    if gui is not None and robots:
        sessions = getattr(gui, "_driver_control_sessions", {})
        robots_needing_fallback = []
        for robot in robots:
            session = sessions.get(robot)
            if session and session.is_alive():
                for cmd in _driver_kill_commands(robot):
                    session.send(cmd)
                session.close()
                sessions.pop(robot, None)
            else:
                robots_needing_fallback.append(robot)
        gui._driver_control_sessions = sessions

    if gui is not None and robots_needing_fallback:
        _run_remote_commands(
            gui,
            "Stopping remote driver launch",
            [_driver_kill_script],
            use_workspace_debug=True,
            target_robots=robots_needing_fallback,
        )

    # Best-effort fallback in case processes were started outside this session.
    try:
        _popen_with_debug("pkill -f 'terminator.*Driver'", gui, shell=True)
    except Exception as e:
        print(f"Error stopping processes: {e}")

def move_to_home_pose(gui, UR_prefix):
    """Moves the selected robots to the initial pose with the correct namespace and move_group_name."""
    selected_robots = gui.get_selected_robots()

    # Set move_group_name based on UR_prefix
    move_group_name = "UR_arm_l" if UR_prefix == "UR10_l" else "UR_arm_r"

    for robot in selected_robots:
        # Special case for mur620c with UR10_r
        if robot == "mur620c" and UR_prefix == "UR10_r":
            home_position = "Home_custom"
        elif robot in ["mur620a", "mur620b"]:
            home_position = "Home_custom"
        else:  # Default case for mur620c, mur620d
            home_position = "Home_custom"

        # ROS launch command with namespace
        command = f"ROS_NAMESPACE={robot} roslaunch ur_utilities move_UR_to_home_pose.launch tf_prefix:={robot} UR_prefix:={UR_prefix} home_position:={home_position} move_group_name:={move_group_name}"
        print(f"Executing: {command}")
        _popen_with_debug(command, gui, shell=True)


def _get_gui_component_name(gui) -> str:
    if gui is not None and hasattr(gui, "get_selected_component_name"):
        getter = getattr(gui, "get_selected_component_name")
        if callable(getter):
            try:
                name = getter()
                if isinstance(name, str) and name.strip():
                    return name.strip()
            except Exception as exc:
                print(f"Failed to fetch component selection from GUI: {exc}")
    return DEFAULT_COMPONENT_NAME


def _get_gui_component_transform(gui) -> dict:
    component_name = _get_gui_component_name(gui)
    ros_iface = getattr(gui, "ros_interface", None)
    if ros_iface is not None and hasattr(ros_iface, "get_component_transform"):
        try:
            return ros_iface.get_component_transform(component_name)
        except Exception as exc:
            print(f"Failed to fetch component transform for {component_name}: {exc}")
    return _load_component_transform_entry(component_name)

def parse_mir_path(gui):
    selected_robots = gui.get_selected_robots()
    if not selected_robots:
        print("No robot selected. Skipping MiR path parsing.")
        return

    component_name = _get_gui_component_name(gui)
    transform = _get_gui_component_transform(gui)
    component_arg = shlex.quote(component_name)
    transform_flags = " ".join(
        f"{key}:={value}" for key, value in transform.items()
    )
    command = f"roslaunch parse_mir_path parse_mir_path.launch component_name:={component_arg} {transform_flags}"

    _run_remote_commands(
        gui,
        "Parsing MiR path",
        [command],
        use_workspace_debug=True,
        target_robots=selected_robots,
    )

def parse_ur_path(gui):
    selected_robots = gui.get_selected_robots()
    if not selected_robots:
        print("No robot selected. Skipping UR path parsing.")
        return

    component_name = _get_gui_component_name(gui)
    transform = _get_gui_component_transform(gui)
    component_arg = shlex.quote(component_name)
    transform_flags = " ".join(
        f"{key}:={value}" for key, value in transform.items()
    )
    command = f"roslaunch parse_ur_path parse_ur_path.launch component_name:={component_arg} {transform_flags}"

    _run_remote_commands(
        gui,
        "Parsing UR path",
        [command],
        use_workspace_debug=True,
        target_robots=selected_robots,
    )


def _persist_gui_index_setting(gui):
    if gui is None:
        return
    ros_iface = getattr(gui, "ros_interface", None)
    idx_spin = getattr(gui, "idx_spin", None)
    if ros_iface is None or idx_spin is None:
        return
    try:
        ros_iface.persist_path_index(idx_spin.value())
    except Exception as exc:
        print(f"Failed to persist path index default: {exc}")


def move_mir_to_start_pose(gui):
    """Moves the MIR robot to the start pose."""
    selected_robots = gui.get_selected_robots()
    if selected_robots is None:
        print("MIR robot not selected. Skipping move to start pose.")
        return

    # Ensure only one MIR robot is selected
    if len(selected_robots) != 1:
        print("Please select only the MIR robot to move to the start pose.")
        return
    _persist_gui_index_setting(gui)
    command = f"roslaunch move_mir_to_start_pose move_mir_to_start_pose.launch robot_name:={selected_robots[0]} initial_path_index:={gui.idx_spin.value()}"
    rospy.loginfo(f"Executing: {command}")
    _popen_with_debug(command, gui, shell=True)

def move_ur_to_start_pose(gui):
    """Moves the UR robot to the start pose."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()

    if not selected_robots or not selected_urs:
        print("No robots or URs selected. Skipping move to start pose.")
        return

    # Ensure only one UR and one mir robot is selected
    if len(selected_robots) != 1 or len(selected_urs) != 1:
        print("Please select exactly one UR and one MIR robot to move to the start pose.")
        return

    # ToDo: to risky to use for now
    if selected_urs[0] == "UR10_l":
        move_group_name = "UR_arm_l"
        planning_group = "UR10_l/tool0"
    else:
        move_group_name = "UR_arm_r"
        planning_group = "UR10_r/tool0"

    spray_distance = gui.get_spray_distance()

    _persist_gui_index_setting(gui)
    for robot in selected_robots:
        for ur in selected_urs:
            command = f"roslaunch move_ur_to_start_pose move_ur_to_start_pose.launch robot_name:={robot} initial_path_index:={gui.idx_spin.value()} spray_distance:={spray_distance}"
            print(f"Executing: {command}")
            _popen_with_debug(command, gui, shell=True)

def mir_follow_trajectory(gui):
    """Moves the MIR robot along a predefined trajectory."""
    selected_robots = gui.get_selected_robots()
    if not selected_robots:
        print("No MIR robot selected. Skipping follow trajectory.")
        return
    # Ensure only one MIR robot is selected
    if len(selected_robots) != 1:
        print("Please select only the MIR robot to follow the trajectory.")
        return
    _persist_gui_index_setting(gui)
    initial_idx = gui.idx_spin.value()
    _run_remote_commands(
        gui,
        "Launching MiR trajectory follower",
        [lambda robot: f"roslaunch mir_trajectory_follower mir_trajectory_follower.launch robot_name:={robot} initial_path_index:={initial_idx}"],
        use_workspace_debug=True,
        target_robots=selected_robots,
    )

def increment_path_index(gui):
    """Increments the path index for the MIR robot."""
    selected_robots = gui.get_selected_robots()
    if not selected_robots:
        print("No robot selected. Skipping path index increment.")
        return

    initial_idx = gui.idx_spin.value()
    _run_remote_commands(
        gui,
        "Incrementing path index",
        [f"roslaunch print_gui increment_path_index.launch initial_path_index:={initial_idx}"],
        use_workspace_debug=True,
        target_robots=selected_robots,
    )

    # if not rospy.core.is_initialized():
    #     rospy.init_node("additive_manufacturing_gui", anonymous=True, disable_signals=True)
    # rospy.Subscriber('/path_index', Int32, gui.ros_interface._path_idx_cb, queue_size=10)


def stop_mir_motion(gui):
    """Stops MiR motion on the selected robots via SSH."""
    _run_remote_commands(
        gui,
        "Stopping MiR motion",
        [
            "pkill -f mir_trajectory_follower || true",
            "pkill -f increment_path_index || true",
        ],
    )


def stop_ur_motion(gui):
    """Stops UR motion on the selected robots via SSH."""
    _run_remote_commands(
        gui,
        "Stopping UR motion",
        [
            "pkill -f 'ur_direction_controller|orthogonal_error_correction|move_ur_to_start_pose|ur_vel_induced_by_mir|world_twist_in_mir|twist_combiner|ur_yaw_controller' || true",
        ],
    )


def stop_all_but_drivers(gui):
    """Stops common non-driver processes on the selected robots via SSH."""
    if gui is None:
        return

    selected_robots = gui.get_selected_robots()
    if not selected_robots:
        print("No robots selected. Skipping non-driver stop.")
        return

    workspace = gui.get_workspace_name()
    commands = _stop_non_driver_commands()
    sessions = getattr(gui, "_driver_control_sessions", {})
    robots_needing_fallback = []

    for robot in selected_robots:
        session = sessions.get(robot)
        if session is None or not session.is_alive():
            session = _ensure_driver_control_session(gui, robot, workspace)
        if session and session.is_alive():
            for cmd in commands:
                session.send(cmd)
        else:
            robots_needing_fallback.append(robot)

    gui._driver_control_sessions = sessions

    if robots_needing_fallback:
        _run_remote_commands(
            gui,
            "Stopping non-driver processes",
            [_stop_non_driver_script],
            use_workspace_debug=True,
            target_robots=robots_needing_fallback,
        )

def ur_follow_trajectory(gui, ur_follow_settings: dict):
    """Moves the UR robot along a predefined trajectory."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    metric = ur_follow_settings.get("idx_metric")
    threshold = ur_follow_settings.get("threshold")
    initial_path_index = gui.idx_spin.value()
    spray_distance = gui.get_spray_distance()
    rospy.loginfo(f"Selected metric: {metric}")

    if not selected_robots or not selected_urs:
        print("No UR robot selected. Skipping follow trajectory.")
        return

    # Ensure only one UR and one MIR robot is selected
    if len(selected_robots) != 1 or len(selected_urs) != 1:
        print("Please select exactly one UR and one MIR robot to follow the trajectory.")
        return

    ur = selected_urs[0]

    cmd_template = (
        "roslaunch print_hw complete_ur_trajectory_follower_ff_only.launch "
        "robot_name:={robot} prefix_ur:={ur}/ metric:='{metric}' "
        "threshold:={threshold} initial_path_index:={initial_path_index} "
        "nozzle_height_default:={spray_distance}"
    )

    _run_remote_commands(
        gui,
        "Launching UR trajectory follower",
        [
            lambda robot, template=cmd_template: template.format(
                robot=robot,
                ur=ur,
                metric=metric,
                threshold=threshold,
                initial_path_index=initial_path_index,
                spray_distance=spray_distance,
            )
        ],
        use_workspace_debug=True,
        target_robots=selected_robots,
    )

def stop_idx_advancer(gui):
    """Stops the path index advancer nodes on the selected robots via SSH."""
    _run_remote_commands(
        gui,
        "Stopping path index advancer",
        [
            "pkill -f path_index_advancer || true",
            "pkill -f increment_path_index || true",
        ],
    )


def target_broadcaster(gui):
    """Broadcasts Target Poses."""
    command = f"roslaunch print_hw target_broadcaster.launch initial_path_index:={gui.idx_spin.value()}"
    print(f"Executing: {command}")
    _popen_with_debug(command, gui, shell=True)