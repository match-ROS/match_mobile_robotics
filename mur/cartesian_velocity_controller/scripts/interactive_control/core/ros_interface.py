"""
ROS Interface - Handles ROS initialization, TF, and communication.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from typing import Optional, Dict, Any, Tuple, List, Sequence

from ..config import (
    DEFAULT_CONTROLLER_NODE,
    DEFAULT_GLOBAL_FRAME,
    DEFAULT_EE_FRAME,
    PUBLISHER_CONNECTION_TIMEOUT,
    TF_LOOKUP_TIMEOUT,
    EE_POSE_CACHE_DURATION,
)


class ROSInterface:
    """
    Manages ROS communication and TF lookups.
    
    Handles:
    - ROS node initialization
    - TF buffer and listener
    - Target pose publishing with connection verification
    - Frame configuration from parameter server
    """
    
    def __init__(self,
                 controller_node_name: str = DEFAULT_CONTROLLER_NODE,
                 global_frame: str = DEFAULT_GLOBAL_FRAME,
                 ee_frame: str = DEFAULT_EE_FRAME,
                 target_pose_topic: Optional[str] = None):
        """
        Initialize the ROS interface.
        
        Args:
            controller_node_name: Name of the velocity controller node
            global_frame: Global reference frame (can be overridden from params)
            ee_frame: End effector frame name
        """
        self.controller_node_name = controller_node_name
        self._global_frame = global_frame
        self.ee_frame = ee_frame
        self.target_pose_topic = target_pose_topic
        
        # TF components
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        
        # Publisher
        self.target_pose_pub: Optional[rospy.Publisher] = None
        self._target_pose_topic_resolved: Optional[str] = None

        # UI/performance helpers
        self._last_tf_pair: Optional[Tuple[str, str]] = None
        self._ee_pose_cache: Optional[Dict[str, Any]] = None
        self._ee_pose_cache_stamp = rospy.Time(0)
        
        self._initialized = False
    
    @property
    def global_frame(self) -> str:
        """Get the current global frame."""
        return self._global_frame
    
    def initialize(self, node_name: str = 'robot_interactive_control') -> bool:
        """
        Initialize ROS node and components.
        
        Args:
            node_name: Name for this ROS node
            
        Returns:
            True if successful
        """
        try:
            # Initialize ROS node once (allow multiple controllers in one process)
            already_initialized = False
            try:
                already_initialized = bool(rospy.core.is_initialized())
            except Exception:
                try:
                    _ = rospy.get_name()
                    already_initialized = True
                except Exception:
                    already_initialized = False

            if not already_initialized:
                rospy.init_node(node_name, anonymous=True)

            # Resolve controller node name (may be relative)
            self.controller_node_name = rospy.resolve_name(self.controller_node_name)
            
            # Read global_frame from parameter server (FIX: use controller's frame)
            self._global_frame = rospy.get_param(
                f'{self.controller_node_name}/global_frame',
                self._global_frame
            )
            rospy.loginfo(f"Using global_frame: {self._global_frame}")
            
            # Initialize TF2
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            
            # Create publisher for target pose
            resolved_topic = None
            if self.target_pose_topic:
                resolved_topic = rospy.resolve_name(self.target_pose_topic)
            else:
                # Default: publish in the controller node namespace (e.g. /mur620/cartesian_velocity_controller_l/target_pose)
                resolved_topic = f"{self.controller_node_name}/target_pose"
            self._target_pose_topic_resolved = resolved_topic

            self.target_pose_pub = rospy.Publisher(
                resolved_topic,
                PoseStamped,
                queue_size=1
            )
            
            # Wait for TF to be available
            rospy.sleep(0.5)
            
            self._initialized = True
            return True
            
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to initialize ROS: {e}")
            return False
    
    def is_initialized(self) -> bool:
        """Check if the interface is initialized."""
        return self._initialized

    def _infer_tf_prefix(self) -> Optional[str]:
        """
        Infer a TF prefix / robot namespace from the controller node name.

        Example:
          controller_node_name = "/mur620/cartesian_velocity_controller_l" -> "mur620"
        """
        try:
            name = str(self.controller_node_name or "")
            if not name.startswith("/"):
                return None
            parts = [p for p in name.split("/") if p]
            if not parts:
                return None
            return parts[0]
        except Exception:
            return None

    @staticmethod
    def _prefixed(prefix: str, frame: str) -> str:
        """Prefix TF frame if not already prefixed."""
        if not prefix:
            return frame
        if frame.startswith(f"{prefix}/"):
            return frame
        return f"{prefix}/{frame}"

    def _candidate_frame_pairs(self) -> List[Tuple[str, str]]:
        """
        Build a small set of fallback (global, ee) pairs.

        Some setups publish TF frames with a robot prefix (e.g. "mur620/base_link"),
        while configs/CLI may use unprefixed frames (e.g. "base_link").
        """
        base_global = self._global_frame
        base_ee = self.ee_frame
        prefix = self._infer_tf_prefix()
        if not prefix:
            return [(base_global, base_ee)]

        pref_global = self._prefixed(prefix, base_global)
        pref_ee = self._prefixed(prefix, base_ee)

        # Try most likely combos first
        pairs: List[Tuple[str, str]] = [
            (base_global, base_ee),
            (base_global, pref_ee),
            (pref_global, base_ee),
            (pref_global, pref_ee),
        ]

        # De-duplicate while preserving order
        seen = set()
        out: List[Tuple[str, str]] = []
        for g, e in pairs:
            key = (g, e)
            if key in seen:
                continue
            seen.add(key)
            out.append(key)
        return out
    
    def get_current_ee_pose(self) -> Optional[Dict[str, Any]]:
        """
        Get the current end effector pose in the global frame.
        
        Returns:
            Dict with 'position' [x,y,z] and 'orientation' [qx,qy,qz,qw],
            or None if lookup fails
        """
        if not self._initialized or self.tf_buffer is None:
            return None
        
        # Cache to keep the terminal UI snappy (menus call this often)
        try:
            cache_s = float(EE_POSE_CACHE_DURATION)
        except Exception:
            cache_s = 0.0
        if cache_s > 0.0 and self._ee_pose_cache is not None:
            try:
                if (rospy.Time.now() - self._ee_pose_cache_stamp).to_sec() <= cache_s:
                    return dict(self._ee_pose_cache)
            except Exception:
                pass

        # Build candidate pairs, trying the last successful pair first
        pairs = self._candidate_frame_pairs()
        if self._last_tf_pair in pairs:
            pairs = [self._last_tf_pair] + [p for p in pairs if p != self._last_tf_pair]

        try:
            per_pair_timeout_s = max(0.0, float(TF_LOOKUP_TIMEOUT))
        except Exception:
            per_pair_timeout_s = 0.05

        last_err: Optional[Exception] = None
        for global_frame, ee_frame in pairs:
            try:
                # Avoid blocking the UI: first check availability with a small timeout.
                if not self.tf_buffer.can_transform(
                    global_frame,
                    ee_frame,
                    rospy.Time(0),
                    rospy.Duration(per_pair_timeout_s),
                ):
                    continue

                # Now do the lookup without an additional blocking timeout.
                transform = self.tf_buffer.lookup_transform(global_frame, ee_frame, rospy.Time(0))

                pos = transform.transform.translation
                rot = transform.transform.rotation
                pose = {
                    "frame_id": global_frame,
                    "ee_frame": ee_frame,
                    "position": [pos.x, pos.y, pos.z],
                    "orientation": [rot.x, rot.y, rot.z, rot.w]
                }
                self._last_tf_pair = (global_frame, ee_frame)
                self._ee_pose_cache = dict(pose)
                self._ee_pose_cache_stamp = rospy.Time.now()
                return pose

            except tf2_ros.TransformException as e:
                last_err = e
                continue

        if last_err is not None:
            # Throttle to avoid slowing the UI and spamming logs
            try:
                rospy.logwarn_throttle(2.0, f"Cannot get EE pose: {last_err}")
            except Exception:
                rospy.logwarn(f"Cannot get EE pose: {last_err}")
        return None
    
    def publish_target_pose(self, pose_data: Dict[str, Any]) -> bool:
        """
        Publish a target pose to the velocity controller.
        
        Includes connection verification to ensure the controller receives the message.
        
        Args:
            pose_data: Dict with 'position' [x,y,z] and 'orientation' [qx,qy,qz,qw]
            
        Returns:
            True if published successfully
        """
        if not self._initialized or self.target_pose_pub is None:
            rospy.logerr("ROS interface not initialized")
            return False
        
        # FIX: Wait for subscriber connection before publishing
        timeout = rospy.Time.now() + rospy.Duration(PUBLISHER_CONNECTION_TIMEOUT)
        while self.target_pose_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout:
                rospy.logwarn(
                    f"No subscribers connected to {self._target_pose_topic_resolved}. "
                    "Is the cartesian velocity controller running?"
                )
                return False
            if rospy.is_shutdown():
                return False
            rospy.sleep(0.1)
        
        # Build the message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        # Prefer explicit frame_id from pose data; otherwise use controller global_frame
        pose_msg.header.frame_id = pose_data.get("frame_id") or self._global_frame
        
        pose_msg.pose.position.x = pose_data["position"][0]
        pose_msg.pose.position.y = pose_data["position"][1]
        pose_msg.pose.position.z = pose_data["position"][2]
        
        pose_msg.pose.orientation.x = pose_data["orientation"][0]
        pose_msg.pose.orientation.y = pose_data["orientation"][1]
        pose_msg.pose.orientation.z = pose_data["orientation"][2]
        pose_msg.pose.orientation.w = pose_data["orientation"][3]
        
        self.target_pose_pub.publish(pose_msg)
        
        rospy.loginfo(
            f"Published target pose: position=[{pose_data['position'][0]:.3f}, "
            f"{pose_data['position'][1]:.3f}, {pose_data['position'][2]:.3f}]"
        )
        
        return True
    
    def has_subscriber(self) -> bool:
        """Check if the target_pose topic has subscribers."""
        if self.target_pose_pub is None:
            return False
        return self.target_pose_pub.get_num_connections() > 0
    
    def refresh_frame_config(self):
        """Refresh the global frame configuration from parameter server."""
        try:
            new_frame = rospy.get_param(
                f'{self.controller_node_name}/global_frame',
                self._global_frame
            )
            if new_frame != self._global_frame:
                rospy.loginfo(f"Updated global_frame: {self._global_frame} -> {new_frame}")
                self._global_frame = new_frame
        except Exception as e:
            rospy.logwarn(f"Could not refresh frame config: {e}")

    # =========================================================================
    # Direct velocity publish (debug utilities)
    # =========================================================================

    @staticmethod
    def _parent_namespace(resolved_node_name: str) -> str:
        """
        Return the parent namespace of a resolved ROS node name.

        Examples:
          "/mur620/cartesian_velocity_controller_l" -> "/mur620"
          "/a/b/c" -> "/a/b"
        """
        try:
            name = str(resolved_node_name or "")
            if not name.startswith("/"):
                return "/"
            parts = [p for p in name.split("/") if p]
            if len(parts) <= 1:
                return "/"
            return "/" + "/".join(parts[:-1])
        except Exception:
            return "/"

    def _resolve_topic_in_controller_ns(self, topic: Optional[str]) -> Optional[str]:
        """
        Resolve a topic name relative to the controller node namespace.

        The cartesian controller often stores relative topic names like:
          "joint_group_vel_controller_l/unsafe/command"
        which should resolve under the controller's parent namespace.
        """
        if not topic:
            return None
        t = str(topic)
        if t.startswith("/"):
            return t
        parent = self._parent_namespace(self.controller_node_name)
        if parent == "/":
            return "/" + t.lstrip("/")
        return parent.rstrip("/") + "/" + t.lstrip("/")

    def resolve_velocity_command_topic(self, override: Optional[str] = None) -> Optional[str]:
        """
        Resolve the joint velocity command topic used by the controller.

        - If override is provided, it is used.
        - Otherwise reads "<controller_node>/velocity_command_topic" from param server.
        - Relative names are resolved in the controller namespace (not this script node namespace).
        """
        if not self._initialized:
            return None
        raw = override
        if raw is None:
            try:
                raw = rospy.get_param(f"{self.controller_node_name}/velocity_command_topic", None)
            except Exception:
                raw = None
        return self._resolve_topic_in_controller_ns(raw)

    def publish_direct_joint_velocity(
        self,
        joint_velocities: Sequence[float],
        *,
        velocity_command_topic_override: Optional[str] = None,
        rate_hz: float = 50.0,
        duration_s: float = 1.0,
        wait_for_subscriber_timeout_s: float = PUBLISHER_CONNECTION_TIMEOUT,
        stop_at_end: bool = True,
        stop_event=None,
    ) -> bool:
        """
        Publish joint velocity commands directly (std_msgs/Float64MultiArray).

        Modes:
        - If stop_event is provided: publish until stop_event.is_set()
        - Else: publish for duration_s seconds (duration_s <= 0 publishes once)
        """
        if not self._initialized:
            rospy.logerr("ROS interface not initialized")
            return False

        from std_msgs.msg import Float64MultiArray

        topic = self.resolve_velocity_command_topic(velocity_command_topic_override)
        if not topic:
            rospy.logerr("Could not resolve velocity command topic")
            return False

        pub = rospy.Publisher(topic, Float64MultiArray, queue_size=1)

        # Wait for at least one subscriber (typically the joint velocity controller)
        wait_timeout = max(0.0, float(wait_for_subscriber_timeout_s or 0.0))
        if wait_timeout > 0:
            timeout_t = rospy.Time.now() + rospy.Duration(wait_timeout)
            while pub.get_num_connections() == 0:
                if rospy.is_shutdown():
                    return False
                if rospy.Time.now() > timeout_t:
                    rospy.logerr(f"No subscribers on {topic} (timeout). Not publishing.")
                    return False
                rospy.sleep(0.05)

        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_velocities]

        # Publish once if duration <= 0 and no stop_event
        if stop_event is None and float(duration_s) <= 0.0:
            pub.publish(msg)
            if stop_at_end and not rospy.is_shutdown():
                zero = Float64MultiArray()
                zero.data = [0.0 for _ in msg.data]
                pub.publish(zero)
            return True

        rate = rospy.Rate(max(0.1, float(rate_hz)))
        start = rospy.Time.now()

        try:
            while not rospy.is_shutdown():
                if stop_event is not None:
                    try:
                        if stop_event.is_set():
                            break
                    except Exception:
                        # If stop_event isn't a threading.Event-like object, ignore it
                        pass
                else:
                    if float(duration_s) > 0 and (rospy.Time.now() - start).to_sec() >= float(duration_s):
                        break

                pub.publish(msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            return False
        finally:
            if stop_at_end and not rospy.is_shutdown():
                zero = Float64MultiArray()
                zero.data = [0.0 for _ in msg.data]
                pub.publish(zero)
                rospy.sleep(0.05)

        return True

    # =========================================================================
    # Controller joint names (UI helpers)
    # =========================================================================

    def _topic_parent_ns(self, resolved_topic: str) -> str:
        """Return the parent namespace of a resolved topic (e.g. '/a/b/c' -> '/a/b')."""
        try:
            t = str(resolved_topic or "")
            if not t.startswith("/"):
                return "/"
            parts = [p for p in t.split("/") if p]
            if len(parts) <= 1:
                return "/"
            return "/" + "/".join(parts[:-1])
        except Exception:
            return "/"

    def resolve_velocity_controller_ns(self, override_velocity_command_topic: Optional[str] = None) -> Optional[str]:
        """
        Resolve the ros_control velocity controller namespace behind the command topic.

        Example:
          velocity command topic = "/mur620/joint_group_vel_controller_l/unsafe/command"
          -> controller ns = "/mur620/joint_group_vel_controller_l/unsafe"
        """
        topic = self.resolve_velocity_command_topic(override_velocity_command_topic)
        if not topic:
            return None
        t = str(topic)
        if t.endswith("/command"):
            return t[: -len("/command")]
        return self._topic_parent_ns(t)

    def get_velocity_controller_joint_list(self, timeout_s: float = 0.0) -> List[str]:
        """
        Return the joint list configured in the downstream JointGroupVelocityController.

        This reads the `joints` parameter from the ros_control controller namespace.
        """
        if not self._initialized:
            return []

        # Optionally wait a little for params to appear (controller spawner delay)
        try:
            wait_s = max(0.0, float(timeout_s))
        except Exception:
            wait_s = 0.0

        ns = self.resolve_velocity_controller_ns()
        if not ns:
            return []

        param_name = f"{ns}/joints"
        end_t = rospy.Time.now() + rospy.Duration(wait_s)
        while True:
            try:
                if rospy.has_param(param_name):
                    joints = rospy.get_param(param_name, [])
                    if isinstance(joints, (list, tuple)) and joints:
                        return [str(j) for j in joints]
                    return []
            except Exception:
                pass
            if wait_s <= 0.0:
                break
            if rospy.is_shutdown() or rospy.Time.now() > end_t:
                break
            rospy.sleep(0.05)

        return []

    def resolve_joint_state_topic(self, override: Optional[str] = None) -> Optional[str]:
        """
        Resolve the joint_state topic used by the controller.
        Falls back to "/joint_states" if not set.
        """
        if not self._initialized:
            return None
        raw = override
        if raw is None:
            try:
                raw = rospy.get_param(f"{self.controller_node_name}/joint_state_topic", None)
            except Exception:
                raw = None
        raw = raw or "/joint_states"
        return self._resolve_topic_in_controller_ns(raw)

    def resolve_joint_velocity_feedback_topic(self) -> Optional[str]:
        """
        Resolve the joint velocity feedback topic published by the controller.

        The controller publishes it in its private namespace (pnh), so the topic is:
          "<controller_node>/joint_velocity_feedback"
        e.g. "/mur620/cartesian_velocity_controller_l/joint_velocity_feedback"
        """
        if not self._initialized:
            return None
        return f"{self.controller_node_name}/joint_velocity_feedback"

    def get_controller_joint_names(self, timeout_s: float = 1.0) -> List[str]:
        """
        Best-effort joint name list in the same order expected by the controller.

        Priority:
        1) cartesian_velocity_controller/JointVelocityFeedback (preferred: controller order)
        2) ros_control JointGroupVelocityController `joints` param (correct order for command vector)
        3) sensor_msgs/JointState (fallback: may include extra joints / different order) - DISABLED by default
        """
        if not self._initialized:
            return []

        # 1) Preferred: joint_velocity_feedback (contains joint_names in controller order)
        try:
            topic = self.resolve_joint_velocity_feedback_topic()
            if topic:
                from cartesian_velocity_controller.msg import JointVelocityFeedback  # type: ignore
                msg = rospy.wait_for_message(topic, JointVelocityFeedback, timeout=max(0.01, float(timeout_s)))
                names = list(getattr(msg, "joint_names", []) or [])
                if names:
                    return [str(n) for n in names]
        except Exception:
            pass

        # 2) ros_control controller param (safe for sizing commands)
        joints = self.get_velocity_controller_joint_list(timeout_s=float(timeout_s))
        if joints:
            return joints

        # NOTE: we intentionally do NOT fall back to /joint_states here, because it may include
        # additional joints (e.g., mobile base, grippers, lifts) leading to wrong command size.

        return []

    def get_command_joint_count(self, timeout_s: float = 1.0) -> int:
        """Return the expected command vector size for direct joint velocity publishing."""
        names = self.get_controller_joint_names(timeout_s=float(timeout_s))
        return int(len(names or []))
