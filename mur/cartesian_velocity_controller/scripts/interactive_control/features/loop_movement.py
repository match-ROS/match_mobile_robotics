"""
Loop Movement - Handles cyclic movement through a sequence of poses.
"""

import math
import time
import threading
import numpy as np
import rospy
from typing import Dict, List, Callable, Optional, Any

from ..config import LOOP_CHECK_INTERVAL


class LoopMovement:
    """
    Manages cyclic movement through a sequence of saved poses.
    
    Features:
    - Configurable pose sequence
    - Position and orientation tolerance checking
    - Dwell time at each pose
    - Thread-safe start/stop
    """
    
    def __init__(self,
                 get_pose_callback: Callable[[str], Optional[Dict]],
                 send_pose_callback: Callable[[Dict], bool],
                 get_ee_pose_callback: Callable[[], Optional[Dict]],
                 get_tolerance_callback: Callable[[], tuple],
                 get_dwell_callback: Callable[[], float],
                 get_loop_poses_callback: Callable[[], List[str]]):
        """
        Initialize the loop movement controller.
        
        Args:
            get_pose_callback: Function to get pose data by name
            send_pose_callback: Function to send a pose to the robot
            get_ee_pose_callback: Function to get current EE pose
            get_tolerance_callback: Function returning (pos_tol, orient_tol)
            get_dwell_callback: Function returning dwell time
            get_loop_poses_callback: Function returning list of pose names
        """
        self._get_pose = get_pose_callback
        self._send_pose = send_pose_callback
        self._get_ee_pose = get_ee_pose_callback
        self._get_tolerances = get_tolerance_callback
        self._get_dwell = get_dwell_callback
        self._get_loop_poses = get_loop_poses_callback
        
        # State
        self._enabled = False
        self._current_pose_idx = 0
        
        # Threading
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
    
    # =========================================================================
    # Properties
    # =========================================================================
    
    @property
    def enabled(self) -> bool:
        """Check if loop movement is currently running."""
        with self._lock:
            return self._enabled
    
    @property
    def current_pose_index(self) -> int:
        """Get the current pose index in the loop."""
        with self._lock:
            return self._current_pose_idx
    
    # =========================================================================
    # Error Computation
    # =========================================================================
    
    @staticmethod
    def compute_position_error(current_pos: List[float], target_pos: List[float]) -> float:
        """
        Compute Euclidean distance between positions.
        
        Args:
            current_pos: Current position [x, y, z]
            target_pos: Target position [x, y, z]
            
        Returns:
            Distance in meters
        """
        return math.sqrt(
            (current_pos[0] - target_pos[0]) ** 2 +
            (current_pos[1] - target_pos[1]) ** 2 +
            (current_pos[2] - target_pos[2]) ** 2
        )
    
    @staticmethod
    def compute_orientation_error(current_quat: List[float], target_quat: List[float]) -> float:
        """
        Compute angular error between quaternions.
        
        The error is the minimum rotation angle needed to go from
        current to target orientation.
        
        Args:
            current_quat: Current quaternion [qx, qy, qz, qw]
            target_quat: Target quaternion [qx, qy, qz, qw]
            
        Returns:
            Error in radians
        """
        q1 = np.array(current_quat)
        q2 = np.array(target_quat)
        
        # Normalize
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Dot product
        dot = np.abs(np.dot(q1, q2))
        
        # Clamp for numerical stability
        dot = min(1.0, max(-1.0, dot))
        
        # Angle is 2 * arccos(|dot|)
        return 2.0 * math.acos(dot)
    
    def is_target_reached(self, target_pose: Dict[str, Any]) -> bool:
        """
        Check if the target pose has been reached within tolerances.
        
        Args:
            target_pose: Target pose with 'position' and 'orientation'
            
        Returns:
            True if both position and orientation tolerances are satisfied
        """
        current_pose = self._get_ee_pose()
        if current_pose is None:
            return False
        
        pos_tol, orient_tol = self._get_tolerances()
        
        pos_error = self.compute_position_error(
            current_pose['position'],
            target_pose['position']
        )
        
        orient_error = self.compute_orientation_error(
            current_pose['orientation'],
            target_pose['orientation']
        )
        
        return pos_error < pos_tol and orient_error < orient_tol
    
    def get_current_errors(self, target_pose: Dict[str, Any]) -> Optional[tuple]:
        """
        Get current position and orientation errors.
        
        Args:
            target_pose: Target pose
            
        Returns:
            Tuple of (pos_error, orient_error) or None if EE pose unavailable
        """
        current_pose = self._get_ee_pose()
        if current_pose is None:
            return None
        
        pos_error = self.compute_position_error(
            current_pose['position'],
            target_pose['position']
        )
        
        orient_error = self.compute_orientation_error(
            current_pose['orientation'],
            target_pose['orientation']
        )
        
        return pos_error, orient_error
    
    # =========================================================================
    # Loop Control
    # =========================================================================
    
    def start(self) -> bool:
        """
        Start the loop movement.
        
        Returns:
            True if started successfully
        """
        with self._lock:
            if self._enabled:
                rospy.logwarn("Loop already running")
                return False
        
        # Validate poses
        loop_poses = self._get_loop_poses()
        valid_poses = [p for p in loop_poses if self._get_pose(p) is not None]
        
        if not valid_poses:
            rospy.logerr("No valid poses configured for loop")
            return False
        
        # Start thread
        with self._lock:
            self._enabled = True
            self._current_pose_idx = 0
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop_thread, daemon=True)
        self._thread.start()
        
        rospy.loginfo(f"Loop movement started with {len(valid_poses)} poses")
        return True
    
    def stop(self) -> bool:
        """
        Stop the loop movement.
        
        Returns:
            True if stopped successfully
        """
        with self._lock:
            if not self._enabled:
                rospy.logwarn("Loop not running")
                return False
        
        self._stop_event.set()
        
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        
        with self._lock:
            self._enabled = False
        
        rospy.loginfo("Loop movement stopped")
        return True
    
    def _loop_thread(self):
        """Main loop thread function."""
        rospy.loginfo("Loop thread started")
        
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            # Get valid poses
            loop_poses = self._get_loop_poses()
            valid_poses = [p for p in loop_poses if self._get_pose(p) is not None]
            
            if not valid_poses:
                rospy.logwarn("No valid poses, stopping loop")
                break
            
            # Get current pose
            with self._lock:
                idx = self._current_pose_idx % len(valid_poses)
            
            pose_name = valid_poses[idx]
            target_pose = self._get_pose(pose_name)
            
            if target_pose is None:
                rospy.logwarn(f"Pose '{pose_name}' not found, skipping")
                with self._lock:
                    self._current_pose_idx = (idx + 1) % len(valid_poses)
                continue
            
            rospy.loginfo(f"Loop: moving to '{pose_name}' ({idx + 1}/{len(valid_poses)})")
            
            # Send pose
            self._send_pose(target_pose)
            
            # Wait for target to be reached
            reached = False
            while not self._stop_event.is_set() and not rospy.is_shutdown():
                if self.is_target_reached(target_pose):
                    reached = True
                    break
                time.sleep(LOOP_CHECK_INTERVAL)
            
            if self._stop_event.is_set():
                break
            
            if reached:
                rospy.loginfo(f"Loop: pose '{pose_name}' reached")
                
                # Dwell time
                dwell = self._get_dwell()
                if dwell > 0:
                    start_time = time.time()
                    while time.time() - start_time < dwell:
                        if self._stop_event.is_set():
                            break
                        time.sleep(LOOP_CHECK_INTERVAL)
                
                if self._stop_event.is_set():
                    break
                
                # Move to next pose
                with self._lock:
                    self._current_pose_idx = (idx + 1) % len(valid_poses)
        
        rospy.loginfo("Loop thread exiting")
        with self._lock:
            self._enabled = False
    
    def get_current_pose_name(self) -> Optional[str]:
        """
        Get the name of the current target pose in the loop.
        
        Returns:
            Pose name or None if loop not active
        """
        if not self._enabled:
            return None
        
        loop_poses = self._get_loop_poses()
        valid_poses = [p for p in loop_poses if self._get_pose(p) is not None]
        
        if not valid_poses:
            return None
        
        with self._lock:
            idx = self._current_pose_idx % len(valid_poses)
        
        return valid_poses[idx]

