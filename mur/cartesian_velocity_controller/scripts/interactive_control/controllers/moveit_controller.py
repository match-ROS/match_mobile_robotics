"""
MoveIt Pose Controller - Sends poses via MoveIt planning and execution.
"""

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from typing import Dict, Any, Optional

# MoveIt is optional
try:
    import moveit_commander
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class MoveItPoseController:
    """
    Sends target poses to the robot via MoveIt motion planning.
    
    Provides trajectory planning and execution for goal poses.
    """
    
    def __init__(self, move_group_name: str = "manipulator"):
        """
        Initialize the MoveIt pose controller.
        
        Args:
            move_group_name: Name of the MoveIt move group
        """
        self.move_group_name = move_group_name
        self._move_group = None
        self._robot = None
        self._initialized = False
        
        if not MOVEIT_AVAILABLE:
            rospy.logwarn("moveit_commander not available. MoveIt functionality disabled.")
    
    @staticmethod
    def is_available() -> bool:
        """Check if MoveIt is available."""
        return MOVEIT_AVAILABLE
    
    def initialize(self) -> bool:
        """
        Initialize MoveIt components.
        
        Returns:
            True if successful
        """
        if not MOVEIT_AVAILABLE:
            return False
        
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self._robot = moveit_commander.RobotCommander()
            self._move_group = moveit_commander.MoveGroupCommander(self.move_group_name)
            
            # Configure planning
            self._move_group.set_planning_time(5.0)
            self._move_group.set_num_planning_attempts(5)
            
            self._initialized = True
            rospy.loginfo(f"MoveIt initialized with group '{self.move_group_name}'")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveIt: {e}")
            self._move_group = None
            return False
    
    def is_initialized(self) -> bool:
        """Check if MoveIt is initialized."""
        return self._initialized and self._move_group is not None
    
    def send_pose(self, pose_data: Dict[str, Any], global_frame: str) -> bool:
        """
        Send a target pose via MoveIt planning and execution.
        
        Args:
            pose_data: Dict with 'position' [x,y,z] and 'orientation' [qx,qy,qz,qw]
            global_frame: Reference frame for the pose
            
        Returns:
            True if motion was successful
        """
        if not self._initialized or self._move_group is None:
            rospy.logerr("MoveIt not initialized")
            return False
        
        try:
            pose_target = PoseStamped()
            # Prefer explicit frame_id from pose data; fallback to provided global_frame
            pose_target.header.frame_id = pose_data.get("frame_id") or global_frame
            pose_target.header.stamp = rospy.Time.now()
            
            pose_target.pose.position.x = pose_data["position"][0]
            pose_target.pose.position.y = pose_data["position"][1]
            pose_target.pose.position.z = pose_data["position"][2]
            
            pose_target.pose.orientation.x = pose_data["orientation"][0]
            pose_target.pose.orientation.y = pose_data["orientation"][1]
            pose_target.pose.orientation.z = pose_data["orientation"][2]
            pose_target.pose.orientation.w = pose_data["orientation"][3]
            
            self._move_group.set_pose_target(pose_target)
            
            rospy.loginfo("MoveIt: Planning trajectory...")
            success = self._move_group.go(wait=True)
            
            # Clean up
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            
            if success:
                rospy.loginfo("MoveIt: Motion completed successfully")
            else:
                rospy.logwarn("MoveIt: Motion planning or execution failed")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"MoveIt error: {e}")
            return False
    
    def send_pose_components(self, x: float, y: float, z: float,
                             qx: float, qy: float, qz: float, qw: float,
                             global_frame: str) -> bool:
        """
        Send a target pose using individual components.
        
        Args:
            x, y, z: Position coordinates
            qx, qy, qz, qw: Quaternion components
            global_frame: Reference frame
            
        Returns:
            True if motion was successful
        """
        pose_data = {
            "position": [x, y, z],
            "orientation": [qx, qy, qz, qw]
        }
        return self.send_pose(pose_data, global_frame)
    
    def set_planning_time(self, seconds: float):
        """Set the planning time limit."""
        if self._move_group:
            self._move_group.set_planning_time(seconds)
    
    def set_planning_attempts(self, attempts: int):
        """Set the number of planning attempts."""
        if self._move_group:
            self._move_group.set_num_planning_attempts(attempts)
    
    def shutdown(self):
        """Clean up MoveIt resources."""
        if MOVEIT_AVAILABLE and self._initialized:
            try:
                moveit_commander.roscpp_shutdown()
            except Exception:
                pass

