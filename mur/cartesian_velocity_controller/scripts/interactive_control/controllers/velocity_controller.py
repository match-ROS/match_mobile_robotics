"""
Velocity Pose Controller - Sends poses via the velocity controller topic.
"""

import rospy
from typing import Dict, Any, Optional

from ..core.ros_interface import ROSInterface


class VelocityPoseController:
    """
    Sends target poses to the robot via the cartesian velocity controller.
    
    Uses the ROSInterface for publishing and frame management.
    """
    
    def __init__(self, ros_interface: ROSInterface):
        """
        Initialize the velocity pose controller.
        
        Args:
            ros_interface: Initialized ROSInterface instance
        """
        self.ros = ros_interface
    
    def send_pose(self, pose_data: Dict[str, Any]) -> bool:
        """
        Send a target pose via the velocity controller topic.
        
        Args:
            pose_data: Dict with 'position' [x,y,z] and 'orientation' [qx,qy,qz,qw]
            
        Returns:
            True if sent successfully
        """
        if not self.ros.is_initialized():
            rospy.logerr("ROS interface not initialized")
            return False
        
        return self.ros.publish_target_pose(pose_data)
    
    def send_pose_components(self, x: float, y: float, z: float,
                             qx: float = 0.0, qy: float = 0.707,
                             qz: float = 0.0, qw: float = 0.707) -> bool:
        """
        Send a target pose using individual components.
        
        Default orientation is pointing downward.
        
        Args:
            x, y, z: Position coordinates
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            True if sent successfully
        """
        pose_data = {
            "position": [x, y, z],
            "orientation": [qx, qy, qz, qw]
        }
        return self.send_pose(pose_data)
    
    def is_controller_connected(self) -> bool:
        """
        Check if the velocity controller is connected and receiving messages.
        
        Returns:
            True if at least one subscriber is connected to the target_pose topic
        """
        return self.ros.has_subscriber()

