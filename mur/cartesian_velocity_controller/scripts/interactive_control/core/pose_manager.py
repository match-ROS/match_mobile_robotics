"""
Pose Manager - Handles loading, saving, and managing robot poses.
"""

import os
import yaml
from typing import Dict, List, Optional, Any

from ..config import (
    DEFAULT_POSITION_TOLERANCE,
    DEFAULT_ORIENTATION_TOLERANCE,
    DEFAULT_DWELL_TIME
)


class PoseManager:
    """
    Manages saved robot poses and loop configuration.
    
    Handles:
    - Loading/saving poses from YAML file
    - Adding/deleting poses
    - Loop configuration management
    """
    
    def __init__(self, poses_file_path: str, default_frame_id: Optional[str] = None):
        """
        Initialize the PoseManager.
        
        Args:
            poses_file_path: Path to the YAML file for storing poses
            default_frame_id: If set, missing pose frame_id will be upgraded to this value on load
        """
        self.poses_file_path = poses_file_path
        self.default_frame_id = default_frame_id
        self.saved_poses: Dict[str, Dict[str, Any]] = {}
        self.loop_config: Dict[str, Any] = {
            'poses': [],
            'position_tolerance': DEFAULT_POSITION_TOLERANCE,
            'orientation_tolerance': DEFAULT_ORIENTATION_TOLERANCE,
            'dwell_time': DEFAULT_DWELL_TIME
        }
    
    def load(self) -> bool:
        """
        Load poses and loop configuration from YAML file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if os.path.exists(self.poses_file_path):
                with open(self.poses_file_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                
                # Extract loop configuration if present
                if 'loop_config' in data:
                    loaded_loop_config = data.pop('loop_config')
                    # Merge with defaults to ensure all fields exist
                    self.loop_config.update(loaded_loop_config)
                
                self.saved_poses = data
                # Upgrade legacy poses to include frame_id for consistency (backward compatible)
                changed = False
                if self.default_frame_id:
                    for _name, pose in self.saved_poses.items():
                        if not isinstance(pose, dict):
                            continue
                        if "frame_id" not in pose:
                            pose["frame_id"] = str(self.default_frame_id)
                            changed = True
                if changed:
                    print(f"Upgraded saved poses with default frame_id='{self.default_frame_id}'.")
                    self.save()
                return True
            else:
                self.saved_poses = {}
                return True  # File doesn't exist yet, but that's OK
                
        except Exception as e:
            print(f"Error loading poses: {e}")
            self.saved_poses = {}
            return False
    
    def save(self) -> bool:
        """
        Save poses and loop configuration to YAML file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.poses_file_path), exist_ok=True)
            
            # Combine poses and loop configuration
            data_to_save = dict(self.saved_poses)
            data_to_save['loop_config'] = self.loop_config
            
            with open(self.poses_file_path, 'w') as f:
                yaml.dump(data_to_save, f, default_flow_style=False, allow_unicode=True)
            return True
            
        except Exception as e:
            print(f"Error saving poses: {e}")
            return False
    
    def add_pose(self, name: str, position: List[float], orientation: List[float],
                 description: str = "", frame_id: Optional[str] = None,
                 ee_frame: Optional[str] = None) -> bool:
        """
        Add or update a pose.
        
        Args:
            name: Pose name (will be sanitized)
            position: [x, y, z] position
            orientation: [qx, qy, qz, qw] quaternion
            description: Optional description
            
        Returns:
            True if successful
        """
        # Sanitize name
        clean_name = name.replace(" ", "_").lower()
        
        pose: Dict[str, Any] = {
            "position": list(position),
            "orientation": list(orientation),
            "description": description
        }

        if frame_id:
            pose["frame_id"] = str(frame_id)
        if ee_frame:
            pose["ee_frame"] = str(ee_frame)

        self.saved_poses[clean_name] = pose
        
        return self.save()
    
    def delete_pose(self, name: str) -> bool:
        """
        Delete a pose.
        
        Args:
            name: Pose name
            
        Returns:
            True if successful, False if pose not found
        """
        if name not in self.saved_poses:
            return False
        
        del self.saved_poses[name]
        
        # Also remove from loop config if present
        if name in self.loop_config['poses']:
            self.loop_config['poses'].remove(name)
        
        return self.save()
    
    def get_pose(self, name: str) -> Optional[Dict[str, Any]]:
        """
        Get a pose by name.
        
        Args:
            name: Pose name
            
        Returns:
            Pose data dict or None if not found
        """
        return self.saved_poses.get(name)
    
    def list_poses(self) -> List[str]:
        """
        Get list of all pose names.
        
        Returns:
            List of pose names
        """
        return list(self.saved_poses.keys())
    
    def pose_exists(self, name: str) -> bool:
        """Check if a pose exists."""
        return name in self.saved_poses
    
    def get_pose_count(self) -> int:
        """Get the number of saved poses."""
        return len(self.saved_poses)
    
    # =========================================================================
    # Loop Configuration
    # =========================================================================
    
    def set_loop_poses(self, poses: List[str]) -> bool:
        """
        Set the poses for loop movement.
        
        Args:
            poses: List of pose names
            
        Returns:
            True if successful
        """
        # Filter to only existing poses
        valid_poses = [p for p in poses if p in self.saved_poses]
        self.loop_config['poses'] = valid_poses
        return self.save()
    
    def get_loop_poses(self) -> List[str]:
        """Get the list of poses configured for loop movement."""
        return self.loop_config.get('poses', [])
    
    def get_valid_loop_poses(self) -> List[str]:
        """Get loop poses that actually exist in saved poses."""
        return [p for p in self.loop_config.get('poses', []) if p in self.saved_poses]
    
    def set_loop_tolerances(self, position_tol: float, orientation_tol: float) -> bool:
        """
        Set loop movement tolerances.
        
        Args:
            position_tol: Position tolerance in meters
            orientation_tol: Orientation tolerance in radians
            
        Returns:
            True if successful
        """
        self.loop_config['position_tolerance'] = position_tol
        self.loop_config['orientation_tolerance'] = orientation_tol
        return self.save()
    
    def set_loop_dwell_time(self, dwell_time: float) -> bool:
        """
        Set dwell time for loop movement.
        
        Args:
            dwell_time: Time to wait at each pose in seconds
            
        Returns:
            True if successful
        """
        self.loop_config['dwell_time'] = dwell_time
        return self.save()
    
    def get_position_tolerance(self) -> float:
        """Get position tolerance for loop movement."""
        return self.loop_config.get('position_tolerance', DEFAULT_POSITION_TOLERANCE)
    
    def get_orientation_tolerance(self) -> float:
        """Get orientation tolerance for loop movement."""
        return self.loop_config.get('orientation_tolerance', DEFAULT_ORIENTATION_TOLERANCE)
    
    def get_dwell_time(self) -> float:
        """Get dwell time for loop movement."""
        return self.loop_config.get('dwell_time', DEFAULT_DWELL_TIME)

