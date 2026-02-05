#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Object state management and data classes.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from shape_msgs.msg import SolidPrimitive


# Supported primitive types
PRIMITIVE_TYPES = {
    "box": SolidPrimitive.BOX,
    "sphere": SolidPrimitive.SPHERE,
    "cylinder": SolidPrimitive.CYLINDER,
    "cone": SolidPrimitive.CONE,
}

# Default dimensions for each type
DEFAULT_DIMENSIONS = {
    "box": [0.1, 0.1, 0.1],       # [x, y, z]
    "sphere": [0.1],              # [radius]
    "cylinder": [0.2, 0.05],      # [height, radius]
    "cone": [0.2, 0.05],          # [height, radius]
}


@dataclass
class ObjectInfo:
    """Information about a collision object."""
    id: str
    frame_id: str
    primitive_type: str
    dimensions: List[float]
    position: List[float]
    orientation: List[float]
    enabled: bool = True
    # Loop configuration
    loop_enabled: bool = False
    loop_waypoints: List[Dict] = field(default_factory=list)
    loop_current_idx: int = 0


@dataclass
class LoopWaypoint:
    """Waypoint for loop movement."""
    position: List[float]
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    duration: float = 1.0


class ObjectStateManager:
    """Manages the state of collision objects."""
    
    def __init__(self, global_frame: str = "world"):
        """Initialize the object state manager.
        
        Args:
            global_frame: The global reference frame name
        """
        self.objects: Dict[str, ObjectInfo] = {}
        self.disabled_objects: Dict[str, 'CollisionObject'] = {}
        self.global_frame = global_frame
    
    def add_object_info(self, obj_info: ObjectInfo) -> None:
        """Add or update an object's information.
        
        Args:
            obj_info: The object information to add/update
        """
        self.objects[obj_info.id] = obj_info
    
    def get_object_info(self, obj_id: str) -> Optional[ObjectInfo]:
        """Get information about an object.
        
        Args:
            obj_id: The object ID
            
        Returns:
            ObjectInfo if found, None otherwise
        """
        return self.objects.get(obj_id)
    
    def remove_object(self, obj_id: str) -> bool:
        """Remove an object from tracking.
        
        Args:
            obj_id: The object ID to remove
            
        Returns:
            True if removed, False if not found
        """
        if obj_id in self.objects:
            del self.objects[obj_id]
            return True
        if obj_id in self.disabled_objects:
            del self.disabled_objects[obj_id]
            return True
        return False
    
    def get_active_objects(self) -> List[ObjectInfo]:
        """Get list of active (enabled) objects.
        
        Returns:
            List of enabled ObjectInfo instances
        """
        return [o for o in self.objects.values() if o.enabled]
    
    def get_all_object_ids(self) -> List[str]:
        """Get all object IDs (active and disabled).
        
        Returns:
            List of all object IDs
        """
        return list(self.objects.keys()) + list(self.disabled_objects.keys())
    
    def disable_object(self, obj_id: str, collision_obj: 'CollisionObject') -> bool:
        """Mark an object as disabled and store backup.
        
        Args:
            obj_id: The object ID
            collision_obj: The CollisionObject to backup
            
        Returns:
            True if disabled successfully
        """
        if obj_id in self.objects:
            self.objects[obj_id].enabled = False
            self.disabled_objects[obj_id] = collision_obj
            return True
        return False
    
    def enable_object(self, obj_id: str) -> Optional['CollisionObject']:
        """Re-enable a disabled object.
        
        Args:
            obj_id: The object ID
            
        Returns:
            The stored CollisionObject if found, None otherwise
        """
        if obj_id in self.disabled_objects:
            obj = self.disabled_objects.pop(obj_id)
            if obj_id in self.objects:
                self.objects[obj_id].enabled = True
            return obj
        return None
    
    def get_objects_with_sequences(self) -> List[ObjectInfo]:
        """Get objects that have loop sequences configured.
        
        Returns:
            List of ObjectInfo with loop_waypoints
        """
        return [o for o in self.objects.values() 
                if o.enabled or o.loop_waypoints]
    
    def update_position(self, obj_id: str, position: List[float], 
                       orientation: Optional[List[float]] = None) -> bool:
        """Update an object's position.
        
        Args:
            obj_id: The object ID
            position: New position [x, y, z]
            orientation: New orientation [x, y, z, w] (optional)
            
        Returns:
            True if updated successfully
        """
        if obj_id in self.objects:
            self.objects[obj_id].position = position
            if orientation:
                self.objects[obj_id].orientation = orientation
            return True
        return False

