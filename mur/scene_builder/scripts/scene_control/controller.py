#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main controller orchestrating scene control components.
"""

import signal
import sys
import time
import threading
from typing import Dict, List, Optional

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from .utils import (
    clear_screen, print_header, print_success, print_error,
    print_warning, print_info, wait_for_key
)
from .object_manager import ObjectInfo, ObjectStateManager, PRIMITIVE_TYPES
from .ros_interface import ROSInterface
from .menu_ui import MenuUI


class SceneInteractiveController:
    """Main controller for interactive scene management."""
    
    def __init__(self):
        """Initialize the controller."""
        # State manager
        self.state_manager = ObjectStateManager()
        
        # ROS interface
        self.ros_interface = ROSInterface(self.state_manager)
        
        # Menu UI
        self.menu_ui = MenuUI(self.state_manager)
        
        # Loop management
        self.loop_threads: Dict[str, threading.Thread] = {}
        self.loop_stop_events: Dict[str, threading.Event] = {}
        
        # Running flag
        self.running = True
        
        # Wire up menu callbacks
        self._setup_menu_callbacks()
    
    def _setup_menu_callbacks(self):
        """Setup callbacks for menu UI."""
        self.menu_ui.on_add_object = self.add_object
        self.menu_ui.on_remove_object = self.remove_object
        self.menu_ui.on_disable_object = self.disable_object
        self.menu_ui.on_enable_object = self.enable_object
        self.menu_ui.on_move_object = self.move_object
        self.menu_ui.on_start_loop = self.start_loop
        self.menu_ui.on_stop_loop = self.stop_loop
        self.menu_ui.on_is_loop_running = self.is_loop_running
        self.menu_ui.on_add_waypoint = self.add_loop_waypoint
        self.menu_ui.on_remove_waypoint = self.remove_loop_waypoint
        self.menu_ui.on_clear_waypoints = self.clear_loop_waypoints
        self.menu_ui.on_refresh = self.refresh_objects
        self.menu_ui.on_get_distance_info = lambda: self.ros_interface.last_distance_info
    
    def initialize(self) -> bool:
        """Initialize the controller.
        
        Returns:
            True if initialization successful
        """
        if not self.ros_interface.initialize():
            return False
        
        # Load configuration
        self.ros_interface.load_config()
        
        # Sync with scene
        self.ros_interface.sync_with_scene()
        
        return True
    
    def refresh_objects(self):
        """Refresh object list from scene."""
        self.ros_interface.sync_with_scene()
    
    def add_object(self, obj_id: str, primitive_type: str, 
                   dimensions: List[float], position: List[float],
                   orientation: Optional[List[float]] = None) -> bool:
        """Add a new object to the scene.
        
        Args:
            obj_id: Unique object ID
            primitive_type: Type of primitive (box, sphere, cylinder)
            dimensions: Primitive dimensions
            position: Position [x, y, z]
            orientation: Quaternion orientation [x, y, z, w]
            
        Returns:
            True if object was added successfully
        """
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
        
        if primitive_type not in PRIMITIVE_TYPES:
            print_error(f"Unsupported primitive type: {primitive_type}")
            return False
        
        # Create CollisionObject message
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = self.state_manager.global_frame
        obj.header.stamp = rospy.Time.now()
        obj.operation = CollisionObject.ADD
        
        # Create primitive
        primitive = SolidPrimitive()
        primitive.type = PRIMITIVE_TYPES[primitive_type]
        primitive.dimensions = dimensions
        obj.primitives = [primitive]
        
        # Set pose
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        obj.primitive_poses = [pose]
        
        # Publish object
        self.ros_interface.publish_collision_object(obj)
        
        # Update internal state
        info = ObjectInfo(
            id=obj_id,
            frame_id=self.state_manager.global_frame,
            primitive_type=primitive_type,
            dimensions=dimensions,
            position=position,
            orientation=orientation,
            enabled=True
        )
        self.state_manager.objects[obj_id] = info
        
        return True
    
    def remove_object(self, obj_id: str) -> bool:
        """Remove an object from the scene.
        
        Args:
            obj_id: Object ID to remove
            
        Returns:
            True if object was removed successfully
        """
        # Stop any active loops
        self.stop_loop(obj_id)
        
        # Create removal message
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = self.state_manager.global_frame
        obj.header.stamp = rospy.Time.now()
        obj.operation = CollisionObject.REMOVE
        
        # Publish removal
        self.ros_interface.publish_collision_object(obj)
        
        # Remove from internal state
        self.state_manager.remove_object(obj_id)
        
        self.ros_interface.save_config()
        
        return True
    
    def disable_object(self, obj_id: str) -> bool:
        """Disable an object (remove from scene but keep data).
        
        Args:
            obj_id: Object ID to disable
            
        Returns:
            True if object was disabled
        """
        if obj_id not in self.state_manager.objects:
            return False
        
        obj_info = self.state_manager.objects[obj_id]
        
        # Stop any active loops
        self.stop_loop(obj_id)
        
        # Create backup CollisionObject
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = obj_info.frame_id
        obj.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = PRIMITIVE_TYPES.get(obj_info.primitive_type, SolidPrimitive.BOX)
        primitive.dimensions = obj_info.dimensions
        obj.primitives = [primitive]
        
        pose = Pose()
        pose.position.x = obj_info.position[0]
        pose.position.y = obj_info.position[1]
        pose.position.z = obj_info.position[2]
        pose.orientation.x = obj_info.orientation[0]
        pose.orientation.y = obj_info.orientation[1]
        pose.orientation.z = obj_info.orientation[2]
        pose.orientation.w = obj_info.orientation[3]
        obj.primitive_poses = [pose]
        
        self.state_manager.disabled_objects[obj_id] = obj
        
        # Remove from scene
        remove_obj = CollisionObject()
        remove_obj.id = obj_id
        remove_obj.header.frame_id = self.state_manager.global_frame
        remove_obj.header.stamp = rospy.Time.now()
        remove_obj.operation = CollisionObject.REMOVE
        self.ros_interface.publish_collision_object(remove_obj)
        
        # Update state
        obj_info.enabled = False
        
        self.ros_interface.save_config()
        
        return True
    
    def enable_object(self, obj_id: str) -> bool:
        """Re-enable a previously disabled object.
        
        Args:
            obj_id: Object ID to re-enable
            
        Returns:
            True if object was re-enabled
        """
        if obj_id not in self.state_manager.disabled_objects:
            return False
        
        obj = self.state_manager.disabled_objects[obj_id]
        obj.header.stamp = rospy.Time.now()
        
        # Republish object
        self.ros_interface.publish_collision_object(obj)
        
        # Remove from backup
        del self.state_manager.disabled_objects[obj_id]
        
        # Update state
        if obj_id in self.state_manager.objects:
            self.state_manager.objects[obj_id].enabled = True
        
        self.ros_interface.save_config()
        
        return True
    
    def move_object(self, obj_id: str, target_position: List[float],
                    target_orientation: Optional[List[float]] = None,
                    duration: float = 1.0) -> bool:
        """Move an object to a new position.
        
        Args:
            obj_id: Object ID to move
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [x, y, z, w]
            duration: Movement duration in seconds
            
        Returns:
            True if command was sent
        """
        if target_orientation is None:
            if obj_id in self.state_manager.objects:
                target_orientation = self.state_manager.objects[obj_id].orientation
            else:
                target_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Send command
        success = self.ros_interface.publish_object_command(
            obj_id, target_position, target_orientation, duration)
        
        # Update internal state
        if success and obj_id in self.state_manager.objects:
            self.state_manager.objects[obj_id].position = target_position
            self.state_manager.objects[obj_id].orientation = target_orientation
        
        return success
    
    # =========================================================================
    # LOOP MOVEMENT METHODS
    # =========================================================================
    
    def add_loop_waypoint(self, obj_id: str, position: List[float],
                          orientation: Optional[List[float]] = None,
                          duration: float = 1.0) -> bool:
        """Add a waypoint to object's loop sequence.
        
        Args:
            obj_id: Object ID
            position: Waypoint position [x, y, z]
            orientation: Waypoint orientation [x, y, z, w]
            duration: Time to reach waypoint
            
        Returns:
            True if waypoint was added
        """
        if obj_id not in self.state_manager.objects:
            return False
        
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
        
        waypoint = {
            'position': position,
            'orientation': orientation,
            'duration': duration
        }
        
        self.state_manager.objects[obj_id].loop_waypoints.append(waypoint)
        self.ros_interface.save_config()
        return True
    
    def remove_loop_waypoint(self, obj_id: str, index: int) -> bool:
        """Remove a waypoint from object's loop sequence.
        
        Args:
            obj_id: Object ID
            index: Waypoint index to remove
            
        Returns:
            True if waypoint was removed
        """
        if obj_id not in self.state_manager.objects:
            return False
        
        waypoints = self.state_manager.objects[obj_id].loop_waypoints
        if 0 <= index < len(waypoints):
            waypoints.pop(index)
            self.ros_interface.save_config()
            return True
        return False
    
    def clear_loop_waypoints(self, obj_id: str) -> bool:
        """Clear all waypoints from object's loop sequence.
        
        Args:
            obj_id: Object ID
            
        Returns:
            True if waypoints were cleared
        """
        if obj_id not in self.state_manager.objects:
            return False
        
        self.state_manager.objects[obj_id].loop_waypoints = []
        self.ros_interface.save_config()
        return True
    
    def start_loop(self, obj_id: str) -> bool:
        """Start loop movement for an object.
        
        Args:
            obj_id: Object ID
            
        Returns:
            True if loop was started
        """
        if obj_id not in self.state_manager.objects:
            return False
        
        obj = self.state_manager.objects[obj_id]
        if not obj.loop_waypoints:
            print_error("No waypoints configured for loop")
            return False
        
        # Try to use service if available
        success, message = self.ros_interface.set_motion_sequence(
            obj_id, obj.loop_waypoints, True)
        
        if success:
            obj.loop_enabled = True
            self.ros_interface.save_config()
            return True
        else:
            if message != "Service not available":
                print_warning(f"Service: {message}")
        
        # Fallback: use local threading
        if obj_id in self.loop_threads and self.loop_threads[obj_id].is_alive():
            print_warning("Loop already running for this object")
            return False
        
        # Create stop event and start thread
        self.loop_stop_events[obj_id] = threading.Event()
        self.loop_threads[obj_id] = threading.Thread(
            target=self._loop_thread_func,
            args=(obj_id,),
            daemon=True
        )
        self.loop_threads[obj_id].start()
        
        obj.loop_enabled = True
        self.ros_interface.save_config()
        
        return True
    
    def stop_loop(self, obj_id: str) -> bool:
        """Stop loop movement for an object.
        
        Args:
            obj_id: Object ID
            
        Returns:
            True if loop was stopped
        """
        # Try to use service if available
        success, message = self.ros_interface.clear_motion_sequence(obj_id)
        if not success and message != "Service not available":
            print_warning(f"Service: {message}")
        
        # Stop local threading if active
        if obj_id in self.loop_stop_events:
            self.loop_stop_events[obj_id].set()
            
            if obj_id in self.loop_threads:
                self.loop_threads[obj_id].join(timeout=2.0)
                del self.loop_threads[obj_id]
            
            del self.loop_stop_events[obj_id]
        
        if obj_id in self.state_manager.objects:
            self.state_manager.objects[obj_id].loop_enabled = False
            self.ros_interface.save_config()
        
        return True
    
    def _loop_thread_func(self, obj_id: str):
        """Thread function for loop movement.
        
        Args:
            obj_id: Object ID to animate
        """
        rospy.loginfo(f"Loop movement started for '{obj_id}'")
        
        if obj_id not in self.state_manager.objects:
            return
        
        obj = self.state_manager.objects[obj_id]
        waypoints = obj.loop_waypoints
        
        if not waypoints:
            obj.loop_enabled = False
            return
        
        idx = 0
        stop_event = self.loop_stop_events.get(obj_id)
        
        while not stop_event.is_set() and not rospy.is_shutdown():
            wp = waypoints[idx]
            
            position = wp.get('position', [0, 0, 0])
            orientation = wp.get('orientation', [0, 0, 0, 1])
            duration = wp.get('duration', 1.0)
            
            rospy.logdebug(f"Loop: moving '{obj_id}' to waypoint {idx + 1}/{len(waypoints)}")
            
            # Send command
            self.move_object(obj_id, position, orientation, duration)
            
            # Wait for movement duration (with interruption check)
            wait_time = duration
            start_time = time.time()
            while time.time() - start_time < wait_time:
                if stop_event.is_set():
                    break
                time.sleep(0.05)
            
            if stop_event.is_set():
                break
            
            # Move to next waypoint
            idx = (idx + 1) % len(waypoints)
            obj.loop_current_idx = idx
        
        rospy.loginfo(f"Loop movement stopped for '{obj_id}'")
        obj.loop_enabled = False
    
    def is_loop_running(self, obj_id: str) -> bool:
        """Check if loop is running for an object.
        
        Args:
            obj_id: Object ID
            
        Returns:
            True if loop is running
        """
        # Check local threading first
        if obj_id in self.loop_threads and self.loop_threads[obj_id].is_alive():
            return True
        
        # Check service if available
        success, loop_active = self.ros_interface.get_motion_sequence_status(obj_id)
        if success:
            return loop_active
        
        return False
    
    def run(self):
        """Main loop of the controller."""
        if not self.initialize():
            print_error("Cannot initialize ROS. Make sure roscore is running.")
            sys.exit(1)
        
        # User confirmation before starting interactive interface
        print()
        print_success("Initialization complete!")
        
        # Count objects and sequences
        objects_in_scene = sum(1 for o in self.state_manager.objects.values() if o.enabled)
        objects_with_sequences = sum(1 for o in self.state_manager.objects.values() if o.loop_waypoints)
        
        print_info(f"Objects in scene: {objects_in_scene}")
        print_info(f"Objects with motion sequences: {objects_with_sequences}")
        print_info(f"Disabled objects: {len(self.state_manager.disabled_objects)}")
        
        if objects_with_sequences > 0:
            print_info("Use option [6] 🔁 to manage loop movements")
        
        wait_for_key()
        
        signal.signal(signal.SIGINT, lambda s, f: self.shutdown())
        
        while self.running and not rospy.is_shutdown():
            try:
                choice = self.menu_ui.show_main_menu()
                
                if choice == "0":
                    self.shutdown()
                elif choice == "1":
                    self.menu_ui.show_objects_status()
                elif choice == "2":
                    self.menu_ui.show_add_object_menu()
                elif choice == "3":
                    self.menu_ui.show_remove_object_menu()
                elif choice == "4":
                    self.menu_ui.show_enable_disable_menu()
                elif choice == "5":
                    self.menu_ui.show_move_object_menu()
                elif choice == "6":
                    self.menu_ui.show_loop_menu()
                elif choice == "7":
                    self.menu_ui.show_distances_menu()
                elif choice == "8":
                    self.refresh_objects()
                    print_success("Scene updated!")
                    wait_for_key()
                else:
                    print_error("Invalid option")
                    wait_for_key()
                    
            except rospy.ROSInterruptException:
                break
            except EOFError:
                break
    
    def shutdown(self):
        """Terminate the controller."""
        self.running = False
        
        # Stop all active loops
        for obj_id in list(self.loop_threads.keys()):
            self.stop_loop(obj_id)
        
        # Save configuration
        self.ros_interface.save_config()
        
        clear_screen()
        print_header("👋 ARRIVEDERCI!")
        print_success("Script terminato correttamente.")
        print()
        
        self.ros_interface.shutdown()

