#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS interface for publishers, subscribers, and service clients.
"""

import os
import sys
from typing import Optional, Callable

import rospy
import rospkg
import yaml
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from .utils import print_success, print_warning, print_info, print_error
from .object_manager import ObjectInfo, ObjectStateManager, PRIMITIVE_TYPES

# Try to import MoveIt
try:
    import moveit_commander
    from moveit_commander import PlanningSceneInterface
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False

# Try to import scene_builder messages
try:
    from scene_builder.msg import ObjectCommand, ObjectVelocityCommand, DistanceInfo
    from scene_builder.srv import (
        SetMotionSequence, SetMotionSequenceRequest,
        GetMotionSequence, GetMotionSequenceRequest,
        ClearMotionSequence, ClearMotionSequenceRequest,
        ListObjects, ListObjectsRequest
    )
    SCENE_BUILDER_MSGS_AVAILABLE = True
except ImportError:
    SCENE_BUILDER_MSGS_AVAILABLE = False
    rospy.logwarn("scene_builder messages not available. Some features may be limited.")


# Configuration file path
CONFIG_FILE = "config/scene_config.yaml"


class ROSInterface:
    """Handles all ROS communication (publishers, subscribers, services)."""
    
    def __init__(self, state_manager: ObjectStateManager):
        """Initialize ROS interface.
        
        Args:
            state_manager: Object state manager reference
        """
        self.state_manager = state_manager
        
        # Planning scene interface
        self.planning_scene_interface: Optional[PlanningSceneInterface] = None
        
        # Publishers
        self.add_object_pub = None
        self.object_command_pub = None
        self.object_velocity_pub = None
        self.object_animation_pub = None
        self.planning_scene_pub = None
        
        # Subscribers
        self.distance_info_sub = None
        
        # Service proxies
        self.set_motion_sequence_srv = None
        self.get_motion_sequence_srv = None
        self.clear_motion_sequence_srv = None
        self.list_objects_srv = None
        
        # Last distance info message
        self.last_distance_info = None
        
        # Configuration file path
        self.config_file_path = None
    
    def initialize(self) -> bool:
        """Initialize ROS node and all communication channels.
        
        Returns:
            True if initialization successful
        """
        try:
            rospy.init_node('scene_interactive_control', anonymous=True)
            
            # Find configuration file path
            self._find_config_path()
            
            # Initialize MoveIt if available
            self._init_moveit()
            
            # Read global frame from parameter server
            self.state_manager.global_frame = rospy.get_param(
                '/object_command_node/planning_frame', 'world')
            
            # Setup publishers
            self._setup_publishers()
            
            # Setup subscribers
            self._setup_subscribers()
            
            # Setup service clients
            self._setup_services()
            
            # Wait for publishers to be ready
            rospy.sleep(0.5)
            
            return True
            
        except rospy.ROSException as e:
            print_error(f"ROS initialization error: {e}")
            return False
    
    def _find_config_path(self):
        """Find the configuration file path."""
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('scene_builder')
            self.config_file_path = os.path.join(pkg_path, CONFIG_FILE)
        except rospkg.ResourceNotFound:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.config_file_path = os.path.join(script_dir, '..', '..', CONFIG_FILE)
    
    def _init_moveit(self):
        """Initialize MoveIt components."""
        if not MOVEIT_AVAILABLE:
            return
        
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.planning_scene_interface = PlanningSceneInterface()
            print_success("MoveIt PlanningSceneInterface initialized!")
            rospy.sleep(0.5)
        except Exception as e:
            print_warning(f"Cannot initialize MoveIt: {e}")
            self.planning_scene_interface = None
    
    def _setup_publishers(self):
        """Setup ROS publishers."""
        self.add_object_pub = rospy.Publisher(
            '/add_object', CollisionObject, queue_size=10)
        
        if SCENE_BUILDER_MSGS_AVAILABLE:
            self.object_command_pub = rospy.Publisher(
                '/object_command', ObjectCommand, queue_size=10)
            self.object_velocity_pub = rospy.Publisher(
                '/object_velocity_command', ObjectVelocityCommand, queue_size=10)
        
        self.object_animation_pub = rospy.Publisher(
            '/object_animation', PoseArray, queue_size=10)
        
        self.planning_scene_pub = rospy.Publisher(
            '/planning_scene', PlanningScene, queue_size=1, latch=True)
    
    def _setup_subscribers(self):
        """Setup ROS subscribers."""
        if SCENE_BUILDER_MSGS_AVAILABLE:
            self.distance_info_sub = rospy.Subscriber(
                '/distance_info', DistanceInfo, self._distance_info_callback)
    
    def _setup_services(self):
        """Setup ROS service clients."""
        if not SCENE_BUILDER_MSGS_AVAILABLE:
            return
        
        try:
            rospy.wait_for_service('/object_command_node/set_motion_sequence', timeout=2.0)
            self.set_motion_sequence_srv = rospy.ServiceProxy(
                '/object_command_node/set_motion_sequence', SetMotionSequence)
            self.get_motion_sequence_srv = rospy.ServiceProxy(
                '/object_command_node/get_motion_sequence', GetMotionSequence)
            self.clear_motion_sequence_srv = rospy.ServiceProxy(
                '/object_command_node/clear_motion_sequence', ClearMotionSequence)
            self.list_objects_srv = rospy.ServiceProxy(
                '/object_command_node/list_objects', ListObjects)
            print_success("Motion sequence services (object_command_node) available!")
        except rospy.ROSException:
            print_info("Standalone mode: loops managed locally (object_command_node not started)")
    
    def _distance_info_callback(self, msg):
        """Callback for distance info messages."""
        self.last_distance_info = msg
    
    def sync_with_scene(self):
        """Synchronize internal state with MoveIt scene."""
        if self.planning_scene_interface is None:
            return
        
        try:
            scene_objects = self.planning_scene_interface.get_objects()
            
            for obj_id, obj in scene_objects.items():
                if obj_id not in self.state_manager.objects:
                    self._add_object_to_state(obj)
            
            # Remove objects no longer in scene
            for obj_id in list(self.state_manager.objects.keys()):
                if (obj_id not in scene_objects and 
                    obj_id not in self.state_manager.disabled_objects):
                    del self.state_manager.objects[obj_id]
                    
        except Exception as e:
            print_warning(f"Scene synchronization error: {e}")
    
    def _add_object_to_state(self, obj: CollisionObject):
        """Add a CollisionObject to internal state."""
        if not obj.primitives:
            return
        
        primitive = obj.primitives[0]
        
        # Determine primitive type
        primitive_type = "unknown"
        dimensions = []
        
        if primitive.type == SolidPrimitive.BOX:
            primitive_type = "box"
            dimensions = list(primitive.dimensions[:3])
        elif primitive.type == SolidPrimitive.SPHERE:
            primitive_type = "sphere"
            dimensions = [primitive.dimensions[SolidPrimitive.SPHERE_RADIUS]]
        elif primitive.type == SolidPrimitive.CYLINDER:
            primitive_type = "cylinder"
            dimensions = [primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT],
                         primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS]]
        elif primitive.type == SolidPrimitive.CONE:
            primitive_type = "cone"
            dimensions = [primitive.dimensions[SolidPrimitive.CONE_HEIGHT],
                         primitive.dimensions[SolidPrimitive.CONE_RADIUS]]
        
        # Get pose
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]
        
        if (obj.pose.position.x != 0.0 or obj.pose.position.y != 0.0 or 
            obj.pose.position.z != 0.0 or obj.pose.orientation.w != 0.0):
            position = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            orientation = [obj.pose.orientation.x, obj.pose.orientation.y,
                          obj.pose.orientation.z, obj.pose.orientation.w]
        elif obj.primitive_poses:
            pose = obj.primitive_poses[0]
            position = [pose.position.x, pose.position.y, pose.position.z]
            orientation = [pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w]
        
        # Create ObjectInfo
        info = ObjectInfo(
            id=obj.id,
            frame_id=obj.header.frame_id or self.state_manager.global_frame,
            primitive_type=primitive_type,
            dimensions=dimensions,
            position=position,
            orientation=orientation,
            enabled=True
        )
        
        # Preserve loop configuration if already exists
        if obj.id in self.state_manager.objects:
            old_info = self.state_manager.objects[obj.id]
            info.loop_enabled = old_info.loop_enabled
            info.loop_waypoints = old_info.loop_waypoints
        
        self.state_manager.objects[obj.id] = info
    
    def publish_collision_object(self, obj: CollisionObject):
        """Publish a collision object."""
        if self.add_object_pub:
            self.add_object_pub.publish(obj)
    
    def publish_object_command(self, obj_id: str, target_position: list,
                               target_orientation: list, duration: float) -> bool:
        """Publish an object movement command.
        
        Args:
            obj_id: Object ID
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [x, y, z, w]
            duration: Movement duration in seconds
            
        Returns:
            True if command was published
        """
        if not SCENE_BUILDER_MSGS_AVAILABLE or not self.object_command_pub:
            print_error("scene_builder messages not available")
            return False
        
        cmd = ObjectCommand()
        cmd.object_id = obj_id
        cmd.target_pose.position.x = target_position[0]
        cmd.target_pose.position.y = target_position[1]
        cmd.target_pose.position.z = target_position[2]
        cmd.target_pose.orientation.x = target_orientation[0]
        cmd.target_pose.orientation.y = target_orientation[1]
        cmd.target_pose.orientation.z = target_orientation[2]
        cmd.target_pose.orientation.w = target_orientation[3]
        cmd.move_duration = duration
        
        self.object_command_pub.publish(cmd)
        return True
    
    def set_motion_sequence(self, obj_id: str, waypoints: list, loop: bool) -> tuple:
        """Set motion sequence via service.
        
        Args:
            obj_id: Object ID
            waypoints: List of waypoint dictionaries
            loop: Whether to loop the sequence
            
        Returns:
            Tuple of (success, message)
        """
        if self.set_motion_sequence_srv is None:
            return False, "Service not available"
        
        try:
            req = SetMotionSequenceRequest()
            req.object_id = obj_id
            req.loop = loop
            
            for wp in waypoints:
                pose = Pose()
                pos = wp.get('position', [0, 0, 0])
                orient = wp.get('orientation', [0, 0, 0, 1])
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
                pose.orientation.x = orient[0]
                pose.orientation.y = orient[1]
                pose.orientation.z = orient[2]
                pose.orientation.w = orient[3]
                req.waypoints.append(pose)
                req.durations.append(wp.get('duration', 1.0))
            
            response = self.set_motion_sequence_srv(req)
            return response.success, response.message
        except rospy.ServiceException as e:
            return False, str(e)
    
    def clear_motion_sequence(self, obj_id: str) -> tuple:
        """Clear motion sequence via service.
        
        Args:
            obj_id: Object ID
            
        Returns:
            Tuple of (success, message)
        """
        if self.clear_motion_sequence_srv is None:
            return False, "Service not available"
        
        try:
            req = ClearMotionSequenceRequest()
            req.object_id = obj_id
            response = self.clear_motion_sequence_srv(req)
            return response.success, response.message
        except rospy.ServiceException as e:
            return False, str(e)
    
    def get_motion_sequence_status(self, obj_id: str) -> tuple:
        """Get motion sequence status via service.
        
        Args:
            obj_id: Object ID
            
        Returns:
            Tuple of (success, loop_active)
        """
        if self.get_motion_sequence_srv is None:
            return False, False
        
        try:
            req = GetMotionSequenceRequest()
            req.object_id = obj_id
            response = self.get_motion_sequence_srv(req)
            return response.success, response.loop_active
        except rospy.ServiceException:
            return False, False
    
    def load_config(self):
        """Load configuration from YAML file."""
        if not os.path.exists(self.config_file_path):
            return
        
        try:
            with open(self.config_file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            
            self._load_objects_from_config(data)
            self._load_sequences_from_config(data)
            self._load_disabled_from_config(data)
                
        except Exception as e:
            print_warning(f"Configuration loading error: {e}")
    
    def _load_objects_from_config(self, data: dict):
        """Load object definitions from config data."""
        objects_def = data.get('objects', {})
        for obj_id, obj_data in objects_def.items():
            if obj_id not in self.state_manager.objects:
                # Support both 'primitive'/'pose' format and flat format
                if 'primitive' in obj_data:
                    primitive = obj_data.get('primitive', {})
                    ptype = primitive.get('type', 'box')
                    dims = primitive.get('dimensions', [0.1, 0.1, 0.1])
                    pose = obj_data.get('pose', {})
                    pos = pose.get('position', [0, 0, 0])
                    orient = pose.get('orientation', [0, 0, 0, 1])
                else:
                    # Legacy flat format
                    ptype = obj_data.get('type', 'box')
                    dims = obj_data.get('dimensions', [0.1, 0.1, 0.1])
                    pos = obj_data.get('position', [0, 0, 0])
                    orient = obj_data.get('orientation', [0, 0, 0, 1])
                
                self.state_manager.objects[obj_id] = ObjectInfo(
                    id=obj_id,
                    frame_id=self.state_manager.global_frame,
                    primitive_type=ptype,
                    dimensions=dims,
                    position=pos,
                    orientation=orient,
                    enabled=True
                )
    
    def _load_sequences_from_config(self, data: dict):
        """Load motion sequences from config data."""
        motion_sequences = data.get('motion_sequences', {})
        for obj_id, seq_config in motion_sequences.items():
            waypoints = seq_config.get('waypoints', [])
            loop_enabled = seq_config.get('loop', False)
            
            if obj_id in self.state_manager.objects:
                self.state_manager.objects[obj_id].loop_waypoints = waypoints
                self.state_manager.objects[obj_id].loop_enabled = loop_enabled
            else:
                # Create placeholder object with sequence
                self.state_manager.objects[obj_id] = ObjectInfo(
                    id=obj_id,
                    frame_id=self.state_manager.global_frame,
                    primitive_type="unknown",
                    dimensions=[],
                    position=waypoints[0].get('position', [0, 0, 0]) if waypoints else [0, 0, 0],
                    orientation=waypoints[0].get('orientation', [0, 0, 0, 1]) if waypoints else [0, 0, 0, 1],
                    enabled=False,
                    loop_enabled=loop_enabled,
                    loop_waypoints=waypoints
                )
    
    def _load_disabled_from_config(self, data: dict):
        """Load disabled objects from config data."""
        disabled = data.get('disabled_objects', {})
        for obj_id, obj_data in disabled.items():
            if not obj_data:
                continue
            
            obj = CollisionObject()
            obj.id = obj_id
            obj.header.frame_id = obj_data.get('frame_id', self.state_manager.global_frame)
            obj.operation = CollisionObject.ADD
            
            # Support both formats
            if 'primitive' in obj_data:
                prim_data = obj_data.get('primitive', {})
                ptype = prim_data.get('type', 'box')
                dims = prim_data.get('dimensions', [0.1, 0.1, 0.1])
                pose_data = obj_data.get('pose', {})
                pos = pose_data.get('position', [0, 0, 0])
                orient = pose_data.get('orientation', [0, 0, 0, 1])
            else:
                ptype = obj_data.get('type', obj_data.get('primitive_type', 'box'))
                dims = obj_data.get('dimensions', [0.1, 0.1, 0.1])
                pos = obj_data.get('position', [0, 0, 0])
                orient = obj_data.get('orientation', [0, 0, 0, 1])
            
            primitive = SolidPrimitive()
            if ptype == 'box':
                primitive.type = SolidPrimitive.BOX
            elif ptype == 'sphere':
                primitive.type = SolidPrimitive.SPHERE
            elif ptype == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = dims
            obj.primitives = [primitive]
            
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            pose.orientation.x = orient[0]
            pose.orientation.y = orient[1]
            pose.orientation.z = orient[2]
            pose.orientation.w = orient[3]
            obj.primitive_poses = [pose]
            
            self.state_manager.disabled_objects[obj_id] = obj
    
    def save_config(self) -> bool:
        """Save configuration to YAML file.
        
        Returns:
            True if saved successfully
        """
        try:
            os.makedirs(os.path.dirname(self.config_file_path), exist_ok=True)
            
            # Load existing file to preserve structure
            existing_data = {}
            if os.path.exists(self.config_file_path):
                with open(self.config_file_path, 'r') as f:
                    existing_data = yaml.safe_load(f) or {}
            
            # Initialize sections
            if 'objects' not in existing_data:
                existing_data['objects'] = {}
            if 'motion_sequences' not in existing_data:
                existing_data['motion_sequences'] = {}
            if 'disabled_objects' not in existing_data:
                existing_data['disabled_objects'] = {}
            
            # Update objects
            for obj_id, obj in self.state_manager.objects.items():
                if obj.enabled and obj.primitive_type != "unknown":
                    existing_data['objects'][obj_id] = {
                        'primitive': {
                            'type': obj.primitive_type,
                            'dimensions': obj.dimensions
                        },
                        'pose': {
                            'position': obj.position,
                            'orientation': obj.orientation
                        }
                    }
            
            # Update motion sequences
            for obj_id, obj in self.state_manager.objects.items():
                if obj.loop_waypoints:
                    existing_data['motion_sequences'][obj_id] = {
                        'loop': obj.loop_enabled,
                        'waypoints': obj.loop_waypoints
                    }
            
            # Update disabled objects
            existing_data['disabled_objects'] = {}
            for obj_id, obj in self.state_manager.disabled_objects.items():
                ptype = 'box'
                dims = list(obj.primitives[0].dimensions) if obj.primitives else []
                pos = [0, 0, 0]
                orient = [0, 0, 0, 1]
                
                if obj.primitives:
                    prim_type = obj.primitives[0].type
                    if prim_type == SolidPrimitive.BOX:
                        ptype = 'box'
                    elif prim_type == SolidPrimitive.SPHERE:
                        ptype = 'sphere'
                    elif prim_type == SolidPrimitive.CYLINDER:
                        ptype = 'cylinder'
                
                if obj.primitive_poses:
                    pose = obj.primitive_poses[0]
                    pos = [pose.position.x, pose.position.y, pose.position.z]
                    orient = [pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w]
                
                existing_data['disabled_objects'][obj_id] = {
                    'frame_id': obj.header.frame_id,
                    'primitive': {
                        'type': ptype,
                        'dimensions': dims
                    },
                    'pose': {
                        'position': pos,
                        'orientation': orient
                    }
                }
            
            with open(self.config_file_path, 'w') as f:
                yaml.dump(existing_data, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=False)
            
            return True
        except Exception as e:
            print_error(f"Configuration save error: {e}")
            return False
    
    def shutdown(self):
        """Clean shutdown of ROS components."""
        if MOVEIT_AVAILABLE:
            try:
                moveit_commander.roscpp_shutdown()
            except:
                pass

