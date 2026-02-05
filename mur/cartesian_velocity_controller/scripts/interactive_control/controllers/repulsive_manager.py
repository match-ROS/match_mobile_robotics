"""
Repulsive Manager - Handles repulsive velocity configuration.

Supports:
- Payload link: Primary link for repulsive velocity (no null space projection)
- Null space links: Secondary links for repulsive velocity (with null space projection)
- Collision objects: All objects or specific objects from the scene
"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from typing import Optional, Dict, Any, List, Set

from ..config import (
    DEFAULT_CONTROLLER_NODE,
    SERVICE_TIMEOUT,
    AVAILABLE_LINKS,
    AVAILABLE_OBSTACLES,
    COLLISION_OBJECTS_MODE_ALL,
    COLLISION_OBJECTS_MODE_SPECIFIC
)


class RepulsiveManager:
    """
    Manages the repulsive velocity configuration for obstacle avoidance.
    
    Handles:
    - Enabling/disabling repulsive velocity
    - Setting payload link (direct repulsion, no null space)
    - Setting null space repulsive links (repulsion with null space projection)
    - Setting collision objects mode (all or specific)
    - Reading current configuration
    """
    
    def __init__(self, controller_node_name: str = DEFAULT_CONTROLLER_NODE):
        """
        Initialize the repulsive manager.
        
        Args:
            controller_node_name: Name of the velocity controller node
        """
        self.controller_node_name = controller_node_name
        
        # State
        self._enabled = False
        self._payload_link = "wrist_3_link"
        self._null_space_links: List[str] = ["upper_arm_link", "forearm_link"]
        self._collision_objects_mode = COLLISION_OBJECTS_MODE_ALL
        self._collision_objects: List[str] = []
        
        # Legacy state (for backward compatibility)
        self._current_link = ""
        self._current_obstacle = ""
        
        # Publishers
        self._set_link_pub: Optional[rospy.Publisher] = None
        self._set_object_pub: Optional[rospy.Publisher] = None
        self._set_payload_link_pub: Optional[rospy.Publisher] = None
        self._set_collision_mode_pub: Optional[rospy.Publisher] = None
        
        # Service proxies
        self._set_enabled_srv: Optional[rospy.ServiceProxy] = None
        self._get_config_srv: Optional[rospy.ServiceProxy] = None
        
        self._initialized = False
    
    def initialize(self) -> bool:
        """
        Initialize publishers and service proxies.
        
        Returns:
            True if at least publishers are initialized
        """
        try:
            # Publishers for dynamic configuration
            # Legacy publishers
            self._set_link_pub = rospy.Publisher(
                f'{self.controller_node_name}/repulsive/set_target_link',
                String,
                queue_size=1
            )
            self._set_object_pub = rospy.Publisher(
                f'{self.controller_node_name}/repulsive/set_target_object',
                String,
                queue_size=1
            )
            
            # New publishers for multi-link configuration
            self._set_payload_link_pub = rospy.Publisher(
                f'{self.controller_node_name}/repulsive/set_payload_link',
                String,
                queue_size=1
            )
            self._set_collision_mode_pub = rospy.Publisher(
                f'{self.controller_node_name}/repulsive/set_collision_objects_mode',
                String,
                queue_size=1
            )
            
            # Try to connect to services (optional)
            try:
                rospy.wait_for_service(
                    f'{self.controller_node_name}/repulsive/set_enabled',
                    timeout=SERVICE_TIMEOUT
                )
                self._set_enabled_srv = rospy.ServiceProxy(
                    f'{self.controller_node_name}/repulsive/set_enabled',
                    SetBool
                )
                self._get_config_srv = rospy.ServiceProxy(
                    f'{self.controller_node_name}/repulsive/get_config',
                    Trigger
                )
                rospy.loginfo("Repulsive configuration services available")
            except rospy.ROSException:
                rospy.logwarn("Repulsive configuration services not available")
            
            self._initialized = True
            
            # Try to read current configuration
            self.refresh_config()
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize repulsive manager: {e}")
            return False
    
    def is_initialized(self) -> bool:
        """Check if the manager is initialized."""
        return self._initialized
    
    def has_services(self) -> bool:
        """Check if services are available."""
        return self._set_enabled_srv is not None
    
    # =========================================================================
    # State Properties
    # =========================================================================
    
    @property
    def enabled(self) -> bool:
        """Get whether repulsive velocity is enabled."""
        return self._enabled
    
    @property
    def payload_link(self) -> str:
        """Get the current payload link (direct repulsion, no null space)."""
        return self._payload_link
    
    @property
    def null_space_links(self) -> List[str]:
        """Get the current null space repulsive links."""
        return self._null_space_links.copy()
    
    @property
    def collision_objects_mode(self) -> str:
        """Get the collision objects mode ('all' or 'specific')."""
        return self._collision_objects_mode
    
    @property
    def collision_objects(self) -> List[str]:
        """Get the list of specific collision objects."""
        return self._collision_objects.copy()
    
    # Legacy properties (for backward compatibility)
    @property
    def current_link(self) -> str:
        """Get the current target link (legacy)."""
        return self._current_link
    
    @property
    def current_obstacle(self) -> str:
        """Get the current target obstacle (legacy)."""
        return self._current_obstacle
    
    # =========================================================================
    # Configuration Methods
    # =========================================================================
    
    def set_enabled(self, enabled: bool) -> bool:
        """
        Enable or disable repulsive velocity.
        
        Args:
            enabled: True to enable, False to disable
            
        Returns:
            True if successful
        """
        if self._set_enabled_srv is not None:
            try:
                response = self._set_enabled_srv(enabled)
                if response.success:
                    self._enabled = enabled
                    return True
                else:
                    rospy.logwarn(f"Set enabled service returned: {response.message}")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"Set enabled service call failed: {e}")
                return False
        else:
            rospy.logwarn("Set enabled service not available")
            return False
    
    def enable(self) -> bool:
        """Enable repulsive velocity."""
        return self.set_enabled(True)
    
    def disable(self) -> bool:
        """Disable repulsive velocity."""
        return self.set_enabled(False)
    
    def set_target_link(self, link: str) -> bool:
        """
        Set the target link for repulsive velocity.
        
        Args:
            link: Link name, or empty string for all links
            
        Returns:
            True if published successfully
        """
        if self._set_link_pub is None:
            return False
        
        try:
            msg = String()
            msg.data = link
            self._set_link_pub.publish(msg)
            self._current_link = link
            rospy.loginfo(f"Set target link to: {link or '<all>'}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set target link: {e}")
            return False
    
    def set_target_object(self, obstacle: str) -> bool:
        """
        Set the target obstacle for repulsive velocity (legacy).
        
        Args:
            obstacle: Obstacle ID, or empty string for all obstacles
            
        Returns:
            True if published successfully
        """
        if self._set_object_pub is None:
            return False
        
        try:
            msg = String()
            msg.data = obstacle
            self._set_object_pub.publish(msg)
            self._current_obstacle = obstacle
            rospy.loginfo(f"Set target obstacle to: {obstacle or '<all>'}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set target obstacle: {e}")
            return False
    
    # =========================================================================
    # Payload Link Configuration
    # =========================================================================
    
    def set_payload_link(self, link: str) -> bool:
        """
        Set the payload link for direct repulsive velocity (no null space).
        
        This link receives repulsive velocities that are NOT projected into
        the null space, making them the primary avoidance task.
        
        Args:
            link: Link name, or empty string to disable payload repulsion
            
        Returns:
            True if published successfully
        """
        if self._set_payload_link_pub is None:
            return False
        
        try:
            msg = String()
            msg.data = link
            self._set_payload_link_pub.publish(msg)
            self._payload_link = link
            rospy.loginfo(f"Set payload link to: {link or '<disabled>'}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set payload link: {e}")
            return False
    
    # =========================================================================
    # Collision Objects Mode Configuration
    # =========================================================================
    
    def set_collision_objects_mode(self, mode: str) -> bool:
        """
        Set the collision objects mode.
        
        Args:
            mode: 'all' to consider all objects from distance_info,
                  'specific' to only consider objects in the list
            
        Returns:
            True if published successfully
        """
        if self._set_collision_mode_pub is None:
            return False
        
        if mode not in [COLLISION_OBJECTS_MODE_ALL, COLLISION_OBJECTS_MODE_SPECIFIC]:
            rospy.logwarn(f"Invalid collision objects mode: {mode}")
            return False
        
        try:
            msg = String()
            msg.data = mode
            self._set_collision_mode_pub.publish(msg)
            self._collision_objects_mode = mode
            rospy.loginfo(f"Set collision objects mode to: {mode}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to set collision objects mode: {e}")
            return False
    
    def use_all_collision_objects(self) -> bool:
        """Enable considering all collision objects from the scene."""
        return self.set_collision_objects_mode(COLLISION_OBJECTS_MODE_ALL)
    
    def use_specific_collision_objects(self) -> bool:
        """Enable considering only specific collision objects."""
        return self.set_collision_objects_mode(COLLISION_OBJECTS_MODE_SPECIFIC)
    
    def refresh_config(self, *, use_service: bool = True):
        """Refresh configuration from the controller.

        Args:
            use_service: If True, try the controller service first (if available).
                         If False, read from the parameter server only (faster, non-blocking).
        """
        # Try service first (may block if server is slow)
        if use_service and self._get_config_srv is not None:
            try:
                response = self._get_config_srv()
                if response.success:
                    self._parse_config_message(response.message)
                    return
            except rospy.ServiceException:
                pass
        
        # Fallback to parameter server
        try:
            # New configuration
            self._payload_link = rospy.get_param(
                f'{self.controller_node_name}/payload_link',
                self._payload_link
            )
            self._null_space_links = rospy.get_param(
                f'{self.controller_node_name}/null_space_repulsive_links',
                self._null_space_links
            )
            collision_mode = rospy.get_param(
                f'{self.controller_node_name}/collision_objects_mode',
                self._collision_objects_mode
            )
            self._collision_objects_mode = collision_mode.lower() if isinstance(collision_mode, str) else collision_mode
            self._collision_objects = rospy.get_param(
                f'{self.controller_node_name}/collision_objects',
                self._collision_objects
            )
            
            # Legacy configuration (for backward compatibility)
            self._current_link = rospy.get_param(
                f'{self.controller_node_name}/target_link',
                self._current_link
            )
            self._current_obstacle = rospy.get_param(
                f'{self.controller_node_name}/target_object_id',
                self._current_obstacle
            )
            
            # Read enabled state
            self._enabled = rospy.get_param(
                f'{self.controller_node_name}/repulsive_enabled',
                self._enabled
            )
                
        except Exception as e:
            rospy.logwarn(f"Could not read repulsive config: {e}")
    
    def _parse_config_message(self, msg: str):
        """Parse the configuration message from get_config service."""
        try:
            self._enabled = "enabled=true" in msg.lower()
            
            # Parse payload_link
            if "payload_link='" in msg:
                start = msg.find("payload_link='") + len("payload_link='")
                end = msg.find("'", start)
                link = msg[start:end]
                if link != "<disabled>":
                    self._payload_link = link
                else:
                    self._payload_link = ""
            
            # Parse null_space_links (format: null_space_links=['link1', 'link2'])
            if "null_space_links=[" in msg:
                start = msg.find("null_space_links=[") + len("null_space_links=[")
                end = msg.find("]", start)
                links_str = msg[start:end]
                if links_str.strip():
                    # Extract link names from format: 'link1', 'link2'
                    self._null_space_links = [
                        s.strip().strip("'\"") 
                        for s in links_str.split(",") 
                        if s.strip()
                    ]
                else:
                    self._null_space_links = []
            
            # Parse collision_mode
            if "collision_mode=" in msg:
                start = msg.find("collision_mode=") + len("collision_mode=")
                end_comma = msg.find(",", start)
                end_space = msg.find(" ", start)
                end = min(e for e in [end_comma, end_space, len(msg)] if e > start)
                mode = msg[start:end].strip()
                self._collision_objects_mode = mode.lower()
            
            # Parse collision_objects (format: collision_objects=['obj1', 'obj2'])
            if "collision_objects=[" in msg:
                start = msg.find("collision_objects=[") + len("collision_objects=[")
                end = msg.find("]", start)
                objects_str = msg[start:end]
                if objects_str.strip():
                    self._collision_objects = [
                        s.strip().strip("'\"") 
                        for s in objects_str.split(",") 
                        if s.strip()
                    ]
                else:
                    self._collision_objects = []
            
            # Legacy: parse target_link and target_object
            if "target_link='" in msg:
                start = msg.find("target_link='") + len("target_link='")
                end = msg.find("'", start)
                link = msg[start:end]
                if link != "<all>":
                    self._current_link = link
            
            if "target_object='" in msg:
                start = msg.find("target_object='") + len("target_object='")
                end = msg.find("'", start)
                obj = msg[start:end]
                if obj != "<all>":
                    self._current_obstacle = obj
                    
        except Exception as e:
            rospy.logwarn(f"Failed to parse config message: {e}")
    
    def get_config(self) -> Dict[str, Any]:
        """
        Get the current configuration as a dictionary.
        
        Returns:
            Dict with configuration values
        """
        return {
            'enabled': self._enabled,
            'payload_link': self._payload_link,
            'null_space_links': self._null_space_links.copy(),
            'collision_objects_mode': self._collision_objects_mode,
            'collision_objects': self._collision_objects.copy(),
            # Legacy (for backward compatibility)
            'link': self._current_link,
            'obstacle': self._current_obstacle
        }
    
    @staticmethod
    def get_available_links():
        """Get list of available links."""
        return AVAILABLE_LINKS
    
    @staticmethod
    def get_available_obstacles():
        """Get list of available obstacles."""
        return AVAILABLE_OBSTACLES
    
    @staticmethod
    def get_collision_modes():
        """Get available collision object modes."""
        return [COLLISION_OBJECTS_MODE_ALL, COLLISION_OBJECTS_MODE_SPECIFIC]

