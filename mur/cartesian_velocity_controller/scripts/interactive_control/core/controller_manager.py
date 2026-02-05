"""
Controller Manager - Handles switching between ROS controllers.
"""

import rospy
from controller_manager_msgs.srv import (
    SwitchController, 
    SwitchControllerRequest,
    ListControllers
)
from typing import Optional

from ..config import (
    VELOCITY_CONTROLLER,
    MOVEIT_CONTROLLER,
    SERVICE_TIMEOUT
)


class ControllerManager:
    """
    Manages switching between different ROS controllers.
    
    Handles:
    - Switching between velocity and MoveIt controllers
    - Querying active controller state
    """
    
    # Controller type constants
    TYPE_VELOCITY = "velocity"
    TYPE_MOVEIT = "moveit"
    TYPE_UNKNOWN = "unknown"
    
    def __init__(self,
                 velocity_controller: str = VELOCITY_CONTROLLER,
                 moveit_controller: str = MOVEIT_CONTROLLER,
                 controller_manager_ns: str = "controller_manager",
                 switch_timeout_s: float = 0.0):
        """
        Initialize the controller manager.
        
        Args:
            velocity_controller: Name of the velocity controller
            moveit_controller: Name of the MoveIt trajectory controller
        """
        self.velocity_controller = velocity_controller
        self.moveit_controller = moveit_controller
        self.controller_manager_ns = controller_manager_ns
        # Matches the typical CLI usage in docs: timeout: 0.0
        self.switch_timeout_s = switch_timeout_s
        
        self._switch_srv: Optional[rospy.ServiceProxy] = None
        self._list_srv: Optional[rospy.ServiceProxy] = None
        
        self._active_controller: str = self.TYPE_UNKNOWN
        self._initialized = False
    
    def initialize(self) -> bool:
        """
        Initialize service proxies for controller management.
        
        Returns:
            True if successful
        """
        try:
            cm_ns = rospy.resolve_name(self.controller_manager_ns)
            switch_srv_name = f"{cm_ns}/switch_controller"
            list_srv_name = f"{cm_ns}/list_controllers"

            rospy.wait_for_service(switch_srv_name, timeout=SERVICE_TIMEOUT)
            self._switch_srv = rospy.ServiceProxy(
                switch_srv_name,
                SwitchController
            )
            
            rospy.wait_for_service(list_srv_name, timeout=SERVICE_TIMEOUT)
            self._list_srv = rospy.ServiceProxy(
                list_srv_name,
                ListControllers
            )
            
            self._initialized = True
            self.update_active_controller()
            return True
            
        except rospy.ROSException as e:
            rospy.logwarn(f"Controller manager not available: {e}")
            return False
    
    def is_initialized(self) -> bool:
        """Check if the manager is initialized."""
        return self._initialized
    
    def update_active_controller(self) -> str:
        """
        Query and update the currently active controller.
        
        Returns:
            Controller type string (TYPE_VELOCITY, TYPE_MOVEIT, or TYPE_UNKNOWN)
        """
        if not self._initialized or self._list_srv is None:
            self._active_controller = self.TYPE_UNKNOWN
            return self._active_controller
        
        def _matches(configured: str, reported: str) -> bool:
            """
            Match controller names robustly.

            In some setups list_controllers may return names with prefixes.
            We primarily expect exact matches, but allow suffix matches on path
            boundaries (e.g. "/mur620/<name>" vs "<name>").
            """
            if configured == reported:
                return True
            # Suffix match on slash boundary only
            return reported.endswith(f"/{configured}")

        try:
            response = self._list_srv()
            for controller in response.controller:
                if _matches(self.velocity_controller, controller.name) and controller.state == "running":
                    self._active_controller = self.TYPE_VELOCITY
                    return self._active_controller
                elif _matches(self.moveit_controller, controller.name) and controller.state == "running":
                    self._active_controller = self.TYPE_MOVEIT
                    return self._active_controller
            
            self._active_controller = self.TYPE_UNKNOWN
            return self._active_controller
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to list controllers: {e}")
            self._active_controller = self.TYPE_UNKNOWN
            return self._active_controller
    
    def get_active_controller(self) -> str:
        """
        Get the cached active controller type.
        
        Call update_active_controller() first to refresh.
        
        Returns:
            Controller type string
        """
        return self._active_controller
    
    def is_velocity_active(self) -> bool:
        """Check if velocity controller is active."""
        return self._active_controller == self.TYPE_VELOCITY
    
    def is_moveit_active(self) -> bool:
        """Check if MoveIt controller is active."""
        return self._active_controller == self.TYPE_MOVEIT
    
    def switch_to_velocity(self) -> bool:
        """
        Switch to the velocity controller.
        
        Returns:
            True if switch was successful
        """
        return self._switch_controller(
            start=self.velocity_controller,
            stop=self.moveit_controller
        )
    
    def switch_to_moveit(self) -> bool:
        """
        Switch to the MoveIt trajectory controller.
        
        Returns:
            True if switch was successful
        """
        return self._switch_controller(
            start=self.moveit_controller,
            stop=self.velocity_controller
        )
    
    def _switch_controller(self, start: str, stop: str) -> bool:
        """
        Execute a controller switch.
        
        Args:
            start: Controller to start
            stop: Controller to stop
            
        Returns:
            True if successful
        """
        if not self._initialized or self._switch_srv is None:
            rospy.logerr("Controller manager not initialized")
            return False
        
        try:
            request = SwitchControllerRequest()
            request.start_controllers = [start]
            request.stop_controllers = [stop]
            # STRICT: atomic operation - if start fails, stop won't execute
            request.strictness = SwitchControllerRequest.STRICT
            request.start_asap = False
            # Align with typical controller_manager CLI usage: timeout: 0.0
            # (0.0 means "no timeout" in practice for many setups)
            # controller_manager_msgs/SwitchController.srv expects float64 timeout (seconds), not rospy.Duration.
            request.timeout = max(0.0, float(self.switch_timeout_s))
            
            response = self._switch_srv(request)
            
            if response.ok:
                self.update_active_controller()
                rospy.loginfo(f"Switched controller: stopped {stop}, started {start}")
                return True
            else:
                rospy.logerr(
                    f"Controller switch failed. Make sure both controllers are loaded. "
                    f"Tried to start: {start}, stop: {stop}"
                )
                return False
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Controller switch service call failed: {e}")
            return False
    
    def get_controller_names(self) -> dict:
        """
        Get the configured controller names.
        
        Returns:
            Dict with 'velocity' and 'moveit' controller names
        """
        return {
            'velocity': self.velocity_controller,
            'moveit': self.moveit_controller
        }

