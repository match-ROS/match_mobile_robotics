"""
Interactive Controller - Main orchestrator for the interactive robot control system.
"""

import os
import sys
import signal
import rospy
import rospkg
from typing import Optional

from .config import (
    DEFAULT_CONTROLLER_NODE,
    DEFAULT_GLOBAL_FRAME,
    DEFAULT_EE_FRAME,
    POSES_FILE
)
from .core.ros_interface import ROSInterface
from .core.pose_manager import PoseManager
from .core.controller_manager import ControllerManager
from .controllers.velocity_controller import VelocityPoseController
from .controllers.moveit_controller import MoveItPoseController
from .controllers.repulsive_manager import RepulsiveManager
from .features.loop_movement import LoopMovement
from .ui.menu_manager import MenuManager
from .ui.terminal_utils import (
    clear_screen,
    print_header,
    print_info,
    print_success,
    print_error,
    print_warning,
    wait_for_key
)


class InteractiveController:
    """
    Main controller that orchestrates all components of the interactive
    robot control system.
    
    This class:
    - Initializes and coordinates all subcomponents
    - Provides high-level methods for common operations
    - Handles the main event loop
    - Manages graceful shutdown
    """
    
    def __init__(self,
                 controller_node_name: str = DEFAULT_CONTROLLER_NODE,
                 global_frame: str = DEFAULT_GLOBAL_FRAME,
                 ee_frame: str = DEFAULT_EE_FRAME,
                 target_pose_topic: Optional[str] = None,
                 velocity_controller: Optional[str] = None,
                 moveit_controller: Optional[str] = None,
                 controller_manager_ns: Optional[str] = None,
                 move_group_name: Optional[str] = None):
        """
        Initialize the interactive controller.
        
        Args:
            controller_node_name: Name of the cartesian velocity controller node
            global_frame: Global reference frame
            ee_frame: End effector frame
        """
        # Store config
        self._controller_node_name = controller_node_name
        self._global_frame = global_frame
        self._ee_frame = ee_frame
        self._target_pose_topic = target_pose_topic
        self._velocity_controller = velocity_controller
        self._moveit_controller = moveit_controller
        self._controller_manager_ns = controller_manager_ns
        self._move_group_name = move_group_name
        
        # Component instances (initialized in initialize())
        self.ros: Optional[ROSInterface] = None
        self.poses: Optional[PoseManager] = None
        self.controller_mgr: Optional[ControllerManager] = None
        self.velocity_ctrl: Optional[VelocityPoseController] = None
        self.moveit_ctrl: Optional[MoveItPoseController] = None
        self.repulsive: Optional[RepulsiveManager] = None
        self.loop: Optional[LoopMovement] = None
        self.menu: Optional[MenuManager] = None
        
        # State
        self._running = True
        self._initialized = False
    
    def initialize(self) -> bool:
        """
        Initialize all components.
        
        Returns:
            True if initialization was successful
        """
        try:
            # Initialize ROS interface
            self.ros = ROSInterface(
                controller_node_name=self._controller_node_name,
                global_frame=self._global_frame,
                ee_frame=self._ee_frame,
                target_pose_topic=self._target_pose_topic,
            )
            
            if not self.ros.initialize():
                print_error("Failed to initialize ROS interface")
                return False
            
            # Determine poses file path
            poses_file_path = self._find_poses_file()
            
            # Initialize pose manager
            self.poses = PoseManager(poses_file_path, default_frame_id=self.ros.global_frame)
            self.poses.load()
            
            # Initialize controller manager (allow namespacing via private params)
            velocity_ctrl = self._velocity_controller or rospy.get_param("~velocity_controller", None)
            moveit_ctrl = self._moveit_controller or rospy.get_param("~moveit_controller", None)
            controller_manager_ns = self._controller_manager_ns or rospy.get_param("~controller_manager_ns", "controller_manager")

            self.controller_mgr = ControllerManager(
                velocity_controller=velocity_ctrl or "joint_group_vel_controller",
                moveit_controller=moveit_ctrl or "vel_joint_traj_controller",
                controller_manager_ns=controller_manager_ns,
            )
            if not self.controller_mgr.initialize():
                print_warning("Controller manager not available - some features disabled")
            
            # Initialize velocity pose controller
            self.velocity_ctrl = VelocityPoseController(self.ros)
            
            # Initialize MoveIt pose controller
            move_group_name = self._move_group_name or rospy.get_param("~move_group_name", "manipulator")
            self.moveit_ctrl = MoveItPoseController(move_group_name=move_group_name)
            if MoveItPoseController.is_available():
                if not self.moveit_ctrl.initialize():
                    print_warning("MoveIt initialization failed - MoveIt features disabled")
            
            # Initialize repulsive manager
            self.repulsive = RepulsiveManager(self._controller_node_name)
            self.repulsive.initialize()
            
            # Initialize loop movement
            self.loop = LoopMovement(
                get_pose_callback=self.poses.get_pose,
                send_pose_callback=self.velocity_ctrl.send_pose,
                get_ee_pose_callback=self.ros.get_current_ee_pose,
                get_tolerance_callback=lambda: (
                    self.poses.get_position_tolerance(),
                    self.poses.get_orientation_tolerance()
                ),
                get_dwell_callback=self.poses.get_dwell_time,
                get_loop_poses_callback=self.poses.get_valid_loop_poses
            )
            
            # Initialize menu manager
            self.menu = MenuManager(self)
            
            self._initialized = True
            return True
            
        except Exception as e:
            print_error(f"Initialization failed: {e}")
            rospy.logerr(f"Initialization failed: {e}")
            return False
    
    def _find_poses_file(self) -> str:
        """Find the poses file path."""
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('cartesian_velocity_controller')
            return os.path.join(pkg_path, POSES_FILE)
        except rospkg.ResourceNotFound:
            # Fallback: relative to script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            return os.path.join(script_dir, '..', '..', POSES_FILE)
    
    # =========================================================================
    # High-Level Operations
    # =========================================================================
    
    def refresh(self, *, fast: bool = True):
        """Refresh component states.

        The interactive UI calls refresh very frequently (e.g. every time the main
        menu is rendered). A full refresh can be slow because it may:
        - call controller_manager services
        - call repulsive configuration services
        - reload YAML from disk

        Args:
            fast: If True, refresh only lightweight state (params) to keep the UI responsive.
                  If False, perform a full refresh.
        """
        if not fast:
            if self.controller_mgr:
                self.controller_mgr.update_active_controller()
            if self.repulsive:
                # Service calls may block; only do in full refresh.
                self.repulsive.refresh_config(use_service=True)
            if self.ros:
                self.ros.refresh_frame_config()
            if self.poses:
                self.poses.load()
            return

        # Fast refresh: keep UI responsive (no blocking services, no disk I/O)
        if self.repulsive:
            self.repulsive.refresh_config(use_service=False)
        if self.ros:
            self.ros.refresh_frame_config()
    
    def send_pose(self, pose_name: str) -> bool:
        """
        Send a saved pose to the robot using the appropriate controller.
        
        Args:
            pose_name: Name of the saved pose
            
        Returns:
            True if successful
        """
        pose_data = self.poses.get_pose(pose_name)
        if pose_data is None:
            print_error(f"Pose '{pose_name}' not found")
            return False
        
        # Update controller state
        self.controller_mgr.update_active_controller()
        
        if self.controller_mgr.is_moveit_active():
            print_info(f"Sending pose '{pose_name}' via MoveIt...")
            return self.moveit_ctrl.send_pose(pose_data, self.ros.global_frame)
        else:
            print_info(f"Sending pose '{pose_name}' via velocity controller...")
            return self.velocity_ctrl.send_pose(pose_data)
    
    def send_custom_pose(self, x: float, y: float, z: float,
                         qx: float = 0.0, qy: float = 0.707,
                         qz: float = 0.0, qw: float = 0.707) -> bool:
        """
        Send a custom pose to the robot.
        
        Args:
            x, y, z: Position coordinates
            qx, qy, qz, qw: Quaternion orientation
            
        Returns:
            True if successful
        """
        pose_data = {
            "position": [x, y, z],
            "orientation": [qx, qy, qz, qw]
        }
        
        self.controller_mgr.update_active_controller()
        
        if self.controller_mgr.is_moveit_active():
            print_info("Sending custom pose via MoveIt...")
            return self.moveit_ctrl.send_pose(pose_data, self.ros.global_frame)
        else:
            print_info("Sending custom pose via velocity controller...")
            return self.velocity_ctrl.send_pose(pose_data)
    
    def save_current_pose(self, name: str, description: str = "") -> bool:
        """
        Save the current EE pose.
        
        Args:
            name: Pose name
            description: Optional description
            
        Returns:
            True if successful
        """
        current_pose = self.ros.get_current_ee_pose()
        if current_pose is None:
            print_error("Cannot read current EE pose")
            return False
        
        return self.poses.add_pose(
            name=name,
            position=current_pose["position"],
            orientation=current_pose["orientation"],
            description=description,
            frame_id=current_pose.get("frame_id") or self.ros.global_frame,
            ee_frame=current_pose.get("ee_frame")
        )
    
    def start_loop(self) -> bool:
        """
        Start loop movement.
        
        Ensures velocity controller is active before starting.
        
        Returns:
            True if started successfully
        """
        # Make sure velocity controller is active for loop
        self.controller_mgr.update_active_controller()
        if not self.controller_mgr.is_velocity_active():
            print_warning("Switching to velocity controller for loop...")
            if not self.controller_mgr.switch_to_velocity():
                print_error("Cannot switch to velocity controller")
                return False
        
        return self.loop.start()
    
    # =========================================================================
    # Main Loop
    # =========================================================================
    
    def run(self):
        """Run the main interactive loop."""
        if not self._initialized:
            if not self.initialize():
                print_error("Cannot start: initialization failed")
                print_error("Make sure roscore is running")
                sys.exit(1)
        
        # Show initialization summary
        print()
        print_success("Initialization complete!")
        print_info(f"Saved poses: {self.poses.get_pose_count()}")
        print_info(f"Active controller: {self.controller_mgr.get_active_controller()}")
        print_info(f"Global frame: {self.ros.global_frame}")
        wait_for_key()
        
        # Set up signal handler
        signal.signal(signal.SIGINT, lambda s, f: self.shutdown())
        
        # Main loop
        while self._running and not rospy.is_shutdown():
            try:
                choice = self.menu.show_main_menu()
                
                if choice == "0":
                    self.shutdown()
                elif choice == "1":
                    self.menu.show_pose_menu()
                elif choice == "2":
                    self.menu.show_custom_pose_menu()
                elif choice == "3":
                    self.menu.show_save_pose_menu()
                elif choice == "4":
                    self.menu.show_controller_switch_menu()
                elif choice == "5":
                    self.menu.show_repulsive_menu()
                elif choice == "6":
                    self.menu.show_loop_menu()
                elif choice == "7":
                    self.menu.show_status()
                elif choice == "8":
                    self.menu.show_delete_pose_menu()
                elif choice == "9":
                    self.menu.show_direct_joint_velocity_menu()
                elif choice == "10":
                    self.menu.show_frame_info_menu()
                else:
                    print_error("Invalid option")
                    wait_for_key()
                    
            except rospy.ROSInterruptException:
                break
            except EOFError:
                break
            except KeyboardInterrupt:
                break
    
    def shutdown(self):
        """Perform graceful shutdown."""
        self._running = False
        
        # Stop loop if running
        if self.loop and self.loop.enabled:
            self.loop.stop()
        
        # Clean up MoveIt
        if self.moveit_ctrl:
            self.moveit_ctrl.shutdown()
        
        clear_screen()
        print_header("👋 ARRIVEDERCI!")
        print_success("Script terminated correctly.")
        print()

