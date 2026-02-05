"""
Configuration constants for the interactive control system.
"""

from typing import List

# =============================================================================
# CONTROLLER CONFIGURATION
# =============================================================================

# Controller names used by controller_manager
VELOCITY_CONTROLLER: str = "joint_group_vel_controller"
MOVEIT_CONTROLLER: str = "vel_joint_traj_controller"

# Default controller node name
DEFAULT_CONTROLLER_NODE: str = "cartesian_velocity_controller"

# =============================================================================
# FILE PATHS
# =============================================================================

# Poses file (relative to package root)
POSES_FILE: str = "config/saved_poses.yaml"

# =============================================================================
# FRAME CONFIGURATION
# =============================================================================

# Default reference frames
# NOTE: for MUR/MiR-based setups the TF tree is usually rooted at base_link.
# Using "world" as default causes TF lookup failures unless such frame exists.
DEFAULT_GLOBAL_FRAME: str = "base_link"
DEFAULT_EE_FRAME: str = "tool0"

# =============================================================================
# ROBOT LINKS
# =============================================================================

# Available links for repulsive velocity monitoring
AVAILABLE_LINKS: List[str] = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
]

# =============================================================================
# OBSTACLES / COLLISION OBJECTS
# =============================================================================

# Predefined obstacle names (for backward compatibility)
AVAILABLE_OBSTACLES: List[str] = [
    "sphere_1",
    "sphere_2",
    "sphere_3",
    "box_1",
    "box_2",
    "cylinder_1",
    "obstacle_1",
    "obstacle_2",
]

# =============================================================================
# REPULSIVE VELOCITY CONFIGURATION
# =============================================================================

# Collision objects mode
COLLISION_OBJECTS_MODE_ALL: str = "all"
COLLISION_OBJECTS_MODE_SPECIFIC: str = "specific"

# =============================================================================
# LOOP MOVEMENT DEFAULTS
# =============================================================================

DEFAULT_POSITION_TOLERANCE: float = 0.02  # meters
DEFAULT_ORIENTATION_TOLERANCE: float = 0.1  # radians
DEFAULT_DWELL_TIME: float = 0.5  # seconds

# =============================================================================
# TIMING
# =============================================================================

# Timeout for waiting for subscriber connections
PUBLISHER_CONNECTION_TIMEOUT: float = 2.0  # seconds

# Timeout for service calls
SERVICE_TIMEOUT: float = 2.0  # seconds

# TF lookup timeout used by the interactive UI.
# Keep it small to avoid blocking the terminal menus when TF isn't available.
TF_LOOKUP_TIMEOUT: float = 0.05  # seconds

# Cache duration for displaying the EE pose in the UI (avoid multiple TF queries per render)
EE_POSE_CACHE_DURATION: float = 0.20  # seconds

# Loop check interval
LOOP_CHECK_INTERVAL: float = 0.05  # seconds (50ms)

