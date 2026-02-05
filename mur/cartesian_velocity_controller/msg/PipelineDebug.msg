# Pipeline Debug message
# Contains all internal data for pipeline analysis
# Follows the architecture: Global Planner -> Local Planner -> Motion Generator -> PID + IK -> Safety Limiter

std_msgs/Header header

# =============================================================================
# LEVEL A: GLOBAL PLANNER
# =============================================================================

# Distance from active waypoint to current TCP position [m]
float64 distance_waypoint_to_current

# Active waypoint index
int32 active_waypoint_index

# Total number of waypoints
int32 total_waypoints

# Active waypoint pose
geometry_msgs/Pose active_waypoint

# Current TCP pose
geometry_msgs/Pose current_pose

# =============================================================================
# LEVEL B: LOCAL PLANNER (Virtual Point with Artificial Fields)
# =============================================================================

# Distance from P_target_raw to active waypoint [m]
float64 distance_target_raw_to_waypoint

# Distance from P_target_raw to current pose [m]
float64 distance_target_raw_to_current

# P_target_raw pose (raw target from local planner)
geometry_msgs/Pose target_raw

# --- V_desired composition ---

# Attractive velocity toward waypoint (V_goal)
geometry_msgs/Vector3 v_goal_linear
geometry_msgs/Vector3 v_goal_angular

# Repulsive velocity from obstacles at TCP (V_obs)
geometry_msgs/Vector3 v_obs_linear

# Repulsive velocity from link POIs (V_link) - converted back to TCP
geometry_msgs/Vector3 v_link_linear

# Combined desired velocity (V_desired = k_att*V_goal + k_rep*V_obs + k_rep_link*V_link)
geometry_msgs/Vector3 v_desired_linear
geometry_msgs/Vector3 v_desired_angular

# Gains used for combination
float64 k_attractive
float64 k_repulsive_tcp
float64 k_repulsive_links

# Virtual Target Leash (Soft Leash)
float64 virtual_target_scaling_factor

# Obstacle info (summary)
float64 closest_obstacle_distance
string closest_obstacle_id
string closest_link_name

# =============================================================================
# POI REPULSION DEBUG (Per-POI detailed information)
# =============================================================================

# Array of detailed info for each active POI
POIDebugInfo[] poi_debug_info

# Number of active POIs (within influence distance)
uint32 active_poi_count

# Total repulsive joint velocity from all link POIs (before safety limiter)
float64[] repulsive_joint_velocity

# =============================================================================
# LEVEL C: MOTION GENERATOR (Second Order Filter)
# =============================================================================

# Distance from P_target_filtered to current robot pose [m]
float64 distance_target_filtered_to_current

# P_target_filtered pose
geometry_msgs/Pose target_filtered

# Filtered velocity output
geometry_msgs/Vector3 v_filtered_linear
geometry_msgs/Vector3 v_filtered_angular

# Filtered acceleration output
geometry_msgs/Vector3 acceleration_filtered_linear
geometry_msgs/Vector3 acceleration_filtered_angular

# Filtered jerk output
geometry_msgs/Vector3 jerk_filtered_linear
geometry_msgs/Vector3 jerk_filtered_angular

# Filter time constant (tau)
float64 filter_tau

# =============================================================================
# LEVEL D: PID CONTROLLER
# =============================================================================

# Position error (P_target_filtered - P_current)
geometry_msgs/Vector3 pid_position_error
geometry_msgs/Vector3 pid_orientation_error

# Error norms
float64 pid_position_error_norm
float64 pid_orientation_error_norm

# PID gains (current values)
float64 pid_kp_position
float64 pid_ki_position
float64 pid_kd_position
float64 pid_kp_orientation
float64 pid_ki_orientation
float64 pid_kd_orientation

# PID components - Linear
geometry_msgs/Vector3 pid_p_term_linear
geometry_msgs/Vector3 pid_i_term_linear
geometry_msgs/Vector3 pid_d_term_linear
geometry_msgs/Vector3 pid_integral_linear

# PID components - Angular
geometry_msgs/Vector3 pid_p_term_angular
geometry_msgs/Vector3 pid_i_term_angular
geometry_msgs/Vector3 pid_d_term_angular
geometry_msgs/Vector3 pid_integral_angular

# Feed-forward velocity (V_ff = V_filtered)
geometry_msgs/Vector3 pid_feedforward_linear
geometry_msgs/Vector3 pid_feedforward_angular

# Total PID output velocity
geometry_msgs/Vector3 pid_output_linear
geometry_msgs/Vector3 pid_output_angular

# Total cartesian velocity command (feedforward + pid)
geometry_msgs/Vector3 cartesian_cmd_linear
geometry_msgs/Vector3 cartesian_cmd_angular

# =============================================================================
# LEVEL D: JACOBIAN INVERSE KINEMATICS
# =============================================================================

# Maximum damping factor applied across singular directions (SDLS)
float64 jacobian_damping_factor

# Minimum singular value of Jacobian
float64 jacobian_min_singular_value

# Singular values of the (weighted) Jacobian
float64[] jacobian_singular_values

# Per-direction damping factors (λᵢ) from SDLS
float64[] jacobian_damping_factors

# Joint weights used for weighted pseudo-inverse
float64[] joint_weights

# Joint velocity command from pseudo-inverse (before safety limiter)
float64[] joint_velocity_from_ik

# =============================================================================
# SAFETY LIMITER
# =============================================================================

# Joint velocity command after safety limiter
float64[] joint_velocity_after_limiter

# Joint acceleration derived from command [rad/s^2]
float64[] joint_acceleration_command

# Scaling factor applied by safety limiter (1.0 = no scaling)
float64 safety_scaling_factor

# Which limit triggered the scaling (empty if no scaling)
string safety_limiting_reason

# Joint names (for reference)
string[] joint_names
