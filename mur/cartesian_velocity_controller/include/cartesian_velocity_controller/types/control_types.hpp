#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>
#include <limits>

#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @struct PipelineDebugData
 * @brief Comprehensive debug data structure for the entire pipeline.
 * 
 * This structure follows the pipeline architecture:
 * Level A: Global Planner -> Level B: Local Planner -> Level C: Motion Generator -> 
 * Level D: PID + IK -> Safety Limiter
 */
struct PipelineDebugData
{
  // =========================================================================
  // LEVEL A: GLOBAL PLANNER
  // =========================================================================
  
  /// Distance from active waypoint to current TCP position [m]
  double distance_waypoint_to_current{0.0};
  
  /// Active waypoint index
  int active_waypoint_index{0};
  
  /// Total number of waypoints
  int total_waypoints{0};
  
  /// Active waypoint pose
  Eigen::Isometry3d active_waypoint{Eigen::Isometry3d::Identity()};
  
  /// Current TCP pose
  Eigen::Isometry3d current_pose{Eigen::Isometry3d::Identity()};

  // =========================================================================
  // LEVEL B: LOCAL PLANNER (Virtual Point with Artificial Fields)
  // =========================================================================
  
  /// Distance from P_target_raw to active waypoint [m]
  double distance_target_raw_to_waypoint{0.0};
  
  /// Distance from P_target_raw to current pose [m]
  double distance_target_raw_to_current{0.0};
  
  /// P_target_raw pose (raw target from local planner)
  Eigen::Isometry3d target_raw{Eigen::Isometry3d::Identity()};
  
  /// Attractive velocity toward waypoint (V_goal)
  Eigen::Vector3d v_goal_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_goal_angular{Eigen::Vector3d::Zero()};
  
  /// Repulsive velocity from obstacles at TCP (V_obs)
  Eigen::Vector3d v_obs_linear{Eigen::Vector3d::Zero()};
  
  /// Repulsive velocity from link POIs (V_link) - converted back to TCP
  Eigen::Vector3d v_link_linear{Eigen::Vector3d::Zero()};
  
  /// Combined desired velocity (V_desired)
  Eigen::Vector3d v_desired_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_desired_angular{Eigen::Vector3d::Zero()};
  
  /// Gains used for combination
  double k_attractive{0.0};
  double k_repulsive_tcp{0.0};
  double k_repulsive_links{0.0};

  /// Virtual Target Leash scaling factor (1.0 = full speed, 0.0 = stopped)
  double virtual_target_scaling_factor{1.0};
  
  /// Obstacle info (summary)
  double closest_obstacle_distance{std::numeric_limits<double>::infinity()};
  std::string closest_obstacle_id;
  std::string closest_link_name;

  // =========================================================================
  // POI REPULSION DEBUG
  // =========================================================================
  
  /// Array of LinkPOI with computed repulsive velocities (for detailed debug)
  std::vector<LinkPOI> poi_debug_data;
  
  /// Total repulsive joint velocity from all link POIs
  Eigen::VectorXd repulsive_joint_velocity;

  // =========================================================================
  // LEVEL C: MOTION GENERATOR (Second Order Filter)
  // =========================================================================
  
  /// Distance from P_target_filtered to current robot pose [m]
  double distance_target_filtered_to_current{0.0};
  
  /// P_target_filtered pose
  Eigen::Isometry3d target_filtered{Eigen::Isometry3d::Identity()};
  
  /// Filtered velocity output
  Eigen::Vector3d v_filtered_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_filtered_angular{Eigen::Vector3d::Zero()};
  
  /// Filtered acceleration output
  Eigen::Vector3d acceleration_filtered_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_filtered_angular{Eigen::Vector3d::Zero()};
  
  /// Filtered jerk output
  Eigen::Vector3d jerk_filtered_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_filtered_angular{Eigen::Vector3d::Zero()};
  
  /// Filter time constant (tau)
  double filter_tau{0.0};

  // =========================================================================
  // LEVEL D: PID CONTROLLER
  // =========================================================================
  
  /// Position error (P_target_filtered - P_current)
  Eigen::Vector3d pid_position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_orientation_error{Eigen::Vector3d::Zero()};
  
  /// Error norms
  double pid_position_error_norm{0.0};
  double pid_orientation_error_norm{0.0};
  
  /// PID gains (current values)
  double pid_kp_position{0.0};
  double pid_ki_position{0.0};
  double pid_kd_position{0.0};
  double pid_kp_orientation{0.0};
  double pid_ki_orientation{0.0};
  double pid_kd_orientation{0.0};
  
  /// PID components - Linear
  Eigen::Vector3d pid_p_term_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_i_term_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_d_term_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_integral_linear{Eigen::Vector3d::Zero()};
  
  /// PID components - Angular
  Eigen::Vector3d pid_p_term_angular{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_i_term_angular{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_d_term_angular{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_integral_angular{Eigen::Vector3d::Zero()};
  
  /// Feed-forward velocity (V_ff = V_filtered)
  Eigen::Vector3d pid_feedforward_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_feedforward_angular{Eigen::Vector3d::Zero()};
  
  /// Total PID output velocity
  Eigen::Vector3d pid_output_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pid_output_angular{Eigen::Vector3d::Zero()};
  
  /// Total cartesian velocity command (feedforward + pid)
  Eigen::Vector3d cartesian_cmd_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cartesian_cmd_angular{Eigen::Vector3d::Zero()};

  // =========================================================================
  // LEVEL D: JACOBIAN INVERSE KINEMATICS
  // =========================================================================
  
  /// Maximum damping factor applied across singular directions (SDLS)
  double jacobian_damping_factor{0.0};
  
  /// Minimum singular value of Jacobian
  double jacobian_min_singular_value{0.0};

  /// Singular values of the (weighted) Jacobian
  Eigen::VectorXd jacobian_singular_values;

  /// Per-direction damping factors (λᵢ) from SDLS
  Eigen::VectorXd jacobian_damping_factors;

  /// Joint weights used for weighted pseudo-inverse
  Eigen::VectorXd joint_weights;
  
  /// Joint velocity command from pseudo-inverse (before safety limiter)
  Eigen::VectorXd joint_velocity_from_ik;

  // =========================================================================
  // SAFETY LIMITER
  // =========================================================================
  
  /// Joint velocity command after safety limiter
  Eigen::VectorXd joint_velocity_after_limiter;

  /// Joint acceleration derived from command [rad/s^2]
  Eigen::VectorXd joint_acceleration_command;
  
  /// Scaling factor applied by safety limiter (1.0 = no scaling)
  double safety_scaling_factor{1.0};
  
  /// Which limit triggered the scaling (empty if no scaling)
  std::string safety_limiting_reason;
  
  /// Joint names (for reference)
  std::vector<std::string> joint_names;

  /// Mutex for thread-safe access
  mutable std::mutex mutex;
};

// Keep ControllerDebugData as alias for backward compatibility during transition
using ControllerDebugData = PipelineDebugData;

}  // namespace cartesian_velocity_controller
