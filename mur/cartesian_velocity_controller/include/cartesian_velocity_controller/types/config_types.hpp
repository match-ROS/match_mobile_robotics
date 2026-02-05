#pragma once

#include <string>
#include <vector>

namespace cartesian_velocity_controller
{

/**
 * @brief Jacobian solver configuration
 */
struct JacobianSolverConfig
{
  double singularity_threshold{0.05};
  double max_damping{0.2};
};

/**
 * @enum RepulsiveVelocityMode
 * @brief Mode for computing repulsive velocity magnitude.
 */
enum class RepulsiveVelocityMode
{
  LINEAR,        ///< Linear ramp in influence zone
  QUADRATIC,     ///< Parabolic profile in influence zone: v = v_max * r^2
  SMOOTHSTEP,    ///< C¹ smooth profile: v = v_max * (3r^2 - 2r^3)
  SMOOTHERSTEP   ///< C² smoother profile: v = v_max * (6r^5 - 15r^4 + 10r^3)
};

/**
 * @struct RepulsiveControllerConfig
 * @brief Configuration for the repulsive velocity controller.
 *
 * Supports two types of links:
 * - Payload link: Primary link where repulsive velocities are applied WITHOUT null space projection
 * - Null space links: Secondary links where repulsive velocities are applied WITH null space projection
 */
struct RepulsiveControllerConfig
{
  /// Distance thresholds
  double min_distance{0.2};       ///< Maximum repulsion at this distance
  double max_distance{1.0};       ///< No repulsion beyond this distance
  
  /// Velocity limits
  double max_linear_speed{0.5};
  double max_joint_speed{1.0};
  
  /// Primary link for repulsive velocity (NO null space projection)
  std::string payload_link;
  
  /// Secondary links for repulsive velocity (WITH null space projection)
  std::vector<std::string> null_space_repulsive_links;
  
  /// Velocity mode (linear or quadratic)
  RepulsiveVelocityMode velocity_mode{RepulsiveVelocityMode::QUADRATIC};
  
  /// Repulsive gain for quadratic mode (k in v = k/d²), 0 = auto-calibrate
  double repulsive_gain{0.0};
  
  bool enabled{true};
};

/**
 * @struct AttractiveControllerConfig
 * @brief Configuration for the attractive (PID) controller.
 */
struct AttractiveControllerConfig
{
  /// Proportional gains
  double position_gain{1.0};
  double orientation_gain{0.5};
  
  /// Integral gains
  double position_integral_gain{0.0};
  double orientation_integral_gain{0.0};
  
  /// Derivative gains
  double position_derivative_gain{0.0};
  double orientation_derivative_gain{0.0};
  
  /// Velocity limits
  double max_linear_speed{0.4};
  double max_angular_speed{0.5};
  
  /// Integral activation thresholds
  double integral_position_activation_distance{0.05};
  double integral_orientation_activation_distance{0.1};
  
  /// Feed Forward
  bool feedforward_enabled{true};
  double feedforward_gain{1.0};
  
  /// Derivative low-pass filter time constant
  double derivative_filter_tau{0.02};
  
  bool enabled{true};
};

/**
 * @struct MarkerPublisherConfig
 * @brief Configuration for RViz marker visualization.
 */
struct MarkerPublisherConfig
{
  double arrow_scale{1.0};
  std::string global_frame{"world"};
  /// Optional prefix added to visualization_msgs/Marker.ns (e.g. "ur10_l", "ur10_r")
  /// When set, namespaces become "<ns_prefix>/<base_ns>" so RViz can show each arm separately.
  std::string ns_prefix{""};
};

/**
 * @struct VelocityLimits
 * @brief Velocity filter limits for a single DOF or Cartesian axis.
 */
struct VelocityLimits
{
  double max_velocity{1.0};       ///< Maximum velocity (rad/s or m/s)
  double max_acceleration{5.0};   ///< Maximum acceleration (rad/s² or m/s²)
  double max_jerk{50.0};          ///< Maximum jerk (rad/s³ or m/s³)

  static VelocityLimits create(double vel, double accel, double jerk)
  {
    VelocityLimits limits;
    limits.max_velocity = vel;
    limits.max_acceleration = accel;
    limits.max_jerk = jerk;
    return limits;
  }
};

/**
 * @struct RepulsivePointConfig
 * @brief Configuration for a repulsive Point of Interest (POI).
 *
 * Each POI on the robot (tcp, elbow, wrist, forearm_mid) can be configured
 * with individual weight, radius, and enabled state. The configuration can
 * be modified at runtime via dynamic_reconfigure or internal C++ logic.
 */
struct RepulsivePointConfig
{
  std::string name;           ///< POI identifier (e.g., "tcp", "elbow", "wrist")
  double weight{1.0};         ///< Weight of the repulsive contribution (0.0 - 2.0)
  double radius{0.05};        ///< Influence radius (m) - inflates the POI sphere
  bool enabled{true};         ///< Whether this POI is active
  bool is_tcp{false};         ///< true = generates ObstacleInfo, false = generates LinkPOI
};

}  // namespace cartesian_velocity_controller
