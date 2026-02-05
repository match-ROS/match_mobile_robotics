#pragma once

/**
 * @file pipeline_types.hpp
 * @brief Types for the new 4-level velocity pipeline architecture.
 *
 * This file defines the data structures used throughout the refactored
 * velocity command pipeline:
 * - Level A: GlobalPlanner (waypoint management)
 * - Level B: LocalPlanner (attractive/repulsive velocity computation)
 * - Level C: MotionGenerator (velocity filtering with τ)
 * - Level D: PID + IK (with feed forward)
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

// Include config_types.hpp for RepulsiveVelocityMode enum
#include "cartesian_velocity_controller/types/config_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @struct ObstacleInfo
 * @brief Information about an obstacle for repulsive velocity computation.
 *
 * Contains pre-computed distance information for efficient repulsive
 * velocity calculation in the LocalPlanner.
 */
struct ObstacleInfo
{
  /// Unique identifier for the obstacle
  std::string id;

  /// Position of the closest point on the obstacle (world frame)
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};

  /// Pre-computed distance from the reference point to this obstacle (effective: surface-to-surface)
  double distance{std::numeric_limits<double>::infinity()};

  /// Raw distance (center-to-center) before radius subtraction
  double distance_raw{std::numeric_limits<double>::infinity()};

  /// Vector from obstacle to reference point (for repulsive direction)
  Eigen::Vector3d distance_vector{Eigen::Vector3d::Zero()};

  /// Influence distance for this obstacle (beyond this, no repulsion)
  double influence_distance{1.0};

  /// Minimum safe distance (maximum repulsion at this distance)
  double min_safe_distance{0.05};

  /// Characteristic radius of the obstacle
  double object_characteristic_radius{0.0};

  /// Radius of the POI (robot body inflation)
  double poi_radius{0.05};

  /**
   * @brief Check if the obstacle is within influence range.
   * @return true if distance < influence_distance
   */
  bool isInInfluenceRange() const
  {
    return distance < influence_distance;
  }

  /**
   * @brief Get normalized repulsive direction (away from obstacle).
   * @return Unit vector pointing away from obstacle
   */
  Eigen::Vector3d getRepulsiveDirection() const
  {
    double norm = distance_vector.norm();
    if (norm < 1e-10)
    {
      return Eigen::Vector3d::Zero();
    }
    return distance_vector / norm;
  }
};

/**
 * @struct LinkPOI
 * @brief Point of Interest on a robot link for skeleton-based repulsion.
 *
 * Represents a monitoring point on an intermediate link (elbow, wrist, etc.)
 * for computing repulsive velocities that protect the robot body, not just
 * the TCP/payload.
 */
struct LinkPOI
{
  /// Name of the POI (e.g., "tcp", "elbow", "wrist", "forearm_mid")
  std::string point_name;

  /// Name of the link this POI belongs to
  std::string link_name;

  /// Position of the POI in world frame
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};

  /// Position of the POI in link frame (constant offset)
  Eigen::Vector3d position_link{Eigen::Vector3d::Zero()};

  /// Distance to the closest obstacle from this POI (effective: surface-to-surface)
  double distance_to_closest_obstacle{std::numeric_limits<double>::infinity()};

  /// Raw distance (center-to-center) before radius subtraction
  double distance_raw{std::numeric_limits<double>::infinity()};

  /// Vector from POI to obstacle center (world frame)
  Eigen::Vector3d distance_vector{Eigen::Vector3d::Zero()};

  /// Direction for repulsive velocity (normalized, world frame, away from obstacle)
  Eigen::Vector3d repulsive_direction{Eigen::Vector3d::Zero()};

  /// ID of the closest obstacle to this POI
  std::string closest_obstacle_id;

  /// Weight for this POI's contribution (0.0 - 2.0)
  double weight{1.0};

  /// Radius of this POI (robot body inflation)
  double poi_radius{0.05};

  /// Characteristic radius of the closest obstacle
  double object_characteristic_radius{0.0};

  /// Repulsive velocity in Cartesian space BEFORE Jacobian projection (world frame)
  Eigen::Vector3d repulsive_velocity{Eigen::Vector3d::Zero()};

  /// Magnitude of the repulsive velocity [m/s]
  double repulsive_velocity_magnitude{0.0};

  /**
   * @brief Check if this POI has an active repulsion.
   * @param influence_distance The influence distance threshold
   * @return true if there's an obstacle within influence range
   */
  bool hasActiveRepulsion(double influence_distance) const
  {
    return distance_to_closest_obstacle < influence_distance;
  }
};

/**
 * @struct WaypointInfo
 * @brief Extended waypoint information with metadata.
 */
struct WaypointInfo
{
  /// The 6D pose of this waypoint
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};

  /// Optional name/identifier for this waypoint
  std::string name;

  /// Optional custom switch distance for this waypoint (0 = use default)
  double switch_distance{0.0};

  /// Optional custom orientation threshold for this waypoint (0 = use default)
  double orientation_threshold{0.0};

  /// Whether orientation should be considered for switching from this waypoint
  bool use_orientation_for_switch{true};

  /**
   * @brief Construct a WaypointInfo from just a pose.
   * @param p The pose
   */
  explicit WaypointInfo(const Eigen::Isometry3d& p)
    : pose(p)
  {
  }

  /**
   * @brief Default constructor.
   */
  WaypointInfo() = default;
};

/**
 * @struct LocalPlannerOutput
 * @brief Output from the LocalPlanner computation.
 *
 * Contains all velocity components and the integrated target position.
 */
struct LocalPlannerOutput
{
  /// Attractive velocity toward waypoint (linear, world frame)
  Eigen::Vector3d attractive_linear{Eigen::Vector3d::Zero()};

  /// Attractive velocity toward waypoint (angular, world frame)
  Eigen::Vector3d attractive_angular{Eigen::Vector3d::Zero()};

  /// Repulsive velocity from TCP/payload obstacles (linear, world frame)
  Eigen::Vector3d repulsive_obstacle_linear{Eigen::Vector3d::Zero()};

  /// Repulsive velocities from link POIs (already converted to joint space)
  Eigen::VectorXd repulsive_links_joint;

  /// Combined desired velocity (attractive + repulsive) - linear
  Eigen::Vector3d combined_linear{Eigen::Vector3d::Zero()};

  /// Combined desired velocity (attractive + repulsive) - angular
  Eigen::Vector3d combined_angular{Eigen::Vector3d::Zero()};

  /// Integrated target position (virtual point)
  Eigen::Isometry3d target_raw{Eigen::Isometry3d::Identity()};

  /// Distance to current waypoint
  double distance_to_waypoint{std::numeric_limits<double>::infinity()};

  /// Angular distance to current waypoint (rad)
  double angular_distance_to_waypoint{0.0};

  /// Closest obstacle distance (for diagnostics)
  double closest_obstacle_distance{std::numeric_limits<double>::infinity()};

  /// Link POIs with computed repulsive velocities (for debug/visualization)
  std::vector<LinkPOI> link_pois_with_velocities;

  /// Scaling factor applied by Virtual Target Leash (1.0 = no scaling, 0.0 = stopped)
  double virtual_target_scaling_factor{1.0};
};

/**
 * @struct MotionGeneratorOutput
 * @brief Output from the MotionGenerator (velocity filter).
 */
struct MotionGeneratorOutput
{
  /// Filtered linear velocity
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};

  /// Filtered angular velocity
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};

  /// Filtered linear acceleration
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};

  /// Filtered angular acceleration
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};

  /// Filtered target pose
  Eigen::Isometry3d filtered_pose{Eigen::Isometry3d::Identity()};

  /// Current time constant (τ) used
  double tau{0.1};
};

/**
 * @struct PIDControllerOutput
 * @brief Output from the Cartesian PID controller.
 */
struct PIDControllerOutput
{
  /// Computed linear velocity from PID
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};

  /// Computed angular velocity from PID
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};

  /// Joint velocities after IK conversion
  Eigen::VectorXd joint_velocity;

  /// Position error (for diagnostics)
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};

  /// Orientation error as axis-angle (for diagnostics)
  Eigen::Vector3d orientation_error{Eigen::Vector3d::Zero()};
};

/**
 * @struct SafetyLimiterOutput
 * @brief Output from the JointSafetyLimiter.
 */
struct SafetyLimiterOutput
{
  /// Final joint velocities (after uniform scaling)
  Eigen::VectorXd joint_velocity;

  /// Scaling factor applied (1.0 = no scaling, <1.0 = limited)
  double scaling_factor{1.0};

  /// Which limit triggered the scaling (for diagnostics)
  enum class LimitType
  {
    NONE,
    VELOCITY,
    ACCELERATION
  } limit_type{LimitType::NONE};

  /// Index of the joint that triggered the limit (-1 if none)
  int limiting_joint{-1};
};

// Note: RepulsiveVelocityMode is defined in config_types.hpp to avoid duplication

}  // namespace cartesian_velocity_controller

