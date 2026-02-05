#pragma once

/**
 * @file local_planner.hpp
 * @brief Local Planner for velocity field computation (Level B of pipeline).
 *
 * The LocalPlanner computes the desired velocity by combining:
 * 1. Attractive velocity toward the current waypoint
 * 2. Repulsive velocity from obstacles (TCP/payload)
 * 3. Repulsive velocity from link POIs (skeleton-based protection)
 *
 * The combined velocity is integrated to produce a "virtual target" position
 * that smoothly moves toward the waypoint while avoiding obstacles.
 *
 * Features:
 * - Quadratic repulsive velocity (1/d²) for natural obstacle avoidance
 * - Dynamic gains for fuzzy control integration
 * - Separate handling of TCP and link POI repulsion
 * - Velocity magnitude limiting
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <memory>
#include <vector>

#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

// Forward declarations
class RobotStateManager;
class JacobianSolver;

/**
 * @class LocalPlanner
 * @brief Computes velocity fields and integrates the virtual target position.
 *
 * The LocalPlanner is the second level (Level B) of the pipeline. It receives
 * the current waypoint from the GlobalPlanner and the robot's current pose,
 * then computes desired velocities using potential field methods.
 *
 * The key innovation is the integration of the target position:
 * P_target_raw = P_prev + V_desired * dt
 *
 * This creates a "virtual point" that moves smoothly, which is then tracked
 * by the PID controller in Level D.
 */
class LocalPlanner
{
public:
  /// Small constant to avoid division by zero
  static constexpr double kEpsilon = 1e-10;

  /**
   * @brief Construct a LocalPlanner.
   * @param robot_state Shared pointer to robot state manager (for POI velocity conversion)
   * @param solver Shared pointer to Jacobian solver (for POI velocity conversion)
   */
  LocalPlanner(std::shared_ptr<RobotStateManager> robot_state = nullptr,
               std::shared_ptr<JacobianSolver> solver = nullptr);

  /**
   * @brief Destructor.
   */
  ~LocalPlanner() = default;

  // ============== Main Computation ==============

  /**
   * @brief Compute all velocity components and update the virtual target.
   * @param current_pose Current TCP pose of the robot
   * @param waypoint Target waypoint pose
   * @param obstacles Vector of obstacles for TCP/payload repulsion
   * @param link_pois Vector of link POIs for skeleton-based repulsion
   * @param dt Time step since last call (seconds)
   * @return LocalPlannerOutput with all computed velocities and integrated target
   *
   * This is the main computation method called on each control cycle.
   */
  LocalPlannerOutput compute(
      const Eigen::Isometry3d& current_pose,
      const Eigen::Isometry3d& waypoint,
      const std::vector<ObstacleInfo>& obstacles,
      const std::vector<LinkPOI>& link_pois,
      double dt);

  // ============== Dynamic Parameters (for Fuzzy Control) ==============

  /**
   * @brief Set the attractive gain.
   * @param k_att Gain for attractive velocity (default: 1.0)
   *
   * The attractive velocity is: V_att = k_att * normalize(waypoint - current_pos)
   */
  void setAttractiveGain(double k_att);

  /**
   * @brief Get the current attractive gain.
   * @return Attractive gain
   */
  double getAttractiveGain() const;

  /**
   * @brief Set the repulsive gain for TCP/payload obstacles.
   * @param k_rep Gain for repulsive velocity (default: 1.0)
   */
  void setRepulsiveObstacleGain(double k_rep);

  /**
   * @brief Get the current repulsive obstacle gain.
   * @return Repulsive obstacle gain
   */
  double getRepulsiveObstacleGain() const;

  /**
   * @brief Set the repulsive gain for link POIs.
   * @param k_rep_link Gain for link repulsive velocity (default: 0.5)
   */
  void setRepulsiveLinkGain(double k_rep_link);

  /**
   * @brief Get the current repulsive link gain.
   * @return Repulsive link gain
   */
  double getRepulsiveLinkGain() const;

  // ============== Velocity Limits ==============

  /**
   * @brief Set the maximum linear velocity magnitude.
   * @param max_vel Maximum velocity in m/s
   */
  void setMaxLinearVelocity(double max_vel);

  /**
   * @brief Get the maximum linear velocity.
   * @return Maximum linear velocity in m/s
   */
  double getMaxLinearVelocity() const;

  /**
   * @brief Set the maximum angular velocity magnitude.
   * @param max_vel Maximum velocity in rad/s
   */
  void setMaxAngularVelocity(double max_vel);

  /**
   * @brief Get the maximum angular velocity.
   * @return Maximum angular velocity in rad/s
   */
  double getMaxAngularVelocity() const;

  /**
   * @brief Set thresholds below which target integration is frozen to avoid jitter chasing.
   * @param linear_thresh Linear velocity magnitude threshold (m/s)
   * @param angular_thresh Angular velocity magnitude threshold (rad/s)
   */
  void setIntegrationFreezeThresholds(double linear_thresh, double angular_thresh);

  // ============== Repulsive Parameters ==============

  /**
   * @brief Set the influence distance for repulsion.
   * @param distance Distance in meters beyond which no repulsion occurs
   */
  void setInfluenceDistance(double distance);

  /**
   * @brief Get the influence distance.
   * @return Influence distance in meters
   */
  double getInfluenceDistance() const;

  /**
   * @brief Set the minimum safe distance.
   * @param distance Minimum distance in meters (maximum repulsion at this distance)
   */
  void setMinSafeDistance(double distance);

  /**
   * @brief Get the minimum safe distance.
   * @return Minimum safe distance in meters
   */
  double getMinSafeDistance() const;

  /**
   * @brief Set the repulsive velocity mode.
   * @param mode LINEAR or QUADRATIC
   */
  void setRepulsiveMode(RepulsiveVelocityMode mode);

  /**
   * @brief Get the repulsive velocity mode.
   * @return Current mode
   */
  RepulsiveVelocityMode getRepulsiveMode() const;

  // ============== Repulsive Smoothing (3.1 + 3.3) ==============

  /**
   * @brief Configure asymmetric EMA smoothing for repulsive velocities.
   *
   * If both taus are <= 0, the filter is disabled.
   * - tau_rise: applied when |v_raw| increases (fast reaction)
   * - tau_fall: applied when |v_raw| decreases (slow release)
   */
  void setRepulsiveVelocityFilterTaus(double tau_rise, double tau_fall);
  void getRepulsiveVelocityFilterTaus(double& tau_rise, double& tau_fall) const;

  /**
   * @brief Configure asymmetric rate limiter for repulsive velocities.
   *
   * If both max accelerations are <= 0, the limiter is disabled.
   * - max_acc_rise: allowed acceleration when |v_target| increases
   * - max_acc_fall: allowed acceleration when |v_target| decreases
   */
  void setRepulsiveVelocityMaxAccelerations(double max_acc_rise, double max_acc_fall);
  void getRepulsiveVelocityMaxAccelerations(double& max_acc_rise, double& max_acc_fall) const;

  // ============== Virtual Target Leash (Soft Leash) ==============

  /**
   * @brief Enable or disable the virtual target leash.
   * @param enabled true to enable
   * 
   * The leash prevents the virtual target from moving too far ahead of the robot
   * (overshoot) when the robot is lagging behind (e.g. due to dynamics or obstacles).
   */
  void setVirtualTargetLeashEnabled(bool enabled);

  /**
   * @brief Set parameters for the virtual target leash.
   * @param start_dist Distance (m) where scaling starts (scale = 1.0)
   * @param stop_dist Distance (m) where scaling ends (scale = 0.0)
   * @param reset_thresh Distance (m) where hard reset is triggered
   */
  void setVirtualTargetLeashParams(double start_dist, double stop_dist, double reset_thresh);

  /**
   * @brief Get whether the leash is enabled.
   */
  bool getVirtualTargetLeashEnabled() const;

  /**
   * @brief Get leash parameters.
   */
  void getVirtualTargetLeashParams(double& start_dist, double& stop_dist, double& reset_thresh) const;

  // ============== State Management ==============

  /**
   * @brief Reset the planner state.
   *
   * Clears the integrated target position. Call this when starting a new motion
   * or when the robot has reached its target.
   */
  void reset();

  /**
   * @brief Reset the virtual target to a specific pose.
   * @param pose Initial pose for the virtual target
   */
  void resetToPosition(const Eigen::Isometry3d& pose);

  /**
   * @brief Check if the planner has been initialized with a position.
   * @return true if initialized
   */
  bool isInitialized() const;

  /**
   * @brief Get the current virtual target position.
   * @return Current integrated target pose
   */
  Eigen::Isometry3d getVirtualTarget() const;

private:
  static Eigen::Vector3d applyAsymmetricEma(
      const Eigen::Vector3d& v_raw,
      const Eigen::Vector3d& v_prev_filtered,
      double dt,
      double tau_rise,
      double tau_fall);

  static Eigen::Vector3d applyAsymmetricRateLimiter(
      const Eigen::Vector3d& v_target,
      const Eigen::Vector3d& v_prev_out,
      double dt,
      double max_acc_rise,
      double max_acc_fall);

  /**
   * @brief Compute attractive velocity toward waypoint.
   * @param current_pos Current position (translation only)
   * @param waypoint_pos Waypoint position (translation only)
   * @return Attractive linear velocity vector
   */
  Eigen::Vector3d computeAttractiveLinearVelocity(
      const Eigen::Vector3d& current_pos,
      const Eigen::Vector3d& waypoint_pos) const;

  /**
   * @brief Compute attractive angular velocity toward waypoint.
   * @param current_orientation Current orientation
   * @param waypoint_orientation Target orientation
   * @return Attractive angular velocity vector (axis-angle)
   */
  Eigen::Vector3d computeAttractiveAngularVelocity(
      const Eigen::Quaterniond& current_orientation,
      const Eigen::Quaterniond& waypoint_orientation) const;

  /**
   * @brief Compute repulsive velocity from a single obstacle.
   * @param point Point to repel from (TCP position)
   * @param obstacle Obstacle information
   * @return Repulsive velocity contribution
   */
  Eigen::Vector3d computeRepulsiveVelocityFromObstacle(
      const Eigen::Vector3d& point,
      const ObstacleInfo& obstacle) const;

  /**
   * @brief Compute total repulsive velocity from all obstacles.
   * @param point Point to repel from
   * @param obstacles Vector of all obstacles
   * @return Combined repulsive velocity
   */
  Eigen::Vector3d computeRepulsiveVelocityTotal(
      const Eigen::Vector3d& point,
      const std::vector<ObstacleInfo>& obstacles) const;

  /**
   * @brief Compute repulsive joint velocity from link POIs.
   * @param link_pois Vector of link POIs with their distances (will be updated with computed velocities)
   * @return Joint velocity vector
   *
   * This method uses the partial Jacobian for each POI to convert the
   * Cartesian repulsive velocity to joint space.
   * The repulsive_velocity and repulsive_velocity_magnitude fields of each POI
   * are populated with the computed Cartesian velocity BEFORE Jacobian projection.
   */
  Eigen::VectorXd computeRepulsiveLinkJointVelocity(
      std::vector<LinkPOI>& link_pois) const;

  /**
   * @brief Integrate the virtual target position.
   * @param velocity_linear Linear velocity to integrate
   * @param velocity_angular Angular velocity to integrate
   * @param dt Time step
   */
  void integrateTarget(const Eigen::Vector3d& velocity_linear,
                       const Eigen::Vector3d& velocity_angular,
                       double dt);

  /**
   * @brief Apply magnitude limit to a velocity vector.
   * @param velocity Input velocity
   * @param max_magnitude Maximum allowed magnitude
   * @return Limited velocity (preserves direction)
   */
  static Eigen::Vector3d limitVelocity(const Eigen::Vector3d& velocity, double max_magnitude);

  // ============== Dependencies ==============

  std::shared_ptr<RobotStateManager> robot_state_;
  std::shared_ptr<JacobianSolver> solver_;

  // ============== Dynamic Parameters (protected by mutex) ==============

  double k_attractive_{1.0};
  double k_repulsive_obstacle_{1.0};
  double k_repulsive_link_{0.5};
  double max_linear_velocity_{0.5};    // m/s
  double max_angular_velocity_{1.0};   // rad/s
  double freeze_linear_threshold_{1e-4};   // m/s
  double freeze_angular_threshold_{1e-3};  // rad/s
  double influence_distance_{1.0};     // m
  double min_safe_distance_{0.05};     // m
  RepulsiveVelocityMode repulsive_mode_{RepulsiveVelocityMode::QUADRATIC};
  
  // Repulsive smoothing configuration (EMA + rate limiter, asymmetric)
  double repulsive_filter_tau_rise_{0.0};  // [s] 0 = disabled
  double repulsive_filter_tau_fall_{0.0};  // [s] 0 = disabled
  double repulsive_max_acc_rise_{0.0};     // [m/s^2] 0 = disabled
  double repulsive_max_acc_fall_{0.0};     // [m/s^2] 0 = disabled

  // Soft Leash Parameters
  bool leash_enabled_{true};
  double leash_start_dist_{0.10};
  double leash_stop_dist_{0.20};
  double leash_reset_threshold_{0.50};
  
  mutable std::mutex params_mutex_;

  // ============== State ==============

  Eigen::Isometry3d target_raw_{Eigen::Isometry3d::Identity()};
  bool has_target_{false};
  mutable std::mutex state_mutex_;

  // Repulsive smoothing state
  Eigen::Vector3d v_repulsive_obstacle_filtered_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_repulsive_links_filtered_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_repulsive_obstacle_prev_out_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_repulsive_links_prev_out_{Eigen::Vector3d::Zero()};
};

}  // namespace cartesian_velocity_controller

