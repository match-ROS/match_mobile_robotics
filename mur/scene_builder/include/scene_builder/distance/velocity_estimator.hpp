/**
 * @file velocity_estimator.hpp
 * @brief Velocity estimation for collision objects in the planning scene
 *
 * This file provides the VelocityEstimator class that estimates velocities
 * of collision objects by tracking their positions over time.
 */

#ifndef SCENE_BUILDER_DISTANCE_VELOCITY_ESTIMATOR_HPP
#define SCENE_BUILDER_DISTANCE_VELOCITY_ESTIMATOR_HPP

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <Eigen/Dense>

#include <ros/time.h>

#include <map>
#include <mutex>
#include <string>

namespace scene_builder
{
namespace distance
{

/**
 * @brief Estimated velocity data for an object
 */
struct ObjectVelocity
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();   ///< Linear velocity (m/s)
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();  ///< Angular velocity (rad/s)
  bool valid = false;                                  ///< Whether the estimate is valid
};

/**
 * @brief Configuration for velocity estimation
 */
struct VelocityEstimatorConfig
{
  double min_dt = 0.001;           ///< Minimum time delta to avoid division by zero (s)
  double max_dt = 1.0;             ///< Maximum time delta before invalidating estimate (s)
  double velocity_filter_alpha = 0.3;  ///< Low-pass filter coefficient (0-1, higher = less filtering)
  double max_linear_velocity = 10.0;   ///< Maximum allowed linear velocity magnitude (m/s)
  double max_angular_velocity = 20.0;  ///< Maximum allowed angular velocity magnitude (rad/s)
};

/**
 * @brief Estimates velocities of collision objects in the planning scene
 *
 * VelocityEstimator tracks the positions of collision objects over time
 * and computes velocity estimates using finite differences with optional
 * low-pass filtering.
 *
 * Features:
 * - Automatic tracking of all world objects in the scene
 * - Low-pass filtering to reduce noise
 * - Automatic cleanup of removed objects
 * - Thread-safe operation
 */
class VelocityEstimator
{
public:
  /**
   * @brief Constructor
   * @param config Configuration for velocity estimation
   */
  explicit VelocityEstimator(const VelocityEstimatorConfig& config = {});

  /**
   * @brief Updates velocity estimates for all objects in the scene
   * @param scene_monitor Planning scene monitor to read object positions from
   * @param current_time Current timestamp
   *
   * This method should be called periodically (e.g., in a timer callback)
   * to update velocity estimates.
   */
  void update(const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor,
              const ros::Time& current_time);

  /**
   * @brief Gets the estimated velocity for a specific object
   * @param object_id ID of the collision object
   * @return Estimated velocity (check valid field before using)
   */
  ObjectVelocity getVelocity(const std::string& object_id) const;

  /**
   * @brief Gets all estimated velocities
   * @return Map of object IDs to their estimated velocities
   */
  std::map<std::string, ObjectVelocity> getAllVelocities() const;

  /**
   * @brief Resets all velocity estimates
   */
  void reset();

  /**
   * @brief Sets the configuration
   * @param config New configuration
   */
  void setConfig(const VelocityEstimatorConfig& config);

  /**
   * @brief Gets the current configuration
   * @return Current configuration
   */
  const VelocityEstimatorConfig& getConfig() const { return config_; }

private:
  /**
   * @brief Internal state for tracking an object
   */
  struct ObjectState
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  ///< Last known pose
    ros::Time timestamp;                                       ///< Timestamp of last pose
    ObjectVelocity velocity;                                   ///< Current velocity estimate
    bool initialized = false;                                  ///< Whether we have a previous pose
  };

  /**
   * @brief Computes angular velocity from rotation difference
   * @param R1 Previous rotation matrix
   * @param R2 Current rotation matrix
   * @param dt Time delta
   * @return Angular velocity vector
   */
  Eigen::Vector3d computeAngularVelocity(const Eigen::Matrix3d& R1,
                                          const Eigen::Matrix3d& R2,
                                          double dt) const;

  /**
   * @brief Applies low-pass filter to a vector
   * @param current Current filtered value
   * @param measurement New measurement
   * @param alpha Filter coefficient
   * @return Filtered value
   */
  Eigen::Vector3d lowPassFilter(const Eigen::Vector3d& current,
                                 const Eigen::Vector3d& measurement,
                                 double alpha) const;

  /**
   * @brief Clamps velocity magnitude
   * @param velocity Velocity vector to clamp
   * @param max_magnitude Maximum allowed magnitude
   * @return Clamped velocity vector
   */
  Eigen::Vector3d clampVelocity(const Eigen::Vector3d& velocity,
                                 double max_magnitude) const;

  VelocityEstimatorConfig config_;                      ///< Current configuration
  std::map<std::string, ObjectState> object_states_;    ///< Tracked object states
  mutable std::mutex mutex_;                            ///< Mutex for thread safety
};

}  // namespace distance
}  // namespace scene_builder

#endif  // SCENE_BUILDER_DISTANCE_VELOCITY_ESTIMATOR_HPP
