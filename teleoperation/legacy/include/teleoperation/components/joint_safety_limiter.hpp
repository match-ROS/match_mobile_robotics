#pragma once

/**
 * @file joint_safety_limiter.hpp
 * @brief Joint Safety Limiter with uniform scaling (final stage of pipeline).
 */

#include <Eigen/Core>
#include <mutex>
#include <vector>
#include <string>

#include "teleoperation/types.hpp"

namespace teleoperation
{

/**
 * @brief Enforces joint limits with uniform velocity scaling.
 *
 * Checks:
 * - Maximum joint velocities
 * - Maximum joint accelerations (optional, requires previous velocity)
 *
 * Scaling is uniform across all joints to preserve Cartesian direction.
 */
class JointSafetyLimiter
{
public:
  static constexpr double kEpsilon = 1e-10;

  explicit JointSafetyLimiter(std::size_t num_joints);
  JointSafetyLimiter(const Eigen::VectorXd& max_velocities,
                     const Eigen::VectorXd& max_accelerations);
  ~JointSafetyLimiter() = default;

  SafetyLimiterOutput limit(const Eigen::VectorXd& commanded_velocity,
                            const Eigen::VectorXd& previous_velocity,
                            double dt);
  Eigen::VectorXd limitSimple(const Eigen::VectorXd& commanded_velocity,
                              const Eigen::VectorXd& previous_velocity,
                              double dt);

  void setJointVelocityLimits(const Eigen::VectorXd& max_velocities);
  void setJointVelocityLimit(std::size_t joint_index, double max_velocity);
  Eigen::VectorXd getJointVelocityLimits() const;

  void setJointAccelerationLimits(const Eigen::VectorXd& max_accelerations);
  void setJointAccelerationLimit(std::size_t joint_index, double max_acceleration);
  Eigen::VectorXd getJointAccelerationLimits() const;

  void setUniformLimits(double max_velocity, double max_acceleration);

  double getLastScalingFactor() const;
  std::size_t getNumJoints() const { return num_joints_; }
  bool wasLimited() const;
  SafetyLimiterOutput::LimitType getLastLimitType() const;
  int getLastLimitingJoint() const;

  void setVelocityLimitingEnabled(bool enabled);
  bool isVelocityLimitingEnabled() const;

  void setAccelerationLimitingEnabled(bool enabled);
  bool isAccelerationLimitingEnabled() const;

  void reset();

private:
  double computeVelocityScalingFactor(const Eigen::VectorXd& velocity,
                                      int& limiting_joint) const;
  double computeAccelerationScalingFactor(const Eigen::VectorXd& velocity,
                                          const Eigen::VectorXd& prev_velocity,
                                          double dt,
                                          int& limiting_joint) const;

  std::size_t num_joints_;
  Eigen::VectorXd max_joint_velocities_;
  Eigen::VectorXd max_joint_accelerations_;
  mutable std::mutex limits_mutex_;

  bool velocity_limiting_enabled_{true};
  bool acceleration_limiting_enabled_{true};

  // State
  double last_scaling_factor_{1.0};
  SafetyLimiterOutput::LimitType last_limit_type_{SafetyLimiterOutput::LimitType::NONE};
  int last_limiting_joint_{-1};
  mutable std::mutex state_mutex_;
};

}  // namespace teleoperation

