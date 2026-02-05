#pragma once

/**
 * @file joint_safety_limiter.hpp
 * @brief Joint Safety Limiter with uniform scaling (final stage of pipeline).
 *
 * The JointSafetyLimiter is the final safety layer before commands are sent
 * to the robot. It enforces joint velocity and acceleration limits by applying
 * UNIFORM scaling to the entire joint velocity vector.
 *
 * Key design principle: Instead of clamping individual joints (which can
 * distort the Cartesian motion direction), this limiter scales ALL joints
 * by the same factor. This preserves the intended Cartesian direction while
 * respecting joint limits.
 *
 * Algorithm:
 * 1. Check each joint velocity against its maximum
 * 2. Check each joint acceleration against its maximum
 * 3. Compute the minimum scaling factor required
 * 4. Apply uniform scaling to all joints
 */

#include <Eigen/Core>
#include <mutex>
#include <vector>
#include <string>

#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @class JointSafetyLimiter
 * @brief Enforces joint limits with uniform velocity scaling.
 *
 * This class provides the final safety check before joint velocity commands
 * are sent to the robot. Unlike per-joint clamping, it scales ALL joints
 * uniformly to preserve the Cartesian direction of motion.
 *
 * The limiter checks:
 * - Maximum joint velocities
 * - Maximum joint accelerations (requires previous velocity)
 *
 * The scaling factor is the minimum factor needed to bring all joints within
 * their limits simultaneously.
 */
class JointSafetyLimiter
{
public:
  /// Small constant to avoid division by zero
  static constexpr double kEpsilon = 1e-10;

  /**
   * @brief Construct a JointSafetyLimiter for a given number of joints.
   * @param num_joints Number of joints to handle
   */
  explicit JointSafetyLimiter(std::size_t num_joints);

  /**
   * @brief Construct with specific limits.
   * @param max_velocities Maximum velocity for each joint (rad/s)
   * @param max_accelerations Maximum acceleration for each joint (rad/s²)
   */
  JointSafetyLimiter(const Eigen::VectorXd& max_velocities,
                     const Eigen::VectorXd& max_accelerations);

  /**
   * @brief Destructor.
   */
  ~JointSafetyLimiter() = default;

  // ============== Main Limiting Function ==============

  /**
   * @brief Apply uniform scaling to enforce limits.
   * @param commanded_velocity Desired joint velocities
   * @param previous_velocity Previous command velocities (for acceleration check)
   * @param dt Time step since last command (seconds)
   * @return SafetyLimiterOutput with scaled velocities and diagnostics
   *
   * This is the main method to call on each control cycle. It computes the
   * scaling factor and returns the limited velocities along with diagnostic
   * information.
   */
  SafetyLimiterOutput limit(const Eigen::VectorXd& commanded_velocity,
                            const Eigen::VectorXd& previous_velocity,
                            double dt);

  /**
   * @brief Apply uniform scaling (simplified version without output struct).
   * @param commanded_velocity Desired joint velocities
   * @param previous_velocity Previous command velocities
   * @param dt Time step
   * @return Scaled joint velocities
   */
  Eigen::VectorXd limitSimple(const Eigen::VectorXd& commanded_velocity,
                              const Eigen::VectorXd& previous_velocity,
                              double dt);

  // ============== Limit Configuration ==============

  /**
   * @brief Set maximum velocities for all joints.
   * @param max_velocities Vector of maximum velocities (one per joint)
   */
  void setJointVelocityLimits(const Eigen::VectorXd& max_velocities);

  /**
   * @brief Set maximum velocity for a single joint.
   * @param joint_index Joint index (0-based)
   * @param max_velocity Maximum velocity (rad/s)
   */
  void setJointVelocityLimit(std::size_t joint_index, double max_velocity);

  /**
   * @brief Get the maximum velocity limits.
   * @return Vector of maximum velocities
   */
  Eigen::VectorXd getJointVelocityLimits() const;

  /**
   * @brief Set maximum accelerations for all joints.
   * @param max_accelerations Vector of maximum accelerations (one per joint)
   */
  void setJointAccelerationLimits(const Eigen::VectorXd& max_accelerations);

  /**
   * @brief Set maximum acceleration for a single joint.
   * @param joint_index Joint index (0-based)
   * @param max_acceleration Maximum acceleration (rad/s²)
   */
  void setJointAccelerationLimit(std::size_t joint_index, double max_acceleration);

  /**
   * @brief Get the maximum acceleration limits.
   * @return Vector of maximum accelerations
   */
  Eigen::VectorXd getJointAccelerationLimits() const;

  /**
   * @brief Set uniform limits for all joints.
   * @param max_velocity Maximum velocity for all joints (rad/s)
   * @param max_acceleration Maximum acceleration for all joints (rad/s²)
   */
  void setUniformLimits(double max_velocity, double max_acceleration);

  // ============== State ==============

  /**
   * @brief Get the last computed scaling factor.
   * @return Scaling factor (1.0 = no scaling, < 1.0 = limited)
   */
  double getLastScalingFactor() const;

  /**
   * @brief Get the number of joints.
   * @return Number of joints
   */
  std::size_t getNumJoints() const { return num_joints_; }

  /**
   * @brief Check if the last command was limited.
   * @return true if scaling was applied (factor < 1.0)
   */
  bool wasLimited() const;

  /**
   * @brief Get which limit type triggered scaling.
   * @return The type of limit that caused scaling
   */
  SafetyLimiterOutput::LimitType getLastLimitType() const;

  /**
   * @brief Get which joint triggered the scaling.
   * @return Joint index, or -1 if no limiting occurred
   */
  int getLastLimitingJoint() const;

  // ============== Enable/Disable ==============

  /**
   * @brief Enable or disable velocity limiting.
   * @param enabled If false, velocities pass through unchanged
   */
  void setVelocityLimitingEnabled(bool enabled);

  /**
   * @brief Check if velocity limiting is enabled.
   * @return true if velocity limiting is active
   */
  bool isVelocityLimitingEnabled() const;

  /**
   * @brief Enable or disable acceleration limiting.
   * @param enabled If false, acceleration is not checked
   */
  void setAccelerationLimitingEnabled(bool enabled);

  /**
   * @brief Check if acceleration limiting is enabled.
   * @return true if acceleration limiting is active
   */
  bool isAccelerationLimitingEnabled() const;

  // ============== Reset ==============

  /**
   * @brief Reset the limiter state.
   *
   * Clears the last scaling factor and limit info.
   */
  void reset();

private:
  /**
   * @brief Compute the velocity scaling factor.
   * @param velocity Joint velocity vector
   * @param limiting_joint Output: which joint triggers the limit (-1 if none)
   * @return Scaling factor (1.0 if all within limits)
   */
  double computeVelocityScalingFactor(const Eigen::VectorXd& velocity,
                                      int& limiting_joint) const;

  /**
   * @brief Compute the acceleration scaling factor.
   * @param velocity Current commanded velocity
   * @param prev_velocity Previous velocity
   * @param dt Time step
   * @param limiting_joint Output: which joint triggers the limit (-1 if none)
   * @return Scaling factor (1.0 if all within limits)
   */
  double computeAccelerationScalingFactor(const Eigen::VectorXd& velocity,
                                          const Eigen::VectorXd& prev_velocity,
                                          double dt,
                                          int& limiting_joint) const;

  // ============== Configuration ==============

  std::size_t num_joints_;
  Eigen::VectorXd max_joint_velocities_;
  Eigen::VectorXd max_joint_accelerations_;
  mutable std::mutex limits_mutex_;

  bool velocity_limiting_enabled_{true};
  bool acceleration_limiting_enabled_{true};

  // ============== State ==============

  double last_scaling_factor_{1.0};
  SafetyLimiterOutput::LimitType last_limit_type_{SafetyLimiterOutput::LimitType::NONE};
  int last_limiting_joint_{-1};
  mutable std::mutex state_mutex_;
};

}  // namespace cartesian_velocity_controller

