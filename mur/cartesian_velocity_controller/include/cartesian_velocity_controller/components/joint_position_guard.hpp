#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>

#include "cartesian_velocity_controller/types/controller_joint_limits.hpp"

namespace cartesian_velocity_controller
{

/**
 * @brief Runtime guardrail to prevent controller commands from driving joints into position limits.
 *
 * Applies:
 * - soft braking close to the limit (continuous scaling)
 * - hard margin behavior with optional re-entry velocity
 *
 * Intended insertion point: after Jacobian solve (+ optional injection) and before JointVelocityFilter.
 */
class JointPositionGuard
{
public:
  struct Entry
  {
    std::string joint_name;
    int index{-1};

    double min{0.0};
    double max{0.0};

    JointRuntimeGuardConfig runtime{};
  };

  JointPositionGuard() = default;

  /**
   * @brief Configure from controller joint limits.
   * @param cfg controller_joint_limits config
   * @param joint_index_map mapping joint name -> index in q/qdot vectors
   */
  void configure(const ControllerJointLimitsConfig& cfg,
                 const std::unordered_map<std::string, std::size_t>& joint_index_map);

  bool isEnabled() const { return enabled_; }

  /**
   * @brief Apply guardrail in-place on joint velocity command.
   * @param joint_positions current q [rad]
   * @param joint_velocities in/out qdot [rad/s]
   */
  void apply(const Eigen::VectorXd& joint_positions, Eigen::VectorXd& joint_velocities) const;

  /**
   * @brief Update runtime guard parameters for a specific joint at runtime.
   * @param joint_name Name of the joint to update
   * @param soft_zone New soft braking zone [rad]
   * @param margin New hard margin [rad]
   * @param reentry_velocity New reentry velocity [rad/s]
   * @return true if joint was found and updated, false otherwise
   */
  bool updateRuntimeGuard(const std::string& joint_name, double soft_zone, double margin, double reentry_velocity);

  /**
   * @brief Update position limits for a specific joint at runtime.
   * @param joint_name Name of the joint to update
   * @param min New minimum position limit [rad]
   * @param max New maximum position limit [rad]
   * @return true if joint was found and updated, false otherwise
   */
  bool updateJointLimits(const std::string& joint_name, double min, double max);

  /**
   * @brief Get the current runtime guard config for a specific joint.
   * @param joint_name Name of the joint
   * @param config Output config (valid only if function returns true)
   * @return true if joint was found, false otherwise
   */
  bool getRuntimeGuard(const std::string& joint_name, JointRuntimeGuardConfig& config) const;

  /**
   * @brief Get the current position limits for a specific joint.
   * @param joint_name Name of the joint
   * @param min Output minimum position limit [rad]
   * @param max Output maximum position limit [rad]
   * @return true if joint was found, false otherwise
   */
  bool getJointLimits(const std::string& joint_name, double& min, double& max) const;

private:
  static double clamp01(double x);
  static double smoothstep(double x);

  bool enabled_{false};
  std::vector<Entry> entries_;
};

}  // namespace cartesian_velocity_controller

