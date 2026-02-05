#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cartesian_velocity_controller/types/config_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @brief Level C: Motion Generator - Second-order filter with time constant τ
 *
 * This filter sits between LocalPlanner (Level B) and PID+IK (Level D).
 * It filters Cartesian velocities to make motion realizable while respecting
 * acceleration and jerk limits.
 *
 * Algorithm (from PIPELINE_REFACTORING_PLAN.md):
 *   a_desired = (V_desired - V_corrente) / τ
 *   jerk = (a_desired - a_corrente) / dt
 *
 *   Limits applied:
 *     |jerk| ≤ max_jerk
 *     |acceleration| ≤ max_acceleration
 *     |velocity| ≤ max_velocity
 *
 *   OUTPUT: V_filtrata, P_target_filtrato
 *
 * The time constant τ can be adjusted at runtime (e.g., by a fuzzy controller)
 * to tune responsiveness vs smoothness.
 */
class CartesianVelocityFilter
{
public:
  /**
   * @brief Construct with default limits.
   */
  CartesianVelocityFilter();

  /**
   * @brief Construct with specified limits.
   * @param linear_limits Limits for linear velocity (m/s, m/s², m/s³)
   * @param angular_limits Limits for angular velocity (rad/s, rad/s², rad/s³)
   */
  CartesianVelocityFilter(const VelocityLimits& linear_limits,
                          const VelocityLimits& angular_limits);

  /**
   * @brief Reset the filter state to zero.
   */
  void reset();

  /**
   * @brief Reset the filter and initialize to a specific pose.
   * @param position Initial pose for position tracking
   *
   * Call this when starting motion to initialize P_target_filtrato.
   */
  void resetToPosition(const Eigen::Isometry3d& position);

  /**
   * @brief Reset the filter and initialize to a specific state (pose, velocity, acceleration, jerk).
   * @param position Initial pose
   * @param linear_velocity Initial linear velocity
   * @param angular_velocity Initial angular velocity
   * @param linear_acceleration Initial linear acceleration (optional, default zero)
   * @param angular_acceleration Initial angular acceleration (optional, default zero)
   * @param linear_jerk Initial linear jerk (optional, default zero)
   * @param angular_jerk Initial angular jerk (optional, default zero)
   *
   * Call this when switching targets while moving to preserve continuity.
   */
  void resetToState(const Eigen::Isometry3d& position,
                    const Eigen::Vector3d& linear_velocity,
                    const Eigen::Vector3d& angular_velocity,
                    const Eigen::Vector3d& linear_acceleration = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& angular_acceleration = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& linear_jerk = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& angular_jerk = Eigen::Vector3d::Zero());

  /**
   * @brief Filter a 6D Cartesian velocity command.
   * @param desired_velocity Desired velocity [vx, vy, vz, wx, wy, wz] from LocalPlanner
   * @param dt Time step since last call (seconds)
   * @return Filtered velocity command (V_filtrata)
   *
   * Also updates internal P_target_filtrato via integration.
   */
  Eigen::Matrix<double, 6, 1> filter(const Eigen::Matrix<double, 6, 1>& desired_velocity,
                                     double dt);

  // ============== dt handling (robustness to jitter) ==============

  /**
   * @brief Use a fixed internal timestep with an accumulator and sub-steps.
   *
   * If dt_nominal > 0, each call accumulates clamped dt and executes 0..N internal
   * steps with dt_nominal. This makes the jerk->acc->vel integration far more
   * deterministic under loop jitter.
   */
  void setDtNominal(double dt_nominal);
  double getDtNominal() const { return dt_nominal_; }

  /// Clamp incoming dt before accumulation.
  void setDtClamp(double min_dt, double max_dt);

  /// Max internal steps per control cycle (CPU guardrail).
  void setMaxSubsteps(int max_substeps);

  /// If incoming dt exceeds this threshold, apply a large-dt policy instead of integrating.
  void setResetDtThreshold(double reset_dt_threshold);

  enum class LargeDtPolicy
  {
    HOLD_LAST,
    RESET_TO_ZERO,
    RESET_TO_DESIRED
  };

  void setLargeDtPolicy(LargeDtPolicy policy) { large_dt_policy_ = policy; }
  LargeDtPolicy getLargeDtPolicy() const { return large_dt_policy_; }

  // ============== Limiting mode ==============

  /// true: uniform scaling (preserve direction). false: per-axis clamping.
  void setUniformScalingEnabled(bool enabled) { uniform_scaling_enabled_ = enabled; }
  bool isUniformScalingEnabled() const { return uniform_scaling_enabled_; }

  // ============== Output Accessors ==============

  /** @brief Get the filtered position (P_target_filtrato) */
  Eigen::Isometry3d getFilteredPosition() const { return filtered_position_; }

  /** @brief Get current filtered linear velocity */
  Eigen::Vector3d getCurrentLinearVelocity() const { return linear_velocity_; }

  /** @brief Get current filtered angular velocity */
  Eigen::Vector3d getCurrentAngularVelocity() const { return angular_velocity_; }

  /** @brief Get current linear acceleration */
  Eigen::Vector3d getCurrentLinearAcceleration() const { return linear_acceleration_; }

  /** @brief Get current angular acceleration */
  Eigen::Vector3d getCurrentAngularAcceleration() const { return angular_acceleration_; }

  /** @brief Get current linear jerk */
  Eigen::Vector3d getCurrentLinearJerk() const { return linear_jerk_; }

  /** @brief Get current angular jerk */
  Eigen::Vector3d getCurrentAngularJerk() const { return angular_jerk_; }

  // ============== Configuration ==============

  /** @brief Set limits for linear velocity */
  void setLinearLimits(const VelocityLimits& limits) { linear_limits_ = limits; }

  /** @brief Set limits for angular velocity */
  void setAngularLimits(const VelocityLimits& limits) { angular_limits_ = limits; }

  /** @brief Get linear velocity limits */
  VelocityLimits getLinearLimits() const { return linear_limits_; }

  /** @brief Get angular velocity limits */
  VelocityLimits getAngularLimits() const { return angular_limits_; }

  /** @brief Enable or disable the filter */
  void setEnabled(bool enabled) { enabled_ = enabled; }

  /** @brief Check if filter is enabled */
  bool isEnabled() const { return enabled_; }

  // ============== Time Constant (τ) Control ==============

  /**
   * @brief Set the time constant τ for proportional acceleration control.
   * @param tau Time constant in seconds (typical range: 0.02 - 0.5)
   *
   * Small τ = more reactive, Large τ = smoother motion.
   * Can be adjusted at runtime by a fuzzy controller.
   */
  void setTimeConstant(double tau);

  /** @brief Get the current time constant */
  double getTimeConstant() const { return tau_; }

  /**
   * @brief Set separate time constants for linear and angular motion.
   * @param tau_linear Time constant for linear velocity
   * @param tau_angular Time constant for angular velocity
   */
  void setTimeConstants(double tau_linear, double tau_angular);

  /** @brief Get the linear time constant */
  double getLinearTimeConstant() const { return tau_linear_; }

  /** @brief Get the angular time constant */
  double getAngularTimeConstant() const { return tau_angular_; }

private:
  // Limits
  VelocityLimits linear_limits_;
  VelocityLimits angular_limits_;

  // State: velocity, acceleration, and jerk
  Eigen::Vector3d linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_jerk_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_jerk_{Eigen::Vector3d::Zero()};

  // State: filtered position (P_target_filtrato)
  Eigen::Isometry3d filtered_position_{Eigen::Isometry3d::Identity()};
  bool has_position_{false};

  // Configuration
  bool enabled_{true};
  double min_dt_{1e-6};

  // Fixed-dt integration (optional)
  double dt_nominal_{0.0};               ///< If >0 enables fixed internal dt mode
  double min_dt_clamp_{1e-6};            ///< Clamp for incoming dt before accumulation
  double max_dt_clamp_{0.1};
  int max_substeps_{10};
  double reset_dt_threshold_{0.25};      ///< If dt > this, apply large-dt policy
  LargeDtPolicy large_dt_policy_{LargeDtPolicy::HOLD_LAST};
  double accumulator_{0.0};

  bool uniform_scaling_enabled_{true};

  // Time constants
  double tau_{0.1};  // 100ms default
  double tau_linear_{0.1};
  double tau_angular_{0.1};
  bool use_separate_tau_{false};

  /**
   * @brief Filter a 3D vector using the Level C algorithm.
   *
   * Algorithm:
   *   1. a_desired = (v_desired - v_current) / τ
   *   2. jerk = (a_desired - a_current) / dt
   *   3. Limit jerk magnitude
   *   4. a_new = a_current + jerk * dt, then limit acceleration
   *   5. v_new = v_current + a_new * dt, then limit velocity
   */
  Eigen::Vector3d filterVector(const Eigen::Vector3d& desired,
                               Eigen::Vector3d& current_vel,
                               Eigen::Vector3d& current_accel,
                               Eigen::Vector3d& current_jerk,
                               const VelocityLimits& limits,
                               double tau,
                               double dt);

  /**
   * @brief Integrate filtered velocity to update P_target_filtrato.
   */
  void integratePosition(const Eigen::Vector3d& linear_vel,
                         const Eigen::Vector3d& angular_vel,
                         double dt);
};

}  // namespace cartesian_velocity_controller
