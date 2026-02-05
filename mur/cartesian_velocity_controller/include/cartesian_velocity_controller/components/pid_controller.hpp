#pragma once

/**
 * @file pid_controller.hpp
 * @brief Generic PID controller with feedforward, saturation and dynamic anti-windup.
 */

#include <Eigen/Core>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace cartesian_velocity_controller
{

/**
 * @struct PIDConfig
 * @brief Configuration for PID controller.
 */
struct PIDConfig
{
  double kp{1.0};                    ///< Proportional gain
  double ki{0.0};                    ///< Integral gain
  double kd{0.0};                    ///< Derivative gain
  double kff{0.0};                   ///< Feedforward gain [0..1]
  double output_limit{1.0};          ///< Output saturation limit (symmetric: [-limit, +limit])
  double derivative_filter_tau{0.1}; ///< Low-pass filter time constant for derivative (s)
  bool enabled{true};
};

/**
 * @class PIDController
 * @brief Generic N-dimensional PID controller with feedforward, saturation and anti-windup.
 *
 * Features:
 * - Proportional, Integral, Derivative control
 * - Feedforward term
 * - Output saturation
 * - Dynamic anti-windup: integral headroom = output_limit - |P|
 * - Filtered derivative (low-pass)
 * - Thread-safe configuration updates
 *
 * Usage:
 * @code
 *   PIDController pid(3);  // 3D controller (e.g., position)
 *   pid.setConfig({.kp=2.0, .ki=0.1, .kd=0.05, .output_limit=0.5});
 *
 *   Eigen::Vector3d error = target - current;
 *   Eigen::Vector3d feedforward = trajectory_velocity;
 *   Eigen::Vector3d output = pid.compute(error, feedforward, dt);
 * @endcode
 */
class PIDController
{
public:
  static constexpr double kEpsilon = 1e-10;

  /**
   * @brief Construct PID controller for N-dimensional signals.
   * @param dimensions Number of dimensions (e.g., 3 for position, 6 for twist)
   * @param config Initial configuration
   */
  explicit PIDController(int dimensions, const PIDConfig& config = {});

  /**
   * @brief Set configuration (thread-safe).
   */
  void setConfig(const PIDConfig& config);

  /**
   * @brief Get current configuration (thread-safe).
   */
  PIDConfig getConfig() const;

  /**
   * @brief Compute PID output.
   * @param error Current error (setpoint - measurement)
   * @param feedforward Feedforward signal (e.g., desired velocity)
   * @param dt Time step (seconds)
   * @return Saturated output
   */
  Eigen::VectorXd compute(const Eigen::VectorXd& error,
                          const Eigen::VectorXd& feedforward,
                          double dt);

  /**
   * @brief Compute PID output without feedforward.
   */
  Eigen::VectorXd compute(const Eigen::VectorXd& error, double dt);

  /**
   * @brief Reset integral accumulator and derivative state.
   */
  void reset();

  /**
   * @brief Reset only integral accumulator.
   */
  void resetIntegral();

  /**
   * @brief Get last computed error.
   */
  Eigen::VectorXd getLastError() const;

  /**
   * @brief Get current integral accumulator value.
   */
  Eigen::VectorXd getIntegral() const;

  /**
   * @brief Get last computed P term (Kp * error).
   */
  Eigen::VectorXd getLastPTerm() const;

  /**
   * @brief Get last computed I term (Ki * integral).
   */
  Eigen::VectorXd getLastITerm() const;

  /**
   * @brief Get last computed D term (Kd * derivative).
   */
  Eigen::VectorXd getLastDTerm() const;

  /**
   * @brief Set integral accumulator directly (use with caution).
   */
  void setIntegral(const Eigen::VectorXd& integral);

  /**
   * @brief Enable or disable the controller.
   */
  void setEnabled(bool enabled);

  /**
   * @brief Check if controller is enabled.
   */
  bool isEnabled() const;

  /**
   * @brief Get number of dimensions.
   */
  int dimensions() const { return dimensions_; }

private:
  /**
   * @brief Apply dynamic anti-windup to integral term.
   *
   * Limits integral contribution so that |I| <= max(0, output_limit - |P|)
   * This prevents integral from adding to output when P alone saturates.
   */
  void applyAntiWindup(Eigen::VectorXd& integral_output,
                       double p_magnitude,
                       double output_limit);

  /**
   * @brief Saturate output vector to limit.
   */
  void saturate(Eigen::VectorXd& output, double limit) const;

  int dimensions_;
  PIDConfig config_;
  mutable std::mutex config_mutex_;

  // State
  Eigen::VectorXd integral_;
  Eigen::VectorXd prev_error_;
  Eigen::VectorXd derivative_filtered_;
  Eigen::VectorXd last_error_;
  Eigen::VectorXd last_p_term_;
  Eigen::VectorXd last_i_term_;
  Eigen::VectorXd last_d_term_;
  bool has_prev_error_{false};
  mutable std::mutex state_mutex_;
};

}  // namespace cartesian_velocity_controller

