/**
 * @file velocity_filter.cpp
 * @brief Level C: Motion Generator - Second-order filter implementation
 */

#include "cartesian_velocity_controller/velocity_filter.hpp"

#include <algorithm>
#include <cmath>

namespace cartesian_velocity_controller
{

namespace
{
constexpr double kEpsilon = 1e-9;
}  // namespace

// ============================================================================
// CartesianVelocityFilter Implementation
// ============================================================================

CartesianVelocityFilter::CartesianVelocityFilter()
{
  // Default limits for Cartesian motion
  linear_limits_.max_velocity = 0.5;       // m/s
  linear_limits_.max_acceleration = 1.0;   // m/s²
  linear_limits_.max_jerk = 10.0;          // m/s³

  angular_limits_.max_velocity = 1.0;      // rad/s
  angular_limits_.max_acceleration = 2.0;  // rad/s²
  angular_limits_.max_jerk = 20.0;         // rad/s³
}

CartesianVelocityFilter::CartesianVelocityFilter(const VelocityLimits& linear_limits,
                                                 const VelocityLimits& angular_limits)
  : linear_limits_(linear_limits)
  , angular_limits_(angular_limits)
{
}

void CartesianVelocityFilter::reset()
{
  linear_velocity_.setZero();
  linear_acceleration_.setZero();
  linear_jerk_.setZero();
  angular_velocity_.setZero();
  angular_acceleration_.setZero();
  angular_jerk_.setZero();
  filtered_position_ = Eigen::Isometry3d::Identity();
  has_position_ = false;
}

void CartesianVelocityFilter::resetToPosition(const Eigen::Isometry3d& position)
{
  linear_velocity_.setZero();
  linear_acceleration_.setZero();
  linear_jerk_.setZero();
  angular_velocity_.setZero();
  angular_acceleration_.setZero();
  angular_jerk_.setZero();
  filtered_position_ = position;
  has_position_ = true;
}

void CartesianVelocityFilter::resetToState(const Eigen::Isometry3d& position,
                                           const Eigen::Vector3d& linear_velocity,
                                           const Eigen::Vector3d& angular_velocity,
                                           const Eigen::Vector3d& linear_acceleration,
                                           const Eigen::Vector3d& angular_acceleration,
                                           const Eigen::Vector3d& linear_jerk,
                                           const Eigen::Vector3d& angular_jerk)
{
  linear_velocity_ = linear_velocity;
  linear_acceleration_ = linear_acceleration;
  linear_jerk_ = linear_jerk;
  angular_velocity_ = angular_velocity;
  angular_acceleration_ = angular_acceleration;
  angular_jerk_ = angular_jerk;
  filtered_position_ = position;
  has_position_ = true;
}

Eigen::Matrix<double, 6, 1> CartesianVelocityFilter::filter(
    const Eigen::Matrix<double, 6, 1>& desired_velocity,
    double dt)
{
  // Fixed-dt mode (robust to jitter): accumulate dt and run sub-steps with dt_nominal_
  if (dt_nominal_ > 0.0)
  {
    // Large-dt outlier handling (use raw dt for the threshold check)
    if (dt > reset_dt_threshold_)
    {
      if (large_dt_policy_ == LargeDtPolicy::HOLD_LAST)
      {
        Eigen::Matrix<double, 6, 1> output;
        output.head<3>() = linear_velocity_;
        output.tail<3>() = angular_velocity_;
        return output;
      }
      else if (large_dt_policy_ == LargeDtPolicy::RESET_TO_ZERO)
      {
        linear_velocity_.setZero();
        linear_acceleration_.setZero();
        linear_jerk_.setZero();
        angular_velocity_.setZero();
        angular_acceleration_.setZero();
        angular_jerk_.setZero();
        accumulator_ = 0.0;

        Eigen::Matrix<double, 6, 1> output = Eigen::Matrix<double, 6, 1>::Zero();
        return output;
      }
      else  // RESET_TO_DESIRED
      {
        linear_velocity_ = desired_velocity.head<3>();
        angular_velocity_ = desired_velocity.tail<3>();
        linear_acceleration_.setZero();
        angular_acceleration_.setZero();
        linear_jerk_.setZero();
        angular_jerk_.setZero();
        accumulator_ = 0.0;

        Eigen::Matrix<double, 6, 1> output = desired_velocity;
        return output;
      }
    }

    const double dt_clamped = std::clamp(dt, min_dt_clamp_, max_dt_clamp_);
    accumulator_ += dt_clamped;

    int steps = 0;
    while (accumulator_ >= dt_nominal_ && steps < max_substeps_)
    {
      const double step_dt = dt_nominal_;

      Eigen::Vector3d desired_linear = desired_velocity.head<3>();
      Eigen::Vector3d desired_angular = desired_velocity.tail<3>();

      Eigen::Vector3d filtered_linear;
      Eigen::Vector3d filtered_angular;

      if (enabled_)
      {
        // Get time constants (unified or separate)
        double tau_lin = use_separate_tau_ ? tau_linear_ : tau_;
        double tau_ang = use_separate_tau_ ? tau_angular_ : tau_;

        filtered_linear = filterVector(desired_linear,
                                       linear_velocity_,
                                       linear_acceleration_,
                                       linear_jerk_,
                                       linear_limits_,
                                       tau_lin,
                                       step_dt);

        filtered_angular = filterVector(desired_angular,
                                        angular_velocity_,
                                        angular_acceleration_,
                                        angular_jerk_,
                                        angular_limits_,
                                        tau_ang,
                                        step_dt);
      }
      else
      {
        // Pass through but update state for continuity
        Eigen::Vector3d new_linear_accel = (desired_linear - linear_velocity_) / step_dt;
        linear_jerk_ = (new_linear_accel - linear_acceleration_) / step_dt;
        linear_acceleration_ = new_linear_accel;
        linear_velocity_ = desired_linear;
        filtered_linear = desired_linear;

        Eigen::Vector3d new_angular_accel = (desired_angular - angular_velocity_) / step_dt;
        angular_jerk_ = (new_angular_accel - angular_acceleration_) / step_dt;
        angular_acceleration_ = new_angular_accel;
        angular_velocity_ = desired_angular;
        filtered_angular = desired_angular;
      }

      // Integrate position (P_target_filtrato)
      if (has_position_)
      {
        integratePosition(filtered_linear, filtered_angular, step_dt);
      }

      accumulator_ -= dt_nominal_;
      steps++;
    }

    Eigen::Matrix<double, 6, 1> output;
    output.head<3>() = linear_velocity_;
    output.tail<3>() = angular_velocity_;
    return output;
  }

  // Raw-dt mode (legacy)
  if (dt < min_dt_)
  {
    Eigen::Matrix<double, 6, 1> output;
    output.head<3>() = linear_velocity_;
    output.tail<3>() = angular_velocity_;
    return output;
  }

  Eigen::Vector3d desired_linear = desired_velocity.head<3>();
  Eigen::Vector3d desired_angular = desired_velocity.tail<3>();

  Eigen::Vector3d filtered_linear;
  Eigen::Vector3d filtered_angular;

  if (enabled_)
  {
    // Get time constants (unified or separate)
    double tau_lin = use_separate_tau_ ? tau_linear_ : tau_;
    double tau_ang = use_separate_tau_ ? tau_angular_ : tau_;

    filtered_linear = filterVector(desired_linear,
                                   linear_velocity_,
                                   linear_acceleration_,
                                   linear_jerk_,
                                   linear_limits_,
                                   tau_lin,
                                   dt);

    filtered_angular = filterVector(desired_angular,
                                    angular_velocity_,
                                    angular_acceleration_,
                                    angular_jerk_,
                                    angular_limits_,
                                    tau_ang,
                                    dt);
  }
  else
  {
    // Pass through but update state for continuity
    Eigen::Vector3d new_linear_accel = (desired_linear - linear_velocity_) / dt;
    linear_jerk_ = (new_linear_accel - linear_acceleration_) / dt;
    linear_acceleration_ = new_linear_accel;
    linear_velocity_ = desired_linear;
    filtered_linear = desired_linear;

    Eigen::Vector3d new_angular_accel = (desired_angular - angular_velocity_) / dt;
    angular_jerk_ = (new_angular_accel - angular_acceleration_) / dt;
    angular_acceleration_ = new_angular_accel;
    angular_velocity_ = desired_angular;
    filtered_angular = desired_angular;
  }

  // Integrate position (P_target_filtrato)
  if (has_position_)
  {
    integratePosition(filtered_linear, filtered_angular, dt);
  }

  Eigen::Matrix<double, 6, 1> output;
  output.head<3>() = filtered_linear;
  output.tail<3>() = filtered_angular;
  return output;
}

void CartesianVelocityFilter::setTimeConstant(double tau)
{
  // Clamp to reasonable range
  tau_ = std::max(0.01, std::min(1.0, tau));
  // When setting unified tau, disable separate tau mode
  use_separate_tau_ = false;
}

void CartesianVelocityFilter::setTimeConstants(double tau_linear, double tau_angular)
{
  tau_linear_ = std::max(0.01, std::min(1.0, tau_linear));
  tau_angular_ = std::max(0.01, std::min(1.0, tau_angular));
  use_separate_tau_ = true;
}

Eigen::Vector3d CartesianVelocityFilter::filterVector(const Eigen::Vector3d& desired,
                                                      Eigen::Vector3d& current_vel,
                                                      Eigen::Vector3d& current_accel,
                                                      Eigen::Vector3d& current_jerk,
                                                      const VelocityLimits& limits,
                                                      double tau,
                                                      double dt)
{
  // Level C Algorithm from PIPELINE_REFACTORING_PLAN.md:
  //
  // Step 1: a_desired = (V_desired - V_corrente) / τ
  Eigen::Vector3d a_desired = (desired - current_vel) / tau;

  // Step 2: jerk = (a_desired - a_corrente) / dt
  Eigen::Vector3d jerk = (a_desired - current_accel) / dt;

  // Step 3: Limit jerk magnitude |jerk| ≤ max_jerk
  if (uniform_scaling_enabled_)
  {
    double jerk_magnitude = jerk.norm();
    if (jerk_magnitude > limits.max_jerk && jerk_magnitude > kEpsilon)
    {
      jerk = jerk * (limits.max_jerk / jerk_magnitude);
    }
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      jerk[i] = std::clamp(jerk[i], -limits.max_jerk, limits.max_jerk);
    }
  }

  // Step 4: Compute jerk-limited acceleration
  Eigen::Vector3d a_new = current_accel + jerk * dt;

  // Step 5: Limit acceleration magnitude |acceleration| ≤ max_acceleration
  if (uniform_scaling_enabled_)
  {
    double a_magnitude = a_new.norm();
    if (a_magnitude > limits.max_acceleration && a_magnitude > kEpsilon)
    {
      a_new = a_new * (limits.max_acceleration / a_magnitude);
    }
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      a_new[i] = std::clamp(a_new[i], -limits.max_acceleration, limits.max_acceleration);
    }
  }

  // Step 6: Compute new velocity
  Eigen::Vector3d v_new = current_vel + a_new * dt;

  // Step 7: Limit velocity magnitude |velocity| ≤ max_velocity
  if (uniform_scaling_enabled_)
  {
    double v_magnitude = v_new.norm();
    if (v_magnitude > limits.max_velocity && v_magnitude > kEpsilon)
    {
      v_new = v_new * (limits.max_velocity / v_magnitude);
      // Recalculate acceleration for consistency
      a_new = (v_new - current_vel) / dt;
    }
  }
  else
  {
    for (int i = 0; i < 3; ++i)
    {
      v_new[i] = std::clamp(v_new[i], -limits.max_velocity, limits.max_velocity);
    }
    a_new = (v_new - current_vel) / dt;
  }

  // Update state
  current_vel = v_new;
  current_accel = a_new;
  current_jerk = jerk;

  return v_new;
}

void CartesianVelocityFilter::integratePosition(const Eigen::Vector3d& linear_vel,
                                                const Eigen::Vector3d& angular_vel,
                                                double dt)
{
  // Integrate position: P_new = P_old + V * dt
  filtered_position_.translation() += linear_vel * dt;

  // Integrate orientation using exponential map
  double angle = angular_vel.norm();
  if (angle > kEpsilon)
  {
    Eigen::Vector3d axis = angular_vel / angle;
    double delta_angle = angle * dt;
    Eigen::Quaterniond delta_rot(Eigen::AngleAxisd(delta_angle, axis));
    Eigen::Quaterniond current_rot(filtered_position_.rotation());
    filtered_position_.linear() = (delta_rot * current_rot).toRotationMatrix();
  }
}

void CartesianVelocityFilter::setDtNominal(double dt_nominal)
{
  if (dt_nominal <= 0.0)
  {
    dt_nominal_ = 0.0;
    accumulator_ = 0.0;
    return;
  }

  // Keep it sane and consistent with min_dt_
  dt_nominal_ = std::max(dt_nominal, min_dt_);
  accumulator_ = 0.0;
}

void CartesianVelocityFilter::setDtClamp(double min_dt, double max_dt)
{
  min_dt_clamp_ = std::max(min_dt, 1e-9);
  max_dt_clamp_ = std::max(max_dt, min_dt_clamp_);
}

void CartesianVelocityFilter::setMaxSubsteps(int max_substeps)
{
  max_substeps_ = std::max(0, max_substeps);
}

void CartesianVelocityFilter::setResetDtThreshold(double reset_dt_threshold)
{
  reset_dt_threshold_ = std::max(0.0, reset_dt_threshold);
}

}  // namespace cartesian_velocity_controller
