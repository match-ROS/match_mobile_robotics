/**
 * @file joint_velocity_filter.cpp
 * @brief Implementation of JointVelocityFilter.
 */

#include "cartesian_velocity_controller/joint_velocity_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cartesian_velocity_controller
{

namespace
{
constexpr double kEpsilon = 1e-12;
}

JointVelocityFilter::JointVelocityFilter(std::size_t num_joints)
  : num_joints_(num_joints)
  , max_velocity_(Eigen::VectorXd::Constant(static_cast<int>(num_joints), 1.0))
  , max_acceleration_(Eigen::VectorXd::Constant(static_cast<int>(num_joints), 5.0))
  , max_jerk_(Eigen::VectorXd::Constant(static_cast<int>(num_joints), 50.0))
  , qdot_(Eigen::VectorXd::Zero(static_cast<int>(num_joints)))
  , qddot_(Eigen::VectorXd::Zero(static_cast<int>(num_joints)))
  , qjerk_(Eigen::VectorXd::Zero(static_cast<int>(num_joints)))
{
  if (num_joints_ == 0)
  {
    throw std::invalid_argument("JointVelocityFilter: num_joints must be > 0");
  }
}

void JointVelocityFilter::reset()
{
  qdot_.setZero();
  qddot_.setZero();
  qjerk_.setZero();
  accumulator_ = 0.0;
}

void JointVelocityFilter::resetToState(const Eigen::VectorXd& qdot,
                                       const Eigen::VectorXd& qddot,
                                       const Eigen::VectorXd& qjerk)
{
  if (qdot.size() != static_cast<int>(num_joints_))
  {
    throw std::invalid_argument("JointVelocityFilter::resetToState: qdot size mismatch");
  }

  qdot_ = qdot;

  if (qddot.size() == 0)
  {
    qddot_.setZero();
  }
  else
  {
    if (qddot.size() != static_cast<int>(num_joints_))
    {
      throw std::invalid_argument("JointVelocityFilter::resetToState: qddot size mismatch");
    }
    qddot_ = qddot;
  }

  if (qjerk.size() == 0)
  {
    qjerk_.setZero();
  }
  else
  {
    if (qjerk.size() != static_cast<int>(num_joints_))
    {
      throw std::invalid_argument("JointVelocityFilter::resetToState: qjerk size mismatch");
    }
    qjerk_ = qjerk;
  }

  accumulator_ = 0.0;
}

void JointVelocityFilter::setTimeConstant(double tau)
{
  // Keep consistent with the cartesian filter clamping philosophy
  tau_ = std::clamp(tau, 0.01, 1.0);
}

void JointVelocityFilter::setLimits(const Eigen::VectorXd& max_velocity,
                                    const Eigen::VectorXd& max_acceleration,
                                    const Eigen::VectorXd& max_jerk)
{
  if (max_velocity.size() != static_cast<int>(num_joints_) ||
      max_acceleration.size() != static_cast<int>(num_joints_) ||
      max_jerk.size() != static_cast<int>(num_joints_))
  {
    throw std::invalid_argument("JointVelocityFilter::setLimits: vector size mismatch");
  }

  max_velocity_ = max_velocity.cwiseAbs();
  max_acceleration_ = max_acceleration.cwiseAbs();
  max_jerk_ = max_jerk.cwiseAbs();
}

void JointVelocityFilter::setDtNominal(double dt_nominal)
{
  if (dt_nominal <= 0.0)
  {
    dt_nominal_ = 0.0;
    accumulator_ = 0.0;
    return;
  }
  dt_nominal_ = std::max(dt_nominal, 1e-9);
  accumulator_ = 0.0;
}

void JointVelocityFilter::setDtClamp(double min_dt, double max_dt)
{
  min_dt_clamp_ = std::max(min_dt, 1e-9);
  max_dt_clamp_ = std::max(max_dt, min_dt_clamp_);
}

void JointVelocityFilter::setMaxSubsteps(int max_substeps)
{
  max_substeps_ = std::max(0, max_substeps);
}

void JointVelocityFilter::setResetDtThreshold(double reset_dt_threshold)
{
  reset_dt_threshold_ = std::max(0.0, reset_dt_threshold);
}

double JointVelocityFilter::computeUniformScalingFactor(const Eigen::VectorXd& v,
                                                        const Eigen::VectorXd& vmax)
{
  double s = 1.0;
  const int n = v.size();
  for (int i = 0; i < n; ++i)
  {
    const double abs_vi = std::abs(v[i]);
    const double lim = vmax[i];
    if (abs_vi > lim && abs_vi > kEpsilon)
    {
      s = std::min(s, lim / abs_vi);
    }
  }
  return std::clamp(s, 0.0, 1.0);
}

void JointVelocityFilter::clampInPlace(Eigen::VectorXd& v, const Eigen::VectorXd& vmax)
{
  const int n = v.size();
  for (int i = 0; i < n; ++i)
  {
    v[i] = std::clamp(v[i], -vmax[i], vmax[i]);
  }
}

void JointVelocityFilter::step(const Eigen::VectorXd& desired_qdot, double dt)
{
  // 1) desired acceleration from tau
  const Eigen::VectorXd qddot_des = (desired_qdot - qdot_) / tau_;

  // 2) commanded jerk
  Eigen::VectorXd qjerk_cmd = (qddot_des - qddot_) / dt;

  // 3) jerk limiting
  if (uniform_scaling_enabled_)
  {
    const double s = computeUniformScalingFactor(qjerk_cmd, max_jerk_);
    qjerk_ = qjerk_cmd * s;
  }
  else
  {
    qjerk_ = qjerk_cmd;
    clampInPlace(qjerk_, max_jerk_);
  }

  // 4) integrate acceleration + limit
  Eigen::VectorXd qddot_new = qddot_ + qjerk_ * dt;
  if (uniform_scaling_enabled_)
  {
    const double s = computeUniformScalingFactor(qddot_new, max_acceleration_);
    qddot_new *= s;
  }
  else
  {
    clampInPlace(qddot_new, max_acceleration_);
  }

  // 5) integrate velocity + limit
  Eigen::VectorXd qdot_new = qdot_ + qddot_new * dt;
  if (uniform_scaling_enabled_)
  {
    const double s = computeUniformScalingFactor(qdot_new, max_velocity_);
    qdot_new *= s;
  }
  else
  {
    clampInPlace(qdot_new, max_velocity_);
  }

  // 6) update state
  qdot_ = qdot_new;
  qddot_ = qddot_new;
}

Eigen::VectorXd JointVelocityFilter::filter(const Eigen::VectorXd& desired_qdot, double dt)
{
  if (desired_qdot.size() != static_cast<int>(num_joints_))
  {
    // In a realtime loop it's safer to avoid throwing; just hold last.
    return qdot_;
  }

  // Fixed-dt mode
  if (dt_nominal_ > 0.0)
  {
    if (dt > reset_dt_threshold_)
    {
      if (large_dt_policy_ == LargeDtPolicy::HOLD_LAST)
      {
        return qdot_;
      }
      else if (large_dt_policy_ == LargeDtPolicy::RESET_TO_ZERO)
      {
        reset();
        return qdot_;
      }
      else  // RESET_TO_DESIRED
      {
        qdot_ = desired_qdot;
        qddot_.setZero();
        qjerk_.setZero();
        accumulator_ = 0.0;
        return qdot_;
      }
    }

    const double dt_clamped = std::clamp(dt, min_dt_clamp_, max_dt_clamp_);
    accumulator_ += dt_clamped;

    int steps = 0;
    while (accumulator_ >= dt_nominal_ && steps < max_substeps_)
    {
      const double step_dt = dt_nominal_;
      if (step_dt > kEpsilon)
      {
        if (enabled_)
        {
          step(desired_qdot, step_dt);
        }
        else
        {
          // Pass-through but update state for continuity
          const Eigen::VectorXd qddot_new = (desired_qdot - qdot_) / step_dt;
          qjerk_ = (qddot_new - qddot_) / step_dt;
          qddot_ = qddot_new;
          qdot_ = desired_qdot;
        }
      }

      accumulator_ -= dt_nominal_;
      steps++;
    }

    return qdot_;
  }

  // Raw-dt mode
  if (dt < min_dt_clamp_)
  {
    return qdot_;
  }

  if (enabled_)
  {
    step(desired_qdot, dt);
    return qdot_;
  }

  // Pass-through raw-dt
  const Eigen::VectorXd qddot_new = (desired_qdot - qdot_) / dt;
  qjerk_ = (qddot_new - qddot_) / dt;
  qddot_ = qddot_new;
  qdot_ = desired_qdot;
  return qdot_;
}

}  // namespace cartesian_velocity_controller

