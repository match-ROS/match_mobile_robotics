/**
 * @file joint_safety_limiter.cpp
 * @brief Implementation of the JointSafetyLimiter component.
 */

#include "teleoperation/components/joint_safety_limiter.hpp"

#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace teleoperation
{

JointSafetyLimiter::JointSafetyLimiter(std::size_t num_joints)
  : num_joints_(num_joints)
  , max_joint_velocities_(Eigen::VectorXd::Constant(num_joints, 1.0))
  , max_joint_accelerations_(Eigen::VectorXd::Constant(num_joints, 5.0))
{
  if (num_joints == 0)
  {
    throw std::invalid_argument("JointSafetyLimiter: num_joints must be > 0");
  }
  ROS_DEBUG_NAMED("teleop_joint_safety_limiter", "JointSafetyLimiter initialized with %zu joints", num_joints);
}

JointSafetyLimiter::JointSafetyLimiter(const Eigen::VectorXd& max_velocities,
                                       const Eigen::VectorXd& max_accelerations)
  : num_joints_(static_cast<std::size_t>(max_velocities.size()))
  , max_joint_velocities_(max_velocities.cwiseAbs())
  , max_joint_accelerations_(max_accelerations.cwiseAbs())
{
  if (max_velocities.size() != max_accelerations.size())
  {
    throw std::invalid_argument("JointSafetyLimiter: velocity and acceleration vector sizes must match");
  }
  if (num_joints_ == 0)
  {
    throw std::invalid_argument("JointSafetyLimiter: vectors cannot be empty");
  }
  ROS_DEBUG_NAMED("teleop_joint_safety_limiter", "JointSafetyLimiter initialized with %zu joints", num_joints_);
}

SafetyLimiterOutput JointSafetyLimiter::limit(const Eigen::VectorXd& commanded_velocity,
                                              const Eigen::VectorXd& previous_velocity,
                                              double dt)
{
  SafetyLimiterOutput output;

  if (commanded_velocity.size() != static_cast<int>(num_joints_))
  {
    ROS_ERROR_NAMED("teleop_joint_safety_limiter",
                    "Input velocity size (%d) does not match joint count (%zu)",
                    static_cast<int>(commanded_velocity.size()), num_joints_);
    output.joint_velocity = Eigen::VectorXd::Zero(static_cast<int>(num_joints_));
    output.scaling_factor = 0.0;
    return output;
  }

  double scaling_factor = 1.0;
  SafetyLimiterOutput::LimitType limit_type = SafetyLimiterOutput::LimitType::NONE;
  int limiting_joint = -1;

  // Velocity limits
  if (velocity_limiting_enabled_)
  {
    int vel_limiting_joint = -1;
    const double vel_factor = computeVelocityScalingFactor(commanded_velocity, vel_limiting_joint);
    if (vel_factor < scaling_factor)
    {
      scaling_factor = vel_factor;
      limit_type = SafetyLimiterOutput::LimitType::VELOCITY;
      limiting_joint = vel_limiting_joint;
    }
  }

  // Acceleration limits
  if (acceleration_limiting_enabled_ && dt > kEpsilon &&
      previous_velocity.size() == static_cast<int>(num_joints_))
  {
    int acc_limiting_joint = -1;
    const double acc_factor = computeAccelerationScalingFactor(commanded_velocity,
                                                               previous_velocity,
                                                               dt,
                                                               acc_limiting_joint);
    if (acc_factor < scaling_factor)
    {
      scaling_factor = acc_factor;
      limit_type = SafetyLimiterOutput::LimitType::ACCELERATION;
      limiting_joint = acc_limiting_joint;
    }
  }

  scaling_factor = std::max(0.0, std::min(1.0, scaling_factor));

  output.joint_velocity = commanded_velocity * scaling_factor;
  output.scaling_factor = scaling_factor;
  output.limit_type = limit_type;
  output.limiting_joint = limiting_joint;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_scaling_factor_ = scaling_factor;
    last_limit_type_ = limit_type;
    last_limiting_joint_ = limiting_joint;
  }

  if (scaling_factor < 0.99)
  {
    ROS_DEBUG_THROTTLE_NAMED(1.0, "teleop_joint_safety_limiter",
                             "Scaling applied: factor=%.3f, type=%s, joint=%d",
                             scaling_factor,
                             limit_type == SafetyLimiterOutput::LimitType::VELOCITY ? "VELOCITY" :
                             limit_type == SafetyLimiterOutput::LimitType::ACCELERATION ? "ACCELERATION" : "NONE",
                             limiting_joint);
  }

  return output;
}

Eigen::VectorXd JointSafetyLimiter::limitSimple(const Eigen::VectorXd& commanded_velocity,
                                                const Eigen::VectorXd& previous_velocity,
                                                double dt)
{
  return limit(commanded_velocity, previous_velocity, dt).joint_velocity;
}

void JointSafetyLimiter::setJointVelocityLimits(const Eigen::VectorXd& max_velocities)
{
  if (max_velocities.size() != static_cast<int>(num_joints_))
  {
    ROS_ERROR_NAMED("teleop_joint_safety_limiter",
                    "Velocity limits size (%d) does not match joint count (%zu)",
                    static_cast<int>(max_velocities.size()), num_joints_);
    return;
  }
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_velocities_ = max_velocities.cwiseAbs();
}

void JointSafetyLimiter::setJointVelocityLimit(std::size_t joint_index, double max_velocity)
{
  if (joint_index >= num_joints_)
  {
    ROS_ERROR_NAMED("teleop_joint_safety_limiter",
                    "Invalid joint index %zu (max: %zu)", joint_index, num_joints_ - 1);
    return;
  }
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_velocities_[static_cast<Eigen::Index>(joint_index)] = std::abs(max_velocity);
}

Eigen::VectorXd JointSafetyLimiter::getJointVelocityLimits() const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  return max_joint_velocities_;
}

void JointSafetyLimiter::setJointAccelerationLimits(const Eigen::VectorXd& max_accelerations)
{
  if (max_accelerations.size() != static_cast<int>(num_joints_))
  {
    ROS_ERROR_NAMED("teleop_joint_safety_limiter",
                    "Acceleration limits size (%d) does not match joint count (%zu)",
                    static_cast<int>(max_accelerations.size()), num_joints_);
    return;
  }
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_accelerations_ = max_accelerations.cwiseAbs();
}

void JointSafetyLimiter::setJointAccelerationLimit(std::size_t joint_index, double max_acceleration)
{
  if (joint_index >= num_joints_)
  {
    ROS_ERROR_NAMED("teleop_joint_safety_limiter",
                    "Invalid joint index %zu (max: %zu)", joint_index, num_joints_ - 1);
    return;
  }
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_accelerations_[static_cast<Eigen::Index>(joint_index)] = std::abs(max_acceleration);
}

Eigen::VectorXd JointSafetyLimiter::getJointAccelerationLimits() const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  return max_joint_accelerations_;
}

void JointSafetyLimiter::setUniformLimits(double max_velocity, double max_acceleration)
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_velocities_ = Eigen::VectorXd::Constant(static_cast<int>(num_joints_), std::abs(max_velocity));
  max_joint_accelerations_ = Eigen::VectorXd::Constant(static_cast<int>(num_joints_), std::abs(max_acceleration));
}

double JointSafetyLimiter::getLastScalingFactor() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_scaling_factor_;
}

bool JointSafetyLimiter::wasLimited() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_scaling_factor_ < (1.0 - kEpsilon);
}

SafetyLimiterOutput::LimitType JointSafetyLimiter::getLastLimitType() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_limit_type_;
}

int JointSafetyLimiter::getLastLimitingJoint() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_limiting_joint_;
}

void JointSafetyLimiter::setVelocityLimitingEnabled(bool enabled)
{
  velocity_limiting_enabled_ = enabled;
  ROS_DEBUG_NAMED("teleop_joint_safety_limiter", "Velocity limiting %s", enabled ? "enabled" : "disabled");
}

bool JointSafetyLimiter::isVelocityLimitingEnabled() const
{
  return velocity_limiting_enabled_;
}

void JointSafetyLimiter::setAccelerationLimitingEnabled(bool enabled)
{
  acceleration_limiting_enabled_ = enabled;
  ROS_DEBUG_NAMED("teleop_joint_safety_limiter", "Acceleration limiting %s", enabled ? "enabled" : "disabled");
}

bool JointSafetyLimiter::isAccelerationLimitingEnabled() const
{
  return acceleration_limiting_enabled_;
}

void JointSafetyLimiter::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_scaling_factor_ = 1.0;
  last_limit_type_ = SafetyLimiterOutput::LimitType::NONE;
  last_limiting_joint_ = -1;
}

double JointSafetyLimiter::computeVelocityScalingFactor(const Eigen::VectorXd& velocity,
                                                        int& limiting_joint) const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  double min_factor = 1.0;
  limiting_joint = -1;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    const double vel_abs = std::abs(velocity[static_cast<Eigen::Index>(i)]);
    const double max_vel = max_joint_velocities_[static_cast<Eigen::Index>(i)];
    if (vel_abs > max_vel && vel_abs > kEpsilon)
    {
      const double factor = max_vel / vel_abs;
      if (factor < min_factor)
      {
        min_factor = factor;
        limiting_joint = static_cast<int>(i);
      }
    }
  }
  return min_factor;
}

double JointSafetyLimiter::computeAccelerationScalingFactor(const Eigen::VectorXd& velocity,
                                                            const Eigen::VectorXd& prev_velocity,
                                                            double dt,
                                                            int& limiting_joint) const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  double min_factor = 1.0;
  limiting_joint = -1;

  if (dt < kEpsilon) return 1.0;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const double delta_v = velocity[idx] - prev_velocity[idx];
    const double acc = delta_v / dt;
    const double acc_abs = std::abs(acc);
    const double max_acc = max_joint_accelerations_[idx];

    if (acc_abs > max_acc && acc_abs > kEpsilon)
    {
      const double sign = (delta_v >= 0.0) ? 1.0 : -1.0;
      const double v_max_reachable = prev_velocity[idx] + sign * max_acc * dt;

      const double vel_abs = std::abs(velocity[idx]);
      if (vel_abs > kEpsilon)
      {
        const double factor = v_max_reachable / velocity[idx];
        if (factor > 0.0 && factor < min_factor)
        {
          min_factor = factor;
          limiting_joint = static_cast<int>(i);
        }
        else if (factor <= 0.0)
        {
          const double safe_factor = std::abs(prev_velocity[idx]) / vel_abs;
          if (safe_factor < min_factor && safe_factor >= 0.0)
          {
            min_factor = std::max(0.0, safe_factor);
            limiting_joint = static_cast<int>(i);
          }
        }
      }
    }
  }

  min_factor = std::max(0.0, std::min(1.0, min_factor));
  return min_factor;
}

}  // namespace teleoperation

