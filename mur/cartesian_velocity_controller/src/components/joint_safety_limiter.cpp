/**
 * @file joint_safety_limiter.cpp
 * @brief Implementation of the JointSafetyLimiter component.
 */

#include "cartesian_velocity_controller/components/joint_safety_limiter.hpp"

#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace cartesian_velocity_controller
{

JointSafetyLimiter::JointSafetyLimiter(std::size_t num_joints)
  : num_joints_(num_joints)
  , max_joint_velocities_(Eigen::VectorXd::Constant(num_joints, 1.0))  // Default 1 rad/s
  , max_joint_accelerations_(Eigen::VectorXd::Constant(num_joints, 5.0))  // Default 5 rad/s²
{
  if (num_joints == 0)
  {
    throw std::invalid_argument("JointSafetyLimiter: num_joints must be > 0");
  }

  ROS_DEBUG_NAMED("joint_safety_limiter", "JointSafetyLimiter initialized with %zu joints", num_joints);
}

JointSafetyLimiter::JointSafetyLimiter(const Eigen::VectorXd& max_velocities,
                                       const Eigen::VectorXd& max_accelerations)
  : num_joints_(max_velocities.size())
  , max_joint_velocities_(max_velocities.cwiseAbs())  // Ensure positive
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

  ROS_DEBUG_NAMED("joint_safety_limiter", "JointSafetyLimiter initialized with %zu joints", num_joints_);
}

// ============== Main Limiting Function ==============

SafetyLimiterOutput JointSafetyLimiter::limit(const Eigen::VectorXd& commanded_velocity,
                                              const Eigen::VectorXd& previous_velocity,
                                              double dt)
{
  SafetyLimiterOutput output;

  // Validate input size
  if (commanded_velocity.size() != static_cast<int>(num_joints_))
  {
    ROS_ERROR_NAMED("joint_safety_limiter",
                    "Input velocity size (%d) does not match joint count (%zu)",
                    static_cast<int>(commanded_velocity.size()), num_joints_);
    output.joint_velocity = Eigen::VectorXd::Zero(num_joints_);
    output.scaling_factor = 0.0;
    return output;
  }

  double scaling_factor = 1.0;
  SafetyLimiterOutput::LimitType limit_type = SafetyLimiterOutput::LimitType::NONE;
  int limiting_joint = -1;

  // Check velocity limits
  if (velocity_limiting_enabled_)
  {
    int vel_limiting_joint = -1;
    double vel_factor = computeVelocityScalingFactor(commanded_velocity, vel_limiting_joint);

    if (vel_factor < scaling_factor)
    {
      scaling_factor = vel_factor;
      limit_type = SafetyLimiterOutput::LimitType::VELOCITY;
      limiting_joint = vel_limiting_joint;
    }
  }

  // Check acceleration limits
  if (acceleration_limiting_enabled_ && dt > kEpsilon &&
      previous_velocity.size() == static_cast<int>(num_joints_))
  {
    int acc_limiting_joint = -1;
    double acc_factor = computeAccelerationScalingFactor(commanded_velocity, previous_velocity,
                                                         dt, acc_limiting_joint);

    if (acc_factor < scaling_factor)
    {
      scaling_factor = acc_factor;
      limit_type = SafetyLimiterOutput::LimitType::ACCELERATION;
      limiting_joint = acc_limiting_joint;
    }
  }

  // Apply uniform scaling
  output.joint_velocity = commanded_velocity * scaling_factor;
  output.scaling_factor = scaling_factor;
  output.limit_type = limit_type;
  output.limiting_joint = limiting_joint;

  // Update internal state
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_scaling_factor_ = scaling_factor;
    last_limit_type_ = limit_type;
    last_limiting_joint_ = limiting_joint;
  }

  // Log if significant scaling occurred
  if (scaling_factor < 0.99)
  {
    ROS_DEBUG_THROTTLE_NAMED(1.0, "joint_safety_limiter",
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
  SafetyLimiterOutput output = limit(commanded_velocity, previous_velocity, dt);
  return output.joint_velocity;
}

// ============== Limit Configuration ==============

void JointSafetyLimiter::setJointVelocityLimits(const Eigen::VectorXd& max_velocities)
{
  if (max_velocities.size() != static_cast<int>(num_joints_))
  {
    ROS_ERROR_NAMED("joint_safety_limiter",
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
    ROS_ERROR_NAMED("joint_safety_limiter",
                    "Invalid joint index %zu (max: %zu)", joint_index, num_joints_ - 1);
    return;
  }

  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_velocities_[joint_index] = std::abs(max_velocity);
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
    ROS_ERROR_NAMED("joint_safety_limiter",
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
    ROS_ERROR_NAMED("joint_safety_limiter",
                    "Invalid joint index %zu (max: %zu)", joint_index, num_joints_ - 1);
    return;
  }

  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_accelerations_[joint_index] = std::abs(max_acceleration);
}

Eigen::VectorXd JointSafetyLimiter::getJointAccelerationLimits() const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  return max_joint_accelerations_;
}

void JointSafetyLimiter::setUniformLimits(double max_velocity, double max_acceleration)
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  max_joint_velocities_ = Eigen::VectorXd::Constant(num_joints_, std::abs(max_velocity));
  max_joint_accelerations_ = Eigen::VectorXd::Constant(num_joints_, std::abs(max_acceleration));
}

// ============== State ==============

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

// ============== Enable/Disable ==============

void JointSafetyLimiter::setVelocityLimitingEnabled(bool enabled)
{
  velocity_limiting_enabled_ = enabled;
  ROS_DEBUG_NAMED("joint_safety_limiter", "Velocity limiting %s",
                  enabled ? "enabled" : "disabled");
}

bool JointSafetyLimiter::isVelocityLimitingEnabled() const
{
  return velocity_limiting_enabled_;
}

void JointSafetyLimiter::setAccelerationLimitingEnabled(bool enabled)
{
  acceleration_limiting_enabled_ = enabled;
  ROS_DEBUG_NAMED("joint_safety_limiter", "Acceleration limiting %s",
                  enabled ? "enabled" : "disabled");
}

bool JointSafetyLimiter::isAccelerationLimitingEnabled() const
{
  return acceleration_limiting_enabled_;
}

// ============== Reset ==============

void JointSafetyLimiter::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_scaling_factor_ = 1.0;
  last_limit_type_ = SafetyLimiterOutput::LimitType::NONE;
  last_limiting_joint_ = -1;
  ROS_DEBUG_NAMED("joint_safety_limiter", "JointSafetyLimiter reset");
}

// ============== Private Methods ==============

double JointSafetyLimiter::computeVelocityScalingFactor(const Eigen::VectorXd& velocity,
                                                        int& limiting_joint) const
{
  std::lock_guard<std::mutex> lock(limits_mutex_);

  double min_factor = 1.0;
  limiting_joint = -1;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    double vel_abs = std::abs(velocity[i]);
    double max_vel = max_joint_velocities_[i];

    if (vel_abs > max_vel && vel_abs > kEpsilon)
    {
      double factor = max_vel / vel_abs;
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

  if (dt < kEpsilon)
  {
    return 1.0;  // Cannot compute acceleration with zero dt
  }

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    double delta_v = velocity[i] - prev_velocity[i];
    double acc = delta_v / dt;
    double acc_abs = std::abs(acc);
    double max_acc = max_joint_accelerations_[i];

    if (acc_abs > max_acc && acc_abs > kEpsilon)
    {
      // Compute the maximum achievable velocity given acceleration constraint
      // v_new = v_prev + sign(delta_v) * max_acc * dt
      // We need: |v_new - v_prev| / dt <= max_acc
      // So: |v_new| <= |v_prev| + max_acc * dt (if accelerating in same direction)
      //     |v_new| >= |v_prev| - max_acc * dt (if decelerating)

      // The scaling factor is computed differently for acceleration limiting:
      // We want to find factor such that:
      // |(velocity * factor) - prev_velocity| / dt <= max_acc
      // This is: |velocity * factor - prev_velocity| <= max_acc * dt

      // For simplicity, we approximate:
      // If we need to reduce velocity, the maximum achievable velocity is:
      // v_max_reachable = prev_velocity[i] + sign(delta_v) * max_acc * dt

      double sign = (delta_v >= 0) ? 1.0 : -1.0;
      double v_max_reachable = prev_velocity[i] + sign * max_acc * dt;

      // Compute factor to scale velocity[i] to v_max_reachable
      double vel_abs = std::abs(velocity[i]);
      if (vel_abs > kEpsilon)
      {
        // The velocity we can reach is v_max_reachable
        // We need: velocity[i] * factor = some value that respects acc limit
        // The maximum magnitude we can command is |v_max_reachable|

        // But we need uniform scaling, so we compute what factor would bring
        // this joint within its acceleration limit
        // |velocity[i] * factor - prev_velocity[i]| / dt = max_acc
        // velocity[i] * factor = prev_velocity[i] + sign * max_acc * dt
        // factor = (prev_velocity[i] + sign * max_acc * dt) / velocity[i]

        double factor = v_max_reachable / velocity[i];

        // Ensure factor is positive and meaningful
        if (factor > 0 && factor < min_factor)
        {
          min_factor = factor;
          limiting_joint = static_cast<int>(i);
        }
        else if (factor <= 0)
        {
          // Sign mismatch - the acceleration-limited velocity has opposite sign
          // This can happen during direction reversal
          // In this case, we need to limit more aggressively
          double safe_factor = std::abs(prev_velocity[i]) / vel_abs;
          if (safe_factor < min_factor && safe_factor >= 0)
          {
            min_factor = std::max(0.0, safe_factor);
            limiting_joint = static_cast<int>(i);
          }
        }
      }
    }
  }

  // Ensure factor is valid
  min_factor = std::max(0.0, std::min(1.0, min_factor));

  return min_factor;
}

}  // namespace cartesian_velocity_controller

