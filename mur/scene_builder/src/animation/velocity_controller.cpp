/**
 * @file velocity_controller.cpp
 * @brief Implementation of velocity-based motion controller
 */

#include "scene_builder/animation/velocity_controller.hpp"
#include "scene_builder/core/pose_utils.hpp"

#include <cmath>
#include <algorithm>

namespace scene_builder
{
namespace animation
{

VelocityController::VelocityController(double max_velocity, double default_timeout)
  : max_velocity_(max_velocity)
  , default_timeout_(default_timeout)
{
}

void VelocityController::applyVelocity(objects::ObjectState& state, const objects::VelocityCommand& cmd)
{
  // Clamp velocity to maximum allowed
  objects::VelocityCommand clamped_cmd = cmd;
  clamped_cmd.twist = clampVelocity(cmd.twist);
  clamped_cmd.stamp = ros::Time::now();

  state.active_velocity = clamped_cmd;

  // Clear waypoint queue when velocity is applied
  state.waypoint_queue.clear();
}

void VelocityController::stopVelocity(objects::ObjectState& state)
{
  state.active_velocity.reset();
}

bool VelocityController::hasActiveVelocity(const objects::ObjectState& state) const
{
  return state.active_velocity.has_value();
}

std::optional<geometry_msgs::Pose> VelocityController::update(objects::ObjectState& state,
                                                              const ros::Time& now,
                                                              double dt)
{
  if (!state.active_velocity)
  {
    return std::nullopt;
  }

  // Check timeout
  const ros::Duration since_command = now - state.active_velocity->stamp;
  ros::Duration timeout = state.active_velocity->timeout;

  // Use default timeout if not specified
  if (timeout <= ros::Duration(0.0))
  {
    timeout = ros::Duration(default_timeout_);
  }

  // Deactivate if timeout expired
  if (timeout > ros::Duration(0.0) && since_command > timeout)
  {
    state.active_velocity.reset();
    return std::nullopt;
  }

  // Integrate velocity into pose
  geometry_msgs::Pose new_pose = core::integrateTwist(
      state.last_pose,
      state.active_velocity->twist,
      dt);

  return new_pose;
}

geometry_msgs::Twist VelocityController::clampVelocity(const geometry_msgs::Twist& twist) const
{
  if (max_velocity_ <= 0.0)
  {
    return twist;  // No clamping if max_velocity is not set
  }

  // Calculate velocity norms
  const double lin_norm = std::sqrt(
      twist.linear.x * twist.linear.x +
      twist.linear.y * twist.linear.y +
      twist.linear.z * twist.linear.z);

  const double ang_norm = std::sqrt(
      twist.angular.x * twist.angular.x +
      twist.angular.y * twist.angular.y +
      twist.angular.z * twist.angular.z);

  const double norm = std::max(lin_norm, ang_norm);

  // Clamp if exceeds maximum
  if (norm > max_velocity_ && norm > 1e-6)
  {
    const double scale = max_velocity_ / norm;
    geometry_msgs::Twist clamped;
    clamped.linear.x = twist.linear.x * scale;
    clamped.linear.y = twist.linear.y * scale;
    clamped.linear.z = twist.linear.z * scale;
    clamped.angular.x = twist.angular.x * scale;
    clamped.angular.y = twist.angular.y * scale;
    clamped.angular.z = twist.angular.z * scale;
    return clamped;
  }

  return twist;
}

}  // namespace animation
}  // namespace scene_builder

