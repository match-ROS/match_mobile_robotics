/**
 * @file waypoint_animator.cpp
 * @brief Implementation of waypoint-based animation controller
 */

#include "scene_builder/animation/waypoint_animator.hpp"
#include "scene_builder/core/pose_utils.hpp"

#include <algorithm>

namespace scene_builder
{
namespace animation
{

WaypointAnimator::WaypointAnimator(double default_duration)
  : default_duration_(default_duration)
{
}

void WaypointAnimator::queueWaypoint(objects::ObjectState& state, const objects::WaypointCommand& cmd)
{
  objects::WaypointCommand sanitized_cmd = cmd;
  sanitized_cmd.target_pose = core::sanitizePose(cmd.target_pose);
  sanitized_cmd.started = false;
  state.waypoint_queue.push_back(sanitized_cmd);
}

void WaypointAnimator::clearQueue(objects::ObjectState& state)
{
  state.waypoint_queue.clear();
}

bool WaypointAnimator::hasActiveAnimation(const objects::ObjectState& state) const
{
  return !state.waypoint_queue.empty();
}

std::optional<geometry_msgs::Pose> WaypointAnimator::update(objects::ObjectState& state, const ros::Time& now)
{
  if (state.waypoint_queue.empty())
  {
    return std::nullopt;
  }

  objects::WaypointCommand& cmd = state.waypoint_queue.front();

  // Start the waypoint if not already started
  if (!cmd.started)
  {
    cmd.start_time = now;
    cmd.start_pose = state.last_pose;
    cmd.started = true;
  }

  // Calculate interpolation ratio (0..1)
  double duration = cmd.duration_hint.toSec();
  if (duration < 1e-6)
  {
    duration = default_duration_;
  }

  double ratio = 1.0;
  if (duration > 1e-6)
  {
    const double elapsed = (now - cmd.start_time).toSec();
    ratio = std::clamp(elapsed / duration, 0.0, 1.0);
  }

  // Interpolate pose
  geometry_msgs::Pose interpolated = core::interpolatePose(cmd.start_pose, cmd.target_pose, ratio);

  // Check if waypoint is complete
  if (ratio >= 1.0 - 1e-3)
  {
    // Use exact target pose when complete
    interpolated = cmd.target_pose;
    state.waypoint_queue.pop_front();
  }

  return interpolated;
}

}  // namespace animation
}  // namespace scene_builder

