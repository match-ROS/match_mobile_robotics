/**
 * @file global_planner.cpp
 * @brief Implementation of the GlobalPlanner component.
 */

#include "cartesian_velocity_controller/components/global_planner.hpp"

#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace cartesian_velocity_controller
{

GlobalPlanner::GlobalPlanner()
{
  ROS_DEBUG_NAMED("global_planner", "GlobalPlanner initialized");
}

// ============== Waypoint Management ==============

void GlobalPlanner::setWaypoints(const std::vector<Eigen::Isometry3d>& waypoints)
{
  std::vector<WaypointInfo> infos;
  infos.reserve(waypoints.size());
  for (const auto& wp : waypoints)
  {
    infos.emplace_back(wp);
  }
  setWaypoints(infos);
}

void GlobalPlanner::setWaypoints(const std::vector<WaypointInfo>& waypoints)
{
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  ROS_INFO_NAMED("global_planner", "Set %zu waypoints", waypoints.size());
}

void GlobalPlanner::addWaypoint(const Eigen::Isometry3d& waypoint)
{
  addWaypoint(WaypointInfo(waypoint));
}

void GlobalPlanner::addWaypoint(const WaypointInfo& waypoint)
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  waypoints_.push_back(waypoint);
  ROS_DEBUG_NAMED("global_planner", "Added waypoint, total: %zu", waypoints_.size());
}

bool GlobalPlanner::insertWaypoint(std::size_t index, const Eigen::Isometry3d& waypoint)
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (index > waypoints_.size())
  {
    ROS_WARN_NAMED("global_planner", "Invalid insert index %zu (size: %zu)", index, waypoints_.size());
    return false;
  }

  waypoints_.insert(waypoints_.begin() + index, WaypointInfo(waypoint));

  // Adjust current index if needed
  if (index <= current_waypoint_index_ && !waypoints_.empty())
  {
    current_waypoint_index_++;
  }

  ROS_DEBUG_NAMED("global_planner", "Inserted waypoint at index %zu", index);
  return true;
}

bool GlobalPlanner::removeWaypoint(std::size_t index)
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (index >= waypoints_.size())
  {
    ROS_WARN_NAMED("global_planner", "Invalid remove index %zu (size: %zu)", index, waypoints_.size());
    return false;
  }

  waypoints_.erase(waypoints_.begin() + index);

  // Adjust current index if needed
  if (current_waypoint_index_ >= waypoints_.size() && !waypoints_.empty())
  {
    current_waypoint_index_ = waypoints_.size() - 1;
  }

  ROS_DEBUG_NAMED("global_planner", "Removed waypoint at index %zu", index);
  return true;
}

void GlobalPlanner::clearWaypoints()
{
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_.clear();
    current_waypoint_index_ = 0;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  ROS_INFO_NAMED("global_planner", "Cleared all waypoints");
}

std::size_t GlobalPlanner::getWaypointCount() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  return waypoints_.size();
}

bool GlobalPlanner::hasWaypoints() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  return !waypoints_.empty();
}

std::vector<WaypointInfo> GlobalPlanner::getWaypoints() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  return waypoints_;
}

// ============== Navigation ==============

Eigen::Isometry3d GlobalPlanner::getCurrentWaypoint() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty())
  {
    return Eigen::Isometry3d::Identity();
  }

  return waypoints_[current_waypoint_index_].pose;
}

WaypointInfo GlobalPlanner::getCurrentWaypointInfo() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty())
  {
    return WaypointInfo();
  }

  return waypoints_[current_waypoint_index_];
}

std::size_t GlobalPlanner::getCurrentWaypointIndex() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  return current_waypoint_index_;
}

bool GlobalPlanner::advanceToNextWaypoint()
{
  std::unique_lock<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty())
  {
    return false;
  }

  if (current_waypoint_index_ >= waypoints_.size() - 1)
  {
    // Already at final waypoint
    return false;
  }

  std::size_t old_index = current_waypoint_index_;
  current_waypoint_index_++;
  Eigen::Isometry3d reached_pose = waypoints_[old_index].pose;
  lock.unlock();

  // Reset distance tracking for new waypoint
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  // Invoke callback
  {
    std::lock_guard<std::mutex> cb_lock(callback_mutex_);
    if (waypoint_reached_callback_)
    {
      waypoint_reached_callback_(old_index, reached_pose);
    }
  }

  ROS_INFO_NAMED("global_planner", "Advanced to waypoint %zu", current_waypoint_index_);
  return true;
}

bool GlobalPlanner::jumpToWaypoint(std::size_t index)
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (index >= waypoints_.size())
  {
    ROS_WARN_NAMED("global_planner", "Invalid waypoint index %zu (size: %zu)", index, waypoints_.size());
    return false;
  }

  current_waypoint_index_ = index;

  // Reset distance tracking
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  ROS_INFO_NAMED("global_planner", "Jumped to waypoint %zu", index);
  return true;
}

bool GlobalPlanner::isAtFinalWaypoint() const
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);

  if (waypoints_.empty())
  {
    return false;
  }

  return current_waypoint_index_ >= waypoints_.size() - 1;
}

void GlobalPlanner::resetToStart()
{
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    current_waypoint_index_ = 0;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  ROS_INFO_NAMED("global_planner", "Reset to first waypoint");
}

// ============== Position Update and Switch Logic ==============

bool GlobalPlanner::updateCurrentPosition(const Eigen::Isometry3d& current_pose)
{
  Eigen::Isometry3d target_pose;
  double switch_dist;
  bool use_orientation;
  double orientation_thresh;
  double custom_switch_dist = 0.0;
  double custom_orientation_thresh = 0.0;
  bool custom_use_orientation = true;

  // Get current waypoint and parameters
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    if (waypoints_.empty())
    {
      return false;
    }
    target_pose = waypoints_[current_waypoint_index_].pose;

    // Get custom parameters if set
    const WaypointInfo& wp = waypoints_[current_waypoint_index_];
    custom_switch_dist = wp.switch_distance;
    custom_orientation_thresh = wp.orientation_threshold;
    custom_use_orientation = wp.use_orientation_for_switch;
  }

  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    switch_dist = (custom_switch_dist > 0.0) ? custom_switch_dist : waypoint_switch_distance_;
    use_orientation = custom_use_orientation && use_orientation_for_switch_;
    orientation_thresh = (custom_orientation_thresh > 0.0) ? custom_orientation_thresh : orientation_switch_threshold_;
  }

  // Compute distances
  double pos_dist = computePositionDistance(current_pose, target_pose);
  double ang_dist = computeOrientationDistance(current_pose, target_pose);

  // Update state
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_distance_ = pos_dist;
    last_angular_distance_ = ang_dist;
  }

  // Check if we should switch to next waypoint
  bool position_ok = pos_dist < switch_dist;
  bool orientation_ok = !use_orientation || (ang_dist < orientation_thresh);

  if (position_ok && orientation_ok)
  {
    // Check if this is the final waypoint
    bool is_final;
    {
      std::lock_guard<std::mutex> lock(waypoints_mutex_);
      is_final = (current_waypoint_index_ >= waypoints_.size() - 1);
    }

    if (is_final)
    {
      // Stay at final waypoint, call path completed callback once
      static bool path_completed_called = false;
      if (!path_completed_called)
      {
        std::lock_guard<std::mutex> cb_lock(callback_mutex_);
        if (path_completed_callback_)
        {
          path_completed_callback_();
        }
        path_completed_called = true;
      }
      return false;
    }

    // Advance to next waypoint
    return advanceToNextWaypoint();
  }

  return false;
}

double GlobalPlanner::getDistanceToCurrentWaypoint() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_position_distance_;
}

double GlobalPlanner::getAngularDistanceToCurrentWaypoint() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_angular_distance_;
}

// ============== Dynamic Parameters ==============

void GlobalPlanner::setWaypointSwitchDistance(double distance)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  waypoint_switch_distance_ = std::max(0.001, distance);  // Min 1mm
  ROS_DEBUG_NAMED("global_planner", "Waypoint switch distance set to %.3f m", waypoint_switch_distance_);
}

double GlobalPlanner::getWaypointSwitchDistance() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return waypoint_switch_distance_;
}

void GlobalPlanner::setUseOrientationForSwitch(bool use_orientation)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  use_orientation_for_switch_ = use_orientation;
  ROS_DEBUG_NAMED("global_planner", "Use orientation for switch: %s", use_orientation ? "true" : "false");
}

bool GlobalPlanner::getUseOrientationForSwitch() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return use_orientation_for_switch_;
}

void GlobalPlanner::setOrientationSwitchThreshold(double threshold)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  orientation_switch_threshold_ = std::max(0.001, threshold);  // Min 0.001 rad
  ROS_DEBUG_NAMED("global_planner", "Orientation switch threshold set to %.3f rad", orientation_switch_threshold_);
}

double GlobalPlanner::getOrientationSwitchThreshold() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return orientation_switch_threshold_;
}

// ============== Callbacks ==============

void GlobalPlanner::setWaypointReachedCallback(WaypointReachedCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  waypoint_reached_callback_ = callback;
}

void GlobalPlanner::setPathCompletedCallback(PathCompletedCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  path_completed_callback_ = callback;
}

// ============== Reset ==============

void GlobalPlanner::reset()
{
  {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    current_waypoint_index_ = 0;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_distance_ = std::numeric_limits<double>::infinity();
    last_angular_distance_ = 0.0;
  }

  ROS_DEBUG_NAMED("global_planner", "GlobalPlanner reset");
}

// ============== Private Methods ==============

double GlobalPlanner::computePositionDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  return (a.translation() - b.translation()).norm();
}

double GlobalPlanner::computeOrientationDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  // Compute quaternion difference and extract angle
  Eigen::Quaterniond q_a(a.rotation());
  Eigen::Quaterniond q_b(b.rotation());

  // Ensure we take the shorter path
  if (q_a.dot(q_b) < 0.0)
  {
    q_b.coeffs() = -q_b.coeffs();
  }

  // Compute relative rotation
  Eigen::Quaterniond q_diff = q_a.inverse() * q_b;

  // Extract angle (returns value in [0, pi])
  double angle = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

  return angle;
}

}  // namespace cartesian_velocity_controller

