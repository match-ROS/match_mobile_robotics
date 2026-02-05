/**
 * @file velocity_estimator.cpp
 * @brief Implementation of velocity estimation for collision objects
 */

#include "scene_builder/distance/velocity_estimator.hpp"

#include <moveit/planning_scene/planning_scene.h>

#include <ros/console.h>

namespace scene_builder
{
namespace distance
{

VelocityEstimator::VelocityEstimator(const VelocityEstimatorConfig& config)
  : config_(config)
{
}

void VelocityEstimator::update(
    const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor,
    const ros::Time& current_time)
{
  if (!scene_monitor)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Get current object poses from planning scene
  std::map<std::string, Eigen::Isometry3d> current_poses;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene_ro(scene_monitor);
    if (!scene_ro)
    {
      ROS_WARN_THROTTLE(1.0, "VelocityEstimator: Cannot get planning scene lock");
      return;
    }

    const collision_detection::WorldConstPtr& world = scene_ro->getWorld();
    if (!world)
    {
      return;
    }

    // Get all object IDs
    const std::vector<std::string>& object_ids = world->getObjectIds();
    for (const auto& id : object_ids)
    {
      bool found = false;
      const Eigen::Isometry3d& pose = world->getTransform(id, found);
      if (found)
      {
        current_poses[id] = pose;
      }
    }
  }

  // Track which objects are still present
  std::set<std::string> present_objects;

  // Update velocity estimates for each object
  for (const auto& [object_id, current_pose] : current_poses)
  {
    present_objects.insert(object_id);

    auto it = object_states_.find(object_id);
    if (it == object_states_.end())
    {
      // New object - initialize state
      ObjectState state;
      state.pose = current_pose;
      state.timestamp = current_time;
      state.initialized = true;
      state.velocity.valid = false;
      object_states_[object_id] = state;
      continue;
    }

    ObjectState& state = it->second;

    if (!state.initialized)
    {
      // First time seeing this object
      state.pose = current_pose;
      state.timestamp = current_time;
      state.initialized = true;
      state.velocity.valid = false;
      continue;
    }

    // Compute time delta
    double dt = (current_time - state.timestamp).toSec();

    if (dt < config_.min_dt)
    {
      // Time delta too small, skip update
      continue;
    }

    if (dt > config_.max_dt)
    {
      // Time delta too large, reset and invalidate velocity
      state.pose = current_pose;
      state.timestamp = current_time;
      state.velocity.valid = false;
      continue;
    }

    // Compute linear velocity
    Eigen::Vector3d position_diff = current_pose.translation() - state.pose.translation();
    Eigen::Vector3d linear_velocity = position_diff / dt;

    // Clamp linear velocity
    linear_velocity = clampVelocity(linear_velocity, config_.max_linear_velocity);

    // Compute angular velocity
    Eigen::Vector3d angular_velocity = computeAngularVelocity(
        state.pose.rotation(), current_pose.rotation(), dt);

    // Clamp angular velocity
    angular_velocity = clampVelocity(angular_velocity, config_.max_angular_velocity);

    // Apply low-pass filter if we have a previous estimate
    if (state.velocity.valid)
    {
      linear_velocity = lowPassFilter(state.velocity.linear, linear_velocity,
                                       config_.velocity_filter_alpha);
      angular_velocity = lowPassFilter(state.velocity.angular, angular_velocity,
                                         config_.velocity_filter_alpha);
    }

    // Update state
    state.pose = current_pose;
    state.timestamp = current_time;
    state.velocity.linear = linear_velocity;
    state.velocity.angular = angular_velocity;
    state.velocity.valid = true;
  }

  // Remove objects that are no longer present
  for (auto it = object_states_.begin(); it != object_states_.end();)
  {
    if (present_objects.find(it->first) == present_objects.end())
    {
      it = object_states_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

ObjectVelocity VelocityEstimator::getVelocity(const std::string& object_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = object_states_.find(object_id);
  if (it != object_states_.end())
  {
    return it->second.velocity;
  }

  return ObjectVelocity{};  // Return invalid velocity
}

std::map<std::string, ObjectVelocity> VelocityEstimator::getAllVelocities() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<std::string, ObjectVelocity> result;
  for (const auto& [id, state] : object_states_)
  {
    result[id] = state.velocity;
  }
  return result;
}

void VelocityEstimator::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  object_states_.clear();
}

void VelocityEstimator::setConfig(const VelocityEstimatorConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

Eigen::Vector3d VelocityEstimator::computeAngularVelocity(
    const Eigen::Matrix3d& R1,
    const Eigen::Matrix3d& R2,
    double dt) const
{
  // Compute relative rotation: R_rel = R2 * R1^T
  Eigen::Matrix3d R_rel = R2 * R1.transpose();

  // Convert to angle-axis representation
  Eigen::AngleAxisd angle_axis(R_rel);

  // Angular velocity = axis * angle / dt
  return angle_axis.axis() * angle_axis.angle() / dt;
}

Eigen::Vector3d VelocityEstimator::lowPassFilter(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& measurement,
    double alpha) const
{
  // Simple exponential moving average: y = alpha * x + (1 - alpha) * y_prev
  return alpha * measurement + (1.0 - alpha) * current;
}

Eigen::Vector3d VelocityEstimator::clampVelocity(
    const Eigen::Vector3d& velocity,
    double max_magnitude) const
{
  double magnitude = velocity.norm();
  if (magnitude > max_magnitude && magnitude > 1e-6)
  {
    return velocity * (max_magnitude / magnitude);
  }
  return velocity;
}

}  // namespace distance
}  // namespace scene_builder
