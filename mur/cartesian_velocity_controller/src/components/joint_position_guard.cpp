/**
 * @file joint_position_guard.cpp
 * @brief Runtime joint position guard implementation.
 */

#include "cartesian_velocity_controller/components/joint_position_guard.hpp"

#include <algorithm>
#include <cmath>

namespace cartesian_velocity_controller
{

double JointPositionGuard::clamp01(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

double JointPositionGuard::smoothstep(double x)
{
  const double t = clamp01(x);
  return t * t * (3.0 - 2.0 * t);
}

void JointPositionGuard::configure(const ControllerJointLimitsConfig& cfg,
                                  const std::unordered_map<std::string, std::size_t>& joint_index_map)
{
  entries_.clear();
  enabled_ = false;

  if (!cfg.enabled) return;

  for (const auto& kv : cfg.limits)
  {
    const std::string& name = kv.first;
    const ControllerJointLimit& lim = kv.second;
    if (!lim.enabled || !lim.runtime_guard.enabled) continue;

    auto it = joint_index_map.find(name);
    if (it == joint_index_map.end()) continue;

    Entry e;
    e.joint_name = name;
    e.index = static_cast<int>(it->second);
    e.min = lim.min;
    e.max = lim.max;
    e.runtime = lim.runtime_guard;
    entries_.push_back(e);
  }

  enabled_ = !entries_.empty();
}

void JointPositionGuard::apply(const Eigen::VectorXd& joint_positions, Eigen::VectorXd& joint_velocities) const
{
  if (!enabled_) return;
  if (joint_positions.size() != joint_velocities.size()) return;

  for (const auto& e : entries_)
  {
    if (e.index < 0 || e.index >= joint_velocities.size()) continue;

    const double q = joint_positions[e.index];
    double qdot = joint_velocities[e.index];

    if (!std::isfinite(q) || !std::isfinite(qdot)) continue;
    if (std::abs(qdot) < 1e-12) continue;

    const double soft_zone = std::max(0.0, e.runtime.soft_zone);
    const double margin = std::max(0.0, e.runtime.margin);
    const double reentry = std::max(0.0, e.runtime.reentry_velocity);

    // Distance to the relevant limit along the motion direction.
    if (qdot > 0.0)
    {
      const double d = e.max - q;

      // Hard margin: if inside (or beyond) and still going towards max, force re-entry.
      if (d <= margin)
      {
        joint_velocities[e.index] = (reentry > 0.0) ? (-reentry) : 0.0;
        continue;
      }

      // Soft braking: scale down as you approach max.
      if (soft_zone > 0.0 && d < soft_zone)
      {
        const double x = clamp01(d / soft_zone);
        const double s = smoothstep(x);
        joint_velocities[e.index] = s * qdot;
        continue;
      }
    }
    else  // qdot < 0
    {
      const double d = q - e.min;

      if (d <= margin)
      {
        joint_velocities[e.index] = (reentry > 0.0) ? (+reentry) : 0.0;
        continue;
      }

      if (soft_zone > 0.0 && d < soft_zone)
      {
        const double x = clamp01(d / soft_zone);
        const double s = smoothstep(x);
        joint_velocities[e.index] = s * qdot;
        continue;
      }
    }
  }
}

bool JointPositionGuard::updateRuntimeGuard(const std::string& joint_name, double soft_zone, double margin, double reentry_velocity)
{
  for (auto& e : entries_)
  {
    if (e.joint_name == joint_name)
    {
      e.runtime.soft_zone = std::max(0.0, soft_zone);
      e.runtime.margin = std::max(0.0, margin);
      e.runtime.reentry_velocity = std::max(0.0, reentry_velocity);
      return true;
    }
  }
  return false;
}

bool JointPositionGuard::getRuntimeGuard(const std::string& joint_name, JointRuntimeGuardConfig& config) const
{
  for (const auto& e : entries_)
  {
    if (e.joint_name == joint_name)
    {
      config = e.runtime;
      return true;
    }
  }
  return false;
}

bool JointPositionGuard::updateJointLimits(const std::string& joint_name, double min, double max)
{
  for (auto& e : entries_)
  {
    if (e.joint_name == joint_name)
    {
      e.min = min;
      e.max = max;
      return true;
    }
  }
  return false;
}

bool JointPositionGuard::getJointLimits(const std::string& joint_name, double& min, double& max) const
{
  for (const auto& e : entries_)
  {
    if (e.joint_name == joint_name)
    {
      min = e.min;
      max = e.max;
      return true;
    }
  }
  return false;
}

}  // namespace cartesian_velocity_controller

