#pragma once

#include <string>
#include <unordered_map>

namespace cartesian_velocity_controller
{

struct JointRuntimeGuardConfig
{
  bool enabled{false};

  // Start braking when distance to the limit (along motion direction) is < soft_zone [rad]
  double soft_zone{0.0};

  // Hard numeric margin [rad]. Inside this margin (and moving towards the limit),
  // a reentry velocity can be forced.
  double margin{0.0};

  // Forced reentry velocity magnitude [rad/s] when inside margin and moving toward the limit.
  double reentry_velocity{0.0};
};

struct ControllerJointLimit
{
  bool enabled{false};
  double min{0.0};  // rad
  double max{0.0};  // rad

  JointRuntimeGuardConfig runtime_guard{};
};

struct ControllerJointLimitsConfig
{
  bool enabled{false};
  std::unordered_map<std::string, ControllerJointLimit> limits;

  bool hasAnyEnabledLimit() const
  {
    for (const auto& kv : limits)
    {
      if (kv.second.enabled) return true;
    }
    return false;
  }

  bool hasAnyRuntimeGuardEnabled() const
  {
    for (const auto& kv : limits)
    {
      if (kv.second.enabled && kv.second.runtime_guard.enabled) return true;
    }
    return false;
  }
};

struct ElbowInjectionConfig
{
  bool enabled{false};

  // Prefer name-based mapping (robust across joint order changes)
  std::string elbow_joint_name{"elbow_joint"};
  int elbow_index{2};

  // Zone near the controller-only MAX limit [rad]
  double critical_zone{0.40};

  // Target gating threshold [rad]
  double target_near_limit_threshold{0.25};

  double k{1.0};
  double max_push_velocity{0.20};  // rad/s
};

}  // namespace cartesian_velocity_controller

