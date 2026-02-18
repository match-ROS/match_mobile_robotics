#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

#include "teleoperation/core/math_utils.hpp"

namespace teleoperation
{

class JerkLimiter3
{
public:
  JerkLimiter3() = default;

  void reset()
  {
    a_.setZero();
    initialized_ = false;
  }

  void setInitialAccel(const Eigen::Vector3d& a0)
  {
    a_ = a0;
    initialized_ = true;
  }

  const Eigen::Vector3d& accel() const { return a_; }

  Eigen::Vector3d step(const Eigen::Vector3d& a_des,
                       double dt,
                       const Eigen::Vector3d& max_accel_abs,
                       const Eigen::Vector3d& max_jerk_abs)
  {
    if (!(dt > 0.0) || !std::isfinite(dt))
    {
      return a_;
    }

    if (!initialized_)
    {
      a_ = a_des;
      a_ = clampAbs3(a_, max_accel_abs);
      initialized_ = true;
      return a_;
    }

    const Eigen::Vector3d da = a_des - a_;
    Eigen::Vector3d da_limited = da;
    for (int i = 0; i < 3; ++i)
    {
      const double j = std::max(0.0, max_jerk_abs[i]);
      const double max_da = j * dt;
      da_limited[i] = std::clamp(da[i], -max_da, max_da);
    }

    a_ += da_limited;
    a_ = clampAbs3(a_, max_accel_abs);
    return a_;
  }

private:
  Eigen::Vector3d a_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
};

}  // namespace teleoperation

