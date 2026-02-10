#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace teleoperation
{

constexpr double kMathEps = 1e-10;

inline double clamp01(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

inline Eigen::Vector3d clampNorm3(const Eigen::Vector3d& v, double max_norm)
{
  if (max_norm <= kMathEps)
  {
    return Eigen::Vector3d::Zero();
  }

  const double n = v.norm();
  if (n > max_norm && n > kMathEps)
  {
    return v * (max_norm / n);
  }
  return v;
}

inline Eigen::Vector3d applyDeadbandAbs3(const Eigen::Vector3d& v, double deadband_abs)
{
  if (deadband_abs <= 0.0)
  {
    return v;
  }

  Eigen::Vector3d out = v;
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(out[i]) < deadband_abs)
    {
      out[i] = 0.0;
    }
  }
  return out;
}

inline Eigen::Vector3d ema3(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return a * curr + (1.0 - a) * prev;
}

inline Eigen::Vector3d orientationErrorAxisAngle(const Eigen::Quaterniond& q_current,
                                                 const Eigen::Quaterniond& q_target)
{
  Eigen::Quaterniond q_curr = q_current.normalized();
  Eigen::Quaterniond q_tgt = q_target.normalized();

  if (q_curr.dot(q_tgt) < 0.0)
  {
    q_tgt.coeffs() = -q_tgt.coeffs();
  }

  const Eigen::Quaterniond q_error = q_tgt * q_curr.inverse();
  const Eigen::AngleAxisd aa(q_error);
  const double angle = aa.angle();
  if (!std::isfinite(angle) || std::abs(angle) < kMathEps)
  {
    return Eigen::Vector3d::Zero();
  }
  return angle * aa.axis();
}

// Compatibility alias used by existing code.
inline Eigen::Vector3d applyDeadbandAbs(const Eigen::Vector3d& v, double deadband_abs)
{
  return applyDeadbandAbs3(v, deadband_abs);
}

}  // namespace teleoperation
