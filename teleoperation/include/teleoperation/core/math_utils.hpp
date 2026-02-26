#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace teleoperation
{

constexpr double kMathEps = 1e-10;
constexpr double kDefaultEps = 1e-9;

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

inline Eigen::Vector3d clampAbs3(const Eigen::Vector3d& v, const Eigen::Vector3d& max_abs)
{
  Eigen::Vector3d out = v;
  for (int i = 0; i < 3; ++i)
  {
    const double m = std::max(0.0, max_abs[i]);
    out[i] = std::clamp(out[i], -m, m);
  }
  return out;
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

inline Eigen::Vector3d softDeadzoneNorm3(const Eigen::Vector3d& v, double deadzone_norm)
{
  if (deadzone_norm <= 0.0)
  {
    return v;
  }

  const double n = v.norm();
  if (!(n > kMathEps) || !std::isfinite(n))
  {
    return Eigen::Vector3d::Zero();
  }

  if (n <= deadzone_norm)
  {
    return Eigen::Vector3d::Zero();
  }

  const double scale = (n - deadzone_norm) / n;
  return scale * v;
}

inline Eigen::Vector3d ema3(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return a * curr + (1.0 - a) * prev;
}

inline double lowpassAlphaFromCutoffHz(double dt, double cutoff_hz)
{
  if (!(dt > 0.0) || !std::isfinite(dt))
  {
    return 0.0;
  }
  if (!(cutoff_hz > 0.0) || !std::isfinite(cutoff_hz))
  {
    return 0.0;
  }

  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  return clamp01(dt / (tau + dt));
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

inline Eigen::Vector3d softDeadzoneNormWithHysteresis(const Eigen::Vector3d& v,
                                                       double db_enter,
                                                       double db_exit,
                                                       bool& active)
{
  const double n = v.norm();
  if (!std::isfinite(n))
  {
    active = false;
    return Eigen::Vector3d::Zero();
  }

  const double enter = std::max(0.0, db_enter);
  const double exit = std::max(0.0, db_exit);

  if (!active)
  {
    if (n <= enter)
    {
      return Eigen::Vector3d::Zero();
    }
    active = true;
    return softDeadzoneNorm3(v, enter);
  }

  if (n <= exit)
  {
    active = false;
    return Eigen::Vector3d::Zero();
  }
  return softDeadzoneNorm3(v, enter);
}

// Compatibility alias used by existing code.
inline Eigen::Vector3d applyDeadbandAbs(const Eigen::Vector3d& v, double deadband_abs)
{
  return applyDeadbandAbs3(v, deadband_abs);
}

}  // namespace teleoperation
