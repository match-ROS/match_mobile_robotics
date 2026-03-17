#pragma once

#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>

#include "teleoperation/core/math_utils.hpp"
#include "teleoperation/core/types.hpp"

namespace teleoperation
{

inline Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

inline Wrench3 filterClampDeadbandWrench(const Wrench3& prev,
                                         const Wrench3& curr,
                                         bool use_filter,
                                         double alpha,
                                         double force_db,
                                         double torque_db,
                                         double max_f,
                                         double max_tau,
                                         bool use_torques)
{
  Wrench3 out;

  const Eigen::Vector3d f0 = use_filter ? ema3(prev.f, curr.f, alpha) : curr.f;
  const Eigen::Vector3d t0 = use_filter ? ema3(prev.tau, curr.tau, alpha) : curr.tau;

  out.f = applyDeadbandAbs3(f0, force_db);
  out.f = clampNorm3(out.f, max_f);

  if (use_torques)
  {
    out.tau = applyDeadbandAbs3(t0, torque_db);
    out.tau = clampNorm3(out.tau, max_tau);
  }
  else
  {
    out.tau.setZero();
  }

  return out;
}

struct WrenchDeadbandState
{
  bool f_active{false};
  bool tau_active{false};
};

inline Wrench3 filterClampDeadbandWrenchNorm(const Wrench3& prev,
                                              const Wrench3& curr,
                                              bool use_filter,
                                              double alpha,
                                              double force_db_enter,
                                              double force_db_exit,
                                              double torque_db_enter,
                                              double torque_db_exit,
                                              double max_f,
                                              double max_tau,
                                              bool use_torques,
                                              double cross_deadband_scale,
                                              WrenchDeadbandState& db_state)
{
  Wrench3 out;

  const Eigen::Vector3d f0 = use_filter ? ema3(prev.f, curr.f, alpha) : curr.f;
  const Eigen::Vector3d t0 = use_filter ? ema3(prev.tau, curr.tau, alpha) : curr.tau;

  const double scale = std::max(0.0, std::min(1.0, cross_deadband_scale));
  const double force_enter = std::max(0.0, force_db_enter);
  const double force_exit = std::max(0.0, force_db_exit);
  const double torque_enter = std::max(0.0, torque_db_enter);
  const double torque_exit = std::max(0.0, torque_db_exit);

  // When one channel is already active (or clearly exceeds its own entry threshold),
  // reduce the deadband of the other channel. scale=1 keeps the old independent
  // behavior, scale=0 makes it a full bypass.
  const bool relax_force_from_tau = use_torques && (db_state.tau_active || (t0.norm() > torque_enter));
  const bool relax_tau_from_force = db_state.f_active || (f0.norm() > force_enter);

  const double force_enter_eff = relax_force_from_tau ? (force_enter * scale) : force_enter;
  const double force_exit_eff = relax_force_from_tau ? (force_exit * scale) : force_exit;
  const double torque_enter_eff = relax_tau_from_force ? (torque_enter * scale) : torque_enter;
  const double torque_exit_eff = relax_tau_from_force ? (torque_exit * scale) : torque_exit;

  out.f = softDeadzoneNormWithHysteresis(f0, force_enter_eff, force_exit_eff, db_state.f_active);
  out.f = clampNorm3(out.f, max_f);

  if (use_torques)
  {
    out.tau = softDeadzoneNormWithHysteresis(t0, torque_enter_eff, torque_exit_eff, db_state.tau_active);
    out.tau = clampNorm3(out.tau, max_tau);
  }
  else
  {
    out.tau.setZero();
    db_state.tau_active = false;
  }

  return out;
}

}  // namespace teleoperation
