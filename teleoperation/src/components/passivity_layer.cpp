#include "teleoperation/components/passivity_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace teleoperation
{

namespace
{

constexpr double kEps = 1e-12;

double sanitizeFiniteNonNegative(double value, double fallback)
{
  if (!std::isfinite(value) || value < 0.0)
  {
    return fallback;
  }
  return value;
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double dampingPower(const Eigen::Vector3d& velocity, const Eigen::Vector3d& damping)
{
  if (!velocity.allFinite() || !damping.allFinite())
  {
    return 0.0;
  }

  const Eigen::Vector3d damped = damping.cwiseMax(Eigen::Vector3d::Zero());
  return std::max(0.0, velocity.cwiseProduct(damped).dot(velocity));
}

}  // namespace

PassivityLayer::PassivityLayer(const PassivityLayerConfig& config)
{
  setConfig(config);
}

void PassivityLayer::setConfig(const PassivityLayerConfig& config)
{
  config_ = sanitizeConfig(config);
  reset();
}

const PassivityLayerConfig& PassivityLayer::config() const
{
  return config_;
}

void PassivityLayer::reset()
{
  energy_ = std::clamp(config_.tank_energy_init, config_.tank_energy_min, config_.tank_energy_max);
  gamma_applied_ = 1.0;
}

PassivityLayerResult PassivityLayer::step(const Eigen::Vector3d& force_candidate,
                                          const Eigen::Vector3d& torque_candidate,
                                          const Eigen::Vector3d& linear_velocity,
                                          const Eigen::Vector3d& angular_velocity,
                                          const Eigen::Vector3d& damping_linear,
                                          const Eigen::Vector3d& damping_angular,
                                          double dt)
{
  PassivityLayerResult result;
  result.energy_before = energy_;

  const bool valid_dt = std::isfinite(dt) && (dt > 0.0);
  const Eigen::Vector3d safe_force = force_candidate.allFinite() ? force_candidate : Eigen::Vector3d::Zero();
  const Eigen::Vector3d safe_torque = torque_candidate.allFinite() ? torque_candidate : Eigen::Vector3d::Zero();
  const Eigen::Vector3d safe_v_lin = linear_velocity.allFinite() ? linear_velocity : Eigen::Vector3d::Zero();
  const Eigen::Vector3d safe_v_ang = angular_velocity.allFinite() ? angular_velocity : Eigen::Vector3d::Zero();

  if (!config_.enabled || !valid_dt)
  {
    result.gamma_applied = config_.enabled ? gamma_applied_ : 1.0;
    result.gamma_raw = result.gamma_applied;
    result.force_used = result.gamma_applied * safe_force;
    result.torque_used = config_.linear_only ? safe_torque : (result.gamma_applied * safe_torque);
    result.power_out_requested = safe_force.dot(safe_v_lin) +
                                 (config_.linear_only ? 0.0 : safe_torque.dot(safe_v_ang));
    result.power_out_applied = result.force_used.dot(safe_v_lin) +
                               (config_.linear_only ? 0.0 : result.torque_used.dot(safe_v_ang));
    result.power_diss = dampingPower(safe_v_lin, damping_linear) +
                        (config_.linear_only ? 0.0 : dampingPower(safe_v_ang, damping_angular));
    result.energy_after = energy_;
    return result;
  }

  result.power_out_requested = safe_force.dot(safe_v_lin) +
                               (config_.linear_only ? 0.0 : safe_torque.dot(safe_v_ang));
  result.power_diss = dampingPower(safe_v_lin, damping_linear) +
                      (config_.linear_only ? 0.0 : dampingPower(safe_v_ang, damping_angular));

  double gamma_target = 1.0;
  if (result.power_out_requested > config_.power_deadband)
  {
    const double available_energy = std::max(0.0, energy_ - config_.tank_energy_min);
    const double denom = dt * std::max(kEps, config_.discharge_gain) * result.power_out_requested;
    gamma_target = clamp01(available_energy / std::max(kEps, denom));
  }
  result.gamma_raw = gamma_target;

  double gamma_next = gamma_target;
  if (gamma_target > gamma_applied_)
  {
    gamma_next = gamma_applied_;
    if (config_.gamma_lowpass_alpha > 0.0)
    {
      gamma_next += config_.gamma_lowpass_alpha * (gamma_target - gamma_applied_);
    }
    else
    {
      gamma_next = gamma_target;
    }

    if (config_.gamma_rate_limit > 0.0)
    {
      gamma_next = std::min(gamma_next, gamma_applied_ + config_.gamma_rate_limit * dt);
    }
  }

  gamma_applied_ = std::clamp(gamma_next, config_.gamma_min, 1.0);
  result.gamma_applied = gamma_applied_;
  result.force_used = gamma_applied_ * safe_force;
  result.torque_used = config_.linear_only ? safe_torque : (gamma_applied_ * safe_torque);
  result.power_out_applied = result.force_used.dot(safe_v_lin) +
                             (config_.linear_only ? 0.0 : result.torque_used.dot(safe_v_ang));

  const double discharged_power = std::max(0.0, result.power_out_applied);
  energy_ = std::clamp(energy_ + dt * (config_.recharge_gain * result.power_diss -
                                       config_.discharge_gain * discharged_power),
                       config_.tank_energy_min,
                       config_.tank_energy_max);
  result.energy_after = energy_;
  return result;
}

PassivityLayerConfig PassivityLayer::sanitizeConfig(PassivityLayerConfig config)
{
  config.tank_energy_min = sanitizeFiniteNonNegative(config.tank_energy_min, 0.0);
  config.tank_energy_max = sanitizeFiniteNonNegative(config.tank_energy_max, config.tank_energy_min);
  if (config.tank_energy_max < config.tank_energy_min)
  {
    config.tank_energy_max = config.tank_energy_min;
  }

  config.tank_energy_init = sanitizeFiniteNonNegative(config.tank_energy_init, config.tank_energy_min);
  config.tank_energy_init = std::clamp(config.tank_energy_init, config.tank_energy_min, config.tank_energy_max);
  config.recharge_gain = sanitizeFiniteNonNegative(config.recharge_gain, 1.0);
  config.discharge_gain = sanitizeFiniteNonNegative(config.discharge_gain, 1.0);
  config.power_deadband = sanitizeFiniteNonNegative(config.power_deadband, 0.0);
  config.gamma_min = clamp01(config.gamma_min);
  config.gamma_lowpass_alpha = clamp01(config.gamma_lowpass_alpha);
  config.gamma_rate_limit = sanitizeFiniteNonNegative(config.gamma_rate_limit, 0.0);
  return config;
}

}  // namespace teleoperation
