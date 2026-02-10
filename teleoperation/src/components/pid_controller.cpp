#include "teleoperation/components/pid_controller.hpp"

#include <algorithm>

namespace teleoperation
{

PIDController::PIDController(int dimensions, const PIDConfig& config)
  : dimensions_(dimensions)
  , config_(config)
  , integral_(Eigen::VectorXd::Zero(dimensions))
  , prev_error_(Eigen::VectorXd::Zero(dimensions))
  , derivative_filtered_(Eigen::VectorXd::Zero(dimensions))
  , last_error_(Eigen::VectorXd::Zero(dimensions))
  , last_p_term_(Eigen::VectorXd::Zero(dimensions))
  , last_i_term_(Eigen::VectorXd::Zero(dimensions))
  , last_d_term_(Eigen::VectorXd::Zero(dimensions))
{
}

void PIDController::setConfig(const PIDConfig& config)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
}

PIDConfig PIDController::getConfig() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_;
}

Eigen::VectorXd PIDController::compute(const Eigen::VectorXd& error,
                                       const Eigen::VectorXd& feedforward,
                                       double dt)
{
  PIDConfig config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config = config_;
  }

  Eigen::VectorXd output = Eigen::VectorXd::Zero(dimensions_);
  if (!config.enabled || dt <= kEpsilon)
  {
    return output;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  last_error_ = error;

  Eigen::VectorXd p_term = config.kp * error;
  const double p_magnitude = p_term.norm();

  Eigen::VectorXd i_term = Eigen::VectorXd::Zero(dimensions_);
  if (config.ki > kEpsilon)
  {
    integral_ += error * dt;
    i_term = config.ki * integral_;

    applyAntiWindup(i_term, p_magnitude, config.output_limit);
    integral_ = i_term / config.ki;
  }

  Eigen::VectorXd d_term = Eigen::VectorXd::Zero(dimensions_);
  if (config.kd > kEpsilon && has_prev_error_)
  {
    const Eigen::VectorXd derivative_raw = (error - prev_error_) / dt;
    const double alpha = dt / (config.derivative_filter_tau + dt);
    derivative_filtered_ = alpha * derivative_raw + (1.0 - alpha) * derivative_filtered_;
    d_term = config.kd * derivative_filtered_;
  }

  prev_error_ = error;
  has_prev_error_ = true;

  last_p_term_ = p_term;
  last_i_term_ = i_term;
  last_d_term_ = d_term;

  Eigen::VectorXd ff_term = Eigen::VectorXd::Zero(dimensions_);
  if (config.kff > kEpsilon && feedforward.size() == dimensions_)
  {
    ff_term = config.kff * feedforward;
  }

  output = p_term + i_term + d_term + ff_term;
  saturate(output, config.output_limit);
  return output;
}

Eigen::VectorXd PIDController::compute(const Eigen::VectorXd& error, double dt)
{
  return compute(error, Eigen::VectorXd::Zero(dimensions_), dt);
}

void PIDController::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  integral_.setZero();
  prev_error_.setZero();
  derivative_filtered_.setZero();
  last_error_.setZero();
  last_p_term_.setZero();
  last_i_term_.setZero();
  last_d_term_.setZero();
  has_prev_error_ = false;
}

void PIDController::resetIntegral()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  integral_.setZero();
}

Eigen::VectorXd PIDController::getLastError() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_error_;
}

Eigen::VectorXd PIDController::getIntegral() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return integral_;
}

Eigen::VectorXd PIDController::getLastPTerm() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_p_term_;
}

Eigen::VectorXd PIDController::getLastITerm() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_i_term_;
}

Eigen::VectorXd PIDController::getLastDTerm() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_d_term_;
}

void PIDController::setIntegral(const Eigen::VectorXd& integral)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (integral.size() == dimensions_)
  {
    integral_ = integral;
  }
}

void PIDController::setEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_.enabled = enabled;
}

bool PIDController::isEnabled() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_.enabled;
}

void PIDController::applyAntiWindup(Eigen::VectorXd& integral_output,
                                    double p_magnitude,
                                    double output_limit)
{
  const double integral_headroom = std::max(0.0, output_limit - p_magnitude);
  const double i_magnitude = integral_output.norm();
  if (i_magnitude > integral_headroom && i_magnitude > kEpsilon)
  {
    integral_output *= (integral_headroom / i_magnitude);
  }
}

void PIDController::saturate(Eigen::VectorXd& output, double limit) const
{
  const double magnitude = output.norm();
  if (magnitude > limit && magnitude > kEpsilon)
  {
    output *= (limit / magnitude);
  }
}

}  // namespace teleoperation
