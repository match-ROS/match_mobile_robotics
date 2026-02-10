#pragma once

#include <Eigen/Core>

#include <cmath>
#include <mutex>

namespace teleoperation
{

struct PIDConfig
{
  double kp{1.0};
  double ki{0.0};
  double kd{0.0};
  double kff{0.0};
  double output_limit{1.0};          // Saturation on output vector norm.
  double derivative_filter_tau{0.1}; // Low-pass filter time constant for derivative [s].
  bool enabled{true};
};

class PIDController
{
public:
  static constexpr double kEpsilon = 1e-10;

  explicit PIDController(int dimensions, const PIDConfig& config = {});

  void setConfig(const PIDConfig& config);
  PIDConfig getConfig() const;

  Eigen::VectorXd compute(const Eigen::VectorXd& error,
                          const Eigen::VectorXd& feedforward,
                          double dt);
  Eigen::VectorXd compute(const Eigen::VectorXd& error, double dt);

  void reset();
  void resetIntegral();

  Eigen::VectorXd getLastError() const;
  Eigen::VectorXd getIntegral() const;
  Eigen::VectorXd getLastPTerm() const;
  Eigen::VectorXd getLastITerm() const;
  Eigen::VectorXd getLastDTerm() const;

  void setIntegral(const Eigen::VectorXd& integral);

  void setEnabled(bool enabled);
  bool isEnabled() const;

  int dimensions() const
  {
    return dimensions_;
  }

private:
  void applyAntiWindup(Eigen::VectorXd& integral_output,
                       double p_magnitude,
                       double output_limit);
  void saturate(Eigen::VectorXd& output, double limit) const;

  int dimensions_;
  PIDConfig config_;
  mutable std::mutex config_mutex_;

  Eigen::VectorXd integral_;
  Eigen::VectorXd prev_error_;
  Eigen::VectorXd derivative_filtered_;
  Eigen::VectorXd last_error_;
  Eigen::VectorXd last_p_term_;
  Eigen::VectorXd last_i_term_;
  Eigen::VectorXd last_d_term_;
  bool has_prev_error_{false};
  mutable std::mutex state_mutex_;
};

}  // namespace teleoperation
