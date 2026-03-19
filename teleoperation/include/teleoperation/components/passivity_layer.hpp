#pragma once

#include <Eigen/Core>

namespace teleoperation
{

struct PassivityLayerConfig
{
  bool enabled{false};
  bool linear_only{false};
  double tank_energy_init{1.0};
  double tank_energy_min{0.0};
  double tank_energy_max{10.0};
  double recharge_gain{1.0};
  double discharge_gain{1.0};
  double power_deadband{0.0};
  double gamma_lowpass_alpha{0.0};
  double gamma_rate_limit{0.0};  // 1/s, applied only while gamma recovers upward.
};

struct PassivityLayerResult
{
  double energy_before{0.0};
  double energy_after{0.0};
  double gamma_raw{1.0};
  double gamma_applied{1.0};
  double power_out_requested{0.0};
  double power_out_applied{0.0};
  double power_diss{0.0};
  Eigen::Vector3d force_used{Eigen::Vector3d::Zero()};
  Eigen::Vector3d torque_used{Eigen::Vector3d::Zero()};
};

class PassivityLayer
{
public:
  explicit PassivityLayer(const PassivityLayerConfig& config = {});

  void setConfig(const PassivityLayerConfig& config);
  const PassivityLayerConfig& config() const;

  void reset();

  PassivityLayerResult step(const Eigen::Vector3d& force_candidate,
                            const Eigen::Vector3d& torque_candidate,
                            const Eigen::Vector3d& linear_velocity,
                            const Eigen::Vector3d& angular_velocity,
                            const Eigen::Vector3d& damping_linear,
                            const Eigen::Vector3d& damping_angular,
                            double dt);

  double energy() const
  {
    return energy_;
  }

private:
  static PassivityLayerConfig sanitizeConfig(PassivityLayerConfig config);

  PassivityLayerConfig config_;
  double energy_{0.0};
  double gamma_applied_{1.0};
};

}  // namespace teleoperation
