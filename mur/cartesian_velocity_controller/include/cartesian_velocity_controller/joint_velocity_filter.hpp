#pragma once

#include <Eigen/Core>

namespace cartesian_velocity_controller
{

/**
 * @brief JointVelocityFilter: jerk->acc->vel filter in joint space with time constant tau.
 *
 * This filter is intended to smooth joint-velocity commands before the final
 * JointSafetyLimiter. It supports:
 * - Time constant tau (controls how aggressively we chase desired qdot)
 * - Per-joint max velocity / acceleration / jerk limits
 * - Optional uniform scaling (preserve direction of qdot vector) vs per-joint clamping
 * - Optional fixed internal dt (dt_nominal) with accumulator/substeps for robustness to jitter
 */
class JointVelocityFilter
{
public:
  enum class LargeDtPolicy
  {
    HOLD_LAST,
    RESET_TO_ZERO,
    RESET_TO_DESIRED
  };

  explicit JointVelocityFilter(std::size_t num_joints);

  void reset();
  void resetToState(const Eigen::VectorXd& qdot,
                    const Eigen::VectorXd& qddot = Eigen::VectorXd(),
                    const Eigen::VectorXd& qjerk = Eigen::VectorXd());

  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isEnabled() const { return enabled_; }

  void setTimeConstant(double tau);
  double getTimeConstant() const { return tau_; }

  void setUniformScalingEnabled(bool enabled) { uniform_scaling_enabled_ = enabled; }
  bool isUniformScalingEnabled() const { return uniform_scaling_enabled_; }

  void setLimits(const Eigen::VectorXd& max_velocity,
                 const Eigen::VectorXd& max_acceleration,
                 const Eigen::VectorXd& max_jerk);

  Eigen::VectorXd getMaxVelocity() const { return max_velocity_; }
  Eigen::VectorXd getMaxAcceleration() const { return max_acceleration_; }
  Eigen::VectorXd getMaxJerk() const { return max_jerk_; }

  // dt policy
  void setDtNominal(double dt_nominal);
  double getDtNominal() const { return dt_nominal_; }

  void setDtClamp(double min_dt, double max_dt);
  void setMaxSubsteps(int max_substeps);
  void setResetDtThreshold(double reset_dt_threshold);
  void setLargeDtPolicy(LargeDtPolicy policy) { large_dt_policy_ = policy; }
  LargeDtPolicy getLargeDtPolicy() const { return large_dt_policy_; }

  /**
   * @brief Filter desired joint velocities.
   * @param desired_qdot Desired qdot (size = num_joints)
   * @param dt Wall-clock dt since last call (seconds)
   */
  Eigen::VectorXd filter(const Eigen::VectorXd& desired_qdot, double dt);

  // State accessors (useful for debug)
  Eigen::VectorXd getCurrentVelocity() const { return qdot_; }
  Eigen::VectorXd getCurrentAcceleration() const { return qddot_; }
  Eigen::VectorXd getCurrentJerk() const { return qjerk_; }

private:
  void step(const Eigen::VectorXd& desired_qdot, double dt);

  static double computeUniformScalingFactor(const Eigen::VectorXd& v, const Eigen::VectorXd& vmax);
  static void clampInPlace(Eigen::VectorXd& v, const Eigen::VectorXd& vmax);

private:
  std::size_t num_joints_{0};

  bool enabled_{false};                 ///< default false to avoid behavior changes
  bool uniform_scaling_enabled_{true};  ///< default true (old-style scaling), can be disabled

  // Time constant
  double tau_{0.1};

  // Limits (positive)
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_acceleration_;
  Eigen::VectorXd max_jerk_;

  // State
  Eigen::VectorXd qdot_;
  Eigen::VectorXd qddot_;
  Eigen::VectorXd qjerk_;

  // Fixed-dt integration (optional)
  double dt_nominal_{0.0};
  double min_dt_clamp_{1e-6};
  double max_dt_clamp_{0.1};
  int max_substeps_{10};
  double reset_dt_threshold_{0.25};
  LargeDtPolicy large_dt_policy_{LargeDtPolicy::HOLD_LAST};
  double accumulator_{0.0};
};

}  // namespace cartesian_velocity_controller

