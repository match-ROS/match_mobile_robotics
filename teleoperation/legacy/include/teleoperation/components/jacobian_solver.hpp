#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>

#include "teleoperation/types.hpp"

namespace teleoperation
{

/**
 * @brief Computes Jacobian pseudo-inverse and null space projections.
 *
 * - Damped pseudo-inverse computation (SDLS/DLS-like)
 * - Null space projector computation
 * - Task space masking for partial DOF control
 */
class JacobianSolver
{
public:
  /// Small constant to avoid division by zero
  static constexpr double kEpsilon = 1e-10;

  explicit JacobianSolver(const JacobianSolverConfig& config = {});

  /// Update configuration
  void setConfig(const JacobianSolverConfig& config);

  /// Get current configuration
  const JacobianSolverConfig& getConfig() const { return config_; }

  /**
   * @brief Compute damped pseudo-inverse of Jacobian.
   *
   * @param jacobian The Jacobian matrix (m x n, task space x joints)
   * @return Pseudo-inverse matrix (n x m)
   */
  Eigen::MatrixXd computeDampedPseudoInverse(const Eigen::MatrixXd& jacobian) const;

  /**
   * @brief Compute null space projector.
   *
   * Computes N = I - J⁺ * J, which projects velocities into the null space
   * of the primary task.
   */
  Eigen::MatrixXd computeNullSpaceProjector(const Eigen::MatrixXd& jacobian) const;

  /**
   * @brief Build a masked Jacobian with only selected DOFs.
   *
   * @param full_jacobian Full 6xN Jacobian
   * @param task_space_mask Boolean mask for each of the 6 DOFs (x,y,z,rx,ry,rz)
   * @return Masked Jacobian with only active DOF rows
   */
  Eigen::MatrixXd buildMaskedJacobian(
      const Eigen::MatrixXd& full_jacobian,
      const std::array<bool, 6>& task_space_mask) const;

  double getLastMinSingularValue() const { return last_min_singular_value_; }
  double getLastDampingFactor() const { return last_damping_factor_; }
  Eigen::VectorXd getLastSingularValues() const { return last_singular_values_; }
  Eigen::VectorXd getLastDampingFactors() const { return last_damping_factors_; }

private:
  double computeDampingSquare(const Eigen::MatrixXd& jacobian) const;

  JacobianSolverConfig config_;

  // Debug/diagnostic values from last computation (mutable for const methods)
  mutable double last_min_singular_value_{0.0};
  mutable double last_damping_factor_{0.0};
  mutable Eigen::VectorXd last_singular_values_;
  mutable Eigen::VectorXd last_damping_factors_;
};

}  // namespace teleoperation

