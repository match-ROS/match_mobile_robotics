#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include "cartesian_velocity_controller/types/config_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @brief Computes Jacobian pseudo-inverse and null space projections
 * 
 * This class handles the mathematical operations for Jacobian-based control:
 * - Damped weighted pseudo-inverse computation
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
   * @brief Compute damped weighted pseudo-inverse of Jacobian
   * 
   * Implements Selectively Damped Least Squares (SDLS) with joint
   * weighting. The Jacobian is first weighted with W^{-1/2}, then an SVD
   * is computed. Each singular direction is damped independently:
   *   σᵢ / (σᵢ² + λᵢ²)
   * where λᵢ is computed dynamically from the proximity to the configured
   * singularity threshold. Directions far from singularity are left
   * undamped (λᵢ = 0), while near-singular directions receive higher
   * damping up to max_damping.
   * 
   * @param jacobian The Jacobian matrix (m x n, task space x joints)
   * @param weights Joint weights (higher weight = less movement for that joint)
   * @return Pseudo-inverse matrix (n x m)
   */
  Eigen::MatrixXd computeDampedWeightedPseudoInverse(
      const Eigen::MatrixXd& jacobian,
      const Eigen::VectorXd& weights) const;

  /**
   * @brief Compute null space projector
   * 
   * Computes N = I - J⁺ * J, which projects velocities into the null space
   * of the primary task.
   * 
   * @param jacobian The Jacobian matrix
   * @param weights Joint weights for pseudo-inverse computation
   * @return Null space projector matrix (n x n)
   */
  Eigen::MatrixXd computeNullSpaceProjector(
      const Eigen::MatrixXd& jacobian,
      const Eigen::VectorXd& weights) const;

  /**
   * @brief Build a masked Jacobian with only selected DOFs
   * 
   * @param full_jacobian Full 6xN Jacobian
   * @param task_space_mask Boolean mask for each of the 6 DOFs (x,y,z,rx,ry,rz)
   * @return Masked Jacobian with only active DOF rows
   */
  Eigen::MatrixXd buildMaskedJacobian(
      const Eigen::MatrixXd& full_jacobian,
      const std::array<bool, 6>& task_space_mask) const;

  /// Get the minimum singular value from the last pseudo-inverse computation
  double getLastMinSingularValue() const { return last_min_singular_value_; }

  /// Get the maximum damping factor (max λᵢ) from the last computation
  double getLastDampingFactor() const { return last_damping_factor_; }

  /// Get all singular values from the last computation
  Eigen::VectorXd getLastSingularValues() const { return last_singular_values_; }

  /// Get all per-direction damping factors (λᵢ) from the last computation
  Eigen::VectorXd getLastDampingFactors() const { return last_damping_factors_; }

private:
  /**
   * @brief Compute damping factor squared based on singularity proximity
   * 
   * Uses smooth damping that increases as the smallest singular value
   * approaches the singularity threshold.
   * 
   * @param jacobian The Jacobian matrix
   * @return λ² (damping factor squared)
   */
  double computeDampingSquare(const Eigen::MatrixXd& jacobian) const;

  JacobianSolverConfig config_;
  
  // Debug/diagnostic values from last computation (mutable for const methods)
  mutable double last_min_singular_value_{0.0};
  mutable double last_damping_factor_{0.0};
  mutable Eigen::VectorXd last_singular_values_;
  mutable Eigen::VectorXd last_damping_factors_;
};

}  // namespace cartesian_velocity_controller

