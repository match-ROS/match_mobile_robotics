#include "cartesian_velocity_controller/components/jacobian_solver.hpp"
#include <algorithm>
#include <cmath>

namespace cartesian_velocity_controller
{

JacobianSolver::JacobianSolver(const JacobianSolverConfig& config)
  : config_(config)
{
}

void JacobianSolver::setConfig(const JacobianSolverConfig& config)
{
  config_ = config;
}

double JacobianSolver::computeDampingSquare(const Eigen::MatrixXd& jacobian) const
{
  if (jacobian.size() == 0)
  {
    return 0.0;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues().size() == 0)
  {
    return 0.0;
  }

  double s_min = svd.singularValues().minCoeff();

  // Store for debugging
  last_min_singular_value_ = s_min;

  if (s_min >= config_.singularity_threshold)
  {
    last_damping_factor_ = 0.0;
    return 0.0;
  }

  double ratio = s_min / std::max(config_.singularity_threshold, kEpsilon);
  double damping_sq = (1.0 - ratio * ratio) * config_.max_damping * config_.max_damping;
  double result = std::max(damping_sq, 0.0);

  // Store damping factor for debugging
  last_damping_factor_ = std::sqrt(result);

  return result;
}

Eigen::MatrixXd JacobianSolver::computeDampedWeightedPseudoInverse(
    const Eigen::MatrixXd& jacobian,
    const Eigen::VectorXd& weights) const
{
  if (jacobian.rows() == 0 || jacobian.cols() == 0)
  {
    return Eigen::MatrixXd();
  }

  const int n_joints = jacobian.cols();

  // Build W^{-1/2} from weights (higher weight -> less movement)
  Eigen::VectorXd w_inv_sqrt(n_joints);
  for (int i = 0; i < n_joints; ++i)
  {
    double w = (i < weights.size()) ? weights[i] : 1.0;
    w_inv_sqrt[i] = 1.0 / std::sqrt(std::max(w, kEpsilon));  // Avoid division by zero
  }

  // Weight the Jacobian columns with W^{-1/2}
  Eigen::MatrixXd weighted_jacobian = jacobian;
  for (int i = 0; i < n_joints; ++i)
  {
    weighted_jacobian.col(i) *= w_inv_sqrt[i];
  }

  // SVD of the weighted Jacobian (thin for efficiency)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      weighted_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const Eigen::VectorXd& singular_values = svd.singularValues();
  if (singular_values.size() == 0)
  {
    last_min_singular_value_ = 0.0;
    last_damping_factor_ = 0.0;
    last_singular_values_.resize(0);
    last_damping_factors_.resize(0);
    return Eigen::MatrixXd();
  }

  last_min_singular_value_ = singular_values.minCoeff();
  last_singular_values_ = singular_values;

  const double threshold = std::max(config_.singularity_threshold, kEpsilon);
  const double max_damping = std::max(config_.max_damping, 0.0);

  const int r = singular_values.size();  // rank for thin SVD
  Eigen::MatrixXd damped_sigma = Eigen::MatrixXd::Zero(r, r);
  last_damping_factors_.resize(r);

  double max_lambda_used = 0.0;
  for (int i = 0; i < r; ++i)
  {
    const double sigma = singular_values[i];

    double lambda_i = 0.0;
    if (sigma < threshold)
    {
      const double ratio = sigma / threshold;
      const double lambda_sq = (1.0 - ratio * ratio) * max_damping * max_damping;
      lambda_i = std::sqrt(std::max(lambda_sq, 0.0));
    }

    max_lambda_used = std::max(max_lambda_used, lambda_i);
    last_damping_factors_[i] = lambda_i;

    const double denom = sigma * sigma + lambda_i * lambda_i;
    const double damp_coeff = (denom > kEpsilon) ? (sigma / denom) : 0.0;
    damped_sigma(i, i) = damp_coeff;
  }

  // Store damping factor for diagnostics (max λ applied)
  last_damping_factor_ = max_lambda_used;

  // Pseudo-inverse of weighted Jacobian with selective damping
  Eigen::MatrixXd weighted_pinv = svd.matrixV() * damped_sigma * svd.matrixU().transpose();

  // Unweight: J⁺ = W^{-1/2} * (J W^{-1/2})⁺
  Eigen::MatrixXd Winv_sqrt = w_inv_sqrt.asDiagonal();
  return Winv_sqrt * weighted_pinv;
}

Eigen::MatrixXd JacobianSolver::buildMaskedJacobian(
    const Eigen::MatrixXd& full_jacobian,
    const std::array<bool, 6>& task_space_mask) const
{
  // full_jacobian is 6 x n_joints
  // Returns a matrix with only the rows where task_space_mask[i] == true

  int active_count = 0;
  for (bool active : task_space_mask)
  {
    if (active)
    {
      ++active_count;
    }
  }

  if (active_count == 0)
  {
    // No DOFs in primary task, return empty matrix
    return Eigen::MatrixXd();
  }

  if (active_count == 6)
  {
    // All DOFs active, return full Jacobian
    return full_jacobian;
  }

  const int n_joints = full_jacobian.cols();
  Eigen::MatrixXd masked_jacobian(active_count, n_joints);

  int row_idx = 0;
  for (int i = 0; i < 6; ++i)
  {
    if (task_space_mask[i] && i < full_jacobian.rows())
    {
      masked_jacobian.row(row_idx) = full_jacobian.row(i);
      ++row_idx;
    }
  }

  return masked_jacobian;
}

Eigen::MatrixXd JacobianSolver::computeNullSpaceProjector(
    const Eigen::MatrixXd& jacobian,
    const Eigen::VectorXd& weights) const
{
  // Null space projector: N = I - J⁺ * J
  // where J⁺ is the weighted damped pseudo-inverse

  const int n_joints = jacobian.cols();

  if (jacobian.rows() == 0 || n_joints == 0)
  {
    // Empty Jacobian means full null space (identity projector)
    return Eigen::MatrixXd::Identity(n_joints, n_joints);
  }

  // Compute the weighted damped pseudo-inverse
  Eigen::MatrixXd J_pinv = computeDampedWeightedPseudoInverse(jacobian, weights);

  if (J_pinv.size() == 0)
  {
    // Failed to compute pseudo-inverse, return identity (full null space)
    return Eigen::MatrixXd::Identity(n_joints, n_joints);
  }

  // N = I - J⁺ * J
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n_joints, n_joints);
  Eigen::MatrixXd projector = identity - J_pinv * jacobian;

  return projector;
}

}  // namespace cartesian_velocity_controller

