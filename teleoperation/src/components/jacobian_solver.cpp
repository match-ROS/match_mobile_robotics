/**
 * @file jacobian_solver.cpp
 * @brief Implementation of JacobianSolver (damped pseudo-inverse).
 */

#include "teleoperation/components/jacobian_solver.hpp"

#include <algorithm>
#include <cmath>

namespace teleoperation
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
  if (jacobian.size() == 0) return 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues().size() == 0) return 0.0;

  const double s_min = svd.singularValues().minCoeff();
  last_min_singular_value_ = s_min;

  if (s_min >= config_.singularity_threshold)
  {
    last_damping_factor_ = 0.0;
    return 0.0;
  }

  const double ratio = s_min / std::max(config_.singularity_threshold, kEpsilon);
  const double damping_sq = (1.0 - ratio * ratio) * config_.max_damping * config_.max_damping;
  const double result = std::max(damping_sq, 0.0);
  last_damping_factor_ = std::sqrt(result);
  return result;
}

Eigen::MatrixXd JacobianSolver::computeDampedPseudoInverse(const Eigen::MatrixXd& jacobian) const
{
  if (jacobian.rows() == 0 || jacobian.cols() == 0)
  {
    return Eigen::MatrixXd();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

  const int r = singular_values.size();
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

  last_damping_factor_ = max_lambda_used;

  // Pseudo-inverse of Jacobian with selective damping
  return svd.matrixV() * damped_sigma * svd.matrixU().transpose();
}

Eigen::MatrixXd JacobianSolver::buildMaskedJacobian(
    const Eigen::MatrixXd& full_jacobian,
    const std::array<bool, 6>& task_space_mask) const
{
  int active_count = 0;
  for (bool active : task_space_mask)
  {
    if (active) ++active_count;
  }

  if (active_count == 0)
  {
    return Eigen::MatrixXd();
  }
  if (active_count == 6)
  {
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
    const Eigen::MatrixXd& jacobian) const
{
  const int n_joints = jacobian.cols();
  if (jacobian.rows() == 0 || n_joints == 0)
  {
    return Eigen::MatrixXd::Identity(n_joints, n_joints);
  }

  const Eigen::MatrixXd J_pinv = computeDampedPseudoInverse(jacobian);
  if (J_pinv.size() == 0)
  {
    return Eigen::MatrixXd::Identity(n_joints, n_joints);
  }

  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n_joints, n_joints);
  return identity - J_pinv * jacobian;
}

}  // namespace teleoperation

