#pragma once

#include <Eigen/Core>

namespace teleoperation
{

/**
 * @brief Jacobian solver configuration (SDLS/DLS)
 */
struct JacobianSolverConfig
{
  double singularity_threshold{0.05};
  double max_damping{0.2};
};

/**
 * @brief Output from the JointSafetyLimiter (uniform scaling)
 */
struct SafetyLimiterOutput
{
  Eigen::VectorXd joint_velocity;
  double scaling_factor{1.0};

  enum class LimitType
  {
    NONE,
    VELOCITY,
    ACCELERATION
  } limit_type{LimitType::NONE};

  int limiting_joint{-1};
};

}  // namespace teleoperation

