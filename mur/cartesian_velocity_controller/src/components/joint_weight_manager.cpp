#include "cartesian_velocity_controller/components/joint_weight_manager.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace cartesian_velocity_controller
{

// ============================================================================
// ElbowSingularityStrategy
// ============================================================================

ElbowSingularityStrategy::ElbowSingularityStrategy()
{
}

void ElbowSingularityStrategy::setElbowIndex(int index)
{
  elbow_index_ = index;
}

void ElbowSingularityStrategy::setSingularityBuffer(double buffer)
{
  buffer_zone_ = std::max(0.001, buffer);
}

void ElbowSingularityStrategy::setMaxWeightPenalty(double max_weight)
{
  max_weight_ = std::max(1.0, max_weight);
}

void ElbowSingularityStrategy::computeWeights(const Eigen::VectorXd& joint_positions,
                                              Eigen::VectorXd& weights)
{
  if (elbow_index_ < 0 || elbow_index_ >= joint_positions.size() || 
      elbow_index_ >= weights.size())
  {
    return;
  }

  // Get current elbow position (assume 0 is fully extended)
  // For UR robots, 0 is typically "up" or "straight", check specific robot config.
  // The user says "braccio completamente esteso, q_elbow ~ 0".
  double q_elbow = joint_positions[elbow_index_];
  
  // We care about distance to 0
  double dist_to_singularity = std::abs(q_elbow);

  if (dist_to_singularity < buffer_zone_)
  {
    // Smooth weighting function:
    // w = 1 + K_max * (1 - dist/buffer)^2
    // When dist = buffer -> w = 1
    // When dist = 0      -> w = 1 + K_max
    
    double ratio = dist_to_singularity / buffer_zone_;
    double factor = std::max(0.0, 1.0 - ratio);
    double penalty = max_weight_ * factor * factor;
    
    weights[elbow_index_] *= (1.0 + penalty);
  }
}

// ============================================================================
// JointWeightManager
// ============================================================================

JointWeightManager::JointWeightManager(std::size_t num_joints)
  : num_joints_(num_joints)
{
  current_weights_ = Eigen::VectorXd::Ones(num_joints_);

  // Default strategies
  elbow_strategy_ = std::make_shared<ElbowSingularityStrategy>();
  strategies_.push_back(elbow_strategy_);
}

void JointWeightManager::update(const Eigen::VectorXd& joint_positions)
{
  if (joint_positions.size() != static_cast<long>(num_joints_))
  {
    // Mismatch in joint count
    return;
  }

  // Reset weights to 1.0
  current_weights_.setOnes();

  // Apply all strategies
  for (const auto& strategy : strategies_)
  {
    strategy->computeWeights(joint_positions, current_weights_);
  }
}

const Eigen::VectorXd& JointWeightManager::getWeights() const
{
  return current_weights_;
}

void JointWeightManager::setElbowSingularityConfig(int index, double buffer, double max_weight)
{
  if (elbow_strategy_)
  {
    elbow_strategy_->setElbowIndex(index);
    elbow_strategy_->setSingularityBuffer(buffer);
    elbow_strategy_->setMaxWeightPenalty(max_weight);
  }
}

}  // namespace cartesian_velocity_controller

