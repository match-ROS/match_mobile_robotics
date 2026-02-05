#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <string>

namespace cartesian_velocity_controller
{

/**
 * @brief Base interface for joint weighting strategies.
 */
class WeightStrategy
{
public:
  virtual ~WeightStrategy() = default;

  /**
   * @brief Compute weights based on current joint positions.
   * @param joint_positions Current joint positions [rad]
   * @param weights Reference to the weights vector to update (multiplicative)
   */
  virtual void computeWeights(const Eigen::VectorXd& joint_positions, 
                              Eigen::VectorXd& weights) = 0;
};

/**
 * @brief Strategy to penalize the elbow joint when approaching singularity (extension).
 * 
 * Singularity happens when elbow joint is near 0.
 * We apply a weight that increases as the joint approaches 0.
 */
class ElbowSingularityStrategy : public WeightStrategy
{
public:
  ElbowSingularityStrategy();

  void computeWeights(const Eigen::VectorXd& joint_positions, 
                      Eigen::VectorXd& weights) override;

  void setElbowIndex(int index);
  void setSingularityBuffer(double buffer);
  void setMaxWeightPenalty(double max_weight);

  int getElbowIndex() const { return elbow_index_; }
  double getSingularityBuffer() const { return buffer_zone_; }
  double getMaxWeightPenalty() const { return max_weight_; }

private:
  int elbow_index_{2};           ///< Index of the elbow joint (UR10e standard is 2)
  double buffer_zone_{0.2};      ///< Rads from singularity to start weighting
  double max_weight_{50.0};      ///< Max weight to apply at singularity
};

/**
 * @brief Manager class that aggregates multiple weighting strategies.
 */
class JointWeightManager
{
public:
  JointWeightManager(std::size_t num_joints);

  /**
   * @brief Update weights based on current joint positions.
   * @param joint_positions Current joint positions
   */
  void update(const Eigen::VectorXd& joint_positions);

  /**
   * @brief Get the currently computed weights.
   */
  const Eigen::VectorXd& getWeights() const;

  // Configuration accessors
  void setElbowSingularityConfig(int index, double buffer, double max_weight);
  
  // Getters for individual strategies (downcasting needed if we want full generality later, 
  // but for now we expose specific helpers)
  std::shared_ptr<ElbowSingularityStrategy> getElbowStrategy() const { return elbow_strategy_; }

private:
  std::size_t num_joints_;
  Eigen::VectorXd current_weights_;
  
  std::vector<std::shared_ptr<WeightStrategy>> strategies_;
  std::shared_ptr<ElbowSingularityStrategy> elbow_strategy_;
};

}  // namespace cartesian_velocity_controller

