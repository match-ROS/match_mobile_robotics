/**
 * @file robot_state_manager.cpp
 * @brief Implementation of RobotStateManager component (minimal).
 */

#include "teleoperation/components/robot_state_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace teleoperation
{

RobotStateManager::RobotStateManager(const std::string& group_name,
                                     const std::string& tcp_link,
                                     const std::string& robot_description_param)
  : group_name_(group_name)
  , tcp_link_(tcp_link)
{
  robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_description_param);
  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_)
  {
    throw std::runtime_error("Failed to load robot model from parameter '" + robot_description_param + "'.");
  }

  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
  if (!joint_model_group_)
  {
    throw std::runtime_error("Joint model group '" + group_name_ + "' not found in robot model.");
  }

  joint_names_ = joint_model_group_->getActiveJointModelNames();
  if (joint_names_.empty())
  {
    throw std::runtime_error("Joint model group '" + group_name_ + "' has no active joints.");
  }

  joint_index_map_.clear();
  joint_index_map_.reserve(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_index_map_[joint_names_[i]] = i;
  }

  robot_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  robot_state_->update();

  ROS_INFO_STREAM_NAMED("teleop_robot_state_manager",
                        "RobotStateManager initialized for group '" << group_name_
                        << "' with " << joint_names_.size() << " joints, TCP link: '" << tcp_link_ << "'");
}

void RobotStateManager::updateFromJointState(const sensor_msgs::JointState& msg)
{
  if (msg.name.size() != msg.position.size())
  {
    ROS_WARN_THROTTLE(5.0,
                      "JointState message has inconsistent sizes (name: %zu, position: %zu).",
                      msg.name.size(), msg.position.size());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!robot_state_) return;

  if (current_joint_velocities_.size() != static_cast<Eigen::Index>(joint_names_.size()))
  {
    current_joint_velocities_.resize(static_cast<Eigen::Index>(joint_names_.size()));
    current_joint_velocities_.setZero();
  }

  const bool has_velocities = (msg.velocity.size() == msg.name.size());

  bool updated_any = false;
  bool saw_nonfinite = false;

  for (std::size_t i = 0; i < msg.name.size(); ++i)
  {
    auto it = joint_index_map_.find(msg.name[i]);
    if (it == joint_index_map_.end()) continue;

    const double q = msg.position[i];
    if (!std::isfinite(q))
    {
      saw_nonfinite = true;
      continue;
    }

    robot_state_->setVariablePosition(msg.name[i], q);
    updated_any = true;

    if (has_velocities)
    {
      const double qdot = msg.velocity[i];
      if (std::isfinite(qdot))
      {
        current_joint_velocities_[static_cast<Eigen::Index>(it->second)] = qdot;
      }
      else
      {
        saw_nonfinite = true;
      }
    }
  }

  if (saw_nonfinite)
  {
    ROS_WARN_THROTTLE(2.0,
                      "Received non-finite joint state values for group '%s'. Ignoring those entries.",
                      group_name_.c_str());
  }

  if (updated_any)
  {
    velocities_available_ = has_velocities;
    robot_state_->update();
    robot_state_ready_ = true;
  }
}

bool RobotStateManager::isReady() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return robot_state_ready_;
}

bool RobotStateManager::computeTcpPose(const Eigen::Isometry3d& tcp_offset,
                                       Eigen::Isometry3d& tcp_pose) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!robot_state_ready_ || !robot_state_) return false;

  const moveit::core::LinkModel* tcp_link_model = robot_state_->getLinkModel(tcp_link_);
  if (!tcp_link_model)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "TCP link '" << tcp_link_ << "' not found in robot model.");
    return false;
  }

  const Eigen::Isometry3d T_world_tcp_link = robot_state_->getGlobalLinkTransform(tcp_link_model);
  tcp_pose = T_world_tcp_link * tcp_offset;

  if (!tcp_pose.matrix().allFinite())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Computed TCP pose is non-finite for group '" << group_name_
                                   << "' (TCP link: '" << tcp_link_ << "').");
    return false;
  }

  return true;
}

bool RobotStateManager::getJacobian(const std::string& link_name,
                                    const Eigen::Vector3d& reference_point,
                                    Eigen::MatrixXd& jacobian) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!robot_state_ready_ || !robot_state_) return false;
  if (!joint_model_group_) return false;

  const moveit::core::LinkModel* link_model = robot_state_->getLinkModel(link_name);
  if (!link_model)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Link '" << link_name << "' not found in robot model.");
    return false;
  }

  const bool success = robot_state_->getJacobian(joint_model_group_, link_model, reference_point, jacobian);
  if (!success)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Failed to compute Jacobian for link '" << link_name << "'");
    return false;
  }

  // Remap Jacobian columns to match joint_names_ ordering if needed
  const std::vector<const moveit::core::JointModel*>& joint_models =
      joint_model_group_->getActiveJointModels();

  bool needs_remapping = false;
  for (std::size_t i = 0; i < joint_models.size() && i < joint_names_.size(); ++i)
  {
    if (joint_models[i]->getName() != joint_names_[i])
    {
      needs_remapping = true;
      break;
    }
  }

  if (needs_remapping)
  {
    Eigen::MatrixXd remapped = Eigen::MatrixXd::Zero(6, static_cast<int>(joint_names_.size()));
    for (std::size_t col = 0; col < joint_models.size(); ++col)
    {
      auto it = joint_index_map_.find(joint_models[col]->getName());
      if (it != joint_index_map_.end())
      {
        remapped.col(static_cast<Eigen::Index>(it->second)) = jacobian.col(static_cast<Eigen::Index>(col));
      }
    }
    jacobian = remapped;
  }

  return true;
}

bool RobotStateManager::getCurrentJointPositions(Eigen::VectorXd& positions) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!robot_state_ready_ || !robot_state_) return false;

  positions.resize(static_cast<int>(joint_names_.size()));
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    positions[static_cast<Eigen::Index>(i)] = robot_state_->getVariablePosition(joint_names_[i]);
  }
  return true;
}

bool RobotStateManager::getCurrentJointVelocities(Eigen::VectorXd& velocities) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!robot_state_ready_ || !velocities_available_) return false;
  velocities = current_joint_velocities_;
  return true;
}

moveit::core::RobotState RobotStateManager::getRobotStateCopy() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (robot_state_)
  {
    return *robot_state_;
  }
  return moveit::core::RobotState(robot_model_);
}

}  // namespace teleoperation

