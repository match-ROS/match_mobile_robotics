/**
 * @file robot_state_manager.cpp
 * @brief Implementation of RobotStateManager component.
 */

#include "cartesian_velocity_controller/components/robot_state_manager.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cartesian_velocity_controller
{

RobotStateManager::RobotStateManager(const std::string& group_name,
                                     const std::string& tcp_link,
                                     const std::string& robot_description_param)
  : group_name_(group_name), tcp_link_(tcp_link)
{
  // Load robot model from parameter server
  robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_description_param);
  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_)
  {
    throw std::runtime_error("Failed to load robot model from parameter 'robot_description'.");
  }

  // Get joint model group
  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
  if (!joint_model_group_)
  {
    throw std::runtime_error("Joint model group '" + group_name_ + "' not found in robot model.");
  }

  // Get active joint names
  joint_names_ = joint_model_group_->getActiveJointModelNames();
  if (joint_names_.empty())
  {
    throw std::runtime_error("Joint model group '" + group_name_ + "' has no active joints.");
  }

  // Build joint index map for fast lookup
  joint_index_map_.clear();
  joint_index_map_.reserve(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_index_map_[joint_names_[i]] = i;
  }

  // Initialize robot state
  robot_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  robot_state_->update();

  ROS_INFO_STREAM_NAMED("robot_state_manager",
                        "RobotStateManager initialized for group '" << group_name_
                        << "' with " << joint_names_.size() << " joints, TCP link: '" << tcp_link_ << "'");
}

void RobotStateManager::updateFromJointState(const sensor_msgs::JointState& msg)
{
  if (msg.name.size() != msg.position.size())
  {
    ROS_WARN_THROTTLE(5.0, "JointState message has inconsistent sizes (name: %zu, position: %zu).",
                      msg.name.size(), msg.position.size());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!robot_state_)
  {
    return;
  }

  // Initialize velocity storage if needed
  if (current_joint_velocities_.size() != static_cast<Eigen::Index>(joint_names_.size()))
  {
    current_joint_velocities_.resize(joint_names_.size());
    current_joint_velocities_.setZero();
  }

  // Check if velocities are available in the message
  bool has_velocities = (msg.velocity.size() == msg.name.size());

  bool updated_any = false;
  bool saw_nonfinite = false;

  // Update joint positions and velocities from message
  for (std::size_t i = 0; i < msg.name.size(); ++i)
  {
    auto it = joint_index_map_.find(msg.name[i]);
    if (it != joint_index_map_.end())
    {
      const double q = msg.position[i];
      if (!std::isfinite(q))
      {
        saw_nonfinite = true;
        continue;
      }

      robot_state_->setVariablePosition(msg.name[i], q);
      updated_any = true;
      
      // Store velocity if available
      if (has_velocities)
      {
        const double qdot = msg.velocity[i];
        if (std::isfinite(qdot))
        {
          current_joint_velocities_[it->second] = qdot;
        }
        else
        {
          saw_nonfinite = true;
        }
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

  if (!robot_state_ready_ || !robot_state_)
  {
    return false;
  }

  const moveit::core::LinkModel* tcp_link_model = robot_state_->getLinkModel(tcp_link_);
  if (!tcp_link_model)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "TCP link '" << tcp_link_ << "' not found in robot model.");
    return false;
  }

  // Get the transform of the TCP link in the world frame
  Eigen::Isometry3d T_world_tcp_link = robot_state_->getGlobalLinkTransform(tcp_link_model);

  // Apply TCP offset: T_world_tcp = T_world_tcp_link * tcp_offset
  tcp_pose = T_world_tcp_link * tcp_offset;

  // Defensive: if joint states contain NaN/Inf, MoveIt transforms can become non-finite.
  if (!tcp_pose.matrix().allFinite())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Computed TCP pose is non-finite for group '" << group_name_
                                   << "' (TCP link: '" << tcp_link_ << "').");
    return false;
  }

  return true;
}

bool RobotStateManager::checkPoseReachability(const Eigen::Isometry3d& pose,
                                              const Eigen::Isometry3d& tcp_offset,
                                              Eigen::VectorXd& joint_solution,
                                              const ControllerJointLimitsConfig* controller_joint_limits) const
{
  if (!robot_model_ || !joint_model_group_)
  {
    return false;
  }

  // Create a temporary robot state for IK computation
  moveit::core::RobotState ik_state(robot_model_);

  // Copy current state as seed for IK
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (robot_state_ready_ && robot_state_)
    {
      ik_state = *robot_state_;
    }
    else
    {
      ik_state.setToDefaultValues();
    }
  }

  // Get the TCP link for IK
  const moveit::core::LinkModel* tcp_link_model = ik_state.getLinkModel(tcp_link_);
  if (!tcp_link_model)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "TCP link '" << tcp_link_ << "' not found for IK.");
    return false;
  }

  // Compute the link pose from the TCP pose (remove TCP offset)
  Eigen::Isometry3d link_pose = pose * tcp_offset.inverse();

  // Try to solve IK (optionally with controller-only joint limits via validity callback)
  const double ik_timeout = 0.1;  // seconds

  moveit::core::GroupStateValidityCallbackFn validity_cb;
  if (controller_joint_limits && controller_joint_limits->enabled && controller_joint_limits->hasAnyEnabledLimit())
  {
    validity_cb = [&](moveit::core::RobotState* state,
                      const moveit::core::JointModelGroup* /*group*/,
                      const double* /*joint_group_variable_values*/) -> bool {
      // Note: state is allowed to be modified by contract, but we only read from it.
      // Apply margin-based validation: q in [min + margin, max - margin]
      for (const auto& kv : controller_joint_limits->limits)
      {
        const std::string& joint_name = kv.first;
        const ControllerJointLimit& lim = kv.second;
        if (!lim.enabled) continue;

        const double q = state->getVariablePosition(joint_name);
        if (!std::isfinite(q)) return false;

        const double margin = std::max(0.0, lim.runtime_guard.margin);
        const double q_min = lim.min + margin;
        const double q_max = lim.max - margin;
        if (q < q_min || q > q_max) return false;
      }
      return true;
    };
  }

  const bool found_ik = ik_state.setFromIK(joint_model_group_, link_pose, tcp_link_, ik_timeout, validity_cb);

  if (found_ik)
  {
    // Extract joint values
    joint_solution.resize(joint_names_.size());
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_solution[i] = ik_state.getVariablePosition(joint_names_[i]);
    }
    return true;
  }

  return false;
}

bool RobotStateManager::getJacobian(const std::string& link_name,
                                    const Eigen::Vector3d& reference_point,
                                    Eigen::MatrixXd& jacobian) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!robot_state_ready_ || !robot_state_)
  {
    return false;
  }

  if (!joint_model_group_)
  {
    return false;
  }

  // MoveIt's getJacobian returns true on success
  bool success = robot_state_->getJacobian(
      joint_model_group_,
      robot_state_->getLinkModel(link_name),
      reference_point,
      jacobian);

  if (!success)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Failed to compute Jacobian for link '" << link_name << "'");
    return false;
  }

  // Remap Jacobian columns to match our joint ordering
  // MoveIt's Jacobian columns are ordered by the joint model group's variable indices
  // We need to ensure consistency with our joint_names_ ordering

  // Get the active joint models from the group
  const std::vector<const moveit::core::JointModel*>& joint_models =
      joint_model_group_->getActiveJointModels();

  // Check if remapping is needed (it usually isn't if joint_names_ matches group order)
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
    Eigen::MatrixXd remapped_jacobian = Eigen::MatrixXd::Zero(6, joint_names_.size());
    for (std::size_t col = 0; col < joint_models.size(); ++col)
    {
      auto it = joint_index_map_.find(joint_models[col]->getName());
      if (it != joint_index_map_.end())
      {
        remapped_jacobian.col(it->second) = jacobian.col(col);
      }
    }
    jacobian = remapped_jacobian;
  }

  return true;
}

bool RobotStateManager::getCurrentJointPositions(Eigen::VectorXd& positions) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!robot_state_ready_ || !robot_state_)
  {
    return false;
  }

  positions.resize(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    positions[i] = robot_state_->getVariablePosition(joint_names_[i]);
  }

  return true;
}

bool RobotStateManager::getCurrentJointVelocities(Eigen::VectorXd& velocities) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!robot_state_ready_ || !velocities_available_)
  {
    return false;
  }

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

  // Return default state if not initialized
  return moveit::core::RobotState(robot_model_);
}

}  // namespace cartesian_velocity_controller

