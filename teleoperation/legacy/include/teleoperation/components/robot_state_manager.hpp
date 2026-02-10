#pragma once

/**
 * @file robot_state_manager.hpp
 * @brief Minimal robot model/state wrapper for FK and Jacobian.
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace teleoperation
{

class RobotStateManager
{
public:
  RobotStateManager(const std::string& group_name,
                    const std::string& tcp_link,
                    const std::string& robot_description_param = "robot_description");

  void updateFromJointState(const sensor_msgs::JointState& msg);
  bool isReady() const;

  const std::vector<std::string>& getJointNames() const { return joint_names_; }
  std::size_t getJointCount() const { return joint_names_.size(); }
  moveit::core::RobotModelConstPtr getRobotModel() const { return robot_model_; }
  const moveit::core::JointModelGroup* getJointModelGroup() const { return joint_model_group_; }
  std::string getModelRootFrame() const
  {
    return robot_model_ ? robot_model_->getRootLinkName() : "";
  }

  const std::string& getTcpLink() const { return tcp_link_; }
  const std::string& getGroupName() const { return group_name_; }

  bool computeTcpPose(const Eigen::Isometry3d& tcp_offset,
                      Eigen::Isometry3d& tcp_pose) const;

  bool getJacobian(const std::string& link_name,
                   const Eigen::Vector3d& reference_point,
                   Eigen::MatrixXd& jacobian) const;

  bool getCurrentJointPositions(Eigen::VectorXd& positions) const;
  bool getCurrentJointVelocities(Eigen::VectorXd& velocities) const;

  moveit::core::RobotState getRobotStateCopy() const;

  const std::unordered_map<std::string, std::size_t>& getJointIndexMap() const
  {
    return joint_index_map_;
  }

private:
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_map_;

  std::string group_name_;
  std::string tcp_link_;

  mutable std::mutex state_mutex_;
  bool robot_state_ready_{false};

  Eigen::VectorXd current_joint_velocities_;
  bool velocities_available_{false};
};

}  // namespace teleoperation

