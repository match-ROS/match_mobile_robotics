#pragma once

/**
 * @file robot_state_manager.hpp
 * @brief Robot state management component for the cartesian velocity controller.
 *
 * This component encapsulates robot model loading, state tracking, forward kinematics,
 * inverse kinematics, and Jacobian computation. It provides thread-safe access to
 * robot state data for other controller components.
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

#include "cartesian_velocity_controller/types/controller_joint_limits.hpp"

namespace cartesian_velocity_controller
{

/**
 * @class RobotStateManager
 * @brief Manages robot model, state, and kinematic computations.
 *
 * This class is responsible for:
 * - Loading and maintaining the robot model from URDF/SRDF
 * - Tracking joint states from sensor messages
 * - Computing forward kinematics (TCP pose)
 * - Computing inverse kinematics (reachability check)
 * - Computing Jacobian matrices
 *
 * All state access is thread-safe via internal mutex protection.
 */
class RobotStateManager
{
public:
  /**
   * @brief Construct a new RobotStateManager.
   * @param group_name Name of the joint model group (e.g., "manipulator")
   * @param tcp_link Name of the TCP link (e.g., "tool0")
   * @param robot_description_param Name of the URDF parameter (default: "robot_description")
   * @throws std::runtime_error if robot model cannot be loaded
   */
  RobotStateManager(const std::string& group_name,
                    const std::string& tcp_link,
                    const std::string& robot_description_param = "robot_description");

  /**
   * @brief Update robot state from JointState message.
   *
   * This method should be called from the joint state callback.
   * It updates internal state in a thread-safe manner.
   *
   * @param msg JointState message containing current joint positions
   */
  void updateFromJointState(const sensor_msgs::JointState& msg);

  /**
   * @brief Check if robot state has been initialized.
   * @return true if at least one joint state message has been received
   */
  bool isReady() const;

  // ============== Getters for Model Information ==============

  /**
   * @brief Get the joint names for the configured group.
   * @return Vector of joint names
   */
  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  /**
   * @brief Get the number of joints in the group.
   * @return Number of active joints
   */
  std::size_t getJointCount() const { return joint_names_.size(); }

  /**
   * @brief Get the robot model.
   * @return Shared pointer to the robot model
   */
  moveit::core::RobotModelConstPtr getRobotModel() const { return robot_model_; }

  /**
   * @brief Get the joint model group.
   * @return Pointer to the joint model group
   */
  const moveit::core::JointModelGroup* getJointModelGroup() const { return joint_model_group_; }

  /**
   * @brief Get the model root frame (URDF root link).
   * 
   * This is the frame in which getGlobalLinkTransform and getJacobian operate.
   * It may differ from the controller's configured global_frame.
   * 
   * @return Name of the URDF root link
   */
  std::string getModelRootFrame() const 
  { 
    return robot_model_ ? robot_model_->getRootLinkName() : ""; 
  }

  /**
   * @brief Get the TCP link name.
   * @return TCP link name string
   */
  const std::string& getTcpLink() const { return tcp_link_; }

  /**
   * @brief Get the group name.
   * @return Group name string
   */
  const std::string& getGroupName() const { return group_name_; }

  // ============== Kinematic Computations ==============

  /**
   * @brief Compute TCP pose in world frame using forward kinematics.
   *
   * @param tcp_offset Offset from TCP link to actual TCP point
   * @param tcp_pose Output: computed TCP pose in world frame
   * @return true if computation was successful
   */
  bool computeTcpPose(const Eigen::Isometry3d& tcp_offset,
                      Eigen::Isometry3d& tcp_pose) const;

  /**
   * @brief Check if a pose is reachable using inverse kinematics.
   *
   * @param pose Target pose in world frame
   * @param tcp_offset TCP offset to account for
   * @param joint_solution Output: joint values if pose is reachable
   * @return true if pose is reachable
   */
  bool checkPoseReachability(const Eigen::Isometry3d& pose,
                             const Eigen::Isometry3d& tcp_offset,
                             Eigen::VectorXd& joint_solution,
                             const ControllerJointLimitsConfig* controller_joint_limits = nullptr) const;

  /**
   * @brief Compute Jacobian matrix for a link.
   *
   * @param link_name Name of the link to compute Jacobian for
   * @param reference_point Reference point in link frame
   * @param jacobian Output: Jacobian matrix (6 x num_joints)
   * @return true if computation was successful
   */
  bool getJacobian(const std::string& link_name,
                   const Eigen::Vector3d& reference_point,
                   Eigen::MatrixXd& jacobian) const;

  /**
   * @brief Get current joint positions.
   *
   * @param positions Output: current joint positions vector
   * @return true if state is ready and positions were retrieved
   */
  bool getCurrentJointPositions(Eigen::VectorXd& positions) const;

  /**
   * @brief Get current joint velocities from joint states.
   *
   * @param velocities Output: current joint velocities vector
   * @return true if velocities are available and were retrieved
   */
  bool getCurrentJointVelocities(Eigen::VectorXd& velocities) const;

  /**
   * @brief Get a thread-safe copy of the current robot state.
   *
   * Use this for operations that need a consistent state snapshot
   * without holding the lock for extended periods.
   *
   * @return Copy of the current robot state
   */
  moveit::core::RobotState getRobotStateCopy() const;

  /**
   * @brief Get joint index map for column remapping.
   *
   * This maps joint names to their indices in the joint vector.
   *
   * @return Reference to joint index map
   */
  const std::unordered_map<std::string, std::size_t>& getJointIndexMap() const
  {
    return joint_index_map_;
  }

private:
  // Robot model and state
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};

  // Joint information
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_map_;

  // Configuration
  std::string group_name_;
  std::string tcp_link_;

  // Thread safety
  mutable std::mutex state_mutex_;
  bool robot_state_ready_{false};

  // Joint velocities from joint states
  Eigen::VectorXd current_joint_velocities_;
  bool velocities_available_{false};
};

}  // namespace cartesian_velocity_controller

