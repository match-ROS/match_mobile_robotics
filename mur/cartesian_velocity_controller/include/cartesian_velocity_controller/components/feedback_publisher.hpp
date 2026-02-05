#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

#include "cartesian_velocity_controller/EndEffectorState.h"
#include "cartesian_velocity_controller/PipelineDebug.h"
#include "cartesian_velocity_controller/POIDebugInfo.h"
#include "cartesian_velocity_controller/JointVelocityFeedback.h"
#include "cartesian_velocity_controller/types/control_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @brief Component responsible for publishing debug and feedback messages.
 * 
 * This component handles the publication of:
 * - End effector state (position, velocity, acceleration, jerk)
 * - Pipeline debug information (all stages of the control pipeline)
 * - Joint velocity feedback
 */
class FeedbackPublisher
{
public:
  /// Small value for numerical comparisons
  static constexpr double kEpsilon = 1e-6;

  /**
   * @brief Construct a FeedbackPublisher.
   * @param nh NodeHandle for creating publishers
   * @param global_frame Global reference frame name
   */
  FeedbackPublisher(ros::NodeHandle& nh, const std::string& global_frame);

  /**
   * @brief Set the global frame.
   * @param global_frame New global frame name
   */
  void setGlobalFrame(const std::string& global_frame) { global_frame_ = global_frame; }

  /**
   * @brief Get the global frame.
   * @return Current global frame name
   */
  const std::string& getGlobalFrame() const { return global_frame_; }

  /**
   * @brief Publish end effector state message.
   * 
   * Calculates velocities, accelerations, and jerks from the pose and
   * previous state data.
   * 
   * @param stamp Timestamp for the message
   * @param dt Time since last call (for derivative calculations)
   * @param tcp_pose Current TCP pose in world frame
   */
  void publishEndEffectorState(const ros::Time& stamp, double dt,
                               const Eigen::Isometry3d& tcp_pose);

  /**
   * @brief Publish pipeline debug message.
   * 
   * Contains comprehensive debug information from all pipeline stages:
   * - Level A: Global Planner (waypoint distances)
   * - Level B: Local Planner (V_desired composition, target_raw)
   * - Level C: Motion Generator (filtered velocities, accelerations, jerks)
   * - Level D: PID + IK (all PID components, Jacobian info)
   * - Safety Limiter (final joint commands, scaling factor)
   * 
   * @param stamp Timestamp for the message
   * @param data PipelineDebugData containing all debug information
   */
  void publishPipelineDebug(const ros::Time& stamp, const PipelineDebugData& data);

  /**
   * @brief Publish joint velocity feedback message.
   * 
   * @param stamp Timestamp for the message
   * @param joint_names Names of the joints
   * @param commanded_vel Commanded velocities (after safety limiter)
   * @param current_positions Current joint positions
   * @param actual_velocities Actual velocities from joint states (empty if unavailable)
   */
  void publishJointVelocityFeedback(const ros::Time& stamp,
                                    const std::vector<std::string>& joint_names,
                                    const std::vector<double>& commanded_vel,
                                    const std::vector<double>& current_positions,
                                    const std::vector<double>& actual_velocities);

  /**
   * @brief Reset internal state.
   * 
   * Clears derivative calculation history.
   */
  void reset();

  /**
   * @brief Check if end effector state publisher has subscribers.
   * @return true if there are subscribers
   */
  bool hasEndEffectorStateSubscribers() const 
  { return ee_state_pub_.getNumSubscribers() > 0; }

  /**
   * @brief Check if pipeline debug publisher has subscribers.
   * @return true if there are subscribers
   */
  bool hasPipelineDebugSubscribers() const 
  { return pipeline_debug_pub_.getNumSubscribers() > 0; }

  /**
   * @brief Check if joint feedback publisher has subscribers.
   * @return true if there are subscribers
   */
  bool hasJointFeedbackSubscribers() const 
  { return joint_feedback_pub_.getNumSubscribers() > 0; }

private:
  /// Helper to convert Eigen::Isometry3d to geometry_msgs::Pose
  static geometry_msgs::Pose isometryToPose(const Eigen::Isometry3d& iso);
  
  /// Helper to convert Eigen::Vector3d to geometry_msgs::Vector3
  static geometry_msgs::Vector3 eigenToVector3(const Eigen::Vector3d& v);

  ros::Publisher ee_state_pub_;
  ros::Publisher pipeline_debug_pub_;
  ros::Publisher joint_feedback_pub_;
  std::string global_frame_;

  // State for derivative calculations (end effector)
  Eigen::Vector3d prev_ee_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond prev_ee_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d prev_ee_linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_ee_angular_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_ee_linear_acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_ee_angular_acceleration_{Eigen::Vector3d::Zero()};
  bool ee_state_initialized_{false};
};

}  // namespace cartesian_velocity_controller
