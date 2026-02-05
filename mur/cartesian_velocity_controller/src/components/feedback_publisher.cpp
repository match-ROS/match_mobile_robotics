/**
 * @file feedback_publisher.cpp
 * @brief Feedback publisher component for pipeline debug and monitoring.
 *
 * Publishes comprehensive debug information from all pipeline stages.
 */

#include "cartesian_velocity_controller/components/feedback_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

namespace cartesian_velocity_controller
{

FeedbackPublisher::FeedbackPublisher(ros::NodeHandle& nh, const std::string& global_frame)
  : global_frame_(global_frame)
{
  ee_state_pub_ = nh.advertise<EndEffectorState>("end_effector_state", 10);
  pipeline_debug_pub_ = nh.advertise<PipelineDebug>("pipeline_debug", 10);
  joint_feedback_pub_ = nh.advertise<JointVelocityFeedback>("joint_velocity_feedback", 10);
}

void FeedbackPublisher::reset()
{
  prev_ee_position_ = Eigen::Vector3d::Zero();
  prev_ee_orientation_ = Eigen::Quaterniond::Identity();
  prev_ee_linear_velocity_ = Eigen::Vector3d::Zero();
  prev_ee_angular_velocity_ = Eigen::Vector3d::Zero();
  prev_ee_linear_acceleration_ = Eigen::Vector3d::Zero();
  prev_ee_angular_acceleration_ = Eigen::Vector3d::Zero();
  ee_state_initialized_ = false;
}

geometry_msgs::Pose FeedbackPublisher::isometryToPose(const Eigen::Isometry3d& iso)
{
  geometry_msgs::Pose pose;
  pose.position.x = iso.translation().x();
  pose.position.y = iso.translation().y();
  pose.position.z = iso.translation().z();
  
  Eigen::Quaterniond quat(iso.rotation());
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  
  return pose;
}

geometry_msgs::Vector3 FeedbackPublisher::eigenToVector3(const Eigen::Vector3d& v)
{
  geometry_msgs::Vector3 vec;
  vec.x = v.x();
  vec.y = v.y();
  vec.z = v.z();
  return vec;
}

void FeedbackPublisher::publishEndEffectorState(const ros::Time& stamp, double dt,
                                                 const Eigen::Isometry3d& tcp_pose)
{
  if (ee_state_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  EndEffectorState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = global_frame_;

  // Position
  msg.position.x = tcp_pose.translation().x();
  msg.position.y = tcp_pose.translation().y();
  msg.position.z = tcp_pose.translation().z();

  // Orientation as quaternion
  Eigen::Quaterniond quat(tcp_pose.rotation());
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();

  // Orientation as RPY
  tf2::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
  tf2::Matrix3x3 mat(tf_quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  msg.orientation_rpy.x = roll;
  msg.orientation_rpy.y = pitch;
  msg.orientation_rpy.z = yaw;

  // Calculate derivatives if we have previous data
  if (ee_state_initialized_ && dt > kEpsilon)
  {
    // Linear velocity
    Eigen::Vector3d current_pos = tcp_pose.translation();
    Eigen::Vector3d linear_vel = (current_pos - prev_ee_position_) / dt;
    msg.linear_velocity.x = linear_vel.x();
    msg.linear_velocity.y = linear_vel.y();
    msg.linear_velocity.z = linear_vel.z();

    // Angular velocity from quaternion derivative
    Eigen::Quaterniond current_quat(tcp_pose.rotation());
    Eigen::Quaterniond q_diff = current_quat * prev_ee_orientation_.inverse();
    if (q_diff.w() < 0.0)
    {
      q_diff.coeffs() = -q_diff.coeffs();
    }
    double angle = 2.0 * std::acos(std::clamp(q_diff.w(), -1.0, 1.0));
    Eigen::Vector3d axis(q_diff.x(), q_diff.y(), q_diff.z());
    double sin_half = axis.norm();
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
    if (sin_half > kEpsilon)
    {
      axis /= sin_half;
      angular_vel = (angle / dt) * axis;
    }
    msg.angular_velocity.x = angular_vel.x();
    msg.angular_velocity.y = angular_vel.y();
    msg.angular_velocity.z = angular_vel.z();

    // Linear acceleration
    Eigen::Vector3d linear_accel = (linear_vel - prev_ee_linear_velocity_) / dt;
    msg.linear_acceleration.x = linear_accel.x();
    msg.linear_acceleration.y = linear_accel.y();
    msg.linear_acceleration.z = linear_accel.z();

    // Angular acceleration
    Eigen::Vector3d angular_accel = (angular_vel - prev_ee_angular_velocity_) / dt;
    msg.angular_acceleration.x = angular_accel.x();
    msg.angular_acceleration.y = angular_accel.y();
    msg.angular_acceleration.z = angular_accel.z();

    // Linear jerk
    Eigen::Vector3d linear_jerk = (linear_accel - prev_ee_linear_acceleration_) / dt;
    msg.linear_jerk.x = linear_jerk.x();
    msg.linear_jerk.y = linear_jerk.y();
    msg.linear_jerk.z = linear_jerk.z();

    // Angular jerk
    Eigen::Vector3d angular_jerk = (angular_accel - prev_ee_angular_acceleration_) / dt;
    msg.angular_jerk.x = angular_jerk.x();
    msg.angular_jerk.y = angular_jerk.y();
    msg.angular_jerk.z = angular_jerk.z();

    // Update previous values
    prev_ee_linear_velocity_ = linear_vel;
    prev_ee_angular_velocity_ = angular_vel;
    prev_ee_linear_acceleration_ = linear_accel;
    prev_ee_angular_acceleration_ = angular_accel;
  }
  else
  {
    ee_state_initialized_ = true;
  }

  // Update position/orientation for next iteration
  prev_ee_position_ = tcp_pose.translation();
  prev_ee_orientation_ = Eigen::Quaterniond(tcp_pose.rotation());

  ee_state_pub_.publish(msg);
}

void FeedbackPublisher::publishPipelineDebug(const ros::Time& stamp, const PipelineDebugData& data)
{
  if (pipeline_debug_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  PipelineDebug msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = global_frame_;

  // =========================================================================
  // LEVEL A: GLOBAL PLANNER
  // =========================================================================
  msg.distance_waypoint_to_current = data.distance_waypoint_to_current;
  msg.active_waypoint_index = data.active_waypoint_index;
  msg.total_waypoints = data.total_waypoints;
  msg.active_waypoint = isometryToPose(data.active_waypoint);
  msg.current_pose = isometryToPose(data.current_pose);

  // =========================================================================
  // LEVEL B: LOCAL PLANNER
  // =========================================================================
  msg.distance_target_raw_to_waypoint = data.distance_target_raw_to_waypoint;
  msg.distance_target_raw_to_current = data.distance_target_raw_to_current;
  msg.target_raw = isometryToPose(data.target_raw);
  
  // V_desired composition
  msg.v_goal_linear = eigenToVector3(data.v_goal_linear);
  msg.v_goal_angular = eigenToVector3(data.v_goal_angular);
  msg.v_obs_linear = eigenToVector3(data.v_obs_linear);
  msg.v_link_linear = eigenToVector3(data.v_link_linear);
  msg.v_desired_linear = eigenToVector3(data.v_desired_linear);
  msg.v_desired_angular = eigenToVector3(data.v_desired_angular);
  
  // Gains
  msg.k_attractive = data.k_attractive;
  msg.k_repulsive_tcp = data.k_repulsive_tcp;
  msg.k_repulsive_links = data.k_repulsive_links;

  // Virtual Target Leash
  msg.virtual_target_scaling_factor = data.virtual_target_scaling_factor;
  
  // Obstacle info (summary)
  msg.closest_obstacle_distance = data.closest_obstacle_distance;
  msg.closest_obstacle_id = data.closest_obstacle_id;
  msg.closest_link_name = data.closest_link_name;

  // =========================================================================
  // POI REPULSION DEBUG
  // =========================================================================
  
  // Count active POIs and convert to POIDebugInfo messages
  msg.active_poi_count = 0;
  for (const auto& poi : data.poi_debug_data)
  {
    POIDebugInfo poi_info;
    
    // POI identification
    poi_info.point_name = poi.point_name;
    poi_info.link_name = poi.link_name;
    
    // POI position and radius
    poi_info.position.x = poi.position_world.x();
    poi_info.position.y = poi.position_world.y();
    poi_info.position.z = poi.position_world.z();
    poi_info.poi_radius = poi.poi_radius;
    
    // Obstacle information
    poi_info.closest_obstacle_id = poi.closest_obstacle_id;
    poi_info.obstacle_radius = poi.object_characteristic_radius;
    
    // Distance information
    poi_info.distance_vector.x = poi.distance_vector.x();
    poi_info.distance_vector.y = poi.distance_vector.y();
    poi_info.distance_vector.z = poi.distance_vector.z();
    poi_info.distance_raw = poi.distance_raw;
    poi_info.distance_effective = poi.distance_to_closest_obstacle;
    
    // Repulsive velocity (BEFORE Jacobian projection)
    poi_info.repulsive_velocity.x = poi.repulsive_velocity.x();
    poi_info.repulsive_velocity.y = poi.repulsive_velocity.y();
    poi_info.repulsive_velocity.z = poi.repulsive_velocity.z();
    poi_info.repulsive_velocity_magnitude = poi.repulsive_velocity_magnitude;
    
    // POI configuration
    poi_info.weight = poi.weight;
    poi_info.is_active = (poi.repulsive_velocity_magnitude > 1e-6);
    
    msg.poi_debug_info.push_back(poi_info);
    
    if (poi_info.is_active)
    {
      msg.active_poi_count++;
    }
  }
  
  // Repulsive joint velocity from all link POIs
  msg.repulsive_joint_velocity.resize(data.repulsive_joint_velocity.size());
  for (int i = 0; i < data.repulsive_joint_velocity.size(); ++i)
  {
    msg.repulsive_joint_velocity[i] = data.repulsive_joint_velocity[i];
  }

  // =========================================================================
  // LEVEL C: MOTION GENERATOR
  // =========================================================================
  msg.distance_target_filtered_to_current = data.distance_target_filtered_to_current;
  msg.target_filtered = isometryToPose(data.target_filtered);
  
  // Filtered outputs
  msg.v_filtered_linear = eigenToVector3(data.v_filtered_linear);
  msg.v_filtered_angular = eigenToVector3(data.v_filtered_angular);
  msg.acceleration_filtered_linear = eigenToVector3(data.acceleration_filtered_linear);
  msg.acceleration_filtered_angular = eigenToVector3(data.acceleration_filtered_angular);
  msg.jerk_filtered_linear = eigenToVector3(data.jerk_filtered_linear);
  msg.jerk_filtered_angular = eigenToVector3(data.jerk_filtered_angular);
  msg.filter_tau = data.filter_tau;

  // =========================================================================
  // LEVEL D: PID CONTROLLER
  // =========================================================================
  msg.pid_position_error = eigenToVector3(data.pid_position_error);
  msg.pid_orientation_error = eigenToVector3(data.pid_orientation_error);
  msg.pid_position_error_norm = data.pid_position_error_norm;
  msg.pid_orientation_error_norm = data.pid_orientation_error_norm;
  
  // PID gains
  msg.pid_kp_position = data.pid_kp_position;
  msg.pid_ki_position = data.pid_ki_position;
  msg.pid_kd_position = data.pid_kd_position;
  msg.pid_kp_orientation = data.pid_kp_orientation;
  msg.pid_ki_orientation = data.pid_ki_orientation;
  msg.pid_kd_orientation = data.pid_kd_orientation;
  
  // PID components - Linear
  msg.pid_p_term_linear = eigenToVector3(data.pid_p_term_linear);
  msg.pid_i_term_linear = eigenToVector3(data.pid_i_term_linear);
  msg.pid_d_term_linear = eigenToVector3(data.pid_d_term_linear);
  msg.pid_integral_linear = eigenToVector3(data.pid_integral_linear);
  
  // PID components - Angular
  msg.pid_p_term_angular = eigenToVector3(data.pid_p_term_angular);
  msg.pid_i_term_angular = eigenToVector3(data.pid_i_term_angular);
  msg.pid_d_term_angular = eigenToVector3(data.pid_d_term_angular);
  msg.pid_integral_angular = eigenToVector3(data.pid_integral_angular);
  
  // Feed-forward and outputs
  msg.pid_feedforward_linear = eigenToVector3(data.pid_feedforward_linear);
  msg.pid_feedforward_angular = eigenToVector3(data.pid_feedforward_angular);
  msg.pid_output_linear = eigenToVector3(data.pid_output_linear);
  msg.pid_output_angular = eigenToVector3(data.pid_output_angular);
  msg.cartesian_cmd_linear = eigenToVector3(data.cartesian_cmd_linear);
  msg.cartesian_cmd_angular = eigenToVector3(data.cartesian_cmd_angular);

  // =========================================================================
  // LEVEL D: JACOBIAN INVERSE KINEMATICS
  // =========================================================================
  msg.jacobian_damping_factor = data.jacobian_damping_factor;
  msg.jacobian_min_singular_value = data.jacobian_min_singular_value;

  // Singular values (weighted Jacobian) and per-direction damping (λᵢ)
  msg.jacobian_singular_values.resize(data.jacobian_singular_values.size());
  for (int i = 0; i < data.jacobian_singular_values.size(); ++i)
  {
    msg.jacobian_singular_values[i] = data.jacobian_singular_values[i];
  }

  msg.jacobian_damping_factors.resize(data.jacobian_damping_factors.size());
  for (int i = 0; i < data.jacobian_damping_factors.size(); ++i)
  {
    msg.jacobian_damping_factors[i] = data.jacobian_damping_factors[i];
  }
  
  // Joint weights
  msg.joint_weights.resize(data.joint_weights.size());
  for (int i = 0; i < data.joint_weights.size(); ++i)
  {
    msg.joint_weights[i] = data.joint_weights[i];
  }

  // Joint velocity from IK
  msg.joint_velocity_from_ik.resize(data.joint_velocity_from_ik.size());
  for (int i = 0; i < data.joint_velocity_from_ik.size(); ++i)
  {
    msg.joint_velocity_from_ik[i] = data.joint_velocity_from_ik[i];
  }

  // =========================================================================
  // SAFETY LIMITER
  // =========================================================================
  msg.joint_velocity_after_limiter.resize(data.joint_velocity_after_limiter.size());
  for (int i = 0; i < data.joint_velocity_after_limiter.size(); ++i)
  {
    msg.joint_velocity_after_limiter[i] = data.joint_velocity_after_limiter[i];
  }

  // Joint acceleration command
  msg.joint_acceleration_command.resize(data.joint_acceleration_command.size());
  for (int i = 0; i < data.joint_acceleration_command.size(); ++i)
  {
    msg.joint_acceleration_command[i] = data.joint_acceleration_command[i];
  }

  msg.safety_scaling_factor = data.safety_scaling_factor;
  msg.safety_limiting_reason = data.safety_limiting_reason;
  msg.joint_names = data.joint_names;

  pipeline_debug_pub_.publish(msg);
}

void FeedbackPublisher::publishJointVelocityFeedback(const ros::Time& stamp,
                                                      const std::vector<std::string>& joint_names,
                                                      const std::vector<double>& commanded_vel,
                                                      const std::vector<double>& current_positions,
                                                      const std::vector<double>& actual_velocities)
{
  if (joint_feedback_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  JointVelocityFeedback msg;
  msg.header.stamp = stamp;

  // Joint names
  msg.joint_names = joint_names;

  // Commanded velocity
  msg.commanded_velocity = commanded_vel;

  // Current positions
  msg.current_position = current_positions;

  // Actual velocity from joint states
  msg.actual_velocity = actual_velocities;

  joint_feedback_pub_.publish(msg);
}

}  // namespace cartesian_velocity_controller
