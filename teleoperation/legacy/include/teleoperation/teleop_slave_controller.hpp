#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <mutex>
#include <memory>
#include <string>

#include "teleoperation/components/robot_state_manager.hpp"
#include "teleoperation/components/jacobian_solver.hpp"
#include "teleoperation/components/pid_controller.hpp"
#include "teleoperation/components/joint_safety_limiter.hpp"

namespace teleoperation
{

class TeleopSlaveController
{
public:
  TeleopSlaveController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void start();
  void stop();

private:
  void loadParameters();
  void setupRosInterfaces();

  void targetPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);
  void feedforwardTwistCb(const geometry_msgs::TwistStampedConstPtr& msg);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg);

  void controlLoopCb(const ros::TimerEvent& ev);

  void publishZeroVelocity(const std::string& reason);

  // Optional Jacobian frame conversion (rotation only), to match the frame used for v_cmd.
  bool ensureJacobianFrameTransformReady(const std::string& target_frame);
  bool applyJacobianFrameTransformIfConfigured(Eigen::MatrixXd& jacobian, const std::string& target_frame);

  bool tryGetInputs(geometry_msgs::PoseStamped& target_pose,
                    geometry_msgs::TwistStamped& ff_twist,
                    ros::Time& target_stamp,
                    ros::Time& ff_stamp) const;

  bool transformTargetPoseToModelFrame(const geometry_msgs::PoseStamped& in,
                                      const std::string& model_frame,
                                      geometry_msgs::PoseStamped& out) const;

  bool rotateTwistToModelFrame(const geometry_msgs::TwistStamped& in,
                               const std::string& model_frame,
                               geometry_msgs::TwistStamped& out) const;

  static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& p);
  static geometry_msgs::Pose eigenToPoseMsg(const Eigen::Isometry3d& T);

  static Eigen::VectorXd twistMsgToEigen6(const geometry_msgs::Twist& t);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // TF for frame conversions (target pose / twist)
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Core components
  std::unique_ptr<RobotStateManager> robot_state_manager_;
  JacobianSolver jacobian_solver_;
  PIDController pid_pos_{3};
  PIDController pid_ori_{3};
  std::unique_ptr<JointSafetyLimiter> joint_safety_limiter_;

  // ROS interfaces
  ros::Subscriber sub_target_pose_;
  ros::Subscriber sub_ff_twist_;
  ros::Subscriber sub_joint_states_;
  ros::Publisher pub_qdot_cmd_;
  ros::Timer control_timer_;

  // Cached inputs
  mutable std::mutex input_mutex_;
  geometry_msgs::PoseStamped last_target_pose_;
  geometry_msgs::TwistStamped last_ff_twist_;
  ros::Time last_target_pose_stamp_{0};
  ros::Time last_ff_twist_stamp_{0};
  bool has_target_pose_{false};
  bool has_ff_twist_{false};

  // Controller state
  bool is_running_{false};
  ros::Time last_control_time_{0};
  Eigen::VectorXd prev_qdot_cmd_;
  bool has_prev_qdot_{false};

  // TCP pose EMA filter
  bool has_filtered_tcp_pose_{false};
  Eigen::Isometry3d filtered_tcp_pose_{Eigen::Isometry3d::Identity()};

  // Parameters
  // TF prefix for multi-robot setups (e.g. "mur620_s"). If set, the controller
  // will use "<tf_prefix>/<robot_model_root_link>" as the TF target frame when
  // transforming target pose / twist inputs.
  std::string tf_prefix_{""};

  std::string group_name_;
  std::string tcp_link_;
  std::string robot_description_param_{"robot_description"};

  std::string target_pose_topic_{"target_pose"};
  std::string feedforward_twist_topic_{"feedforward_twist"};
  std::string joint_state_topic_{"joint_states"};
  std::string velocity_command_topic_{"velocity_command_topic"};

  double control_rate_{250.0};
  double k_ff_{1.0};

  double target_pose_timeout_{0.2};
  double feedforward_timeout_{0.2};

  // Deadband (DISATTIVABILE)
  bool deadband_enabled_{false};
  double position_deadband_m_{0.0};
  double orientation_deadband_rad_{0.0};

  // TCP pose EMA filter (MANTIENI)
  double tcp_pose_filter_alpha_{0.2};

  // Jacobian
  // If jacobian_source_frame is empty, no conversion is applied (legacy behavior).
  // If set, the controller rotates the Jacobian rows from jacobian_source_frame into
  // jacobian_target_frame (if set) or into the model_frame used for v_cmd.
  std::string jacobian_source_frame_{""};
  std::string jacobian_target_frame_{""};

  // Cached 6x6 rotation transform used to express Jacobian rows in target frame.
  std::mutex jacobian_frame_mutex_;
  bool jacobian_frame_transform_ready_{false};
  Eigen::Matrix<double, 6, 6> jacobian_frame_transform_{Eigen::Matrix<double, 6, 6>::Identity()};

  // Limits
  Eigen::VectorXd max_joint_velocities_;
  Eigen::VectorXd max_joint_accelerations_;
  bool acceleration_limiting_enabled_{false};  // DISATTIVABILE

  // TCP offset (optional)
  Eigen::Isometry3d tcp_offset_{Eigen::Isometry3d::Identity()};

  // Queue size
  int queue_size_{1};
};

}  // namespace teleoperation

