#pragma once

/**
 * @file cartesian_velocity_controller.hpp
 * @brief Main orchestrator for the Cartesian velocity controller pipeline.
 *
 * Pipeline Architecture (Attractive Component Only):
 * - Level A: Global Planner (waypoint management)
 * - Level B: Local Planner (attractive velocity computation + target integration)
 * - Level C: Motion Generator (Cartesian velocity filter with τ)
 * - Level D: PID + IK (feed forward + PID + damped Jacobian inverse)
 * - Final: Joint Safety Limiter (uniform scaling)
 */

#include <ros/ros.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include "cartesian_velocity_controller/GetFrameInfo.h"
#include "cartesian_velocity_controller/GetJacobian.h"
#include <boost/thread/recursive_mutex.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cartesian_velocity_controller/types/config_types.hpp"
#include "cartesian_velocity_controller/types/controller_joint_limits.hpp"
#include "cartesian_velocity_controller/types/pipeline_types.hpp"
#include "cartesian_velocity_controller/ControllerTuningConfig.h"

namespace cartesian_velocity_controller
{

// Forward declarations
class RobotStateManager;
class JacobianSolver;
class GlobalPlanner;
class LocalPlanner;
class CartesianVelocityFilter;
class JointVelocityFilter;
class PIDController;
class JointSafetyLimiter;
class MarkerPublisher;
class FeedbackPublisher;
class RepulsionDataManager;
class JointWeightManager;
class JointPositionGuard;
namespace map3d
{
class Map3DManager;
}

/**
 * @class CartesianVelocityController
 * @brief Main controller that orchestrates the velocity control pipeline.
 *
 * Pipeline flow:
 * 1. Target pose → GlobalPlanner (manages waypoints)
 * 2. GlobalPlanner → LocalPlanner (computes attractive velocity, integrates target_raw)
 * 3. LocalPlanner → CartesianVelocityFilter (filters velocity, produces target_filtered)
 * 4. CartesianVelocityFilter → PIDController (PID on pose error)
 * 5. PIDController → JacobianSolver (converts Cartesian to joint velocities)
 * 6. Joint velocities → JointSafetyLimiter (uniform scaling for safety)
 * 7. Publish command
 */
class CartesianVelocityController
{
public:
  CartesianVelocityController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~CartesianVelocityController();

  void start();
  void stop();

  // ============== Target Pose Control ==============

  /**
   * @brief Set target pose (single waypoint mode).
   * @return true if pose was accepted, false if rejected (e.g., not reachable)
   */
  bool setTargetPose(const Eigen::Isometry3d& pose);
  bool setTargetPose(const geometry_msgs::Pose& pose);
  bool setTargetPose(const geometry_msgs::PoseStamped& pose_stamped);
  
  /**
   * @brief Set multiple waypoints for path following.
   * @return true if all waypoints are reachable, false if any was rejected
   */
  bool setWaypoints(const std::vector<Eigen::Isometry3d>& waypoints);
  
  void clearTargetPose();
  Eigen::Isometry3d getTargetPose() const;
  
  /**
   * @brief Check if we've reached the current/final target.
   */
  bool hasReachedTarget() const;

  // ============== PID Control ==============

  void setPIDGains(double kp_pos, double ki_pos, double kd_pos,
                   double kp_ori, double ki_ori, double kd_ori);
  void setFeedForwardGain(double kff_pos, double kff_ori);
  void resetPIDControllers();

  // ============== Velocity Filter (Level C) ==============

  void setCartesianFilterEnabled(bool enabled);
  bool isCartesianFilterEnabled() const;

  void setCartesianFilterTimeConstant(double tau);
  double getCartesianFilterTimeConstant() const;

  void resetVelocityFilter();

  // ============== Joint Safety Limiter ==============

  void setJointVelocityLimit(double max_vel);
  void setJointAccelerationLimit(double max_acc);

  // ============== TCP Configuration ==============

  void setTcpOffset(const Eigen::Isometry3d& offset);
  Eigen::Isometry3d getTcpOffset() const;
  std::string getTcpLink() const;

  // ============== Diagnostics ==============

  double getDistanceToTarget() const;
  double getOrientationErrorToTarget() const;
  Eigen::VectorXd getLastJointVelocityCommand() const;

private:
  enum class StartupBehavior
  {
    HOLD_INITIAL_POSE,  ///< Current behavior: capture pose and hold it as initial target
    ZERO_VELOCITY       ///< New behavior: publish zero velocity until user sets a target
  };

  struct PoseTrackingParams
  {
    bool enabled{true};
    double tau_linear{0.1};   // seconds
    double tau_angular{0.1};  // seconds
    double k_linear{1.0};     // dimensionless
    double k_angular{1.0};    // dimensionless

    // Base limits (unscaled). Will be scaled by velocity_scale_factor_ at runtime.
    double max_linear_velocity_base{0.5};   // m/s
    double max_angular_velocity_base{1.0};  // rad/s
  };

  // Initialization
  void loadParameters();
  void loadPipelineParameters();
  void initializeComponents();
  void setupRosInterfaces();
  void captureControllerState();
  void captureInitialPose();

  // ROS Callbacks
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void controlLoopCallback(const ros::TimerEvent& event);

  // Pipeline execution
  void executePipeline(double dt);
  
  // Velocity Publishing
  void publishVelocityCommand(const Eigen::VectorXd& joint_velocities);
  void publishZeroVelocity();

  // Controller Switching
  bool switchControllers(const std::vector<std::string>& start, const std::vector<std::string>& stop);
  void handleShutdown();
  void handleSigint();
  static void sigintHandler(int signum);

  // Utility
  Eigen::Isometry3d poseToIsometry(const geometry_msgs::Pose& pose) const;
  Eigen::Isometry3d filterTcpPose(const Eigen::Isometry3d& measured_pose);
  static void applyDeadband(Eigen::Vector3d& vec, double threshold);
  bool resetVirtualTargetsToCurrentPose();
  void setupDynamicReconfigure();
  void dynamicReconfigureCallback(ControllerTuningConfig& config, uint32_t level);
  bool getFrameInfoCallback(GetFrameInfo::Request& req, GetFrameInfo::Response& res);
  bool getJacobianCallback(GetJacobian::Request& req, GetJacobian::Response& res);

  // ============== ROS Interfaces ==============
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // TF2 (for target PoseStamped frame transformations)
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  double tf_timeout_{0.1};
  bool reject_on_tf_failure_{true};
  bool accept_empty_frame_as_global_{true};

  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_pose_sub_;
  ros::Publisher velocity_pub_;
  ros::ServiceClient switch_client_;
  ros::ServiceClient list_client_;
  ros::ServiceServer get_frame_info_server_;
  ros::ServiceServer get_jacobian_server_;
  ros::Timer control_timer_;

  // ============== Pipeline Components ==============
  std::shared_ptr<RobotStateManager> robot_state_;
  std::shared_ptr<JacobianSolver> jacobian_solver_;
  
  // Level A: Global Planner
  std::unique_ptr<GlobalPlanner> global_planner_;
  
  // Level B: Local Planner
  std::unique_ptr<LocalPlanner> local_planner_;
  
  // Level C: Motion Generator (Velocity Filter)
  std::unique_ptr<CartesianVelocityFilter> velocity_filter_;

  // Joint-space smoothness filter (before safety limiter)
  std::unique_ptr<JointVelocityFilter> joint_velocity_filter_;
  
  // Level D: PID Controllers
  std::unique_ptr<PIDController> pid_position_;
  std::unique_ptr<PIDController> pid_orientation_;
  
  // Final: Joint Safety Limiter
  std::unique_ptr<JointSafetyLimiter> safety_limiter_;
  
  // Visualization and Feedback
  std::unique_ptr<MarkerPublisher> marker_publisher_;
  std::unique_ptr<FeedbackPublisher> feedback_publisher_;
  
  // Repulsion Data Manager
  std::unique_ptr<RepulsionDataManager> repulsion_manager_;

  // Local 3D map used for repulsion queries
  std::shared_ptr<map3d::Map3DManager> map3d_manager_;

  // Joint Weight Manager
  std::unique_ptr<JointWeightManager> weight_manager_;

  // ============== Configuration ==============
  std::string group_name_;
  std::string tcp_link_;
  std::string global_frame_;
  std::string joint_state_topic_;
  std::string velocity_command_topic_;
  std::string start_controller_name_;
  std::string stop_controller_name_;
  std::string controller_manager_ns_;
  std::string robot_description_param_;
  bool map3d_enabled_{true};

  double control_rate_{50.0};
  double command_timeout_{0.5};
  
  // TCP offset
  Eigen::Isometry3d tcp_offset_{Eigen::Isometry3d::Identity()};
  mutable std::mutex tcp_mutex_;

  // Joint weights for weighted pseudo-inverse
  Eigen::VectorXd joint_weights_;

  // Pose filtering and deadband
  Eigen::Isometry3d filtered_tcp_pose_{Eigen::Isometry3d::Identity()};
  bool has_filtered_tcp_pose_{false};
  double pose_filter_alpha_{0.85};
  double position_deadband_{0.0002};     ///< meters
  double orientation_deadband_{0.001};  ///< radians
  bool repulsive_enabled_{false};
  double acceleration_scale_factor_{1.0};
  double velocity_scale_factor_{1.0};

  // Pose tracking (target_raw -> target_filtered) used to generate the filter input twist
  PoseTrackingParams pose_tracking_;

  // Optional extra caps for the velocity filter (Level C). If not provided via params,
  // the filter max velocities will track the local planner caps.
  bool has_cartesian_linear_max_velocity_override_{false};
  bool has_cartesian_angular_max_velocity_override_{false};
  double cartesian_linear_max_velocity_override_base_{0.0};
  double cartesian_angular_max_velocity_override_base_{0.0};

  bool reset_filter_on_target_change_{true};
  bool reachability_check_enabled_{true};  ///< If true, validate poses via IK before accepting

  // Controller-only joint limits (reachability + runtime guardrail)
  ControllerJointLimitsConfig controller_joint_limits_;

  // Elbow velocity injection (anti-singularity, gated via IK target solution)
  ElbowInjectionConfig elbow_injection_;
  int elbow_injection_index_resolved_{-1};
  bool elbow_injection_has_qmax_{false};
  double elbow_injection_qmax_{0.0};

  // IK solution cache for target gating
  bool has_last_target_ik_solution_{false};
  Eigen::VectorXd last_target_ik_solution_;
  std::vector<Eigen::VectorXd> waypoint_ik_solutions_;
  std::vector<bool> waypoint_ik_solutions_valid_;

  // Runtime guardrail component
  std::unique_ptr<JointPositionGuard> joint_position_guard_;
  bool joint_velocity_filter_forced_on_{false};

  // Dynamic reconfigure
  using DynReconfServer = dynamic_reconfigure::Server<ControllerTuningConfig>;
  using DynReconfMutex = boost::recursive_mutex;
  std::unique_ptr<DynReconfServer> dynamic_reconfigure_server_;
  std::shared_ptr<DynReconfMutex> dynamic_reconfigure_mutex_;

  // ============== Control State ==============
  std_msgs::Float64MultiArray last_command_;
  Eigen::VectorXd previous_joint_velocity_;
  ros::Time last_command_stamp_;
  
  bool is_running_{false};
  bool has_target_{false};
  StartupBehavior startup_behavior_{StartupBehavior::HOLD_INITIAL_POSE};
  bool idle_until_target_{false};
  bool pose_initialized_{false};  ///< True after initial pose has been captured
  bool controllers_switched_{false};
  bool controller_state_captured_{false};
  bool start_controller_initially_running_{false};
  bool stop_controller_initially_running_{false};
  bool start_controller_started_by_node_{false};
  bool stop_controller_stopped_by_node_{false};

  // ============== Diagnostics ==============
  double last_position_error_{0.0};
  double last_orientation_error_{0.0};
  mutable std::mutex diagnostics_mutex_;

  // Singleton for SIGINT handling
  static CartesianVelocityController* instance_;
};

}  // namespace cartesian_velocity_controller
