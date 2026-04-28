/**
 * @file cartesian_velocity_controller.cpp
 * @brief Implementation of the Cartesian velocity controller pipeline.
 *
 * Pipeline flow:
 * 1. Target pose → GlobalPlanner
 * 2. GlobalPlanner → LocalPlanner (attractive velocity + target_raw integration)
 * 3. LocalPlanner → VelocityFilter (filtering + target_filtered)
 * 4. VelocityFilter → PIDController (pose error correction)
 * 5. PIDController → JacobianSolver (Cartesian to joint space)
 * 6. Joint velocities → JointSafetyLimiter (uniform scaling)
 * 7. Publish command
 */

#include "cartesian_velocity_controller/cartesian_velocity_controller.hpp"

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cctype>
#include <boost/bind.hpp>

#include <ros/names.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Pipeline components
#include "cartesian_velocity_controller/components/robot_state_manager.hpp"
#include "cartesian_velocity_controller/components/jacobian_solver.hpp"
#include "cartesian_velocity_controller/components/global_planner.hpp"
#include "cartesian_velocity_controller/components/local_planner.hpp"
#include "cartesian_velocity_controller/velocity_filter.hpp"
#include "cartesian_velocity_controller/joint_velocity_filter.hpp"
#include "cartesian_velocity_controller/components/pid_controller.hpp"
#include "cartesian_velocity_controller/components/joint_safety_limiter.hpp"
#include "cartesian_velocity_controller/components/marker_publisher.hpp"
#include "cartesian_velocity_controller/components/feedback_publisher.hpp"
#include "cartesian_velocity_controller/components/repulsion_data_manager.hpp"
#include "cartesian_velocity_controller/components/joint_weight_manager.hpp"
#include "cartesian_velocity_controller/components/joint_position_guard.hpp"
#include "cartesian_velocity_controller/map3d/map3d_manager.hpp"

#include <xmlrpcpp/XmlRpcValue.h>

namespace
{
constexpr double kPoseTrackEps = 1e-10;

using cartesian_velocity_controller::RepulsiveVelocityMode;

static std::string toLowerCopy(const std::string& in)
{
  std::string out;
  out.reserve(in.size());
  for (char c : in)
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  return out;
}

static RepulsiveVelocityMode repulsiveModeFromString(const std::string& s_in)
{
  std::string s;
  s.reserve(s_in.size());
  for (char c : s_in)
    s.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

  if (s == "LINEAR") return cartesian_velocity_controller::RepulsiveVelocityMode::LINEAR;
  if (s == "QUADRATIC") return cartesian_velocity_controller::RepulsiveVelocityMode::QUADRATIC;
  if (s == "SMOOTHSTEP") return cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHSTEP;
  if (s == "SMOOTHERSTEP") return cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHERSTEP;
  return cartesian_velocity_controller::RepulsiveVelocityMode::QUADRATIC;
}

static RepulsiveVelocityMode repulsiveModeFromInt(int v)
{
  switch (v)
  {
    case 0: return cartesian_velocity_controller::RepulsiveVelocityMode::LINEAR;
    case 1: return cartesian_velocity_controller::RepulsiveVelocityMode::QUADRATIC;
    case 2: return cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHSTEP;
    case 3: return cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHERSTEP;
    default: return cartesian_velocity_controller::RepulsiveVelocityMode::QUADRATIC;
  }
}

static Eigen::Vector3d vector3ToEigen(const geometry_msgs::Vector3& v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

static Eigen::Vector3d rotateVectorByTransform(const geometry_msgs::TransformStamped& transform,
                                               const Eigen::Vector3d& v)
{
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  const tf2::Vector3 rotated = tf2::quatRotate(q, tf2::Vector3(v.x(), v.y(), v.z()));
  return Eigen::Vector3d(rotated.x(), rotated.y(), rotated.z());
}

static int repulsiveModeToInt(RepulsiveVelocityMode m)
{
  switch (m)
  {
    case cartesian_velocity_controller::RepulsiveVelocityMode::LINEAR: return 0;
    case cartesian_velocity_controller::RepulsiveVelocityMode::QUADRATIC: return 1;
    case cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHSTEP: return 2;
    case cartesian_velocity_controller::RepulsiveVelocityMode::SMOOTHERSTEP: return 3;
    default: return 1;
  }
}

inline double clamp01(double x)
{
  return std::clamp(x, 0.0, 1.0);
}

inline double smoothstep(double x)
{
  const double t = clamp01(x);
  return t * t * (3.0 - 2.0 * t);
}

inline Eigen::Vector3d limitNorm(const Eigen::Vector3d& v, double max_norm)
{
  if (max_norm < kPoseTrackEps) return Eigen::Vector3d::Zero();
  const double n = v.norm();
  if (n > max_norm && n > kPoseTrackEps)
  {
    return v * (max_norm / n);
  }
  return v;
}

inline Eigen::Vector3d orientationErrorAxisAngle(const Eigen::Quaterniond& q_current,
                                                 const Eigen::Quaterniond& q_target)
{
  Eigen::Quaterniond q_curr = q_current.normalized();
  Eigen::Quaterniond q_tgt = q_target.normalized();

  // Ensure shortest path
  if (q_curr.dot(q_tgt) < 0.0)
  {
    q_tgt.coeffs() = -q_tgt.coeffs();
  }

  // Relative rotation error: R_error = R_target * R_current^{-1}
  const Eigen::Quaterniond q_error = q_tgt * q_curr.inverse();
  const Eigen::AngleAxisd aa(q_error);
  const double angle = aa.angle();
  if (!std::isfinite(angle) || std::abs(angle) < kPoseTrackEps)
  {
    return Eigen::Vector3d::Zero();
  }
  return angle * aa.axis();
}

inline bool xmlRpcToBool(const XmlRpc::XmlRpcValue& v, bool& out)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    out = static_cast<bool>(v);
    return true;
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = (static_cast<int>(v) != 0);
    return true;
  }
  return false;
}

inline bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& v, double& out)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(v);
    return true;
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(v);
    return true;
  }
  return false;
}
}  // namespace

namespace cartesian_velocity_controller
{

CartesianVelocityController* CartesianVelocityController::instance_ = nullptr;

CartesianVelocityController::CartesianVelocityController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
{
  // TF listener must exist to populate the buffer for PoseStamped transformations.
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  loadParameters();
  loadPipelineParameters();
  initializeComponents();
  setupRosInterfaces();
  setupDynamicReconfigure();

  const double period = control_rate_ > 0.0 ? 1.0 / control_rate_ : 0.02;
  control_timer_ = nh_.createTimer(ros::Duration(period),
                                   &CartesianVelocityController::controlLoopCallback,
                                   this, false, false);

  instance_ = this;
  signal(SIGINT, CartesianVelocityController::sigintHandler);

  ROS_INFO_STREAM_NAMED("cartesian_velocity_controller",
                        "CartesianVelocityController ready. TCP: " << tcp_link_
                        << ", Rate: " << control_rate_ << " Hz");
}

CartesianVelocityController::~CartesianVelocityController()
{
  if (instance_ == this)
  {
    instance_ = nullptr;
  }
  handleShutdown();
}

// ============================================================================
// Start/Stop
// ============================================================================

void CartesianVelocityController::start()
{
  if (is_running_) return;

  captureControllerState();

  start_controller_started_by_node_ = false;
  stop_controller_stopped_by_node_ = false;

  std::vector<std::string> start_list, stop_list;

  if (!start_controller_name_.empty() && !start_controller_initially_running_)
  {
    start_list.push_back(start_controller_name_);
    start_controller_started_by_node_ = true;
  }

  if (!stop_controller_name_.empty() && stop_controller_initially_running_)
  {
    stop_list.push_back(stop_controller_name_);
    stop_controller_stopped_by_node_ = true;
  }

  if (!start_list.empty() || !stop_list.empty())
  {
    if (switchControllers(start_list, stop_list))
    {
      controllers_switched_ = start_controller_started_by_node_ || stop_controller_stopped_by_node_;
    }
    else
    {
      start_controller_started_by_node_ = false;
      stop_controller_stopped_by_node_ = false;
      controllers_switched_ = false;
    }
  }

  // Reset pipeline components
  resetPIDControllers();
  resetVelocityFilter();
  
  if (global_planner_)
  {
    global_planner_->reset();
  }
  
  if (local_planner_)
  {
    local_planner_->reset();
  }
  
  if (safety_limiter_)
  {
    safety_limiter_->reset();
  }

  // Initialize previous velocity
  std::size_t num_joints = robot_state_ ? robot_state_->getJointCount() : 6;
  previous_joint_velocity_ = Eigen::VectorXd::Zero(num_joints);

  // Reset pose initialization flag so that initial pose is captured on first control loop iteration
  pose_initialized_ = false;
  has_filtered_tcp_pose_ = false;

  control_timer_.start();
  is_running_ = true;

  ROS_INFO_NAMED("cartesian_velocity_controller", "Controller started");
}

void CartesianVelocityController::stop()
{
  if (!is_running_) return;

  control_timer_.stop();

  // Restore original controller state
  if (controllers_switched_)
  {
    std::vector<std::string> start_list, stop_list;

    if (stop_controller_stopped_by_node_ && !stop_controller_name_.empty())
    {
      start_list.push_back(stop_controller_name_);
    }
    if (start_controller_started_by_node_ && !start_controller_name_.empty())
    {
      stop_list.push_back(start_controller_name_);
    }

    if (!start_list.empty() || !stop_list.empty())
    {
      switchControllers(start_list, stop_list);
    }
  }

  controllers_switched_ = false;
  start_controller_started_by_node_ = false;
  stop_controller_stopped_by_node_ = false;
  controller_state_captured_ = false;
  is_running_ = false;

  publishZeroVelocity();

  if (marker_publisher_)
  {
    marker_publisher_->clear();
  }

  ROS_INFO_NAMED("cartesian_velocity_controller", "Controller stopped");
}

// ============================================================================
// Parameter Loading
// ============================================================================

void CartesianVelocityController::loadParameters()
{
  pnh_.param<std::string>("group_name", group_name_, "manipulator");
  pnh_.param<std::string>("tcp_link", tcp_link_, "tool0");
  pnh_.param<std::string>("global_frame", global_frame_, "world");
  // Optional Jacobian frame conversion (rotation only).
  // If jacobian_source_frame is empty, the Jacobian is left untouched (legacy behavior).
  // If set, the Jacobian rows are rotated into jacobian_target_frame (default: global_frame).
  pnh_.param<std::string>("jacobian_source_frame", jacobian_source_frame_, "");
  pnh_.param<std::string>("jacobian_target_frame", jacobian_target_frame_, global_frame_);
  pnh_.param<std::string>("joint_state_topic", joint_state_topic_, "/joint_states");
  pnh_.param<std::string>("velocity_command_topic", velocity_command_topic_, "/joint_group_vel_controller/command");
  pnh_.param<std::string>("start_controller", start_controller_name_, "joint_group_vel_controller");
  pnh_.param<std::string>("stop_controller", stop_controller_name_, "vel_joint_traj_controller");
  pnh_.param<std::string>("controller_manager_ns", controller_manager_ns_, "controller_manager");
  pnh_.param<std::string>("robot_description_param", robot_description_param_, "robot_description");
  pnh_.param("map3d/enabled", map3d_enabled_, true);

  // TF / PoseStamped handling (robust frame behavior)
  pnh_.param("tf_timeout", tf_timeout_, 0.1);
  pnh_.param("reject_on_tf_failure", reject_on_tf_failure_, true);
  pnh_.param("accept_empty_frame_as_global", accept_empty_frame_as_global_, true);

  // Startup behavior:
  // - hold_current_pose (default): capture initial TCP pose and hold it as target
  // - zero_velocity: publish zero velocity until a user target is received
  std::string startup_behavior_str;
  pnh_.param<std::string>("startup_behavior", startup_behavior_str, "hold_current_pose");
  const std::string startup_behavior_norm = toLowerCopy(startup_behavior_str);
  if (startup_behavior_norm == "hold_current_pose" ||
      startup_behavior_norm == "hold_position" ||
      startup_behavior_norm == "hold_initial_pose")
  {
    startup_behavior_ = StartupBehavior::HOLD_INITIAL_POSE;
    idle_until_target_ = false;
  }
  else if (startup_behavior_norm == "zero_velocity" ||
           startup_behavior_norm == "idle_zero_velocity" ||
           startup_behavior_norm == "idle")
  {
    startup_behavior_ = StartupBehavior::ZERO_VELOCITY;
    idle_until_target_ = true;
  }
  else
  {
    ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                          "Unknown startup_behavior '" << startup_behavior_str
                                                       << "'. Falling back to 'hold_current_pose'.");
    startup_behavior_ = StartupBehavior::HOLD_INITIAL_POSE;
    idle_until_target_ = false;
  }

  pnh_.param("control_rate", control_rate_, 50.0);
  pnh_.param("command_timeout", command_timeout_, 0.5);
  pnh_.param("target_state_timeout", target_state_timeout_, 0.25);
  pnh_.param("target_state_zero_velocity_on_timeout", target_state_zero_velocity_on_timeout_, false);
  target_state_timeout_ = std::max(0.0, target_state_timeout_);

  pnh_.param("pose_filter_alpha", pose_filter_alpha_, 0.85);
  pose_filter_alpha_ = std::clamp(pose_filter_alpha_, 0.0, 1.0);

  pnh_.param("pid_position_deadband", position_deadband_, 0.0002);
  pnh_.param("pid_orientation_deadband", orientation_deadband_, 0.001);
  position_deadband_ = std::max(0.0, position_deadband_);
  orientation_deadband_ = std::max(0.0, orientation_deadband_);

  // TCP offset
  std::vector<double> tcp_offset_pos, tcp_offset_rpy;
  pnh_.param("tcp_offset_position", tcp_offset_pos, std::vector<double>{0.0, 0.0, 0.0});
  pnh_.param("tcp_offset_orientation_rpy", tcp_offset_rpy, std::vector<double>{0.0, 0.0, 0.0});

  if (tcp_offset_pos.size() >= 3 && tcp_offset_rpy.size() >= 3)
  {
    tcp_offset_ = Eigen::Isometry3d::Identity();
    tcp_offset_.translation() = Eigen::Vector3d(tcp_offset_pos[0], tcp_offset_pos[1], tcp_offset_pos[2]);
    Eigen::AngleAxisd roll(tcp_offset_rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(tcp_offset_rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(tcp_offset_rpy[2], Eigen::Vector3d::UnitZ());
    tcp_offset_.linear() = (yaw * pitch * roll).toRotationMatrix();
  }
}

void CartesianVelocityController::loadPipelineParameters()
{
  // Parameters are loaded in initializeComponents() where they're used
}

// ============================================================================
// Component Initialization
// ============================================================================

void CartesianVelocityController::initializeComponents()
{
  // Robot State Manager
  robot_state_ = std::make_shared<RobotStateManager>(group_name_, tcp_link_, robot_description_param_);
  const std::size_t num_joints = robot_state_->getJointCount();

  // --------------------------------------------------------------------------
  // Controller-only joint limits (reachability + runtime guardrail)
  // --------------------------------------------------------------------------
  controller_joint_limits_ = ControllerJointLimitsConfig{};
  pnh_.param("controller_joint_limits/enabled", controller_joint_limits_.enabled, false);

  if (controller_joint_limits_.enabled)
  {
    XmlRpc::XmlRpcValue limits_struct;
    if (pnh_.getParam("controller_joint_limits/limits", limits_struct) &&
        limits_struct.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      const auto& joint_index_map = robot_state_->getJointIndexMap();

      for (auto it = limits_struct.begin(); it != limits_struct.end(); ++it)
      {
        const std::string joint_name = it->first;
        const XmlRpc::XmlRpcValue& joint_cfg = it->second;

        if (joint_index_map.find(joint_name) == joint_index_map.end())
        {
          ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                                "controller_joint_limits: joint '" << joint_name
                                                                  << "' not in group '" << group_name_
                                                                  << "'. Ignoring.");
          continue;
        }

        if (joint_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                                "controller_joint_limits: joint '" << joint_name
                                                                  << "' is not a struct. Ignoring.");
          continue;
        }

        ControllerJointLimit lim;
        lim.enabled = true;

        if (joint_cfg.hasMember("enabled"))
        {
          (void)xmlRpcToBool(joint_cfg["enabled"], lim.enabled);
        }

        if (!joint_cfg.hasMember("min") || !joint_cfg.hasMember("max") ||
            !xmlRpcToDouble(joint_cfg["min"], lim.min) || !xmlRpcToDouble(joint_cfg["max"], lim.max))
        {
          ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                                "controller_joint_limits: joint '" << joint_name
                                                                  << "' missing valid min/max. Ignoring.");
          continue;
        }

        if (!(lim.min < lim.max))
        {
          ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                                "controller_joint_limits: joint '" << joint_name
                                                                  << "' has invalid range (min>=max). Ignoring.");
          continue;
        }

        if (joint_cfg.hasMember("runtime_guard"))
        {
          const XmlRpc::XmlRpcValue& rg = joint_cfg["runtime_guard"];
          if (rg.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            if (rg.hasMember("enabled")) (void)xmlRpcToBool(rg["enabled"], lim.runtime_guard.enabled);
            if (rg.hasMember("soft_zone")) (void)xmlRpcToDouble(rg["soft_zone"], lim.runtime_guard.soft_zone);
            if (rg.hasMember("margin")) (void)xmlRpcToDouble(rg["margin"], lim.runtime_guard.margin);
            if (rg.hasMember("reentry_velocity")) (void)xmlRpcToDouble(rg["reentry_velocity"], lim.runtime_guard.reentry_velocity);
          }
        }

        controller_joint_limits_.limits[joint_name] = lim;
      }
    }
    else
    {
      ROS_WARN_NAMED("cartesian_velocity_controller",
                     "controller_joint_limits enabled but no valid 'controller_joint_limits/limits' found.");
    }
  }

  // Runtime joint position guardrail component
  joint_position_guard_ = std::make_unique<JointPositionGuard>();
  joint_position_guard_->configure(controller_joint_limits_, robot_state_->getJointIndexMap());

  // --------------------------------------------------------------------------
  // Elbow injection (anti-singularity) configuration
  // --------------------------------------------------------------------------
  elbow_injection_ = ElbowInjectionConfig{};
  pnh_.param("elbow_injection/enabled", elbow_injection_.enabled, false);
  pnh_.param<std::string>("elbow_injection/elbow_joint_name", elbow_injection_.elbow_joint_name, elbow_injection_.elbow_joint_name);
  pnh_.param("elbow_injection/elbow_index", elbow_injection_.elbow_index, elbow_injection_.elbow_index);
  pnh_.param("elbow_injection/critical_zone", elbow_injection_.critical_zone, elbow_injection_.critical_zone);
  pnh_.param("elbow_injection/target_near_limit_threshold", elbow_injection_.target_near_limit_threshold,
             elbow_injection_.target_near_limit_threshold);
  pnh_.param("elbow_injection/k", elbow_injection_.k, elbow_injection_.k);
  pnh_.param("elbow_injection/max_push_velocity", elbow_injection_.max_push_velocity, elbow_injection_.max_push_velocity);

  // Resolve elbow index (prefer name mapping)
  elbow_injection_index_resolved_ = -1;
  std::string elbow_joint_name_used;
  const auto& joint_index_map = robot_state_->getJointIndexMap();
  const auto& joint_names = robot_state_->getJointNames();

  if (!elbow_injection_.elbow_joint_name.empty())
  {
    auto it = joint_index_map.find(elbow_injection_.elbow_joint_name);
    if (it != joint_index_map.end())
    {
      elbow_injection_index_resolved_ = static_cast<int>(it->second);
      elbow_joint_name_used = elbow_injection_.elbow_joint_name;
    }
    else
    {
      ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                            "elbow_injection: elbow_joint_name '" << elbow_injection_.elbow_joint_name
                                                                 << "' not found in group. Falling back to elbow_index.");
    }
  }

  if (elbow_injection_index_resolved_ < 0)
  {
    if (elbow_injection_.elbow_index >= 0 && elbow_injection_.elbow_index < static_cast<int>(joint_names.size()))
    {
      elbow_injection_index_resolved_ = elbow_injection_.elbow_index;
      elbow_joint_name_used = joint_names[static_cast<std::size_t>(elbow_injection_index_resolved_)];
    }
  }

  // Resolve elbow q_max from controller-only limits (required for injection behavior)
  elbow_injection_has_qmax_ = false;
  elbow_injection_qmax_ = 0.0;
  if (!elbow_joint_name_used.empty())
  {
    auto lim_it = controller_joint_limits_.limits.find(elbow_joint_name_used);
    if (lim_it != controller_joint_limits_.limits.end() && lim_it->second.enabled)
    {
      elbow_injection_qmax_ = lim_it->second.max;
      elbow_injection_has_qmax_ = true;
    }
  }

  if (elbow_injection_.enabled && (!elbow_injection_has_qmax_ || elbow_injection_index_resolved_ < 0))
  {
    ROS_WARN_NAMED("cartesian_velocity_controller",
                   "elbow_injection enabled but elbow joint/q_max could not be resolved from controller_joint_limits. Disabling injection.");
    elbow_injection_.enabled = false;
  }

  // Reset IK caches (used for target gating)
  has_last_target_ik_solution_ = false;
  last_target_ik_solution_.resize(0);
  waypoint_ik_solutions_.clear();
  waypoint_ik_solutions_valid_.clear();

  // Joint Weight Manager
  weight_manager_ = std::make_unique<JointWeightManager>(num_joints);
  
  // Load params for weight manager
  int elbow_idx;
  double sing_buf, max_w_pen;
  pnh_.param("singularity_avoidance/elbow_index", elbow_idx, 2);
  pnh_.param("singularity_avoidance/buffer_zone", sing_buf, 0.2);
  pnh_.param("singularity_avoidance/max_weight_penalty", max_w_pen, 50.0);
  
  weight_manager_->setElbowSingularityConfig(elbow_idx, sing_buf, max_w_pen);

  // Joint weights (initialized to ones, will be updated by manager)
  joint_weights_ = Eigen::VectorXd::Ones(num_joints);

  // Jacobian Solver
  JacobianSolverConfig jacobian_config;
  pnh_.param("jacobian_solver/singularity_threshold", jacobian_config.singularity_threshold, 0.05);
  pnh_.param("jacobian_solver/max_damping", jacobian_config.max_damping, 0.3);
  jacobian_solver_ = std::make_shared<JacobianSolver>(jacobian_config);

  // Level A: Global Planner
  global_planner_ = std::make_unique<GlobalPlanner>();
  
  double wp_switch_dist, ori_switch_thresh;
  bool use_ori_switch;
  pnh_.param("global_planner/waypoint_switch_distance", wp_switch_dist, 0.02);
  pnh_.param("global_planner/orientation_switch_threshold", ori_switch_thresh, 0.05);
  pnh_.param("global_planner/use_orientation_for_switch", use_ori_switch, true);
  
  global_planner_->setWaypointSwitchDistance(wp_switch_dist);
  global_planner_->setOrientationSwitchThreshold(ori_switch_thresh);
  global_planner_->setUseOrientationForSwitch(use_ori_switch);

  // Reachability check via IK (validates poses before accepting them)
  // Backward compatibility:
  // - legacy param: ~block_unreachable_targets
  // - new param:    ~global_planner/reachability_check_enabled
  const bool has_new_reach_param = pnh_.hasParam("global_planner/reachability_check_enabled");
  bool legacy_block_unreachable = true;
  const bool has_legacy_param = pnh_.getParam("block_unreachable_targets", legacy_block_unreachable);

  if (!has_new_reach_param && has_legacy_param)
  {
    reachability_check_enabled_ = legacy_block_unreachable;
  }
  else
  {
    pnh_.param("global_planner/reachability_check_enabled", reachability_check_enabled_, true);
    if (has_new_reach_param && has_legacy_param)
    {
      ROS_WARN_NAMED("cartesian_velocity_controller",
                     "Both 'block_unreachable_targets' (legacy) and 'global_planner/reachability_check_enabled' are set. "
                     "Using the new parameter and ignoring the legacy one.");
    }
  }

  // Level B: Local Planner
  local_planner_ = std::make_unique<LocalPlanner>(robot_state_, jacobian_solver_);
  
  double att_gain, max_lin_vel, max_ang_vel;
  pnh_.param("local_planner/attractive_gain", att_gain, 1.0);
  pnh_.param("local_planner/max_linear_velocity", max_lin_vel, 0.5);
  pnh_.param("local_planner/max_angular_velocity", max_ang_vel, 1.0);
  
  // Scaling factors
  pnh_.param("local_planner/velocity_scale_factor", velocity_scale_factor_, 1.0);
  pnh_.param("acceleration_scale_factor", acceleration_scale_factor_, 1.0);
  velocity_scale_factor_ = std::clamp(velocity_scale_factor_, 0.0, 1.0);
  acceleration_scale_factor_ = std::clamp(acceleration_scale_factor_, 0.0, 1.0);

  local_planner_->setAttractiveGain(att_gain);
  // Apply scaling to initial limits
  local_planner_->setMaxLinearVelocity(max_lin_vel * velocity_scale_factor_);
  local_planner_->setMaxAngularVelocity(max_ang_vel * velocity_scale_factor_);

  double freeze_lin, freeze_ang;
  pnh_.param("local_planner/integration_freeze_linear", freeze_lin, 1e-4);
  pnh_.param("local_planner/integration_freeze_angular", freeze_ang, 1e-3);
  local_planner_->setIntegrationFreezeThresholds(freeze_lin, freeze_ang);

  // Repulsive parameters
  double rep_obs_gain, rep_link_gain, influence_dist, min_safe_dist;
  pnh_.param("local_planner/repulsive_enabled", repulsive_enabled_, false);
  pnh_.param("local_planner/repulsive_obstacle_gain", rep_obs_gain, 0.0);
  pnh_.param("local_planner/repulsive_link_gain", rep_link_gain, 0.0);
  pnh_.param("local_planner/influence_distance", influence_dist, 0.5);
  pnh_.param("local_planner/min_safe_distance", min_safe_dist, 0.05);

  // Repulsive profile + smoothing (3.1, 3.3, 3.4)
  std::string rep_profile_str;
  pnh_.param<std::string>("local_planner/repulsive_velocity_profile", rep_profile_str, "QUADRATIC");
  const RepulsiveVelocityMode rep_mode = repulsiveModeFromString(rep_profile_str);
  local_planner_->setRepulsiveMode(rep_mode);

  double rep_tau_rise, rep_tau_fall;
  double rep_max_acc_rise, rep_max_acc_fall;
  pnh_.param("local_planner/repulsive_filter_tau_rise", rep_tau_rise, 0.0);
  pnh_.param("local_planner/repulsive_filter_tau_fall", rep_tau_fall, 0.0);
  pnh_.param("local_planner/repulsive_max_acc_rise", rep_max_acc_rise, 0.0);
  pnh_.param("local_planner/repulsive_max_acc_fall", rep_max_acc_fall, 0.0);
  local_planner_->setRepulsiveVelocityFilterTaus(rep_tau_rise, rep_tau_fall);
  local_planner_->setRepulsiveVelocityMaxAccelerations(rep_max_acc_rise, rep_max_acc_fall);

  local_planner_->setRepulsiveObstacleGain(repulsive_enabled_ ? rep_obs_gain : 0.0);
  local_planner_->setRepulsiveLinkGain(repulsive_enabled_ ? rep_link_gain : 0.0);
  local_planner_->setInfluenceDistance(influence_dist);
  local_planner_->setMinSafeDistance(min_safe_dist);

  // Soft Leash
  bool leash_enabled;
  double leash_start, leash_stop, leash_reset;
  pnh_.param("local_planner/leash_enabled", leash_enabled, true);
  pnh_.param("local_planner/leash_start_distance", leash_start, 0.10);
  pnh_.param("local_planner/leash_stop_distance", leash_stop, 0.20);
  pnh_.param("local_planner/leash_reset_threshold", leash_reset, 0.50);

  local_planner_->setVirtualTargetLeashEnabled(leash_enabled);
  local_planner_->setVirtualTargetLeashParams(leash_start, leash_stop, leash_reset);

  // Level C: Motion Generator (Velocity Filter)
  VelocityLimits linear_limits, angular_limits;
  // Reuse local planner velocity caps by default; optional extra cap via cartesian_*_max_velocity.
  double planner_lin_vel_scaled = max_lin_vel * velocity_scale_factor_;
  double planner_ang_vel_scaled = max_ang_vel * velocity_scale_factor_;

  // Optional filter caps (base values, scaled by velocity_scale_factor_)
  double filter_lin_vel_base = 0.0;
  if (pnh_.getParam("cartesian_linear_max_velocity", filter_lin_vel_base))
  {
    has_cartesian_linear_max_velocity_override_ = true;
    cartesian_linear_max_velocity_override_base_ = std::max(0.0, filter_lin_vel_base);
  }
  else
  {
    has_cartesian_linear_max_velocity_override_ = false;
  }

  double filter_ang_vel_base = 0.0;
  if (pnh_.getParam("cartesian_angular_max_velocity", filter_ang_vel_base))
  {
    has_cartesian_angular_max_velocity_override_ = true;
    cartesian_angular_max_velocity_override_base_ = std::max(0.0, filter_ang_vel_base);
  }
  else
  {
    has_cartesian_angular_max_velocity_override_ = false;
  }

  const double filter_lin_vel_scaled =
      has_cartesian_linear_max_velocity_override_
          ? (cartesian_linear_max_velocity_override_base_ * velocity_scale_factor_)
          : planner_lin_vel_scaled;
  const double filter_ang_vel_scaled =
      has_cartesian_angular_max_velocity_override_
          ? (cartesian_angular_max_velocity_override_base_ * velocity_scale_factor_)
          : planner_ang_vel_scaled;

  // Effective filter max velocities: additional cap (never higher than planner cap)
  linear_limits.max_velocity = std::min(planner_lin_vel_scaled, filter_lin_vel_scaled);
  pnh_.param("cartesian_linear_max_acceleration", linear_limits.max_acceleration, 1.0);
  pnh_.param("cartesian_linear_max_jerk", linear_limits.max_jerk, 10.0);
  angular_limits.max_velocity = std::min(planner_ang_vel_scaled, filter_ang_vel_scaled);
  pnh_.param("cartesian_angular_max_acceleration", angular_limits.max_acceleration, 2.0);
  pnh_.param("cartesian_angular_max_jerk", angular_limits.max_jerk, 20.0);
  
  // Apply scaling
  linear_limits.max_acceleration *= acceleration_scale_factor_;
  linear_limits.max_jerk *= acceleration_scale_factor_;
  angular_limits.max_acceleration *= acceleration_scale_factor_;
  angular_limits.max_jerk *= acceleration_scale_factor_;

  velocity_filter_ = std::make_unique<CartesianVelocityFilter>(linear_limits, angular_limits);
  
  bool filter_enabled;
  double filter_tau;
  pnh_.param("cartesian_filter_enabled", filter_enabled, true);
  pnh_.param("cartesian_filter_tau", filter_tau, 0.1);
  pnh_.param("reset_filter_on_target_change", reset_filter_on_target_change_, true);
  
  velocity_filter_->setEnabled(filter_enabled);
  velocity_filter_->setTimeConstant(filter_tau);

  // Velocity filter dt policy (fixed internal dt with accumulator/substeps)
  const double default_dt_nominal = (control_rate_ > 1e-6) ? (1.0 / control_rate_) : 0.01;
  double cart_dt_nominal;
  double cart_min_dt;
  double cart_max_dt;
  int cart_max_substeps;
  double cart_reset_dt_threshold;
  std::string cart_large_dt_policy;
  bool cart_uniform_scaling;

  pnh_.param("cartesian_velocity_filter/dt_nominal", cart_dt_nominal, default_dt_nominal);
  pnh_.param("cartesian_velocity_filter/min_dt", cart_min_dt, 1e-5);
  pnh_.param("cartesian_velocity_filter/max_dt", cart_max_dt, 0.05);
  pnh_.param("cartesian_velocity_filter/max_substeps", cart_max_substeps, 10);
  pnh_.param("cartesian_velocity_filter/reset_dt_threshold", cart_reset_dt_threshold, 0.25);
  pnh_.param<std::string>("cartesian_velocity_filter/large_dt_policy", cart_large_dt_policy, "hold_last");
  pnh_.param("cartesian_velocity_filter/uniform_scaling_enabled", cart_uniform_scaling, true);

  velocity_filter_->setDtClamp(cart_min_dt, cart_max_dt);
  velocity_filter_->setMaxSubsteps(cart_max_substeps);
  velocity_filter_->setResetDtThreshold(cart_reset_dt_threshold);
  velocity_filter_->setDtNominal(cart_dt_nominal);
  velocity_filter_->setUniformScalingEnabled(cart_uniform_scaling);

  if (cart_large_dt_policy == "reset_to_zero")
  {
    velocity_filter_->setLargeDtPolicy(CartesianVelocityFilter::LargeDtPolicy::RESET_TO_ZERO);
  }
  else if (cart_large_dt_policy == "reset_to_desired")
  {
    velocity_filter_->setLargeDtPolicy(CartesianVelocityFilter::LargeDtPolicy::RESET_TO_DESIRED);
  }
  else
  {
    velocity_filter_->setLargeDtPolicy(CartesianVelocityFilter::LargeDtPolicy::HOLD_LAST);
  }

  // Pose tracking (target_raw -> target_filtered) for Level C input generation
  // Default tracking tau should be significantly larger than filter tau for stability (e.g. 4x)
  // to ensure the loop is not underdamped.
  double default_tracking_tau = std::max(0.2, 4.0 * filter_tau);

  pnh_.param("pose_tracking/enabled", pose_tracking_.enabled, true);
  pnh_.param("pose_tracking/tau_linear", pose_tracking_.tau_linear, default_tracking_tau);
  pnh_.param("pose_tracking/tau_angular", pose_tracking_.tau_angular, default_tracking_tau);
  pnh_.param("pose_tracking/k_linear", pose_tracking_.k_linear, 1.0);
  pnh_.param("pose_tracking/k_angular", pose_tracking_.k_angular, 1.0);
  pnh_.param("pose_tracking/max_linear_velocity", pose_tracking_.max_linear_velocity_base, max_lin_vel);
  pnh_.param("pose_tracking/max_angular_velocity", pose_tracking_.max_angular_velocity_base, max_ang_vel);

  pose_tracking_.tau_linear = std::clamp(pose_tracking_.tau_linear, 0.01, 5.0);
  pose_tracking_.tau_angular = std::clamp(pose_tracking_.tau_angular, 0.01, 5.0);
  pose_tracking_.k_linear = std::clamp(pose_tracking_.k_linear, 0.0, 50.0);
  pose_tracking_.k_angular = std::clamp(pose_tracking_.k_angular, 0.0, 50.0);
  pose_tracking_.max_linear_velocity_base = std::max(0.0, pose_tracking_.max_linear_velocity_base);
  pose_tracking_.max_angular_velocity_base = std::max(0.0, pose_tracking_.max_angular_velocity_base);

  // Level D: PID Controllers
  PIDConfig pos_pid_config, ori_pid_config;
  
  pnh_.param("pid_controller/position/kp", pos_pid_config.kp, 2.0);
  pnh_.param("pid_controller/position/ki", pos_pid_config.ki, 0.1);
  pnh_.param("pid_controller/position/kd", pos_pid_config.kd, 0.05);
  pnh_.param("pid_controller/position/kff", pos_pid_config.kff, 1.0);
  pnh_.param("pid_controller/position/output_limit", pos_pid_config.output_limit, 0.5);
  // Apply scaling
  pos_pid_config.output_limit *= velocity_scale_factor_;
  pnh_.param("pid_controller/position/derivative_filter_tau", pos_pid_config.derivative_filter_tau, 0.02);
  
  pnh_.param("pid_controller/orientation/kp", ori_pid_config.kp, 1.5);
  pnh_.param("pid_controller/orientation/ki", ori_pid_config.ki, 0.05);
  pnh_.param("pid_controller/orientation/kd", ori_pid_config.kd, 0.02);
  pnh_.param("pid_controller/orientation/kff", ori_pid_config.kff, 1.0);
  pnh_.param("pid_controller/orientation/output_limit", ori_pid_config.output_limit, 1.0);
  // Apply scaling
  ori_pid_config.output_limit *= velocity_scale_factor_;
  pnh_.param("pid_controller/orientation/derivative_filter_tau", ori_pid_config.derivative_filter_tau, 0.02);
  
  pid_position_ = std::make_unique<PIDController>(3, pos_pid_config);
  pid_orientation_ = std::make_unique<PIDController>(3, ori_pid_config);

  // JointVelocityFilter (joint-space smoothness stage, before safety limiter)
  joint_velocity_filter_ = std::make_unique<JointVelocityFilter>(num_joints);
  {
    auto loadJointLimitVec = [&](const std::string& key, double default_scalar) -> Eigen::VectorXd {
      Eigen::VectorXd out = Eigen::VectorXd::Constant(num_joints, std::abs(default_scalar));
      std::vector<double> values;
      if (pnh_.getParam(key, values))
      {
        if (values.size() == static_cast<std::size_t>(num_joints))
        {
          for (std::size_t i = 0; i < values.size(); ++i) out[static_cast<int>(i)] = std::abs(values[i]);
        }
        else if (values.size() == 2 && num_joints == 6)
        {
          // Convenience: [first3, last3]
          const double a = std::abs(values[0]);
          const double b = std::abs(values[1]);
          out << a, a, a, b, b, b;
        }
        else
        {
          ROS_WARN_STREAM_NAMED("cartesian_velocity_controller",
                                "Param '" << key << "' has size " << values.size()
                                          << " (expected " << num_joints
                                          << " or 2 for 3+3 on 6-DOF). Using scalar default.");
        }
      }
      else
      {
        double scalar;
        pnh_.param(key, scalar, default_scalar);
        out = Eigen::VectorXd::Constant(num_joints, std::abs(scalar));
      }
      return out;
    };

    bool joint_filter_enabled;
    double joint_filter_tau;
    bool joint_filter_uniform_scaling;
    pnh_.param("joint_velocity_filter/enabled", joint_filter_enabled, false);
    pnh_.param("joint_velocity_filter/tau", joint_filter_tau, 0.1);
    pnh_.param("joint_velocity_filter/uniform_scaling_enabled", joint_filter_uniform_scaling, false);

    Eigen::VectorXd j_max_vel = loadJointLimitVec("joint_velocity_filter/max_joint_velocity", 1.0);
    Eigen::VectorXd j_max_acc = loadJointLimitVec("joint_velocity_filter/max_joint_acceleration", 5.0);
    Eigen::VectorXd j_max_jerk = loadJointLimitVec("joint_velocity_filter/max_joint_jerk", 50.0);

    joint_velocity_filter_->setEnabled(joint_filter_enabled);
    joint_velocity_filter_->setTimeConstant(joint_filter_tau);
    joint_velocity_filter_->setUniformScalingEnabled(joint_filter_uniform_scaling);
    joint_velocity_filter_->setLimits(j_max_vel, j_max_acc, j_max_jerk);

    // dt policy (same style as cartesian filter)
    double joint_dt_nominal;
    double joint_min_dt;
    double joint_max_dt;
    int joint_max_substeps;
    double joint_reset_dt_threshold;
    std::string joint_large_dt_policy;

    pnh_.param("joint_velocity_filter/dt_nominal", joint_dt_nominal, default_dt_nominal);
    pnh_.param("joint_velocity_filter/min_dt", joint_min_dt, 1e-5);
    pnh_.param("joint_velocity_filter/max_dt", joint_max_dt, 0.05);
    pnh_.param("joint_velocity_filter/max_substeps", joint_max_substeps, 10);
    pnh_.param("joint_velocity_filter/reset_dt_threshold", joint_reset_dt_threshold, 0.25);
    pnh_.param<std::string>("joint_velocity_filter/large_dt_policy", joint_large_dt_policy, "hold_last");

    joint_velocity_filter_->setDtClamp(joint_min_dt, joint_max_dt);
    joint_velocity_filter_->setMaxSubsteps(joint_max_substeps);
    joint_velocity_filter_->setResetDtThreshold(joint_reset_dt_threshold);
    joint_velocity_filter_->setDtNominal(joint_dt_nominal);

    if (joint_large_dt_policy == "reset_to_zero")
    {
      joint_velocity_filter_->setLargeDtPolicy(JointVelocityFilter::LargeDtPolicy::RESET_TO_ZERO);
    }
    else if (joint_large_dt_policy == "reset_to_desired")
    {
      joint_velocity_filter_->setLargeDtPolicy(JointVelocityFilter::LargeDtPolicy::RESET_TO_DESIRED);
    }
    else
    {
      joint_velocity_filter_->setLargeDtPolicy(JointVelocityFilter::LargeDtPolicy::HOLD_LAST);
    }
  }

  // Requirement: if runtime joint guard or elbow injection are enabled, we need jerk/acc-safe smoothing.
  const bool requires_joint_filter =
      (joint_position_guard_ && joint_position_guard_->isEnabled()) || elbow_injection_.enabled;
  if (requires_joint_filter && joint_velocity_filter_ && !joint_velocity_filter_->isEnabled())
  {
    joint_velocity_filter_->setEnabled(true);
    joint_velocity_filter_forced_on_ = true;
    ROS_WARN_NAMED("cartesian_velocity_controller",
                   "joint_velocity_filter was disabled but is required by controller joint guard / elbow injection. Forcing it ON.");
  }

  // Final: Joint Safety Limiter
  double max_joint_vel, max_joint_acc;
  pnh_.param("joint_safety_limiter/max_joint_velocity", max_joint_vel, 1.0);
  pnh_.param("joint_safety_limiter/max_joint_acceleration", max_joint_acc, 5.0);
  
  Eigen::VectorXd max_vels = Eigen::VectorXd::Constant(num_joints, max_joint_vel);
  Eigen::VectorXd max_accs = Eigen::VectorXd::Constant(num_joints, max_joint_acc);
  safety_limiter_ = std::make_unique<JointSafetyLimiter>(max_vels, max_accs);
  
  bool safety_enabled;
  pnh_.param("joint_safety_limiter/enabled", safety_enabled, true);

  bool vel_limit_en, acc_limit_en;
  pnh_.param("joint_safety_limiter/velocity_limiting_enabled", vel_limit_en, true);
  pnh_.param("joint_safety_limiter/acceleration_limiting_enabled", acc_limit_en, true);
  safety_limiter_->setVelocityLimitingEnabled(safety_enabled ? vel_limit_en : false);
  safety_limiter_->setAccelerationLimitingEnabled(safety_enabled ? acc_limit_en : false);

  // Marker Publisher
  MarkerPublisherConfig marker_config;
  marker_config.global_frame = global_frame_;
  pnh_.param("marker_publisher/arrow_scale", marker_config.arrow_scale, 1.0);
  pnh_.param<std::string>("marker_publisher/ns_prefix", marker_config.ns_prefix, std::string(""));
  // Opzione B: marker per-istanza nel namespace privato (~*_markers)
  marker_publisher_ = std::make_unique<MarkerPublisher>(pnh_, marker_config);

  // Feedback Publisher
  feedback_publisher_ = std::make_unique<FeedbackPublisher>(pnh_, global_frame_);

  if (map3d_enabled_)
  {
    // Map3D (local distance field) - runs asynchronously
    map3d_manager_ = std::make_shared<cartesian_velocity_controller::map3d::Map3DManager>(nh_, pnh_);
    map3d_manager_->start();

    // Repulsion Data Manager (depends on Map3D)
    repulsion_manager_ = std::make_unique<RepulsionDataManager>(nh_, robot_state_);
    repulsion_manager_->setMap3DManager(map3d_manager_);
    repulsion_manager_->loadConfig(pnh_);
  }
  else
  {
    map3d_manager_.reset();
    repulsion_manager_.reset();

    if (repulsive_enabled_)
    {
      ROS_WARN_NAMED("cartesian_velocity_controller",
                     "map3d/enabled is false: disabling repulsive velocity (Map3D not available).");
      repulsive_enabled_ = false;
    }

    // Be explicit: ensure the local planner doesn't keep non-zero repulsive gains.
    if (local_planner_)
    {
      local_planner_->setRepulsiveObstacleGain(0.0);
      local_planner_->setRepulsiveLinkGain(0.0);
    }
  }

  // Initialize command storage
  last_command_.data.assign(num_joints, 0.0);
  previous_joint_velocity_ = Eigen::VectorXd::Zero(num_joints);
  last_command_stamp_ = ros::Time::now();

  ROS_INFO_STREAM_NAMED("cartesian_velocity_controller",
                        "Pipeline components initialized for " << num_joints << " joints");
}

void CartesianVelocityController::setupRosInterfaces()
{
  velocity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(velocity_command_topic_, 1);
  joint_state_sub_ = nh_.subscribe(joint_state_topic_, 50,
                                   &CartesianVelocityController::jointStateCallback, this);
  // Opzione B: input per-istanza nel namespace privato (~target_pose)
  target_pose_sub_ = pnh_.subscribe("target_pose", 1,
                                   &CartesianVelocityController::targetPoseCallback, this);
  target_state_sub_ = pnh_.subscribe("target_state", 1,
                                     &CartesianVelocityController::targetStateCallback, this);

  // Debug service for frame verification
  get_frame_info_server_ = pnh_.advertiseService("get_frame_info",
                                                  &CartesianVelocityController::getFrameInfoCallback, this);
  // Debug/utility service: retrieve the current Jacobian (per-node namespace via private NH)
  get_jacobian_server_ = pnh_.advertiseService("get_jacobian",
                                               &CartesianVelocityController::getJacobianCallback, this);
  validate_poses_server_ = pnh_.advertiseService("validate_poses",
                                                 &CartesianVelocityController::validatePosesCallback, this);

  const std::string cm_ns = controller_manager_ns_.empty() ? "controller_manager" : controller_manager_ns_;
  const std::string switch_srv = ros::names::append(cm_ns, "switch_controller");
  const std::string list_srv = ros::names::append(cm_ns, "list_controllers");

  switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(switch_srv);
  list_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(list_srv);
}

void CartesianVelocityController::captureControllerState()
{
  if (controller_state_captured_) return;

  start_controller_initially_running_ = false;
  stop_controller_initially_running_ = false;

  if (start_controller_name_.empty() && stop_controller_name_.empty())
  {
    controller_state_captured_ = true;
    return;
  }

  controller_manager_msgs::ListControllers srv;
  if (list_client_.exists() && list_client_.call(srv))
  {
    for (const auto& ctrl : srv.response.controller)
    {
      if (ctrl.name == start_controller_name_)
        start_controller_initially_running_ = (ctrl.state == "running");
      else if (ctrl.name == stop_controller_name_)
        stop_controller_initially_running_ = (ctrl.state == "running");
    }
  }
  else
  {
    stop_controller_initially_running_ = true;
  }

  controller_state_captured_ = true;
}

void CartesianVelocityController::captureInitialPose()
{
  if (!robot_state_ || !robot_state_->isReady()) return;

  Eigen::Isometry3d tcp_offset;
  {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    tcp_offset = tcp_offset_;
  }

  Eigen::Isometry3d current_tcp_pose;
  if (robot_state_->computeTcpPose(tcp_offset, current_tcp_pose))
  {
    if (global_planner_)
    {
      global_planner_->clearWaypoints();
      if (startup_behavior_ == StartupBehavior::HOLD_INITIAL_POSE)
      {
        // Default/current behavior: hold initial pose until user sets a target.
        global_planner_->addWaypoint(current_tcp_pose);
        idle_until_target_ = false;
      }
      else
      {
        // New behavior: don't seed an initial waypoint. The loop will publish zero
        // velocity and stay idle until the user provides a target.
        idle_until_target_ = true;
      }

      // No user target on startup
      has_target_ = false;

      // Clear IK caches (no user target -> injection gate should be OPEN)
      has_last_target_ik_solution_ = false;
      waypoint_ik_solutions_.clear();
      waypoint_ik_solutions_valid_.clear();
      
      if (startup_behavior_ == StartupBehavior::HOLD_INITIAL_POSE)
      {
        ROS_INFO_NAMED("cartesian_velocity_controller",
                       "Initial pose captured (startup_behavior=hold_current_pose): [%.3f, %.3f, %.3f]",
                       current_tcp_pose.translation().x(),
                       current_tcp_pose.translation().y(),
                       current_tcp_pose.translation().z());
      }
      else
      {
        ROS_INFO_NAMED("cartesian_velocity_controller",
                       "Initial pose captured (startup_behavior=zero_velocity). Idling until target is set: [%.3f, %.3f, %.3f]",
                       current_tcp_pose.translation().x(),
                       current_tcp_pose.translation().y(),
                       current_tcp_pose.translation().z());
      }
    }
    
    // Initialize local planner and velocity filter to current pose
    if (local_planner_)
    {
      local_planner_->resetToPosition(current_tcp_pose);
    }
    if (velocity_filter_)
    {
      velocity_filter_->resetToPosition(current_tcp_pose);
    }

    // Initialize pose filter state to avoid startup transients
    filtered_tcp_pose_ = current_tcp_pose;
    has_filtered_tcp_pose_ = true;
  }
}

// ============================================================================
// Dynamic Reconfigure
// ============================================================================

void CartesianVelocityController::setupDynamicReconfigure()
{
  // Sync parameters to parameter server to ensure dynamic reconfigure picks up the correct initial values
  // This handles the case where parameters are loaded from namespaced paths (e.g. local_planner/...)
  // but dynamic reconfigure expects them in the node's namespace (e.g. ~attractive_gain)

  // Store reconstructed base values for config initialization
  double base_max_lin_vel = 0.5;
  double base_max_ang_vel = 1.0;
  double base_lin_acc = 1.0;
  double base_lin_jerk = 10.0;
  double base_ang_acc = 2.0;
  double base_ang_jerk = 20.0;

  // Global planner
  if (global_planner_)
  {
    pnh_.setParam("waypoint_switch_distance", global_planner_->getWaypointSwitchDistance());
    pnh_.setParam("orientation_switch_threshold", global_planner_->getOrientationSwitchThreshold());
  }

  // Local planner
  if (local_planner_)
  {
    pnh_.setParam("attractive_gain", local_planner_->getAttractiveGain());
    
    // Reconstruct base values from scaled values
    double max_lin = local_planner_->getMaxLinearVelocity();
    double max_ang = local_planner_->getMaxAngularVelocity();
    if (velocity_scale_factor_ > 1e-6)
    {
      max_lin /= velocity_scale_factor_;
      max_ang /= velocity_scale_factor_;
    }
    base_max_lin_vel = max_lin;
    base_max_ang_vel = max_ang;
    
    pnh_.setParam("max_linear_velocity", max_lin);
    pnh_.setParam("max_angular_velocity", max_ang);
    pnh_.setParam("velocity_scale_factor", velocity_scale_factor_);

    pnh_.setParam("repulsive_enabled", repulsive_enabled_);
    pnh_.setParam("repulsive_obstacle_gain", local_planner_->getRepulsiveObstacleGain());
    pnh_.setParam("repulsive_link_gain", local_planner_->getRepulsiveLinkGain());
    pnh_.setParam("influence_distance", local_planner_->getInfluenceDistance());
    pnh_.setParam("min_safe_distance", local_planner_->getMinSafeDistance());

    // Repulsive profile + smoothing
    pnh_.setParam("repulsive_velocity_profile", repulsiveModeToInt(local_planner_->getRepulsiveMode()));
    double tau_rise = 0.0, tau_fall = 0.0;
    double max_acc_rise = 0.0, max_acc_fall = 0.0;
    local_planner_->getRepulsiveVelocityFilterTaus(tau_rise, tau_fall);
    local_planner_->getRepulsiveVelocityMaxAccelerations(max_acc_rise, max_acc_fall);
    pnh_.setParam("repulsive_filter_tau_rise", tau_rise);
    pnh_.setParam("repulsive_filter_tau_fall", tau_fall);
    pnh_.setParam("repulsive_max_acc_rise", max_acc_rise);
    pnh_.setParam("repulsive_max_acc_fall", max_acc_fall);
  }

  // Velocity filter
  if (velocity_filter_)
  {
    auto lin_limits = velocity_filter_->getLinearLimits();
    auto ang_limits = velocity_filter_->getAngularLimits();
    
    double lin_acc = lin_limits.max_acceleration;
    double lin_jerk = lin_limits.max_jerk;
    double ang_acc = ang_limits.max_acceleration;
    double ang_jerk = ang_limits.max_jerk;

    if (acceleration_scale_factor_ > 1e-6)
    {
      lin_acc /= acceleration_scale_factor_;
      lin_jerk /= acceleration_scale_factor_;
      ang_acc /= acceleration_scale_factor_;
      ang_jerk /= acceleration_scale_factor_;
    }
    base_lin_acc = lin_acc;
    base_lin_jerk = lin_jerk;
    base_ang_acc = ang_acc;
    base_ang_jerk = ang_jerk;

    pnh_.setParam("cartesian_filter_tau", velocity_filter_->getTimeConstant());
    pnh_.setParam("cartesian_linear_max_acceleration", lin_acc);
    pnh_.setParam("cartesian_linear_max_jerk", lin_jerk);
    pnh_.setParam("cartesian_angular_max_acceleration", ang_acc);
    pnh_.setParam("cartesian_angular_max_jerk", ang_jerk);
    pnh_.setParam("acceleration_scale_factor", acceleration_scale_factor_);
    pnh_.setParam("cartesian_filter_uniform_scaling_enabled", velocity_filter_->isUniformScalingEnabled());
  }

  // Joint velocity filter (pre-safety)
  if (joint_velocity_filter_)
  {
    pnh_.setParam("joint_filter_enabled", joint_velocity_filter_->isEnabled());
    pnh_.setParam("joint_filter_tau", joint_velocity_filter_->getTimeConstant());
    pnh_.setParam("joint_filter_uniform_scaling_enabled", joint_velocity_filter_->isUniformScalingEnabled());

    const Eigen::VectorXd max_acc = joint_velocity_filter_->getMaxAcceleration();
    const Eigen::VectorXd max_jerk = joint_velocity_filter_->getMaxJerk();

    double acc_arm = (max_acc.size() > 0) ? max_acc[0] : 0.0;
    double acc_wrist = (max_acc.size() >= 6) ? max_acc[3] : acc_arm;
    double jerk_arm = (max_jerk.size() > 0) ? max_jerk[0] : 0.0;
    double jerk_wrist = (max_jerk.size() >= 6) ? max_jerk[3] : jerk_arm;

    pnh_.setParam("joint_filter_max_acceleration_arm", acc_arm);
    pnh_.setParam("joint_filter_max_acceleration_wrist", acc_wrist);
    pnh_.setParam("joint_filter_max_jerk_arm", jerk_arm);
    pnh_.setParam("joint_filter_max_jerk_wrist", jerk_wrist);
  }

  // Filter reset option
  pnh_.setParam("reset_filter_on_target_change", reset_filter_on_target_change_);

  // PID
  if (pid_position_)
  {
    PIDConfig pos_cfg = pid_position_->getConfig();
    pnh_.setParam("pid_pos_kp", pos_cfg.kp);
    pnh_.setParam("pid_pos_ki", pos_cfg.ki);
    pnh_.setParam("pid_pos_kd", pos_cfg.kd);
    pnh_.setParam("pid_pos_kff", pos_cfg.kff);
    pnh_.setParam("pid_pos_derivative_filter_tau", pos_cfg.derivative_filter_tau);
    pnh_.setParam("pid_pos_deadband", position_deadband_);
  }
  
  if (pid_orientation_)
  {
    PIDConfig ori_cfg = pid_orientation_->getConfig();
    pnh_.setParam("pid_ori_kp", ori_cfg.kp);
    pnh_.setParam("pid_ori_ki", ori_cfg.ki);
    pnh_.setParam("pid_ori_kd", ori_cfg.kd);
    pnh_.setParam("pid_ori_kff", ori_cfg.kff);
    pnh_.setParam("pid_ori_derivative_filter_tau", ori_cfg.derivative_filter_tau);
    pnh_.setParam("pid_ori_deadband", orientation_deadband_);
  }

  // Joint Safety Limiter
  if (safety_limiter_)
  {
    Eigen::VectorXd max_vels = safety_limiter_->getJointVelocityLimits();
    Eigen::VectorXd max_accs = safety_limiter_->getJointAccelerationLimits();
    double max_v = (max_vels.size() > 0) ? max_vels(0) : 1.0;
    double max_a = (max_accs.size() > 0) ? max_accs(0) : 5.0;
    pnh_.setParam("max_joint_velocity", max_v);
    pnh_.setParam("max_joint_acceleration", max_a);
  }

  // Jacobian Solver
  if (jacobian_solver_)
  {
    JacobianSolverConfig jac_cfg = jacobian_solver_->getConfig();
    pnh_.setParam("singularity_threshold", jac_cfg.singularity_threshold);
    pnh_.setParam("max_damping", jac_cfg.max_damping);
  }

  // Singularity Avoidance
  if (weight_manager_)
  {
    auto strategy = weight_manager_->getElbowStrategy();
    if (strategy)
    {
      pnh_.setParam("singularity_avoidance_buffer", strategy->getSingularityBuffer());
      pnh_.setParam("singularity_avoidance_weight", strategy->getMaxWeightPenalty());
    }
  }

  // POI
  if (repulsion_manager_)
  {
    auto tcp_cfg = repulsion_manager_->getPointConfig("tcp");
    pnh_.setParam("poi_tcp_enabled", tcp_cfg.enabled);
    pnh_.setParam("poi_tcp_weight", tcp_cfg.weight);
    pnh_.setParam("poi_tcp_radius", tcp_cfg.radius);
    
    auto elbow_cfg = repulsion_manager_->getPointConfig("elbow");
    pnh_.setParam("poi_elbow_enabled", elbow_cfg.enabled);
    pnh_.setParam("poi_elbow_weight", elbow_cfg.weight);
    pnh_.setParam("poi_elbow_radius", elbow_cfg.radius);
    
    auto wrist_cfg = repulsion_manager_->getPointConfig("wrist");
    pnh_.setParam("poi_wrist_enabled", wrist_cfg.enabled);
    pnh_.setParam("poi_wrist_weight", wrist_cfg.weight);
    pnh_.setParam("poi_wrist_radius", wrist_cfg.radius);
    
    auto forearm_cfg = repulsion_manager_->getPointConfig("forearm_mid");
    pnh_.setParam("poi_forearm_enabled", forearm_cfg.enabled);
    pnh_.setParam("poi_forearm_weight", forearm_cfg.weight);
    pnh_.setParam("poi_forearm_radius", forearm_cfg.radius);

    // Repulsion map smoothing/prediction
    pnh_.setParam("gradient_filter_alpha", repulsion_manager_->getGradientFilterAlpha());
    pnh_.setParam("poi_predict_enable", repulsion_manager_->getPoiPredictEnabled());
    pnh_.setParam("poi_predict_horizon", repulsion_manager_->getPoiPredictHorizon());
    pnh_.setParam("poi_velocity_filter_tau", repulsion_manager_->getPoiVelocityFilterTau());
    pnh_.setParam("poi_velocity_max", repulsion_manager_->getPoiVelocityMax());
    pnh_.setParam("poi_predict_conservative_min_distance", repulsion_manager_->getPoiPredictConservativeMinDistance());
  }

  dynamic_reconfigure_mutex_ = std::make_shared<DynReconfMutex>();
  dynamic_reconfigure_server_ =
      std::make_unique<DynReconfServer>(*dynamic_reconfigure_mutex_, pnh_);

  ControllerTuningConfig config;
  config.pose_filter_alpha = pose_filter_alpha_;
  config.velocity_scale_factor = velocity_scale_factor_;
  config.acceleration_scale_factor = acceleration_scale_factor_;
  config.reset_filter_on_target_change = reset_filter_on_target_change_;

  if (velocity_filter_)
  {
    config.cartesian_filter_tau = velocity_filter_->getTimeConstant();
    config.cartesian_linear_max_acceleration = base_lin_acc;
    config.cartesian_linear_max_jerk = base_lin_jerk;
    config.cartesian_angular_max_acceleration = base_ang_acc;
    config.cartesian_angular_max_jerk = base_ang_jerk;
    config.cartesian_filter_uniform_scaling_enabled = velocity_filter_->isUniformScalingEnabled();
  }

  if (joint_velocity_filter_)
  {
    config.joint_filter_enabled = joint_velocity_filter_->isEnabled();
    config.joint_filter_tau = joint_velocity_filter_->getTimeConstant();
    config.joint_filter_uniform_scaling_enabled = joint_velocity_filter_->isUniformScalingEnabled();

    const Eigen::VectorXd max_acc = joint_velocity_filter_->getMaxAcceleration();
    const Eigen::VectorXd max_jerk = joint_velocity_filter_->getMaxJerk();
    config.joint_filter_max_acceleration_arm = (max_acc.size() > 0) ? max_acc[0] : 0.0;
    config.joint_filter_max_acceleration_wrist = (max_acc.size() >= 6) ? max_acc[3] : config.joint_filter_max_acceleration_arm;
    config.joint_filter_max_jerk_arm = (max_jerk.size() > 0) ? max_jerk[0] : 0.0;
    config.joint_filter_max_jerk_wrist = (max_jerk.size() >= 6) ? max_jerk[3] : config.joint_filter_max_jerk_arm;
  }

  if (global_planner_)
  {
    config.waypoint_switch_distance = global_planner_->getWaypointSwitchDistance();
    config.orientation_switch_threshold = global_planner_->getOrientationSwitchThreshold();
  }

  if (local_planner_)
  {
    config.attractive_gain = local_planner_->getAttractiveGain();
    config.max_linear_velocity = base_max_lin_vel;
    config.max_angular_velocity = base_max_ang_vel;
    config.repulsive_enabled = repulsive_enabled_;
    config.repulsive_obstacle_gain = local_planner_->getRepulsiveObstacleGain();
    config.repulsive_link_gain = local_planner_->getRepulsiveLinkGain();
    config.influence_distance = local_planner_->getInfluenceDistance();
    config.min_safe_distance = local_planner_->getMinSafeDistance();

    // New repulsive tuning
    config.repulsive_velocity_profile = repulsiveModeToInt(local_planner_->getRepulsiveMode());
    double tau_rise = 0.0, tau_fall = 0.0;
    double max_acc_rise = 0.0, max_acc_fall = 0.0;
    local_planner_->getRepulsiveVelocityFilterTaus(tau_rise, tau_fall);
    local_planner_->getRepulsiveVelocityMaxAccelerations(max_acc_rise, max_acc_fall);
    config.repulsive_filter_tau_rise = tau_rise;
    config.repulsive_filter_tau_fall = tau_fall;
    config.repulsive_max_acc_rise = max_acc_rise;
    config.repulsive_max_acc_fall = max_acc_fall;
    
    config.leash_enabled = local_planner_->getVirtualTargetLeashEnabled();
    double start, stop, reset;
    local_planner_->getVirtualTargetLeashParams(start, stop, reset);
    config.leash_start_distance = start;
    config.leash_stop_distance = stop;
    config.leash_reset_threshold = reset;
  }

  if (pid_position_)
  {
    PIDConfig pos_cfg = pid_position_->getConfig();
    config.pid_pos_kp = pos_cfg.kp;
    config.pid_pos_ki = pos_cfg.ki;
    config.pid_pos_kd = pos_cfg.kd;
    config.pid_pos_kff = pos_cfg.kff;
    config.pid_pos_derivative_filter_tau = pos_cfg.derivative_filter_tau;
    config.pid_pos_deadband = position_deadband_;
  }

  if (pid_orientation_)
  {
    PIDConfig ori_cfg = pid_orientation_->getConfig();
    config.pid_ori_kp = ori_cfg.kp;
    config.pid_ori_ki = ori_cfg.ki;
    config.pid_ori_kd = ori_cfg.kd;
    config.pid_ori_kff = ori_cfg.kff;
    config.pid_ori_derivative_filter_tau = ori_cfg.derivative_filter_tau;
    config.pid_ori_deadband = orientation_deadband_;
  }

  if (safety_limiter_)
  {
    Eigen::VectorXd max_vels = safety_limiter_->getJointVelocityLimits();
    Eigen::VectorXd max_accs = safety_limiter_->getJointAccelerationLimits();
    config.max_joint_velocity = (max_vels.size() > 0) ? max_vels(0) : 1.0;
    config.max_joint_acceleration = (max_accs.size() > 0) ? max_accs(0) : 5.0;
  }

  if (jacobian_solver_)
  {
    JacobianSolverConfig jac_cfg = jacobian_solver_->getConfig();
    config.singularity_threshold = jac_cfg.singularity_threshold;
    config.max_damping = jac_cfg.max_damping;
  }

  if (weight_manager_)
  {
    auto strategy = weight_manager_->getElbowStrategy();
    if (strategy)
    {
      config.singularity_avoidance_buffer = strategy->getSingularityBuffer();
      config.singularity_avoidance_weight = strategy->getMaxWeightPenalty();
    }
  }

  // POI configuration from RepulsionDataManager
  if (repulsion_manager_)
  {
    auto tcp_cfg = repulsion_manager_->getPointConfig("tcp");
    config.poi_tcp_enabled = tcp_cfg.enabled;
    config.poi_tcp_weight = tcp_cfg.weight;
    config.poi_tcp_radius = tcp_cfg.radius;
    
    auto elbow_cfg = repulsion_manager_->getPointConfig("elbow");
    config.poi_elbow_enabled = elbow_cfg.enabled;
    config.poi_elbow_weight = elbow_cfg.weight;
    config.poi_elbow_radius = elbow_cfg.radius;
    
    auto wrist_cfg = repulsion_manager_->getPointConfig("wrist");
    config.poi_wrist_enabled = wrist_cfg.enabled;
    config.poi_wrist_weight = wrist_cfg.weight;
    config.poi_wrist_radius = wrist_cfg.radius;
    
    auto forearm_cfg = repulsion_manager_->getPointConfig("forearm_mid");
    config.poi_forearm_enabled = forearm_cfg.enabled;
    config.poi_forearm_weight = forearm_cfg.weight;
    config.poi_forearm_radius = forearm_cfg.radius;

    // Map-query smoothing/prediction
    config.gradient_filter_alpha = repulsion_manager_->getGradientFilterAlpha();
    config.poi_predict_enable = repulsion_manager_->getPoiPredictEnabled();
    config.poi_predict_horizon = repulsion_manager_->getPoiPredictHorizon();
    config.poi_velocity_filter_tau = repulsion_manager_->getPoiVelocityFilterTau();
    config.poi_velocity_max = repulsion_manager_->getPoiVelocityMax();
    config.poi_predict_conservative_min_distance = repulsion_manager_->getPoiPredictConservativeMinDistance();
  }

  // Elbow injection configuration
  config.elbow_injection_critical_zone = elbow_injection_.critical_zone;
  config.elbow_injection_target_near_limit_threshold = elbow_injection_.target_near_limit_threshold;
  config.elbow_injection_k = elbow_injection_.k;
  config.elbow_injection_max_push_velocity = elbow_injection_.max_push_velocity;

  // Elbow runtime guard configuration
  if (joint_position_guard_)
  {
    // Get position limits
    double elbow_min = -2.9;
    double elbow_max = -0.01;
    if (joint_position_guard_->getJointLimits("elbow_joint", elbow_min, elbow_max))
    {
      config.elbow_joint_min = elbow_min;
      config.elbow_joint_max = elbow_max;
    }

    // Get runtime guard config
    JointRuntimeGuardConfig guard_cfg;
    if (joint_position_guard_->getRuntimeGuard("elbow_joint", guard_cfg))
    {
      config.elbow_guard_soft_zone = guard_cfg.soft_zone;
      config.elbow_guard_margin = guard_cfg.margin;
      config.elbow_guard_reentry_velocity = guard_cfg.reentry_velocity;
    }
  }

  // Map3D debug configuration
  if (map3d_manager_)
  {
    config.map3d_slice_z = map3d_manager_->getConfig().debug_slice_z;
  }

  dynamic_reconfigure::Server<ControllerTuningConfig>::CallbackType cb =
      boost::bind(&CartesianVelocityController::dynamicReconfigureCallback,
                  this,
                  boost::placeholders::_1,
                  boost::placeholders::_2);
  dynamic_reconfigure_server_->setCallback(cb);
  dynamic_reconfigure_server_->updateConfig(config);
}

void CartesianVelocityController::dynamicReconfigureCallback(ControllerTuningConfig& config,
                                                             uint32_t /*level*/)
{
  std::lock_guard<DynReconfMutex> lock(*dynamic_reconfigure_mutex_);

  // Pose filter
  pose_filter_alpha_ = std::clamp(config.pose_filter_alpha, 0.0, 1.0);
  config.pose_filter_alpha = pose_filter_alpha_;

  // Global planner
  if (global_planner_)
  {
    double wp_switch = std::max(0.001, config.waypoint_switch_distance);
    double ori_thresh = std::max(0.0, config.orientation_switch_threshold);
    global_planner_->setWaypointSwitchDistance(wp_switch);
    global_planner_->setOrientationSwitchThreshold(ori_thresh);
    config.waypoint_switch_distance = wp_switch;
    config.orientation_switch_threshold = ori_thresh;
  }

  // Scaling factors
  velocity_scale_factor_ = std::clamp(config.velocity_scale_factor, 0.0, 1.0);
  acceleration_scale_factor_ = std::clamp(config.acceleration_scale_factor, 0.0, 1.0);
  config.velocity_scale_factor = velocity_scale_factor_;
  config.acceleration_scale_factor = acceleration_scale_factor_;
  
  // Filter reset option
  reset_filter_on_target_change_ = config.reset_filter_on_target_change;
  config.reset_filter_on_target_change = reset_filter_on_target_change_;

  // Local planner (Level B)
  // Base limits (unscaled)
  double max_lin_vel_base = std::max(0.01, config.max_linear_velocity);
  double max_ang_vel_base = std::max(0.01, config.max_angular_velocity);

  // Scaled limits
  double max_lin_vel = max_lin_vel_base * velocity_scale_factor_;
  double max_ang_vel = max_ang_vel_base * velocity_scale_factor_;

  double attractive_gain = std::max(0.0, config.attractive_gain);
  double influence_dist = std::max(0.01, config.influence_distance);
  double min_safe_dist = std::max(0.001, config.min_safe_distance);

  repulsive_enabled_ = config.repulsive_enabled;
  double rep_obs_gain = repulsive_enabled_ ? std::max(0.0, config.repulsive_obstacle_gain) : 0.0;
  double rep_link_gain = repulsive_enabled_ ? std::max(0.0, config.repulsive_link_gain) : 0.0;

  if (local_planner_)
  {
    local_planner_->setAttractiveGain(attractive_gain);
    local_planner_->setMaxLinearVelocity(max_lin_vel);
    local_planner_->setMaxAngularVelocity(max_ang_vel);
    local_planner_->setRepulsiveObstacleGain(rep_obs_gain);
    local_planner_->setRepulsiveLinkGain(rep_link_gain);
    local_planner_->setInfluenceDistance(influence_dist);
    local_planner_->setMinSafeDistance(min_safe_dist);

    // Repulsive profile + smoothing
    const int profile_i = std::clamp(config.repulsive_velocity_profile, 0, 3);
    local_planner_->setRepulsiveMode(repulsiveModeFromInt(profile_i));
    config.repulsive_velocity_profile = profile_i;

    const double tau_rise = std::max(0.0, config.repulsive_filter_tau_rise);
    const double tau_fall = std::max(0.0, config.repulsive_filter_tau_fall);
    const double max_acc_rise = std::max(0.0, config.repulsive_max_acc_rise);
    const double max_acc_fall = std::max(0.0, config.repulsive_max_acc_fall);

    local_planner_->setRepulsiveVelocityFilterTaus(tau_rise, tau_fall);
    local_planner_->setRepulsiveVelocityMaxAccelerations(max_acc_rise, max_acc_fall);

    config.repulsive_filter_tau_rise = tau_rise;
    config.repulsive_filter_tau_fall = tau_fall;
    config.repulsive_max_acc_rise = max_acc_rise;
    config.repulsive_max_acc_fall = max_acc_fall;
  }

  config.max_linear_velocity = max_lin_vel_base;
  config.max_angular_velocity = max_ang_vel_base;
  config.attractive_gain = attractive_gain;
  config.repulsive_obstacle_gain = rep_obs_gain;
  config.repulsive_link_gain = rep_link_gain;
  config.influence_distance = influence_dist;
  config.min_safe_distance = min_safe_dist;

  // Soft Leash
  bool leash_enabled = config.leash_enabled;
  double leash_start = std::max(0.01, config.leash_start_distance);
  double leash_stop = std::max(leash_start + 0.01, config.leash_stop_distance);
  double leash_reset = std::max(leash_stop + 0.01, config.leash_reset_threshold);

  if (local_planner_)
  {
    local_planner_->setVirtualTargetLeashEnabled(leash_enabled);
    local_planner_->setVirtualTargetLeashParams(leash_start, leash_stop, leash_reset);
  }
  
  config.leash_start_distance = leash_start;
  config.leash_stop_distance = leash_stop;
  config.leash_reset_threshold = leash_reset;

  // Velocity filter (Level C) - keep caps tied to local planner limits
  if (velocity_filter_)
  {
    // Base limits
    double lin_acc_base = std::max(0.0, config.cartesian_linear_max_acceleration);
    double lin_jerk_base = std::max(0.0, config.cartesian_linear_max_jerk);
    double ang_acc_base = std::max(0.0, config.cartesian_angular_max_acceleration);
    double ang_jerk_base = std::max(0.0, config.cartesian_angular_max_jerk);

    VelocityLimits lin_limits = velocity_filter_->getLinearLimits();
    VelocityLimits ang_limits = velocity_filter_->getAngularLimits();

    const double filter_lin_vel_scaled =
        has_cartesian_linear_max_velocity_override_
            ? (std::max(0.0, cartesian_linear_max_velocity_override_base_) * velocity_scale_factor_)
            : max_lin_vel;
    const double filter_ang_vel_scaled =
        has_cartesian_angular_max_velocity_override_
            ? (std::max(0.0, cartesian_angular_max_velocity_override_base_) * velocity_scale_factor_)
            : max_ang_vel;

    // Effective filter max velocities: additional cap (never higher than planner cap)
    lin_limits.max_velocity = std::min(max_lin_vel, filter_lin_vel_scaled);
    lin_limits.max_acceleration = lin_acc_base * acceleration_scale_factor_;
    lin_limits.max_jerk = lin_jerk_base * acceleration_scale_factor_;

    ang_limits.max_velocity = std::min(max_ang_vel, filter_ang_vel_scaled);
    ang_limits.max_acceleration = ang_acc_base * acceleration_scale_factor_;
    ang_limits.max_jerk = ang_jerk_base * acceleration_scale_factor_;

    velocity_filter_->setLinearLimits(lin_limits);
    velocity_filter_->setAngularLimits(ang_limits);
    velocity_filter_->setTimeConstant(config.cartesian_filter_tau);

    // Limiting mode
    velocity_filter_->setUniformScalingEnabled(config.cartesian_filter_uniform_scaling_enabled);
    config.cartesian_filter_uniform_scaling_enabled = velocity_filter_->isUniformScalingEnabled();

    // Update config with BASE values for UI consistency
    config.cartesian_linear_max_acceleration = lin_acc_base;
    config.cartesian_linear_max_jerk = lin_jerk_base;
    config.cartesian_angular_max_acceleration = ang_acc_base;
    config.cartesian_angular_max_jerk = ang_jerk_base;
    config.cartesian_filter_tau = velocity_filter_->getTimeConstant();
  }

  // Joint velocity filter (pre-safety)
  if (joint_velocity_filter_)
  {
    // Enable + tau + mode
    joint_velocity_filter_->setEnabled(config.joint_filter_enabled);
    joint_velocity_filter_->setTimeConstant(config.joint_filter_tau);
    joint_velocity_filter_->setUniformScalingEnabled(config.joint_filter_uniform_scaling_enabled);

    config.joint_filter_enabled = joint_velocity_filter_->isEnabled();
    config.joint_filter_tau = joint_velocity_filter_->getTimeConstant();
    config.joint_filter_uniform_scaling_enabled = joint_velocity_filter_->isUniformScalingEnabled();

    // Limits: dynamic reconfigure uses a convenient 3+3 split (if n>=6), without touching max velocity.
    Eigen::VectorXd max_vel = joint_velocity_filter_->getMaxVelocity();
    Eigen::VectorXd max_acc = joint_velocity_filter_->getMaxAcceleration();
    Eigen::VectorXd max_jerk = joint_velocity_filter_->getMaxJerk();

    const int n = max_acc.size();
    const double acc_arm = std::max(0.0, config.joint_filter_max_acceleration_arm);
    const double acc_wrist = std::max(0.0, config.joint_filter_max_acceleration_wrist);
    const double jerk_arm = std::max(0.0, config.joint_filter_max_jerk_arm);
    const double jerk_wrist = std::max(0.0, config.joint_filter_max_jerk_wrist);

    if (n >= 6)
    {
      for (int i = 0; i < 3; ++i) max_acc[i] = acc_arm;
      for (int i = 3; i < 6; ++i) max_acc[i] = acc_wrist;
      for (int i = 0; i < 3; ++i) max_jerk[i] = jerk_arm;
      for (int i = 3; i < 6; ++i) max_jerk[i] = jerk_wrist;
    }
    else if (n > 0)
    {
      max_acc = Eigen::VectorXd::Constant(n, acc_arm);
      max_jerk = Eigen::VectorXd::Constant(n, jerk_arm);
    }

    joint_velocity_filter_->setLimits(max_vel, max_acc, max_jerk);

    config.joint_filter_max_acceleration_arm = acc_arm;
    config.joint_filter_max_acceleration_wrist = acc_wrist;
    config.joint_filter_max_jerk_arm = jerk_arm;
    config.joint_filter_max_jerk_wrist = jerk_wrist;
  }

  // PID (Level D)
  position_deadband_ = std::max(0.0, config.pid_pos_deadband);
  orientation_deadband_ = std::max(0.0, config.pid_ori_deadband);

  if (pid_position_)
  {
    PIDConfig pos_cfg = pid_position_->getConfig();
    pos_cfg.kp = config.pid_pos_kp;
    pos_cfg.ki = config.pid_pos_ki;
    pos_cfg.kd = config.pid_pos_kd;
    pos_cfg.kff = config.pid_pos_kff;
    pos_cfg.derivative_filter_tau = config.pid_pos_derivative_filter_tau;
    pos_cfg.output_limit = max_lin_vel;  // keep tied to local planner velocity cap (scaled)
    pid_position_->setConfig(pos_cfg);
    config.pid_pos_derivative_filter_tau = pos_cfg.derivative_filter_tau;
    config.pid_pos_kp = pos_cfg.kp;
    config.pid_pos_ki = pos_cfg.ki;
    config.pid_pos_kd = pos_cfg.kd;
    config.pid_pos_kff = pos_cfg.kff;
  }

  if (pid_orientation_)
  {
    PIDConfig ori_cfg = pid_orientation_->getConfig();
    ori_cfg.kp = config.pid_ori_kp;
    ori_cfg.ki = config.pid_ori_ki;
    ori_cfg.kd = config.pid_ori_kd;
    ori_cfg.kff = config.pid_ori_kff;
    ori_cfg.derivative_filter_tau = config.pid_ori_derivative_filter_tau;
    ori_cfg.output_limit = max_ang_vel;  // keep tied to local planner angular cap (scaled)
    pid_orientation_->setConfig(ori_cfg);
    config.pid_ori_derivative_filter_tau = ori_cfg.derivative_filter_tau;
    config.pid_ori_kp = ori_cfg.kp;
    config.pid_ori_ki = ori_cfg.ki;
    config.pid_ori_kd = ori_cfg.kd;
    config.pid_ori_kff = ori_cfg.kff;
  }

  config.pid_pos_deadband = position_deadband_;
  config.pid_ori_deadband = orientation_deadband_;

  // Joint safety limiter
  if (safety_limiter_)
  {
    std::size_t num_joints = safety_limiter_->getNumJoints();
    double max_joint_vel = std::max(0.0, config.max_joint_velocity);
    double max_joint_acc = std::max(0.0, config.max_joint_acceleration);

    Eigen::VectorXd max_vels = Eigen::VectorXd::Constant(num_joints, max_joint_vel);
    Eigen::VectorXd max_accs = Eigen::VectorXd::Constant(num_joints, max_joint_acc);
    safety_limiter_->setJointVelocityLimits(max_vels);
    safety_limiter_->setJointAccelerationLimits(max_accs);

    config.max_joint_velocity = max_joint_vel;
    config.max_joint_acceleration = max_joint_acc;
  }

  // Jacobian solver
  if (jacobian_solver_)
  {
    JacobianSolverConfig jac_cfg = jacobian_solver_->getConfig();
    jac_cfg.singularity_threshold = std::max(0.0, config.singularity_threshold);
    jac_cfg.max_damping = std::max(0.0, config.max_damping);
    jacobian_solver_->setConfig(jac_cfg);
    config.singularity_threshold = jac_cfg.singularity_threshold;
    config.max_damping = jac_cfg.max_damping;
  }

  // Singularity Avoidance (Joint Weights)
  if (weight_manager_)
  {
    auto strategy = weight_manager_->getElbowStrategy();
    if (strategy)
    {
       int idx = strategy->getElbowIndex();
       double buffer = std::max(0.01, config.singularity_avoidance_buffer);
       double weight = std::max(1.0, config.singularity_avoidance_weight);
       
       weight_manager_->setElbowSingularityConfig(idx, buffer, weight);
       
       config.singularity_avoidance_buffer = buffer;
       config.singularity_avoidance_weight = weight;
    }
  }

  // POI configuration
  if (repulsion_manager_)
  {
    // TCP POI
    repulsion_manager_->setPointEnabled("tcp", config.poi_tcp_enabled);
    repulsion_manager_->setPointWeight("tcp", config.poi_tcp_weight);
    repulsion_manager_->setPointRadius("tcp", config.poi_tcp_radius);
    
    // Elbow POI
    repulsion_manager_->setPointEnabled("elbow", config.poi_elbow_enabled);
    repulsion_manager_->setPointWeight("elbow", config.poi_elbow_weight);
    repulsion_manager_->setPointRadius("elbow", config.poi_elbow_radius);
    
    // Wrist POI
    repulsion_manager_->setPointEnabled("wrist", config.poi_wrist_enabled);
    repulsion_manager_->setPointWeight("wrist", config.poi_wrist_weight);
    repulsion_manager_->setPointRadius("wrist", config.poi_wrist_radius);
    
    // Forearm POI
    repulsion_manager_->setPointEnabled("forearm_mid", config.poi_forearm_enabled);
    repulsion_manager_->setPointWeight("forearm_mid", config.poi_forearm_weight);
    repulsion_manager_->setPointRadius("forearm_mid", config.poi_forearm_radius);

    // Repulsion map smoothing/prediction (3.6, 3.7)
    repulsion_manager_->setGradientFilterAlpha(std::clamp(config.gradient_filter_alpha, 0.0, 1.0));
    repulsion_manager_->setPoiPredictEnabled(config.poi_predict_enable);
    repulsion_manager_->setPoiPredictHorizon(std::max(0.0, config.poi_predict_horizon));
    repulsion_manager_->setPoiVelocityFilterTau(std::max(0.0, config.poi_velocity_filter_tau));
    repulsion_manager_->setPoiVelocityMax(std::max(0.0, config.poi_velocity_max));
    repulsion_manager_->setPoiPredictConservativeMinDistance(config.poi_predict_conservative_min_distance);

    config.gradient_filter_alpha = repulsion_manager_->getGradientFilterAlpha();
    config.poi_predict_enable = repulsion_manager_->getPoiPredictEnabled();
    config.poi_predict_horizon = repulsion_manager_->getPoiPredictHorizon();
    config.poi_velocity_filter_tau = repulsion_manager_->getPoiVelocityFilterTau();
    config.poi_velocity_max = repulsion_manager_->getPoiVelocityMax();
    config.poi_predict_conservative_min_distance = repulsion_manager_->getPoiPredictConservativeMinDistance();
  }

  // Elbow injection configuration (anti-singularity push)
  {
    double critical_zone = std::max(0.01, config.elbow_injection_critical_zone);
    double target_threshold = std::max(0.0, config.elbow_injection_target_near_limit_threshold);
    double k = std::max(0.1, config.elbow_injection_k);
    double max_push = std::max(0.0, config.elbow_injection_max_push_velocity);

    elbow_injection_.critical_zone = critical_zone;
    elbow_injection_.target_near_limit_threshold = target_threshold;
    elbow_injection_.k = k;
    elbow_injection_.max_push_velocity = max_push;

    config.elbow_injection_critical_zone = critical_zone;
    config.elbow_injection_target_near_limit_threshold = target_threshold;
    config.elbow_injection_k = k;
    config.elbow_injection_max_push_velocity = max_push;
  }

  // Elbow runtime guard configuration
  if (joint_position_guard_)
  {
    // Update position limits
    double elbow_min = config.elbow_joint_min;
    double elbow_max = config.elbow_joint_max;
    // Ensure min < max
    if (elbow_min > elbow_max)
    {
      std::swap(elbow_min, elbow_max);
    }
    joint_position_guard_->updateJointLimits("elbow_joint", elbow_min, elbow_max);
    config.elbow_joint_min = elbow_min;
    config.elbow_joint_max = elbow_max;

    // Update runtime guard parameters
    double soft_zone = std::max(0.0, config.elbow_guard_soft_zone);
    double margin = std::max(0.0, config.elbow_guard_margin);
    double reentry = std::max(0.0, config.elbow_guard_reentry_velocity);

    joint_position_guard_->updateRuntimeGuard("elbow_joint", soft_zone, margin, reentry);

    config.elbow_guard_soft_zone = soft_zone;
    config.elbow_guard_margin = margin;
    config.elbow_guard_reentry_velocity = reentry;
  }

  // Map3D debug configuration
  if (map3d_manager_)
  {
    map3d_manager_->setDebugSliceZ(config.map3d_slice_z);
  }
}

// ============================================================================
// ROS Callbacks
// ============================================================================

void CartesianVelocityController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (robot_state_)
  {
    robot_state_->updateFromJointState(*msg);
  }
}

void CartesianVelocityController::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    target_state_mode_active_ = false;
    target_state_.active = false;
  }
  setTargetPose(*msg);
}

void CartesianVelocityController::targetStateCallback(const CartesianTrajectorySetpoint::ConstPtr& msg)
{
  Eigen::Isometry3d pose;
  Eigen::Matrix<double, 6, 1> velocity;
  if (!transformTrajectorySetpoint(*msg, pose, velocity))
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    target_state_.pose = pose;
    target_state_.velocity = msg->active ? velocity : Eigen::Matrix<double, 6, 1>::Zero();
    target_state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    target_state_.active = msg->active;
    target_state_.valid = true;
    target_state_mode_active_ = true;
  }

  has_target_ = true;
  idle_until_target_ = false;
}

void CartesianVelocityController::controlLoopCallback(const ros::TimerEvent& event)
{
  // Compute dt
  double dt = control_rate_ > 0.0 ? 1.0 / control_rate_ : 0.02;
  if (!event.last_real.isZero())
  {
    dt = (event.current_real - event.last_real).toSec();
    dt = std::clamp(dt, 0.001, 0.1);  // Clamp to reasonable range
  }

  // Check if robot state is ready
  if (!robot_state_ || !robot_state_->isReady())
  {
    return;
  }

  // Initialize pose on first iteration after start()
  if (!pose_initialized_)
  {
    captureInitialPose();
    pose_initialized_ = true;
    return;
  }

  // Execute pipeline
  if (idle_until_target_ && !has_target_)
  {
    publishZeroVelocity();
    return;
  }
  executePipeline(dt);
}

// ============================================================================
// Pipeline Execution
// ============================================================================

void CartesianVelocityController::executePipeline(double dt)
{
  const std::size_t num_joints = robot_state_->getJointCount();

  // Get TCP offset
  Eigen::Isometry3d tcp_offset;
  {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    tcp_offset = tcp_offset_;
  }

  // ========== Get Current TCP Pose ==========
  Eigen::Isometry3d raw_tcp_pose;
  if (!robot_state_->computeTcpPose(tcp_offset, raw_tcp_pose))
  {
    publishZeroVelocity();
    return;
  }

  // Lightly filter the TCP pose to attenuate sensor noise without lagging the loop
  Eigen::Isometry3d current_tcp_pose = filterTcpPose(raw_tcp_pose);

  // ========== Level A: Global Planner / Trajectory Setpoint ==========
  TargetStateCache trajectory_setpoint;
  bool trajectory_mode = false;
  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    trajectory_mode = target_state_mode_active_ && target_state_.valid;
    if (trajectory_mode)
    {
      trajectory_setpoint = target_state_;
    }
  }

  if (trajectory_mode && target_state_timeout_ > 0.0 && !trajectory_setpoint.stamp.isZero())
  {
    const double age = (ros::Time::now() - trajectory_setpoint.stamp).toSec();
    if (age > target_state_timeout_)
    {
      trajectory_setpoint.active = false;
      trajectory_setpoint.velocity.setZero();
      if (target_state_zero_velocity_on_timeout_)
      {
        ROS_WARN_THROTTLE_NAMED(2.0, "cartesian_velocity_controller",
                                "target_state timed out after %.3f s; publishing zero velocity", age);
        publishZeroVelocity();
        return;
      }
      ROS_WARN_THROTTLE_NAMED(2.0, "cartesian_velocity_controller",
                              "target_state timed out after %.3f s; holding last target pose", age);
    }
  }

  // If no user target is set, hold current pose (still allows repulsion/guardrails to act).
  const bool has_waypoints = (!trajectory_mode && global_planner_ && global_planner_->hasWaypoints());
  std::size_t waypoint_index = 0;
  Eigen::Isometry3d waypoint = current_tcp_pose;

  if (trajectory_mode)
  {
    waypoint = trajectory_setpoint.pose;
  }
  else if (has_waypoints)
  {
    // Update position and check for waypoint switch
    global_planner_->updateCurrentPosition(current_tcp_pose);

    // Get current waypoint
    waypoint = global_planner_->getCurrentWaypoint();
    waypoint_index = global_planner_->getCurrentWaypointIndex();
  }

  // ========== Level B: Local Planner ==========
  // Compute attractive velocity and integrate virtual target
  std::vector<ObstacleInfo> obstacles;
  std::vector<LinkPOI> link_pois;
  
  // Get repulsion data from RepulsionDataManager if enabled
  if (repulsive_enabled_ && repulsion_manager_)
  {
    repulsion_manager_->getRepulsionData(
        obstacles, link_pois, current_tcp_pose, global_frame_, dt);
  }

  LocalPlannerOutput local_output;
  if (trajectory_mode)
  {
    local_output.target_raw = waypoint;
    local_output.combined_linear = trajectory_setpoint.velocity.head<3>();
    local_output.combined_angular = trajectory_setpoint.velocity.tail<3>();
    local_output.distance_to_waypoint = (waypoint.translation() - current_tcp_pose.translation()).norm();
  }
  else
  {
    local_output = local_planner_->compute(
        current_tcp_pose, waypoint, obstacles, link_pois, dt);
  }

  // ========== Level C: Motion Generator (Velocity Filter) ==========
  const Eigen::Isometry3d target_raw = local_output.target_raw;

  Eigen::Matrix<double, 6, 1> desired_twist;
  if (trajectory_mode)
  {
    desired_twist = trajectory_setpoint.active ? trajectory_setpoint.velocity
                                               : Eigen::Matrix<double, 6, 1>::Zero();
  }
  else
  {
    // Pose tracking: generate the motion-generator input velocity from pose error
    const Eigen::Isometry3d target_filtered_prev = velocity_filter_->getFilteredPosition();

    const Eigen::Vector3d pos_err_tf = target_raw.translation() - target_filtered_prev.translation();
    const Eigen::Vector3d ori_err_tf = orientationErrorAxisAngle(
        Eigen::Quaterniond(target_filtered_prev.rotation()),
        Eigen::Quaterniond(target_raw.rotation()));

    const double tau_lin = std::clamp(pose_tracking_.tau_linear, 0.01, 5.0);
    const double tau_ang = std::clamp(pose_tracking_.tau_angular, 0.01, 5.0);
    const double k_lin = std::clamp(pose_tracking_.k_linear, 0.0, 50.0);
    const double k_ang = std::clamp(pose_tracking_.k_angular, 0.0, 50.0);

    Eigen::Vector3d v_des_lin = (k_lin / tau_lin) * pos_err_tf;
    Eigen::Vector3d v_des_ang = (k_ang / tau_ang) * ori_err_tf;

    // Effective caps: pose_tracking caps (scaled) but never above local planner caps
    const double local_max_lin = local_planner_ ? local_planner_->getMaxLinearVelocity() : 0.5;
    const double local_max_ang = local_planner_ ? local_planner_->getMaxAngularVelocity() : 1.0;

    const double pose_max_lin_scaled =
        (pose_tracking_.max_linear_velocity_base > 1e-9)
            ? (pose_tracking_.max_linear_velocity_base * velocity_scale_factor_)
            : local_max_lin;
    const double pose_max_ang_scaled =
        (pose_tracking_.max_angular_velocity_base > 1e-9)
            ? (pose_tracking_.max_angular_velocity_base * velocity_scale_factor_)
            : local_max_ang;

    const double v_cap_lin = std::min(std::max(0.0, pose_max_lin_scaled), local_max_lin);
    const double v_cap_ang = std::min(std::max(0.0, pose_max_ang_scaled), local_max_ang);

    v_des_lin = limitNorm(v_des_lin, v_cap_lin);
    v_des_ang = limitNorm(v_des_ang, v_cap_ang);

    desired_twist.head<3>() = pose_tracking_.enabled ? v_des_lin : Eigen::Vector3d::Zero();
    desired_twist.tail<3>() = pose_tracking_.enabled ? v_des_ang : Eigen::Vector3d::Zero();
  }

  Eigen::Matrix<double, 6, 1> filtered_twist = velocity_filter_->filter(desired_twist, dt);

  // Get filtered target position for PID
  Eigen::Isometry3d target_filtered = trajectory_mode ? target_raw : velocity_filter_->getFilteredPosition();

  // ========== Level D: PID Controller ==========
  // Compute position error
  Eigen::Vector3d position_error = target_filtered.translation() - current_tcp_pose.translation();

  // Compute orientation error (axis-angle representation)
  Eigen::Quaterniond q_current(current_tcp_pose.rotation());
  Eigen::Quaterniond q_target(target_filtered.rotation());
  
  // Ensure shortest path
  if (q_current.dot(q_target) < 0.0)
  {
    q_target.coeffs() = -q_target.coeffs();
  }
  
  // Relative rotation error: R_error = R_target * R_current^{-1}
  Eigen::Quaterniond q_error = q_target * q_current.inverse();
  
  // Convert to axis-angle
  Eigen::Vector3d orientation_error;
  Eigen::AngleAxisd aa(q_error);
  orientation_error = aa.angle() * aa.axis();

  // Deadband to avoid hunting on tiny errors
  applyDeadband(position_error, position_deadband_);
  applyDeadband(orientation_error, orientation_deadband_);

  // Feed-forward velocities from filter
  Eigen::Vector3d feedforward_linear = filtered_twist.head<3>();
  Eigen::Vector3d feedforward_angular = filtered_twist.tail<3>();

  // PID computation
  Eigen::VectorXd pid_vel_linear = pid_position_->compute(position_error, feedforward_linear, dt);
  Eigen::VectorXd pid_vel_angular = pid_orientation_->compute(orientation_error, feedforward_angular, dt);

  // Combine into twist
  Eigen::Matrix<double, 6, 1> command_twist;
  command_twist.head<3>() = pid_vel_linear;
  command_twist.tail<3>() = pid_vel_angular;

  // ========== Jacobian Inverse (Cartesian to Joint Space) ==========
  Eigen::MatrixXd jacobian;
  if (!robot_state_->getJacobian(tcp_link_, tcp_offset.translation(), jacobian))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to compute Jacobian");
    publishZeroVelocity();
    return;
  }

  if (!applyJacobianFrameTransformIfConfigured(jacobian))
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "cartesian_velocity_controller",
                            "Failed to apply Jacobian frame transform (source='%s', target='%s').",
                            jacobian_source_frame_.c_str(),
                            (jacobian_target_frame_.empty() ? global_frame_ : jacobian_target_frame_).c_str());
    publishZeroVelocity();
    return;
  }

  // Snapshot current joint positions (used by weights + guardrails)
  Eigen::VectorXd current_joint_positions;
  (void)robot_state_->getCurrentJointPositions(current_joint_positions);

  // Update joint weights based on current configuration (e.g., avoid singularities)
  if (weight_manager_)
  {
    weight_manager_->update(current_joint_positions);
    joint_weights_ = weight_manager_->getWeights();
  }

  // Compute damped weighted pseudo-inverse
  Eigen::MatrixXd jacobian_pinv = jacobian_solver_->computeDampedWeightedPseudoInverse(
      jacobian, joint_weights_);

  // Convert to joint velocities
  Eigen::VectorXd joint_velocities = jacobian_pinv * command_twist;

  // ========== Controller-side guardrails (joint limits + elbow injection) ==========
  {
    const bool requires_joint_filter =
        (joint_position_guard_ && joint_position_guard_->isEnabled()) || elbow_injection_.enabled;
    if (requires_joint_filter && joint_velocity_filter_ && !joint_velocity_filter_->isEnabled())
    {
      // Enforce jerk/acc-safe behavior even if disabled at runtime.
      joint_velocity_filter_->setEnabled(true);
      joint_velocity_filter_forced_on_ = true;
      ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                              "joint_velocity_filter forced ON (required by joint guard / elbow injection).");
    }

    // Elbow injection (anti-singularity, gated by target IK)
    if (elbow_injection_.enabled && elbow_injection_has_qmax_ && elbow_injection_index_resolved_ >= 0 &&
        elbow_injection_index_resolved_ < joint_velocities.size() &&
        current_joint_positions.size() == joint_velocities.size())
    {
      const int idx = elbow_injection_index_resolved_;
      const double q_elbow = current_joint_positions[idx];
      const double qmax = elbow_injection_qmax_;

      const double critical_zone = std::max(1e-6, elbow_injection_.critical_zone);
      const double d = qmax - q_elbow;  // distance to MAX
      const double x = clamp01(d / critical_zone);
      const double w = 1.0 - smoothstep(x);  // ~1 near limit, ~0 far

      const double max_push = std::max(0.0, elbow_injection_.max_push_velocity);
      const double raw_push = std::max(0.0, elbow_injection_.k) * w;
      const double push = std::min(raw_push, max_push);
      const double qdot_bias = -push;  // push away from MAX (towards flexion)

      double gate = 1.0;
      if (has_target_)
      {
        // With a user target, gate using the IK solution of the current target/waypoint.
        gate = 0.0;
        bool has_target_solution = false;
        double q_elbow_target = 0.0;

        if (has_waypoints && waypoint_index < waypoint_ik_solutions_.size() &&
            waypoint_index < waypoint_ik_solutions_valid_.size() && waypoint_ik_solutions_valid_[waypoint_index] &&
            waypoint_ik_solutions_[waypoint_index].size() == joint_velocities.size())
        {
          q_elbow_target = waypoint_ik_solutions_[waypoint_index][idx];
          has_target_solution = true;
        }
        else if (has_last_target_ik_solution_ && last_target_ik_solution_.size() == joint_velocities.size())
        {
          q_elbow_target = last_target_ik_solution_[idx];
          has_target_solution = true;
        }

        if (has_target_solution)
        {
          const double d_target = qmax - q_elbow_target;
          const double thr = std::max(0.0, elbow_injection_.target_near_limit_threshold);
          gate = (d_target < thr) ? 0.0 : 1.0;
        }
      }

      joint_velocities[idx] += gate * qdot_bias;
    }

    // Runtime joint position guardrail (soft braking + hard margin)
    if (joint_position_guard_ && joint_position_guard_->isEnabled() &&
        current_joint_positions.size() == joint_velocities.size())
    {
      joint_position_guard_->apply(current_joint_positions, joint_velocities);
    }
  }

  // Optional joint-space smoothing (before safety limiter). Even if disabled,
  // we call it to keep internal state continuous.
  Eigen::VectorXd joint_velocities_smoothed = joint_velocities;
  if (joint_velocity_filter_)
  {
    joint_velocities_smoothed = joint_velocity_filter_->filter(joint_velocities, dt);
  }

  // ========== Final: Joint Safety Limiter ==========
  SafetyLimiterOutput safety_output = safety_limiter_->limit(
      joint_velocities_smoothed, previous_joint_velocity_, dt);

  Eigen::VectorXd final_joint_velocities = safety_output.joint_velocity;

  // Compute joint acceleration command for debugging
  Eigen::VectorXd joint_acceleration_command = Eigen::VectorXd::Zero(num_joints);
  if (dt > 1e-6)
  {
    joint_acceleration_command = (final_joint_velocities - previous_joint_velocity_) / dt;
  }

  // Store for next iteration
  previous_joint_velocity_ = final_joint_velocities;

  // ========== Publish Command ==========
  publishVelocityCommand(final_joint_velocities);

  // ========== Update Diagnostics ==========
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    last_position_error_ = position_error.norm();
    last_orientation_error_ = orientation_error.norm();
  }

  // ========== Visualization ==========
  if (marker_publisher_)
  {
    ros::Time now = ros::Time::now();
    
    // Calculate Cartesian velocity from link repulsion (projected to TCP)
    Eigen::Vector3d v_link_cartesian = Eigen::Vector3d::Zero();
    if (local_output.repulsive_links_joint.size() == jacobian.cols())
    {
      Eigen::VectorXd v_rep_six = jacobian * local_output.repulsive_links_joint;
      v_link_cartesian = v_rep_six.head<3>();
    }

    // Velocity components (v_goal, v_obs, v_link)
    marker_publisher_->publishVelocityMarkers(
        now,
        current_tcp_pose.translation(),
        local_output.attractive_linear,
        local_output.attractive_angular,
        local_output.repulsive_obstacle_linear,
        v_link_cartesian);

    // TCP pose from forward kinematics (position + orientation)
    marker_publisher_->publishTcpFKMarker(now, current_tcp_pose);
    
    // Target points (active_waypoint, target_raw, target_filtered)
    marker_publisher_->publishTargetMarkers(
        now,
        waypoint,
        local_output.target_raw,
        target_filtered);
    
    // Cartesian command with magnitude labels
    marker_publisher_->publishCommandMarkers(
        now,
        current_tcp_pose.translation(),
        command_twist.head<3>(),
        command_twist.tail<3>());
    
    // Repulsion POI markers (if repulsion is enabled)
    // Always call to visualize TCP POI even if no obstacles are present
    if (repulsive_enabled_)
    {
      double influence_dist = local_planner_ ? local_planner_->getInfluenceDistance() : 0.5;
      double tcp_radius = 0.05;
      if (repulsion_manager_)
      {
        tcp_radius = repulsion_manager_->getPointConfig("tcp").radius;
      }
      marker_publisher_->publishRepulsionMarkers(now, obstacles, local_output.link_pois_with_velocities, influence_dist, current_tcp_pose.translation(), tcp_radius);
    }
  }

  // ========== Debug Logging ==========
  ROS_DEBUG_THROTTLE(1.0, "Pipeline: pos_err=%.4f, ori_err=%.4f, scale=%.2f",
                     position_error.norm(), orientation_error.norm(),
                     safety_output.scaling_factor);

  // ========== Feedback Publisher ==========
  if (feedback_publisher_)
  {
    ros::Time now = ros::Time::now();

    // Publish end-effector state (pose, velocity, acceleration, jerk)
    feedback_publisher_->publishEndEffectorState(now, dt, current_tcp_pose);

    // Publish joint velocity feedback
    Eigen::VectorXd current_positions;
    Eigen::VectorXd current_velocities;
    robot_state_->getCurrentJointPositions(current_positions);
    robot_state_->getCurrentJointVelocities(current_velocities);

    std::vector<double> cmd_vel(final_joint_velocities.data(),
                                final_joint_velocities.data() + final_joint_velocities.size());
    std::vector<double> curr_pos(current_positions.data(),
                                 current_positions.data() + current_positions.size());
    std::vector<double> act_vel(current_velocities.data(),
                                current_velocities.data() + current_velocities.size());

    feedback_publisher_->publishJointVelocityFeedback(
        now, robot_state_->getJointNames(), cmd_vel, curr_pos, act_vel);

    // Publish pipeline debug information
    PipelineDebugData debug_data;
    
    // Level A: Global Planner
    debug_data.current_pose = current_tcp_pose;
    debug_data.active_waypoint = waypoint;
    debug_data.distance_waypoint_to_current = (waypoint.translation() - current_tcp_pose.translation()).norm();
    debug_data.active_waypoint_index = global_planner_->getCurrentWaypointIndex();
    debug_data.total_waypoints = global_planner_->getWaypointCount();
    
    // Level B: Local Planner
    debug_data.target_raw = local_output.target_raw;
    debug_data.v_goal_linear = local_output.attractive_linear;
    debug_data.v_goal_angular = local_output.attractive_angular;
    debug_data.v_obs_linear = local_output.repulsive_obstacle_linear;
    debug_data.v_link_linear = Eigen::Vector3d::Zero();  // Link repulsion is in joint space
    // v_desired_* represents the actual command that goes into the motion generator (Level C)
    debug_data.v_desired_linear = desired_twist.head<3>();
    debug_data.v_desired_angular = desired_twist.tail<3>();
    debug_data.distance_target_raw_to_waypoint = (local_output.target_raw.translation() - waypoint.translation()).norm();
    debug_data.distance_target_raw_to_current = (local_output.target_raw.translation() - current_tcp_pose.translation()).norm();
    debug_data.virtual_target_scaling_factor = local_output.virtual_target_scaling_factor;
    
    // Local Planner gains
    if (local_planner_)
    {
      debug_data.k_attractive = local_planner_->getAttractiveGain();
      debug_data.k_repulsive_tcp = local_planner_->getRepulsiveObstacleGain();
      debug_data.k_repulsive_links = local_planner_->getRepulsiveLinkGain();
    }
    
    // POI repulsion debug data
    debug_data.poi_debug_data = local_output.link_pois_with_velocities;
    debug_data.repulsive_joint_velocity = local_output.repulsive_links_joint;
    
    // Find closest obstacle among all POIs for summary info
    for (const auto& poi : local_output.link_pois_with_velocities)
    {
      if (poi.distance_to_closest_obstacle < debug_data.closest_obstacle_distance)
      {
        debug_data.closest_obstacle_distance = poi.distance_to_closest_obstacle;
        debug_data.closest_obstacle_id = poi.closest_obstacle_id;
        debug_data.closest_link_name = poi.link_name;
      }
    }
    
    // Level C: Motion Generator (Velocity Filter)
    debug_data.target_filtered = target_filtered;
    debug_data.v_filtered_linear = filtered_twist.head<3>();
    debug_data.v_filtered_angular = filtered_twist.tail<3>();
    debug_data.distance_target_filtered_to_current = (target_filtered.translation() - current_tcp_pose.translation()).norm();
    
    // Velocity filter outputs (acceleration, jerk, tau)
    if (velocity_filter_)
    {
      debug_data.acceleration_filtered_linear = velocity_filter_->getCurrentLinearAcceleration();
      debug_data.acceleration_filtered_angular = velocity_filter_->getCurrentAngularAcceleration();
      debug_data.jerk_filtered_linear = velocity_filter_->getCurrentLinearJerk();
      debug_data.jerk_filtered_angular = velocity_filter_->getCurrentAngularJerk();
      debug_data.filter_tau = velocity_filter_->getTimeConstant();
    }
    
    // Level D: PID Controller
    debug_data.pid_position_error = position_error;
    debug_data.pid_orientation_error = orientation_error;
    debug_data.pid_position_error_norm = position_error.norm();
    debug_data.pid_orientation_error_norm = orientation_error.norm();
    debug_data.pid_feedforward_linear = feedforward_linear;
    debug_data.pid_feedforward_angular = feedforward_angular;
    debug_data.pid_output_linear = pid_vel_linear;
    debug_data.pid_output_angular = pid_vel_angular;
    debug_data.cartesian_cmd_linear = command_twist.head<3>();
    debug_data.cartesian_cmd_angular = command_twist.tail<3>();
    
    // PID gains and components
    if (pid_position_)
    {
      PIDConfig pos_config = pid_position_->getConfig();
      debug_data.pid_kp_position = pos_config.kp;
      debug_data.pid_ki_position = pos_config.ki;
      debug_data.pid_kd_position = pos_config.kd;
      
      Eigen::VectorXd p_term = pid_position_->getLastPTerm();
      Eigen::VectorXd i_term = pid_position_->getLastITerm();
      Eigen::VectorXd d_term = pid_position_->getLastDTerm();
      Eigen::VectorXd integral = pid_position_->getIntegral();
      
      if (p_term.size() >= 3 && i_term.size() >= 3 && d_term.size() >= 3 && integral.size() >= 3)
      {
        debug_data.pid_p_term_linear = p_term.head<3>();
        debug_data.pid_i_term_linear = i_term.head<3>();
        debug_data.pid_d_term_linear = d_term.head<3>();
        debug_data.pid_integral_linear = integral.head<3>();
      }
    }
    
    if (pid_orientation_)
    {
      PIDConfig ori_config = pid_orientation_->getConfig();
      debug_data.pid_kp_orientation = ori_config.kp;
      debug_data.pid_ki_orientation = ori_config.ki;
      debug_data.pid_kd_orientation = ori_config.kd;
      
      Eigen::VectorXd p_term = pid_orientation_->getLastPTerm();
      Eigen::VectorXd i_term = pid_orientation_->getLastITerm();
      Eigen::VectorXd d_term = pid_orientation_->getLastDTerm();
      Eigen::VectorXd integral = pid_orientation_->getIntegral();
      
      if (p_term.size() >= 3 && i_term.size() >= 3 && d_term.size() >= 3 && integral.size() >= 3)
      {
        debug_data.pid_p_term_angular = p_term.head<3>();
        debug_data.pid_i_term_angular = i_term.head<3>();
        debug_data.pid_d_term_angular = d_term.head<3>();
        debug_data.pid_integral_angular = integral.head<3>();
      }
    }
    
    // Level D: Jacobian IK
    debug_data.joint_velocity_from_ik = joint_velocities;
    debug_data.jacobian_damping_factor = jacobian_solver_->getLastDampingFactor();
    debug_data.jacobian_min_singular_value = jacobian_solver_->getLastMinSingularValue();
    debug_data.jacobian_singular_values = jacobian_solver_->getLastSingularValues();
    debug_data.jacobian_damping_factors = jacobian_solver_->getLastDampingFactors();
    
    // Joint weights
    debug_data.joint_weights = joint_weights_;

    // Safety Limiter
    debug_data.joint_velocity_after_limiter = final_joint_velocities;
    debug_data.joint_acceleration_command = joint_acceleration_command;
    debug_data.safety_scaling_factor = safety_output.scaling_factor;
    
    // Convert limit_type enum to string
    switch (safety_output.limit_type)
    {
      case SafetyLimiterOutput::LimitType::VELOCITY:
        debug_data.safety_limiting_reason = "velocity_limit";
        break;
      case SafetyLimiterOutput::LimitType::ACCELERATION:
        debug_data.safety_limiting_reason = "acceleration_limit";
        break;
      default:
        debug_data.safety_limiting_reason = "";
        break;
    }
    
    debug_data.joint_names = robot_state_->getJointNames();

    feedback_publisher_->publishPipelineDebug(now, debug_data);
  }
}

// ============================================================================
// Velocity Publishing
// ============================================================================

void CartesianVelocityController::publishVelocityCommand(const Eigen::VectorXd& joint_velocities)
{
  if (joint_velocities.size() != static_cast<int>(last_command_.data.size()))
  {
    last_command_.data.resize(joint_velocities.size());
  }

  for (int i = 0; i < joint_velocities.size(); ++i)
  {
    last_command_.data[i] = joint_velocities[i];
  }

  velocity_pub_.publish(last_command_);
  last_command_stamp_ = ros::Time::now();
}

void CartesianVelocityController::publishZeroVelocity()
{
  std::size_t num_joints = robot_state_ ? robot_state_->getJointCount() : last_command_.data.size();
  publishVelocityCommand(Eigen::VectorXd::Zero(num_joints));
}

// ============================================================================
// Target Pose API
// ============================================================================

bool CartesianVelocityController::setTargetPose(const Eigen::Isometry3d& pose)
{
  if (!global_planner_)
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    target_state_mode_active_ = false;
    target_state_.active = false;
  }

  // Check reachability via inverse kinematics if enabled.
  // Also compute + cache a valid IK solution for target gating (elbow injection).
  const bool want_ik_solution = (robot_state_ != nullptr) && (reachability_check_enabled_ || elbow_injection_.enabled);
  if (want_ik_solution)
  {
    Eigen::Isometry3d tcp_offset;
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      tcp_offset = tcp_offset_;
    }

    Eigen::VectorXd joint_solution;
    const ControllerJointLimitsConfig* limits_ptr =
        (controller_joint_limits_.enabled && controller_joint_limits_.hasAnyEnabledLimit()) ? &controller_joint_limits_ : nullptr;

    const bool ok = robot_state_->checkPoseReachability(pose, tcp_offset, joint_solution, limits_ptr);
    if (ok)
    {
      has_last_target_ik_solution_ = true;
      last_target_ik_solution_ = joint_solution;
      waypoint_ik_solutions_.assign(1, joint_solution);
      waypoint_ik_solutions_valid_.assign(1, true);
    }
    else
    {
      has_last_target_ik_solution_ = false;
      last_target_ik_solution_.resize(0);
      waypoint_ik_solutions_.clear();
      waypoint_ik_solutions_valid_.clear();
    }

    if (reachability_check_enabled_ && !ok)
    {
      ROS_WARN_NAMED("cartesian_velocity_controller",
                     "Target pose REJECTED (not reachable via IK or violates controller limits): [%.3f, %.3f, %.3f]",
                     pose.translation().x(),
                     pose.translation().y(),
                     pose.translation().z());
      return false;
    }
  }

  // Clear existing waypoints and set single target
  global_planner_->clearWaypoints();
  global_planner_->addWaypoint(pose);
  has_target_ = true;
  idle_until_target_ = false;
  
  if (reset_filter_on_target_change_)
  {
    resetVirtualTargetsToCurrentPose();
  }
  
  ROS_INFO_NAMED("cartesian_velocity_controller",
                 "Target pose set: [%.3f, %.3f, %.3f]",
                 pose.translation().x(),
                 pose.translation().y(),
                 pose.translation().z());
  return true;
}

bool CartesianVelocityController::setTargetPose(const geometry_msgs::Pose& pose)
{
  return setTargetPose(poseToIsometry(pose));
}

bool CartesianVelocityController::setTargetPose(const geometry_msgs::PoseStamped& pose_stamped)
{
  // Handle empty frame_id explicitly (common in quick scripts / tests).
  const std::string in_frame = pose_stamped.header.frame_id;
  if (in_frame.empty())
  {
    if (!accept_empty_frame_as_global_)
    {
      ROS_WARN_THROTTLE(5.0, "Received target_pose with empty frame_id (global_frame='%s'). Rejecting.",
                        global_frame_.c_str());
      return false;
    }
    // Treat as already expressed in global frame.
    return setTargetPose(pose_stamped.pose);
  }

  // Fast path: already in global frame.
  if (in_frame == global_frame_)
  {
    return setTargetPose(pose_stamped.pose);
  }

  // Transform target pose into global_frame_ using TF2.
  try
  {
    const geometry_msgs::TransformStamped T =
        tf_buffer_.lookupTransform(global_frame_, in_frame, ros::Time(0), ros::Duration(tf_timeout_));

    geometry_msgs::PoseStamped transformed = pose_stamped;
    tf2::doTransform(pose_stamped, transformed, T);

    return setTargetPose(transformed.pose);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "TF transform failed (%s -> %s): %s",
                      in_frame.c_str(), global_frame_.c_str(), ex.what());
    if (reject_on_tf_failure_)
    {
      return false;
    }
    // Unsafe fallback (explicitly configurable): accept pose as-is.
    return setTargetPose(pose_stamped.pose);
  }
}

bool CartesianVelocityController::transformTrajectorySetpoint(
    const CartesianTrajectorySetpoint& in,
    Eigen::Isometry3d& pose_out,
    Eigen::Matrix<double, 6, 1>& velocity_out) const
{
  velocity_out.setZero();

  const std::string in_frame = in.header.frame_id;
  if (in_frame.empty())
  {
    if (!accept_empty_frame_as_global_)
    {
      ROS_WARN_THROTTLE(5.0, "Received target_state with empty frame_id (global_frame='%s'). Rejecting.",
                        global_frame_.c_str());
      return false;
    }
    pose_out = poseToIsometry(in.pose);
    velocity_out.head<3>() = vector3ToEigen(in.velocity.linear);
    velocity_out.tail<3>() = vector3ToEigen(in.velocity.angular);
    return true;
  }

  if (in_frame == global_frame_)
  {
    pose_out = poseToIsometry(in.pose);
    velocity_out.head<3>() = vector3ToEigen(in.velocity.linear);
    velocity_out.tail<3>() = vector3ToEigen(in.velocity.angular);
    return true;
  }

  try
  {
    const geometry_msgs::TransformStamped T =
        tf_buffer_.lookupTransform(global_frame_, in_frame, ros::Time(0), ros::Duration(tf_timeout_));

    geometry_msgs::PoseStamped pose_in;
    pose_in.header = in.header;
    pose_in.pose = in.pose;

    geometry_msgs::PoseStamped pose_transformed;
    tf2::doTransform(pose_in, pose_transformed, T);
    pose_out = poseToIsometry(pose_transformed.pose);

    velocity_out.head<3>() = rotateVectorByTransform(T, vector3ToEigen(in.velocity.linear));
    velocity_out.tail<3>() = rotateVectorByTransform(T, vector3ToEigen(in.velocity.angular));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "TF transform failed for target_state (%s -> %s): %s",
                      in_frame.c_str(), global_frame_.c_str(), ex.what());
    if (reject_on_tf_failure_)
    {
      return false;
    }

    pose_out = poseToIsometry(in.pose);
    velocity_out.head<3>() = vector3ToEigen(in.velocity.linear);
    velocity_out.tail<3>() = vector3ToEigen(in.velocity.angular);
    return true;
  }
}

bool CartesianVelocityController::setWaypoints(const std::vector<Eigen::Isometry3d>& waypoints)
{
  if (!global_planner_ || waypoints.empty())
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    target_state_mode_active_ = false;
    target_state_.active = false;
  }

  // Check reachability for all waypoints if enabled.
  // Also compute + cache IK solutions for gating (optional but recommended).
  const bool want_ik_solution = (robot_state_ != nullptr) && (reachability_check_enabled_ || elbow_injection_.enabled);
  if (want_ik_solution)
  {
    Eigen::Isometry3d tcp_offset;
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      tcp_offset = tcp_offset_;
    }

    const ControllerJointLimitsConfig* limits_ptr =
        (controller_joint_limits_.enabled && controller_joint_limits_.hasAnyEnabledLimit()) ? &controller_joint_limits_ : nullptr;

    std::vector<Eigen::VectorXd> solutions;
    std::vector<bool> valid;
    solutions.resize(waypoints.size());
    valid.resize(waypoints.size(), false);

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      Eigen::VectorXd joint_solution;
      const bool ok = robot_state_->checkPoseReachability(waypoints[i], tcp_offset, joint_solution, limits_ptr);
      if (ok)
      {
        solutions[i] = joint_solution;
        valid[i] = true;
      }

      if (reachability_check_enabled_ && !ok)
      {
        ROS_WARN_NAMED("cartesian_velocity_controller",
                       "Waypoint %zu REJECTED (not reachable via IK or violates controller limits): [%.3f, %.3f, %.3f]",
                       i,
                       waypoints[i].translation().x(),
                       waypoints[i].translation().y(),
                       waypoints[i].translation().z());
        return false;
      }
    }

    waypoint_ik_solutions_ = std::move(solutions);
    waypoint_ik_solutions_valid_ = std::move(valid);

    // Cache the current active waypoint solution (index 0 after setWaypoints)
    if (!waypoint_ik_solutions_.empty() && !waypoint_ik_solutions_valid_.empty() && waypoint_ik_solutions_valid_[0])
    {
      has_last_target_ik_solution_ = true;
      last_target_ik_solution_ = waypoint_ik_solutions_[0];
    }
    else
    {
      has_last_target_ik_solution_ = false;
      last_target_ik_solution_.resize(0);
    }
  }

  global_planner_->setWaypoints(waypoints);
  has_target_ = true;
  idle_until_target_ = false;
  
  if (reset_filter_on_target_change_)
  {
    resetVirtualTargetsToCurrentPose();
  }
  
  ROS_INFO_NAMED("cartesian_velocity_controller", "Set %zu waypoints (all reachable)", waypoints.size());
  return true;
}

void CartesianVelocityController::clearTargetPose()
{
  {
    std::lock_guard<std::mutex> lock(target_state_mutex_);
    target_state_mode_active_ = false;
    target_state_.active = false;
    target_state_.valid = false;
  }

  if (global_planner_)
  {
    global_planner_->clearWaypoints();
  }
  has_target_ = false;
  idle_until_target_ = (startup_behavior_ == StartupBehavior::ZERO_VELOCITY);

  // Clear IK caches (no target -> gate should be OPEN)
  has_last_target_ik_solution_ = false;
  last_target_ik_solution_.resize(0);
  waypoint_ik_solutions_.clear();
  waypoint_ik_solutions_valid_.clear();
  
  if (local_planner_)
  {
    local_planner_->reset();
  }
  
  publishZeroVelocity();
  
  ROS_INFO_NAMED("cartesian_velocity_controller", "Target cleared");
}

Eigen::Isometry3d CartesianVelocityController::getTargetPose() const
{
  if (global_planner_ && global_planner_->hasWaypoints())
  {
    return global_planner_->getCurrentWaypoint();
  }
  return Eigen::Isometry3d::Identity();
}

bool CartesianVelocityController::hasReachedTarget() const
{
  if (!global_planner_ || !global_planner_->hasWaypoints())
  {
    return true;  // No target = reached
  }
  
  // Check if at final waypoint and within threshold
  if (global_planner_->isAtFinalWaypoint())
  {
    double pos_dist = global_planner_->getDistanceToCurrentWaypoint();
    double ori_dist = global_planner_->getAngularDistanceToCurrentWaypoint();
    
    return (pos_dist < global_planner_->getWaypointSwitchDistance() &&
            ori_dist < global_planner_->getOrientationSwitchThreshold());
  }
  
  return false;
}

// ============================================================================
// PID Control API
// ============================================================================

void CartesianVelocityController::setPIDGains(double kp_pos, double ki_pos, double kd_pos,
                                              double kp_ori, double ki_ori, double kd_ori)
{
  if (pid_position_)
  {
    PIDConfig config = pid_position_->getConfig();
    config.kp = kp_pos;
    config.ki = ki_pos;
    config.kd = kd_pos;
    pid_position_->setConfig(config);
  }
  
  if (pid_orientation_)
  {
    PIDConfig config = pid_orientation_->getConfig();
    config.kp = kp_ori;
    config.ki = ki_ori;
    config.kd = kd_ori;
    pid_orientation_->setConfig(config);
  }
}

void CartesianVelocityController::setFeedForwardGain(double kff_pos, double kff_ori)
{
  if (pid_position_)
  {
    PIDConfig config = pid_position_->getConfig();
    config.kff = kff_pos;
    pid_position_->setConfig(config);
  }
  
  if (pid_orientation_)
  {
    PIDConfig config = pid_orientation_->getConfig();
    config.kff = kff_ori;
    pid_orientation_->setConfig(config);
  }
}

void CartesianVelocityController::resetPIDControllers()
{
  if (pid_position_) pid_position_->reset();
  if (pid_orientation_) pid_orientation_->reset();
}

// ============================================================================
// Velocity Filter API
// ============================================================================

void CartesianVelocityController::setCartesianFilterEnabled(bool enabled)
{
  if (velocity_filter_)
  {
    velocity_filter_->setEnabled(enabled);
  }
}

bool CartesianVelocityController::isCartesianFilterEnabled() const
{
  return velocity_filter_ && velocity_filter_->isEnabled();
}

void CartesianVelocityController::setCartesianFilterTimeConstant(double tau)
{
  if (velocity_filter_)
  {
    velocity_filter_->setTimeConstant(tau);
  }
}

double CartesianVelocityController::getCartesianFilterTimeConstant() const
{
  return velocity_filter_ ? velocity_filter_->getTimeConstant() : 0.1;
}

void CartesianVelocityController::resetVelocityFilter()
{
  if (velocity_filter_)
  {
    velocity_filter_->reset();
  }
}

// ============================================================================
// Joint Safety Limiter API
// ============================================================================

void CartesianVelocityController::setJointVelocityLimit(double max_vel)
{
  if (safety_limiter_)
  {
    std::size_t num_joints = safety_limiter_->getNumJoints();
    safety_limiter_->setJointVelocityLimits(Eigen::VectorXd::Constant(num_joints, max_vel));
  }
}

void CartesianVelocityController::setJointAccelerationLimit(double max_acc)
{
  if (safety_limiter_)
  {
    std::size_t num_joints = safety_limiter_->getNumJoints();
    safety_limiter_->setJointAccelerationLimits(Eigen::VectorXd::Constant(num_joints, max_acc));
  }
}

// ============================================================================
// TCP Configuration API
// ============================================================================

void CartesianVelocityController::setTcpOffset(const Eigen::Isometry3d& offset)
{
  std::lock_guard<std::mutex> lock(tcp_mutex_);
  tcp_offset_ = offset;
}

Eigen::Isometry3d CartesianVelocityController::getTcpOffset() const
{
  std::lock_guard<std::mutex> lock(tcp_mutex_);
  return tcp_offset_;
}

std::string CartesianVelocityController::getTcpLink() const
{
  return tcp_link_;
}

// ============================================================================
// Diagnostics API
// ============================================================================

double CartesianVelocityController::getDistanceToTarget() const
{
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  return last_position_error_;
}

double CartesianVelocityController::getOrientationErrorToTarget() const
{
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  return last_orientation_error_;
}

Eigen::VectorXd CartesianVelocityController::getLastJointVelocityCommand() const
{
  Eigen::VectorXd result(last_command_.data.size());
  for (std::size_t i = 0; i < last_command_.data.size(); ++i)
  {
    result[i] = last_command_.data[i];
  }
  return result;
}

// ============================================================================
// Controller Switching
// ============================================================================

bool CartesianVelocityController::switchControllers(const std::vector<std::string>& start,
                                                    const std::vector<std::string>& stop)
{
  if (!switch_client_.exists())
  {
    if (!switch_client_.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN("controller_manager/switch_controller service unavailable.");
      return false;
    }
  }

  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start;
  srv.request.stop_controllers = stop;
  srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  srv.request.timeout = 2.0;

  if (!switch_client_.call(srv) || !srv.response.ok)
  {
    ROS_WARN("Switch controller failed.");
    return false;
  }

  return true;
}

// ============================================================================
// Shutdown Handling
// ============================================================================

void CartesianVelocityController::handleShutdown()
{
  stop();
}

void CartesianVelocityController::sigintHandler(int /*signum*/)
{
  signal(SIGINT, SIG_DFL);
  if (instance_)
  {
    instance_->handleSigint();
  }
  else
  {
    ros::shutdown();
  }
}

void CartesianVelocityController::handleSigint()
{
  ROS_WARN_NAMED("cartesian_velocity_controller", "SIGINT received. Safe shutdown.");

  control_timer_.stop();
  
  // Publish zero velocity for a short duration
  publishZeroVelocity();
  ros::Duration(0.1).sleep();
  publishZeroVelocity();

  if (marker_publisher_)
  {
    marker_publisher_->clear();
  }

  // Restore controllers
  if (controllers_switched_)
  {
    std::vector<std::string> start_list, stop_list;
    if (stop_controller_stopped_by_node_ && !stop_controller_name_.empty())
    {
      start_list.push_back(stop_controller_name_);
    }
    if (start_controller_started_by_node_ && !start_controller_name_.empty())
    {
      stop_list.push_back(start_controller_name_);
    }
    if (!start_list.empty() || !stop_list.empty())
    {
      switchControllers(start_list, stop_list);
    }
  }

  is_running_ = false;
  ros::shutdown();
}

// ============================================================================
// Utility Functions
// ============================================================================

Eigen::Isometry3d CartesianVelocityController::poseToIsometry(const geometry_msgs::Pose& pose) const
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  // Defensive: many publishers leave orientation as (0,0,0,0) or non-normalized.
  // A zero-norm quaternion would generate NaNs and can destabilize the controller.
  const double n = q.norm();
  if (!std::isfinite(n) || n < 1e-12)
  {
    ROS_WARN_THROTTLE_NAMED(2.0, "cartesian_velocity_controller",
                            "Received target pose with invalid quaternion (norm=%.3e). Using identity orientation.", n);
    q = Eigen::Quaterniond::Identity();
  }
  else
  {
    q.normalize();
    // Ensure a consistent hemisphere (optional but helps continuity).
    if (q.w() < 0.0) q.coeffs() = -q.coeffs();
  }
  result.linear() = q.toRotationMatrix();
  return result;
}

Eigen::Isometry3d CartesianVelocityController::filterTcpPose(const Eigen::Isometry3d& measured_pose)
{
  if (!has_filtered_tcp_pose_)
  {
    filtered_tcp_pose_ = measured_pose;
    has_filtered_tcp_pose_ = true;
    return filtered_tcp_pose_;
  }

  const double alpha = pose_filter_alpha_;
  const double one_minus_alpha = 1.0 - alpha;

  // Blend translation
  filtered_tcp_pose_.translation() =
      alpha * measured_pose.translation() + one_minus_alpha * filtered_tcp_pose_.translation();

  // Blend orientation with slerp (ensure shortest path)
  Eigen::Quaterniond q_measured(measured_pose.rotation());
  Eigen::Quaterniond q_prev(filtered_tcp_pose_.rotation());
  if (q_prev.dot(q_measured) < 0.0)
  {
    q_measured.coeffs() = -q_measured.coeffs();
  }
  Eigen::Quaterniond q_filtered = q_prev.slerp(alpha, q_measured);
  filtered_tcp_pose_.linear() = q_filtered.normalized().toRotationMatrix();

  return filtered_tcp_pose_;
}

bool CartesianVelocityController::resetVirtualTargetsToCurrentPose()
{
  if (!robot_state_ || !robot_state_->isReady())
  {
    ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                            "Cannot reset virtual targets: robot state not ready");
    return false;
  }

  Eigen::Isometry3d tcp_offset;
  {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    tcp_offset = tcp_offset_;
  }

  Eigen::Isometry3d current_tcp_pose;
  if (!robot_state_->computeTcpPose(tcp_offset, current_tcp_pose))
  {
    ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                            "Cannot reset virtual targets: failed to compute TCP pose");
    return false;
  }

  // Calculate current Cartesian velocity for filter initialization
  // This prevents velocity jumps when switching targets while moving
  Eigen::Vector3d current_lin_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_ang_vel = Eigen::Vector3d::Zero();
  
  Eigen::VectorXd joint_vels;
  Eigen::MatrixXd jacobian;
  if (robot_state_->getCurrentJointVelocities(joint_vels) &&
      robot_state_->getJacobian(tcp_link_, tcp_offset.translation(), jacobian))
  {
    if (applyJacobianFrameTransformIfConfigured(jacobian))
    {
      Eigen::VectorXd cart_vel = jacobian * joint_vels;
      current_lin_vel = cart_vel.head<3>();
      current_ang_vel = cart_vel.tail<3>();
    }
  }

  if (local_planner_)
  {
    local_planner_->resetToPosition(current_tcp_pose);
  }
  if (velocity_filter_)
  {
    // Capture current filter state (acceleration and jerk) to preserve continuity
    Eigen::Vector3d lin_acc = velocity_filter_->getCurrentLinearAcceleration();
    Eigen::Vector3d ang_acc = velocity_filter_->getCurrentAngularAcceleration();
    Eigen::Vector3d lin_jerk = velocity_filter_->getCurrentLinearJerk();
    Eigen::Vector3d ang_jerk = velocity_filter_->getCurrentAngularJerk();

    velocity_filter_->resetToState(current_tcp_pose,
                                   current_lin_vel, current_ang_vel,
                                   lin_acc, ang_acc,
                                   lin_jerk, ang_jerk);
  }

  // Reset PID controllers to clear accumulated integral error from previous motion
  resetPIDControllers();

  ROS_DEBUG_NAMED("cartesian_velocity_controller",
                  "Virtual targets reset to current TCP pose");
  return true;
}

bool CartesianVelocityController::ensureJacobianFrameTransformReady()
{
  // Disabled: preserve legacy behavior (no conversion).
  if (jacobian_source_frame_.empty())
  {
    return true;
  }

  // Fast path: already cached.
  {
    std::lock_guard<std::mutex> lock(jacobian_frame_mutex_);
    if (jacobian_frame_transform_ready_)
    {
      return true;
    }
  }

  const std::string source = jacobian_source_frame_;
  const std::string target = jacobian_target_frame_.empty() ? global_frame_ : jacobian_target_frame_;

  // Identity case.
  if (source == target)
  {
    std::lock_guard<std::mutex> lock(jacobian_frame_mutex_);
    jacobian_frame_transform_.setIdentity();
    jacobian_frame_transform_ready_ = true;
    return true;
  }

  try
  {
    const geometry_msgs::TransformStamped tf =
        tf_buffer_.lookupTransform(target, source, ros::Time(0), ros::Duration(tf_timeout_));

    const auto& r = tf.transform.rotation;
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()))
    {
      ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                              "Jacobian frame TF rotation contains non-finite values (%s -> %s).",
                              source.c_str(), target.c_str());
      return false;
    }

    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    Eigen::Matrix<double, 6, 6> X = Eigen::Matrix<double, 6, 6>::Zero();
    X.topLeftCorner<3, 3>() = R;
    X.bottomRightCorner<3, 3>() = R;

    {
      std::lock_guard<std::mutex> lock(jacobian_frame_mutex_);
      jacobian_frame_transform_ = X;
      jacobian_frame_transform_ready_ = true;
    }

    ROS_INFO_STREAM_NAMED("cartesian_velocity_controller",
                          "Cached Jacobian frame rotation: '" << source << "' -> '" << target << "'");
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                            "Jacobian frame TF lookup failed (%s -> %s): %s",
                            source.c_str(), target.c_str(), ex.what());
    return false;
  }
}

bool CartesianVelocityController::applyJacobianFrameTransformIfConfigured(Eigen::MatrixXd& jacobian)
{
  if (jacobian_source_frame_.empty())
  {
    return true;  // disabled
  }

  if (jacobian.rows() != 6)
  {
    ROS_WARN_THROTTLE_NAMED(5.0, "cartesian_velocity_controller",
                            "Jacobian has %ld rows (expected 6). Cannot apply frame transform.",
                            static_cast<long>(jacobian.rows()));
    return false;
  }

  if (!ensureJacobianFrameTransformReady())
  {
    return false;
  }

  Eigen::Matrix<double, 6, 6> X;
  {
    std::lock_guard<std::mutex> lock(jacobian_frame_mutex_);
    X = jacobian_frame_transform_;
  }

  jacobian = X * jacobian;
  return true;
}

void CartesianVelocityController::applyDeadband(Eigen::Vector3d& vec, double threshold)
{
  if (vec.norm() < threshold)
  {
    vec.setZero();
  }
}

// ============================================================================
// Debug Service: Get Frame Info
// ============================================================================

bool CartesianVelocityController::getFrameInfoCallback(GetFrameInfo::Request& /*req*/,
                                                        GetFrameInfo::Response& res)
{
  // Get model root frame from RobotStateManager
  if (robot_state_)
  {
    res.model_root_frame = robot_state_->getModelRootFrame();
    res.tcp_link = robot_state_->getTcpLink();
    res.group_name = robot_state_->getGroupName();

    // Get current joint names and positions
    res.joint_names = robot_state_->getJointNames();
    Eigen::VectorXd positions;
    if (robot_state_->getCurrentJointPositions(positions))
    {
      res.joint_positions.resize(positions.size());
      for (Eigen::Index i = 0; i < positions.size(); ++i)
      {
        res.joint_positions[i] = positions[i];
      }
    }

    // Get current TCP pose in model frame (via forward kinematics)
    Eigen::Isometry3d tcp_offset;
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      tcp_offset = tcp_offset_;
    }

    Eigen::Isometry3d tcp_pose;
    if (robot_state_->computeTcpPose(tcp_offset, tcp_pose))
    {
      res.tcp_pose_in_model_frame.position.x = tcp_pose.translation().x();
      res.tcp_pose_in_model_frame.position.y = tcp_pose.translation().y();
      res.tcp_pose_in_model_frame.position.z = tcp_pose.translation().z();

      Eigen::Quaterniond q(tcp_pose.rotation());
      res.tcp_pose_in_model_frame.orientation.w = q.w();
      res.tcp_pose_in_model_frame.orientation.x = q.x();
      res.tcp_pose_in_model_frame.orientation.y = q.y();
      res.tcp_pose_in_model_frame.orientation.z = q.z();
    }
  }
  else
  {
    res.model_root_frame = "(robot_state not initialized)";
  }

  // Configured global frame
  res.config_global_frame = global_frame_;

  // Try to get TF transform from model_root_frame to config_global_frame
  res.tf_available = false;
  res.frames_match = false;

  if (!res.model_root_frame.empty() && !res.config_global_frame.empty())
  {
    // Check if frames are identical (then no transform needed, they match)
    if (res.model_root_frame == res.config_global_frame)
    {
      res.frames_match = true;
      res.tf_available = true;
      // Identity transform
      res.tf_model_to_global.translation.x = 0.0;
      res.tf_model_to_global.translation.y = 0.0;
      res.tf_model_to_global.translation.z = 0.0;
      res.tf_model_to_global.rotation.w = 1.0;
      res.tf_model_to_global.rotation.x = 0.0;
      res.tf_model_to_global.rotation.y = 0.0;
      res.tf_model_to_global.rotation.z = 0.0;
    }
    else
    {
      try
      {
        geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
            res.config_global_frame, res.model_root_frame, ros::Time(0), ros::Duration(0.1));

        res.tf_available = true;
        res.tf_model_to_global = tf.transform;

        // Check if it's approximately identity
        const double trans_norm = std::sqrt(
            tf.transform.translation.x * tf.transform.translation.x +
            tf.transform.translation.y * tf.transform.translation.y +
            tf.transform.translation.z * tf.transform.translation.z);
        
        // For rotation, check if quaternion is close to identity (w≈1, xyz≈0)
        const double rot_error = std::sqrt(
            (tf.transform.rotation.w - 1.0) * (tf.transform.rotation.w - 1.0) +
            tf.transform.rotation.x * tf.transform.rotation.x +
            tf.transform.rotation.y * tf.transform.rotation.y +
            tf.transform.rotation.z * tf.transform.rotation.z);

        res.frames_match = (trans_norm < 0.001 && rot_error < 0.001);
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(5.0, "GetFrameInfo: TF lookup failed: %s", ex.what());
        res.tf_available = false;
        res.frames_match = false;
      }
    }
  }

  return true;
}

// ============================================================================
// Debug/Utility Service: Get Jacobian
// ============================================================================

bool CartesianVelocityController::getJacobianCallback(GetJacobian::Request& /*req*/,
                                                      GetJacobian::Response& res)
{
  res.success = false;
  res.message.clear();
  res.frame_id.clear();
  res.tcp_link = tcp_link_;
  res.rows = 0;
  res.cols = 0;
  res.data.clear();

  if (!robot_state_ || !robot_state_->isReady())
  {
    res.message = "Robot state not ready (no JointState received yet).";
    return true;  // service call succeeded, request handled
  }

  Eigen::Isometry3d tcp_offset;
  {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    tcp_offset = tcp_offset_;
  }

  Eigen::MatrixXd jacobian;
  if (!robot_state_->getJacobian(tcp_link_, tcp_offset.translation(), jacobian))
  {
    res.message = "Failed to compute Jacobian for tcp_link='" + tcp_link_ + "'.";
    return true;
  }

  if (jacobian.rows() == 0 || jacobian.cols() == 0)
  {
    res.message = "Jacobian is empty.";
    return true;
  }

  if (!applyJacobianFrameTransformIfConfigured(jacobian))
  {
    res.message = "Failed to apply Jacobian frame transform (TF not available or invalid).";
    return true;
  }

  res.frame_id = jacobian_source_frame_.empty()
                     ? robot_state_->getModelRootFrame()
                     : (jacobian_target_frame_.empty() ? global_frame_ : jacobian_target_frame_);
  res.tcp_link = tcp_link_;
  res.rows = static_cast<uint32_t>(jacobian.rows());
  res.cols = static_cast<uint32_t>(jacobian.cols());
  res.data.resize(static_cast<std::size_t>(res.rows) * static_cast<std::size_t>(res.cols));

  // Row-major flattening
  for (Eigen::Index r = 0; r < jacobian.rows(); ++r)
  {
    for (Eigen::Index c = 0; c < jacobian.cols(); ++c)
    {
      const std::size_t idx = static_cast<std::size_t>(r) * static_cast<std::size_t>(jacobian.cols()) +
                              static_cast<std::size_t>(c);
      res.data[idx] = jacobian(r, c);
    }
  }

  res.success = true;
  res.message = "OK";
  return true;
}

// ============================================================================
// Utility Service: Validate Poses
// ============================================================================

bool CartesianVelocityController::validatePosesCallback(ValidatePoses::Request& req,
                                                        ValidatePoses::Response& res)
{
  res.success = false;
  res.valid.clear();

  if (!robot_state_ || !robot_state_->isReady())
  {
    res.message = "Robot state not ready (no JointState received yet).";
    return true;
  }

  Eigen::Isometry3d tcp_offset;
  {
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    tcp_offset = tcp_offset_;
  }

  const ControllerJointLimitsConfig* limits_ptr =
      (controller_joint_limits_.enabled && controller_joint_limits_.hasAnyEnabledLimit()) ? &controller_joint_limits_ : nullptr;

  res.valid.resize(req.poses.size(), false);
  std::size_t invalid_count = 0;
  for (std::size_t i = 0; i < req.poses.size(); ++i)
  {
    geometry_msgs::PoseStamped pose_in;
    pose_in.header = req.header;
    pose_in.pose = req.poses[i];

    Eigen::Isometry3d pose;
    if (pose_in.header.frame_id.empty() || pose_in.header.frame_id == global_frame_)
    {
      if (pose_in.header.frame_id.empty() && !accept_empty_frame_as_global_)
      {
        invalid_count++;
        continue;
      }
      pose = poseToIsometry(pose_in.pose);
    }
    else
    {
      try
      {
        const geometry_msgs::TransformStamped T =
            tf_buffer_.lookupTransform(global_frame_, pose_in.header.frame_id, ros::Time(0), ros::Duration(tf_timeout_));
        geometry_msgs::PoseStamped transformed;
        tf2::doTransform(pose_in, transformed, T);
        pose = poseToIsometry(transformed.pose);
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "ValidatePoses TF failed (%s -> %s): %s",
                          pose_in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
        invalid_count++;
        continue;
      }
    }

    Eigen::VectorXd joint_solution;
    const bool ok = robot_state_->checkPoseReachability(pose, tcp_offset, joint_solution, limits_ptr);
    res.valid[i] = ok;
    if (!ok)
    {
      invalid_count++;
    }
  }

  res.success = (invalid_count == 0);
  res.message = res.success ? "OK" : (std::to_string(invalid_count) + " pose(s) are not reachable");
  return true;
}

}  // namespace cartesian_velocity_controller
