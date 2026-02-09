#include "teleoperation/teleop_slave_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

#include "teleoperation/math_utils.hpp"

namespace teleoperation
{

namespace
{
Eigen::Isometry3d emaPose(const Eigen::Isometry3d& prev, const Eigen::Isometry3d& curr, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();

  out.translation() = a * curr.translation() + (1.0 - a) * prev.translation();

  const Eigen::Quaterniond q_prev(prev.rotation());
  const Eigen::Quaterniond q_curr(curr.rotation());
  Eigen::Quaterniond q_out = q_prev.slerp(a, q_curr);
  q_out.normalize();
  out.linear() = q_out.toRotationMatrix();

  return out;
}
}  // namespace

TeleopSlaveController::TeleopSlaveController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
{
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  loadParameters();

  // Init core components
  robot_state_manager_ = std::make_unique<RobotStateManager>(group_name_, tcp_link_, robot_description_param_);

  // Jacobian solver config
  JacobianSolverConfig jcfg;
  pnh_.param("jacobian/singularity_threshold", jcfg.singularity_threshold, jcfg.singularity_threshold);
  pnh_.param("jacobian/max_damping", jcfg.max_damping, jcfg.max_damping);
  jacobian_solver_.setConfig(jcfg);

  // PID configs
  PIDConfig pcfg;
  pnh_.param("pid/position/kp", pcfg.kp, 1.0);
  pnh_.param("pid/position/ki", pcfg.ki, 0.0);
  pnh_.param("pid/position/kd", pcfg.kd, 0.0);
  pnh_.param("pid/position/output_limit", pcfg.output_limit, 0.05);
  pnh_.param("pid/position/derivative_filter_tau", pcfg.derivative_filter_tau, 0.02);
  pcfg.kff = 0.0;
  pcfg.enabled = true;
  pid_pos_.setConfig(pcfg);

  PIDConfig ocfg;
  pnh_.param("pid/orientation/kp", ocfg.kp, 1.0);
  pnh_.param("pid/orientation/ki", ocfg.ki, 0.0);
  pnh_.param("pid/orientation/kd", ocfg.kd, 0.0);
  pnh_.param("pid/orientation/output_limit", ocfg.output_limit, 0.15);
  pnh_.param("pid/orientation/derivative_filter_tau", ocfg.derivative_filter_tau, 0.02);
  ocfg.kff = 0.0;
  ocfg.enabled = true;
  pid_ori_.setConfig(ocfg);

  // Limits & safety limiter
  const std::size_t n = robot_state_manager_->getJointCount();
  joint_safety_limiter_ = std::make_unique<JointSafetyLimiter>(n);

  // Joint velocity limits (required)
  max_joint_velocities_ = Eigen::VectorXd::Constant(static_cast<int>(n), 1.0);
  std::vector<double> vmax;
  const bool has_vmax_vec = pnh_.getParam("limits/max_joint_velocities", vmax);
  if (has_vmax_vec && vmax.size() == n)
  {
    for (std::size_t i = 0; i < n; ++i) max_joint_velocities_[static_cast<Eigen::Index>(i)] = std::abs(vmax[i]);
  }
  else if (!has_vmax_vec || vmax.empty())
  {
    // Allow scalar uniform limit as a simpler config option.
    double vmax_scalar = 1.0;
    pnh_.param("limits/max_joint_velocity", vmax_scalar, vmax_scalar);
    max_joint_velocities_.setConstant(std::abs(vmax_scalar));
  }
  else
  {
    ROS_WARN_NAMED("teleop_slave_controller",
                  "limits/max_joint_velocities has size %zu, expected %zu. Using default 1.0 rad/s each.",
                  vmax.size(), n);
  }
  joint_safety_limiter_->setJointVelocityLimits(max_joint_velocities_);
  joint_safety_limiter_->setVelocityLimitingEnabled(true);  // MANTIENI

  // Acceleration limits (optional / DISATTIVABILE)
  pnh_.param("limits/acceleration_limiting_enabled", acceleration_limiting_enabled_, acceleration_limiting_enabled_);
  max_joint_accelerations_ = Eigen::VectorXd::Constant(static_cast<int>(n), 5.0);
  std::vector<double> amax;
  const bool has_amax_vec = pnh_.getParam("limits/max_joint_accelerations", amax);
  if (has_amax_vec && amax.size() == n)
  {
    for (std::size_t i = 0; i < n; ++i) max_joint_accelerations_[static_cast<Eigen::Index>(i)] = std::abs(amax[i]);
  }
  else if (!has_amax_vec || amax.empty())
  {
    double amax_scalar = 5.0;
    pnh_.param("limits/max_joint_acceleration", amax_scalar, amax_scalar);
    max_joint_accelerations_.setConstant(std::abs(amax_scalar));
  }
  else
  {
    ROS_WARN_NAMED("teleop_slave_controller",
                  "limits/max_joint_accelerations has size %zu, expected %zu. Using default 5.0 rad/s^2 each.",
                  amax.size(), n);
  }
  joint_safety_limiter_->setJointAccelerationLimits(max_joint_accelerations_);
  joint_safety_limiter_->setAccelerationLimitingEnabled(acceleration_limiting_enabled_);

  // Prepare previous qdot storage
  prev_qdot_cmd_.resize(static_cast<int>(n));
  prev_qdot_cmd_.setZero();

  setupRosInterfaces();
}

void TeleopSlaveController::loadParameters()
{
  pnh_.param("tf_prefix", tf_prefix_, tf_prefix_);

  pnh_.param("group_name", group_name_, std::string("manipulator"));
  pnh_.param("tcp_link", tcp_link_, std::string("tool0"));
  pnh_.param("robot_description_param", robot_description_param_, robot_description_param_);

  pnh_.param("target_pose_topic", target_pose_topic_, target_pose_topic_);
  pnh_.param("feedforward_twist_topic", feedforward_twist_topic_, feedforward_twist_topic_);
  pnh_.param("joint_state_topic", joint_state_topic_, joint_state_topic_);
  pnh_.param("velocity_command_topic", velocity_command_topic_, velocity_command_topic_);

  pnh_.param("control_rate", control_rate_, control_rate_);
  pnh_.param("k_ff", k_ff_, k_ff_);

  pnh_.param("target_pose_timeout", target_pose_timeout_, target_pose_timeout_);
  pnh_.param("feedforward_timeout", feedforward_timeout_, feedforward_timeout_);

  pnh_.param("deadband/enabled", deadband_enabled_, deadband_enabled_);
  pnh_.param("deadband/position_m", position_deadband_m_, position_deadband_m_);
  pnh_.param("deadband/orientation_rad", orientation_deadband_rad_, orientation_deadband_rad_);

  pnh_.param("tcp_pose_filter_alpha", tcp_pose_filter_alpha_, tcp_pose_filter_alpha_);
  pnh_.param("queue_size", queue_size_, queue_size_);
}

void TeleopSlaveController::setupRosInterfaces()
{
  sub_target_pose_ = nh_.subscribe(target_pose_topic_, queue_size_, &TeleopSlaveController::targetPoseCb, this);
  sub_ff_twist_ = nh_.subscribe(feedforward_twist_topic_, queue_size_, &TeleopSlaveController::feedforwardTwistCb, this);
  sub_joint_states_ = nh_.subscribe(joint_state_topic_, queue_size_, &TeleopSlaveController::jointStateCb, this);

  pub_qdot_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(velocity_command_topic_, 1);

  const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
  control_timer_ = nh_.createTimer(ros::Duration(period), &TeleopSlaveController::controlLoopCb, this, false, false);
}

void TeleopSlaveController::start()
{
  if (is_running_) return;
  is_running_ = true;
  last_control_time_ = ros::Time(0);
  has_prev_qdot_ = false;
  pid_pos_.reset();
  pid_ori_.reset();
  joint_safety_limiter_->reset();
  has_filtered_tcp_pose_ = false;
  control_timer_.start();
  ROS_INFO_NAMED("teleop_slave_controller", "Teleop slave controller started.");
}

void TeleopSlaveController::stop()
{
  if (!is_running_) return;
  is_running_ = false;
  control_timer_.stop();
  publishZeroVelocity("stop()");
  ROS_INFO_NAMED("teleop_slave_controller", "Teleop slave controller stopped.");
}

void TeleopSlaveController::targetPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  last_target_pose_ = *msg;
  last_target_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  has_target_pose_ = true;
}

void TeleopSlaveController::feedforwardTwistCb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  last_ff_twist_ = *msg;
  last_ff_twist_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  has_ff_twist_ = true;
}

void TeleopSlaveController::jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  if (!robot_state_manager_) return;
  robot_state_manager_->updateFromJointState(*msg);
}

bool TeleopSlaveController::tryGetInputs(geometry_msgs::PoseStamped& target_pose,
                                         geometry_msgs::TwistStamped& ff_twist,
                                         ros::Time& target_stamp,
                                         ros::Time& ff_stamp) const
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!has_target_pose_ || !has_ff_twist_) return false;
  target_pose = last_target_pose_;
  ff_twist = last_ff_twist_;
  target_stamp = last_target_pose_stamp_;
  ff_stamp = last_ff_twist_stamp_;
  return true;
}

bool TeleopSlaveController::transformTargetPoseToModelFrame(const geometry_msgs::PoseStamped& in,
                                                            const std::string& model_frame,
                                                            geometry_msgs::PoseStamped& out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == model_frame)
  {
    out = in;
    out.header.frame_id = model_frame;
    return true;
  }

  try
  {
    const ros::Time stamp = in.header.stamp.isZero() ? ros::Time(0) : in.header.stamp;
    const geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(model_frame, in.header.frame_id, stamp, ros::Duration(0.02));
    tf2::doTransform(in, out, T);
    out.header.frame_id = model_frame;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_controller", "TF failure transforming target pose: %s", ex.what());
    return false;
  }
}

bool TeleopSlaveController::rotateTwistToModelFrame(const geometry_msgs::TwistStamped& in,
                                                    const std::string& model_frame,
                                                    geometry_msgs::TwistStamped& out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == model_frame)
  {
    out = in;
    out.header.frame_id = model_frame;
    return true;
  }

  try
  {
    const ros::Time stamp = in.header.stamp.isZero() ? ros::Time(0) : in.header.stamp;
    const geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(model_frame, in.header.frame_id, stamp, ros::Duration(0.02));

    tf2::Quaternion q_tf;
    tf2::fromMsg(T.transform.rotation, q_tf);
    tf2::Matrix3x3 R(q_tf);

    geometry_msgs::TwistStamped rotated = in;
    rotated.header.frame_id = model_frame;

    const tf2::Vector3 v(in.twist.linear.x, in.twist.linear.y, in.twist.linear.z);
    const tf2::Vector3 w(in.twist.angular.x, in.twist.angular.y, in.twist.angular.z);
    const tf2::Vector3 v2 = R * v;
    const tf2::Vector3 w2 = R * w;

    rotated.twist.linear.x = v2.x();
    rotated.twist.linear.y = v2.y();
    rotated.twist.linear.z = v2.z();
    rotated.twist.angular.x = w2.x();
    rotated.twist.angular.y = w2.y();
    rotated.twist.angular.z = w2.z();

    out = rotated;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_controller", "TF failure rotating ff twist: %s", ex.what());
    return false;
  }
}

Eigen::Isometry3d TeleopSlaveController::poseMsgToEigen(const geometry_msgs::Pose& p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

geometry_msgs::Pose TeleopSlaveController::eigenToPoseMsg(const Eigen::Isometry3d& T)
{
  geometry_msgs::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond q(T.rotation());
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  return p;
}

Eigen::VectorXd TeleopSlaveController::twistMsgToEigen6(const geometry_msgs::Twist& t)
{
  Eigen::VectorXd v(6);
  v << t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z;
  return v;
}

void TeleopSlaveController::publishZeroVelocity(const std::string& reason)
{
  if (!pub_qdot_cmd_) return;

  const std::size_t n = robot_state_manager_ ? robot_state_manager_->getJointCount() : 0;
  std_msgs::Float64MultiArray msg;
  msg.data.resize(n, 0.0);
  pub_qdot_cmd_.publish(msg);
  ROS_DEBUG_THROTTLE_NAMED(1.0, "teleop_slave_controller", "Publishing zero velocity: %s", reason.c_str());
}

void TeleopSlaveController::controlLoopCb(const ros::TimerEvent& ev)
{
  if (!is_running_) return;

  const ros::Time now = ros::Time::now();
  const double dt = (!last_control_time_.isZero()) ? (now - last_control_time_).toSec() : 0.0;
  last_control_time_ = now;

  if (!robot_state_manager_ || !robot_state_manager_->isReady())
  {
    publishZeroVelocity("robot state not ready");
    return;
  }

  geometry_msgs::PoseStamped target_pose_msg;
  geometry_msgs::TwistStamped ff_twist_msg;
  ros::Time target_stamp, ff_stamp;
  if (!tryGetInputs(target_pose_msg, ff_twist_msg, target_stamp, ff_stamp))
  {
    publishZeroVelocity("missing inputs");
    return;
  }

  // Stale input timeout
  if ((now - target_stamp).toSec() > target_pose_timeout_ ||
      (now - ff_stamp).toSec() > feedforward_timeout_)
  {
    publishZeroVelocity("stale inputs");
    return;
  }

  const std::string model_root = robot_state_manager_->getModelRootFrame();
  if (model_root.empty())
  {
    publishZeroVelocity("empty model root frame");
    return;
  }

  // In multi-robot setups, TF frames are typically prefixed (e.g. "mur620_s/base_footprint"),
  // while the MoveIt RobotModel root link name is not. If tf_prefix is provided, use it
  // for TF lookups when transforming inputs to the model frame.
  std::string model_frame = model_root;
  if (!tf_prefix_.empty())
  {
    std::string p = tf_prefix_;
    while (!p.empty() && p.front() == '/') p.erase(0, 1);
    while (!p.empty() && p.back() == '/') p.pop_back();
    if (!p.empty())
    {
      model_frame = p + "/" + model_root;
    }
  }

  geometry_msgs::PoseStamped target_pose_model;
  if (!transformTargetPoseToModelFrame(target_pose_msg, model_frame, target_pose_model))
  {
    publishZeroVelocity("tf failure target pose");
    return;
  }

  geometry_msgs::TwistStamped ff_twist_model;
  if (!rotateTwistToModelFrame(ff_twist_msg, model_frame, ff_twist_model))
  {
    publishZeroVelocity("tf failure ff twist");
    return;
  }

  // FK TCP pose (raw)
  Eigen::Isometry3d tcp_pose_raw;
  if (!robot_state_manager_->computeTcpPose(tcp_offset_, tcp_pose_raw))
  {
    publishZeroVelocity("FK failure");
    return;
  }

  // EMA filter on measured TCP pose (MANTIENI)
  Eigen::Isometry3d tcp_pose = tcp_pose_raw;
  if (tcp_pose_filter_alpha_ > 0.0)
  {
    if (!has_filtered_tcp_pose_)
    {
      filtered_tcp_pose_ = tcp_pose_raw;
      has_filtered_tcp_pose_ = true;
    }
    else
    {
      filtered_tcp_pose_ = emaPose(filtered_tcp_pose_, tcp_pose_raw, tcp_pose_filter_alpha_);
    }
    tcp_pose = filtered_tcp_pose_;
  }

  const Eigen::Isometry3d target_pose = poseMsgToEigen(target_pose_model.pose);

  // Pose error
  Eigen::Vector3d e_p = target_pose.translation() - tcp_pose.translation();
  const Eigen::Quaterniond q_curr(tcp_pose.rotation());
  const Eigen::Quaterniond q_tgt(target_pose.rotation());
  Eigen::Vector3d e_o = orientationErrorAxisAngle(q_curr, q_tgt);

  if (deadband_enabled_)
  {
    e_p = applyDeadbandAbs(e_p, position_deadband_m_);
    e_o = applyDeadbandAbs(e_o, orientation_deadband_rad_);
  }

  // PID correction (small)
  const Eigen::VectorXd corr_p = (dt > 0.0) ? pid_pos_.compute(e_p, dt) : Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd corr_o = (dt > 0.0) ? pid_ori_.compute(e_o, dt) : Eigen::VectorXd::Zero(3);

  Eigen::VectorXd v_cmd(6);
  v_cmd.setZero();

  // Feedforward dominant
  const Eigen::VectorXd v_ff = twistMsgToEigen6(ff_twist_model.twist);
  v_cmd = k_ff_ * v_ff;
  v_cmd.head<3>() += corr_p;
  v_cmd.tail<3>() += corr_o;

  // Jacobian and damped pseudo-inverse
  Eigen::MatrixXd J;
  if (!robot_state_manager_->getJacobian(tcp_link_, Eigen::Vector3d::Zero(), J))
  {
    publishZeroVelocity("Jacobian failure");
    return;
  }
  if (J.rows() != 6 || J.cols() <= 0)
  {
    publishZeroVelocity("Jacobian size invalid");
    return;
  }

  const Eigen::MatrixXd J_pinv = jacobian_solver_.computeDampedPseudoInverse(J);
  if (J_pinv.size() == 0)
  {
    publishZeroVelocity("pseudo-inverse failure");
    return;
  }

  Eigen::VectorXd qdot_cmd = J_pinv * v_cmd;
  if (!qdot_cmd.allFinite())
  {
    publishZeroVelocity("non-finite qdot");
    return;
  }

  // Final joint safety limiter (uniform scaling)
  Eigen::VectorXd prev = prev_qdot_cmd_;
  if (!has_prev_qdot_)
  {
    prev.setZero();
  }
  const double limiter_dt = (dt > 0.0) ? dt : 0.0;
  const SafetyLimiterOutput lim = joint_safety_limiter_->limit(qdot_cmd, prev, limiter_dt);
  qdot_cmd = lim.joint_velocity;

  prev_qdot_cmd_ = qdot_cmd;
  has_prev_qdot_ = true;

  // Publish
  std_msgs::Float64MultiArray out;
  out.data.resize(static_cast<std::size_t>(qdot_cmd.size()));
  for (Eigen::Index i = 0; i < qdot_cmd.size(); ++i)
  {
    out.data[static_cast<std::size_t>(i)] = qdot_cmd[i];
  }
  pub_qdot_cmd_.publish(out);
}

}  // namespace teleoperation

