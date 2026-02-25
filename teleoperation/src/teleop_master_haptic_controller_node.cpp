#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <XmlRpcValue.h>

#include "teleoperation/core/math_utils.hpp"
#include "teleoperation/core/jerk_limiter.hpp"
#include "teleoperation/core/wrench_debug_publisher.hpp"
#include "teleoperation/core/tf_utils.hpp"
#include "teleoperation/core/types.hpp"
#include "teleoperation/core/wrench_utils.hpp"

using Wrench3 = teleoperation::Wrench3;

class TeleopMasterHapticController
{
public:
  TeleopMasterHapticController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("master_wrench_topic", master_wrench_topic_, "wrench");
    pnh_.param<std::string>("slave_wrench_topic", slave_wrench_topic_, "");
    pnh_.param<std::string>("coupling_wrench_topic", coupling_wrench_topic_, "");
    pnh_.param<std::string>("wrench_source_frame_override", wrench_source_frame_override_, "");
    pnh_.param<bool>("use_latest_tf_for_wrench", use_latest_tf_for_wrench_, use_latest_tf_for_wrench_);

    pnh_.param<std::string>("wrench_target_frame", wrench_target_frame_, "base_link");
    pnh_.param<double>("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);

    pnh_.param<std::string>("command_topic", command_topic_, "twist_controller/command");
    pnh_.param<double>("control_rate", control_rate_, control_rate_);

    pnh_.param<double>("mass_linear", mass_linear_, mass_linear_);
    pnh_.param<double>("damping_linear", damping_linear_, damping_linear_);
    pnh_.param<double>("mass_angular", mass_angular_, mass_angular_);
    pnh_.param<double>("damping_angular", damping_angular_, damping_angular_);

    pnh_.param<double>("force_reflection_scale", kf_force_, kf_force_);
    pnh_.param<double>("torque_reflection_scale", kf_torque_, kf_torque_);

    pnh_.param<bool>("use_forces", use_forces_, use_forces_);
    pnh_.param<bool>("use_torques", use_torques_, use_torques_);

    // Filtering
    pnh_.param<double>("wrench_filter_alpha", wrench_filter_alpha_, wrench_filter_alpha_);
    pnh_.param<double>("wrench_filter_cutoff_hz", wrench_filter_cutoff_hz_, wrench_filter_cutoff_hz_);
    pnh_.param<double>("master_wrench_filter_alpha", master_wrench_filter_alpha_, master_wrench_filter_alpha_);
    pnh_.param<double>("master_wrench_filter_cutoff_hz", master_wrench_filter_cutoff_hz_, master_wrench_filter_cutoff_hz_);
    pnh_.param<double>("feedback_wrench_filter_alpha", feedback_wrench_filter_alpha_, feedback_wrench_filter_alpha_);
    pnh_.param<double>("feedback_wrench_filter_cutoff_hz", feedback_wrench_filter_cutoff_hz_, feedback_wrench_filter_cutoff_hz_);

    // Deadzone on norm (soft) + optional hysteresis
    pnh_.param<double>("force_deadband", force_deadband_enter_, force_deadband_enter_);
    pnh_.param<double>("torque_deadband", torque_deadband_enter_, torque_deadband_enter_);
    (void)pnh_.getParam("force_deadband_enter", force_deadband_enter_);
    (void)pnh_.getParam("force_deadband_exit", force_deadband_exit_);
    (void)pnh_.getParam("torque_deadband_enter", torque_deadband_enter_);
    (void)pnh_.getParam("torque_deadband_exit", torque_deadband_exit_);
    if (!(force_deadband_exit_ >= 0.0)) force_deadband_exit_ = force_deadband_enter_;
    if (!(torque_deadband_exit_ >= 0.0)) torque_deadband_exit_ = torque_deadband_enter_;

    pnh_.param<double>("max_force", max_force_, max_force_);
    pnh_.param<double>("max_torque", max_torque_, max_torque_);
    pnh_.param<double>("max_force_hand", max_force_hand_, max_force_hand_);
    pnh_.param<double>("max_torque_hand", max_torque_hand_, max_torque_hand_);
    pnh_.param<double>("max_force_feedback", max_force_feedback_, max_force_feedback_);
    pnh_.param<double>("max_torque_feedback", max_torque_feedback_, max_torque_feedback_);

    pnh_.param<double>("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param<double>("max_angular_speed", max_angular_speed_, max_angular_speed_);

    // dt sanitization (measured dt from TimerEvent, then clamp/substep).
    pnh_.param<double>("dt_min_factor", dt_min_factor_, dt_min_factor_);
    pnh_.param<double>("dt_max_factor", dt_max_factor_, dt_max_factor_);
    pnh_.param<bool>("dt_use_substepping", dt_use_substepping_, dt_use_substepping_);
    pnh_.param<int>("dt_max_substeps", dt_max_substeps_, dt_max_substeps_);

    // Limits for smoothness (anti-windup coherent with jerk-limited acceleration).
    pnh_.param<double>("max_linear_accel", max_linear_accel_, max_linear_accel_);
    pnh_.param<double>("max_linear_jerk", max_linear_jerk_, max_linear_jerk_);
    pnh_.param<double>("max_angular_accel", max_angular_accel_, max_angular_accel_);
    pnh_.param<double>("max_angular_jerk", max_angular_jerk_, max_angular_jerk_);
    pnh_.param<double>("speed_saturation_eps", speed_saturation_eps_, speed_saturation_eps_);

    // Per-axis parameters (optional): override scalar mass/damping if provided.
    (void)tryGetVector3Param(pnh_, "mass_linear_xyz", mass_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "damping_linear_xyz", damping_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "mass_angular_xyz", mass_angular_xyz_);
    (void)tryGetVector3Param(pnh_, "damping_angular_xyz", damping_angular_xyz_);
    (void)tryGetVector3Param(pnh_, "max_linear_accel_xyz", max_linear_accel_xyz_);
    (void)tryGetVector3Param(pnh_, "max_linear_jerk_xyz", max_linear_jerk_xyz_);
    (void)tryGetVector3Param(pnh_, "max_angular_accel_xyz", max_angular_accel_xyz_);
    (void)tryGetVector3Param(pnh_, "max_angular_jerk_xyz", max_angular_jerk_xyz_);

    pnh_.param<double>("wrench_timeout_s", wrench_timeout_s_, wrench_timeout_s_);
    pnh_.param<bool>("reset_on_stale", reset_on_stale_, reset_on_stale_);

    // Slave target publishing (optional: pose + twist from master to slave)
    pnh_.param<bool>("publish_slave_targets", publish_slave_targets_, publish_slave_targets_);
    pnh_.param<std::string>("slave_target_pose_topic", slave_target_pose_topic_, slave_target_pose_topic_);
    pnh_.param<std::string>("slave_feedforward_twist_topic", slave_ff_twist_topic_, slave_ff_twist_topic_);
    pnh_.param<std::string>("slave_base_frame", slave_base_frame_, slave_base_frame_);
    pnh_.param<std::string>("slave_tcp_frame", slave_tcp_frame_, slave_tcp_frame_);
    pnh_.param<std::string>("slave_frame_id_override", slave_frame_id_override_, slave_frame_id_override_);
    pnh_.param<double>("slave_publish_rate", slave_publish_rate_, slave_publish_rate_);

    // Diagnostics
    pnh_.param<bool>("publish_diagnostics", publish_diagnostics_, publish_diagnostics_);
    pnh_.param<double>("diagnostics_rate", diagnostics_rate_, diagnostics_rate_);

    sub_master_wrench_ = nh_.subscribe(master_wrench_topic_, 1, &TeleopMasterHapticController::masterWrenchCb, this,
                                      ros::TransportHints().tcpNoDelay());

    if (!slave_wrench_topic_.empty())
    {
      sub_slave_wrench_ = nh_.subscribe(slave_wrench_topic_, 1, &TeleopMasterHapticController::slaveWrenchCb, this,
                                        ros::TransportHints().tcpNoDelay());
    }
    else
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller", "slave_wrench_topic is empty: force reflection disabled.");
    }

    if (!coupling_wrench_topic_.empty())
    {
      sub_coupling_wrench_ = nh_.subscribe(coupling_wrench_topic_, 1, &TeleopMasterHapticController::couplingWrenchCb, this,
                                           ros::TransportHints().tcpNoDelay());
    }

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    pub_cmd_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>(command_topic_ + "_stamped", 1);
    if (publish_diagnostics_)
    {
      pub_debug_stats_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/admittance_stats", 1);
      pub_debug_dt_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/dt_stats", 1);
      pub_debug_v_pre_ = nh_.advertise<geometry_msgs::TwistStamped>("debug/v_cmd_pre", 1);
      pub_debug_v_post_ = nh_.advertise<geometry_msgs::TwistStamped>("debug/v_cmd_post", 1);
    }
    debug_master_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                "filtered_master_wrench_topic", "debug/master_wrench_filtered");
    debug_slave_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                               "filtered_slave_wrench_topic", "debug/slave_wrench_filtered");
    debug_coupling_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                  "filtered_coupling_wrench_topic", "debug/coupling_wrench_filtered");

    if (publish_slave_targets_)
    {
      pub_slave_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(slave_target_pose_topic_, 1);
      pub_slave_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(slave_ff_twist_topic_, 1);

      const double slave_period = (slave_publish_rate_ > 0.0) ? (1.0 / slave_publish_rate_) : 0.01;
      slave_timer_ = nh_.createTimer(ros::Duration(slave_period),
                                     &TeleopMasterHapticController::slaveTargetTick, this);

      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Slave targets enabled: pose='%s', twist='%s' at %.0f Hz, TF %s->%s",
                     slave_target_pose_topic_.c_str(), slave_ff_twist_topic_.c_str(),
                     slave_publish_rate_, slave_base_frame_.c_str(), slave_tcp_frame_.c_str());
    }

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterHapticController::tick, this);
  }

private:
  static bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& v, double& out)
  {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out = static_cast<int>(v);
      return std::isfinite(out);
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      out = static_cast<double>(v);
      return std::isfinite(out);
    }
    return false;
  }

  static bool tryGetVector3Param(ros::NodeHandle& pnh, const std::string& name, Eigen::Vector3d& out)
  {
    if (!pnh.hasParam(name))
    {
      return false;
    }
    XmlRpc::XmlRpcValue v;
    if (!pnh.getParam(name, v))
    {
      return false;
    }
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() != 3)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' exists but is not a 3-element array. Ignoring.", name.c_str());
      return false;
    }

    double x = 0.0, y = 0.0, z = 0.0;
    if (!xmlRpcToDouble(v[0], x) || !xmlRpcToDouble(v[1], y) || !xmlRpcToDouble(v[2], z))
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' array must contain only int/double values. Ignoring.", name.c_str());
      return false;
    }
    out = Eigen::Vector3d(x, y, z);
    return out.allFinite();
  }

  static Eigen::Vector3d expandScalarTo3(double x)
  {
    return Eigen::Vector3d(x, x, x);
  }

  static Eigen::Vector3d sanitizePositiveVec(const Eigen::Vector3d& v, double min_val)
  {
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(out[i]) || out[i] < min_val)
      {
        out[i] = min_val;
      }
    }
    return out;
  }

  static Eigen::Vector3d sanitizeNonNegativeVec(const Eigen::Vector3d& v)
  {
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(out[i]) || out[i] < 0.0)
      {
        out[i] = 0.0;
      }
    }
    return out;
  }

  static Eigen::Vector3d softDeadzoneNormWithHysteresis(const Eigen::Vector3d& v,
                                                        double db_enter,
                                                        double db_exit,
                                                        bool& active)
  {
    const double n = v.norm();
    if (!std::isfinite(n))
    {
      active = false;
      return Eigen::Vector3d::Zero();
    }

    const double enter = std::max(0.0, db_enter);
    const double exit = std::max(0.0, db_exit);

    if (!active)
    {
      if (n <= enter)
      {
        return Eigen::Vector3d::Zero();
      }
      active = true;
      return teleoperation::softDeadzoneNorm3(v, enter);
    }

    if (n <= exit)
    {
      active = false;
      return Eigen::Vector3d::Zero();
    }
    return teleoperation::softDeadzoneNorm3(v, exit);
  }

  static double computeFilterAlpha(double dt, double alpha_param, double cutoff_hz_param)
  {
    if (cutoff_hz_param > 0.0 && std::isfinite(cutoff_hz_param))
    {
      return teleoperation::lowpassAlphaFromCutoffHz(dt, cutoff_hz_param);
    }
    return std::clamp(alpha_param, 0.0, 1.0);
  }

  void resetControllerState()
  {
    v_lin_cmd_.setZero();
    v_ang_cmd_.setZero();
    a_lin_limiter_.reset();
    a_ang_limiter_.reset();

    has_filtered_master_ = false;
    has_filtered_slave_ = false;
    has_filtered_coupling_ = false;

    f_master_active_ = false;
    tau_master_active_ = false;
    f_slave_active_ = false;
    tau_slave_active_ = false;
    f_coupling_active_ = false;
    tau_coupling_active_ = false;
  }

  bool wrenchMsgToWrench3(const geometry_msgs::WrenchStamped& msg, Wrench3& out) const
  {
    const Eigen::Vector3d f_src = teleoperation::vector3MsgToEigen(msg.wrench.force);
    const Eigen::Vector3d t_src = teleoperation::vector3MsgToEigen(msg.wrench.torque);

    Eigen::Vector3d f_tgt, t_tgt;
    const std::string src_frame = wrench_source_frame_override_.empty() ? msg.header.frame_id : wrench_source_frame_override_;
    const ros::Time stamp = use_latest_tf_for_wrench_ ? ros::Time(0) : msg.header.stamp;

    if (!teleoperation::rotateVectorToFrame(tf_buffer_, wrench_target_frame_, src_frame, stamp, tf_timeout_s_,
                                            f_src, f_tgt, "teleop_master_haptic_controller"))
    {
      return false;
    }
    if (!teleoperation::rotateVectorToFrame(tf_buffer_, wrench_target_frame_, src_frame, stamp, tf_timeout_s_,
                                            t_src, t_tgt, "teleop_master_haptic_controller"))
    {
      return false;
    }

    out.f = f_tgt;
    out.tau = t_tgt;
    return true;
  }

  void masterWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w))
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "masterWrenchCb: wrenchMsgToWrench3 failed (TF rotation from '%s' to '%s'). Dropping message.",
                              (wrench_source_frame_override_.empty() ? msg->header.frame_id : wrench_source_frame_override_).c_str(),
                              wrench_target_frame_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    master_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    master_wrench_raw_ = w;
    has_master_ = true;
  }

  void slaveWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    slave_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    slave_wrench_raw_ = w;
    has_slave_ = true;
  }

  void couplingWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    coupling_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    coupling_wrench_raw_ = w;
    has_coupling_ = true;
  }

  void publishZero()
  {
    geometry_msgs::Twist cmd;
    pub_cmd_.publish(cmd);
    if (pub_cmd_stamped_)
    {
      geometry_msgs::TwistStamped stamped;
      stamped.header.stamp = ros::Time::now();
      stamped.header.frame_id = wrench_target_frame_;
      stamped.twist = cmd;
      pub_cmd_stamped_.publish(stamped);
    }
  }

  void slaveTargetTick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

    // Lookup master TCP pose in its own base frame
    geometry_msgs::TransformStamped T;
    try
    {
      T = tf_buffer_.lookupTransform(slave_base_frame_, slave_tcp_frame_, ros::Time(0),
                                     ros::Duration(tf_timeout_s_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "Slave target TF lookup failed (%s -> %s): %s",
                              slave_base_frame_.c_str(), slave_tcp_frame_.c_str(), ex.what());
      return;
    }

    const std::string frame_out = slave_frame_id_override_.empty()
                                      ? slave_base_frame_
                                      : slave_frame_id_override_;

    // Publish PoseStamped
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = frame_out;
    pose_msg.pose.position.x = T.transform.translation.x;
    pose_msg.pose.position.y = T.transform.translation.y;
    pose_msg.pose.position.z = T.transform.translation.z;
    pose_msg.pose.orientation = T.transform.rotation;
    pub_slave_pose_.publish(pose_msg);

    // Publish commanded twist (already computed by admittance loop)
    Eigen::Vector3d v_lin, v_ang;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v_lin = v_lin_cmd_;
      v_ang = v_ang_cmd_;
    }

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = frame_out;
    twist_msg.twist.linear.x = v_lin.x();
    twist_msg.twist.linear.y = v_lin.y();
    twist_msg.twist.linear.z = v_lin.z();
    twist_msg.twist.angular.x = v_ang.x();
    twist_msg.twist.angular.y = v_ang.y();
    twist_msg.twist.angular.z = v_ang.z();
    pub_slave_twist_.publish(twist_msg);
  }

  void tick(const ros::TimerEvent& ev)
  {
    const ros::Time now = ev.current_real.isZero() ? ros::Time::now() : ev.current_real;

    Wrench3 master_raw, slave_raw, coupling_raw;
    ros::Time master_stamp, slave_stamp, coupling_stamp;
    bool has_slave = false;
    bool has_coupling = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_master_)
      {
        publishZero();
        return;
      }
      master_raw = master_wrench_raw_;
      master_stamp = master_stamp_;

      has_slave = has_slave_;
      slave_raw = slave_wrench_raw_;
      slave_stamp = slave_stamp_;

      has_coupling = has_coupling_;
      coupling_raw = coupling_wrench_raw_;
      coupling_stamp = coupling_stamp_;
    }

    const bool master_stale = ((now - master_stamp).toSec() > wrench_timeout_s_);
    const bool slave_stale = has_slave && ((now - slave_stamp).toSec() > wrench_timeout_s_);
    const bool coupling_stale = has_coupling && ((now - coupling_stamp).toSec() > wrench_timeout_s_);

    if (master_stale || (has_slave && slave_stale) || (has_coupling && coupling_stale))
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "Stale wrench (master=%d slave=%d coupling=%d). Publishing zero.",
                              master_stale, slave_stale, coupling_stale);
      publishZero();
      if (reset_on_stale_)
      {
        resetControllerState();
      }
      last_time_ = now;
      return;
    }

    const double dt_nominal = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    const double dt_min = std::max(0.0, dt_min_factor_) * dt_nominal;
    const double dt_max = std::max(0.0, dt_max_factor_) * dt_nominal;

    double dt_raw = 0.0;
    if (!ev.last_real.isZero() && !ev.current_real.isZero())
    {
      dt_raw = (ev.current_real - ev.last_real).toSec();
    }
    else if (!last_time_.isZero())
    {
      dt_raw = (now - last_time_).toSec();
    }
    last_time_ = now;

    if (!(dt_raw > 0.0) || !std::isfinite(dt_raw))
    {
      publishZero();
      return;
    }

    double dt_used = dt_raw;
    if (dt_min > 0.0 && dt_used < dt_min)
    {
      dt_used = dt_nominal;
    }

    int n_substeps = 1;
    if (dt_max > 0.0 && dt_used > dt_max)
    {
      if (dt_use_substepping_)
      {
        n_substeps = static_cast<int>(std::ceil(dt_used / dt_max));
        n_substeps = std::clamp(n_substeps, 1, std::max(1, dt_max_substeps_));
      }
      else
      {
        dt_used = dt_max;
        n_substeps = 1;
      }
    }
    const double dt_step = dt_used / static_cast<double>(n_substeps);
    if (!(dt_step > 0.0) || !std::isfinite(dt_step))
    {
      publishZero();
      return;
    }

    // Filter + soft deadzone (norm, with optional hysteresis) + clamp on norm.
    // NOTE: 3.4 is intentionally rotation-only: wrenchMsgToWrench3 rotates vectors but does not apply p x f.
    const double dt_for_filter = std::clamp(dt_raw, std::max(1e-6, dt_min), (dt_max > 0.0 ? dt_max : dt_raw));
    const double alpha_master =
        computeFilterAlpha(dt_for_filter,
                           (master_wrench_filter_alpha_ >= 0.0 ? master_wrench_filter_alpha_ : wrench_filter_alpha_),
                           (master_wrench_filter_cutoff_hz_ > 0.0 ? master_wrench_filter_cutoff_hz_ : wrench_filter_cutoff_hz_));
    const double alpha_feedback =
        computeFilterAlpha(dt_for_filter,
                           (feedback_wrench_filter_alpha_ >= 0.0 ? feedback_wrench_filter_alpha_ : wrench_filter_alpha_),
                           (feedback_wrench_filter_cutoff_hz_ > 0.0 ? feedback_wrench_filter_cutoff_hz_ : wrench_filter_cutoff_hz_));

    const bool use_master_filter = (alpha_master > 0.0) && (alpha_master < 1.0);
    const bool use_feedback_filter = (alpha_feedback > 0.0) && (alpha_feedback < 1.0);

    if (!has_filtered_master_)
    {
      master_filt_ = master_raw;
      has_filtered_master_ = true;
    }
    else
    {
      if (use_master_filter)
      {
        master_filt_.f = teleoperation::ema3(master_filt_.f, master_raw.f, alpha_master);
        master_filt_.tau = teleoperation::ema3(master_filt_.tau, master_raw.tau, alpha_master);
      }
      else
      {
        master_filt_ = master_raw;
      }
    }
    master_filt_.f = softDeadzoneNormWithHysteresis(master_filt_.f, force_deadband_enter_, force_deadband_exit_, f_master_active_);
    master_filt_.f = teleoperation::clampNorm3(master_filt_.f, (max_force_hand_ > 0.0 ? max_force_hand_ : max_force_));
    if (!use_forces_)
    {
      master_filt_.f.setZero();
      f_master_active_ = false;
    }
    if (use_torques_)
    {
      master_filt_.tau = softDeadzoneNormWithHysteresis(master_filt_.tau, torque_deadband_enter_, torque_deadband_exit_, tau_master_active_);
      master_filt_.tau = teleoperation::clampNorm3(master_filt_.tau, (max_torque_hand_ > 0.0 ? max_torque_hand_ : max_torque_));
    }
    else
    {
      master_filt_.tau.setZero();
    }

    if (has_slave)
    {
      if (!has_filtered_slave_)
      {
        slave_filt_ = slave_raw;
        has_filtered_slave_ = true;
      }
      else
      {
        if (use_feedback_filter)
        {
          slave_filt_.f = teleoperation::ema3(slave_filt_.f, slave_raw.f, alpha_feedback);
          slave_filt_.tau = teleoperation::ema3(slave_filt_.tau, slave_raw.tau, alpha_feedback);
        }
        else
        {
          slave_filt_ = slave_raw;
        }
      }
      slave_filt_.f = softDeadzoneNormWithHysteresis(slave_filt_.f, force_deadband_enter_, force_deadband_exit_, f_slave_active_);
      slave_filt_.f = teleoperation::clampNorm3(slave_filt_.f, (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_));
      if (!use_forces_)
      {
        slave_filt_.f.setZero();
        f_slave_active_ = false;
      }
      if (use_torques_)
      {
        slave_filt_.tau = softDeadzoneNormWithHysteresis(slave_filt_.tau, torque_deadband_enter_, torque_deadband_exit_, tau_slave_active_);
        slave_filt_.tau = teleoperation::clampNorm3(slave_filt_.tau, (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_));
      }
      else
      {
        slave_filt_.tau.setZero();
      }
    }
    else
    {
      slave_filt_.f.setZero();
      slave_filt_.tau.setZero();
      f_slave_active_ = false;
      tau_slave_active_ = false;
    }

    if (has_coupling)
    {
      if (!has_filtered_coupling_)
      {
        coupling_filt_ = coupling_raw;
        has_filtered_coupling_ = true;
      }
      else
      {
        if (use_feedback_filter)
        {
          coupling_filt_.f = teleoperation::ema3(coupling_filt_.f, coupling_raw.f, alpha_feedback);
          coupling_filt_.tau = teleoperation::ema3(coupling_filt_.tau, coupling_raw.tau, alpha_feedback);
        }
        else
        {
          coupling_filt_ = coupling_raw;
        }
      }
      coupling_filt_.f = softDeadzoneNormWithHysteresis(coupling_filt_.f, force_deadband_enter_, force_deadband_exit_, f_coupling_active_);
      coupling_filt_.f = teleoperation::clampNorm3(coupling_filt_.f, (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_));
      if (!use_forces_)
      {
        coupling_filt_.f.setZero();
        f_coupling_active_ = false;
      }
      if (use_torques_)
      {
        coupling_filt_.tau =
            softDeadzoneNormWithHysteresis(coupling_filt_.tau, torque_deadband_enter_, torque_deadband_exit_, tau_coupling_active_);
        coupling_filt_.tau = teleoperation::clampNorm3(coupling_filt_.tau, (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_));
      }
      else
      {
        coupling_filt_.tau.setZero();
      }
    }
    else
    {
      coupling_filt_.f.setZero();
      coupling_filt_.tau.setZero();
      f_coupling_active_ = false;
      tau_coupling_active_ = false;
    }

    debug_master_filt_pub_.publish(master_filt_, now, wrench_target_frame_);
    debug_slave_filt_pub_.publish(slave_filt_, now, wrench_target_frame_);
    debug_coupling_filt_pub_.publish(coupling_filt_, now, wrench_target_frame_);

    // Admittance dynamics (linear + optional angular).
    const Eigen::Vector3d F_hand = master_filt_.f;
    const Eigen::Vector3d Tau_hand = master_filt_.tau;

    // Force reflection: slave FT typically measures the wrench applied *on the slave tool* by the environment.
    // To obtain an opposing reflected contribution at the master, we invert the slave wrench sign here.
    const Eigen::Vector3d F_feedback = (-kf_force_ * slave_filt_.f) + coupling_filt_.f;
    const Eigen::Vector3d Tau_feedback = (-kf_torque_ * slave_filt_.tau) + coupling_filt_.tau;

    // Per-axis admittance: M dv + D v = (F_hand - F_feedback)
    const Eigen::Vector3d M_lin = sanitizePositiveVec((mass_linear_xyz_.allFinite() ? mass_linear_xyz_ : expandScalarTo3(mass_linear_)), 1e-6);
    const Eigen::Vector3d D_lin = sanitizeNonNegativeVec((damping_linear_xyz_.allFinite() ? damping_linear_xyz_ : expandScalarTo3(damping_linear_)));
    const Eigen::Vector3d M_ang = sanitizePositiveVec((mass_angular_xyz_.allFinite() ? mass_angular_xyz_ : expandScalarTo3(mass_angular_)), 1e-6);
    const Eigen::Vector3d D_ang = sanitizeNonNegativeVec((damping_angular_xyz_.allFinite() ? damping_angular_xyz_ : expandScalarTo3(damping_angular_)));

    // Limits (accel/jerk). Scalars are expanded unless *_xyz override exists.
    const Eigen::Vector3d max_a_lin = sanitizeNonNegativeVec((max_linear_accel_xyz_.allFinite() ? max_linear_accel_xyz_ : expandScalarTo3(max_linear_accel_)));
    const Eigen::Vector3d max_j_lin = sanitizeNonNegativeVec((max_linear_jerk_xyz_.allFinite() ? max_linear_jerk_xyz_ : expandScalarTo3(max_linear_jerk_)));
    const Eigen::Vector3d max_a_ang = sanitizeNonNegativeVec((max_angular_accel_xyz_.allFinite() ? max_angular_accel_xyz_ : expandScalarTo3(max_angular_accel_)));
    const Eigen::Vector3d max_j_ang = sanitizeNonNegativeVec((max_angular_jerk_xyz_.allFinite() ? max_angular_jerk_xyz_ : expandScalarTo3(max_angular_jerk_)));

    Eigen::Vector3d v_lin_pre = v_lin_cmd_;
    Eigen::Vector3d v_ang_pre = v_ang_cmd_;

    Eigen::Vector3d a_lin_des_normed = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_ang_des_normed = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_lin_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_ang_cmd = Eigen::Vector3d::Zero();
    bool lin_saturated = false;
    bool ang_saturated = false;

    for (int k = 0; k < n_substeps; ++k)
    {
      const Eigen::Vector3d rhs_lin = (F_hand - F_feedback) - D_lin.cwiseProduct(v_lin_cmd_);
      a_lin_des_normed = rhs_lin.cwiseQuotient(M_lin);

      // 3.1 + 3.2: jerk-limited acceleration + anti-windup against speed saturation.
      const Eigen::Vector3d max_a_lin_eff = (max_a_lin.maxCoeff() > 0.0 ? max_a_lin : expandScalarTo3(1e9));
      const Eigen::Vector3d max_j_lin_eff = (max_j_lin.maxCoeff() > 0.0 ? max_j_lin : expandScalarTo3(1e9));
      a_lin_cmd = a_lin_limiter_.step(a_lin_des_normed, dt_step, max_a_lin_eff, max_j_lin_eff);

      if (max_linear_speed_ > 0.0 && v_lin_cmd_.norm() >= (max_linear_speed_ - std::max(0.0, speed_saturation_eps_)))
      {
        const double vn = v_lin_cmd_.norm();
        if (vn > teleoperation::kMathEps)
        {
          const Eigen::Vector3d u = v_lin_cmd_ / vn;
          const double a_rad = u.dot(a_lin_cmd);
          if (a_rad > 0.0)
          {
            a_lin_cmd = a_lin_cmd - a_rad * u;
          }
        }
      }

      const Eigen::Vector3d v_lin_next = v_lin_cmd_ + a_lin_cmd * dt_step;
      const Eigen::Vector3d v_lin_sat = teleoperation::clampNorm3(v_lin_next, max_linear_speed_);
      lin_saturated = lin_saturated || ((v_lin_next - v_lin_sat).norm() > 1e-12);
      v_lin_cmd_ = v_lin_sat;

      if (use_torques_)
      {
        const Eigen::Vector3d rhs_ang = (Tau_hand - Tau_feedback) - D_ang.cwiseProduct(v_ang_cmd_);
        a_ang_des_normed = rhs_ang.cwiseQuotient(M_ang);

        const Eigen::Vector3d max_a_ang_eff = (max_a_ang.maxCoeff() > 0.0 ? max_a_ang : expandScalarTo3(1e9));
        const Eigen::Vector3d max_j_ang_eff = (max_j_ang.maxCoeff() > 0.0 ? max_j_ang : expandScalarTo3(1e9));
        a_ang_cmd = a_ang_limiter_.step(a_ang_des_normed, dt_step, max_a_ang_eff, max_j_ang_eff);

        if (max_angular_speed_ > 0.0 && v_ang_cmd_.norm() >= (max_angular_speed_ - std::max(0.0, speed_saturation_eps_)))
        {
          const double wn = v_ang_cmd_.norm();
          if (wn > teleoperation::kMathEps)
          {
            const Eigen::Vector3d u = v_ang_cmd_ / wn;
            const double a_rad = u.dot(a_ang_cmd);
            if (a_rad > 0.0)
            {
              a_ang_cmd = a_ang_cmd - a_rad * u;
            }
          }
        }

        const Eigen::Vector3d v_ang_next = v_ang_cmd_ + a_ang_cmd * dt_step;
        const Eigen::Vector3d v_ang_sat = teleoperation::clampNorm3(v_ang_next, max_angular_speed_);
        ang_saturated = ang_saturated || ((v_ang_next - v_ang_sat).norm() > 1e-12);
        v_ang_cmd_ = v_ang_sat;
      }
      else
      {
        v_ang_cmd_.setZero();
        a_ang_limiter_.reset();
      }

      if (!use_forces_)
      {
        v_lin_cmd_.setZero();
        a_lin_limiter_.reset();
      }
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = v_lin_cmd_.x();
    cmd.linear.y = v_lin_cmd_.y();
    cmd.linear.z = v_lin_cmd_.z();
    cmd.angular.x = v_ang_cmd_.x();
    cmd.angular.y = v_ang_cmd_.y();
    cmd.angular.z = v_ang_cmd_.z();
    pub_cmd_.publish(cmd);

    if (pub_cmd_stamped_)
    {
      geometry_msgs::TwistStamped stamped;
      stamped.header.stamp = now;
      stamped.header.frame_id = wrench_target_frame_;
      stamped.twist = cmd;
      pub_cmd_stamped_.publish(stamped);
    }

    // 3.8 diagnostics
    if (publish_diagnostics_)
    {
      // Throttle diagnostics to diagnostics_rate_
      const double diag_period = (diagnostics_rate_ > 0.0) ? (1.0 / diagnostics_rate_) : 0.0;
      if (diag_period <= 0.0 || (now - last_diag_pub_).toSec() >= diag_period)
      {
        last_diag_pub_ = now;

        dt_min_seen_ = (dt_count_ == 0) ? dt_raw : std::min(dt_min_seen_, dt_raw);
        dt_max_seen_ = (dt_count_ == 0) ? dt_raw : std::max(dt_max_seen_, dt_raw);
        dt_count_ += 1;
        dt_mean_ += (dt_raw - dt_mean_) / static_cast<double>(dt_count_);

        std_msgs::Float64MultiArray dt_msg;
        dt_msg.data = {dt_raw, dt_used, dt_step, static_cast<double>(n_substeps), dt_min_seen_, dt_max_seen_, dt_mean_};
        pub_debug_dt_.publish(dt_msg);

        std_msgs::Float64MultiArray st_msg;
        st_msg.data = {
            F_hand.norm(),
            F_feedback.norm(),
            a_lin_des_normed.norm(),
            a_lin_cmd.norm(),
            v_lin_pre.norm(),
            v_lin_cmd_.norm(),
            lin_saturated ? 1.0 : 0.0,
            Tau_hand.norm(),
            Tau_feedback.norm(),
            a_ang_des_normed.norm(),
            a_ang_cmd.norm(),
            v_ang_pre.norm(),
            v_ang_cmd_.norm(),
            ang_saturated ? 1.0 : 0.0};
        pub_debug_stats_.publish(st_msg);

        geometry_msgs::TwistStamped vpre, vpost;
        vpre.header.stamp = now;
        vpre.header.frame_id = wrench_target_frame_;
        vpre.twist.linear.x = v_lin_pre.x();
        vpre.twist.linear.y = v_lin_pre.y();
        vpre.twist.linear.z = v_lin_pre.z();
        vpre.twist.angular.x = v_ang_pre.x();
        vpre.twist.angular.y = v_ang_pre.y();
        vpre.twist.angular.z = v_ang_pre.z();
        pub_debug_v_pre_.publish(vpre);

        vpost.header = vpre.header;
        vpost.twist = cmd;
        pub_debug_v_post_.publish(vpost);
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_master_wrench_;
  ros::Subscriber sub_slave_wrench_;
  ros::Subscriber sub_coupling_wrench_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_cmd_stamped_;
  teleoperation::WrenchDebugPublisher debug_master_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_slave_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_coupling_filt_pub_;
  ros::Publisher pub_debug_stats_;
  ros::Publisher pub_debug_dt_;
  ros::Publisher pub_debug_v_pre_;
  ros::Publisher pub_debug_v_post_;
  ros::Timer timer_;

  // Slave target publishers
  ros::Publisher pub_slave_pose_;
  ros::Publisher pub_slave_twist_;
  ros::Timer slave_timer_;

  // Params
  std::string master_wrench_topic_;
  std::string slave_wrench_topic_;
  std::string coupling_wrench_topic_;
  std::string wrench_source_frame_override_;
  bool use_latest_tf_for_wrench_{false};
  std::string wrench_target_frame_;
  double tf_timeout_s_{0.02};

  std::string command_topic_;
  double control_rate_{250.0};

  double mass_linear_{4.0};
  double damping_linear_{40.0};
  double mass_angular_{1.0};
  double damping_angular_{5.0};
  Eigen::Vector3d mass_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d damping_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d mass_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d damping_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};

  double kf_force_{0.3};
  double kf_torque_{0.0};
  bool use_forces_{true};
  bool use_torques_{false};

  double wrench_filter_alpha_{0.07};
  double wrench_filter_cutoff_hz_{0.0};
  double master_wrench_filter_alpha_{-1.0};
  double master_wrench_filter_cutoff_hz_{0.0};
  double feedback_wrench_filter_alpha_{-1.0};
  double feedback_wrench_filter_cutoff_hz_{0.0};

  double force_deadband_enter_{1.0};
  double force_deadband_exit_{std::numeric_limits<double>::quiet_NaN()};
  double torque_deadband_enter_{0.2};
  double torque_deadband_exit_{std::numeric_limits<double>::quiet_NaN()};

  double max_force_{150.0};
  double max_torque_{20.0};
  double max_force_hand_{-1.0};
  double max_torque_hand_{-1.0};
  double max_force_feedback_{-1.0};
  double max_torque_feedback_{-1.0};

  double max_linear_speed_{0.25};
  double max_angular_speed_{0.4};

  double dt_min_factor_{0.25};
  double dt_max_factor_{2.0};
  bool dt_use_substepping_{true};
  int dt_max_substeps_{10};

  double max_linear_accel_{0.0};
  double max_linear_jerk_{0.0};
  double max_angular_accel_{0.0};
  double max_angular_jerk_{0.0};
  Eigen::Vector3d max_linear_accel_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_linear_jerk_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_angular_accel_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_angular_jerk_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double speed_saturation_eps_{1e-3};

  double wrench_timeout_s_{0.2};
  bool reset_on_stale_{true};

  // Slave target publishing
  bool publish_slave_targets_{false};
  std::string slave_target_pose_topic_{"target_pose"};
  std::string slave_ff_twist_topic_{"feedforward_twist"};
  std::string slave_base_frame_{"base_link"};
  std::string slave_tcp_frame_{"tool0"};
  std::string slave_frame_id_override_;
  double slave_publish_rate_{250.0};

  bool publish_diagnostics_{true};
  double diagnostics_rate_{50.0};

  // Inputs (raw)
  mutable std::mutex mutex_;
  bool has_master_{false};
  bool has_slave_{false};
  bool has_coupling_{false};
  Wrench3 master_wrench_raw_;
  Wrench3 slave_wrench_raw_;
  Wrench3 coupling_wrench_raw_;
  ros::Time master_stamp_{0};
  ros::Time slave_stamp_{0};
  ros::Time coupling_stamp_{0};

  // Filtered
  bool has_filtered_master_{false};
  bool has_filtered_slave_{false};
  bool has_filtered_coupling_{false};
  Wrench3 master_filt_;
  Wrench3 slave_filt_;
  Wrench3 coupling_filt_;

  // Controller state
  ros::Time last_time_{0};
  ros::Time last_diag_pub_{0};
  double dt_min_seen_{0.0};
  double dt_max_seen_{0.0};
  double dt_mean_{0.0};
  uint64_t dt_count_{0};

  Eigen::Vector3d v_lin_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_cmd_{Eigen::Vector3d::Zero()};

  teleoperation::JerkLimiter3 a_lin_limiter_;
  teleoperation::JerkLimiter3 a_ang_limiter_;

  bool f_master_active_{false};
  bool tau_master_active_{false};
  bool f_slave_active_{false};
  bool tau_slave_active_{false};
  bool f_coupling_active_{false};
  bool tau_coupling_active_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_master_haptic_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopMasterHapticController node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_master_haptic_controller failed: %s", ex.what());
    return 1;
  }
  return 0;
}

