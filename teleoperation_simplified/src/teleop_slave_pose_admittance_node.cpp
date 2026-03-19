#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "teleoperation_simplified/core/jerk_limiter.hpp"
#include "teleoperation_simplified/core/math_utils.hpp"
#include "teleoperation_simplified/core/tf_utils.hpp"
#include "teleoperation_simplified/core/types.hpp"
#include "teleoperation_simplified/core/wrench_debug_publisher.hpp"
#include "teleoperation_simplified/core/wrench_utils.hpp"

namespace
{
using Wrench3 = teleoperation_simplified::Wrench3;

Eigen::Vector3d optionalClampNorm(const Eigen::Vector3d& v, double max_norm)
{
  if (!(max_norm > 0.0) || !std::isfinite(max_norm))
  {
    return v;
  }
  return teleoperation_simplified::clampNorm3(v, max_norm);
}

Eigen::Vector3d accelLimitVector(double max_value)
{
  if (!(max_value > 0.0) || !std::isfinite(max_value))
  {
    return teleoperation_simplified::expandScalarTo3(1e9);
  }
  return teleoperation_simplified::expandScalarTo3(max_value);
}
}  // namespace

class TeleopSlavePoseAdmittance
{
public:
  TeleopSlavePoseAdmittance(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "target_pose");
    pnh_.param<std::string>("wrench_topic", wrench_topic_, "wrench");
    pnh_.param<std::string>("command_topic", command_topic_, "twist_controller/command");
    pnh_.param<std::string>("feedback_pose_topic", feedback_pose_topic_, "slave_actual_pose");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("tcp_frame", tcp_frame_, "tool0");
    pnh_.param<std::string>("feedback_frame_id_override", feedback_frame_id_override_, "");
    pnh_.param<std::string>("wrench_source_frame_override", wrench_source_frame_override_, "");
    pnh_.param<bool>("use_latest_tf_for_wrench", use_latest_tf_for_wrench_, true);

    pnh_.param("control_rate", control_rate_, control_rate_);
    pnh_.param("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);
    pnh_.param("target_pose_timeout", target_pose_timeout_, target_pose_timeout_);
    pnh_.param("wrench_timeout", wrench_timeout_, wrench_timeout_);

    pnh_.param("enable_translation", enable_translation_, enable_translation_);
    pnh_.param("enable_rotation", enable_rotation_, enable_rotation_);
    pnh_.param("mass_linear", mass_linear_, mass_linear_);
    pnh_.param("mass_angular", mass_angular_, mass_angular_);
    pnh_.param("stiffness_linear", stiffness_linear_, stiffness_linear_);
    pnh_.param("damping_linear", damping_linear_, damping_linear_);
    pnh_.param("stiffness_angular", stiffness_angular_, stiffness_angular_);
    pnh_.param("damping_angular", damping_angular_, damping_angular_);

    pnh_.param("wrench_filter_cutoff_hz", wrench_filter_cutoff_hz_, wrench_filter_cutoff_hz_);
    pnh_.param("force_deadband_enter", force_deadband_enter_, force_deadband_enter_);
    pnh_.param("force_deadband_exit", force_deadband_exit_, force_deadband_exit_);
    pnh_.param("torque_deadband_enter", torque_deadband_enter_, torque_deadband_enter_);
    pnh_.param("torque_deadband_exit", torque_deadband_exit_, torque_deadband_exit_);
    pnh_.param("cross_deadband_scale", cross_deadband_scale_, cross_deadband_scale_);
    pnh_.param("max_force", max_force_, max_force_);
    pnh_.param("max_torque", max_torque_, max_torque_);

    pnh_.param("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);
    pnh_.param("max_linear_accel", max_linear_accel_, max_linear_accel_);
    pnh_.param("max_angular_accel", max_angular_accel_, max_angular_accel_);
    pnh_.param("max_linear_jerk", max_linear_jerk_, max_linear_jerk_);
    pnh_.param("max_angular_jerk", max_angular_jerk_, max_angular_jerk_);
    pnh_.param("dt_min_factor", dt_min_factor_, dt_min_factor_);
    pnh_.param("dt_max_factor", dt_max_factor_, dt_max_factor_);
    pnh_.param("dt_use_substepping", dt_use_substepping_, dt_use_substepping_);
    pnh_.param("dt_max_substeps", dt_max_substeps_, dt_max_substeps_);

    sub_target_pose_ = nh_.subscribe(target_pose_topic_, 1, &TeleopSlavePoseAdmittance::targetPoseCb, this,
                                     ros::TransportHints().tcpNoDelay());
    sub_wrench_ = nh_.subscribe(wrench_topic_, 1, &TeleopSlavePoseAdmittance::wrenchCb, this,
                                ros::TransportHints().tcpNoDelay());

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    pub_feedback_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(feedback_pose_topic_, 1);
    debug_wrench_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                           "filtered_wrench_topic", "debug/slave_wrench_filtered");

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopSlavePoseAdmittance::tick, this);
    feedback_timer_ = nh_.createTimer(ros::Duration(period), &TeleopSlavePoseAdmittance::feedbackPoseTick, this);
  }

private:
  bool getTcpPose(Eigen::Isometry3d& t_base_tcp) const
  {
    try
    {
      const geometry_msgs::TransformStamped t =
          tf_buffer_.lookupTransform(base_frame_, tcp_frame_, ros::Time(0), ros::Duration(tf_timeout_s_));

      Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
      out.translation() = Eigen::Vector3d(t.transform.translation.x,
                                          t.transform.translation.y,
                                          t.transform.translation.z);

      Eigen::Quaterniond q(t.transform.rotation.w,
                           t.transform.rotation.x,
                           t.transform.rotation.y,
                           t.transform.rotation.z);
      if (q.norm() < teleoperation_simplified::kMathEps)
      {
        return false;
      }
      q.normalize();
      out.linear() = q.toRotationMatrix();
      t_base_tcp = out;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_pose_admittance", "TF TCP pose failed: %s", ex.what());
      return false;
    }
  }

  bool transformPoseToBase(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out) const
  {
    if (in.header.frame_id.empty() || in.header.frame_id == base_frame_)
    {
      out = in;
      out.header.frame_id = base_frame_;
      return true;
    }

    try
    {
      const ros::Time stamp = in.header.stamp.isZero() ? ros::Time(0) : in.header.stamp;
      const geometry_msgs::TransformStamped t =
          tf_buffer_.lookupTransform(base_frame_, in.header.frame_id, stamp, ros::Duration(tf_timeout_s_));
      tf2::doTransform(in, out, t);
      out.header.frame_id = base_frame_;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_pose_admittance", "TF pose transform failed: %s", ex.what());
      return false;
    }
  }

  bool wrenchMsgToWrench3(const geometry_msgs::WrenchStamped& msg, Wrench3& out) const
  {
    const Eigen::Vector3d f_src = teleoperation_simplified::vector3MsgToEigen(msg.wrench.force);
    const Eigen::Vector3d t_src = teleoperation_simplified::vector3MsgToEigen(msg.wrench.torque);

    const std::string src_frame = wrench_source_frame_override_.empty() ? msg.header.frame_id : wrench_source_frame_override_;
    const ros::Time stamp = use_latest_tf_for_wrench_ ? ros::Time(0) : msg.header.stamp;

    Eigen::Vector3d f_base;
    Eigen::Vector3d t_base;
    if (!teleoperation_simplified::rotateVectorToFrame(tf_buffer_, base_frame_, src_frame, stamp, tf_timeout_s_,
                                                       f_src, f_base, "teleop_slave_pose_admittance"))
    {
      return false;
    }
    if (!teleoperation_simplified::rotateVectorToFrame(tf_buffer_, base_frame_, src_frame, stamp, tf_timeout_s_,
                                                       t_src, t_base, "teleop_slave_pose_admittance"))
    {
      return false;
    }

    out.f = f_base;
    out.tau = t_base;
    return true;
  }

  void targetPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pose_msg_ = *msg;
    target_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_target_pose_ = true;
  }

  void wrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 wrench;
    if (!wrenchMsgToWrench3(*msg, wrench))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    wrench_raw_ = wrench;
    wrench_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_wrench_ = true;
  }

  void publishZero()
  {
    geometry_msgs::Twist cmd;
    pub_cmd_.publish(cmd);
  }

  void resetCommandState()
  {
    v_lin_cmd_.setZero();
    v_ang_cmd_.setZero();
    a_lin_limiter_.reset();
    a_ang_limiter_.reset();
    has_filtered_wrench_ = false;
    wrench_db_state_ = teleoperation_simplified::WrenchDeadbandState{};
  }

  void feedbackPoseTick(const ros::TimerEvent&)
  {
    Eigen::Isometry3d t_base_tcp;
    if (!getTcpPose(t_base_tcp))
    {
      return;
    }

    Eigen::Quaterniond q(t_base_tcp.rotation());
    q.normalize();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = feedback_frame_id_override_.empty() ? base_frame_ : feedback_frame_id_override_;
    pose_msg.pose.position.x = t_base_tcp.translation().x();
    pose_msg.pose.position.y = t_base_tcp.translation().y();
    pose_msg.pose.position.z = t_base_tcp.translation().z();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pub_feedback_pose_.publish(pose_msg);
  }

  void tick(const ros::TimerEvent& ev)
  {
    const ros::Time now = ev.current_real.isZero() ? ros::Time::now() : ev.current_real;

    geometry_msgs::PoseStamped target_pose_msg;
    ros::Time target_stamp;
    Wrench3 wrench_raw;
    ros::Time wrench_stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_target_pose_ || !has_wrench_)
      {
        publishZero();
        return;
      }
      target_pose_msg = target_pose_msg_;
      target_stamp = target_pose_stamp_;
      wrench_raw = wrench_raw_;
      wrench_stamp = wrench_stamp_;
    }

    if ((now - target_stamp).toSec() > target_pose_timeout_ ||
        (now - wrench_stamp).toSec() > wrench_timeout_)
    {
      publishZero();
      resetCommandState();
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
      }
    }
    const double dt_step = dt_used / static_cast<double>(n_substeps);
    if (!(dt_step > 0.0) || !std::isfinite(dt_step))
    {
      publishZero();
      return;
    }

    geometry_msgs::PoseStamped target_pose_base;
    if (!transformPoseToBase(target_pose_msg, target_pose_base))
    {
      publishZero();
      return;
    }

    Eigen::Isometry3d t_base_tcp;
    if (!getTcpPose(t_base_tcp))
    {
      publishZero();
      return;
    }

    Eigen::Vector3d current_pos = t_base_tcp.translation();
    Eigen::Quaterniond current_ori(t_base_tcp.rotation());
    current_ori.normalize();

    Eigen::Vector3d target_pos(target_pose_base.pose.position.x,
                               target_pose_base.pose.position.y,
                               target_pose_base.pose.position.z);
    Eigen::Quaterniond target_ori(target_pose_base.pose.orientation.w,
                                  target_pose_base.pose.orientation.x,
                                  target_pose_base.pose.orientation.y,
                                  target_pose_base.pose.orientation.z);
    if (target_ori.norm() < teleoperation_simplified::kMathEps)
    {
      publishZero();
      return;
    }
    target_ori.normalize();

    const double dt_for_filter = std::clamp(dt_raw, std::max(1e-6, dt_min), (dt_max > 0.0 ? dt_max : dt_raw));
    const double alpha = teleoperation_simplified::lowpassAlphaFromCutoffHz(dt_for_filter, wrench_filter_cutoff_hz_);
    const bool use_filter = (alpha > 0.0) && (alpha < 1.0);

    if (!has_filtered_wrench_)
    {
      wrench_filt_ = teleoperation_simplified::filterClampDeadbandWrenchNorm(
          wrench_raw, wrench_raw, false, alpha,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          max_force_, max_torque_, true, cross_deadband_scale_, wrench_db_state_);
      has_filtered_wrench_ = true;
    }
    else
    {
      wrench_filt_ = teleoperation_simplified::filterClampDeadbandWrenchNorm(
          wrench_filt_, wrench_raw, use_filter, alpha,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          max_force_, max_torque_, true, cross_deadband_scale_, wrench_db_state_);
    }

    if (!enable_translation_)
    {
      wrench_filt_.f.setZero();
      wrench_db_state_.f_active = false;
    }
    if (!enable_rotation_)
    {
      wrench_filt_.tau.setZero();
      wrench_db_state_.tau_active = false;
    }

    debug_wrench_pub_.publish(wrench_filt_, now, base_frame_);

    const Eigen::Vector3d position_error = target_pos - current_pos;
    const Eigen::Vector3d orientation_error =
        teleoperation_simplified::orientationErrorAxisAngle(current_ori, target_ori);

    const double mass_linear = std::max(1e-6, mass_linear_);
    const double damping_linear = std::max(0.0, damping_linear_);
    const double mass_angular = std::max(1e-6, mass_angular_);
    const double damping_angular = std::max(0.0, damping_angular_);

    const Eigen::Vector3d max_a_lin = accelLimitVector(max_linear_accel_);
    const Eigen::Vector3d max_j_lin = accelLimitVector(max_linear_jerk_);
    const Eigen::Vector3d max_a_ang = accelLimitVector(max_angular_accel_);
    const Eigen::Vector3d max_j_ang = accelLimitVector(max_angular_jerk_);

    for (int step = 0; step < n_substeps; ++step)
    {
      if (enable_translation_)
      {
        const Eigen::Vector3d force_virtual = stiffness_linear_ * position_error;
        const Eigen::Vector3d rhs = force_virtual + wrench_filt_.f - damping_linear * v_lin_cmd_;
        const Eigen::Vector3d a_cmd = a_lin_limiter_.step(rhs / mass_linear, dt_step, max_a_lin, max_j_lin);
        v_lin_cmd_ = optionalClampNorm(v_lin_cmd_ + a_cmd * dt_step, max_linear_speed_);
      }
      else
      {
        v_lin_cmd_.setZero();
        a_lin_limiter_.reset();
      }

      if (enable_rotation_)
      {
        const Eigen::Vector3d torque_virtual = stiffness_angular_ * orientation_error;
        const Eigen::Vector3d rhs = torque_virtual + wrench_filt_.tau - damping_angular * v_ang_cmd_;
        const Eigen::Vector3d a_cmd = a_ang_limiter_.step(rhs / mass_angular, dt_step, max_a_ang, max_j_ang);
        v_ang_cmd_ = optionalClampNorm(v_ang_cmd_ + a_cmd * dt_step, max_angular_speed_);
      }
      else
      {
        v_ang_cmd_.setZero();
        a_ang_limiter_.reset();
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
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_target_pose_;
  ros::Subscriber sub_wrench_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_feedback_pose_;
  ros::Timer timer_;
  ros::Timer feedback_timer_;

  teleoperation_simplified::WrenchDebugPublisher debug_wrench_pub_;

  std::string target_pose_topic_;
  std::string wrench_topic_;
  std::string command_topic_;
  std::string feedback_pose_topic_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string feedback_frame_id_override_;
  std::string wrench_source_frame_override_;

  double control_rate_{250.0};
  double tf_timeout_s_{0.02};
  double target_pose_timeout_{0.1};
  double wrench_timeout_{0.1};
  bool use_latest_tf_for_wrench_{true};

  bool enable_translation_{true};
  bool enable_rotation_{false};
  double mass_linear_{5.0};
  double mass_angular_{0.03};
  double stiffness_linear_{70.0};
  double damping_linear_{100.0};
  double stiffness_angular_{3.0};
  double damping_angular_{5.0};

  double wrench_filter_cutoff_hz_{12.0};
  double force_deadband_enter_{1.0};
  double force_deadband_exit_{0.7};
  double torque_deadband_enter_{0.2};
  double torque_deadband_exit_{0.14};
  double cross_deadband_scale_{1.0};
  double max_force_{200.0};
  double max_torque_{40.0};

  double max_linear_speed_{0.7};
  double max_angular_speed_{1.8};
  double max_linear_accel_{1.5};
  double max_angular_accel_{4.0};
  double max_linear_jerk_{25.0};
  double max_angular_jerk_{35.0};
  double dt_min_factor_{0.5};
  double dt_max_factor_{2.0};
  bool dt_use_substepping_{true};
  int dt_max_substeps_{10};

  mutable std::mutex mutex_;
  bool has_target_pose_{false};
  geometry_msgs::PoseStamped target_pose_msg_;
  ros::Time target_pose_stamp_{0};

  bool has_wrench_{false};
  Wrench3 wrench_raw_;
  ros::Time wrench_stamp_{0};

  bool has_filtered_wrench_{false};
  Wrench3 wrench_filt_;
  teleoperation_simplified::WrenchDeadbandState wrench_db_state_;

  ros::Time last_time_{0};
  Eigen::Vector3d v_lin_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_cmd_{Eigen::Vector3d::Zero()};
  teleoperation_simplified::JerkLimiter3 a_lin_limiter_;
  teleoperation_simplified::JerkLimiter3 a_ang_limiter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_slave_pose_admittance");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopSlavePoseAdmittance node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_slave_pose_admittance failed: %s", ex.what());
    return 1;
  }

  return 0;
}
