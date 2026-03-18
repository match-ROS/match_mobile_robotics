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

class TeleopMasterAdmittance
{
public:
  TeleopMasterAdmittance(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("wrench_topic", wrench_topic_, "wrench");
    pnh_.param<std::string>("command_topic", command_topic_, "twist_controller/command");
    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "target_pose");
    pnh_.param<std::string>("slave_actual_pose_topic", slave_actual_pose_topic_, "slave_actual_pose");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("tcp_frame", tcp_frame_, "tool0");
    pnh_.param<std::string>("target_frame_id_override", target_frame_id_override_, "");
    pnh_.param<std::string>("wrench_source_frame_override", wrench_source_frame_override_, "");
    pnh_.param<bool>("use_latest_tf_for_wrench", use_latest_tf_for_wrench_, true);

    pnh_.param("control_rate", control_rate_, control_rate_);
    pnh_.param("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);
    pnh_.param("wrench_timeout_s", wrench_timeout_s_, wrench_timeout_s_);

    pnh_.param("enable_translation", enable_translation_, enable_translation_);
    pnh_.param("enable_rotation", enable_rotation_, enable_rotation_);

    pnh_.param("mass_linear", mass_linear_, mass_linear_);
    pnh_.param("damping_linear", damping_linear_, damping_linear_);
    pnh_.param("mass_angular", mass_angular_, mass_angular_);
    pnh_.param("damping_angular", damping_angular_, damping_angular_);
    pnh_.param("spring_stiffness_linear", spring_stiffness_linear_, spring_stiffness_linear_);
    pnh_.param("spring_stiffness_angular", spring_stiffness_angular_, spring_stiffness_angular_);

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
    pnh_.param("speed_saturation_eps", speed_saturation_eps_, speed_saturation_eps_);

    sub_wrench_ = nh_.subscribe(wrench_topic_, 1, &TeleopMasterAdmittance::wrenchCb, this,
                                ros::TransportHints().tcpNoDelay());
    sub_slave_pose_ = nh_.subscribe(slave_actual_pose_topic_, 1, &TeleopMasterAdmittance::slavePoseCb, this,
                                    ros::TransportHints().tcpNoDelay());

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    pub_target_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 1);
    debug_wrench_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                           "filtered_wrench_topic", "debug/master_wrench_filtered");

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterAdmittance::tick, this);
    target_timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterAdmittance::targetPoseTick, this);
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
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_admittance", "TF TCP pose failed: %s", ex.what());
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
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_admittance", "TF pose transform failed: %s", ex.what());
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
                                                       f_src, f_base, "teleop_master_admittance"))
    {
      return false;
    }
    if (!teleoperation_simplified::rotateVectorToFrame(tf_buffer_, base_frame_, src_frame, stamp, tf_timeout_s_,
                                                       t_src, t_base, "teleop_master_admittance"))
    {
      return false;
    }

    out.f = f_base;
    out.tau = t_base;
    return true;
  }

  void wrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 wrench;
    if (!wrenchMsgToWrench3(*msg, wrench))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    master_wrench_raw_ = wrench;
    master_wrench_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_master_wrench_ = true;
  }

  void slavePoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    geometry_msgs::PoseStamped pose_base;
    if (!transformPoseToBase(*msg, pose_base))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    slave_pose_pos_ = Eigen::Vector3d(pose_base.pose.position.x,
                                      pose_base.pose.position.y,
                                      pose_base.pose.position.z);
    Eigen::Quaterniond q(pose_base.pose.orientation.w,
                         pose_base.pose.orientation.x,
                         pose_base.pose.orientation.y,
                         pose_base.pose.orientation.z);
    if (q.norm() > teleoperation_simplified::kMathEps)
    {
      slave_pose_ori_ = q.normalized();
      has_slave_pose_ = true;
    }
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

  void targetPoseTick(const ros::TimerEvent&)
  {
    Eigen::Isometry3d t_base_tcp;
    if (!getTcpPose(t_base_tcp))
    {
      return;
    }

    geometry_msgs::PoseStamped target_msg;
    target_msg.header.stamp = ros::Time::now();
    target_msg.header.frame_id = target_frame_id_override_.empty() ? base_frame_ : target_frame_id_override_;
    target_msg.pose.position.x = t_base_tcp.translation().x();
    target_msg.pose.position.y = t_base_tcp.translation().y();
    target_msg.pose.position.z = t_base_tcp.translation().z();

    Eigen::Quaterniond q(t_base_tcp.rotation());
    q.normalize();
    target_msg.pose.orientation.w = q.w();
    target_msg.pose.orientation.x = q.x();
    target_msg.pose.orientation.y = q.y();
    target_msg.pose.orientation.z = q.z();
    pub_target_pose_.publish(target_msg);
  }

  void tick(const ros::TimerEvent& ev)
  {
    const ros::Time now = ev.current_real.isZero() ? ros::Time::now() : ev.current_real;

    Wrench3 master_raw;
    ros::Time master_stamp;
    Eigen::Vector3d slave_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond slave_ori = Eigen::Quaterniond::Identity();
    bool has_slave_pose = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_master_wrench_)
      {
        publishZero();
        return;
      }
      master_raw = master_wrench_raw_;
      master_stamp = master_wrench_stamp_;
      slave_pos = slave_pose_pos_;
      slave_ori = slave_pose_ori_;
      has_slave_pose = has_slave_pose_;
    }

    if ((now - master_stamp).toSec() > wrench_timeout_s_)
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

    const double dt_for_filter = std::clamp(dt_raw, std::max(1e-6, dt_min), (dt_max > 0.0 ? dt_max : dt_raw));
    const double alpha = teleoperation_simplified::lowpassAlphaFromCutoffHz(dt_for_filter, wrench_filter_cutoff_hz_);
    const bool use_filter = (alpha > 0.0) && (alpha < 1.0);

    if (!has_filtered_wrench_)
    {
      master_wrench_filt_ = teleoperation_simplified::filterClampDeadbandWrenchNorm(
          master_raw, master_raw, false, alpha,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          max_force_, max_torque_, true, cross_deadband_scale_, wrench_db_state_);
      has_filtered_wrench_ = true;
    }
    else
    {
      master_wrench_filt_ = teleoperation_simplified::filterClampDeadbandWrenchNorm(
          master_wrench_filt_, master_raw, use_filter, alpha,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          max_force_, max_torque_, true, cross_deadband_scale_, wrench_db_state_);
    }

    if (!enable_translation_)
    {
      master_wrench_filt_.f.setZero();
      wrench_db_state_.f_active = false;
    }
    if (!enable_rotation_)
    {
      master_wrench_filt_.tau.setZero();
      wrench_db_state_.tau_active = false;
    }

    debug_wrench_pub_.publish(master_wrench_filt_, now, base_frame_);

    Eigen::Isometry3d t_base_tcp;
    if (!getTcpPose(t_base_tcp))
    {
      publishZero();
      return;
    }

    const Eigen::Vector3d master_pos = t_base_tcp.translation();
    Eigen::Quaterniond master_ori(t_base_tcp.rotation());
    master_ori.normalize();

    if (!has_slave_pose)
    {
      slave_pos = master_pos;
      slave_ori = master_ori;
    }

    Eigen::Vector3d spring_force = Eigen::Vector3d::Zero();
    if (enable_translation_)
    {
      spring_force = spring_stiffness_linear_ * (slave_pos - master_pos);
    }

    Eigen::Vector3d spring_torque = Eigen::Vector3d::Zero();
    if (enable_rotation_)
    {
      spring_torque =
          spring_stiffness_angular_ * teleoperation_simplified::orientationErrorAxisAngle(master_ori, slave_ori);
    }

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
        const Eigen::Vector3d rhs = master_wrench_filt_.f + spring_force - damping_linear * v_lin_cmd_;
        Eigen::Vector3d a_cmd = a_lin_limiter_.step(rhs / mass_linear, dt_step, max_a_lin, max_j_lin);

        if (max_linear_speed_ > 0.0 &&
            v_lin_cmd_.norm() >= (max_linear_speed_ - std::max(0.0, speed_saturation_eps_)))
        {
          const double speed = v_lin_cmd_.norm();
          if (speed > teleoperation_simplified::kMathEps)
          {
            const Eigen::Vector3d dir = v_lin_cmd_ / speed;
            const double a_radial = dir.dot(a_cmd);
            if (a_radial > 0.0)
            {
              a_cmd -= a_radial * dir;
            }
          }
        }

        v_lin_cmd_ = optionalClampNorm(v_lin_cmd_ + a_cmd * dt_step, max_linear_speed_);
      }
      else
      {
        v_lin_cmd_.setZero();
        a_lin_limiter_.reset();
      }

      if (enable_rotation_)
      {
        const Eigen::Vector3d rhs = master_wrench_filt_.tau + spring_torque - damping_angular * v_ang_cmd_;
        Eigen::Vector3d a_cmd = a_ang_limiter_.step(rhs / mass_angular, dt_step, max_a_ang, max_j_ang);

        if (max_angular_speed_ > 0.0 &&
            v_ang_cmd_.norm() >= (max_angular_speed_ - std::max(0.0, speed_saturation_eps_)))
        {
          const double speed = v_ang_cmd_.norm();
          if (speed > teleoperation_simplified::kMathEps)
          {
            const Eigen::Vector3d dir = v_ang_cmd_ / speed;
            const double a_radial = dir.dot(a_cmd);
            if (a_radial > 0.0)
            {
              a_cmd -= a_radial * dir;
            }
          }
        }

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

  ros::Subscriber sub_wrench_;
  ros::Subscriber sub_slave_pose_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_target_pose_;
  ros::Timer timer_;
  ros::Timer target_timer_;

  teleoperation_simplified::WrenchDebugPublisher debug_wrench_pub_;

  std::string wrench_topic_;
  std::string command_topic_;
  std::string target_pose_topic_;
  std::string slave_actual_pose_topic_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string target_frame_id_override_;
  std::string wrench_source_frame_override_;

  double control_rate_{250.0};
  double tf_timeout_s_{0.02};
  double wrench_timeout_s_{0.2};
  bool use_latest_tf_for_wrench_{true};

  bool enable_translation_{true};
  bool enable_rotation_{false};

  double mass_linear_{5.0};
  double damping_linear_{10.0};
  double mass_angular_{0.03};
  double damping_angular_{0.12};
  double spring_stiffness_linear_{10.0};
  double spring_stiffness_angular_{0.001};

  double wrench_filter_cutoff_hz_{12.0};
  double force_deadband_enter_{1.5};
  double force_deadband_exit_{1.0};
  double torque_deadband_enter_{0.11};
  double torque_deadband_exit_{0.08};
  double cross_deadband_scale_{0.35};
  double max_force_{150.0};
  double max_torque_{30.0};

  double max_linear_speed_{0.7};
  double max_angular_speed_{3.0};
  double max_linear_accel_{1.8};
  double max_angular_accel_{5.0};
  double max_linear_jerk_{40.0};
  double max_angular_jerk_{50.0};
  double dt_min_factor_{0.5};
  double dt_max_factor_{2.0};
  bool dt_use_substepping_{true};
  int dt_max_substeps_{10};
  double speed_saturation_eps_{1e-3};

  mutable std::mutex mutex_;
  bool has_master_wrench_{false};
  Wrench3 master_wrench_raw_;
  ros::Time master_wrench_stamp_{0};

  bool has_slave_pose_{false};
  Eigen::Vector3d slave_pose_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond slave_pose_ori_{Eigen::Quaterniond::Identity()};

  bool has_filtered_wrench_{false};
  Wrench3 master_wrench_filt_;
  teleoperation_simplified::WrenchDeadbandState wrench_db_state_;

  ros::Time last_time_{0};
  Eigen::Vector3d v_lin_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_cmd_{Eigen::Vector3d::Zero()};
  teleoperation_simplified::JerkLimiter3 a_lin_limiter_;
  teleoperation_simplified::JerkLimiter3 a_ang_limiter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_master_admittance");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopMasterAdmittance node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_master_admittance failed: %s", ex.what());
    return 1;
  }

  return 0;
}
