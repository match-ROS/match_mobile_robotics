#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include "teleoperation/core/math_utils.hpp"
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

    pnh_.param<bool>("use_torques", use_torques_, use_torques_);

    pnh_.param<double>("wrench_filter_alpha", wrench_filter_alpha_, wrench_filter_alpha_);
    pnh_.param<double>("force_deadband", force_deadband_, force_deadband_);
    pnh_.param<double>("torque_deadband", torque_deadband_, torque_deadband_);
    pnh_.param<double>("max_force", max_force_, max_force_);
    pnh_.param<double>("max_torque", max_torque_, max_torque_);

    pnh_.param<double>("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param<double>("max_angular_speed", max_angular_speed_, max_angular_speed_);

    pnh_.param<double>("wrench_timeout_s", wrench_timeout_s_, wrench_timeout_s_);
    pnh_.param<bool>("reset_on_stale", reset_on_stale_, reset_on_stale_);

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
    debug_master_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                "filtered_master_wrench_topic", "debug/master_wrench_filtered");
    debug_slave_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                               "filtered_slave_wrench_topic", "debug/slave_wrench_filtered");
    debug_coupling_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                  "filtered_coupling_wrench_topic", "debug/coupling_wrench_filtered");

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterHapticController::tick, this);
  }

private:
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
    if (!wrenchMsgToWrench3(*msg, w)) return;

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
  }

  void tick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

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
        v_lin_cmd_.setZero();
        v_ang_cmd_.setZero();
        has_filtered_master_ = false;
        has_filtered_slave_ = false;
        has_filtered_coupling_ = false;
      }
      last_time_ = now;
      return;
    }

    double dt = 0.0;
    if (!last_time_.isZero())
    {
      dt = (now - last_time_).toSec();
    }
    last_time_ = now;
    if (!(dt > 1e-5) || !std::isfinite(dt))
    {
      publishZero();
      return;
    }

    // Filter + clamp + deadband.
    if (!has_filtered_master_)
    {
      master_filt_ = master_raw;
      has_filtered_master_ = true;
    }
    else
    {
      master_filt_ = teleoperation::filterClampDeadbandWrench(master_filt_, master_raw, wrench_filter_alpha_ > 0.0,
                                                               wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                               max_force_, max_torque_, use_torques_);
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
        slave_filt_ = teleoperation::filterClampDeadbandWrench(slave_filt_, slave_raw, wrench_filter_alpha_ > 0.0,
                                                                wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                                max_force_, max_torque_, use_torques_);
      }
    }
    else
    {
      slave_filt_.f.setZero();
      slave_filt_.tau.setZero();
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
        coupling_filt_ = teleoperation::filterClampDeadbandWrench(coupling_filt_, coupling_raw, wrench_filter_alpha_ > 0.0,
                                                                   wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                                   max_force_, max_torque_, use_torques_);
      }
    }
    else
    {
      coupling_filt_.f.setZero();
      coupling_filt_.tau.setZero();
    }

    debug_master_filt_pub_.publish(master_filt_, now, wrench_target_frame_);
    debug_slave_filt_pub_.publish(slave_filt_, now, wrench_target_frame_);
    debug_coupling_filt_pub_.publish(coupling_filt_, now, wrench_target_frame_);

    // Admittance dynamics (linear + optional angular).
    const Eigen::Vector3d F_hand = master_filt_.f;
    const Eigen::Vector3d Tau_hand = master_filt_.tau;

    const Eigen::Vector3d F_feedback = (kf_force_ * slave_filt_.f) + coupling_filt_.f;
    const Eigen::Vector3d Tau_feedback = (kf_torque_ * slave_filt_.tau) + coupling_filt_.tau;

    const double m_lin = std::max(1e-6, mass_linear_);
    const double d_lin = std::max(0.0, damping_linear_);
    const double m_ang = std::max(1e-6, mass_angular_);
    const double d_ang = std::max(0.0, damping_angular_);

    const Eigen::Vector3d a_lin = (F_hand - F_feedback - d_lin * v_lin_cmd_) / m_lin;
    v_lin_cmd_ = v_lin_cmd_ + a_lin * dt;
    v_lin_cmd_ = teleoperation::clampNorm3(v_lin_cmd_, max_linear_speed_);

    if (use_torques_)
    {
      const Eigen::Vector3d a_ang = (Tau_hand - Tau_feedback - d_ang * v_ang_cmd_) / m_ang;
      v_ang_cmd_ = v_ang_cmd_ + a_ang * dt;
      v_ang_cmd_ = teleoperation::clampNorm3(v_ang_cmd_, max_angular_speed_);
    }
    else
    {
      v_ang_cmd_.setZero();
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

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_master_wrench_;
  ros::Subscriber sub_slave_wrench_;
  ros::Subscriber sub_coupling_wrench_;
  ros::Publisher pub_cmd_;
  teleoperation::WrenchDebugPublisher debug_master_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_slave_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_coupling_filt_pub_;
  ros::Timer timer_;

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

  double kf_force_{0.3};
  double kf_torque_{0.0};
  bool use_torques_{false};

  double wrench_filter_alpha_{0.07};
  double force_deadband_{1.0};
  double torque_deadband_{0.2};
  double max_force_{150.0};
  double max_torque_{20.0};

  double max_linear_speed_{0.25};
  double max_angular_speed_{0.4};

  double wrench_timeout_s_{0.2};
  bool reset_on_stale_{true};

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
  Eigen::Vector3d v_lin_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_cmd_{Eigen::Vector3d::Zero()};
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

