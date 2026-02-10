#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

namespace
{
constexpr double kEps = 1e-12;

Eigen::Vector3d clampNorm3(const Eigen::Vector3d& v, double max_norm)
{
  if (max_norm <= kEps) return Eigen::Vector3d::Zero();
  const double n = v.norm();
  if (n > max_norm && n > kEps) return v * (max_norm / n);
  return v;
}

Eigen::Vector3d deadbandAbs3(const Eigen::Vector3d& v, double db)
{
  if (db <= 0.0) return v;
  Eigen::Vector3d out = v;
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(out[i]) < db) out[i] = 0.0;
  }
  return out;
}

Eigen::Vector3d ema3(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return a * curr + (1.0 - a) * prev;
}

struct Wrench3
{
  Eigen::Vector3d f{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tau{Eigen::Vector3d::Zero()};
};

}  // namespace

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

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterHapticController::tick, this);
  }

private:
  static Eigen::Vector3d vecFromMsg(const geometry_msgs::Vector3& v)
  {
    return Eigen::Vector3d(v.x, v.y, v.z);
  }

  bool rotateToTargetFrame(const std::string& source_frame,
                           const ros::Time& stamp,
                           const Eigen::Vector3d& v_in,
                           Eigen::Vector3d& v_out) const
  {
    if (source_frame.empty() || source_frame == wrench_target_frame_)
    {
      v_out = v_in;
      return true;
    }

    try
    {
      const geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(wrench_target_frame_, source_frame,
                                     stamp.isZero() ? ros::Time(0) : stamp,
                                     ros::Duration(tf_timeout_s_));
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      tf2::Matrix3x3 R(q);
      const tf2::Vector3 vin(v_in.x(), v_in.y(), v_in.z());
      const tf2::Vector3 vout = R * vin;
      v_out = Eigen::Vector3d(vout.x(), vout.y(), vout.z());
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "TF rotate failed (%s -> %s): %s",
                              source_frame.c_str(), wrench_target_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool wrenchMsgToWrench3(const geometry_msgs::WrenchStamped& msg, Wrench3& out) const
  {
    const Eigen::Vector3d f_src = vecFromMsg(msg.wrench.force);
    const Eigen::Vector3d t_src = vecFromMsg(msg.wrench.torque);

    Eigen::Vector3d f_tgt, t_tgt;
    const std::string src_frame = msg.header.frame_id;
    const ros::Time stamp = msg.header.stamp;

    if (!rotateToTargetFrame(src_frame, stamp, f_src, f_tgt)) return false;
    if (!rotateToTargetFrame(src_frame, stamp, t_src, t_tgt)) return false;

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

  static Wrench3 filterClampDeadbandWrench(const Wrench3& prev,
                                           const Wrench3& curr,
                                           bool use_filter,
                                           double alpha,
                                           double force_db,
                                           double torque_db,
                                           double max_f,
                                           double max_tau,
                                           bool use_torques)
  {
    Wrench3 out;

    const Eigen::Vector3d f0 = use_filter ? ema3(prev.f, curr.f, alpha) : curr.f;
    const Eigen::Vector3d t0 = use_filter ? ema3(prev.tau, curr.tau, alpha) : curr.tau;

    out.f = deadbandAbs3(f0, force_db);
    out.f = clampNorm3(out.f, max_f);

    if (use_torques)
    {
      out.tau = deadbandAbs3(t0, torque_db);
      out.tau = clampNorm3(out.tau, max_tau);
    }
    else
    {
      out.tau.setZero();
    }

    return out;
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
      master_filt_ = filterClampDeadbandWrench(master_filt_, master_raw, wrench_filter_alpha_ > 0.0,
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
        slave_filt_ = filterClampDeadbandWrench(slave_filt_, slave_raw, wrench_filter_alpha_ > 0.0,
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
        coupling_filt_ = filterClampDeadbandWrench(coupling_filt_, coupling_raw, wrench_filter_alpha_ > 0.0,
                                                   wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                   max_force_, max_torque_, use_torques_);
      }
    }
    else
    {
      coupling_filt_.f.setZero();
      coupling_filt_.tau.setZero();
    }

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
    v_lin_cmd_ = clampNorm3(v_lin_cmd_, max_linear_speed_);

    if (use_torques_)
    {
      const Eigen::Vector3d a_ang = (Tau_hand - Tau_feedback - d_ang * v_ang_cmd_) / m_ang;
      v_ang_cmd_ = v_ang_cmd_ + a_ang * dt;
      v_ang_cmd_ = clampNorm3(v_ang_cmd_, max_angular_speed_);
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
  ros::Timer timer_;

  // Params
  std::string master_wrench_topic_;
  std::string slave_wrench_topic_;
  std::string coupling_wrench_topic_;
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

