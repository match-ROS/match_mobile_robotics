#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include "teleoperation/components/pid_controller.hpp"
#include "teleoperation/core/math_utils.hpp"
#include "teleoperation/core/tf_utils.hpp"
#include "teleoperation/core/types.hpp"
#include "teleoperation/core/wrench_debug_publisher.hpp"
#include "teleoperation/core/wrench_utils.hpp"

namespace
{
using Wrench3 = teleoperation::Wrench3;
}  // namespace

class TeleopSlaveTwistOuterLoop
{
public:
  TeleopSlaveTwistOuterLoop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
    , pid_pos_(3)
    , pid_ori_(3)
  {
    // Frames
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("tcp_frame", tcp_frame_, "tool0");

    // Inputs
    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "target_pose");
    pnh_.param<std::string>("feedforward_twist_topic", feedforward_twist_topic_, "feedforward_twist");
    pnh_.param<std::string>("wrench_topic", wrench_topic_, "wrench");

    // Output
    pnh_.param<std::string>("command_topic", command_topic_, "twist_controller/command");

    // Rates / timeouts
    pnh_.param("control_rate", control_rate_, control_rate_);
    pnh_.param("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);
    pnh_.param("target_pose_timeout", target_pose_timeout_, target_pose_timeout_);
    pnh_.param("feedforward_timeout", feedforward_timeout_, feedforward_timeout_);
    pnh_.param("wrench_timeout", wrench_timeout_, wrench_timeout_);

    // Control gains
    pnh_.param("k_ff", k_ff_, k_ff_);
    pnh_.param("k_adm_linear", k_adm_linear_, k_adm_linear_);
    pnh_.param("k_adm_angular", k_adm_angular_, k_adm_angular_);
    pnh_.param("use_torques", use_torques_, use_torques_);

    // Wrench TF options (matching master pattern)
    pnh_.param<std::string>("wrench_source_frame_override", wrench_source_frame_override_, wrench_source_frame_override_);
    pnh_.param<bool>("use_latest_tf_for_wrench", use_latest_tf_for_wrench_, use_latest_tf_for_wrench_);

    // Scaling alpha(F)
    pnh_.param<std::string>("alpha_mode", alpha_mode_, alpha_mode_);
    pnh_.param("force_start", force_start_, force_start_);
    pnh_.param("force_stop", force_stop_, force_stop_);

    // Wrench filtering & clamps
    pnh_.param("wrench_filter_alpha", wrench_filter_alpha_, wrench_filter_alpha_);
    pnh_.param("force_deadband", force_deadband_, force_deadband_);
    pnh_.param("torque_deadband", torque_deadband_, torque_deadband_);
    pnh_.param("max_force", max_force_, max_force_);
    pnh_.param("max_torque", max_torque_, max_torque_);

    // Velocity limits
    pnh_.param("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);
    pnh_.param("max_compliance_linear_speed", max_compliance_linear_speed_, max_compliance_linear_speed_);
    pnh_.param("max_compliance_angular_speed", max_compliance_angular_speed_, max_compliance_angular_speed_);

    // Hard guard
    pnh_.param("hard_force_threshold", hard_force_threshold_, hard_force_threshold_);
    pnh_.param("hard_force_duration", hard_force_duration_, hard_force_duration_);
    pnh_.param<std::string>("hard_guard_action", hard_guard_action_, hard_guard_action_);
    pnh_.param("retreat_speed", retreat_speed_, retreat_speed_);

    // dt sanitization
    pnh_.param("dt_min_factor", dt_min_factor_, dt_min_factor_);
    pnh_.param("dt_max_factor", dt_max_factor_, dt_max_factor_);

    // PID params (reuse existing component)
    teleoperation::PIDConfig pcfg;
    pnh_.param("pid/position/kp", pcfg.kp, pcfg.kp);
    pnh_.param("pid/position/ki", pcfg.ki, pcfg.ki);
    pnh_.param("pid/position/kd", pcfg.kd, pcfg.kd);
    pnh_.param("pid/position/output_limit", pcfg.output_limit, pcfg.output_limit);
    pnh_.param("pid/position/derivative_filter_tau", pcfg.derivative_filter_tau, pcfg.derivative_filter_tau);
    pcfg.kff = 0.0;
    pcfg.enabled = true;
    pid_pos_.setConfig(pcfg);

    teleoperation::PIDConfig ocfg;
    pnh_.param("pid/orientation/kp", ocfg.kp, ocfg.kp);
    pnh_.param("pid/orientation/ki", ocfg.ki, ocfg.ki);
    pnh_.param("pid/orientation/kd", ocfg.kd, ocfg.kd);
    pnh_.param("pid/orientation/output_limit", ocfg.output_limit, ocfg.output_limit);
    pnh_.param("pid/orientation/derivative_filter_tau", ocfg.derivative_filter_tau, ocfg.derivative_filter_tau);
    ocfg.kff = 0.0;
    ocfg.enabled = true;
    pid_ori_.setConfig(ocfg);

    sub_target_pose_ = nh_.subscribe(target_pose_topic_, 1, &TeleopSlaveTwistOuterLoop::targetPoseCb, this,
                                     ros::TransportHints().tcpNoDelay());
    sub_ff_twist_ = nh_.subscribe(feedforward_twist_topic_, 1, &TeleopSlaveTwistOuterLoop::ffTwistCb, this,
                                  ros::TransportHints().tcpNoDelay());
    sub_wrench_ = nh_.subscribe(wrench_topic_, 1, &TeleopSlaveTwistOuterLoop::wrenchCb, this,
                                ros::TransportHints().tcpNoDelay());

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    debug_wrench_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                "filtered_wrench_topic", "debug/wrench_filtered");

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopSlaveTwistOuterLoop::tick, this);
  }

private:
  void targetPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pose_ = *msg;
    target_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_target_ = true;
  }

  void ffTwistCb(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ff_twist_ = *msg;
    ff_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_ff_ = true;
  }

  void wrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    const Eigen::Vector3d f_src = teleoperation::vector3MsgToEigen(msg->wrench.force);
    const Eigen::Vector3d t_src = teleoperation::vector3MsgToEigen(msg->wrench.torque);
    Eigen::Vector3d f_base, t_base;
    const std::string src_frame = wrench_source_frame_override_.empty()
                                      ? msg->header.frame_id
                                      : wrench_source_frame_override_;
    const ros::Time stamp = use_latest_tf_for_wrench_
                                ? ros::Time(0)
                                : (msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
    if (!teleoperation::rotateVectorToFrame(tf_buffer_, base_frame_, src_frame, stamp, tf_timeout_s_,
                                            f_src, f_base, "teleop_slave_twist_outer_loop"))
    {
      return;
    }
    if (!teleoperation::rotateVectorToFrame(tf_buffer_, base_frame_, src_frame, stamp, tf_timeout_s_,
                                            t_src, t_base, "teleop_slave_twist_outer_loop"))
    {
      return;
    }
    w.f = f_base;
    w.tau = t_base;

    std::lock_guard<std::mutex> lock(mutex_);
    wrench_raw_ = w;
    wrench_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_wrench_ = true;
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
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(base_frame_, in.header.frame_id, stamp, ros::Duration(tf_timeout_s_));
      tf2::doTransform(in, out, T);
      out.header.frame_id = base_frame_;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_twist_outer_loop", "TF pose transform failed: %s", ex.what());
      return false;
    }
  }

  bool rotateTwistToBase(const geometry_msgs::TwistStamped& in, geometry_msgs::TwistStamped& out) const
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
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(base_frame_, in.header.frame_id, stamp, ros::Duration(tf_timeout_s_));
      tf2::Quaternion q;
      tf2::fromMsg(T.transform.rotation, q);
      tf2::Matrix3x3 R(q);

      geometry_msgs::TwistStamped rotated = in;
      rotated.header.frame_id = base_frame_;

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
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_twist_outer_loop", "TF twist rotate failed: %s", ex.what());
      return false;
    }
  }

  bool getTcpPose(Eigen::Isometry3d& T_base_tcp) const
  {
    geometry_msgs::TransformStamped T;
    try
    {
      T = tf_buffer_.lookupTransform(base_frame_, tcp_frame_, ros::Time(0), ros::Duration(tf_timeout_s_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_slave_twist_outer_loop", "TF TCP pose failed: %s", ex.what());
      return false;
    }

    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
    Eigen::Quaterniond q(T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z);
    if (q.norm() < teleoperation::kMathEps) return false;
    q.normalize();
    out.linear() = q.toRotationMatrix();
    T_base_tcp = out;
    return true;
  }

  static double smoothstep01(double t)
  {
    const double x = std::clamp(t, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  double computeAlpha(const Eigen::Vector3d& F_ext_lin, const Eigen::Vector3d& v_ff_lin) const
  {
    double metric = F_ext_lin.norm();
    if (alpha_mode_ == "parallel")
    {
      const double vnorm = v_ff_lin.norm();
      if (vnorm > 1e-6)
      {
        const Eigen::Vector3d d = v_ff_lin / vnorm;
        metric = std::abs(d.dot(F_ext_lin));
      }
    }

    if (metric <= force_start_) return 1.0;
    if (metric >= force_stop_) return 0.0;
    const double t = (metric - force_start_) / std::max(1e-9, (force_stop_ - force_start_));
    return 1.0 - smoothstep01(t);
  }

  void publishZero(const char* reason)
  {
    ROS_DEBUG_THROTTLE_NAMED(1.0, "teleop_slave_twist_outer_loop", "Publishing zero twist: %s", reason);
    geometry_msgs::Twist cmd;
    pub_cmd_.publish(cmd);
  }

  void tick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

    geometry_msgs::PoseStamped target_pose_msg;
    geometry_msgs::TwistStamped ff_twist_msg;
    Wrench3 wrench_raw;
    ros::Time t_pose, t_ff, t_wrench;
    bool has_all = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_all = has_target_ && has_ff_ && has_wrench_;
      target_pose_msg = target_pose_;
      ff_twist_msg = ff_twist_;
      wrench_raw = wrench_raw_;
      t_pose = target_stamp_;
      t_ff = ff_stamp_;
      t_wrench = wrench_stamp_;
    }

    if (!has_all)
    {
      publishZero("missing inputs");
      return;
    }

    if ((now - t_pose).toSec() > target_pose_timeout_ ||
        (now - t_ff).toSec() > feedforward_timeout_ ||
        (now - t_wrench).toSec() > wrench_timeout_)
    {
      publishZero("stale inputs");
      // Reset PID integrators to avoid jumps after input returns.
      pid_pos_.reset();
      pid_ori_.reset();
      hard_guard_active_since_ = ros::Time(0);
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
      publishZero("bad dt");
      return;
    }

    // dt clamping (robustness against timer jitter)
    const double dt_nominal = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    const double dt_min = std::max(0.0, dt_min_factor_) * dt_nominal;
    const double dt_max = std::max(0.0, dt_max_factor_) * dt_nominal;
    if (dt_min > 0.0 && dt < dt_min)
    {
      dt = dt_nominal;
    }
    if (dt_max > 0.0 && dt > dt_max)
    {
      dt = dt_max;
    }

    // Transform inputs to base frame (within this robot TF tree).
    geometry_msgs::PoseStamped target_pose_base;
    if (!transformPoseToBase(target_pose_msg, target_pose_base))
    {
      publishZero("tf target pose");
      return;
    }
    geometry_msgs::TwistStamped ff_twist_base;
    if (!rotateTwistToBase(ff_twist_msg, ff_twist_base))
    {
      publishZero("tf ff twist");
      return;
    }

    // Current TCP pose (measured)
    Eigen::Isometry3d T_base_tcp;
    if (!getTcpPose(T_base_tcp))
    {
      publishZero("tf tcp pose");
      return;
    }

    Eigen::Vector3d p_curr = T_base_tcp.translation();
    Eigen::Quaterniond q_curr(T_base_tcp.rotation());

    Eigen::Vector3d p_tgt(target_pose_base.pose.position.x, target_pose_base.pose.position.y, target_pose_base.pose.position.z);
    Eigen::Quaterniond q_tgt(target_pose_base.pose.orientation.w,
                             target_pose_base.pose.orientation.x,
                             target_pose_base.pose.orientation.y,
                             target_pose_base.pose.orientation.z);
    if (q_tgt.norm() < teleoperation::kMathEps)
    {
      publishZero("bad target quaternion");
      return;
    }
    q_tgt.normalize();

    const Eigen::Vector3d e_p = p_tgt - p_curr;
    const Eigen::Vector3d e_o = teleoperation::orientationErrorAxisAngle(q_curr, q_tgt);

    const Eigen::VectorXd corr_p = pid_pos_.compute(e_p, dt);
    const Eigen::VectorXd corr_o = pid_ori_.compute(e_o, dt);

    // Wrench filtering + deadband + clamp
    if (!has_wrench_filt_)
    {
      wrench_filt_ = teleoperation::filterClampDeadbandWrench(wrench_raw, wrench_raw, false,
                                                               wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                               max_force_, max_torque_, use_torques_);
      has_wrench_filt_ = true;
    }
    else
    {
      wrench_filt_ = teleoperation::filterClampDeadbandWrench(wrench_filt_, wrench_raw, wrench_filter_alpha_ > 0.0,
                                                               wrench_filter_alpha_, force_deadband_, torque_deadband_,
                                                               max_force_, max_torque_, use_torques_);
    }
    debug_wrench_filt_pub_.publish(wrench_filt_, now, base_frame_);

    // Hard guard logic (force)
    const double f_norm = wrench_filt_.f.norm();
    if (f_norm >= hard_force_threshold_)
    {
      if (hard_guard_active_since_.isZero())
      {
        hard_guard_active_since_ = now;
      }
    }
    else
    {
      hard_guard_active_since_ = ros::Time(0);
    }

    const bool guard_active = !hard_guard_active_since_.isZero() &&
                              ((now - hard_guard_active_since_).toSec() >= hard_force_duration_);

    Eigen::Vector3d v_ff_lin(ff_twist_base.twist.linear.x, ff_twist_base.twist.linear.y, ff_twist_base.twist.linear.z);
    Eigen::Vector3d v_ff_ang(ff_twist_base.twist.angular.x, ff_twist_base.twist.angular.y, ff_twist_base.twist.angular.z);

    const double alpha = computeAlpha(wrench_filt_.f, v_ff_lin);

    Eigen::Vector3d v_cmd_lin = alpha * k_ff_ * v_ff_lin + corr_p;
    Eigen::Vector3d v_cmd_ang = alpha * k_ff_ * v_ff_ang + corr_o;

    // Compliance term (velocity from force)
    Eigen::Vector3d v_comp_lin = -k_adm_linear_ * wrench_filt_.f;
    v_comp_lin = teleoperation::clampNorm3(v_comp_lin, max_compliance_linear_speed_);
    v_cmd_lin += v_comp_lin;

    if (use_torques_)
    {
      Eigen::Vector3d v_comp_ang = -k_adm_angular_ * wrench_filt_.tau;
      v_comp_ang = teleoperation::clampNorm3(v_comp_ang, max_compliance_angular_speed_);
      v_cmd_ang += v_comp_ang;
    }

    // Hard guard override (stop or retreat)
    if (guard_active)
    {
      // Reset PID integrators to avoid accumulation during guard
      pid_pos_.reset();
      pid_ori_.reset();

      if (hard_guard_action_ == "stop")
      {
        v_cmd_lin.setZero();
        v_cmd_ang.setZero();
      }
      else if (hard_guard_action_ == "retreat")
      {
        // Retreat opposite to ff direction if available, else opposite to force direction.
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        if (v_ff_lin.norm() > 1e-6)
        {
          dir = -v_ff_lin.normalized();
        }
        else if (wrench_filt_.f.norm() > 1e-6)
        {
          dir = -wrench_filt_.f.normalized();
        }
        v_cmd_lin = retreat_speed_ * dir;
        v_cmd_ang.setZero();
      }
      else
      {
        // Unknown action: default to stop.
        v_cmd_lin.setZero();
        v_cmd_ang.setZero();
      }
    }

    v_cmd_lin = teleoperation::clampNorm3(v_cmd_lin, max_linear_speed_);
    v_cmd_ang = teleoperation::clampNorm3(v_cmd_ang, max_angular_speed_);

    geometry_msgs::Twist cmd;
    cmd.linear.x = v_cmd_lin.x();
    cmd.linear.y = v_cmd_lin.y();
    cmd.linear.z = v_cmd_lin.z();
    cmd.angular.x = v_cmd_ang.x();
    cmd.angular.y = v_cmd_ang.y();
    cmd.angular.z = v_cmd_ang.z();
    pub_cmd_.publish(cmd);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  teleoperation::PIDController pid_pos_;
  teleoperation::PIDController pid_ori_;

  ros::Subscriber sub_target_pose_;
  ros::Subscriber sub_ff_twist_;
  ros::Subscriber sub_wrench_;
  ros::Publisher pub_cmd_;
  teleoperation::WrenchDebugPublisher debug_wrench_filt_pub_;
  ros::Timer timer_;

  // Params
  std::string base_frame_;
  std::string tcp_frame_;
  std::string target_pose_topic_;
  std::string feedforward_twist_topic_;
  std::string wrench_topic_;
  std::string command_topic_;

  double control_rate_{250.0};
  double tf_timeout_s_{0.02};
  double target_pose_timeout_{0.2};
  double feedforward_timeout_{0.2};
  double wrench_timeout_{0.2};

  double k_ff_{1.0};
  double k_adm_linear_{0.0};
  double k_adm_angular_{0.0};
  bool use_torques_{false};

  std::string alpha_mode_{"parallel"};  // "parallel" or "norm"
  double force_start_{8.0};
  double force_stop_{30.0};

  double wrench_filter_alpha_{0.07};
  double force_deadband_{1.0};
  double torque_deadband_{0.2};
  double max_force_{150.0};
  double max_torque_{20.0};

  double max_linear_speed_{0.2};
  double max_angular_speed_{0.3};
  double max_compliance_linear_speed_{0.08};
  double max_compliance_angular_speed_{0.15};

  double hard_force_threshold_{60.0};
  double hard_force_duration_{0.03};
  std::string hard_guard_action_{"stop"};  // "stop" or "retreat"
  double retreat_speed_{0.03};

  // dt sanitization
  double dt_min_factor_{0.5};
  double dt_max_factor_{2.0};

  // Wrench TF options
  std::string wrench_source_frame_override_;
  bool use_latest_tf_for_wrench_{false};

  // State
  mutable std::mutex mutex_;
  bool has_target_{false};
  bool has_ff_{false};
  bool has_wrench_{false};
  geometry_msgs::PoseStamped target_pose_;
  geometry_msgs::TwistStamped ff_twist_;
  Wrench3 wrench_raw_;
  ros::Time target_stamp_{0};
  ros::Time ff_stamp_{0};
  ros::Time wrench_stamp_{0};

  bool has_wrench_filt_{false};
  Wrench3 wrench_filt_;

  ros::Time last_time_{0};
  ros::Time hard_guard_active_since_{0};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_slave_twist_outer_loop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopSlaveTwistOuterLoop node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_slave_twist_outer_loop failed: %s", ex.what());
    return 1;
  }
  return 0;
}

