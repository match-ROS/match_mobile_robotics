#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
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

Eigen::Vector3d applyDeadbandAbs3(const Eigen::Vector3d& v, double deadband_abs)
{
  if (deadband_abs <= 0.0) return v;
  Eigen::Vector3d out = v;
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(out[i]) < deadband_abs) out[i] = 0.0;
  }
  return out;
}

Eigen::Vector3d ema3(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return a * curr + (1.0 - a) * prev;
}

}  // namespace

class TeleopMasterMeasuredStatePublisher
{
public:
  TeleopMasterMeasuredStatePublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("tcp_frame", tcp_frame_, "tool0");

    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "target_pose");
    pnh_.param<std::string>("feedforward_twist_topic", feedforward_twist_topic_, "feedforward_twist");
    pnh_.param<std::string>("frame_id_override", frame_id_override_, "");

    pnh_.param("publish_rate", publish_rate_, publish_rate_);
    pnh_.param("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);

    pnh_.param("twist_filter_alpha", twist_filter_alpha_, twist_filter_alpha_);
    pnh_.param("twist_deadband_linear", twist_deadband_linear_, twist_deadband_linear_);
    pnh_.param("twist_deadband_angular", twist_deadband_angular_, twist_deadband_angular_);

    pnh_.param("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);

    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 1);
    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(feedforward_twist_topic_, 1);

    const double period = (publish_rate_ > 0.0) ? (1.0 / publish_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterMeasuredStatePublisher::tick, this);
  }

private:
  void tick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

    geometry_msgs::TransformStamped T;
    try
    {
      T = tf_buffer_.lookupTransform(base_frame_, tcp_frame_, ros::Time(0), ros::Duration(tf_timeout_s_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_measured_state_publisher",
                              "TF lookup failed (%s -> %s): %s",
                              base_frame_.c_str(), tcp_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = frame_id_override_.empty() ? base_frame_ : frame_id_override_;
    pose_msg.pose.position.x = T.transform.translation.x;
    pose_msg.pose.position.y = T.transform.translation.y;
    pose_msg.pose.position.z = T.transform.translation.z;
    pose_msg.pose.orientation = T.transform.rotation;

    // Build Eigen pose for twist estimation.
    Eigen::Vector3d p(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
    Eigen::Quaterniond q(T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z);
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        q.norm() < kEps)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_measured_state_publisher", "Non-finite TCP quaternion.");
      return;
    }
    q.normalize();

    Eigen::Vector3d v_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ang = Eigen::Vector3d::Zero();

    if (has_prev_pose_)
    {
      const double dt = (now - prev_time_).toSec();
      if (dt > 1e-4 && std::isfinite(dt))
      {
        v_lin = (p - prev_p_) / dt;

        Eigen::Quaterniond q_prev = prev_q_;
        Eigen::Quaterniond q_curr = q;
        // Ensure shortest path.
        if (q_prev.dot(q_curr) < 0.0) q_curr.coeffs() = -q_curr.coeffs();
        const Eigen::Quaterniond q_rel = q_prev.inverse() * q_curr;
        Eigen::AngleAxisd aa(q_rel);
        const double angle = aa.angle();
        if (std::isfinite(angle) && std::abs(angle) > 1e-10)
        {
          v_ang = (angle / dt) * aa.axis();
        }
      }
    }

    prev_p_ = p;
    prev_q_ = q;
    prev_time_ = now;
    has_prev_pose_ = true;

    // Deadband, filter, clamp.
    v_lin = applyDeadbandAbs3(v_lin, twist_deadband_linear_);
    v_ang = applyDeadbandAbs3(v_ang, twist_deadband_angular_);

    if (!has_filtered_twist_)
    {
      v_lin_filt_ = v_lin;
      v_ang_filt_ = v_ang;
      has_filtered_twist_ = true;
    }
    else if (twist_filter_alpha_ > 0.0)
    {
      v_lin_filt_ = ema3(v_lin_filt_, v_lin, twist_filter_alpha_);
      v_ang_filt_ = ema3(v_ang_filt_, v_ang, twist_filter_alpha_);
    }
    else
    {
      v_lin_filt_ = v_lin;
      v_ang_filt_ = v_ang;
    }

    v_lin_filt_ = clampNorm3(v_lin_filt_, max_linear_speed_);
    v_ang_filt_ = clampNorm3(v_ang_filt_, max_angular_speed_);

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = pose_msg.header.frame_id;
    twist_msg.twist.linear.x = v_lin_filt_.x();
    twist_msg.twist.linear.y = v_lin_filt_.y();
    twist_msg.twist.linear.z = v_lin_filt_.z();
    twist_msg.twist.angular.x = v_ang_filt_.x();
    twist_msg.twist.angular.y = v_ang_filt_.y();
    twist_msg.twist.angular.z = v_ang_filt_.z();

    pub_pose_.publish(pose_msg);
    pub_twist_.publish(twist_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher pub_pose_;
  ros::Publisher pub_twist_;
  ros::Timer timer_;

  // Params
  std::string base_frame_;
  std::string tcp_frame_;
  std::string target_pose_topic_;
  std::string feedforward_twist_topic_;
  std::string frame_id_override_;

  double publish_rate_{250.0};
  double tf_timeout_s_{0.02};

  double twist_filter_alpha_{0.2};
  double twist_deadband_linear_{0.0};
  double twist_deadband_angular_{0.0};
  double max_linear_speed_{0.3};
  double max_angular_speed_{0.6};

  // State for finite differences + filtering
  bool has_prev_pose_{false};
  ros::Time prev_time_{0};
  Eigen::Vector3d prev_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond prev_q_{Eigen::Quaterniond::Identity()};

  bool has_filtered_twist_{false};
  Eigen::Vector3d v_lin_filt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_filt_{Eigen::Vector3d::Zero()};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_master_measured_state_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopMasterMeasuredStatePublisher node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_master_measured_state_publisher failed: %s", ex.what());
    return 1;
  }
  return 0;
}

