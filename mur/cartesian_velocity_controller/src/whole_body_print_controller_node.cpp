#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "cartesian_velocity_controller/EndEffectorState.h"

namespace
{
struct PathPoint
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
};

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(rz * ry * rx).normalized();
}

geometry_msgs::Quaternion toMsg(const Eigen::Quaterniond& q)
{
  geometry_msgs::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

bool xmlRpcNumber(const XmlRpc::XmlRpcValue& v, double& out)
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

bool readPoint(const XmlRpc::XmlRpcValue& value, Eigen::Vector3d& out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray && value.size() >= 3)
  {
    double x = 0.0, y = 0.0, z = 0.0;
    if (!xmlRpcNumber(value[0], x) || !xmlRpcNumber(value[1], y) || !xmlRpcNumber(value[2], z))
    {
      return false;
    }
    out = Eigen::Vector3d(x, y, z);
    return true;
  }

  if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
      value.hasMember("x") && value.hasMember("y") && value.hasMember("z"))
  {
    double x = 0.0, y = 0.0, z = 0.0;
    if (!xmlRpcNumber(value["x"], x) || !xmlRpcNumber(value["y"], y) || !xmlRpcNumber(value["z"], z))
    {
      return false;
    }
    out = Eigen::Vector3d(x, y, z);
    return true;
  }

  return false;
}

class PolylinePath
{
public:
  bool load(const ros::NodeHandle& pnh)
  {
    XmlRpc::XmlRpcValue raw_points;
    if (!pnh.getParam("path/points", raw_points) || raw_points.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Missing required parameter path/points");
      return false;
    }

    points_.clear();
    for (int i = 0; i < raw_points.size(); ++i)
    {
      Eigen::Vector3d p;
      if (!readPoint(raw_points[i], p))
      {
        ROS_ERROR("Invalid path point at index %d", i);
        return false;
      }
      points_.push_back({p});
    }

    if (points_.size() < 2)
    {
      ROS_ERROR("path/points must contain at least two points");
      return false;
    }

    segment_lengths_.clear();
    total_length_ = 0.0;
    for (std::size_t i = 0; i + 1 < points_.size(); ++i)
    {
      const double len = (points_[i + 1].p - points_[i].p).norm();
      if (len <= 1e-9)
      {
        ROS_WARN("Skipping zero-length path segment %zu", i);
        segment_lengths_.push_back(0.0);
        continue;
      }
      segment_lengths_.push_back(len);
      total_length_ += len;
    }

    if (total_length_ <= 1e-9)
    {
      ROS_ERROR("Path total length is zero");
      return false;
    }
    return true;
  }

  Eigen::Vector3d sample(double s) const
  {
    s = clamp(s, 0.0, total_length_);
    for (std::size_t i = 0; i < segment_lengths_.size(); ++i)
    {
      const double len = segment_lengths_[i];
      if (len <= 1e-9)
      {
        continue;
      }
      if (s <= len)
      {
        const double t = s / len;
        return points_[i].p + t * (points_[i + 1].p - points_[i].p);
      }
      s -= len;
    }
    return points_.back().p;
  }

  Eigen::Vector3d tangent(double s) const
  {
    s = clamp(s, 0.0, total_length_);
    std::size_t last_valid = 0;
    for (std::size_t i = 0; i < segment_lengths_.size(); ++i)
    {
      const double len = segment_lengths_[i];
      if (len <= 1e-9)
      {
        continue;
      }
      last_valid = i;
      if (s <= len)
      {
        return (points_[i + 1].p - points_[i].p) / len;
      }
      s -= len;
    }
    const double len = segment_lengths_[last_valid];
    return (points_[last_valid + 1].p - points_[last_valid].p) / len;
  }

  const std::vector<PathPoint>& points() const { return points_; }
  double totalLength() const { return total_length_; }

private:
  std::vector<PathPoint> points_;
  std::vector<double> segment_lengths_;
  double total_length_{0.0};
};

class WholeBodyPrintController
{
public:
  WholeBodyPrintController()
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_)
  {
    loadParams();
    if (!path_.load(pnh_))
    {
      ros::shutdown();
      return;
    }

    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 1);
    base_pub_ = nh_.advertise<geometry_msgs::Twist>(base_cmd_vel_topic_, 1);
    if (lifter_enabled_)
    {
      lifter_pub_ = nh_.advertise<std_msgs::Float64>(lifter_command_topic_, 1);
      joint_state_sub_ = nh_.subscribe(joint_state_topic_, 20, &WholeBodyPrintController::jointStateCb, this);
    }
    ee_sub_ = nh_.subscribe(ee_state_topic_, 20, &WholeBodyPrintController::eeStateCb, this);

    path_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("path_marker", 1, true);
    current_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("current_marker", 1);

    pause_srv_ = pnh_.advertiseService("pause", &WholeBodyPrintController::pauseCb, this);
    resume_srv_ = pnh_.advertiseService("resume", &WholeBodyPrintController::resumeCb, this);
    restart_srv_ = pnh_.advertiseService("restart", &WholeBodyPrintController::restartCb, this);
    stop_srv_ = pnh_.advertiseService("stop", &WholeBodyPrintController::stopCb, this);

    last_time_ = ros::Time::now();
    publishPathMarker();
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, rate_hz_)),
                             &WholeBodyPrintController::timerCb, this);

    ROS_INFO("Whole-body print demo ready: frame=%s length=%.3f m speed=%.3f m/s base=%s lifter=%s",
             path_frame_.c_str(),
             path_.totalLength(),
             speed_,
             base_enabled_ ? "enabled" : "disabled",
             lifter_enabled_ ? "enabled" : "disabled");
  }

private:
  void loadParams()
  {
    pnh_.param<std::string>("path/frame_id", path_frame_, "map");
    pnh_.param("path/speed", speed_, 0.03);
    pnh_.param("path/loop", loop_, false);
    pnh_.param("start_paused", paused_, true);
    stopped_ = false;
    has_started_ = !paused_;

    std::vector<double> rpy;
    pnh_.param("path/orientation_rpy", rpy, std::vector<double>{0.0, 0.0, 0.0});
    if (rpy.size() < 3)
    {
      rpy = {0.0, 0.0, 0.0};
    }
    target_orientation_ = rpyToQuat(rpy[0], rpy[1], rpy[2]);

    pnh_.param("rate", rate_hz_, 20.0);
    pnh_.param("tracking/kp_position", kp_position_, 0.8);
    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "cartesian_velocity_controller_l/target_pose");
    pnh_.param<std::string>("ee_state_topic", ee_state_topic_, "cartesian_velocity_controller_l/end_effector_state");

    pnh_.param("base/enabled", base_enabled_, true);
    pnh_.param<std::string>("base/cmd_vel_topic", base_cmd_vel_topic_, "cmd_vel");
    pnh_.param<std::string>("base/base_frame", base_frame_, "base_link");
    pnh_.param("base/preferred_tcp_x", base_preferred_x_, 0.65);
    pnh_.param("base/preferred_tcp_y", base_preferred_y_, 0.0);
    pnh_.param("base/max_linear_velocity", base_max_linear_, 0.08);
    pnh_.param("base/max_angular_velocity", base_max_angular_, 0.25);
    pnh_.param("base/weight_linear", base_weight_linear_, 4.0);
    pnh_.param("base/weight_angular", base_weight_angular_, 2.0);
    pnh_.param("base/k_preferred_x", base_k_preferred_x_, 0.25);
    pnh_.param("base/k_lateral", base_k_lateral_, 0.6);
    pnh_.param("base/k_heading", base_k_heading_, 0.25);
    pnh_.param("base/allow_reverse", base_allow_reverse_, false);
    pnh_.param("base/align_to_path", base_align_to_path_, true);
    pnh_.param("base/tf_timeout", base_tf_timeout_, 0.05);

    pnh_.param("lifter/enabled", lifter_enabled_, false);
    pnh_.param<std::string>("lifter/joint_name", lifter_joint_name_, "left_lift_joint");
    pnh_.param<std::string>("lifter/joint_state_topic", joint_state_topic_, "joint_states");
    pnh_.param<std::string>("lifter/command_topic", lifter_command_topic_, "UR10_l/lift_position_command");
    pnh_.param("lifter/min_position", lifter_min_, 0.0);
    pnh_.param("lifter/max_position", lifter_max_, 0.5);
    pnh_.param("lifter/max_velocity", lifter_max_velocity_, 0.015);
    pnh_.param("lifter/weight", lifter_weight_, 8.0);
    pnh_.param("lifter/k_position", lifter_k_position_, 0.5);

    pnh_.param("solver/damping", solver_damping_, 0.02);
    pnh_.param("solver/task_weight_z", task_weight_z_, 0.7);
  }

  void eeStateCb(const cartesian_velocity_controller::EndEffectorState::ConstPtr& msg)
  {
    current_tcp_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    have_tcp_ = true;
  }

  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      if (msg->name[i] == lifter_joint_name_)
      {
        current_lifter_ = msg->position[i];
        have_lifter_ = true;
        if (!lifter_target_initialized_)
        {
          lifter_target_ = clamp(current_lifter_, lifter_min_, lifter_max_);
          lifter_target_initialized_ = true;
        }
        return;
      }
    }
  }

  bool pauseCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    paused_ = true;
    publishZeroBase();
    res.success = true;
    res.message = "paused";
    return true;
  }

  bool resumeCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    paused_ = false;
    stopped_ = false;
    has_started_ = true;
    last_time_ = ros::Time::now();
    res.success = true;
    res.message = "resumed";
    return true;
  }

  bool restartCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    s_ = 0.0;
    done_ = false;
    stopped_ = false;
    paused_ = true;
    has_started_ = false;
    publishZeroBase();
    res.success = true;
    res.message = "restarted";
    return true;
  }

  bool stopCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    stopped_ = true;
    paused_ = true;
    publishZeroBase();
    res.success = true;
    res.message = "stopped";
    return true;
  }

  void timerCb(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    const double dt = std::max(0.0, (now - last_time_).toSec());
    last_time_ = now;

    if (has_started_ && !paused_ && !stopped_ && !done_)
    {
      s_ += speed_ * dt;
      if (s_ >= path_.totalLength())
      {
        if (loop_)
        {
          s_ = std::fmod(s_, path_.totalLength());
        }
        else
        {
          s_ = path_.totalLength();
          done_ = true;
        }
      }
    }

    if (!has_started_ || stopped_)
    {
      publishZeroBase();
      return;
    }

    const Eigen::Vector3d target = path_.sample(s_);
    const Eigen::Vector3d tangent = path_.tangent(s_);
    publishTargetPose(target, now);
    publishCurrentMarker(target, now);

    if (!paused_ && !done_)
    {
      const Eigen::Vector3d tcp = have_tcp_ ? current_tcp_ : target;
      Eigen::Vector3d desired_linear = speed_ * tangent + kp_position_ * (target - tcp);
      desired_linear.z() *= task_weight_z_;

      Eigen::VectorXd u = solveBaseLifter(desired_linear, tcp, tangent, dt);
      publishBaseCommand(u);
      publishLifterCommand(u, dt);
    }
    else
    {
      publishZeroBase();
    }
  }

  Eigen::VectorXd solveBaseLifter(const Eigen::Vector3d& desired_linear,
                                  const Eigen::Vector3d& tcp,
                                  const Eigen::Vector3d& tangent,
                                  double dt)
  {
    const int cols = (base_enabled_ ? 2 : 0) + (lifter_enabled_ ? 1 : 0);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(std::max(cols, 1));
    if (cols == 0)
    {
      return u;
    }

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, cols);
    Eigen::VectorXd u_ref = Eigen::VectorXd::Zero(cols);
    Eigen::VectorXd reg = Eigen::VectorXd::Constant(cols, solver_damping_);

    int c = 0;
    if (base_enabled_)
    {
      geometry_msgs::TransformStamped tf_path_base;
      geometry_msgs::TransformStamped tf_base_path;
      try
      {
        tf_path_base = tf_buffer_.lookupTransform(path_frame_, base_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
        tf_base_path = tf_buffer_.lookupTransform(base_frame_, path_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "Whole-body base TF failed: %s", ex.what());
        base_tf_ok_ = false;
        return u;
      }
      base_tf_ok_ = true;

      const Eigen::Vector3d base_pos(tf_path_base.transform.translation.x,
                                     tf_path_base.transform.translation.y,
                                     tf_path_base.transform.translation.z);
      const Eigen::Quaterniond q_path_base(tf_path_base.transform.rotation.w,
                                           tf_path_base.transform.rotation.x,
                                           tf_path_base.transform.rotation.y,
                                           tf_path_base.transform.rotation.z);
      const Eigen::Vector3d base_x = q_path_base.normalized() * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d r = tcp - base_pos;
      const Eigen::Vector3d yaw_col(-r.y(), r.x(), 0.0);
      J.col(c) = base_x;
      J.col(c + 1) = yaw_col;

      Eigen::Vector3d tcp_in_base = transformPoint(tf_base_path, tcp);
      const Eigen::Vector3d tangent_in_base = rotateVector(tf_base_path, tangent);
      const double x_error = tcp_in_base.x() - base_preferred_x_;
      const double y_error = tcp_in_base.y() - base_preferred_y_;
      u_ref(c) = base_k_preferred_x_ * x_error;
      if (!base_allow_reverse_)
      {
        u_ref(c) = std::max(0.0, u_ref(c));
      }
      u_ref(c + 1) = base_k_lateral_ * y_error;
      if (base_align_to_path_)
      {
        const double heading = normalizeAngle(std::atan2(tangent_in_base.y(), tangent_in_base.x()));
        u_ref(c + 1) += base_k_heading_ * heading;
      }
      reg(c) = std::max(1e-6, base_weight_linear_);
      reg(c + 1) = std::max(1e-6, base_weight_angular_);
      c += 2;
    }

    if (lifter_enabled_)
    {
      J.col(c) = Eigen::Vector3d::UnitZ();
      double lift_ref = 0.0;
      if (have_lifter_)
      {
        if (!lifter_target_initialized_)
        {
          lifter_target_ = clamp(current_lifter_, lifter_min_, lifter_max_);
          lifter_target_initialized_ = true;
        }
        lift_ref = lifter_k_position_ * (lifter_target_ - current_lifter_);
      }
      u_ref(c) = lift_ref;
      reg(c) = std::max(1e-6, lifter_weight_);
    }

    const Eigen::Matrix3d W = (Eigen::Vector3d(1.0, 1.0, task_weight_z_)).asDiagonal();
    const Eigen::MatrixXd R = reg.asDiagonal();
    const Eigen::MatrixXd A = J.transpose() * W * J + R;
    const Eigen::VectorXd b = J.transpose() * W * desired_linear + R * u_ref;
    u = A.ldlt().solve(b);

    c = 0;
    if (base_enabled_)
    {
      u(c) = clamp(u(c), -base_max_linear_, base_max_linear_);
      if (!base_allow_reverse_)
      {
        u(c) = std::max(0.0, u(c));
      }
      u(c + 1) = clamp(u(c + 1), -base_max_angular_, base_max_angular_);
      c += 2;
    }
    if (lifter_enabled_)
    {
      const double max_step_velocity = std::max(0.0, lifter_max_velocity_);
      u(c) = clamp(u(c), -max_step_velocity, max_step_velocity);
      if (dt <= 0.0)
      {
        u(c) = 0.0;
      }
    }
    return u;
  }

  Eigen::Vector3d rotateVector(const geometry_msgs::TransformStamped& tf, const Eigen::Vector3d& v) const
  {
    const Eigen::Quaterniond q(tf.transform.rotation.w,
                               tf.transform.rotation.x,
                               tf.transform.rotation.y,
                               tf.transform.rotation.z);
    return q.normalized() * v;
  }

  Eigen::Vector3d transformPoint(const geometry_msgs::TransformStamped& tf, const Eigen::Vector3d& p) const
  {
    Eigen::Vector3d out = rotateVector(tf, p);
    out.x() += tf.transform.translation.x;
    out.y() += tf.transform.translation.y;
    out.z() += tf.transform.translation.z;
    return out;
  }

  void publishBaseCommand(const Eigen::VectorXd& u)
  {
    if (!base_enabled_ || !base_tf_ok_)
    {
      publishZeroBase();
      return;
    }
    geometry_msgs::Twist cmd;
    cmd.linear.x = u.size() >= 1 ? u(0) : 0.0;
    cmd.angular.z = u.size() >= 2 ? u(1) : 0.0;
    base_pub_.publish(cmd);
  }

  void publishLifterCommand(const Eigen::VectorXd& u, double dt)
  {
    if (!lifter_enabled_ || !have_lifter_ || !lifter_target_initialized_)
    {
      return;
    }
    const int idx = base_enabled_ ? 2 : 0;
    if (u.size() <= idx || dt <= 0.0)
    {
      return;
    }
    lifter_target_ = clamp(lifter_target_ + u(idx) * dt, lifter_min_, lifter_max_);
    std_msgs::Float64 msg;
    msg.data = lifter_target_;
    lifter_pub_.publish(msg);
  }

  void publishZeroBase()
  {
    if (base_enabled_)
    {
      base_pub_.publish(geometry_msgs::Twist());
    }
  }

  void publishTargetPose(const Eigen::Vector3d& p, const ros::Time& stamp)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = path_frame_;
    msg.pose.position.x = p.x();
    msg.pose.position.y = p.y();
    msg.pose.position.z = p.z();
    msg.pose.orientation = toMsg(target_orientation_);
    target_pub_.publish(msg);
  }

  void publishPathMarker()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = path_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "whole_body_print_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    for (const auto& p : path_.points())
    {
      geometry_msgs::Point q;
      q.x = p.p.x();
      q.y = p.p.y();
      q.z = p.p.z();
      marker.points.push_back(q);
    }
    path_marker_pub_.publish(marker);
  }

  void publishCurrentMarker(const Eigen::Vector3d& p, const ros::Time& stamp)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = path_frame_;
    marker.header.stamp = stamp;
    marker.ns = "whole_body_print_path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 0.8;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    current_marker_pub_.publish(marker);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher target_pub_;
  ros::Publisher base_pub_;
  ros::Publisher lifter_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher current_marker_pub_;
  ros::Subscriber ee_sub_;
  ros::Subscriber joint_state_sub_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer restart_srv_;
  ros::ServiceServer stop_srv_;
  ros::Timer timer_;

  PolylinePath path_;
  std::string path_frame_;
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};
  double speed_{0.03};
  double rate_hz_{20.0};
  double s_{0.0};
  bool loop_{false};
  bool paused_{true};
  bool stopped_{false};
  bool done_{false};
  bool has_started_{false};
  ros::Time last_time_;

  std::string target_pose_topic_;
  std::string ee_state_topic_;
  double kp_position_{0.8};
  Eigen::Vector3d current_tcp_{Eigen::Vector3d::Zero()};
  bool have_tcp_{false};

  bool base_enabled_{true};
  std::string base_cmd_vel_topic_;
  std::string base_frame_;
  double base_preferred_x_{0.65};
  double base_preferred_y_{0.0};
  double base_max_linear_{0.08};
  double base_max_angular_{0.25};
  double base_weight_linear_{4.0};
  double base_weight_angular_{2.0};
  double base_k_preferred_x_{0.25};
  double base_k_lateral_{0.6};
  double base_k_heading_{0.25};
  double base_tf_timeout_{0.05};
  bool base_allow_reverse_{false};
  bool base_align_to_path_{true};
  bool base_tf_ok_{true};

  bool lifter_enabled_{false};
  std::string lifter_joint_name_;
  std::string joint_state_topic_;
  std::string lifter_command_topic_;
  double lifter_min_{0.0};
  double lifter_max_{0.5};
  double lifter_max_velocity_{0.015};
  double lifter_weight_{8.0};
  double lifter_k_position_{0.5};
  double current_lifter_{0.0};
  double lifter_target_{0.0};
  bool have_lifter_{false};
  bool lifter_target_initialized_{false};

  double solver_damping_{0.02};
  double task_weight_z_{0.7};
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "whole_body_print_controller");
  WholeBodyPrintController controller;
  ros::spin();
  return 0;
}
