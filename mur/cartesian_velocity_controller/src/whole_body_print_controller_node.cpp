#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "cartesian_velocity_controller/CartesianTrajectorySetpoint.h"
#include "cartesian_velocity_controller/EndEffectorState.h"
#include "cartesian_velocity_controller/WholeBodyPrintDebug.h"

namespace
{
struct PathPoint
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
};

struct ScanAvoidanceState
{
  bool active{false};
  double min_distance{std::numeric_limits<double>::infinity()};
  double raw_omega{0.0};
  double raw_speed_scale{1.0};
  ros::Time stamp;
};

struct SimulatedObstacle
{
  std::string name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
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

geometry_msgs::Point pointToMsg(const Eigen::Vector3d& p)
{
  geometry_msgs::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
  return msg;
}

geometry_msgs::Vector3 vectorToMsg(const Eigen::Vector3d& v)
{
  geometry_msgs::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
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

bool xmlRpcBool(const XmlRpc::XmlRpcValue& v, bool& out)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    out = static_cast<bool>(v);
    return true;
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(v) != 0;
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

  if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct && value.hasMember("position"))
  {
    return readPoint(value["position"], out);
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

    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < raw_points.size(); ++i)
    {
      Eigen::Vector3d p;
      if (!readPoint(raw_points[i], p))
      {
        ROS_ERROR("Invalid path point at index %d", i);
        return false;
      }
      points.push_back(p);
    }
    return setPoints(points);
  }

  bool setPoints(const std::vector<Eigen::Vector3d>& points)
  {
    points_.clear();
    for (const auto& p : points)
    {
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
    if (!external_target_enabled_ && !path_.setPoints(configured_path_points_))
    {
      ros::shutdown();
      return;
    }

    if (origin_enabled_)
    {
      origin_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
      if (origin_capture_mode_ == "node_start" || (!paused_ && origin_capture_mode_ == "start"))
      {
        captureOrigin();
      }
    }

    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 1);
    arm_target_state_pub_ = nh_.advertise<cartesian_velocity_controller::CartesianTrajectorySetpoint>(
        arm_target_state_topic_, 1);
    base_pub_ = nh_.advertise<geometry_msgs::Twist>(base_cmd_vel_topic_, 1);
    debug_pub_ = pnh_.advertise<cartesian_velocity_controller::WholeBodyPrintDebug>("debug", 10);
    if (lifter_enabled_)
    {
      lifter_pub_ = nh_.advertise<std_msgs::Float32>(lifter_command_topic_, 1);
      joint_state_sub_ = nh_.subscribe(joint_state_topic_, 20, &WholeBodyPrintController::jointStateCb, this);
    }
    ee_sub_ = nh_.subscribe(ee_state_topic_, 20, &WholeBodyPrintController::eeStateCb, this);
    if (external_target_enabled_)
    {
      target_state_sub_ = nh_.subscribe(target_state_topic_, 20, &WholeBodyPrintController::targetStateCb, this);
    }
    if (avoidance_enabled_ && avoidance_use_laser_scans_)
    {
      for (const auto& topic : avoidance_scan_topics_)
      {
        if (!topic.empty())
        {
          scan_subs_.push_back(nh_.subscribe<sensor_msgs::LaserScan>(
              topic, 5, [this, topic](const sensor_msgs::LaserScan::ConstPtr& msg) { scanCb(msg, topic); }));
        }
      }
    }

    path_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("path_marker", 1, true);
    current_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("current_marker", 1);
    preferred_tcp_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("preferred_tcp_marker", 2, true);
    simulated_obstacle_marker_pub_ =
        pnh_.advertise<visualization_msgs::MarkerArray>("simulated_obstacle_markers", 1, true);
    reaction_perimeter_marker_pub_ =
        pnh_.advertise<visualization_msgs::MarkerArray>("reaction_perimeter_markers", 1);

    pause_srv_ = pnh_.advertiseService("pause", &WholeBodyPrintController::pauseCb, this);
    resume_srv_ = pnh_.advertiseService("resume", &WholeBodyPrintController::resumeCb, this);
    restart_srv_ = pnh_.advertiseService("restart", &WholeBodyPrintController::restartCb, this);
    stop_srv_ = pnh_.advertiseService("stop", &WholeBodyPrintController::stopCb, this);

    last_time_ = ros::Time::now();
    if (!external_target_enabled_)
    {
      publishPathMarker();
      publishSimulatedObstacleMarkers(ros::Time::now());
    }
    publishPreferredTcpMarker();
    publishReactionPerimeterMarkers(ros::Time::now());
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, path_rate_hz_)),
                             &WholeBodyPrintController::timerCb, this);

    ROS_INFO("Whole-body print demo ready: frame=%s length=%.3f m speed=%.3f m/s rates[path=%.1f base=%.1f lifter=%.1f debug=%.1f path_marker=%.1f] base=%s lifter=%s",
             path_frame_.c_str(),
             external_target_enabled_ ? 0.0 : path_.totalLength(),
             speed_,
             path_rate_hz_,
             base_rate_hz_,
             lifter_rate_hz_,
             debug_rate_hz_,
             path_marker_rate_hz_,
             base_enabled_ ? "enabled" : "disabled",
             lifter_enabled_ ? "enabled" : "disabled");
    if (simulated_obstacles_enabled_ && !simulated_obstacles_.empty())
    {
      ROS_INFO("Whole-body simulated obstacle avoidance loaded %zu circular obstacle(s) in frame %s",
               simulated_obstacles_.size(),
               path_frame_.c_str());
    }
  }

private:
  void loadParams()
  {
    pnh_.param<std::string>("path/frame_id", path_frame_, "map");
    pnh_.param("path/speed", speed_, 0.03);
    pnh_.param("path/max_linear_velocity", path_max_linear_velocity_, speed_);
    pnh_.param("path/max_linear_acceleration", path_max_linear_acceleration_, 0.0);
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

    pnh_.param("rate", path_rate_hz_, 30.0);
    pnh_.param("rates/path", path_rate_hz_, path_rate_hz_);
    pnh_.param("rates/base", base_rate_hz_, 20.0);
    pnh_.param("rates/lifter", lifter_rate_hz_, 10.0);
    pnh_.param("rates/debug", debug_rate_hz_, 10.0);
    pnh_.param("rates/path_marker", path_marker_rate_hz_, 2.0);
    pnh_.param("tracking/kp_position", kp_position_, 0.8);
    pnh_.param("tracking/arm_full_x_error", arm_full_x_error_, 0.15);
    pnh_.param("tracking/arm_full_y_error", arm_full_y_error_, 0.12);
    pnh_.param("tracking/arm_start_x_error", arm_start_x_error_, 0.45);
    pnh_.param("tracking/arm_start_y_error", arm_start_y_error_, 0.35);
    pnh_.param("tracking/arm_far_scale", arm_far_scale_, 0.0);
    pnh_.param("tracking/arm_gate_only_preposition", arm_gate_only_preposition_, false);
    pnh_.param("tracking/max_tcp_error_for_progress", max_tcp_error_for_progress_, 0.0);
    pnh_.param("tracking/max_base_x_error_for_progress", max_base_x_error_for_progress_, 0.0);
    pnh_.param("tracking/max_base_y_error_for_progress", max_base_y_error_for_progress_, 0.0);
    pnh_.param<std::string>("target_pose_topic", target_pose_topic_, "cartesian_velocity_controller_r/target_pose");
    pnh_.param("publish_target_pose", publish_target_pose_, true);
    pnh_.param("publish_target_state", publish_target_state_, false);
    pnh_.param<std::string>("arm_target_state_topic", arm_target_state_topic_, "cartesian_velocity_controller_r/target_state");
    pnh_.param("target_state_input/enabled", external_target_enabled_, false);
    pnh_.param<std::string>("target_state_input/topic", target_state_topic_, "cartesian_velocity_controller_r/target_state");
    pnh_.param("target_state_input/timeout", target_state_timeout_, 0.5);
    pnh_.param<std::string>("ee_state_topic", ee_state_topic_, "cartesian_velocity_controller_r/end_effector_state");

    pnh_.param("base/enabled", base_enabled_, true);
    pnh_.param<std::string>("base/cmd_vel_topic", base_cmd_vel_topic_, "cmd_vel");
    pnh_.param<std::string>("base/base_frame", base_frame_, "base_link");
    pnh_.param("base/preferred_tcp_x", base_preferred_x_, 0.65);
    pnh_.param("base/preferred_tcp_y", base_preferred_y_, 0.0);
    pnh_.param("base/max_linear_velocity", base_max_linear_, 0.08);
    pnh_.param("base/max_angular_velocity", base_max_angular_, 0.25);
    pnh_.param("base/max_linear_acceleration", base_max_linear_acceleration_, 0.10);
    pnh_.param("base/max_angular_acceleration", base_max_angular_acceleration_, 0.30);
    pnh_.param("base/weight_linear", base_weight_linear_, 4.0);
    pnh_.param("base/weight_angular", base_weight_angular_, 2.0);
    pnh_.param("base/k_preferred_x", base_k_preferred_x_, 0.25);
    pnh_.param("base/k_lateral", base_k_lateral_, 0.6);
    pnh_.param("base/k_heading", base_k_heading_, 0.25);
    pnh_.param("base/allow_reverse", base_allow_reverse_, false);
    pnh_.param("base/align_to_path", base_align_to_path_, true);
    pnh_.param("base/tf_timeout", base_tf_timeout_, 0.05);
    pnh_.param("base/target_filter_tau", base_target_filter_tau_, 1.0);
    loadPathSpecParams();

    pnh_.param("base_avoidance/enabled", avoidance_enabled_, false);
    pnh_.param("base_avoidance/use_laser_scans", avoidance_use_laser_scans_, true);
    pnh_.param<std::string>("base_avoidance/scan_topic", avoidance_scan_topic_, "mir/scan");
    loadScanTopics();
    pnh_.param<std::string>("base_avoidance/distance_mode", avoidance_distance_mode_, "radial");
    if (avoidance_distance_mode_ != "radial" &&
        avoidance_distance_mode_ != "longitudinal" &&
        avoidance_distance_mode_ != "rectangular" &&
        avoidance_distance_mode_ != "rounded_front" &&
        avoidance_distance_mode_ != "front_arc" &&
        avoidance_distance_mode_ != "front_axis" &&
        avoidance_distance_mode_ != "x")
    {
      ROS_WARN("Invalid base_avoidance/distance_mode='%s'; using 'radial'", avoidance_distance_mode_.c_str());
      avoidance_distance_mode_ = "radial";
    }
    pnh_.param("base_avoidance/influence_distance", avoidance_influence_distance_, 1.2);
    pnh_.param("base_avoidance/stop_distance", avoidance_stop_distance_, 0.35);
    pnh_.param("base_avoidance/slowdown_distance", avoidance_slowdown_distance_, 0.8);
    pnh_.param("base_avoidance/lateral_window", avoidance_lateral_window_, 0.9);
    pnh_.param("base_avoidance/influence_lateral_window", avoidance_influence_lateral_window_, avoidance_lateral_window_);
    pnh_.param("base_avoidance/slowdown_lateral_window", avoidance_slowdown_lateral_window_, avoidance_lateral_window_);
    pnh_.param("base_avoidance/stop_lateral_window", avoidance_stop_lateral_window_, avoidance_lateral_window_);
    avoidance_influence_lateral_window_ = std::max(0.0, avoidance_influence_lateral_window_);
    avoidance_slowdown_lateral_window_ = std::max(0.0, avoidance_slowdown_lateral_window_);
    avoidance_stop_lateral_window_ = std::max(0.0, avoidance_stop_lateral_window_);
    pnh_.param("base_avoidance/front_min_x", avoidance_front_min_x_, 0.05);
    pnh_.param("base_avoidance/k_omega", avoidance_k_omega_, 0.8);
    pnh_.param("base_avoidance/max_omega", avoidance_max_omega_, 0.35);
    pnh_.param("base_avoidance/filter_tau", avoidance_filter_tau_, 0.4);
    pnh_.param("base_avoidance/stale_timeout", avoidance_stale_timeout_, 0.5);
    pnh_.param("base_avoidance/perimeter_marker_rate", avoidance_perimeter_marker_rate_hz_, path_marker_rate_hz_);
    avoidance_perimeter_marker_rate_hz_ = std::max(0.0, avoidance_perimeter_marker_rate_hz_);

    pnh_.param("lifter/enabled", lifter_enabled_, false);
    pnh_.param<std::string>("lifter/joint_name", lifter_joint_name_, "right_lift_joint");
    pnh_.param<std::string>("lifter/joint_state_topic", joint_state_topic_, "joint_states");
    pnh_.param<std::string>("lifter/command_topic", lifter_command_topic_, "UR10_r/ewellix_tlt_node_r/command");
    pnh_.param("lifter/min_position", lifter_min_, 0.0);
    pnh_.param("lifter/max_position", lifter_max_, 0.5);
    pnh_.param("lifter/max_velocity", lifter_max_velocity_, 0.015);
    pnh_.param("lifter/weight", lifter_weight_, 8.0);
    pnh_.param("lifter/k_position", lifter_k_position_, 0.5);

    pnh_.param("solver/damping", solver_damping_, 0.02);
    pnh_.param("solver/task_weight_z", task_weight_z_, 0.7);
    pnh_.param("preposition/enabled", preposition_enabled_, true);
    pnh_.param<std::string>("preposition/reached_mode", preposition_reached_mode_, "base_target");
    pnh_.param("preposition/position_tolerance", start_position_tolerance_, 0.01);
    pnh_.param("preposition/base_target_tolerance", start_base_target_tolerance_, 0.15);
    pnh_.param("preposition/dwell_s", preposition_dwell_s_, 1.0);
    pnh_.param("resume_delay_s", resume_delay_s_, 0.0);
    pnh_.param("preposition/resume_delay_s", resume_delay_s_, resume_delay_s_);
    resume_delay_s_ = std::max(0.0, resume_delay_s_);

    if (preposition_reached_mode_ != "base_target" && preposition_reached_mode_ != "tcp")
    {
      ROS_WARN("Invalid preposition/reached_mode='%s'; falling back to 'base_target'",
               preposition_reached_mode_.c_str());
      preposition_reached_mode_ = "base_target";
    }
    preposition_done_ = !preposition_enabled_;

    if (publish_target_pose_ && publish_target_state_)
    {
      ROS_WARN("whole_body_print_controller: both publish_target_pose and publish_target_state are enabled; target_pose can disable trajectory mode in the arm controller");
    }
  }

  void loadPathSpecParams()
  {
    if (!pnh_.hasParam("path_spec/waypoints") && !pnh_.hasParam("path_spec/points"))
    {
      if (!pnh_.getParam("path/points", configured_path_points_raw_))
      {
        return;
      }
      configured_path_points_ = parsePathPoints(configured_path_points_raw_, "path/points", "absolute");
      loadSimulatedObstacleParams();
      return;
    }

    pnh_.param<std::string>("path_spec/frame_id", path_frame_, path_frame_);
    pnh_.param("path_spec/speed", speed_, speed_);
    pnh_.param("path_spec/max_linear_velocity", path_max_linear_velocity_, path_max_linear_velocity_);
    pnh_.param("path_spec/max_linear_acceleration", path_max_linear_acceleration_, path_max_linear_acceleration_);
    pnh_.param("path_spec/loop", loop_, loop_);
    if (path_max_linear_velocity_ > 1e-9)
    {
      speed_ = std::min(speed_, path_max_linear_velocity_);
    }

    std::vector<double> rpy;
    if (pnh_.getParam("path_spec/orientation_rpy", rpy) && rpy.size() >= 3)
    {
      target_orientation_ = rpyToQuat(rpy[0], rpy[1], rpy[2]);
    }

    pnh_.param("path_spec/path_origin/enabled", origin_enabled_, false);
    pnh_.param<std::string>("path_spec/path_origin/capture", origin_capture_mode_, "node_start");
    pnh_.param<std::string>("path_spec/path_origin/parent_frame", origin_parent_frame_, path_frame_);
    pnh_.param<std::string>("path_spec/path_origin/source_frame", origin_source_frame_, base_frame_);
    pnh_.param<std::string>("path_spec/path_origin/frame_id", origin_frame_id_, path_frame_);
    pnh_.param("path_spec/path_origin/capture_timeout", origin_capture_timeout_, 2.0);
    if (origin_enabled_)
    {
      path_frame_ = origin_frame_id_;
    }

    std::string waypoint_mode = "absolute";
    pnh_.param<std::string>("path_spec/waypoint_mode", waypoint_mode, "absolute");

    XmlRpc::XmlRpcValue raw_points;
    const bool has_waypoints = pnh_.getParam("path_spec/waypoints", raw_points);
    const bool has_points = !has_waypoints && pnh_.getParam("path_spec/points", raw_points);
    if (!has_waypoints && !has_points)
    {
      return;
    }
    const std::string source = has_waypoints ? "path_spec/waypoints" : "path_spec/points";
    configured_path_points_ = parsePathPoints(raw_points, source, waypoint_mode);
    loadSimulatedObstacleParams();
  }

  void loadSimulatedObstacleParams()
  {
    simulated_obstacles_.clear();
    simulated_obstacles_enabled_ = false;

    XmlRpc::XmlRpcValue raw;
    std::string source = "path_spec/obstacles";
    if (!pnh_.getParam(source, raw))
    {
      source = "path_spec/simulated_obstacles";
      if (!pnh_.getParam(source, raw))
      {
        return;
      }
    }

    XmlRpc::XmlRpcValue raw_items = raw;
    bool relative_to_first = false;
    bool relative_z = false;

    if (raw.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      if (raw.hasMember("enabled") && !xmlRpcBool(raw["enabled"], simulated_obstacles_enabled_))
      {
        ROS_WARN("Ignoring non-boolean %s/enabled", source.c_str());
      }
      if (!raw.hasMember("enabled"))
      {
        simulated_obstacles_enabled_ = true;
      }
      if (raw.hasMember("relative_to_first_waypoint") &&
          !xmlRpcBool(raw["relative_to_first_waypoint"], relative_to_first))
      {
        ROS_WARN("Ignoring non-boolean %s/relative_to_first_waypoint", source.c_str());
      }
      if (raw.hasMember("relative_z") && !xmlRpcBool(raw["relative_z"], relative_z))
      {
        ROS_WARN("Ignoring non-boolean %s/relative_z", source.c_str());
      }
      if (raw.hasMember("marker_height"))
      {
        double marker_height = simulated_obstacle_marker_height_;
        if (xmlRpcNumber(raw["marker_height"], marker_height))
        {
          simulated_obstacle_marker_height_ = std::max(0.01, marker_height);
        }
      }

      if (raw.hasMember("items"))
      {
        raw_items = raw["items"];
      }
      else if (raw.hasMember("list"))
      {
        raw_items = raw["list"];
      }
      else
      {
        ROS_WARN("%s must contain an 'items' array", source.c_str());
        return;
      }
    }
    else if (raw.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      simulated_obstacles_enabled_ = true;
    }

    if (!simulated_obstacles_enabled_)
    {
      return;
    }
    if (raw_items.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("%s items must be an array", source.c_str());
      return;
    }

    const Eigen::Vector3d first = configured_path_points_.empty()
                                      ? Eigen::Vector3d::Zero()
                                      : configured_path_points_.front();

    for (int i = 0; i < raw_items.size(); ++i)
    {
      SimulatedObstacle obstacle;
      obstacle.name = "obstacle_" + std::to_string(i + 1);
      bool item_enabled = true;
      bool valid = false;

      if (raw_items[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        const XmlRpc::XmlRpcValue& item = raw_items[i];
        if (item.hasMember("enabled") && !xmlRpcBool(item["enabled"], item_enabled))
        {
          ROS_WARN("Ignoring non-boolean %s/items[%d]/enabled", source.c_str(), i);
        }
        if (item.hasMember("name") && item["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          obstacle.name = static_cast<std::string>(item["name"]);
        }
        if (item.hasMember("position"))
        {
          valid = readPoint(item["position"], obstacle.center);
        }
        else if (item.hasMember("center"))
        {
          valid = readPoint(item["center"], obstacle.center);
        }
        else
        {
          valid = readPoint(item, obstacle.center);
        }
        if (!item.hasMember("radius") || !xmlRpcNumber(item["radius"], obstacle.radius))
        {
          ROS_WARN("Invalid or missing radius for %s/items[%d]", source.c_str(), i);
          valid = false;
        }
      }
      else if (raw_items[i].getType() == XmlRpc::XmlRpcValue::TypeArray && raw_items[i].size() >= 4)
      {
        valid = readPoint(raw_items[i], obstacle.center) && xmlRpcNumber(raw_items[i][3], obstacle.radius);
      }

      if (!item_enabled)
      {
        continue;
      }
      if (!valid || obstacle.radius <= 0.0)
      {
        ROS_WARN("Skipping invalid simulated obstacle at %s/items[%d]", source.c_str(), i);
        continue;
      }

      if (relative_to_first)
      {
        obstacle.center.x() += first.x();
        obstacle.center.y() += first.y();
        if (relative_z)
        {
          obstacle.center.z() += first.z();
        }
      }
      simulated_obstacles_.push_back(obstacle);
    }
  }

  std::vector<Eigen::Vector3d> parsePathPoints(const XmlRpc::XmlRpcValue& raw_points,
                                               const std::string& source_name,
                                               const std::string& waypoint_mode) const
  {
    std::vector<Eigen::Vector3d> points;
    if (raw_points.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("%s must be an array", source_name.c_str());
      return points;
    }

    for (int i = 0; i < raw_points.size(); ++i)
    {
      Eigen::Vector3d p;
      if (!readPoint(raw_points[i], p))
      {
        ROS_ERROR("Invalid point in %s at index %d", source_name.c_str(), i);
        points.clear();
        return points;
      }
      points.push_back(p);
    }

    if (waypoint_mode == "first_absolute_then_relative" ||
        waypoint_mode == "first_absolute_rest_relative" ||
        waypoint_mode == "first_absolute_then_offsets")
    {
      if (!points.empty())
      {
        const Eigen::Vector3d first = points.front();
        for (std::size_t i = 1; i < points.size(); ++i)
        {
          points[i] = first + points[i];
        }
      }
    }
    return points;
  }

  void loadScanTopics()
  {
    avoidance_scan_topics_.clear();

    XmlRpc::XmlRpcValue raw_topics;
    if (pnh_.getParam("base_avoidance/scan_topics", raw_topics) &&
        raw_topics.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < raw_topics.size(); ++i)
      {
        if (raw_topics[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_WARN("Ignoring non-string base_avoidance/scan_topics[%d]", i);
          continue;
        }
        const std::string topic = static_cast<std::string>(raw_topics[i]);
        if (!topic.empty() &&
            std::find(avoidance_scan_topics_.begin(), avoidance_scan_topics_.end(), topic) == avoidance_scan_topics_.end())
        {
          avoidance_scan_topics_.push_back(topic);
        }
      }
    }

    if (!avoidance_scan_topic_.empty() &&
        std::find(avoidance_scan_topics_.begin(), avoidance_scan_topics_.end(), avoidance_scan_topic_) ==
            avoidance_scan_topics_.end())
    {
      avoidance_scan_topics_.push_back(avoidance_scan_topic_);
    }

    if (avoidance_scan_topics_.empty())
    {
      avoidance_scan_topics_.push_back("mir/scan");
    }
  }

  void eeStateCb(const cartesian_velocity_controller::EndEffectorState::ConstPtr& msg)
  {
    current_tcp_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    current_tcp_frame_ = msg->header.frame_id;
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

  void targetStateCb(const cartesian_velocity_controller::CartesianTrajectorySetpoint::ConstPtr& msg)
  {
    if (!msg->header.frame_id.empty())
    {
      path_frame_ = msg->header.frame_id;
    }
    external_target_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    external_velocity_ = Eigen::Vector3d(msg->velocity.linear.x, msg->velocity.linear.y, msg->velocity.linear.z);
    Eigen::Quaterniond q(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
    if (q.norm() > 1e-9)
    {
      target_orientation_ = q.normalized();
    }
    external_active_ = msg->active;
    external_path_s_ = msg->path_s;
    external_path_progress_ = msg->path_progress;
    if (external_path_progress_ > 1e-6)
    {
      external_path_length_ = std::max(external_path_length_, external_path_s_ / external_path_progress_);
    }
    external_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_external_target_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& source_topic)
  {
    double omega_sum = 0.0;
    double weight_sum = 0.0;
    double min_dist = std::numeric_limits<double>::infinity();
    double raw_speed_scale = 1.0;
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    geometry_msgs::TransformStamped tf_base_scan;
    const bool need_tf = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
    if (need_tf)
    {
      try
      {
        tf_base_scan = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id,
                                                  ros::Time(0), ros::Duration(base_tf_timeout_));
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "Laser avoidance TF failed (%s -> %s): %s",
                          msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        scan_avoidance_states_[source_topic] =
            ScanAvoidanceState{false, std::numeric_limits<double>::infinity(), 0.0, 1.0, stamp};
        return;
      }
    }

    for (std::size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
      {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      Eigen::Vector3d p(r * std::cos(angle), r * std::sin(angle), 0.0);
      if (need_tf)
      {
        p = transformPoint(tf_base_scan, p);
      }

      const bool in_stop = pointInAvoidanceZone(p, avoidance_stop_distance_, avoidance_stop_lateral_window_);
      const bool in_slowdown = pointInAvoidanceZone(p, avoidance_slowdown_distance_, avoidance_slowdown_lateral_window_);
      const bool in_influence = pointInAvoidanceZone(p, avoidance_influence_distance_, avoidance_influence_lateral_window_);
      if (!in_stop && !in_slowdown && !in_influence)
      {
        continue;
      }

      const double influence_dist = in_influence
                                        ? avoidancePointDistance(p, avoidance_influence_lateral_window_)
                                        : (in_slowdown
                                               ? avoidancePointDistance(p, avoidance_slowdown_lateral_window_)
                                               : avoidancePointDistance(p, avoidance_stop_lateral_window_));
      min_dist = std::min(min_dist, influence_dist);
      if (in_slowdown || in_stop)
      {
        raw_speed_scale = std::min(raw_speed_scale, pointAvoidanceSpeedScale(p, in_stop));
      }

      const double influence = clamp((avoidance_influence_distance_ - influence_dist) /
                                     std::max(1e-6, avoidance_influence_distance_ - avoidance_stop_distance_),
                                     0.0, 1.0);
      const double side = (std::abs(p.y()) > 1e-4) ? (p.y() > 0.0 ? 1.0 : -1.0) : 1.0;
      // y > 0 means obstacle left: rotate right. y < 0 means obstacle right: rotate left.
      omega_sum += -side * influence * influence;
      weight_sum += influence;
    }

    ScanAvoidanceState state;
    state.stamp = stamp;
    if (weight_sum > 1e-6)
    {
      state.active = true;
      state.min_distance = min_dist;
      state.raw_speed_scale = raw_speed_scale;
      state.raw_omega = clamp(avoidance_k_omega_ * omega_sum / weight_sum,
                              -avoidance_max_omega_, avoidance_max_omega_);
    }
    else
    {
      state.active = false;
      state.min_distance = std::numeric_limits<double>::infinity();
      state.raw_speed_scale = 1.0;
      state.raw_omega = 0.0;
    }
    scan_avoidance_states_[source_topic] = state;
    updateAggregatedAvoidance(stamp);
  }

  bool avoidanceUsesLongitudinalDistance() const
  {
    return avoidance_distance_mode_ == "longitudinal" ||
           avoidance_distance_mode_ == "rectangular" ||
           avoidance_distance_mode_ == "front_axis" ||
           avoidance_distance_mode_ == "x";
  }

  bool avoidanceUsesRoundedFrontDistance() const
  {
    return avoidance_distance_mode_ == "rounded_front" ||
           avoidance_distance_mode_ == "front_arc";
  }

  double avoidancePointDistance(const Eigen::Vector3d& p, double lateral_window) const
  {
    if (avoidanceUsesRoundedFrontDistance())
    {
      const double r2 = p.x() * p.x() + p.y() * p.y();
      const double lateral2 = lateral_window * lateral_window;
      return std::sqrt(std::max(0.0, r2 - lateral2));
    }
    if (avoidanceUsesLongitudinalDistance())
    {
      return p.x();
    }
    return std::hypot(p.x(), p.y());
  }

  double avoidanceObstacleSurfaceDistance(const Eigen::Vector3d& p, double radius, double lateral_window) const
  {
    if (avoidanceUsesRoundedFrontDistance())
    {
      const double surface_radius = std::max(0.0, std::hypot(p.x(), p.y()) - radius);
      const double lateral2 = lateral_window * lateral_window;
      return std::sqrt(std::max(0.0, surface_radius * surface_radius - lateral2));
    }
    if (avoidanceUsesLongitudinalDistance())
    {
      return p.x() - radius;
    }
    return std::hypot(p.x(), p.y()) - radius;
  }

  bool pointInAvoidanceZone(const Eigen::Vector3d& p, double distance, double lateral_window) const
  {
    if (distance <= 1e-6 || lateral_window <= 1e-6)
    {
      return false;
    }
    if (p.x() < avoidance_front_min_x_ || std::abs(p.y()) > lateral_window)
    {
      return false;
    }
    return avoidancePointDistance(p, lateral_window) <= distance;
  }

  bool obstacleInAvoidanceZone(const Eigen::Vector3d& p, double radius, double distance, double lateral_window) const
  {
    if (distance <= 1e-6 || lateral_window <= 1e-6)
    {
      return false;
    }
    if (p.x() + radius < avoidance_front_min_x_ || std::abs(p.y()) - radius > lateral_window)
    {
      return false;
    }
    return avoidanceObstacleSurfaceDistance(p, radius, lateral_window) <= distance;
  }

  double pointAvoidanceSpeedScale(const Eigen::Vector3d& p, bool in_stop) const
  {
    if (in_stop)
    {
      return 0.0;
    }
    const double slowdown_dist = avoidancePointDistance(p, avoidance_slowdown_lateral_window_);
    const double scale = clamp((slowdown_dist - avoidance_stop_distance_) /
                                   std::max(1e-6, avoidance_slowdown_distance_ - avoidance_stop_distance_),
                               0.0, 1.0);
    return std::max(1e-3, scale);
  }

  double obstacleAvoidanceSpeedScale(const Eigen::Vector3d& p, double radius, bool in_stop) const
  {
    if (in_stop)
    {
      return 0.0;
    }
    const double slowdown_dist = avoidanceObstacleSurfaceDistance(p, radius, avoidance_slowdown_lateral_window_);
    const double scale = clamp((slowdown_dist - avoidance_stop_distance_) /
                                   std::max(1e-6, avoidance_slowdown_distance_ - avoidance_stop_distance_),
                               0.0, 1.0);
    return std::max(1e-3, scale);
  }

  void updateAggregatedAvoidance(const ros::Time& now)
  {
    avoidance_active_ = false;
    avoidance_min_distance_ = std::numeric_limits<double>::infinity();
    avoidance_raw_omega_ = 0.0;
    avoidance_raw_speed_scale_ = 1.0;
    avoidance_last_scan_time_ = ros::Time();

    double omega_sum = 0.0;
    double weight_sum = 0.0;
    for (const auto& kv : scan_avoidance_states_)
    {
      const ScanAvoidanceState& state = kv.second;
      if (!state.stamp.isZero() && (avoidance_last_scan_time_.isZero() || state.stamp > avoidance_last_scan_time_))
      {
        avoidance_last_scan_time_ = state.stamp;
      }
      if (state.stamp.isZero() || (now - state.stamp).toSec() > avoidance_stale_timeout_ || !state.active)
      {
        continue;
      }
      const double distance_weight = 1.0 / std::max(0.05, state.min_distance);
      omega_sum += state.raw_omega * distance_weight;
      weight_sum += distance_weight;
      avoidance_raw_speed_scale_ = std::min(avoidance_raw_speed_scale_, state.raw_speed_scale);
      avoidance_min_distance_ = std::min(avoidance_min_distance_, state.min_distance);
      avoidance_active_ = true;
    }

    if (weight_sum > 1e-9)
    {
      avoidance_raw_omega_ = clamp(omega_sum / weight_sum, -avoidance_max_omega_, avoidance_max_omega_);
    }
  }

  void updateResumeDelayParam()
  {
    double delay = resume_delay_s_;
    pnh_.param("resume_delay_s", delay, delay);
    pnh_.param("preposition/resume_delay_s", delay, delay);
    resume_delay_s_ = std::max(0.0, delay);
  }

  bool resumeDelayActive(const ros::Time& now)
  {
    if (!resume_delay_active_ || paused_ || stopped_ || !has_started_)
    {
      return false;
    }
    if (now < resume_delay_until_)
    {
      return true;
    }

    resume_delay_active_ = false;
    resume_delay_until_ = ros::Time();
    last_time_ = now;
    ROS_INFO("Whole-body resume delay elapsed; starting prepositioning");
    return true;
  }

  bool pauseCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    paused_ = true;
    resume_delay_active_ = false;
    resume_delay_until_ = ros::Time();
    publishZeroBase();
    res.success = true;
    res.message = "paused";
    return true;
  }

  bool resumeCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    const ros::Time now = ros::Time::now();
    if (origin_enabled_ && origin_capture_mode_ == "start" && !origin_captured_)
    {
      captureOrigin();
    }
    updateResumeDelayParam();
    paused_ = false;
    stopped_ = false;
    has_started_ = true;
    last_time_ = now;
    if (resume_delay_s_ > 1e-9)
    {
      resume_delay_active_ = true;
      resume_delay_until_ = now + ros::Duration(resume_delay_s_);
      publishZeroBase();
      res.message = "resumed; waiting " + std::to_string(resume_delay_s_) + " s before prepositioning";
    }
    else
    {
      resume_delay_active_ = false;
      resume_delay_until_ = ros::Time();
      res.message = "resumed";
    }
    res.success = true;
    return true;
  }

  bool restartCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    s_ = 0.0;
    path_speed_current_ = 0.0;
    done_ = false;
    stopped_ = false;
    paused_ = true;
    has_started_ = false;
    preposition_done_ = !preposition_enabled_;
    dwell_started_ = false;
    resume_delay_active_ = false;
    resume_delay_until_ = ros::Time();
    if (origin_enabled_ && origin_capture_mode_ == "start")
    {
      origin_captured_ = false;
    }
    publishZeroBase();
    res.success = true;
    res.message = "restarted";
    return true;
  }

  bool stopCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    stopped_ = true;
    paused_ = true;
    resume_delay_active_ = false;
    resume_delay_until_ = ros::Time();
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
    publishOriginTransform(now);
    publishReactionPerimeterMarkersIfDue(now);

    if (external_target_enabled_)
    {
      timerCbExternal(now, dt);
      return;
    }

    if (!pathFrameReady())
    {
      publishZeroBase();
      return;
    }
    publishPathMarkerIfDue(now);

    const Eigen::Vector3d start_target = path_.sample(0.0);
    const Eigen::Vector3d start_tangent = path_.tangent(0.0);

    if (resumeDelayActive(now))
    {
      const Eigen::Vector3d tcp = currentTcpInPath(start_target);
      publishArmReference(tcp, Eigen::Vector3d::Zero(), now, false);
      publishZeroBase();
      publishCurrentMarker(start_target, now);
      publishDebug(now,
                   stateString(),
                   start_target,
                   tcp,
                   tcp,
                   targetInBase(start_target),
                   0.0);
      return;
    }

    if (has_started_ && !paused_ && !stopped_ && !done_ && preposition_done_)
    {
      const Eigen::Vector3d progress_target = path_.sample(s_);
      const Eigen::Vector3d progress_tcp = currentTcpInPath(progress_target);
      const Eigen::Vector3d progress_target_in_base = targetInBase(progress_target);
      const double tcp_error_for_progress = have_tcp_ ? (progress_target - progress_tcp).norm() : 0.0;
      const double base_x_error_for_progress = std::abs(progress_target_in_base.x() - base_preferred_x_);
      const double base_y_error_for_progress = std::abs(progress_target_in_base.y() - base_preferred_y_);
      const bool tcp_ok = max_tcp_error_for_progress_ <= 0.0 ||
                          tcp_error_for_progress <= max_tcp_error_for_progress_;
      const bool base_x_ok = max_base_x_error_for_progress_ <= 0.0 ||
                             base_x_error_for_progress <= max_base_x_error_for_progress_;
      const bool base_y_ok = max_base_y_error_for_progress_ <= 0.0 ||
                             base_y_error_for_progress <= max_base_y_error_for_progress_;

      if (tcp_ok && base_x_ok && base_y_ok)
      {
        updatePathSpeed(pathSpeedTarget(path_.totalLength() - s_), dt);
        s_ += path_speed_current_ * dt;
        if (s_ >= path_.totalLength())
        {
          if (loop_)
          {
            s_ = std::fmod(s_, path_.totalLength());
            path_speed_current_ = std::min(path_speed_current_, speed_);
          }
          else
          {
            s_ = path_.totalLength();
            done_ = true;
          }
        }
      }
      else
      {
        updatePathSpeed(0.0, dt);
        ROS_WARN_THROTTLE(1.0,
                          "Holding path progress: tcp_error=%.3f/%.3f base_error=[%.3f/%.3f, %.3f/%.3f]",
                          tcp_error_for_progress,
                          max_tcp_error_for_progress_,
                          base_x_error_for_progress,
                          max_base_x_error_for_progress_,
                          base_y_error_for_progress,
                          max_base_y_error_for_progress_);
      }
    }

    if (!has_started_ || stopped_)
    {
      publishArmReference(currentTcpInPath(start_target), Eigen::Vector3d::Zero(), now, false);
      publishZeroBase();
      publishDebug(now,
                   stateString(),
                   path_.sample(s_),
                   path_.sample(s_),
                   currentTcpInPath(path_.sample(s_)),
                   Eigen::Vector3d::Zero(),
                   0.0);
      return;
    }

    if (paused_)
    {
      publishArmReference(currentTcpInPath(start_target), Eigen::Vector3d::Zero(), now, false);
      publishZeroBase();
      publishDebug(now,
                   stateString(),
                   start_target,
                   currentTcpInPath(start_target),
                   currentTcpInPath(start_target),
                   targetInBase(start_target),
                   0.0);
      return;
    }

    if (!preposition_done_)
    {
      updateBaseTrackingTarget(start_target, dt);
      const Eigen::Vector3d tcp = currentTcpInPath(start_target);
      const Eigen::Vector3d target_in_base = targetInBase(start_target);
      const Eigen::Vector3d arm_target = start_target;
      Eigen::Vector3d arm_velocity = Eigen::Vector3d::Zero();
      publishCurrentMarker(start_target, now);

      if (startReached(start_target))
      {
        if (!dwell_started_)
        {
          dwell_started_ = true;
          dwell_start_time_ = now;
          ROS_INFO("Whole-body preposition reached; dwelling for %.2f s", preposition_dwell_s_);
        }
        publishSmoothZeroBase(now);
        if ((now - dwell_start_time_).toSec() >= std::max(0.0, preposition_dwell_s_))
        {
          preposition_done_ = true;
          dwell_started_ = false;
          path_speed_current_ = 0.0;
          last_time_ = now;
          ROS_INFO("Whole-body path tracking started");
        }
        publishArmReference(arm_target, Eigen::Vector3d::Zero(), now, false);
      }
      else
      {
        dwell_started_ = false;
        Eigen::Vector3d desired_linear = kp_position_ * (base_tracking_target_ - tcp);
        desired_linear.z() *= task_weight_z_;
        Eigen::VectorXd u = solveBaseLifter(desired_linear,
                                           base_tracking_target_,
                                           start_tangent,
                                           currentTcpInBase(target_in_base),
                                           dt);
        arm_velocity = desired_linear - computeExternalTcpVelocityInPath(base_tracking_target_, u);
        publishArmReference(arm_target, arm_velocity, now, true);
        publishBaseCommand(u, now);
        publishLifterCommand(u, dt, now);
      }

      publishDebug(now,
                   stateString(),
                   start_target,
                   arm_target,
                   tcp,
                   target_in_base,
                   1.0);
      return;
    }

    const Eigen::Vector3d target = path_.sample(s_);
    const Eigen::Vector3d tangent = path_.tangent(s_);
    updateBaseTrackingTarget(target, dt);

    const Eigen::Vector3d tcp = currentTcpInPath(target);
    const Eigen::Vector3d target_in_base = targetInBase(target);
    const Eigen::Vector3d tcp_in_base = currentTcpInBase(target_in_base);
    const bool gate_arm_by_base_zone = !arm_gate_only_preposition_;
    const double arm_scale = gate_arm_by_base_zone ? computeArmTrackingScale(target_in_base, tcp_in_base) : 1.0;
    const Eigen::Vector3d arm_target = have_tcp_ ? tcp + arm_scale * (target - tcp) : target;
    Eigen::Vector3d arm_velocity = Eigen::Vector3d::Zero();

    publishCurrentMarker(target, now);

    if (!paused_ && !done_)
    {
      Eigen::Vector3d desired_linear = path_speed_current_ * tangent + kp_position_ * (base_tracking_target_ - tcp);
      desired_linear.z() *= task_weight_z_;

      Eigen::VectorXd u = solveBaseLifter(desired_linear, base_tracking_target_, tangent, tcp_in_base, dt);
      arm_velocity = desired_linear - computeExternalTcpVelocityInPath(base_tracking_target_, u);
      publishArmReference(arm_target, arm_velocity, now, true);
      publishBaseCommand(u, now);
      publishLifterCommand(u, dt, now);
    }
    else
    {
      publishArmReference(tcp, Eigen::Vector3d::Zero(), now, false);
      if (done_)
      {
        publishSmoothZeroBase(now);
      }
      else
      {
        publishZeroBase();
      }
    }

    publishDebug(now, stateString(), target, arm_target, tcp, target_in_base, arm_scale);
  }

  void timerCbExternal(const ros::Time& now, double dt)
  {
    if (!have_external_target_ ||
        (target_state_timeout_ > 0.0 && !external_stamp_.isZero() &&
         (now - external_stamp_).toSec() > target_state_timeout_))
    {
      publishZeroBase();
      publishDebug(now,
                   have_external_target_ ? "target_timeout" : "waiting_target",
                   external_target_,
                   external_target_,
                   currentTcpInPath(external_target_),
                   Eigen::Vector3d::Zero(),
                   0.0);
      return;
    }

    s_ = external_path_s_;
    const Eigen::Vector3d target = external_target_;
    const double speed = external_velocity_.norm();
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitX();
    if (speed > 1e-9)
    {
      tangent = external_velocity_ / speed;
    }
    updateBaseTrackingTarget(target, dt);

    const Eigen::Vector3d tcp = currentTcpInPath(target);
    const Eigen::Vector3d target_in_base = targetInBase(target);
    const Eigen::Vector3d tcp_in_base = currentTcpInBase(target_in_base);
    const bool prepositioning = !external_active_ && external_path_s_ <= 1e-6 && external_path_progress_ <= 1e-6;
    const bool gate_arm_by_base_zone = !arm_gate_only_preposition_ || prepositioning;
    const double arm_scale = gate_arm_by_base_zone ? computeArmTrackingScale(target_in_base, tcp_in_base) : 1.0;
    const Eigen::Vector3d arm_target = have_tcp_ ? tcp + arm_scale * (target - tcp) : target;
    Eigen::Vector3d arm_velocity = Eigen::Vector3d::Zero();

    publishCurrentMarker(target, now);

    if (!paused_ && !stopped_ && (external_active_ || prepositioning))
    {
      Eigen::Vector3d desired_linear = external_velocity_ + kp_position_ * (base_tracking_target_ - tcp);
      desired_linear.z() *= task_weight_z_;

      Eigen::VectorXd u = solveBaseLifter(desired_linear, base_tracking_target_, tangent, tcp_in_base, dt);
      if (prepositioning)
      {
        publishArmReference(tcp, Eigen::Vector3d::Zero(), now, false);
      }
      else
      {
        arm_velocity = desired_linear - computeExternalTcpVelocityInPath(base_tracking_target_, u);
        publishArmReference(arm_target, arm_velocity, now, true);
      }
      publishBaseCommand(u, now);
      publishLifterCommand(u, dt, now);
    }
    else
    {
      publishArmReference(tcp, Eigen::Vector3d::Zero(), now, false);
      publishZeroBase();
    }

    publishDebug(now, stateString(), target, arm_target, tcp, target_in_base, arm_scale);
  }

  Eigen::VectorXd solveBaseLifter(const Eigen::Vector3d& desired_linear,
                                  const Eigen::Vector3d& base_reference_point,
                                  const Eigen::Vector3d& tangent,
                                  const Eigen::Vector3d& tcp_in_base,
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
      const Eigen::Vector3d r = base_reference_point - base_pos;
      const Eigen::Vector3d yaw_col(-r.y(), r.x(), 0.0);
      J.col(c) = base_x;
      J.col(c + 1) = yaw_col;

      const Eigen::Vector3d target_in_base = transformPoint(tf_base_path, base_reference_point);
      const Eigen::Vector3d tangent_in_base = rotateVector(tf_base_path, tangent);
      const Eigen::Vector3d zone_reference_in_base = have_tcp_ ? tcp_in_base : target_in_base;
      const double x_error = zone_reference_in_base.x() - base_preferred_x_;
      const double y_error = zone_reference_in_base.y() - base_preferred_y_;
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
      last_base_linear_saturated_ = std::abs(u(c)) > base_max_linear_;
      u(c) = clamp(u(c), -base_max_linear_, base_max_linear_);
      if (!base_allow_reverse_)
      {
        u(c) = std::max(0.0, u(c));
      }
      last_base_angular_saturated_ = std::abs(u(c + 1)) > base_max_angular_;
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
      last_lifter_velocity_command_ = u(c);
    }
    return u;
  }

  Eigen::Vector3d currentTcpInPath(const Eigen::Vector3d& fallback)
  {
    if (!have_tcp_)
    {
      return fallback;
    }
    if (current_tcp_frame_.empty() || current_tcp_frame_ == path_frame_)
    {
      return current_tcp_;
    }
    try
    {
      const geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(path_frame_, current_tcp_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      return transformPoint(tf, current_tcp_);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "TCP feedback TF failed (%s -> %s): %s",
                        current_tcp_frame_.c_str(), path_frame_.c_str(), ex.what());
      return fallback;
    }
  }

  Eigen::Vector3d targetInBase(const Eigen::Vector3d& target)
  {
    if (!base_enabled_)
    {
      return Eigen::Vector3d::Zero();
    }
    try
    {
      const geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(base_frame_, path_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      last_target_in_base_ = transformPoint(tf, target);
      return last_target_in_base_;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "Target-in-base TF failed (%s -> %s): %s",
                        path_frame_.c_str(), base_frame_.c_str(), ex.what());
      return last_target_in_base_;
    }
  }

  Eigen::Vector3d currentTcpInBase(const Eigen::Vector3d& fallback)
  {
    if (!have_tcp_)
    {
      return fallback;
    }
    if (current_tcp_frame_.empty() || current_tcp_frame_ == base_frame_)
    {
      return current_tcp_;
    }
    try
    {
      const geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(base_frame_, current_tcp_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      return transformPoint(tf, current_tcp_);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "TCP-in-base TF failed (%s -> %s): %s",
                        current_tcp_frame_.c_str(), base_frame_.c_str(), ex.what());
      return fallback;
    }
  }

  bool pathFrameReady() const
  {
    return !origin_enabled_ || origin_captured_;
  }

  void captureOrigin()
  {
    if (!origin_enabled_)
    {
      origin_captured_ = true;
      return;
    }
    try
    {
      origin_transform_ = tf_buffer_.lookupTransform(origin_parent_frame_,
                                                     origin_source_frame_,
                                                     ros::Time(0),
                                                     ros::Duration(origin_capture_timeout_));
      origin_transform_.child_frame_id = origin_frame_id_;
      origin_transform_.header.frame_id = origin_parent_frame_;
      origin_transform_.header.stamp = ros::Time::now();
      origin_captured_ = true;
      ROS_INFO("Captured whole-body path origin '%s' from %s -> %s",
               origin_frame_id_.c_str(),
               origin_parent_frame_.c_str(),
               origin_source_frame_.c_str());
    }
    catch (const tf2::TransformException& ex)
    {
      origin_captured_ = false;
      ROS_WARN("Failed to capture whole-body path origin (%s <- %s): %s",
               origin_parent_frame_.c_str(),
               origin_source_frame_.c_str(),
               ex.what());
    }
  }

  void publishOriginTransform(const ros::Time& stamp)
  {
    if (!origin_enabled_ || !origin_captured_ || !origin_tf_broadcaster_)
    {
      return;
    }
    origin_transform_.header.stamp = stamp;
    origin_tf_broadcaster_->sendTransform(origin_transform_);
  }

  bool lookupPathToBaseTransform(geometry_msgs::TransformStamped& tf) const
  {
    try
    {
      tf = tf_buffer_.lookupTransform(base_frame_, path_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "Path-to-base TF failed (%s -> %s): %s",
                        path_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  Eigen::Quaterniond rotateOrientation(const geometry_msgs::TransformStamped& tf, const Eigen::Quaterniond& q) const
  {
    const Eigen::Quaterniond q_tf(tf.transform.rotation.w,
                                  tf.transform.rotation.x,
                                  tf.transform.rotation.y,
                                  tf.transform.rotation.z);
    return (q_tf.normalized() * q).normalized();
  }

  Eigen::Vector3d computeExternalTcpVelocityInPath(const Eigen::Vector3d& base_reference_point,
                                                   const Eigen::VectorXd& u)
  {
    Eigen::Vector3d external_velocity = Eigen::Vector3d::Zero();
    int c = 0;

    if (base_enabled_ && u.size() >= 2)
    {
      geometry_msgs::TransformStamped tf_path_base;
      try
      {
        tf_path_base = tf_buffer_.lookupTransform(path_frame_, base_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "External TCP velocity TF failed (%s <- %s): %s",
                          path_frame_.c_str(), base_frame_.c_str(), ex.what());
        return external_velocity;
      }

      const Eigen::Vector3d base_pos(tf_path_base.transform.translation.x,
                                     tf_path_base.transform.translation.y,
                                     tf_path_base.transform.translation.z);
      const Eigen::Quaterniond q_path_base(tf_path_base.transform.rotation.w,
                                           tf_path_base.transform.rotation.x,
                                           tf_path_base.transform.rotation.y,
                                           tf_path_base.transform.rotation.z);
      const Eigen::Vector3d base_x = q_path_base.normalized() * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d r = base_reference_point - base_pos;
      const Eigen::Vector3d yaw_col(-r.y(), r.x(), 0.0);
      external_velocity += u(c) * base_x + u(c + 1) * yaw_col;
      c += 2;
    }

    if (lifter_enabled_ && u.size() > c)
    {
      external_velocity.z() += u(c);
    }
    return external_velocity;
  }

  double computeArmTrackingScale(const Eigen::Vector3d& target_in_base,
                                 const Eigen::Vector3d& tcp_in_base)
  {
    if (!base_enabled_)
    {
      base_in_tracking_zone_ = true;
      return 1.0;
    }

    const double target_dx = std::abs(target_in_base.x() - base_preferred_x_);
    const double target_dy = std::abs(target_in_base.y() - base_preferred_y_);
    const double tcp_dx = std::abs(tcp_in_base.x() - base_preferred_x_);
    const double tcp_dy = std::abs(tcp_in_base.y() - base_preferred_y_);
    const double dx = std::max(target_dx, tcp_dx);
    const double dy = std::max(target_dy, tcp_dy);
    const double full_x = std::max(1e-6, arm_full_x_error_);
    const double full_y = std::max(1e-6, arm_full_y_error_);
    const double start_x = std::max(full_x + 1e-6, arm_start_x_error_);
    const double start_y = std::max(full_y + 1e-6, arm_start_y_error_);

    base_in_tracking_zone_ =
        (target_dx <= full_x && target_dy <= full_y && tcp_dx <= full_x && tcp_dy <= full_y);
    if (base_in_tracking_zone_)
    {
      return 1.0;
    }
    if (dx >= start_x || dy >= start_y)
    {
      return clamp(arm_far_scale_, 0.0, 1.0);
    }

    const double rx = std::max(0.0, (dx - full_x) / (start_x - full_x));
    const double ry = std::max(0.0, (dy - full_y) / (start_y - full_y));
    const double blend = clamp(std::max(rx, ry), 0.0, 1.0);
    return clamp(1.0 - blend * (1.0 - arm_far_scale_), 0.0, 1.0);
  }

  bool startReached(const Eigen::Vector3d& start_target)
  {
    if (!preposition_enabled_)
    {
      return true;
    }

    if (preposition_reached_mode_ == "base_target")
    {
      const Eigen::Vector3d target_in_base = targetInBase(start_target);
      const double x_error = target_in_base.x() - base_preferred_x_;
      const double y_error = target_in_base.y() - base_preferred_y_;
      last_start_error_ = std::hypot(x_error, y_error);
      return last_start_error_ <= start_base_target_tolerance_;
    }

    const Eigen::Vector3d tcp = currentTcpInPath(start_target);
    last_start_error_ = (start_target - tcp).norm();
    return last_start_error_ <= start_position_tolerance_;
  }

  void updateBaseTrackingTarget(const Eigen::Vector3d& target, double dt)
  {
    if (!base_tracking_target_initialized_ || base_target_filter_tau_ <= 1e-6 || dt <= 0.0)
    {
      base_tracking_target_ = target;
      base_tracking_target_initialized_ = true;
      return;
    }
    const double alpha = clamp(dt / (base_target_filter_tau_ + dt), 0.0, 1.0);
    base_tracking_target_ += alpha * (target - base_tracking_target_);
  }

  double pathSpeedTarget(double remaining_distance) const
  {
    const double cruise_speed = std::max(0.0, speed_);
    if (path_max_linear_acceleration_ <= 1e-9 || loop_)
    {
      return cruise_speed;
    }

    const double braking_speed =
        std::sqrt(std::max(0.0, 2.0 * path_max_linear_acceleration_ * std::max(0.0, remaining_distance)));
    return std::min(cruise_speed, braking_speed);
  }

  void updatePathSpeed(double target_speed, double dt)
  {
    target_speed = clamp(target_speed, 0.0, std::max(0.0, speed_));
    if (dt <= 0.0 || path_max_linear_acceleration_ <= 1e-9)
    {
      path_speed_current_ = target_speed;
      return;
    }

    const double max_step = path_max_linear_acceleration_ * dt;
    path_speed_current_ += clamp(target_speed - path_speed_current_, -max_step, max_step);
    path_speed_current_ = clamp(path_speed_current_, 0.0, std::max(0.0, speed_));
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

  bool dueByRate(const ros::Time& now, const ros::Time& last, double rate_hz) const
  {
    if (rate_hz <= 0.0)
    {
      return false;
    }
    if (last.isZero())
    {
      return true;
    }
    return (now - last).toSec() >= (1.0 / rate_hz);
  }

  void publishBaseCommand(const Eigen::VectorXd& u, const ros::Time& now)
  {
    if (!base_enabled_ || !base_tf_ok_)
    {
      publishZeroBase();
      return;
    }
    if (!dueByRate(now, last_base_pub_time_, base_rate_hz_))
    {
      return;
    }
    geometry_msgs::Twist cmd;
    cmd.linear.x = clamp(u.size() >= 1 ? u(0) : 0.0, -base_max_linear_, base_max_linear_);
    cmd.angular.z = clamp(u.size() >= 2 ? u(1) : 0.0, -base_max_angular_, base_max_angular_);
    const double dt = last_base_pub_time_.isZero() ? (1.0 / std::max(1.0, base_rate_hz_))
                                                   : std::max(0.0, (now - last_base_pub_time_).toSec());
    last_base_nominal_command_ = limitBaseAcceleration(cmd, last_base_nominal_command_, dt);
    last_base_command_ = applyLaserAvoidance(last_base_nominal_command_, now);
    base_pub_.publish(last_base_command_);
    last_base_pub_time_ = now;
  }

  geometry_msgs::Twist limitBaseAcceleration(const geometry_msgs::Twist& target,
                                             const geometry_msgs::Twist& previous,
                                             double dt) const
  {
    if (dt <= 0.0)
    {
      return previous;
    }

    geometry_msgs::Twist limited = target;
    if (base_max_linear_acceleration_ > 1e-9)
    {
      const double max_step = base_max_linear_acceleration_ * dt;
      limited.linear.x = previous.linear.x +
                         clamp(target.linear.x - previous.linear.x, -max_step, max_step);
    }
    if (base_max_angular_acceleration_ > 1e-9)
    {
      const double max_step = base_max_angular_acceleration_ * dt;
      limited.angular.z = previous.angular.z +
                          clamp(target.angular.z - previous.angular.z, -max_step, max_step);
    }
    limited.linear.x = clamp(limited.linear.x, -base_max_linear_, base_max_linear_);
    limited.angular.z = clamp(limited.angular.z, -base_max_angular_, base_max_angular_);
    return limited;
  }

  geometry_msgs::Twist applyLaserAvoidance(const geometry_msgs::Twist& nominal, const ros::Time& now)
  {
    geometry_msgs::Twist cmd = nominal;
    if (!avoidance_enabled_)
    {
      last_avoidance_omega_ = 0.0;
      last_avoidance_speed_scale_ = 1.0;
      return cmd;
    }

    updateSimulatedObstacleAvoidance(now);
    updateAggregatedAvoidance(now);
    const bool stale = avoidance_last_scan_time_.isZero() ||
                       (now - avoidance_last_scan_time_).toSec() > avoidance_stale_timeout_;
    const double dt = last_base_pub_time_.isZero() ? (1.0 / std::max(1.0, base_rate_hz_))
                                                   : std::max(0.0, (now - last_base_pub_time_).toSec());
    double target_omega = 0.0;
    double speed_scale = 1.0;

    if (!stale && avoidance_active_)
    {
      target_omega = avoidance_raw_omega_;
      speed_scale = avoidance_raw_speed_scale_;
    }
    else if (stale)
    {
      avoidance_active_ = false;
      avoidance_min_distance_ = std::numeric_limits<double>::infinity();
      avoidance_raw_speed_scale_ = 1.0;
    }

    const double alpha = avoidance_filter_tau_ <= 1e-6 ? 1.0 : clamp(dt / (avoidance_filter_tau_ + dt), 0.0, 1.0);
    last_avoidance_omega_ += alpha * (target_omega - last_avoidance_omega_);
    last_avoidance_speed_scale_ += alpha * (speed_scale - last_avoidance_speed_scale_);

    cmd.linear.x *= last_avoidance_speed_scale_;
    cmd.angular.z = clamp(cmd.angular.z + last_avoidance_omega_, -base_max_angular_, base_max_angular_);
    return cmd;
  }

  void updateSimulatedObstacleAvoidance(const ros::Time& now)
  {
    if (!simulated_obstacles_enabled_ || simulated_obstacles_.empty())
    {
      return;
    }

    ScanAvoidanceState state;
    state.stamp = now;

    geometry_msgs::TransformStamped tf_base_path;
    try
    {
      tf_base_path = tf_buffer_.lookupTransform(base_frame_, path_frame_, ros::Time(0), ros::Duration(base_tf_timeout_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "Simulated obstacle avoidance TF failed (%s -> %s): %s",
                        path_frame_.c_str(), base_frame_.c_str(), ex.what());
      scan_avoidance_states_["simulated_obstacles"] = state;
      return;
    }

    double omega_sum = 0.0;
    double weight_sum = 0.0;
    double min_dist = std::numeric_limits<double>::infinity();
    double raw_speed_scale = 1.0;

    for (const auto& obstacle : simulated_obstacles_)
    {
      const Eigen::Vector3d p = transformPoint(tf_base_path, obstacle.center);
      const bool in_stop =
          obstacleInAvoidanceZone(p, obstacle.radius, avoidance_stop_distance_, avoidance_stop_lateral_window_);
      const bool in_slowdown =
          obstacleInAvoidanceZone(p, obstacle.radius, avoidance_slowdown_distance_, avoidance_slowdown_lateral_window_);
      const bool in_influence =
          obstacleInAvoidanceZone(p, obstacle.radius, avoidance_influence_distance_, avoidance_influence_lateral_window_);
      if (!in_stop && !in_slowdown && !in_influence)
      {
        continue;
      }

      const double surface_distance = in_influence
                                          ? avoidanceObstacleSurfaceDistance(
                                                p, obstacle.radius, avoidance_influence_lateral_window_)
                                          : (in_slowdown
                                                 ? avoidanceObstacleSurfaceDistance(
                                                       p, obstacle.radius, avoidance_slowdown_lateral_window_)
                                                 : avoidanceObstacleSurfaceDistance(
                                                       p, obstacle.radius, avoidance_stop_lateral_window_));
      min_dist = std::min(min_dist, surface_distance);
      if (in_slowdown || in_stop)
      {
        raw_speed_scale = std::min(raw_speed_scale, obstacleAvoidanceSpeedScale(p, obstacle.radius, in_stop));
      }

      const double influence = clamp((avoidance_influence_distance_ - surface_distance) /
                                     std::max(1e-6, avoidance_influence_distance_ - avoidance_stop_distance_),
                                     0.0, 1.0);
      const double side = (std::abs(p.y()) > 1e-4) ? (p.y() > 0.0 ? 1.0 : -1.0) : 1.0;
      omega_sum += -side * influence * influence;
      weight_sum += influence;
    }

    if (weight_sum > 1e-6)
    {
      state.active = true;
      state.min_distance = min_dist;
      state.raw_speed_scale = raw_speed_scale;
      state.raw_omega = clamp(avoidance_k_omega_ * omega_sum / weight_sum,
                              -avoidance_max_omega_, avoidance_max_omega_);
    }
    scan_avoidance_states_["simulated_obstacles"] = state;
  }

  void publishLifterCommand(const Eigen::VectorXd& u, double dt, const ros::Time& now)
  {
    if (!lifter_enabled_ || !have_lifter_ || !lifter_target_initialized_)
    {
      return;
    }
    if (!dueByRate(now, last_lifter_pub_time_, lifter_rate_hz_))
    {
      return;
    }
    const int idx = base_enabled_ ? 2 : 0;
    if (u.size() <= idx || dt <= 0.0)
    {
      return;
    }
    lifter_target_ = clamp(lifter_target_ + u(idx) * dt, lifter_min_, lifter_max_);
    std_msgs::Float32 msg;
    msg.data = lifter_target_;
    lifter_pub_.publish(msg);
    last_lifter_pub_time_ = now;
  }

  void publishZeroBase()
  {
    if (base_enabled_)
    {
      last_base_command_ = geometry_msgs::Twist();
      last_base_nominal_command_ = geometry_msgs::Twist();
      last_base_linear_saturated_ = false;
      last_base_angular_saturated_ = false;
      base_pub_.publish(last_base_command_);
    }
  }

  void publishSmoothZeroBase(const ros::Time& now)
  {
    if (!base_enabled_)
    {
      return;
    }
    if (!dueByRate(now, last_base_pub_time_, base_rate_hz_))
    {
      return;
    }

    const double dt = last_base_pub_time_.isZero() ? (1.0 / std::max(1.0, base_rate_hz_))
                                                   : std::max(0.0, (now - last_base_pub_time_).toSec());
    const geometry_msgs::Twist zero;
    last_base_nominal_command_ = limitBaseAcceleration(zero, last_base_nominal_command_, dt);
    last_base_command_ = applyLaserAvoidance(last_base_nominal_command_, now);
    last_base_linear_saturated_ = false;
    last_base_angular_saturated_ = false;
    base_pub_.publish(last_base_command_);
    last_base_pub_time_ = now;
  }

  void publishArmReference(const Eigen::Vector3d& p_path,
                           const Eigen::Vector3d& v_path,
                           const ros::Time& stamp,
                           bool active)
  {
    geometry_msgs::TransformStamped tf_base_path;
    if (!lookupPathToBaseTransform(tf_base_path))
    {
      return;
    }

    const Eigen::Vector3d p_base = transformPoint(tf_base_path, p_path);
    const Eigen::Vector3d v_base = rotateVector(tf_base_path, v_path);
    const Eigen::Quaterniond q_base = rotateOrientation(tf_base_path, target_orientation_);

    if (publish_target_pose_)
    {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = base_frame_;
      msg.pose.position.x = p_base.x();
      msg.pose.position.y = p_base.y();
      msg.pose.position.z = p_base.z();
      msg.pose.orientation = toMsg(q_base);
      target_pub_.publish(msg);
    }

    if (publish_target_state_)
    {
      cartesian_velocity_controller::CartesianTrajectorySetpoint msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = base_frame_;
      msg.pose.position.x = p_base.x();
      msg.pose.position.y = p_base.y();
      msg.pose.position.z = p_base.z();
      msg.pose.orientation = toMsg(q_base);
      msg.velocity.linear = vectorToMsg(active ? v_base : Eigen::Vector3d::Zero());
      msg.velocity.angular = geometry_msgs::Vector3();
      msg.acceleration.linear = geometry_msgs::Vector3();
      msg.acceleration.angular = geometry_msgs::Vector3();
      msg.jerk.linear = geometry_msgs::Vector3();
      msg.jerk.angular = geometry_msgs::Vector3();
      msg.path_s = s_;
      msg.path_progress = external_path_progress_;
      msg.active = active;
      arm_target_state_pub_.publish(msg);
    }
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

  void publishPathMarkerIfDue(const ros::Time& now)
  {
    if (external_target_enabled_ || path_marker_rate_hz_ <= 0.0)
    {
      return;
    }
    if (!dueByRate(now, last_path_marker_pub_time_, path_marker_rate_hz_))
    {
      return;
    }
    publishPathMarker();
    publishSimulatedObstacleMarkers(now);
    last_path_marker_pub_time_ = now;
  }

  void publishSimulatedObstacleMarkers(const ros::Time& stamp)
  {
    if (!simulated_obstacles_enabled_ || simulated_obstacles_.empty())
    {
      return;
    }

    visualization_msgs::MarkerArray arr;
    int id = 0;
    for (const auto& obstacle : simulated_obstacles_)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = path_frame_;
      marker.header.stamp = stamp;
      marker.ns = "simulated_base_obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = obstacle.center.x();
      marker.pose.position.y = obstacle.center.y();
      marker.pose.position.z = obstacle.center.z() + 0.5 * simulated_obstacle_marker_height_;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * obstacle.radius;
      marker.scale.y = 2.0 * obstacle.radius;
      marker.scale.z = simulated_obstacle_marker_height_;
      marker.color.r = 1.0;
      marker.color.g = 0.18;
      marker.color.b = 0.05;
      marker.color.a = 0.45;
      arr.markers.push_back(marker);

      visualization_msgs::Marker label;
      label.header.frame_id = path_frame_;
      label.header.stamp = stamp;
      label.ns = "simulated_base_obstacle_labels";
      label.id = id++;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = obstacle.center.x();
      label.pose.position.y = obstacle.center.y();
      label.pose.position.z = obstacle.center.z() + simulated_obstacle_marker_height_ + 0.15;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.16;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 0.9;
      label.text = obstacle.name;
      arr.markers.push_back(label);
    }

    simulated_obstacle_marker_pub_.publish(arr);
  }

  geometry_msgs::Point reactionPerimeterPoint(double x, double y, double z) const
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  void appendReactionPerimeterPoints(visualization_msgs::Marker& marker,
                                     double radius,
                                     double lateral_window,
                                     double z) const
  {
    if (radius <= 1e-6 || lateral_window <= 1e-6)
    {
      return;
    }

    const double x_min = avoidance_front_min_x_;
    if (x_min >= radius)
    {
      return;
    }

    if (avoidanceUsesLongitudinalDistance())
    {
      const double y_limit = lateral_window;
      marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(radius, -y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(radius, y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(x_min, y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
      return;
    }

    if (avoidanceUsesRoundedFrontDistance())
    {
      const double y_limit = lateral_window;
      const double arc_radius = std::hypot(radius, y_limit);
      const double theta = std::asin(clamp(y_limit / arc_radius, 0.0, 1.0));
      marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(radius, -y_limit, z));

      const int segments = std::max(12, static_cast<int>(std::ceil(2.0 * theta / 0.05)));
      for (int i = 0; i <= segments; ++i)
      {
        const double a = -theta + (2.0 * theta * static_cast<double>(i)) / static_cast<double>(segments);
        marker.points.push_back(reactionPerimeterPoint(arc_radius * std::cos(a), arc_radius * std::sin(a), z));
      }

      marker.points.push_back(reactionPerimeterPoint(radius, y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(x_min, y_limit, z));
      marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
      return;
    }

    const double theta_front = std::acos(clamp(x_min / radius, -1.0, 1.0));
    const double theta_side = lateral_window >= radius
                                  ? 0.5 * std::acos(-1.0)
                                  : std::asin(clamp(lateral_window / radius, 0.0, 1.0));
    const double theta = std::min(theta_front, theta_side);
    if (theta <= 1e-6)
    {
      return;
    }

    const double y_limit = radius * std::sin(theta);
    const double x_arc = radius * std::cos(theta);
    const bool lateral_clipped = x_arc > x_min + 1e-6;
    marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
    if (lateral_clipped)
    {
      marker.points.push_back(reactionPerimeterPoint(x_arc, -y_limit, z));
    }

    const int segments = std::max(12, static_cast<int>(std::ceil(2.0 * theta / 0.05)));
    for (int i = 0; i <= segments; ++i)
    {
      const double a = -theta + (2.0 * theta * static_cast<double>(i)) / static_cast<double>(segments);
      marker.points.push_back(reactionPerimeterPoint(radius * std::cos(a), radius * std::sin(a), z));
    }

    if (lateral_clipped)
    {
      marker.points.push_back(reactionPerimeterPoint(x_min, y_limit, z));
    }
    marker.points.push_back(reactionPerimeterPoint(x_min, -y_limit, z));
  }

  visualization_msgs::Marker reactionPerimeterMarker(const ros::Time& stamp,
                                                     int id,
                                                     const std::string& ns,
                                                     double radius,
                                                     double lateral_window,
                                                     double z,
                                                     double line_width,
                                                     double r,
                                                     double g,
                                                     double b,
                                                     double a) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.frame_locked = true;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_width;
    marker.color.r = static_cast<float>(r);
    marker.color.g = static_cast<float>(g);
    marker.color.b = static_cast<float>(b);
    marker.color.a = static_cast<float>(a);
    appendReactionPerimeterPoints(marker, radius, lateral_window, z);
    if (marker.points.size() < 2)
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    return marker;
  }

  void publishReactionPerimeterMarkers(const ros::Time& stamp)
  {
    visualization_msgs::MarkerArray arr;
    if (!avoidance_enabled_)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      arr.markers.push_back(marker);
      reaction_perimeter_marker_pub_.publish(arr);
      return;
    }

    arr.markers.push_back(reactionPerimeterMarker(stamp,
                                                  0,
                                                  "reaction_perimeter_influence",
                                                  avoidance_influence_distance_,
                                                  avoidance_influence_lateral_window_,
                                                  0.04,
                                                  0.025,
                                                  0.1,
                                                  0.75,
                                                  1.0,
                                                  0.95));
    arr.markers.push_back(reactionPerimeterMarker(stamp,
                                                  1,
                                                  "reaction_perimeter_slowdown",
                                                  avoidance_slowdown_distance_,
                                                  avoidance_slowdown_lateral_window_,
                                                  0.06,
                                                  0.02,
                                                  1.0,
                                                  0.85,
                                                  0.1,
                                                  0.9));
    arr.markers.push_back(reactionPerimeterMarker(stamp,
                                                  2,
                                                  "reaction_perimeter_stop",
                                                  avoidance_stop_distance_,
                                                  avoidance_stop_lateral_window_,
                                                  0.08,
                                                  0.02,
                                                  1.0,
                                                  0.15,
                                                  0.05,
                                                  0.95));
    reaction_perimeter_marker_pub_.publish(arr);
  }

  void publishReactionPerimeterMarkersIfDue(const ros::Time& now)
  {
    if (!dueByRate(now, last_reaction_perimeter_marker_pub_time_, avoidance_perimeter_marker_rate_hz_))
    {
      return;
    }
    publishReactionPerimeterMarkers(now);
    last_reaction_perimeter_marker_pub_time_ = now;
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

  void publishPreferredTcpMarker()
  {
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker target;
    target.header.frame_id = base_frame_;
    target.header.stamp = stamp;
    target.ns = "preferred_tcp";
    target.id = 0;
    target.type = visualization_msgs::Marker::SPHERE;
    target.action = visualization_msgs::Marker::ADD;
    target.frame_locked = true;
    target.pose.position.x = base_preferred_x_;
    target.pose.position.y = base_preferred_y_;
    target.pose.position.z = 0.0;
    target.pose.orientation.w = 1.0;
    target.scale.x = 0.10;
    target.scale.y = 0.10;
    target.scale.z = 0.10;
    target.color.r = 0.0;
    target.color.g = 0.85;
    target.color.b = 0.25;
    target.color.a = 0.95;
    preferred_tcp_marker_pub_.publish(target);

    visualization_msgs::Marker link;
    link.header.frame_id = base_frame_;
    link.header.stamp = stamp;
    link.ns = "preferred_tcp";
    link.id = 1;
    link.type = visualization_msgs::Marker::LINE_STRIP;
    link.action = visualization_msgs::Marker::ADD;
    link.frame_locked = true;
    link.pose.orientation.w = 1.0;
    link.scale.x = 0.02;
    link.color.r = 0.0;
    link.color.g = 0.85;
    link.color.b = 0.25;
    link.color.a = 0.7;
    geometry_msgs::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    geometry_msgs::Point preferred;
    preferred.x = base_preferred_x_;
    preferred.y = base_preferred_y_;
    preferred.z = 0.0;
    link.points.push_back(origin);
    link.points.push_back(preferred);
    preferred_tcp_marker_pub_.publish(link);
  }

  std::string stateString() const
  {
    if (!pathFrameReady()) return "waiting_origin";
    if (stopped_) return "stopped";
    if (!has_started_) return "idle";
    if (paused_) return "paused";
    if (resume_delay_active_) return "resume_delay";
    if (!preposition_done_) return dwell_started_ ? "dwell" : "preposition";
    if (done_) return "done";
    return "tracking";
  }

  void publishDebug(const ros::Time& stamp,
                    const std::string& state,
                    const Eigen::Vector3d& target,
                    const Eigen::Vector3d& arm_target,
                    const Eigen::Vector3d& tcp,
                    const Eigen::Vector3d& target_in_base,
                    double arm_scale)
  {
    if (debug_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    if (!dueByRate(stamp, last_debug_pub_time_, debug_rate_hz_))
    {
      return;
    }

    cartesian_velocity_controller::WholeBodyPrintDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = path_frame_;
    msg.state = state;
    msg.path_frame = path_frame_;
    msg.base_frame = base_frame_;
    msg.path_s = s_;
    const double path_length = external_target_enabled_ ? external_path_length_ : path_.totalLength();
    msg.path_length = path_length;
    msg.path_progress = external_target_enabled_
                            ? clamp(external_path_progress_, 0.0, 1.0)
                            : (path_length > 1e-9 ? clamp(s_ / path_length, 0.0, 1.0) : 0.0);
    msg.target_position = pointToMsg(target);
    msg.arm_target_position = pointToMsg(arm_target);
    msg.current_tcp_position = pointToMsg(tcp);
    msg.target_in_base = pointToMsg(target_in_base);
    msg.tcp_error = vectorToMsg(target - tcp);
    msg.arm_tracking_scale = arm_scale;
    msg.base_enabled = base_enabled_;
    msg.base_tf_ok = base_tf_ok_;
    msg.base_in_tracking_zone = base_in_tracking_zone_;
    msg.base_linear_saturated = last_base_linear_saturated_;
    msg.base_angular_saturated = last_base_angular_saturated_;
    msg.base_command = last_base_command_;
    msg.base_nominal_command = last_base_nominal_command_;
    msg.lifter_enabled = lifter_enabled_;
    msg.lifter_have_state = have_lifter_;
    msg.lifter_position = current_lifter_;
    msg.lifter_target = lifter_target_;
    msg.lifter_velocity_command = last_lifter_velocity_command_;
    msg.preferred_tcp_x = base_preferred_x_;
    msg.preferred_tcp_y = base_preferred_y_;
    msg.avoidance_enabled = avoidance_enabled_;
    msg.avoidance_active = avoidance_active_;
    msg.avoidance_min_distance = std::isfinite(avoidance_min_distance_) ? avoidance_min_distance_ : -1.0;
    msg.avoidance_omega = last_avoidance_omega_;
    msg.avoidance_speed_scale = last_avoidance_speed_scale_;
    debug_pub_.publish(msg);
    last_debug_pub_time_ = stamp;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> origin_tf_broadcaster_;

  ros::Publisher target_pub_;
  ros::Publisher arm_target_state_pub_;
  ros::Publisher base_pub_;
  ros::Publisher lifter_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher current_marker_pub_;
  ros::Publisher preferred_tcp_marker_pub_;
  ros::Publisher simulated_obstacle_marker_pub_;
  ros::Publisher reaction_perimeter_marker_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber ee_sub_;
  ros::Subscriber joint_state_sub_;
  std::vector<ros::Subscriber> scan_subs_;
  ros::Subscriber target_state_sub_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer restart_srv_;
  ros::ServiceServer stop_srv_;
  ros::Timer timer_;

  PolylinePath path_;
  XmlRpc::XmlRpcValue configured_path_points_raw_;
  std::vector<Eigen::Vector3d> configured_path_points_;
  std::string path_frame_;
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};
  double speed_{0.03};
  double path_speed_current_{0.0};
  double path_max_linear_velocity_{0.0};
  double path_max_linear_acceleration_{0.0};
  double path_rate_hz_{30.0};
  double base_rate_hz_{20.0};
  double lifter_rate_hz_{10.0};
  double debug_rate_hz_{10.0};
  double path_marker_rate_hz_{2.0};
  double s_{0.0};
  bool loop_{false};
  bool paused_{true};
  bool stopped_{false};
  bool done_{false};
  bool has_started_{false};
  bool preposition_enabled_{true};
  bool preposition_done_{false};
  bool dwell_started_{false};
  std::string preposition_reached_mode_{"base_target"};
  double start_position_tolerance_{0.01};
  double start_base_target_tolerance_{0.15};
  double preposition_dwell_s_{1.0};
  double last_start_error_{std::numeric_limits<double>::infinity()};
  ros::Time dwell_start_time_;
  double resume_delay_s_{0.0};
  bool resume_delay_active_{false};
  ros::Time resume_delay_until_;
  bool origin_enabled_{false};
  bool origin_captured_{false};
  std::string origin_capture_mode_{"node_start"};
  std::string origin_parent_frame_;
  std::string origin_source_frame_;
  std::string origin_frame_id_;
  double origin_capture_timeout_{2.0};
  geometry_msgs::TransformStamped origin_transform_;
  ros::Time last_time_;

  std::string target_pose_topic_;
  bool publish_target_pose_{true};
  bool publish_target_state_{false};
  std::string arm_target_state_topic_;
  bool external_target_enabled_{false};
  std::string target_state_topic_;
  double target_state_timeout_{0.5};
  bool have_external_target_{false};
  bool external_active_{false};
  ros::Time external_stamp_;
  Eigen::Vector3d external_target_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_velocity_{Eigen::Vector3d::Zero()};
  double external_path_s_{0.0};
  double external_path_progress_{0.0};
  double external_path_length_{0.0};
  std::string ee_state_topic_;
  double kp_position_{0.8};
  double arm_full_x_error_{0.15};
  double arm_full_y_error_{0.12};
  double arm_start_x_error_{0.45};
  double arm_start_y_error_{0.35};
  double arm_far_scale_{0.0};
  bool arm_gate_only_preposition_{false};
  double max_tcp_error_for_progress_{0.0};
  double max_base_x_error_for_progress_{0.0};
  double max_base_y_error_for_progress_{0.0};
  Eigen::Vector3d current_tcp_{Eigen::Vector3d::Zero()};
  std::string current_tcp_frame_;
  bool have_tcp_{false};

  bool base_enabled_{true};
  std::string base_cmd_vel_topic_;
  std::string base_frame_;
  double base_preferred_x_{0.65};
  double base_preferred_y_{0.0};
  double base_max_linear_{0.08};
  double base_max_angular_{0.25};
  double base_max_linear_acceleration_{0.10};
  double base_max_angular_acceleration_{0.30};
  double base_weight_linear_{4.0};
  double base_weight_angular_{2.0};
  double base_k_preferred_x_{0.25};
  double base_k_lateral_{0.6};
  double base_k_heading_{0.25};
  double base_tf_timeout_{0.05};
  double base_target_filter_tau_{1.0};
  bool base_allow_reverse_{false};
  bool base_align_to_path_{true};
  bool base_tf_ok_{true};
  bool base_in_tracking_zone_{false};
  bool base_tracking_target_initialized_{false};
  bool last_base_linear_saturated_{false};
  bool last_base_angular_saturated_{false};
  Eigen::Vector3d base_tracking_target_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_target_in_base_{Eigen::Vector3d::Zero()};
  geometry_msgs::Twist last_base_command_;
  geometry_msgs::Twist last_base_nominal_command_;
  ros::Time last_base_pub_time_;
  ros::Time last_lifter_pub_time_;
  ros::Time last_debug_pub_time_;
  ros::Time last_path_marker_pub_time_;
  ros::Time last_reaction_perimeter_marker_pub_time_;

  bool avoidance_enabled_{false};
  bool avoidance_use_laser_scans_{true};
  std::string avoidance_scan_topic_;
  std::string avoidance_distance_mode_{"radial"};
  std::vector<std::string> avoidance_scan_topics_;
  std::map<std::string, ScanAvoidanceState> scan_avoidance_states_;
  double avoidance_influence_distance_{1.2};
  double avoidance_stop_distance_{0.35};
  double avoidance_slowdown_distance_{0.8};
  double avoidance_lateral_window_{0.9};
  double avoidance_influence_lateral_window_{0.9};
  double avoidance_slowdown_lateral_window_{0.9};
  double avoidance_stop_lateral_window_{0.9};
  double avoidance_front_min_x_{0.05};
  double avoidance_k_omega_{0.8};
  double avoidance_max_omega_{0.35};
  double avoidance_filter_tau_{0.4};
  double avoidance_stale_timeout_{0.5};
  double avoidance_perimeter_marker_rate_hz_{2.0};
  bool avoidance_active_{false};
  double avoidance_min_distance_{std::numeric_limits<double>::infinity()};
  double avoidance_raw_omega_{0.0};
  double avoidance_raw_speed_scale_{1.0};
  double last_avoidance_omega_{0.0};
  double last_avoidance_speed_scale_{1.0};
  ros::Time avoidance_last_scan_time_;
  bool simulated_obstacles_enabled_{false};
  double simulated_obstacle_marker_height_{0.08};
  std::vector<SimulatedObstacle> simulated_obstacles_;

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
  double last_lifter_velocity_command_{0.0};
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
