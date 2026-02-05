/**
 * @file distance_monitor_node.cpp
 * @brief Implementation of the distance monitor ROS node
 */

#include "scene_builder/nodes/distance_monitor_node.hpp"
#include "scene_builder/objects/object_manager.hpp"
#include "scene_builder/RobotPointDistanceInfo.h"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <stdexcept>

namespace scene_builder
{
namespace nodes
{

DistanceMonitorNode::DistanceMonitorNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
{
  loadParameters();
  setupPublishers();
  initializePlanningScene();
  waitForStateMonitor();

  // Initialize distance calculator with shared planning scene monitor
  distance_calculator_ = std::make_unique<distance::DistanceCalculator>(object_loader_move_group_);
  distance_calculator_->setPlanningSceneMonitor(planning_scene_monitor_);

  // Configure visualizer from parameters
  distance::VisualizationConfig viz_config;
  pnh_.param<double>("arrow_length", viz_config.arrow_length, viz_config.arrow_length);
  pnh_.param<double>("arrow_shaft_diameter", viz_config.arrow_shaft_diameter, viz_config.arrow_shaft_diameter);
  pnh_.param<double>("arrow_head_diameter", viz_config.arrow_head_diameter, viz_config.arrow_head_diameter);
  pnh_.param<double>("arrow_head_length", viz_config.arrow_head_length, viz_config.arrow_head_length);
  distance_visualizer_.setConfig(viz_config);

  // Configure robot point tracker from parameters
  distance::RobotPointTrackerConfig point_tracker_config;
  pnh_.param<double>("robot_point_velocity_filter_alpha", point_tracker_config.velocity_filter_alpha,
                     point_tracker_config.velocity_filter_alpha);
  robot_point_tracker_.setConfig(point_tracker_config);

  // Load robot points of interest
  if (!robot_points_param_.empty())
  {
    if (robot_point_tracker_.loadFromParameter(pnh_, robot_points_param_))
    {
      ROS_INFO_STREAM("Loaded robot points of interest from parameter '" << robot_points_param_ << "'");
    }
    else
    {
      ROS_DEBUG_STREAM("No robot points of interest found in parameter '" << robot_points_param_ << "'");
    }
  }

  loadInitialObjects();
}

void DistanceMonitorNode::loadParameters()
{
  pnh_.param<std::string>("joint_state_topic", joint_state_topic_, std::string("/joint_states"));
  pnh_.param<std::string>("monitor_planning_scene_topic", monitored_scene_topic_, std::string("planning_scene"));
  pnh_.param<std::string>("default_objects_param", default_objects_param_, default_objects_param_);
  pnh_.param<std::string>("object_loader_move_group", object_loader_move_group_, object_loader_move_group_);
  pnh_.param<std::string>("robot_points_param", robot_points_param_, robot_points_param_);
  pnh_.param<double>("state_update_frequency", state_update_frequency_, state_update_frequency_);
  pnh_.param<double>("state_wait_timeout", state_wait_timeout_, state_wait_timeout_);
  pnh_.param<double>("computation_rate", computation_rate_, computation_rate_);
}

void DistanceMonitorNode::setupPublishers()
{
  planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("nearest_points_markers", 1, true);
  distance_info_pub_ = nh_.advertise<scene_builder::DistanceInfo>("distance_info", 10, false);
  robot_points_info_pub_ = nh_.advertise<scene_builder::RobotPointsInfo>("robot_points_info", 10, false);
  apply_planning_scene_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
}

void DistanceMonitorNode::initializePlanningScene()
{
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor_->getPlanningScene())
  {
    throw std::runtime_error("Failed to create PlanningSceneMonitor.");
  }

  if (!monitored_scene_topic_.empty())
  {
    planning_scene_monitor_->startSceneMonitor(monitored_scene_topic_);
  }
  else
  {
    planning_scene_monitor_->startSceneMonitor();
  }
  planning_scene_monitor_->startWorldGeometryMonitor();

  if (!joint_state_topic_.empty())
  {
    planning_scene_monitor_->startStateMonitor(joint_state_topic_);
  }
  else
  {
    planning_scene_monitor_->startStateMonitor();
  }

  if (state_update_frequency_ > 0.0)
  {
    planning_scene_monitor_->setStateUpdateFrequency(state_update_frequency_);
  }

  // Expose apply_planning_scene service
  apply_planning_scene_server_ = nh_.advertiseService("apply_planning_scene",
                                                      &DistanceMonitorNode::handleApplyPlanningScene,
                                                      this);
}

void DistanceMonitorNode::waitForStateMonitor()
{
  auto state_monitor = planning_scene_monitor_->getStateMonitor();
  if (state_monitor)
  {
    if (!state_monitor->waitForCurrentState(ros::Time::now(), state_wait_timeout_))
    {
      ROS_WARN_STREAM("Timeout waiting for current robot state (" << state_wait_timeout_ << " s).");
    }
  }
  else
  {
    ROS_WARN_STREAM("StateMonitor not available. Proceeding with latest scene data.");
  }
}

void DistanceMonitorNode::loadInitialObjects()
{
  if (default_objects_param_.empty())
  {
    ROS_DEBUG_NAMED("scene_builder", "default_objects_param is empty; no objects to load.");
    return;
  }

  ros::NodeHandle loader_nh(nh_);
  ObjectManager object_manager(loader_nh, object_loader_move_group_);
  std::vector<moveit_msgs::CollisionObject> loaded_objects;
  object_manager.loadObjectsFromParameter(pnh_, default_objects_param_, &loaded_objects);

  if (loaded_objects.empty())
  {
    ROS_DEBUG_NAMED("scene_builder", "No objects loaded from parameter %s", default_objects_param_.c_str());
    return;
  }

  if (!apply_planning_scene_client_.exists())
  {
    ROS_INFO_STREAM("Waiting for /apply_planning_scene service...");
    apply_planning_scene_client_.waitForExistence();
    ROS_INFO_STREAM("/apply_planning_scene service available.");
  }

  moveit_msgs::PlanningScene planning_scene_msg;
  {
    planning_scene_monitor::LockedPlanningSceneRW scene_rw(planning_scene_monitor_);
    if (!scene_rw)
    {
      ROS_WARN_STREAM("Cannot get write lock on planning scene for initial objects sync.");
      return;
    }

    planning_scene::PlanningScenePtr scene = scene_rw;
    if (!scene)
    {
      ROS_WARN_STREAM("Planning scene not available during initial objects sync.");
      return;
    }

    for (const auto& obj : loaded_objects)
    {
      scene->processCollisionObjectMsg(obj);
    }

    scene->getPlanningSceneMsg(planning_scene_msg);
  }

  planning_scene_msg.is_diff = true;

  if (apply_planning_scene_client_.exists())
  {
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene_msg;
    if (!apply_planning_scene_client_.call(srv))
    {
      ROS_WARN_STREAM("Failed to call /apply_planning_scene service; publishing to topic instead.");
    }
  }

  planning_scene_pub_.publish(planning_scene_msg);
}

void DistanceMonitorNode::start()
{
  const double period = computation_rate_ > 0.0 ? 1.0 / computation_rate_ : 0.1;
  timer_ = nh_.createTimer(ros::Duration(period), &DistanceMonitorNode::timerCallback, this);
}

bool DistanceMonitorNode::handleApplyPlanningScene(moveit_msgs::ApplyPlanningScene::Request& req,
                                                   moveit_msgs::ApplyPlanningScene::Response& res)
{
  if (!planning_scene_monitor_)
  {
    ROS_WARN_STREAM("ApplyPlanningScene requested but PlanningSceneMonitor not initialized.");
    res.success = false;
    return true;
  }

  if (!planning_scene_monitor_->newPlanningSceneMessage(req.scene))
  {
    ROS_WARN_STREAM("Failed to apply planning scene received via service.");
    res.success = false;
    return true;
  }

  planning_scene_pub_.publish(req.scene);

  res.success = true;
  return true;
}

void DistanceMonitorNode::timerCallback(const ros::TimerEvent& event)
{
  // Update velocity estimates for all objects
  velocity_estimator_.update(planning_scene_monitor_, event.current_real);

  // Update robot point tracker
  robot_point_tracker_.update(planning_scene_monitor_, event.current_real);

  // Compute all distances
  auto distances = distance_calculator_->computeAllDistances();

  // Prepare DistanceInfo message
  scene_builder::DistanceInfo info_msg;
  info_msg.header.stamp = event.current_real;
  info_msg.header.frame_id = distance_calculator_->getPlanningFrame();

  for (const auto& dist : distances)
  {
    scene_builder::DistanceContact contact;
    contact.link_name = dist.link_name;
    contact.object_id = dist.object_id;
    contact.distance = dist.distance;
    contact.robot_point.x = dist.robot_point.x();
    contact.robot_point.y = dist.robot_point.y();
    contact.robot_point.z = dist.robot_point.z();
    contact.object_point.x = dist.object_point.x();
    contact.object_point.y = dist.object_point.y();
    contact.object_point.z = dist.object_point.z();
    contact.distance_vector.x = dist.distance_vector.x();
    contact.distance_vector.y = dist.distance_vector.y();
    contact.distance_vector.z = dist.distance_vector.z();

    // Object pose in world frame
    contact.object_pose.position.x = dist.object_pose.translation().x();
    contact.object_pose.position.y = dist.object_pose.translation().y();
    contact.object_pose.position.z = dist.object_pose.translation().z();
    Eigen::Quaterniond q(dist.object_pose.rotation());
    contact.object_pose.orientation.x = q.x();
    contact.object_pose.orientation.y = q.y();
    contact.object_pose.orientation.z = q.z();
    contact.object_pose.orientation.w = q.w();

    // Object velocity from estimator
    auto velocity = velocity_estimator_.getVelocity(dist.object_id);
    if (velocity.valid)
    {
      contact.object_velocity.x = velocity.linear.x();
      contact.object_velocity.y = velocity.linear.y();
      contact.object_velocity.z = velocity.linear.z();
    }
    else
    {
      contact.object_velocity.x = 0.0;
      contact.object_velocity.y = 0.0;
      contact.object_velocity.z = 0.0;
    }

    info_msg.contacts.push_back(contact);
  }

  distance_info_pub_.publish(info_msg);

  // Compute and publish robot points info
  if (robot_points_info_pub_.getNumSubscribers() > 0)
  {
    auto point_distances = robot_point_tracker_.computeDistancesToObjects(planning_scene_monitor_);

    scene_builder::RobotPointsInfo points_msg;
    points_msg.header.stamp = event.current_real;
    points_msg.header.frame_id = distance_calculator_->getPlanningFrame();

    for (const auto& pd : point_distances)
    {
      scene_builder::RobotPointDistanceInfo point_info;
      point_info.point_name = pd.point_name;
      point_info.link_name = pd.link_name;
      point_info.position.x = pd.point_position.x();
      point_info.position.y = pd.point_position.y();
      point_info.position.z = pd.point_position.z();
      point_info.linear_velocity.x = pd.point_velocity.x();
      point_info.linear_velocity.y = pd.point_velocity.y();
      point_info.linear_velocity.z = pd.point_velocity.z();
      point_info.object_id = pd.object_id;
      point_info.distance_vector.x = pd.distance_vector.x();
      point_info.distance_vector.y = pd.distance_vector.y();
      point_info.distance_vector.z = pd.distance_vector.z();
      point_info.distance = pd.distance;
      point_info.object_characteristic_radius = pd.object_characteristic_radius;

      // Object velocity from estimator
      auto obj_velocity = velocity_estimator_.getVelocity(pd.object_id);
      if (obj_velocity.valid)
      {
        point_info.object_velocity.x = obj_velocity.linear.x();
        point_info.object_velocity.y = obj_velocity.linear.y();
        point_info.object_velocity.z = obj_velocity.linear.z();
      }
      else
      {
        point_info.object_velocity.x = 0.0;
        point_info.object_velocity.y = 0.0;
        point_info.object_velocity.z = 0.0;
      }

      points_msg.points.push_back(point_info);
    }

    robot_points_info_pub_.publish(points_msg);
  }

  // Create and publish visualization markers
  auto markers = distance_visualizer_.createMarkers(
      distances,
      info_msg.header.frame_id,
      ros::Time::now());

  publishMarkers(markers);
}

void DistanceMonitorNode::publishMarkers(const visualization_msgs::MarkerArray& markers)
{
  if (marker_pub_.getNumSubscribers() > 0)
  {
    marker_pub_.publish(markers);
  }
}

}  // namespace nodes
}  // namespace scene_builder

