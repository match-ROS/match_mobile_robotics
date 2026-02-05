/**
 * @file distance_monitor_node.hpp
 * @brief ROS node for real-time robot-obstacle distance monitoring
 *
 * This node computes distances between robot links and obstacles in real-time,
 * publishing distance information and visualization markers.
 */

#ifndef SCENE_BUILDER_NODES_DISTANCE_MONITOR_NODE_HPP
#define SCENE_BUILDER_NODES_DISTANCE_MONITOR_NODE_HPP

#include "scene_builder/distance/distance_calculator.hpp"
#include "scene_builder/distance/distance_visualizer.hpp"
#include "scene_builder/distance/velocity_estimator.hpp"
#include "scene_builder/distance/robot_point_tracker.hpp"
#include "scene_builder/DistanceInfo.h"
#include "scene_builder/RobotPointsInfo.h"

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include <ros/timer.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <string>
#include <vector>

namespace scene_builder
{
namespace nodes
{

/**
 * @brief ROS node for real-time robot-obstacle distance monitoring
 *
 * DistanceMonitorNode provides:
 * - Real-time distance computation between robot and obstacles
 * - RViz visualization of nearest points
 * - Distance information publishing
 *
 * PUBLISHED TOPICS:
 * - planning_scene (moveit_msgs/PlanningScene): Scene updates
 * - nearest_points_markers (visualization_msgs/MarkerArray): RViz markers
 * - distance_info (DistanceInfo): Distance information
 * - robot_points_info (RobotPointsInfo): Robot points of interest information
 *
 * ROS PARAMETERS:
 * - ~joint_state_topic (string): Joint states topic (default: "/joint_states")
 * - ~monitor_planning_scene_topic (string): Planning scene topic
 * - ~state_update_frequency (double): State update frequency (Hz)
 * - ~computation_rate (double): Distance computation rate (Hz, default: 15.0)
 * - ~default_objects_param (string): YAML parameter with initial objects
 * - ~object_loader_move_group (string): MoveGroup for loading objects
 * - ~arrow_* (double): Arrow visualization parameters
 */
class DistanceMonitorNode
{
public:
  /**
   * @brief Constructor - Initializes the node and loads parameters
   * @param nh Main NodeHandle
   * @param pnh Private NodeHandle for parameters
   */
  DistanceMonitorNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /**
   * @brief Starts the distance computation timer
   */
  void start();

private:
  /**
   * @brief Loads ROS parameters
   */
  void loadParameters();

  /**
   * @brief Initializes the PlanningSceneMonitor
   */
  void initializePlanningScene();

  /**
   * @brief Waits for the state monitor to receive robot state
   */
  void waitForStateMonitor();

  /**
   * @brief Loads initial objects from configuration
   */
  void loadInitialObjects();

  /**
   * @brief Sets up publishers
   */
  void setupPublishers();

  /**
   * @brief Handles ApplyPlanningScene service requests
   */
  bool handleApplyPlanningScene(moveit_msgs::ApplyPlanningScene::Request& req,
                                moveit_msgs::ApplyPlanningScene::Response& res);

  /**
   * @brief Timer callback - Computes distances and publishes results
   */
  void timerCallback(const ros::TimerEvent& event);

  /**
   * @brief Publishes markers if there are subscribers
   */
  void publishMarkers(const visualization_msgs::MarkerArray& markers);

  // ROS interfaces
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Publishers
  ros::Publisher planning_scene_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher distance_info_pub_;
  ros::Publisher robot_points_info_pub_;

  // Services
  ros::ServiceClient apply_planning_scene_client_;
  ros::ServiceServer apply_planning_scene_server_;

  // Timer
  ros::Timer timer_;

  // MoveIt interfaces
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Distance components
  std::unique_ptr<distance::DistanceCalculator> distance_calculator_;
  distance::DistanceVisualizer distance_visualizer_;
  distance::VelocityEstimator velocity_estimator_;
  distance::RobotPointTracker robot_point_tracker_;

  // Parameters
  std::string joint_state_topic_;
  std::string monitored_scene_topic_;
  std::string default_objects_param_ = "default_objects";
  std::string object_loader_move_group_ = "manipulator";
  std::string robot_points_param_ = "robot_points_of_interest";
  double state_update_frequency_ = 100.0;
  double state_wait_timeout_ = 2.0;
  double computation_rate_ = 15.0;
};

}  // namespace nodes
}  // namespace scene_builder

#endif  // SCENE_BUILDER_NODES_DISTANCE_MONITOR_NODE_HPP

