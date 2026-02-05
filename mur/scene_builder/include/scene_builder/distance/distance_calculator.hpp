/**
 * @file distance_calculator.hpp
 * @brief Distance calculation between robot links and obstacles
 *
 * This file provides the DistanceCalculator class that computes distances
 * between robot links and collision objects in the planning scene.
 */

#ifndef SCENE_BUILDER_DISTANCE_DISTANCE_CALCULATOR_HPP
#define SCENE_BUILDER_DISTANCE_DISTANCE_CALCULATOR_HPP

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>

#include <Eigen/Dense>

#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace scene_builder
{
namespace distance
{

/**
 * @brief Result of a distance computation between robot and obstacle
 */
struct DistanceResult
{
  double distance = std::numeric_limits<double>::infinity();  ///< Distance value (m)
  std::string link_name;                                      ///< Name of the robot link
  std::string object_id;                                      ///< ID of the obstacle
  Eigen::Vector3d robot_point = Eigen::Vector3d::Zero();      ///< Nearest point on robot (world frame)
  Eigen::Vector3d object_point = Eigen::Vector3d::Zero();     ///< Nearest point on obstacle (world frame)
  Eigen::Vector3d distance_vector = Eigen::Vector3d::Zero();  ///< Vector from obstacle to robot point
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();  ///< Object pose in world frame
};

/**
 * @brief Calculator for robot-obstacle distances
 *
 * DistanceCalculator uses MoveIt's collision detection API to compute
 * distances between robot links and scene objects.
 *
 * Features:
 * - Computes all distances or minimum distance only
 * - Supports link filtering
 * - Thread-safe operation
 * - Lazy initialization of PlanningSceneMonitor
 */
class DistanceCalculator
{
public:
  /**
   * @brief Constructor
   * @param move_group_name Name of the MoveGroup for distance queries
   * @param robot_description_param Parameter name for robot description (default: "robot_description")
   */
  explicit DistanceCalculator(const std::string& move_group_name,
                              const std::string& robot_description_param = "robot_description");

  /**
   * @brief Sets the PlanningSceneMonitor to use
   * @param monitor Existing PlanningSceneMonitor to use
   *
   * If a monitor is set, it will be used instead of creating a new one.
   * This allows sharing a single monitor across multiple components.
   */
  void setPlanningSceneMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& monitor);

  /**
   * @brief Computes distances between robot links and all obstacles
   * @param link_filter Optional list of link names to include (empty = all links)
   * @return Vector of DistanceResult for each link-obstacle pair
   */
  std::vector<DistanceResult> computeAllDistances(
      const std::vector<std::string>& link_filter = {});

  /**
   * @brief Computes the minimum distance only
   * @param link_filter Optional list of link names to include (empty = all links)
   * @return Minimum distance result, or empty if computation failed
   */
  std::optional<DistanceResult> computeMinimumDistance(
      const std::vector<std::string>& link_filter = {});

  /**
   * @brief Sets the distance threshold for queries
   * @param threshold Maximum distance to report (infinity = no limit)
   */
  void setDistanceThreshold(double threshold);

  /**
   * @brief Sets the maximum contacts per body pair
   * @param max_contacts Maximum number of contacts to compute per body pair
   */
  void setMaxContactsPerBody(int max_contacts);

  /**
   * @brief Checks if the calculator is properly initialized
   * @return true if ready to compute distances
   */
  bool isInitialized() const;

  /**
   * @brief Gets the planning frame name
   * @return Planning frame from the robot model
   */
  std::string getPlanningFrame() const;

private:
  /**
   * @brief Ensures the PlanningSceneMonitor is initialized
   * @return true if initialization succeeded
   */
  bool ensureInitialized();

  /**
   * @brief Configures the distance request
   */
  void configureDistanceRequest();

  /**
   * @brief Validates link names and creates filter set
   * @param link_names List of link names to validate
   * @param model Robot model for validation
   * @return Set of valid LinkModel pointers, empty if all should be included
   */
  std::set<const moveit::core::LinkModel*> createLinkFilter(
      const std::vector<std::string>& link_names,
      const moveit::core::RobotModelConstPtr& model) const;

  /**
   * @brief Transforms a point from local to world coordinates
   * @param local_point Point in local frame
   * @param body_type Type of body (ROBOT_LINK, WORLD_OBJECT, etc.)
   * @param name Name of the body
   * @param shape_index Index of the collision shape
   * @param robot_state Current robot state
   * @param scene Planning scene
   * @return Point in world coordinates
   */
  Eigen::Vector3d transformToWorld(
      const Eigen::Vector3d& local_point,
      collision_detection::BodyType body_type,
      const std::string& name,
      std::size_t shape_index,
      const moveit::core::RobotState& robot_state,
      const planning_scene::PlanningSceneConstPtr& scene) const;

  std::string move_group_name_;           ///< MoveGroup name for queries
  std::string robot_description_param_;   ///< Robot description parameter name

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;  ///< Scene monitor

  collision_detection::DistanceRequest distance_request_;  ///< Distance query configuration

  mutable std::mutex mutex_;              ///< Mutex for thread safety
  bool initialized_ = false;              ///< Initialization flag
};

}  // namespace distance
}  // namespace scene_builder

#endif  // SCENE_BUILDER_DISTANCE_DISTANCE_CALCULATOR_HPP

