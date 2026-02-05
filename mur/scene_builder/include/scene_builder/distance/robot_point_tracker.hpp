/**
 * @file robot_point_tracker.hpp
 * @brief Tracking of interest points on the robot
 *
 * This file provides the RobotPointTracker class that tracks specific points
 * of interest on the robot (e.g., elbow, wrist) and computes their positions,
 * velocities, and distances to collision objects.
 */

#ifndef SCENE_BUILDER_DISTANCE_ROBOT_POINT_TRACKER_HPP
#define SCENE_BUILDER_DISTANCE_ROBOT_POINT_TRACKER_HPP

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <Eigen/Dense>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace scene_builder
{
namespace distance
{

/**
 * @brief Configuration for a single point of interest on the robot
 */
struct RobotPointConfig
{
  std::string name;                                   ///< Name of the point (e.g., "elbow", "wrist")
  std::string link_name;                              ///< Name of the robot link
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();   ///< Offset in link frame (optional)
};

/**
 * @brief Current state of a tracked robot point
 */
struct RobotPointState
{
  std::string name;                                   ///< Name of the point
  std::string link_name;                              ///< Link name
  Eigen::Vector3d position = Eigen::Vector3d::Zero(); ///< Position in world frame
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); ///< Linear velocity in world frame
  bool valid = false;                                 ///< Whether the state is valid
};

/**
 * @brief Distance from a robot point to an object center
 */
struct PointToObjectDistance
{
  std::string point_name;                             ///< Name of the robot point
  std::string link_name;                              ///< Link name
  Eigen::Vector3d point_position = Eigen::Vector3d::Zero();   ///< Point position in world frame
  Eigen::Vector3d point_velocity = Eigen::Vector3d::Zero();   ///< Point velocity in world frame
  std::string object_id;                              ///< ID of the collision object
  Eigen::Vector3d distance_vector = Eigen::Vector3d::Zero();  ///< Vector from point to object center
  double distance = 0.0;                              ///< Scalar distance
  double object_characteristic_radius = 0.0;          ///< Characteristic radius of the object
};

/**
 * @brief Configuration for the RobotPointTracker
 */
struct RobotPointTrackerConfig
{
  double velocity_filter_alpha = 0.3;   ///< Low-pass filter coefficient for velocity (0-1)
  double min_dt = 0.001;                ///< Minimum time delta to compute velocity (s)
  double max_dt = 1.0;                  ///< Maximum time delta before invalidating velocity (s)
};

/**
 * @brief Tracks points of interest on the robot
 *
 * RobotPointTracker monitors specific points on the robot and computes:
 * - Position in world frame (via forward kinematics)
 * - Velocity (via finite differences with low-pass filtering)
 * - Distance to collision object centers (simple geometric distance, no FCL)
 *
 * Features:
 * - Configurable points with link + offset
 * - Thread-safe operation
 * - Low-pass filtered velocity estimation
 */
class RobotPointTracker
{
public:
  /**
   * @brief Constructor
   * @param config Configuration for tracking
   */
  explicit RobotPointTracker(const RobotPointTrackerConfig& config = {});

  /**
   * @brief Loads point configurations from ROS parameters
   * @param nh NodeHandle to read parameters from
   * @param param_name Name of the parameter containing point definitions
   * @return true if at least one point was loaded successfully
   *
   * Expected parameter format:
   * @code
   * robot_points_of_interest:
   *   elbow:
   *     link: "forearm_link"
   *     offset: [0.0, 0.0, 0.0]  # optional
   *   wrist:
   *     link: "wrist_3_link"
   * @endcode
   */
  bool loadFromParameter(const ros::NodeHandle& nh, const std::string& param_name);

  /**
   * @brief Adds a point of interest to track
   * @param config Point configuration
   */
  void addPoint(const RobotPointConfig& config);

  /**
   * @brief Removes a tracked point
   * @param name Name of the point to remove
   */
  void removePoint(const std::string& name);

  /**
   * @brief Clears all tracked points
   */
  void clearPoints();

  /**
   * @brief Gets the list of configured points
   * @return Vector of point configurations
   */
  std::vector<RobotPointConfig> getPointConfigs() const;

  /**
   * @brief Updates point states from current robot state
   * @param scene_monitor Planning scene monitor to read robot state from
   * @param current_time Current timestamp for velocity computation
   */
  void update(const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor,
              const ros::Time& current_time);

  /**
   * @brief Gets the current state of a specific point
   * @param name Name of the point
   * @return Point state (check valid field before using)
   */
  RobotPointState getPointState(const std::string& name) const;

  /**
   * @brief Gets all point states
   * @return Map of point names to their states
   */
  std::map<std::string, RobotPointState> getAllPointStates() const;

  /**
   * @brief Computes distances from all tracked points to all collision objects
   * @param scene_monitor Planning scene monitor to read object positions from
   * @return Vector of point-to-object distances
   *
   * Note: This computes simple geometric distance from the point to the
   * object's pose origin (center), NOT the FCL minimum distance.
   */
  std::vector<PointToObjectDistance> computeDistancesToObjects(
      const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor) const;

  /**
   * @brief Sets the configuration
   * @param config New configuration
   */
  void setConfig(const RobotPointTrackerConfig& config);

  /**
   * @brief Gets the current configuration
   * @return Current configuration
   */
  const RobotPointTrackerConfig& getConfig() const { return config_; }

  /**
   * @brief Resets velocity estimates (call after large time gaps)
   */
  void resetVelocities();

private:
  /**
   * @brief Internal tracking state for a point
   */
  struct InternalPointState
  {
    RobotPointConfig config;                          ///< Point configuration
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();  ///< Last known position
    Eigen::Vector3d filtered_velocity = Eigen::Vector3d::Zero(); ///< Filtered velocity
    ros::Time last_update_time;                       ///< Time of last update
    bool initialized = false;                         ///< Whether we have a previous position
  };

  /**
   * @brief Applies low-pass filter to velocity
   */
  Eigen::Vector3d filterVelocity(const Eigen::Vector3d& current_filtered,
                                  const Eigen::Vector3d& new_measurement,
                                  double alpha) const;

  RobotPointTrackerConfig config_;                    ///< Current configuration
  std::map<std::string, InternalPointState> points_;  ///< Tracked points
  mutable std::mutex mutex_;                          ///< Mutex for thread safety
};

}  // namespace distance
}  // namespace scene_builder

#endif  // SCENE_BUILDER_DISTANCE_ROBOT_POINT_TRACKER_HPP
