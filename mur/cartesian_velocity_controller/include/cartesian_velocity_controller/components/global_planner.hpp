#pragma once

/**
 * @file global_planner.hpp
 * @brief Global Planner for waypoint-based navigation (Level A of pipeline).
 *
 * The GlobalPlanner manages a list of waypoints and provides the "current active
 * waypoint" to the LocalPlanner. It handles automatic switching to the next
 * waypoint when the robot reaches a configurable distance threshold.
 *
 * Features:
 * - Multiple waypoints with automatic progression
 * - Single waypoint mode (maintains target)
 * - Configurable switch distance (position and optional orientation)
 * - Dynamic parameter support for fuzzy control integration
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <vector>
#include <functional>

#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @class GlobalPlanner
 * @brief Manages waypoint list and provides current target for the LocalPlanner.
 *
 * The GlobalPlanner is the first level (Level A) of the new pipeline architecture.
 * It receives a list of waypoints and outputs the current active waypoint based
 * on the robot's position. When the robot reaches a waypoint (within threshold),
 * it automatically advances to the next one.
 *
 * For single-waypoint operation (the common case), the waypoint remains as the
 * target until explicitly changed or cleared.
 */
class GlobalPlanner
{
public:
  /// Callback type for waypoint reached events
  using WaypointReachedCallback = std::function<void(std::size_t index, const Eigen::Isometry3d& pose)>;

  /// Callback type for path completed events
  using PathCompletedCallback = std::function<void()>;

  /**
   * @brief Default constructor.
   */
  GlobalPlanner();

  /**
   * @brief Destructor.
   */
  ~GlobalPlanner() = default;

  // ============== Waypoint Management ==============

  /**
   * @brief Set a list of waypoints to follow.
   * @param waypoints Vector of poses to follow in order
   *
   * Clears any existing waypoints and resets the current index to 0.
   */
  void setWaypoints(const std::vector<Eigen::Isometry3d>& waypoints);

  /**
   * @brief Set a list of waypoints with extended info.
   * @param waypoints Vector of WaypointInfo structures
   */
  void setWaypoints(const std::vector<WaypointInfo>& waypoints);

  /**
   * @brief Add a single waypoint to the end of the list.
   * @param waypoint Pose to add
   */
  void addWaypoint(const Eigen::Isometry3d& waypoint);

  /**
   * @brief Add a waypoint with extended info.
   * @param waypoint WaypointInfo to add
   */
  void addWaypoint(const WaypointInfo& waypoint);

  /**
   * @brief Insert a waypoint at a specific position.
   * @param index Position to insert (0-based)
   * @param waypoint Pose to insert
   * @return true if inserted successfully
   */
  bool insertWaypoint(std::size_t index, const Eigen::Isometry3d& waypoint);

  /**
   * @brief Remove a waypoint at a specific position.
   * @param index Position to remove (0-based)
   * @return true if removed successfully
   */
  bool removeWaypoint(std::size_t index);

  /**
   * @brief Clear all waypoints.
   *
   * After clearing, hasWaypoints() will return false.
   */
  void clearWaypoints();

  /**
   * @brief Get the total number of waypoints.
   * @return Number of waypoints
   */
  std::size_t getWaypointCount() const;

  /**
   * @brief Check if any waypoints are set.
   * @return true if at least one waypoint exists
   */
  bool hasWaypoints() const;

  /**
   * @brief Get all waypoints.
   * @return Vector of all waypoints
   */
  std::vector<WaypointInfo> getWaypoints() const;

  // ============== Navigation ==============

  /**
   * @brief Get the current active waypoint.
   * @return Current waypoint pose
   *
   * If no waypoints are set, returns identity pose.
   */
  Eigen::Isometry3d getCurrentWaypoint() const;

  /**
   * @brief Get the current active waypoint with full info.
   * @return Current WaypointInfo
   */
  WaypointInfo getCurrentWaypointInfo() const;

  /**
   * @brief Get the index of the current waypoint.
   * @return Current waypoint index (0-based)
   */
  std::size_t getCurrentWaypointIndex() const;

  /**
   * @brief Manually advance to the next waypoint.
   * @return true if advanced, false if already at final waypoint
   */
  bool advanceToNextWaypoint();

  /**
   * @brief Jump to a specific waypoint index.
   * @param index Waypoint index to jump to
   * @return true if valid index, false otherwise
   */
  bool jumpToWaypoint(std::size_t index);

  /**
   * @brief Check if currently at the final waypoint.
   * @return true if at the last waypoint
   */
  bool isAtFinalWaypoint() const;

  /**
   * @brief Reset navigation to the first waypoint.
   */
  void resetToStart();

  // ============== Position Update and Switch Logic ==============

  /**
   * @brief Update with the current robot position and check for waypoint switch.
   * @param current_pose Current TCP pose of the robot
   * @return true if a waypoint switch occurred
   *
   * This method should be called on every control cycle. It computes the
   * distance to the current waypoint and automatically advances if the
   * threshold is met.
   */
  bool updateCurrentPosition(const Eigen::Isometry3d& current_pose);

  /**
   * @brief Get the last computed distance to the current waypoint.
   * @return Distance in meters
   */
  double getDistanceToCurrentWaypoint() const;

  /**
   * @brief Get the last computed angular distance to the current waypoint.
   * @return Angular distance in radians
   */
  double getAngularDistanceToCurrentWaypoint() const;

  // ============== Dynamic Parameters (for Fuzzy Control) ==============

  /**
   * @brief Set the distance threshold for waypoint switching.
   * @param distance Distance in meters
   *
   * When the robot is within this distance of a waypoint, it will advance
   * to the next waypoint. Default: 0.05m (5cm)
   */
  void setWaypointSwitchDistance(double distance);

  /**
   * @brief Get the current waypoint switch distance.
   * @return Distance threshold in meters
   */
  double getWaypointSwitchDistance() const;

  /**
   * @brief Set whether orientation should be considered for waypoint switching.
   * @param use_orientation If true, both position AND orientation must be within threshold
   */
  void setUseOrientationForSwitch(bool use_orientation);

  /**
   * @brief Check if orientation is used for waypoint switching.
   * @return true if orientation is considered
   */
  bool getUseOrientationForSwitch() const;

  /**
   * @brief Set the orientation threshold for waypoint switching.
   * @param threshold Angular threshold in radians
   *
   * Only used if setUseOrientationForSwitch(true). Default: 0.1 rad (~5.7°)
   */
  void setOrientationSwitchThreshold(double threshold);

  /**
   * @brief Get the orientation switch threshold.
   * @return Angular threshold in radians
   */
  double getOrientationSwitchThreshold() const;

  // ============== Callbacks ==============

  /**
   * @brief Set callback for when a waypoint is reached.
   * @param callback Function to call when waypoint is reached
   */
  void setWaypointReachedCallback(WaypointReachedCallback callback);

  /**
   * @brief Set callback for when the entire path is completed.
   * @param callback Function to call when path is complete
   */
  void setPathCompletedCallback(PathCompletedCallback callback);

  // ============== Reset ==============

  /**
   * @brief Reset the planner state.
   *
   * Resets to first waypoint but keeps the waypoint list.
   */
  void reset();

private:
  /**
   * @brief Compute distance between two poses (position only).
   */
  static double computePositionDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b);

  /**
   * @brief Compute angular distance between two poses.
   */
  static double computeOrientationDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b);

  // ============== Waypoint Data ==============

  std::vector<WaypointInfo> waypoints_;
  std::size_t current_waypoint_index_{0};
  mutable std::mutex waypoints_mutex_;

  // ============== Switch Parameters ==============

  double waypoint_switch_distance_{0.05};        // 5cm default
  bool use_orientation_for_switch_{true};
  double orientation_switch_threshold_{0.1};     // ~5.7° default
  mutable std::mutex params_mutex_;

  // ============== State ==============

  double last_position_distance_{std::numeric_limits<double>::infinity()};
  double last_angular_distance_{0.0};
  mutable std::mutex state_mutex_;

  // ============== Callbacks ==============

  WaypointReachedCallback waypoint_reached_callback_;
  PathCompletedCallback path_completed_callback_;
  std::mutex callback_mutex_;
};

}  // namespace cartesian_velocity_controller

