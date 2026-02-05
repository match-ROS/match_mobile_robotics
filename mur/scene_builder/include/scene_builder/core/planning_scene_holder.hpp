/**
 * @file planning_scene_holder.hpp
 * @brief Singleton wrapper for shared PlanningSceneMonitor
 *
 * This class provides a single, shared PlanningSceneMonitor instance
 * that can be used across multiple components, avoiding the overhead
 * of creating multiple monitors.
 */

#ifndef SCENE_BUILDER_CORE_PLANNING_SCENE_HOLDER_HPP
#define SCENE_BUILDER_CORE_PLANNING_SCENE_HOLDER_HPP

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/node_handle.h>

#include <memory>
#include <mutex>
#include <string>

namespace scene_builder
{
namespace core
{

/**
 * @brief Singleton class that holds a shared PlanningSceneMonitor
 *
 * This class ensures that only one PlanningSceneMonitor is created and
 * shared across all components that need access to the planning scene.
 * This reduces memory usage and improves performance.
 *
 * Usage:
 * @code
 * // Initialize once at startup
 * PlanningSceneHolder::instance().initialize(nh);
 *
 * // Get read-only access to the scene
 * auto scene_ro = PlanningSceneHolder::instance().getSceneRO();
 *
 * // Get read-write access to the scene
 * auto scene_rw = PlanningSceneHolder::instance().getSceneRW();
 * @endcode
 *
 * Thread-safety: All operations are protected by mutex.
 */
class PlanningSceneHolder
{
public:
  /**
   * @brief Gets the singleton instance
   * @return Reference to the singleton instance
   */
  static PlanningSceneHolder& instance();

  // Prevent copying
  PlanningSceneHolder(const PlanningSceneHolder&) = delete;
  PlanningSceneHolder& operator=(const PlanningSceneHolder&) = delete;

  /**
   * @brief Initializes the PlanningSceneMonitor
   * @param nh NodeHandle for ROS communication
   * @param robot_description Parameter name for robot description (default: "robot_description")
   * @return true if initialization succeeded or already initialized
   *
   * This method is idempotent - calling it multiple times has no effect
   * after the first successful initialization.
   */
  bool initialize(const ros::NodeHandle& nh,
                  const std::string& robot_description = "robot_description");

  /**
   * @brief Gets the underlying PlanningSceneMonitor
   * @return Shared pointer to the PlanningSceneMonitor, or nullptr if not initialized
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getMonitor();

  /**
   * @brief Gets read-only access to the planning scene
   * @return Locked planning scene for reading
   * @throws std::runtime_error if not initialized
   *
   * The returned object holds a read lock that is released when destroyed.
   */
  planning_scene_monitor::LockedPlanningSceneRO getSceneRO();

  /**
   * @brief Gets read-write access to the planning scene
   * @return Locked planning scene for writing
   * @throws std::runtime_error if not initialized
   *
   * The returned object holds a write lock that is released when destroyed.
   */
  planning_scene_monitor::LockedPlanningSceneRW getSceneRW();

  /**
   * @brief Checks if the PlanningSceneMonitor is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const;

  /**
   * @brief Gets the planning frame name
   * @return Planning frame name, or empty string if not initialized
   */
  std::string getPlanningFrame() const;

  /**
   * @brief Gets the robot model
   * @return Robot model, or nullptr if not initialized
   */
  moveit::core::RobotModelConstPtr getRobotModel() const;

private:
  PlanningSceneHolder() = default;
  ~PlanningSceneHolder() = default;

  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  mutable std::mutex mutex_;
  bool initialized_ = false;
};

}  // namespace core
}  // namespace scene_builder

#endif  // SCENE_BUILDER_CORE_PLANNING_SCENE_HOLDER_HPP

