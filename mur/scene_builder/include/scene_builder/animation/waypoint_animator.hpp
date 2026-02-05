/**
 * @file waypoint_animator.hpp
 * @brief Waypoint-based animation controller for collision objects
 *
 * This file provides the WaypointAnimator class that handles smooth movement
 * of objects between waypoints using interpolation.
 */

#ifndef SCENE_BUILDER_ANIMATION_WAYPOINT_ANIMATOR_HPP
#define SCENE_BUILDER_ANIMATION_WAYPOINT_ANIMATOR_HPP

#include "scene_builder/objects/object_state.hpp"

#include <geometry_msgs/Pose.h>
#include <ros/time.h>

#include <optional>

namespace scene_builder
{
namespace animation
{

/**
 * @brief Controller for waypoint-based object animation
 *
 * WaypointAnimator manages the execution of waypoint queues for objects,
 * providing smooth interpolation between poses over specified durations.
 *
 * Features:
 * - Queues multiple waypoints for sequential execution
 * - Smooth pose interpolation (linear position + SLERP orientation)
 * - Configurable default duration for immediate waypoints
 */
class WaypointAnimator
{
public:
  /**
   * @brief Constructor
   * @param default_duration Default duration for waypoints with zero duration (seconds)
   */
  explicit WaypointAnimator(double default_duration = 0.5);

  /**
   * @brief Queues a waypoint for an object
   * @param state Object state to modify
   * @param cmd Waypoint command to queue
   *
   * The waypoint is added to the end of the queue and will be executed
   * after all previously queued waypoints complete.
   */
  void queueWaypoint(objects::ObjectState& state, const objects::WaypointCommand& cmd);

  /**
   * @brief Clears all queued waypoints for an object
   * @param state Object state to clear
   */
  void clearQueue(objects::ObjectState& state);

  /**
   * @brief Checks if an object has active waypoint animation
   * @param state Object state to check
   * @return true if there are waypoints in the queue
   */
  bool hasActiveAnimation(const objects::ObjectState& state) const;

  /**
   * @brief Updates the animation and returns the new pose if active
   * @param state Object state to update
   * @param now Current timestamp
   * @return New pose if animation is active, empty optional otherwise
   *
   * This method should be called periodically to advance the animation.
   * It handles:
   * - Starting new waypoints (recording start time and pose)
   * - Computing interpolation ratio based on elapsed time
   * - Removing completed waypoints from the queue
   */
  std::optional<geometry_msgs::Pose> update(objects::ObjectState& state, const ros::Time& now);

  /**
   * @brief Sets the default duration for waypoints
   * @param duration Default duration in seconds
   */
  void setDefaultDuration(double duration) { default_duration_ = duration; }

  /**
   * @brief Gets the default duration
   * @return Default duration in seconds
   */
  double getDefaultDuration() const { return default_duration_; }

private:
  double default_duration_;  ///< Default duration for zero-duration waypoints
};

}  // namespace animation
}  // namespace scene_builder

#endif  // SCENE_BUILDER_ANIMATION_WAYPOINT_ANIMATOR_HPP

