/**
 * @file object_state.hpp
 * @brief State representation for managed collision objects
 *
 * Defines the internal state structure for objects managed by ObjectManager,
 * including animation state, waypoint queues, and velocity commands.
 */

#ifndef SCENE_BUILDER_OBJECTS_OBJECT_STATE_HPP
#define SCENE_BUILDER_OBJECTS_OBJECT_STATE_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace scene_builder
{
namespace objects
{

/**
 * @brief Command to move an object toward a target pose (waypoint)
 *
 * Specifies a target pose and suggested duration for the movement.
 * Interpolation between start and target pose is handled automatically.
 */
struct WaypointCommand
{
  geometry_msgs::Pose target_pose;      ///< Target pose to reach
  ros::Duration duration_hint{0.0};     ///< Suggested movement duration (0 = immediate)
  ros::Time start_time;                 ///< Timestamp when movement started
  geometry_msgs::Pose start_pose;       ///< Initial pose of the object
  bool started = false;                 ///< Flag indicating if movement has started
};

/**
 * @brief Command to move an object with constant velocity
 *
 * Applies linear and angular velocity to the object until timeout.
 */
struct VelocityCommand
{
  geometry_msgs::Twist twist;           ///< Linear and angular velocity
  std::string reference_frame;          ///< Reference frame (empty = object frame)
  ros::Duration timeout{0.0};           ///< Timeout after which to stop movement
  ros::Time stamp;                      ///< Timestamp when command was received
};

/**
 * @brief Internal state of a managed object
 *
 * Maintains the waypoint queue, active velocity command, and last known pose.
 * Also tracks motion sequence state for automatic waypoint playback.
 */
struct ObjectState
{
  std::string id;                       ///< Object identifier

  // Pose tracking
  geometry_msgs::Pose last_pose;        ///< Last known pose of the object
  bool has_pose = false;                ///< Flag indicating if last_pose is valid

  // Animation state
  std::deque<WaypointCommand> waypoint_queue;  ///< Queue of waypoints to execute
  std::optional<VelocityCommand> active_velocity;  ///< Active velocity command (if any)

  // Motion sequence state
  std::vector<WaypointCommand> sequence_template;  ///< Waypoints for automatic playback
  bool loop_sequence = false;           ///< Whether to loop the sequence automatically
  std::size_t next_sequence_index = 0;  ///< Index of next waypoint in sequence
  bool sequence_active = false;         ///< Whether automatic sequence is active

  /**
   * @brief Clears all animation commands and sequence state
   */
  void clearCommands()
  {
    waypoint_queue.clear();
    active_velocity.reset();
    sequence_template.clear();
    loop_sequence = false;
    next_sequence_index = 0;
    sequence_active = false;
  }

  /**
   * @brief Checks if the object has any active animation
   * @return true if there's an active velocity command or pending waypoints
   */
  bool hasActiveAnimation() const
  {
    return active_velocity.has_value() || !waypoint_queue.empty();
  }
};

}  // namespace objects
}  // namespace scene_builder

#endif  // SCENE_BUILDER_OBJECTS_OBJECT_STATE_HPP

