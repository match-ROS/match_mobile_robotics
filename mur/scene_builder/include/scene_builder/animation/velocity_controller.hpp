/**
 * @file velocity_controller.hpp
 * @brief Velocity-based motion controller for collision objects
 *
 * This file provides the VelocityController class that handles continuous
 * velocity-based movement of objects with clamping and timeout support.
 */

#ifndef SCENE_BUILDER_ANIMATION_VELOCITY_CONTROLLER_HPP
#define SCENE_BUILDER_ANIMATION_VELOCITY_CONTROLLER_HPP

#include "scene_builder/objects/object_state.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <optional>

namespace scene_builder
{
namespace animation
{

/**
 * @brief Controller for velocity-based object motion
 *
 * VelocityController applies continuous velocity commands to objects,
 * integrating twist over time to compute new poses.
 *
 * Features:
 * - Velocity clamping to maximum allowed values
 * - Automatic timeout for safety
 * - Smooth integration of linear and angular velocities
 */
class VelocityController
{
public:
  /**
   * @brief Constructor
   * @param max_velocity Maximum allowed velocity magnitude (m/s and rad/s)
   * @param default_timeout Default timeout for velocity commands (seconds)
   */
  explicit VelocityController(double max_velocity = 1.0, double default_timeout = 1.0);

  /**
   * @brief Applies a velocity command to an object
   * @param state Object state to modify
   * @param cmd Velocity command to apply
   *
   * This replaces any existing velocity command and clears the waypoint queue.
   */
  void applyVelocity(objects::ObjectState& state, const objects::VelocityCommand& cmd);

  /**
   * @brief Stops any active velocity command for an object
   * @param state Object state to modify
   */
  void stopVelocity(objects::ObjectState& state);

  /**
   * @brief Checks if an object has an active velocity command
   * @param state Object state to check
   * @return true if a velocity command is active
   */
  bool hasActiveVelocity(const objects::ObjectState& state) const;

  /**
   * @brief Updates the object position based on active velocity
   * @param state Object state to update
   * @param now Current timestamp
   * @param dt Time step for integration (seconds)
   * @return New pose if velocity is active, empty optional otherwise
   *
   * This method should be called periodically to integrate velocity.
   * It handles:
   * - Checking for timeout expiration
   * - Integrating twist into current pose
   * - Deactivating expired velocity commands
   */
  std::optional<geometry_msgs::Pose> update(objects::ObjectState& state,
                                            const ros::Time& now,
                                            double dt);

  /**
   * @brief Sets the maximum allowed velocity
   * @param max_vel Maximum velocity magnitude
   */
  void setMaxVelocity(double max_vel) { max_velocity_ = max_vel; }

  /**
   * @brief Gets the maximum allowed velocity
   * @return Maximum velocity magnitude
   */
  double getMaxVelocity() const { return max_velocity_; }

  /**
   * @brief Sets the default timeout for velocity commands
   * @param timeout Default timeout in seconds
   */
  void setDefaultTimeout(double timeout) { default_timeout_ = timeout; }

  /**
   * @brief Gets the default timeout
   * @return Default timeout in seconds
   */
  double getDefaultTimeout() const { return default_timeout_; }

private:
  /**
   * @brief Clamps velocity to maximum allowed magnitude
   * @param twist Input twist to clamp
   * @return Clamped twist
   */
  geometry_msgs::Twist clampVelocity(const geometry_msgs::Twist& twist) const;

  double max_velocity_;      ///< Maximum allowed velocity magnitude
  double default_timeout_;   ///< Default timeout for velocity commands
};

}  // namespace animation
}  // namespace scene_builder

#endif  // SCENE_BUILDER_ANIMATION_VELOCITY_CONTROLLER_HPP

