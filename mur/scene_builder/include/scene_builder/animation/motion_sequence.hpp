/**
 * @file motion_sequence.hpp
 * @brief Motion sequence manager for automatic waypoint playback
 *
 * This file provides the MotionSequenceManager class that handles automatic
 * playback of waypoint sequences with optional looping.
 */

#ifndef SCENE_BUILDER_ANIMATION_MOTION_SEQUENCE_HPP
#define SCENE_BUILDER_ANIMATION_MOTION_SEQUENCE_HPP

#include "scene_builder/objects/object_state.hpp"

#include <vector>
#include <string>

namespace scene_builder
{
namespace animation
{

/**
 * @brief Manager for automatic motion sequence playback
 *
 * MotionSequenceManager handles the automatic execution of waypoint sequences,
 * supporting both one-shot and looping playback modes.
 *
 * Features:
 * - Sequential waypoint execution
 * - Loop mode for continuous animation
 * - Progress tracking (current waypoint index)
 * - Integration with WaypointAnimator queue
 */
class MotionSequenceManager
{
public:
  /**
   * @brief Default constructor
   */
  MotionSequenceManager() = default;

  /**
   * @brief Sets a waypoint sequence for an object
   * @param state Object state to modify
   * @param sequence Sequence of waypoint commands
   * @param loop Whether to loop the sequence
   *
   * This clears any existing sequence and sets up the new one.
   * The sequence will start automatically when update is called.
   */
  void setSequence(objects::ObjectState& state,
                   const std::vector<objects::WaypointCommand>& sequence,
                   bool loop);

  /**
   * @brief Clears the current sequence for an object
   * @param state Object state to modify
   *
   * This stops any ongoing sequence playback and clears the template.
   */
  void clearSequence(objects::ObjectState& state);

  /**
   * @brief Checks if a sequence is currently active
   * @param state Object state to check
   * @return true if a sequence is active
   */
  bool isSequenceActive(const objects::ObjectState& state) const;

  /**
   * @brief Checks if the sequence is in loop mode
   * @param state Object state to check
   * @return true if looping is enabled
   */
  bool isLooping(const objects::ObjectState& state) const;

  /**
   * @brief Gets the current waypoint index in the sequence
   * @param state Object state to query
   * @return Index of the next waypoint to be executed
   */
  std::size_t getCurrentIndex(const objects::ObjectState& state) const;

  /**
   * @brief Gets the sequence template
   * @param state Object state to query
   * @return Reference to the waypoint sequence template
   */
  const std::vector<objects::WaypointCommand>& getSequence(const objects::ObjectState& state) const;

  /**
   * @brief Enqueues the next waypoint from the sequence if needed
   * @param state Object state to modify
   * @return true if a waypoint was enqueued
   *
   * This should be called when the waypoint queue is empty to continue
   * sequence playback. It handles:
   * - Advancing to the next waypoint in the sequence
   * - Looping back to the start if loop mode is enabled
   * - Deactivating the sequence when complete (non-loop mode)
   */
  bool enqueueNextIfNeeded(objects::ObjectState& state);

  /**
   * @brief Gets the sequence state
   * @param state Object state to query
   * @param is_active Output: whether sequence is active
   * @param is_loop Output: whether looping is enabled
   * @param waypoints Output: sequence waypoints
   * @param current_index Output: current waypoint index
   */
  void getSequenceState(const objects::ObjectState& state,
                        bool& is_active,
                        bool& is_loop,
                        std::vector<objects::WaypointCommand>& waypoints,
                        std::size_t& current_index) const;
};

}  // namespace animation
}  // namespace scene_builder

#endif  // SCENE_BUILDER_ANIMATION_MOTION_SEQUENCE_HPP

