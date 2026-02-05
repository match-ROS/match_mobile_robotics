/**
 * @file motion_sequence.cpp
 * @brief Implementation of motion sequence manager
 */

#include "scene_builder/animation/motion_sequence.hpp"
#include "scene_builder/core/pose_utils.hpp"

#include <ros/console.h>

namespace scene_builder
{
namespace animation
{

void MotionSequenceManager::setSequence(objects::ObjectState& state,
                                        const std::vector<objects::WaypointCommand>& sequence,
                                        bool loop)
{
  state.sequence_template = sequence;
  state.loop_sequence = loop;
  state.next_sequence_index = 0;
  state.sequence_active = !sequence.empty();

  // Clear current animation when setting a new sequence
  if (state.sequence_active)
  {
    state.waypoint_queue.clear();
    state.active_velocity.reset();
  }
}

void MotionSequenceManager::clearSequence(objects::ObjectState& state)
{
  state.sequence_template.clear();
  state.loop_sequence = false;
  state.next_sequence_index = 0;
  state.sequence_active = false;
}

bool MotionSequenceManager::isSequenceActive(const objects::ObjectState& state) const
{
  return state.sequence_active;
}

bool MotionSequenceManager::isLooping(const objects::ObjectState& state) const
{
  return state.loop_sequence;
}

std::size_t MotionSequenceManager::getCurrentIndex(const objects::ObjectState& state) const
{
  return state.next_sequence_index;
}

const std::vector<objects::WaypointCommand>& MotionSequenceManager::getSequence(
    const objects::ObjectState& state) const
{
  return state.sequence_template;
}

bool MotionSequenceManager::enqueueNextIfNeeded(objects::ObjectState& state)
{
  // Only enqueue if sequence is active and waypoint queue is empty
  if (!state.sequence_active || state.sequence_template.empty())
  {
    return false;
  }

  // Don't enqueue if there are still waypoints pending
  if (!state.waypoint_queue.empty())
  {
    return false;
  }

  // Check if we've reached the end of the sequence
  if (state.next_sequence_index >= state.sequence_template.size())
  {
    if (state.loop_sequence)
    {
      // Loop back to start
      state.next_sequence_index = 0;
    }
    else
    {
      // Sequence complete, deactivate
      state.sequence_active = false;
      state.sequence_template.clear();
      state.next_sequence_index = 0;
      return false;
    }
  }

  // Enqueue the next waypoint
  const std::size_t queued_index = state.next_sequence_index;
  objects::WaypointCommand cmd = state.sequence_template[queued_index];
  cmd.target_pose = core::sanitizePose(cmd.target_pose);
  cmd.started = false;
  cmd.start_time = ros::Time();
  state.waypoint_queue.push_back(cmd);
  state.next_sequence_index++;

  ROS_DEBUG_STREAM_NAMED("scene_builder",
                         "Enqueued automatic waypoint #" << (queued_index + 1)
                         << " for object '" << state.id << "'.");

  return true;
}

void MotionSequenceManager::getSequenceState(const objects::ObjectState& state,
                                             bool& is_active,
                                             bool& is_loop,
                                             std::vector<objects::WaypointCommand>& waypoints,
                                             std::size_t& current_index) const
{
  is_active = state.sequence_active;
  is_loop = state.loop_sequence;
  waypoints = state.sequence_template;
  current_index = state.next_sequence_index;
}

}  // namespace animation
}  // namespace scene_builder

