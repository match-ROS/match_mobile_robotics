/**
 * @file object_manager.cpp
 * @brief Implementation of the dynamic collision object manager
 *
 * This file implements ObjectManager, which manages:
 * - Adding/removing objects in the MoveIt scene
 * - Animations via waypoints
 * - Control via velocity commands
 * - Motion sequences
 */

#include "scene_builder/objects/object_manager.hpp"
#include "scene_builder/core/pose_utils.hpp"
#include "scene_builder/core/yaml_parser.hpp"
#include "scene_builder/objects/collision_object_factory.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace scene_builder
{
namespace objects
{

namespace
{
/**
 * @brief Checks if a frame ID is empty
 * @param frame Frame name to check
 * @return true if the frame is empty
 */
bool isFrameEmpty(const std::string& frame)
{
  return frame.empty();
}
}  // namespace

ObjectManager::ObjectManager(const ros::NodeHandle& nh, const std::string& move_group_name)
  : planning_scene_interface_("", true, true)
  , move_group_(move_group_name)
  , nh_(nh)
  , waypoint_animator_(0.5)
  , velocity_controller_(1.0, 1.0)
{
  move_group_.setPlanningTime(5.0);

  // Load configurable parameters
  double max_velocity_norm = 1.0;
  double default_velocity_timeout = 1.0;
  nh_.param("max_velocity_norm", max_velocity_norm, max_velocity_norm);
  nh_.param("default_velocity_timeout", default_velocity_timeout, default_velocity_timeout);

  velocity_controller_.setMaxVelocity(max_velocity_norm);
  velocity_controller_.setDefaultTimeout(default_velocity_timeout);
}

void ObjectManager::setMaxVelocityNorm(double value)
{
  velocity_controller_.setMaxVelocity(value);
}

void ObjectManager::setDefaultVelocityTimeout(double value)
{
  velocity_controller_.setDefaultTimeout(value);
}

bool ObjectManager::addObject(const moveit_msgs::CollisionObject& object, std::string& error_message)
{
  std::lock_guard<std::mutex> lk(mutex_);

  if (object.id.empty())
  {
    error_message = "Collision object id is empty";
    return false;
  }

  moveit_msgs::CollisionObject to_add = object;

  if (to_add.header.frame_id.empty())
  {
    to_add.header.frame_id = move_group_.getPlanningFrame();
  }

  // Add missing poses for primitives
  if (to_add.primitives.size() > to_add.primitive_poses.size())
  {
    const std::size_t missing = to_add.primitives.size() - to_add.primitive_poses.size();
    for (std::size_t i = 0; i < missing; ++i)
    {
      to_add.primitive_poses.push_back(core::identityPose());
    }
  }

  // Add missing poses for meshes
  if (to_add.meshes.size() > to_add.mesh_poses.size())
  {
    const std::size_t missing = to_add.meshes.size() - to_add.mesh_poses.size();
    for (std::size_t i = 0; i < missing; ++i)
    {
      to_add.mesh_poses.push_back(core::identityPose());
    }
  }

  std::vector<moveit_msgs::CollisionObject> objects{to_add};
  try
  {
    planning_scene_interface_.applyCollisionObjects(objects);
    object_states_.erase(to_add.id);
    return true;
  }
  catch (const std::exception& ex)
  {
    error_message = ex.what();
    return false;
  }
}

bool ObjectManager::queueWaypoint(const std::string& object_id,
                                  const WaypointCommand& command,
                                  std::string& error_message)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ObjectState* state = nullptr;

  if (!ensureObjectState(object_id, state))
  {
    error_message = "Collision object with id '" + object_id + "' does not exist";
    return false;
  }

  waypoint_animator_.queueWaypoint(*state, command);
  state->active_velocity.reset();
  return true;
}

void ObjectManager::setWaypointSequence(const std::string& object_id,
                                        const std::vector<WaypointCommand>& sequence,
                                        bool loop)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ObjectState* state = nullptr;

  if (!ensureObjectState(object_id, state))
  {
    ROS_WARN_STREAM_NAMED("scene_builder",
                          "Cannot set sequence for object '" << object_id << "': object not found.");
    return;
  }

  sequence_manager_.setSequence(*state, sequence, loop);
}

bool ObjectManager::applyVelocity(const std::string& object_id,
                                  const VelocityCommand& command,
                                  std::string& error_message)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ObjectState* state = nullptr;

  if (!ensureObjectState(object_id, state))
  {
    error_message = "Collision object with id '" + object_id + "' does not exist";
    return false;
  }

  // TODO: Transform velocity from reference frame to object frame if needed
  if (!isFrameEmpty(command.reference_frame))
  {
    geometry_msgs::Pose current_pose;
    if (!getObjectPose(object_id, current_pose))
    {
      error_message = "Cannot apply velocity without current pose";
      return false;
    }
    // Transform not implemented yet - skipping for now
  }

  velocity_controller_.applyVelocity(*state, command);
  return true;
}

bool ObjectManager::removeObject(const std::string& object_id, std::string& error_message)
{
  std::lock_guard<std::mutex> lk(mutex_);

  object_states_.erase(object_id);

  if (object_id.empty())
  {
    error_message = "Collision object id is empty";
    return false;
  }

  try
  {
    planning_scene_interface_.removeCollisionObjects({object_id});
    return true;
  }
  catch (const std::exception& ex)
  {
    error_message = ex.what();
    return false;
  }
}

void ObjectManager::clearCommands(const std::string& object_id)
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = object_states_.find(object_id);
  if (it != object_states_.end())
  {
    waypoint_animator_.clearQueue(it->second);
    velocity_controller_.stopVelocity(it->second);
    sequence_manager_.clearSequence(it->second);
  }
}

bool ObjectManager::getSequenceState(const std::string& object_id,
                                     bool& is_active,
                                     bool& is_loop,
                                     std::vector<WaypointCommand>& waypoints,
                                     std::size_t& current_index)
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = object_states_.find(object_id);
  if (it == object_states_.end())
  {
    const auto existing_objects = planning_scene_interface_.getObjects({object_id});
    if (existing_objects.empty())
    {
      return false;
    }
    is_active = false;
    is_loop = false;
    waypoints.clear();
    current_index = 0;
    return true;
  }

  sequence_manager_.getSequenceState(it->second, is_active, is_loop, waypoints, current_index);
  return true;
}

bool ObjectManager::getObjectPosePublic(const std::string& object_id, geometry_msgs::Pose& pose_out)
{
  std::lock_guard<std::mutex> lk(mutex_);
  return getObjectPose(object_id, pose_out);
}

std::vector<moveit_msgs::CollisionObject> ObjectManager::listObjects()
{
  std::lock_guard<std::mutex> lk(mutex_);

  const auto objects_map = planning_scene_interface_.getObjects();

  std::vector<moveit_msgs::CollisionObject> objects;
  objects.reserve(objects_map.size());
  for (const auto& entry : objects_map)
  {
    objects.push_back(entry.second);
  }
  return objects;
}

void ObjectManager::update(const ros::Time& now, double update_dt)
{
  std::lock_guard<std::mutex> lk(mutex_);

  for (auto& pair : object_states_)
  {
    const std::string& object_id = pair.first;
    ObjectState& state = pair.second;

    std::string error;
    geometry_msgs::Pose current_pose;

    // Initialize pose if not yet done
    if (!state.has_pose)
    {
      if (!getObjectPose(object_id, current_pose))
      {
        continue;
      }
      state.last_pose = current_pose;
      state.has_pose = true;
    }

    // Enqueue next sequence waypoint if needed
    if (!velocity_controller_.hasActiveVelocity(state) && !waypoint_animator_.hasActiveAnimation(state))
    {
      sequence_manager_.enqueueNextIfNeeded(state);
    }

    bool updated = false;

    // MODE 1: Active velocity command
    if (auto new_pose = velocity_controller_.update(state, now, update_dt))
    {
      if (moveObjectInternal(object_id, *new_pose, error))
      {
        state.last_pose = *new_pose;
        updated = true;
      }
    }
    // MODE 2: Waypoints in queue
    else if (auto new_pose = waypoint_animator_.update(state, now))
    {
      if (moveObjectInternal(object_id, *new_pose, error))
      {
        state.last_pose = *new_pose;
        updated = true;
      }
    }

    // If no updates occurred, sync with scene
    if (!updated)
    {
      if (getObjectPose(object_id, current_pose))
      {
        state.last_pose = current_pose;
      }
    }
  }
}

void ObjectManager::loadObjectsFromParameter(const ros::NodeHandle& nh,
                                             const std::string& param,
                                             std::vector<moveit_msgs::CollisionObject>* loaded_objects)
{
  const std::string planning_frame = move_group_.getPlanningFrame();
  auto definitions = core::YamlParser::parseObjects(nh, param, planning_frame);

  if (definitions.empty())
  {
    ROS_DEBUG_STREAM_NAMED("scene_builder", "No objects loaded from param: " << param);
    return;
  }

  std::vector<moveit_msgs::CollisionObject> objects;
  objects.reserve(definitions.size());

  for (const auto& def : definitions)
  {
    try
    {
      auto obj = CollisionObjectFactory::createFromDefinition(def);
      objects.push_back(obj);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM_NAMED("scene_builder", "Failed to create object '" << def.id << "': " << ex.what());
    }
  }

  if (!objects.empty())
  {
    planning_scene_interface_.applyCollisionObjects(objects);
    if (loaded_objects)
    {
      *loaded_objects = objects;
    }
    ROS_INFO_STREAM_NAMED("scene_builder", "Loaded " << objects.size() << " objects from param: " << param);
  }
}

bool ObjectManager::moveObjectInternal(const std::string& object_id,
                                       const geometry_msgs::Pose& pose,
                                       std::string& error_message)
{
  if (object_id.empty())
  {
    error_message = "Collision object id is empty";
    return false;
  }

  const auto existing_objects = planning_scene_interface_.getObjects({object_id});
  if (existing_objects.empty())
  {
    error_message = "Collision object with id '" + object_id + "' does not exist";
    return false;
  }

  const auto& existing = existing_objects.begin()->second;

  std::string frame_id = existing.header.frame_id;
  if (frame_id.empty())
  {
    frame_id = move_group_.getPlanningFrame();
  }

  moveit_msgs::CollisionObject move_cmd = CollisionObjectFactory::createMoveCommand(object_id, frame_id, pose);

  try
  {
    planning_scene_interface_.applyCollisionObject(move_cmd);
    return true;
  }
  catch (const std::exception& ex)
  {
    error_message = ex.what();
    return false;
  }
}

bool ObjectManager::getObjectPose(const std::string& object_id, geometry_msgs::Pose& pose_out)
{
  const auto existing_objects = planning_scene_interface_.getObjects({object_id});
  if (existing_objects.empty())
  {
    return false;
  }

  const auto& existing = existing_objects.begin()->second;

  bool pose_is_set = (existing.pose.position.x != 0.0 ||
                      existing.pose.position.y != 0.0 ||
                      existing.pose.position.z != 0.0);

  const double q_norm_sq = existing.pose.orientation.x * existing.pose.orientation.x +
                           existing.pose.orientation.y * existing.pose.orientation.y +
                           existing.pose.orientation.z * existing.pose.orientation.z +
                           existing.pose.orientation.w * existing.pose.orientation.w;
  if (q_norm_sq > 0.5 && (existing.pose.orientation.w < 0.999 ||
      existing.pose.orientation.x != 0.0 || existing.pose.orientation.y != 0.0 ||
      existing.pose.orientation.z != 0.0))
  {
    pose_is_set = true;
  }

  if (pose_is_set)
  {
    pose_out = existing.pose;
    return true;
  }

  if (!existing.primitive_poses.empty())
  {
    pose_out = existing.primitive_poses.front();
    return true;
  }
  if (!existing.mesh_poses.empty())
  {
    pose_out = existing.mesh_poses.front();
    return true;
  }
  if (!existing.plane_poses.empty())
  {
    pose_out = existing.plane_poses.front();
    return true;
  }
  pose_out = core::identityPose();
  return true;
}

bool ObjectManager::ensureObjectState(const std::string& object_id, ObjectState*& state_out)
{
  auto it = object_states_.find(object_id);
  if (it == object_states_.end())
  {
    geometry_msgs::Pose pose;
    if (!getObjectPose(object_id, pose))
    {
      return false;
    }
    ObjectState state;
    state.id = object_id;
    state.last_pose = pose;
    state.has_pose = true;
    auto inserted = object_states_.emplace(object_id, state);
    it = inserted.first;
  }
  state_out = &it->second;
  return true;
}

}  // namespace objects
}  // namespace scene_builder
