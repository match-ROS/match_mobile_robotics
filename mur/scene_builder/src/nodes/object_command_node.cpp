/**
 * @file object_command_node.cpp
 * @brief Implementation of the object command ROS node
 */

#include "scene_builder/nodes/object_command_node.hpp"

#include <shape_msgs/SolidPrimitive.h>
#include <XmlRpcValue.h>

#include <algorithm>

namespace scene_builder
{
namespace nodes
{

namespace
{
bool validateObjectId(const std::string& id)
{
  return !id.empty();
}

bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double& result)
{
  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeInt:
      result = static_cast<int>(value);
      return true;
    case XmlRpc::XmlRpcValue::TypeDouble:
      result = static_cast<double>(value);
      return true;
    default:
      return false;
  }
}

bool xmlRpcToBool(const XmlRpc::XmlRpcValue& value, bool& result)
{
  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      result = static_cast<bool>(value);
      return true;
    case XmlRpc::XmlRpcValue::TypeInt:
      result = static_cast<int>(value) != 0;
      return true;
    default:
      return false;
  }
}

bool readVector(XmlRpc::XmlRpcValue& array, std::vector<double>& out, std::size_t expected_size)
{
  if (array.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return false;
  }
  const std::size_t arr_size = static_cast<std::size_t>(array.size());
  if (expected_size != 0 && arr_size != expected_size)
  {
    return false;
  }

  out.resize(arr_size);
  for (int i = 0; i < array.size(); ++i)
  {
    double value = 0.0;
    if (!xmlRpcToDouble(array[i], value))
    {
      return false;
    }
    out[static_cast<std::size_t>(i)] = value;
  }
  return true;
}

bool parsePose(XmlRpc::XmlRpcValue& entry, geometry_msgs::Pose& pose_out)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  if (entry.hasMember("position"))
  {
    std::vector<double> position;
    if (!readVector(entry["position"], position, 3))
    {
      return false;
    }
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
  }
  else
  {
    return false;
  }

  if (entry.hasMember("orientation"))
  {
    std::vector<double> orientation;
    if (!readVector(entry["orientation"], orientation, 4))
    {
      return false;
    }
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];
  }

  pose_out = pose;
  return true;
}
}  // namespace

ObjectCommandNode::ObjectCommandNode(const ros::NodeHandle& nh)
  : nh_(nh)
  , pnh_("~")
  , spinner_(1, &callback_queue_)
{
  nh_.setCallbackQueue(&callback_queue_);
  pnh_.setCallbackQueue(&callback_queue_);

  loadParameters();

  manager_ = std::make_unique<objects::ObjectManager>(nh_, move_group_);
  manager_->loadObjectsFromParameter(pnh_, default_objects_param_);

  loadMotionSequences();
  setupPublishersAndSubscribers();
  setupTimers();

  spinner_.start();
}

void ObjectCommandNode::loadParameters()
{
  pnh_.param<std::string>("move_group", move_group_, move_group_);
  pnh_.param<std::string>("default_objects_param", default_objects_param_, default_objects_param_);
  pnh_.param<double>("update_rate", update_rate_, update_rate_);
  pnh_.param<double>("default_command_duration", default_command_duration_, default_command_duration_);
  pnh_.param<std::string>("motion_sequences_param", motion_sequences_param_, motion_sequences_param_);
  pnh_.param<bool>("autostart_loops", autostart_loops_, autostart_loops_);
}

void ObjectCommandNode::loadMotionSequences()
{
  if (motion_sequences_param_.empty())
  {
    ROS_DEBUG_NAMED("scene_builder", "motion_sequences_param is empty; no sequences to load.");
    return;
  }

  XmlRpc::XmlRpcValue sequences_param;
  if (!pnh_.getParam(motion_sequences_param_, sequences_param))
  {
    ROS_DEBUG_STREAM_NAMED("scene_builder",
                           "No motion sequences found for parameter: " << motion_sequences_param_);
    return;
  }

  if (sequences_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM_NAMED("scene_builder",
                           "Parameter " << motion_sequences_param_ << " must be a dictionary of sequences.");
    return;
  }

  for (auto it = sequences_param.begin(); it != sequences_param.end(); ++it)
  {
    const std::string object_id = static_cast<std::string>(it->first);
    XmlRpc::XmlRpcValue& entry = it->second;

    if (!entry.hasMember("waypoints"))
    {
      ROS_WARN_STREAM_NAMED("scene_builder",
                            "Sequence for object '" << object_id << "' is missing 'waypoints'.");
      continue;
    }

    XmlRpc::XmlRpcValue& waypoints = entry["waypoints"];
    if (waypoints.getType() != XmlRpc::XmlRpcValue::TypeArray || waypoints.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("scene_builder",
                            "Waypoints for object '" << object_id << "' must be a non-empty array.");
      continue;
    }

    bool loop = false;
    if (entry.hasMember("loop"))
    {
      if (!xmlRpcToBool(entry["loop"], loop))
      {
        ROS_WARN_STREAM_NAMED("scene_builder",
                              "Field 'loop' for object '" << object_id << "' must be boolean.");
      }
    }

    std::vector<objects::WaypointCommand> sequence;
    sequence.reserve(static_cast<std::size_t>(waypoints.size()));

    for (int i = 0; i < waypoints.size(); ++i)
    {
      if (waypoints[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN_STREAM_NAMED("scene_builder",
                              "Waypoint #" << i << " for '" << object_id << "' must be a dictionary.");
        continue;
      }

      XmlRpc::XmlRpcValue& wp_entry = waypoints[i];
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;

      bool pose_parsed = false;
      if (wp_entry.hasMember("pose"))
      {
        pose_parsed = parsePose(wp_entry["pose"], pose);
      }
      else
      {
        pose_parsed = parsePose(wp_entry, pose);
      }

      if (!pose_parsed)
      {
        ROS_WARN_STREAM_NAMED("scene_builder",
                              "Cannot parse pose for waypoint #" << i << " for '" << object_id << "'.");
        continue;
      }

      double duration = default_command_duration_;
      if (wp_entry.hasMember("duration"))
      {
        if (!xmlRpcToDouble(wp_entry["duration"], duration))
        {
          ROS_WARN_STREAM_NAMED("scene_builder",
                                "Invalid 'duration' for waypoint #" << i << " of object '" << object_id << "'.");
          duration = default_command_duration_;
        }
      }

      objects::WaypointCommand cmd;
      cmd.target_pose = pose;
      cmd.duration_hint = ros::Duration(duration > 0.0 ? duration : default_command_duration_);
      sequence.push_back(cmd);
    }

    if (sequence.empty())
    {
      ROS_WARN_STREAM_NAMED("scene_builder",
                            "No valid waypoints for sequence of object '" << object_id << "'.");
      continue;
    }

    if (!autostart_loops_)
    {
      ROS_INFO_STREAM("Sequence of " << sequence.size() << " waypoints for '" << object_id
                                     << "' available (autostart disabled, use service to start).");
      continue;
    }

    manager_->setWaypointSequence(object_id, sequence, loop);
    ROS_INFO_STREAM("Loaded and started sequence of " << sequence.size() << " waypoints for '" << object_id
                                            << (loop ? "' (loop enabled)." : "'."));
  }
}

void ObjectCommandNode::setupPublishersAndSubscribers()
{
  add_object_sub_ = nh_.subscribe("add_object", 10, &ObjectCommandNode::collisionObjectCallback, this);
  waypoint_sub_ = nh_.subscribe("object_command", 50, &ObjectCommandNode::objectCommandCallback, this);
  velocity_sub_ = nh_.subscribe("object_velocity_command", 50, &ObjectCommandNode::velocityCommandCallback, this);
  animation_sub_ = nh_.subscribe("object_animation", 10, &ObjectCommandNode::animationSequenceCallback, this);

  object_list_pub_ = pnh_.advertise<moveit_msgs::CollisionObject>("objects", 10, true);

  set_motion_sequence_srv_ = pnh_.advertiseService("set_motion_sequence",
                                                    &ObjectCommandNode::handleSetMotionSequence, this);
  get_motion_sequence_srv_ = pnh_.advertiseService("get_motion_sequence",
                                                    &ObjectCommandNode::handleGetMotionSequence, this);
  clear_motion_sequence_srv_ = pnh_.advertiseService("clear_motion_sequence",
                                                      &ObjectCommandNode::handleClearMotionSequence, this);
  list_objects_srv_ = pnh_.advertiseService("list_objects",
                                             &ObjectCommandNode::handleListObjects, this);
}

void ObjectCommandNode::setupTimers()
{
  if (update_rate_ <= 0.0)
  {
    update_rate_ = 60.0;
  }

  const double period = 1.0 / update_rate_;
  update_timer_ = nh_.createTimer(ros::Duration(period), &ObjectCommandNode::updateTimerCallback, this);
}

void ObjectCommandNode::collisionObjectCallback(const moveit_msgs::CollisionObject::ConstPtr& msg)
{
  std::string error;
  if (!manager_->addObject(*msg, error))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to add object '" << msg->id << "': " << error);
    return;
  }
  ROS_INFO_STREAM("Added/updated collision object '" << msg->id << "'.");
}

void ObjectCommandNode::objectCommandCallback(const scene_builder::ObjectCommand::ConstPtr& msg)
{
  if (!validateObjectId(msg->object_id))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Received object command with empty object_id");
    return;
  }

  objects::WaypointCommand cmd;
  cmd.target_pose = msg->target_pose;
  const double duration = msg->move_duration > 0.0 ? msg->move_duration : default_command_duration_;
  cmd.duration_hint = ros::Duration(duration);

  std::string error;
  if (!manager_->queueWaypoint(msg->object_id, cmd, error))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to queue waypoint for '" << msg->object_id << "': " << error);
  }
}

void ObjectCommandNode::velocityCommandCallback(const scene_builder::ObjectVelocityCommand::ConstPtr& msg)
{
  if (!validateObjectId(msg->object_id))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Received velocity command with empty object_id");
    return;
  }

  objects::VelocityCommand cmd;
  cmd.reference_frame = msg->reference_frame;
  cmd.twist = msg->twist;
  cmd.timeout = msg->timeout > 0.0 ? ros::Duration(msg->timeout) : ros::Duration(0.0);

  std::string error;
  if (!manager_->applyVelocity(msg->object_id, cmd, error))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to apply velocity for '" << msg->object_id << "': " << error);
  }
}

void ObjectCommandNode::animationSequenceCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if (msg->poses.empty())
  {
    return;
  }

  const std::string object_id = msg->header.frame_id;
  if (!validateObjectId(object_id))
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Animation sequence requires object_id in header.frame_id");
    return;
  }

  std::string error;
  for (std::size_t i = 0; i < msg->poses.size(); ++i)
  {
    objects::WaypointCommand cmd;
    cmd.target_pose = msg->poses[i];
    cmd.duration_hint = ros::Duration(default_command_duration_);

    if (!manager_->queueWaypoint(object_id, cmd, error))
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Failed to queue animation waypoint for '" << object_id << "': " << error);
      break;
    }
  }
}

void ObjectCommandNode::updateTimerCallback(const ros::TimerEvent& event)
{
  const double dt = (event.current_real - event.last_real).toSec();
  manager_->update(event.current_real, dt);

  if (object_list_pub_.getNumSubscribers() > 0)
  {
    const auto objects = manager_->listObjects();
    for (const auto& obj : objects)
    {
      object_list_pub_.publish(obj);
    }
  }
}

bool ObjectCommandNode::handleSetMotionSequence(SetMotionSequence::Request& req,
                                                SetMotionSequence::Response& res)
{
  if (req.object_id.empty())
  {
    res.success = false;
    res.message = "Object ID is empty";
    return true;
  }

  if (req.waypoints.empty())
  {
    res.success = false;
    res.message = "Waypoints array is empty";
    return true;
  }

  if (!req.durations.empty() && req.durations.size() != req.waypoints.size())
  {
    res.success = false;
    res.message = "Waypoints and durations arrays must have the same size";
    return true;
  }

  std::vector<objects::WaypointCommand> sequence;
  sequence.reserve(req.waypoints.size());

  for (std::size_t i = 0; i < req.waypoints.size(); ++i)
  {
    objects::WaypointCommand cmd;
    cmd.target_pose = req.waypoints[i];

    double duration = default_command_duration_;
    if (!req.durations.empty() && i < req.durations.size())
    {
      duration = req.durations[i] > 0.0 ? req.durations[i] : default_command_duration_;
    }
    cmd.duration_hint = ros::Duration(duration);

    sequence.push_back(cmd);
  }

  manager_->setWaypointSequence(req.object_id, sequence, req.loop);

  res.success = true;
  res.message = "Motion sequence set successfully with " + std::to_string(sequence.size()) + " waypoints";
  ROS_INFO_STREAM("Set motion sequence for '" << req.object_id << "' with " << sequence.size()
                  << " waypoints" << (req.loop ? " (loop enabled)" : ""));

  return true;
}

bool ObjectCommandNode::handleGetMotionSequence(GetMotionSequence::Request& req,
                                                GetMotionSequence::Response& res)
{
  if (req.object_id.empty())
  {
    res.success = false;
    res.message = "Object ID is empty";
    return true;
  }

  bool is_active, is_loop;
  std::vector<objects::WaypointCommand> waypoints;
  std::size_t current_index;

  if (!manager_->getSequenceState(req.object_id, is_active, is_loop, waypoints, current_index))
  {
    res.success = false;
    res.message = "Object '" + req.object_id + "' not found";
    return true;
  }

  res.success = true;
  res.message = "OK";
  res.loop_enabled = is_loop;
  res.loop_active = is_active;
  res.current_waypoint_index = static_cast<uint32_t>(current_index);

  res.waypoints.reserve(waypoints.size());
  res.durations.reserve(waypoints.size());
  for (const auto& wp : waypoints)
  {
    res.waypoints.push_back(wp.target_pose);
    res.durations.push_back(wp.duration_hint.toSec());
  }

  return true;
}

bool ObjectCommandNode::handleClearMotionSequence(ClearMotionSequence::Request& req,
                                                  ClearMotionSequence::Response& res)
{
  if (req.object_id.empty())
  {
    res.success = false;
    res.message = "Object ID is empty";
    return true;
  }

  manager_->clearCommands(req.object_id);

  res.success = true;
  res.message = "Motion sequence cleared for '" + req.object_id + "'";
  ROS_INFO_STREAM("Cleared motion sequence for '" << req.object_id << "'");

  return true;
}

bool ObjectCommandNode::handleListObjects(ListObjects::Request& /* req */,
                                          ListObjects::Response& res)
{
  const auto objects = manager_->listObjects();

  for (const auto& obj : objects)
  {
    res.object_ids.push_back(obj.id);

    geometry_msgs::Pose pose;
    if (manager_->getObjectPosePublic(obj.id, pose))
    {
      res.poses.push_back(pose);
    }
    else
    {
      geometry_msgs::Pose identity;
      identity.orientation.w = 1.0;
      res.poses.push_back(identity);
    }

    std::string ptype = "unknown";
    if (!obj.primitives.empty())
    {
      switch (obj.primitives[0].type)
      {
        case shape_msgs::SolidPrimitive::BOX:
          ptype = "box";
          break;
        case shape_msgs::SolidPrimitive::SPHERE:
          ptype = "sphere";
          break;
        case shape_msgs::SolidPrimitive::CYLINDER:
          ptype = "cylinder";
          break;
        case shape_msgs::SolidPrimitive::CONE:
          ptype = "cone";
          break;
        default:
          ptype = "unknown";
          break;
      }
    }
    res.primitive_types.push_back(ptype);

    bool is_active, is_loop;
    std::vector<objects::WaypointCommand> waypoints;
    std::size_t current_index;
    manager_->getSequenceState(obj.id, is_active, is_loop, waypoints, current_index);
    res.loop_active.push_back(is_active && is_loop);
  }

  return true;
}

}  // namespace nodes
}  // namespace scene_builder

