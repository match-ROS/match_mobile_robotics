/**
 * @file yaml_parser.cpp
 * @brief Implementation of YAML parsing utilities
 */

#include "scene_builder/core/yaml_parser.hpp"
#include "scene_builder/core/pose_utils.hpp"

#include <ros/console.h>

namespace scene_builder
{
namespace core
{

bool YamlParser::xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double& result)
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

bool YamlParser::xmlRpcToBool(const XmlRpc::XmlRpcValue& value, bool& result)
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

bool YamlParser::readVector(XmlRpc::XmlRpcValue& array,
                            std::vector<double>& out,
                            std::size_t expected_size)
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

bool YamlParser::parsePose(XmlRpc::XmlRpcValue& entry, geometry_msgs::Pose& pose_out)
{
  geometry_msgs::Pose pose = identityPose();

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

bool YamlParser::parseObjectEntry(const std::string& id,
                                  XmlRpc::XmlRpcValue& entry,
                                  const std::string& default_frame,
                                  ObjectDefinition& definition)
{
  definition.id = id;

  // Parse frame_id
  if (entry.hasMember("frame_id"))
  {
    definition.frame_id = static_cast<std::string>(entry["frame_id"]);
  }
  if (definition.frame_id.empty())
  {
    definition.frame_id = default_frame;
  }

  // Parse primitive
  if (!entry.hasMember("primitive"))
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Object '" << id << "' is missing 'primitive' definition");
    return false;
  }

  XmlRpc::XmlRpcValue& primitive = entry["primitive"];
  if (!primitive.hasMember("type") || !primitive.hasMember("dimensions"))
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Primitive definition incomplete for object '" << id << "'");
    return false;
  }

  definition.primitive_type = static_cast<std::string>(primitive["type"]);

  // Parse dimensions
  if (!readVector(primitive["dimensions"], definition.dimensions, 0))
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Failed to parse dimensions for object '" << id << "'");
    return false;
  }

  // Validate dimensions based on primitive type
  if (definition.primitive_type == "box" && definition.dimensions.size() != 3)
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Box dimensions must be array of size 3 for object '" << id << "'");
    return false;
  }
  else if (definition.primitive_type == "sphere" && definition.dimensions.size() != 1)
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Sphere dimensions must be array of size 1 (radius) for object '" << id << "'");
    return false;
  }
  else if (definition.primitive_type == "cylinder" && definition.dimensions.size() != 2)
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Cylinder dimensions must be array of size 2 (height, radius) for object '" << id << "'");
    return false;
  }

  // Parse pose
  definition.pose = identityPose();
  if (entry.hasMember("pose"))
  {
    XmlRpc::XmlRpcValue& pose_entry = entry["pose"];
    if (!parsePose(pose_entry, definition.pose))
    {
      ROS_ERROR_STREAM_NAMED("yaml_parser", "Failed to parse pose for object '" << id << "'");
      return false;
    }
  }

  return true;
}

std::vector<ObjectDefinition> YamlParser::parseObjects(
    const ros::NodeHandle& nh,
    const std::string& param_name,
    const std::string& default_frame)
{
  std::vector<ObjectDefinition> result;

  XmlRpc::XmlRpcValue yaml_objects;
  if (!nh.getParam(param_name, yaml_objects))
  {
    ROS_DEBUG_STREAM_NAMED("yaml_parser", "No objects defined under param: " << param_name);
    return result;
  }

  if (yaml_objects.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Expected a dictionary for objects at param: " << param_name);
    return result;
  }

  for (auto it = yaml_objects.begin(); it != yaml_objects.end(); ++it)
  {
    const std::string& object_id = it->first;
    XmlRpc::XmlRpcValue& entry = it->second;

    ObjectDefinition definition;
    if (parseObjectEntry(object_id, entry, default_frame, definition))
    {
      result.push_back(definition);
    }
  }

  return result;
}

std::vector<MotionSequenceDefinition> YamlParser::parseMotionSequences(
    const ros::NodeHandle& nh,
    const std::string& param_name,
    double default_duration)
{
  std::vector<MotionSequenceDefinition> result;

  if (param_name.empty())
  {
    ROS_DEBUG_NAMED("yaml_parser", "Motion sequences param name is empty; nothing to load.");
    return result;
  }

  XmlRpc::XmlRpcValue sequences_param;
  if (!nh.getParam(param_name, sequences_param))
  {
    ROS_DEBUG_STREAM_NAMED("yaml_parser", "No motion sequences found at param: " << param_name);
    return result;
  }

  if (sequences_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM_NAMED("yaml_parser", "Motion sequences param must be a dictionary: " << param_name);
    return result;
  }

  for (auto it = sequences_param.begin(); it != sequences_param.end(); ++it)
  {
    const std::string object_id = static_cast<std::string>(it->first);
    XmlRpc::XmlRpcValue& entry = it->second;

    if (!entry.hasMember("waypoints"))
    {
      ROS_WARN_STREAM_NAMED("yaml_parser", "Sequence for '" << object_id << "' is missing 'waypoints'");
      continue;
    }

    XmlRpc::XmlRpcValue& waypoints = entry["waypoints"];
    if (waypoints.getType() != XmlRpc::XmlRpcValue::TypeArray || waypoints.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("yaml_parser", "Waypoints for '" << object_id << "' must be a non-empty array");
      continue;
    }

    MotionSequenceDefinition sequence_def;
    sequence_def.object_id = object_id;
    sequence_def.loop = false;

    // Parse loop flag
    if (entry.hasMember("loop"))
    {
      if (!xmlRpcToBool(entry["loop"], sequence_def.loop))
      {
        ROS_WARN_STREAM_NAMED("yaml_parser", "'loop' field for '" << object_id << "' must be boolean");
      }
    }

    // Parse waypoints
    for (int i = 0; i < waypoints.size(); ++i)
    {
      if (waypoints[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN_STREAM_NAMED("yaml_parser", "Waypoint #" << i << " for '" << object_id << "' must be a dictionary");
        continue;
      }

      XmlRpc::XmlRpcValue& wp_entry = waypoints[i];
      WaypointDefinition waypoint_def;
      waypoint_def.pose = identityPose();
      waypoint_def.duration = default_duration;

      // Try to parse pose (either nested under "pose" or directly)
      bool pose_parsed = false;
      if (wp_entry.hasMember("pose"))
      {
        pose_parsed = parsePose(wp_entry["pose"], waypoint_def.pose);
      }
      else
      {
        pose_parsed = parsePose(wp_entry, waypoint_def.pose);
      }

      if (!pose_parsed)
      {
        ROS_WARN_STREAM_NAMED("yaml_parser", "Failed to parse pose for waypoint #" << i << " of '" << object_id << "'");
        continue;
      }

      // Parse duration
      if (wp_entry.hasMember("duration"))
      {
        double duration = default_duration;
        if (xmlRpcToDouble(wp_entry["duration"], duration))
        {
          waypoint_def.duration = duration > 0.0 ? duration : default_duration;
        }
      }

      sequence_def.waypoints.push_back(waypoint_def);
    }

    if (sequence_def.waypoints.empty())
    {
      ROS_WARN_STREAM_NAMED("yaml_parser", "No valid waypoints for sequence of '" << object_id << "'");
      continue;
    }

    result.push_back(sequence_def);
  }

  return result;
}

}  // namespace core
}  // namespace scene_builder

