/**
 * @file yaml_parser.hpp
 * @brief YAML parsing utilities for scene objects and motion sequences
 *
 * Provides functions to parse collision object definitions and motion
 * sequences from ROS parameter server YAML configurations.
 */

#ifndef SCENE_BUILDER_CORE_YAML_PARSER_HPP
#define SCENE_BUILDER_CORE_YAML_PARSER_HPP

#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>

#include <string>
#include <vector>

namespace scene_builder
{
namespace core
{

/**
 * @brief Definition of a collision object parsed from YAML
 */
struct ObjectDefinition
{
  std::string id;                      ///< Unique object identifier
  std::string frame_id;                ///< Reference frame for the object
  std::string primitive_type;          ///< Primitive type: "box", "sphere", "cylinder"
  std::vector<double> dimensions;      ///< Dimensions (type-dependent)
  geometry_msgs::Pose pose;            ///< Object pose in frame_id
};

/**
 * @brief Definition of a single waypoint in a motion sequence
 */
struct WaypointDefinition
{
  geometry_msgs::Pose pose;            ///< Target pose for this waypoint
  double duration;                     ///< Duration to reach this waypoint (seconds)
};

/**
 * @brief Definition of a complete motion sequence for an object
 */
struct MotionSequenceDefinition
{
  std::string object_id;               ///< ID of the object this sequence applies to
  bool loop;                           ///< Whether to loop the sequence
  std::vector<WaypointDefinition> waypoints;  ///< Waypoints in the sequence
};

/**
 * @brief YAML parsing utilities for scene builder
 *
 * Static class providing methods to parse object definitions and
 * motion sequences from ROS parameter server.
 *
 * Expected YAML format for objects:
 * @code{.yaml}
 * default_objects:
 *   object_name:
 *     frame_id: "world"
 *     primitive:
 *       type: "box"  # or "sphere", "cylinder"
 *       dimensions: [0.1, 0.1, 0.1]  # box: [x, y, z], sphere: [radius], cylinder: [height, radius]
 *     pose:
 *       position: [0.5, 0.0, 0.5]
 *       orientation: [0, 0, 0, 1]  # [x, y, z, w]
 * @endcode
 *
 * Expected YAML format for motion sequences:
 * @code{.yaml}
 * motion_sequences:
 *   object_name:
 *     loop: true
 *     waypoints:
 *       - position: [0.5, 0.0, 0.5]
 *         orientation: [0, 0, 0, 1]
 *         duration: 1.0
 *       - position: [0.5, 0.5, 0.5]
 *         orientation: [0, 0, 0, 1]
 *         duration: 1.0
 * @endcode
 */
class YamlParser
{
public:
  /**
   * @brief Parses object definitions from a ROS parameter
   * @param nh NodeHandle to search for parameter
   * @param param_name Name of the parameter containing object definitions
   * @param default_frame Default frame_id to use if not specified in YAML
   * @return Vector of parsed object definitions
   *
   * Returns an empty vector if the parameter doesn't exist or is invalid.
   */
  static std::vector<ObjectDefinition> parseObjects(
      const ros::NodeHandle& nh,
      const std::string& param_name,
      const std::string& default_frame = "world");

  /**
   * @brief Parses motion sequence definitions from a ROS parameter
   * @param nh NodeHandle to search for parameter
   * @param param_name Name of the parameter containing motion sequences
   * @param default_duration Default duration for waypoints without explicit duration
   * @return Vector of parsed motion sequence definitions
   *
   * Returns an empty vector if the parameter doesn't exist or is invalid.
   */
  static std::vector<MotionSequenceDefinition> parseMotionSequences(
      const ros::NodeHandle& nh,
      const std::string& param_name,
      double default_duration = 0.5);

  /**
   * @brief Parses a single object entry from XmlRpcValue
   * @param id Object identifier
   * @param entry XmlRpcValue containing object definition
   * @param default_frame Default frame_id to use if not specified
   * @param[out] definition Parsed object definition
   * @return true if parsing succeeded, false otherwise
   */
  static bool parseObjectEntry(
      const std::string& id,
      XmlRpc::XmlRpcValue& entry,
      const std::string& default_frame,
      ObjectDefinition& definition);

  /**
   * @brief Parses a pose from XmlRpcValue
   * @param entry XmlRpcValue containing "position" and "orientation" arrays
   * @param[out] pose_out Parsed pose
   * @return true if parsing succeeded, false otherwise
   */
  static bool parsePose(XmlRpc::XmlRpcValue& entry, geometry_msgs::Pose& pose_out);

  /**
   * @brief Converts XmlRpcValue to double (handles int and double types)
   * @param value XmlRpcValue to convert
   * @param[out] result Converted double value
   * @return true if conversion succeeded, false otherwise
   */
  static bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double& result);

  /**
   * @brief Converts XmlRpcValue to bool (handles bool and int types)
   * @param value XmlRpcValue to convert
   * @param[out] result Converted bool value
   * @return true if conversion succeeded, false otherwise
   */
  static bool xmlRpcToBool(const XmlRpc::XmlRpcValue& value, bool& result);

  /**
   * @brief Reads an array of doubles from XmlRpcValue
   * @param array XmlRpcValue array to read
   * @param[out] out Vector to store results
   * @param expected_size Expected array size (0 = any size)
   * @return true if reading succeeded, false otherwise
   */
  static bool readVector(XmlRpc::XmlRpcValue& array,
                         std::vector<double>& out,
                         std::size_t expected_size = 0);
};

}  // namespace core
}  // namespace scene_builder

#endif  // SCENE_BUILDER_CORE_YAML_PARSER_HPP

