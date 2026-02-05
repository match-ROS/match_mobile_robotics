/**
 * @file collision_object_factory.hpp
 * @brief Factory for creating MoveIt collision objects
 *
 * Provides static methods to create collision objects of various
 * primitive types (box, sphere, cylinder) and move/remove commands.
 */

#ifndef SCENE_BUILDER_OBJECTS_COLLISION_OBJECT_FACTORY_HPP
#define SCENE_BUILDER_OBJECTS_COLLISION_OBJECT_FACTORY_HPP

#include "scene_builder/core/yaml_parser.hpp"

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

#include <string>

namespace scene_builder
{
namespace objects
{

/**
 * @brief Factory class for creating collision objects
 *
 * Provides static methods to create collision objects with
 * various primitive shapes. All methods are thread-safe.
 *
 * Usage:
 * @code
 * // Create a box
 * auto box = CollisionObjectFactory::createBox("my_box", "world", pose, 0.1, 0.2, 0.3);
 *
 * // Create a sphere
 * auto sphere = CollisionObjectFactory::createSphere("my_sphere", "world", pose, 0.1);
 *
 * // Create from YAML definition
 * auto obj = CollisionObjectFactory::createFromDefinition(definition);
 * @endcode
 */
class CollisionObjectFactory
{
public:
  /**
   * @brief Creates a box collision object
   * @param id Unique identifier for the object
   * @param frame_id Reference frame for the object
   * @param pose Pose of the object in frame_id
   * @param size_x Size along X axis (meters)
   * @param size_y Size along Y axis (meters)
   * @param size_z Size along Z axis (meters)
   * @return Configured CollisionObject message
   */
  static moveit_msgs::CollisionObject createBox(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double size_x, double size_y, double size_z);

  /**
   * @brief Creates a sphere collision object
   * @param id Unique identifier for the object
   * @param frame_id Reference frame for the object
   * @param pose Pose of the object in frame_id
   * @param radius Sphere radius (meters)
   * @return Configured CollisionObject message
   */
  static moveit_msgs::CollisionObject createSphere(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double radius);

  /**
   * @brief Creates a cylinder collision object
   * @param id Unique identifier for the object
   * @param frame_id Reference frame for the object
   * @param pose Pose of the object in frame_id
   * @param height Cylinder height (meters)
   * @param radius Cylinder radius (meters)
   * @return Configured CollisionObject message
   */
  static moveit_msgs::CollisionObject createCylinder(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double height, double radius);

  /**
   * @brief Creates a collision object from a parsed YAML definition
   * @param definition Object definition from YAML parser
   * @return Configured CollisionObject message
   * @throws std::invalid_argument if primitive type is not supported
   *
   * Supported primitive types: "box", "sphere", "cylinder"
   */
  static moveit_msgs::CollisionObject createFromDefinition(
      const core::ObjectDefinition& definition);

  /**
   * @brief Creates a MOVE command for an existing object
   * @param id Object identifier
   * @param frame_id Reference frame
   * @param new_pose New pose for the object
   * @return CollisionObject message with MOVE operation
   *
   * Use this to update the pose of an existing object in the scene.
   */
  static moveit_msgs::CollisionObject createMoveCommand(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& new_pose);

  /**
   * @brief Creates a REMOVE command for an existing object
   * @param id Object identifier
   * @param frame_id Reference frame
   * @return CollisionObject message with REMOVE operation
   *
   * Use this to remove an object from the scene.
   */
  static moveit_msgs::CollisionObject createRemoveCommand(
      const std::string& id,
      const std::string& frame_id);
};

}  // namespace objects
}  // namespace scene_builder

#endif  // SCENE_BUILDER_OBJECTS_COLLISION_OBJECT_FACTORY_HPP

