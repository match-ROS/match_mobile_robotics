/**
 * @file collision_object_factory.cpp
 * @brief Implementation of the collision object factory
 */

#include "scene_builder/objects/collision_object_factory.hpp"
#include "scene_builder/core/pose_utils.hpp"

#include <shape_msgs/SolidPrimitive.h>

#include <stdexcept>

namespace scene_builder
{
namespace objects
{

moveit_msgs::CollisionObject CollisionObjectFactory::createBox(
    const std::string& id,
    const std::string& frame_id,
    const geometry_msgs::Pose& pose,
    double size_x, double size_y, double size_z)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::CollisionObject::ADD;

  // Create box primitive
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size_x;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size_y;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size_z;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(core::sanitizePose(pose));

  return obj;
}

moveit_msgs::CollisionObject CollisionObjectFactory::createSphere(
    const std::string& id,
    const std::string& frame_id,
    const geometry_msgs::Pose& pose,
    double radius)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::CollisionObject::ADD;

  // Create sphere primitive
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(core::sanitizePose(pose));

  return obj;
}

moveit_msgs::CollisionObject CollisionObjectFactory::createCylinder(
    const std::string& id,
    const std::string& frame_id,
    const geometry_msgs::Pose& pose,
    double height, double radius)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::CollisionObject::ADD;

  // Create cylinder primitive
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(core::sanitizePose(pose));

  return obj;
}

moveit_msgs::CollisionObject CollisionObjectFactory::createFromDefinition(
    const core::ObjectDefinition& definition)
{
  if (definition.primitive_type == "box")
  {
    if (definition.dimensions.size() != 3)
    {
      throw std::invalid_argument("Box requires 3 dimensions [x, y, z]");
    }
    return createBox(definition.id, definition.frame_id, definition.pose,
                     definition.dimensions[0], definition.dimensions[1], definition.dimensions[2]);
  }
  else if (definition.primitive_type == "sphere")
  {
    if (definition.dimensions.size() != 1)
    {
      throw std::invalid_argument("Sphere requires 1 dimension [radius]");
    }
    return createSphere(definition.id, definition.frame_id, definition.pose,
                        definition.dimensions[0]);
  }
  else if (definition.primitive_type == "cylinder")
  {
    if (definition.dimensions.size() != 2)
    {
      throw std::invalid_argument("Cylinder requires 2 dimensions [height, radius]");
    }
    return createCylinder(definition.id, definition.frame_id, definition.pose,
                          definition.dimensions[0], definition.dimensions[1]);
  }
  else
  {
    throw std::invalid_argument("Unsupported primitive type: " + definition.primitive_type);
  }
}

moveit_msgs::CollisionObject CollisionObjectFactory::createMoveCommand(
    const std::string& id,
    const std::string& frame_id,
    const geometry_msgs::Pose& new_pose)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::CollisionObject::MOVE;
  obj.pose = core::sanitizePose(new_pose);

  return obj;
}

moveit_msgs::CollisionObject CollisionObjectFactory::createRemoveCommand(
    const std::string& id,
    const std::string& frame_id)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::CollisionObject::REMOVE;

  return obj;
}

}  // namespace objects
}  // namespace scene_builder

