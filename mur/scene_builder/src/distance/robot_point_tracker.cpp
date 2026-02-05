/**
 * @file robot_point_tracker.cpp
 * @brief Implementation of the RobotPointTracker class
 */

#include "scene_builder/distance/robot_point_tracker.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>

#include <ros/console.h>

#include <cmath>

namespace scene_builder
{
namespace distance
{

namespace
{
/**
 * @brief Computes the characteristic radius of a collision object
 * 
 * For each shape in the object, computes a bounding radius and returns
 * the maximum across all shapes. This represents the maximum distance
 * from the object center to any point on its surface.
 */
double computeCharacteristicRadius(const collision_detection::World::ObjectConstPtr& object)
{
  if (!object || object->shapes_.empty())
  {
    return 0.0;
  }

  double max_radius = 0.0;

  for (size_t i = 0; i < object->shapes_.size(); ++i)
  {
    const shapes::ShapeConstPtr& shape = object->shapes_[i];
    if (!shape)
    {
      continue;
    }

    double shape_radius = 0.0;

    switch (shape->type)
    {
      case shapes::SPHERE:
      {
        const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(shape.get());
        shape_radius = sphere->radius;
        break;
      }
      case shapes::BOX:
      {
        const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
        // Half-diagonal of the box (distance from center to corner)
        shape_radius = 0.5 * std::sqrt(box->size[0] * box->size[0] +
                                        box->size[1] * box->size[1] +
                                        box->size[2] * box->size[2]);
        break;
      }
      case shapes::CYLINDER:
      {
        const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
        // Distance from center to edge of top/bottom circle
        shape_radius = std::sqrt(cylinder->radius * cylinder->radius +
                                  (cylinder->length / 2.0) * (cylinder->length / 2.0));
        break;
      }
      case shapes::CONE:
      {
        const shapes::Cone* cone = static_cast<const shapes::Cone*>(shape.get());
        // Use the larger of: base radius or half-height
        shape_radius = std::max(cone->radius, cone->length / 2.0);
        break;
      }
      case shapes::MESH:
      {
        const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
        // Compute max distance from origin to any vertex
        for (unsigned int v = 0; v < mesh->vertex_count; ++v)
        {
          double vx = mesh->vertices[3 * v];
          double vy = mesh->vertices[3 * v + 1];
          double vz = mesh->vertices[3 * v + 2];
          double dist = std::sqrt(vx * vx + vy * vy + vz * vz);
          shape_radius = std::max(shape_radius, dist);
        }
        break;
      }
      default:
        // Unknown shape type, use 0
        break;
    }

    // Consider shape's local transform within the object
    if (i < object->shape_poses_.size())
    {
      const Eigen::Isometry3d& shape_pose = object->shape_poses_[i];
      double offset = shape_pose.translation().norm();
      shape_radius += offset;
    }

    max_radius = std::max(max_radius, shape_radius);
  }

  return max_radius;
}
}  // anonymous namespace

RobotPointTracker::RobotPointTracker(const RobotPointTrackerConfig& config)
  : config_(config)
{
}

bool RobotPointTracker::loadFromParameter(const ros::NodeHandle& nh, const std::string& param_name)
{
  XmlRpc::XmlRpcValue points_param;
  if (!nh.getParam(param_name, points_param))
  {
    ROS_DEBUG_NAMED("robot_point_tracker", "Parameter '%s' not found", param_name.c_str());
    return false;
  }

  if (points_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_WARN_NAMED("robot_point_tracker", "Parameter '%s' is not a struct", param_name.c_str());
    return false;
  }

  int loaded_count = 0;
  for (auto it = points_param.begin(); it != points_param.end(); ++it)
  {
    const std::string& point_name = it->first;
    XmlRpc::XmlRpcValue& point_data = it->second;

    if (point_data.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN_NAMED("robot_point_tracker", "Point '%s' is not a struct, skipping", point_name.c_str());
      continue;
    }

    // Link name is required
    if (!point_data.hasMember("link"))
    {
      ROS_WARN_NAMED("robot_point_tracker", "Point '%s' has no 'link' field, skipping", point_name.c_str());
      continue;
    }

    RobotPointConfig config;
    config.name = point_name;
    config.link_name = static_cast<std::string>(point_data["link"]);

    // Offset is optional
    if (point_data.hasMember("offset"))
    {
      XmlRpc::XmlRpcValue& offset = point_data["offset"];
      if (offset.getType() == XmlRpc::XmlRpcValue::TypeArray && offset.size() >= 3)
      {
        config.offset.x() = static_cast<double>(offset[0]);
        config.offset.y() = static_cast<double>(offset[1]);
        config.offset.z() = static_cast<double>(offset[2]);
      }
      else
      {
        ROS_WARN_NAMED("robot_point_tracker", "Point '%s' has invalid offset format, using [0,0,0]",
                       point_name.c_str());
      }
    }

    addPoint(config);
    ++loaded_count;
    ROS_INFO_NAMED("robot_point_tracker", "Loaded point of interest '%s' on link '%s' with offset [%.3f, %.3f, %.3f]",
                   config.name.c_str(), config.link_name.c_str(),
                   config.offset.x(), config.offset.y(), config.offset.z());
  }

  return loaded_count > 0;
}

void RobotPointTracker::addPoint(const RobotPointConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  InternalPointState state;
  state.config = config;
  state.initialized = false;
  points_[config.name] = state;
}

void RobotPointTracker::removePoint(const std::string& name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.erase(name);
}

void RobotPointTracker::clearPoints()
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear();
}

std::vector<RobotPointConfig> RobotPointTracker::getPointConfigs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<RobotPointConfig> configs;
  configs.reserve(points_.size());
  for (const auto& pair : points_)
  {
    configs.push_back(pair.second.config);
  }
  return configs;
}

void RobotPointTracker::update(const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor,
                                const ros::Time& current_time)
{
  if (!scene_monitor)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor);
  if (!scene)
  {
    return;
  }

  const moveit::core::RobotState& robot_state = scene->getCurrentState();
  const moveit::core::RobotModelConstPtr& robot_model = robot_state.getRobotModel();

  for (auto& pair : points_)
  {
    InternalPointState& state = pair.second;
    const RobotPointConfig& config = state.config;

    // Get link
    const moveit::core::LinkModel* link = robot_model->getLinkModel(config.link_name);
    if (!link)
    {
      ROS_WARN_THROTTLE_NAMED(5.0, "robot_point_tracker",
                              "Link '%s' not found for point '%s'",
                              config.link_name.c_str(), config.name.c_str());
      continue;
    }

    // Get link transform in world frame
    const Eigen::Isometry3d& link_transform = robot_state.getGlobalLinkTransform(link);

    // Compute point position in world frame (link origin + offset in link frame)
    Eigen::Vector3d current_position = link_transform * config.offset;

    // Compute velocity using finite differences
    if (state.initialized)
    {
      double dt = (current_time - state.last_update_time).toSec();

      if (dt >= config_.min_dt && dt <= config_.max_dt)
      {
        Eigen::Vector3d raw_velocity = (current_position - state.last_position) / dt;
        state.filtered_velocity = filterVelocity(state.filtered_velocity, raw_velocity, config_.velocity_filter_alpha);
      }
      else if (dt > config_.max_dt)
      {
        // Time gap too large, reset velocity estimate
        state.filtered_velocity = Eigen::Vector3d::Zero();
      }
      // If dt < min_dt, keep previous velocity estimate
    }

    state.last_position = current_position;
    state.last_update_time = current_time;
    state.initialized = true;
  }
}

RobotPointState RobotPointTracker::getPointState(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  RobotPointState result;
  auto it = points_.find(name);
  if (it == points_.end())
  {
    result.valid = false;
    return result;
  }

  const InternalPointState& state = it->second;
  result.name = state.config.name;
  result.link_name = state.config.link_name;
  result.position = state.last_position;
  result.velocity = state.filtered_velocity;
  result.valid = state.initialized;

  return result;
}

std::map<std::string, RobotPointState> RobotPointTracker::getAllPointStates() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<std::string, RobotPointState> result;
  for (const auto& pair : points_)
  {
    const InternalPointState& state = pair.second;
    RobotPointState point_state;
    point_state.name = state.config.name;
    point_state.link_name = state.config.link_name;
    point_state.position = state.last_position;
    point_state.velocity = state.filtered_velocity;
    point_state.valid = state.initialized;
    result[pair.first] = point_state;
  }

  return result;
}

std::vector<PointToObjectDistance> RobotPointTracker::computeDistancesToObjects(
    const planning_scene_monitor::PlanningSceneMonitorPtr& scene_monitor) const
{
  std::vector<PointToObjectDistance> results;

  if (!scene_monitor)
  {
    return results;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor);
  if (!scene)
  {
    return results;
  }

  // Get all collision objects in the world
  const collision_detection::WorldConstPtr& world = scene->getWorld();
  std::vector<std::string> object_ids = world->getObjectIds();

  // For each tracked point, compute distance to each object
  for (const auto& point_pair : points_)
  {
    const InternalPointState& state = point_pair.second;

    if (!state.initialized)
    {
      continue;
    }

    for (const std::string& object_id : object_ids)
    {
      const collision_detection::World::ObjectConstPtr& object = world->getObject(object_id);
      if (!object)
      {
        continue;
      }

      // Get object pose (this is the pose of the object's origin/center)
      const Eigen::Isometry3d& object_pose = object->pose_;
      Eigen::Vector3d object_center = object_pose.translation();

      // Compute distance vector from point to object center
      Eigen::Vector3d distance_vector = object_center - state.last_position;
      double distance = distance_vector.norm();

      PointToObjectDistance dist;
      dist.point_name = state.config.name;
      dist.link_name = state.config.link_name;
      dist.point_position = state.last_position;
      dist.point_velocity = state.filtered_velocity;
      dist.object_id = object_id;
      dist.distance_vector = distance_vector;
      dist.distance = distance;
      dist.object_characteristic_radius = computeCharacteristicRadius(object);

      results.push_back(dist);
    }
  }

  return results;
}

void RobotPointTracker::setConfig(const RobotPointTrackerConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

void RobotPointTracker::resetVelocities()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& pair : points_)
  {
    pair.second.filtered_velocity = Eigen::Vector3d::Zero();
    pair.second.initialized = false;
  }
}

Eigen::Vector3d RobotPointTracker::filterVelocity(const Eigen::Vector3d& current_filtered,
                                                   const Eigen::Vector3d& new_measurement,
                                                   double alpha) const
{
  // Simple exponential moving average (low-pass filter)
  // alpha = 1.0 means no filtering, alpha = 0.0 means infinite filtering
  return alpha * new_measurement + (1.0 - alpha) * current_filtered;
}

}  // namespace distance
}  // namespace scene_builder
