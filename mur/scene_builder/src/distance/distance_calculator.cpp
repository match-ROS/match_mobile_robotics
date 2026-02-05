/**
 * @file distance_calculator.cpp
 * @brief Implementation of distance calculation
 */

#include "scene_builder/distance/distance_calculator.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ros/console.h>
#include <ros/param.h>

namespace scene_builder
{
namespace distance
{

namespace
{
/**
 * @brief Accessor for shape index from distance data (handles API differences)
 */
struct DistanceDataShapeAccessor
{
  template <typename T>
  static auto getShapeId(const T& data, int idx, int) -> decltype(data.shape_id[idx], std::size_t())
  {
    return static_cast<std::size_t>(data.shape_id[idx]);
  }

  template <typename T>
  static auto getShapeId(const T& data, int idx, long) -> decltype(data.shape_index[idx], std::size_t())
  {
    return static_cast<std::size_t>(data.shape_index[idx]);
  }

  template <typename T>
  static std::size_t getShapeId(const T&, int, double)
  {
    static bool warned = false;
    if (!warned)
    {
      ROS_WARN_STREAM("DistanceResultsData does not expose shape indices. Using index 0.");
      warned = true;
    }
    return 0;
  }

  template <typename T>
  static std::size_t get(const T& data, int idx)
  {
    return getShapeId(data, idx, 0);
  }
};
}  // namespace

DistanceCalculator::DistanceCalculator(const std::string& move_group_name,
                                       const std::string& robot_description_param)
  : move_group_name_(move_group_name)
  , robot_description_param_(robot_description_param)
{
  configureDistanceRequest();
}

void DistanceCalculator::setPlanningSceneMonitor(
    const planning_scene_monitor::PlanningSceneMonitorPtr& monitor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planning_scene_monitor_ = monitor;
  initialized_ = (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene());
}

void DistanceCalculator::configureDistanceRequest()
{
  distance_request_.enable_nearest_points = true;
  distance_request_.enable_signed_distance = false;
  distance_request_.type = collision_detection::DistanceRequestType::ALL;
  distance_request_.max_contacts_per_body = 1;
}

void DistanceCalculator::setDistanceThreshold(double threshold)
{
  distance_request_.distance_threshold = threshold;
}

void DistanceCalculator::setMaxContactsPerBody(int max_contacts)
{
  distance_request_.max_contacts_per_body = max_contacts;
}

bool DistanceCalculator::isInitialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

std::string DistanceCalculator::getPlanningFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
  {
    return planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
  }
  return "world";
}

bool DistanceCalculator::ensureInitialized()
{
  if (initialized_)
  {
    return true;
  }

  // Try to find the robot_description parameter
  std::string resolved_description = robot_description_param_;
  if (!ros::param::has(resolved_description))
  {
    std::string search_param;
    if (ros::param::search(robot_description_param_, search_param))
    {
      resolved_description = search_param;
    }
    else
    {
      ROS_ERROR_STREAM("Parameter '" << robot_description_param_ << "' not found");
      return false;
    }
  }

  // Load robot model
  robot_model_loader::RobotModelLoader::Options options(resolved_description);
  auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(options);
  if (!loader->getModel())
  {
    ROS_ERROR_STREAM("Failed to load robot model from '" << resolved_description << "'");
    return false;
  }

  // Create PlanningSceneMonitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(loader);
  if (!planning_scene_monitor_->getPlanningScene())
  {
    ROS_ERROR("Failed to create PlanningSceneMonitor");
    planning_scene_monitor_.reset();
    return false;
  }

  // Start monitors
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  initialized_ = true;
  return true;
}

std::set<const moveit::core::LinkModel*> DistanceCalculator::createLinkFilter(
    const std::vector<std::string>& link_names,
    const moveit::core::RobotModelConstPtr& model) const
{
  std::set<const moveit::core::LinkModel*> filter;

  if (link_names.empty())
  {
    return filter;  // Empty filter means all links
  }

  for (const auto& name : link_names)
  {
    const moveit::core::LinkModel* link = model->getLinkModel(name);
    if (link)
    {
      filter.insert(link);
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Unknown link name: " << name);
    }
  }

  return filter;
}

Eigen::Vector3d DistanceCalculator::transformToWorld(
    const Eigen::Vector3d& local_point,
    collision_detection::BodyType body_type,
    const std::string& name,
    std::size_t shape_index,
    const moveit::core::RobotState& robot_state,
    const planning_scene::PlanningSceneConstPtr& scene) const
{
  if (body_type == collision_detection::BodyType::ROBOT_LINK)
  {
    const moveit::core::LinkModel* link = robot_state.getLinkModel(name);
    if (link)
    {
      const auto& shape_transforms = link->getCollisionOriginTransforms();
      Eigen::Isometry3d shape_tf = Eigen::Isometry3d::Identity();

      if (!shape_transforms.empty())
      {
        if (shape_index < shape_transforms.size())
        {
          shape_tf = shape_transforms[shape_index];
        }
        else
        {
          shape_tf = shape_transforms.front();
        }
      }

      Eigen::Vector3d point_link = shape_tf * local_point;
      const Eigen::Isometry3d& T_world_link = robot_state.getGlobalLinkTransform(link);
      return T_world_link * point_link;
    }
  }
  else if (body_type == collision_detection::BodyType::ROBOT_ATTACHED)
  {
    const moveit::core::AttachedBody* attached = robot_state.getAttachedBody(name);
    if (attached)
    {
      const auto& transforms = attached->getGlobalCollisionBodyTransforms();
      if (shape_index < transforms.size())
      {
        return transforms[shape_index] * local_point;
      }
      if (!transforms.empty())
      {
        return transforms.front() * local_point;
      }
    }
  }
  else if (body_type == collision_detection::BodyType::WORLD_OBJECT)
  {
    bool found = false;
    const Eigen::Isometry3d& T_world_object = scene->getWorld()->getTransform(name, found);
    if (found)
    {
      auto object = scene->getWorld()->getObject(name);
      if (object && shape_index < object->shape_poses_.size())
      {
        return T_world_object * object->shape_poses_[shape_index] * local_point;
      }
      if (object && !object->shape_poses_.empty())
      {
        return T_world_object * object->shape_poses_.front() * local_point;
      }
      return T_world_object * local_point;
    }
  }

  return local_point;
}

std::vector<DistanceResult> DistanceCalculator::computeAllDistances(
    const std::vector<std::string>& link_filter)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DistanceResult> results;

  if (!ensureInitialized())
  {
    return results;
  }

  planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
  if (!scene_ro)
  {
    ROS_ERROR_THROTTLE(1.0, "Cannot get planning scene lock");
    return results;
  }

  const planning_scene::PlanningSceneConstPtr& scene = scene_ro;
  const moveit::core::RobotModelConstPtr& model = scene->getRobotModel();
  moveit::core::RobotState robot_state = scene->getCurrentState();

  // Configure request - use base settings without group filtering
  // to compute distances between all robot links and obstacles
  collision_detection::DistanceRequest request = distance_request_;
  request.acm = &scene->getAllowedCollisionMatrix();

  auto filter_set = createLinkFilter(link_filter, model);
  if (!filter_set.empty())
  {
    request.active_components_only = &filter_set;
  }

  // Execute query
  collision_detection::DistanceResult dist_result;
  scene->getCollisionEnv()->distanceRobot(request, dist_result, robot_state);

  // Process results
  for (const auto& entry : dist_result.distances)
  {
    for (const auto& data : entry.second)
    {
      // Identify robot and object indices
      int robot_idx = -1;
      int object_idx = -1;

      for (int i = 0; i < 2; ++i)
      {
        if (data.body_types[i] == collision_detection::BodyType::ROBOT_LINK ||
            data.body_types[i] == collision_detection::BodyType::ROBOT_ATTACHED)
        {
          if (robot_idx == -1) robot_idx = i;
        }
        else if (data.body_types[i] == collision_detection::BodyType::WORLD_OBJECT)
        {
          if (object_idx == -1) object_idx = i;
        }
      }

      // Handle cases where indices weren't set
      if (robot_idx == -1 && object_idx != -1) robot_idx = 1 - object_idx;
      if (object_idx == -1 && robot_idx != -1) object_idx = 1 - robot_idx;
      if (robot_idx == -1) robot_idx = 0;
      if (object_idx == -1) object_idx = 1 - robot_idx;

      // Transform points to world frame
      const std::size_t robot_shape_idx = DistanceDataShapeAccessor::get(data, robot_idx);
      const std::size_t object_shape_idx = DistanceDataShapeAccessor::get(data, object_idx);

      Eigen::Vector3d robot_point = transformToWorld(
          data.nearest_points[robot_idx],
          data.body_types[robot_idx],
          data.link_names[robot_idx],
          robot_shape_idx,
          robot_state,
          scene);

      Eigen::Vector3d object_point = transformToWorld(
          data.nearest_points[object_idx],
          data.body_types[object_idx],
          data.link_names[object_idx],
          object_shape_idx,
          robot_state,
          scene);

      DistanceResult result;
      result.distance = data.distance;
      result.link_name = data.link_names[robot_idx];
      result.object_id = data.link_names[object_idx];
      result.robot_point = robot_point;
      result.object_point = object_point;
      result.distance_vector = robot_point - object_point;

      // Get object pose in world frame
      if (data.body_types[object_idx] == collision_detection::BodyType::WORLD_OBJECT)
      {
        bool found = false;
        result.object_pose = scene->getWorld()->getTransform(data.link_names[object_idx], found);
        if (!found)
        {
          result.object_pose = Eigen::Isometry3d::Identity();
        }
      }
      else
      {
        result.object_pose = Eigen::Isometry3d::Identity();
      }

      results.push_back(result);
    }
  }

  return results;
}

std::optional<DistanceResult> DistanceCalculator::computeMinimumDistance(
    const std::vector<std::string>& link_filter)
{
  auto all_distances = computeAllDistances(link_filter);

  if (all_distances.empty())
  {
    return std::nullopt;
  }

  // Find minimum
  auto min_it = std::min_element(all_distances.begin(), all_distances.end(),
      [](const DistanceResult& a, const DistanceResult& b) {
        return a.distance < b.distance;
      });

  return *min_it;
}

}  // namespace distance
}  // namespace scene_builder

