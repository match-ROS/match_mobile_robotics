#include "cartesian_velocity_controller/map3d/planning_scene_sphere_reader.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>

#include <Eigen/Geometry>

namespace cartesian_velocity_controller::map3d
{

PlanningSceneSphereReader::PlanningSceneSphereReader(
    const Map3DConfig& cfg,
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
    tf2_ros::Buffer* tf_buffer)
  : cfg_(cfg)
  , psm_(std::move(psm))
  , tf_buffer_(tf_buffer)
{
}

std::string PlanningSceneSphereReader::planningFrame() const
{
  if (!psm_)
    return std::string{};
  auto scene = psm_->getPlanningScene();
  if (!scene)
    return std::string{};
  return scene->getPlanningFrame();
}

std::vector<SphereObstacle> PlanningSceneSphereReader::readSpheres(const std::string& target_frame)
{
  std::vector<SphereObstacle> out;
  if (!psm_)
    return out;

  planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
  const planning_scene::PlanningSceneConstPtr& scene = ls;
  if (!scene)
    return out;

  const std::string scene_frame = scene->getPlanningFrame();

  const collision_detection::WorldConstPtr& world = scene->getWorld();
  if (!world)
    return out;

  const std::vector<std::string>& ids = world->getObjectIds();
  out.reserve(ids.size());

  Eigen::Isometry3d T_target_scene = Eigen::Isometry3d::Identity();
  const bool needs_tf = (!target_frame.empty() && target_frame != scene_frame);
  if (needs_tf)
  {
    if (!tf_buffer_)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "map3d", "No TF buffer available to transform PlanningScene objects");
      return out;
    }
    try
    {
      geometry_msgs::TransformStamped tf =
          tf_buffer_->lookupTransform(target_frame, scene_frame, ros::Time(0), ros::Duration(0.05));
      T_target_scene = tf2::transformToEigen(tf);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "map3d", "TF error planning_scene->%s: %s",
                              target_frame.c_str(), ex.what());
      return out;
    }
  }

  for (const auto& id : ids)
  {
    const collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj)
      continue;

    const auto& shapes = obj->shapes_;
    const auto& poses = obj->shape_poses_;
    const std::size_t n = std::min(shapes.size(), poses.size());
    for (std::size_t i = 0; i < n; ++i)
    {
      const shapes::ShapeConstPtr& s = shapes[i];
      if (!s)
        continue;

      if (s->type != shapes::ShapeType::SPHERE)
        continue;

      const auto* sphere = dynamic_cast<const shapes::Sphere*>(s.get());
      if (!sphere)
        continue;

      // Sphere pose is in scene planning frame
      Eigen::Isometry3d T_scene_obj = poses[i];
      Eigen::Vector3d c_scene = T_scene_obj.translation();
      Eigen::Vector3d c_target = needs_tf ? (T_target_scene * c_scene) : c_scene;

      SphereObstacle o;
      o.id = id;
      o.center = c_target;
      o.radius = sphere->radius;
      out.push_back(o);
    }
  }

  return out;
}

}  // namespace cartesian_velocity_controller::map3d

