#pragma once

#include "cartesian_velocity_controller/map3d/map3d_types.hpp"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

namespace planning_scene_monitor
{
class PlanningSceneMonitor;
}

namespace tf2_ros
{
class Buffer;
}

namespace cartesian_velocity_controller::map3d
{

class PlanningSceneSphereReader
{
public:
  PlanningSceneSphereReader(const Map3DConfig& cfg,
                            std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                            tf2_ros::Buffer* tf_buffer);

  // Reads all collision objects from the PlanningScene and returns only SPHERE primitives,
  // transformed in `target_frame` if needed.
  std::vector<SphereObstacle> readSpheres(const std::string& target_frame);

  std::string planningFrame() const;

private:
  Map3DConfig cfg_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  tf2_ros::Buffer* tf_buffer_{nullptr};
};

}  // namespace cartesian_velocity_controller::map3d

