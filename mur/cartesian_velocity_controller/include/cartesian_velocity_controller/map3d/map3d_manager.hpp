#pragma once

#include "cartesian_velocity_controller/map3d/edt_calculator.hpp"
#include "cartesian_velocity_controller/map3d/map3d_types.hpp"
#include "cartesian_velocity_controller/map3d/planning_scene_sphere_reader.hpp"
#include "cartesian_velocity_controller/map3d/voxel_grid_3d.hpp"
#include "cartesian_velocity_controller/map3d/voxelizer_sphere.hpp"

#include <moveit_msgs/GetPlanningScene.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cartesian_velocity_controller/QueryMap3D.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace planning_scene_monitor
{
class PlanningSceneMonitor;
}

namespace cartesian_velocity_controller::map3d
{

class Map3DManager
{
public:
  Map3DManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~Map3DManager();

  void start();
  void stop();

  Map3DConfig getConfig() const;

  // Query distance field at a world-frame point (world_frame -> map frame internally).
  // - distance returned is to inflated obstacles surface (meters)
  // - gradient is normalized, points towards free space, returned in world_frame
  QueryResult queryWorld(const Eigen::Vector3d& p_world, const std::string& world_frame) const;

  MapMetadata getLatestMetadata() const;

  // Runtime configuration setters for dynamic reconfigure
  void setDebugSliceZ(double z);

private:
  void loadConfig(ros::NodeHandle& pnh);
  void updateLoop();
  void updateOnce();

  void publishDebugOccupiedCloud(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp);
  void publishDebugBoundsMarker(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp);
  void publishDebugSpheresMarker(const std::vector<SphereObstacle>& spheres, const Map3DConfig& cfg, const ros::Time& stamp);
  void publishDebugSlice(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp);
  void publishDebugVoxelGrid(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp);
  std::vector<SphereObstacle> readSpheresFromGetPlanningScene(const Map3DConfig& cfg);

  bool queryServiceCb(cartesian_velocity_controller::QueryMap3D::Request& req,
                      cartesian_velocity_controller::QueryMap3D::Response& res);

  bool transformPoint(const Eigen::Vector3d& p_in,
                      const std::string& frame_in,
                      const std::string& frame_out,
                      Eigen::Vector3d& p_out) const;

  bool transformVector(const Eigen::Vector3d& v_in,
                       const std::string& frame_in,
                       const std::string& frame_out,
                       Eigen::Vector3d& v_out) const;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  mutable std::mutex cfg_mutex_;
  Map3DConfig cfg_;

  ros::Publisher occupied_cloud_pub_;
  ros::Publisher bounds_marker_pub_;
  ros::Publisher spheres_marker_pub_;
  ros::Publisher slice_image_pub_;
  ros::Publisher slice_gradient_pub_;
  ros::Publisher voxel_grid_pub_;

  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceServer query_srv_;

  // TF for frame transforms
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt PlanningScene monitor
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  std::unique_ptr<PlanningSceneSphereReader> sphere_reader_;

  // Builders
  VoxelizerSphereOnly voxelizer_;

  // Double buffer grids
  std::array<VoxelGrid3D, 2> grids_;
  std::atomic<int> front_index_{0};

  // Thread control
  std::atomic<bool> running_{false};
  std::thread worker_;

  // Last metadata (read without locks by copying)
  mutable std::mutex meta_mutex_;
  MapMetadata last_meta_;
};

}  // namespace cartesian_velocity_controller::map3d

