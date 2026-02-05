#pragma once

#include <Eigen/Core>
#include <ros/time.h>

#include <cstdint>
#include <string>
#include <vector>

namespace cartesian_velocity_controller::map3d
{

struct SphereObstacle
{
  std::string id;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};  // in map frame unless stated otherwise
  double radius{0.0};                               // meters
};

struct MapMetadata
{
  ros::Time stamp{0.0};
  uint64_t update_count{0};

  // Telemetry (seconds)
  double t_read_scene{0.0};
  double t_voxelize{0.0};
  double t_edt{0.0};
  double t_total{0.0};
};

struct Map3DConfig
{
  // Dimensions in meters (total extents)
  double size_x{3.0};
  double size_y{3.0};
  double size_z{2.0};

  // Voxel resolution
  double resolution{0.05};

  // Map frame id
  std::string frame_id{"base_link"};

  // Origin offset of the grid center in map frame
  Eigen::Vector3d origin_offset{0.0, 0.0, 0.5};

  // Obstacles inflation (global fixed margin)
  double obstacle_margin{0.05};

  // Query behavior
  double min_distance_eps{1e-3};
  double gradient_eps{1e-6};
  double gradient_clamp_distance{0.02};

  // Update loop
  double update_rate_hz{20.0};  // 0 -> as fast as possible

  // MoveIt / PlanningScene
  std::string robot_description_param{"robot_description"};
  std::string joint_state_topic{"/joint_states"};
  std::string planning_scene_topic{"planning_scene"};
  bool prefer_get_planning_scene_service{true};
  std::string get_planning_scene_service{"/get_planning_scene"};

  // Debug visualization (RViz)
  bool debug_publish_occupied_cloud{false};
  std::string debug_occupied_cloud_topic{"map3d/occupied_voxels"};
  int debug_occupied_cloud_stride{2};      // sample every N voxels per axis (>=1)
  int debug_occupied_cloud_max_points{0};  // 0 = unlimited

  // Debug visualization: map bounds (wireframe box) for RViz
  bool debug_publish_bounds_marker{false};
  std::string debug_bounds_marker_topic{"map3d/bounds"};
  double debug_bounds_marker_line_width{0.01};  // meters
  double debug_bounds_marker_alpha{0.8};
  double debug_bounds_marker_color_r{0.2};
  double debug_bounds_marker_color_g{1.0};
  double debug_bounds_marker_color_b{0.2};

  // Debug visualization: spheres read from PlanningScene
  bool debug_publish_spheres_marker{false};
  std::string debug_spheres_marker_topic{"map3d/spheres"};
  double debug_spheres_marker_alpha{0.7};
  double debug_spheres_marker_color_r{0.2};
  double debug_spheres_marker_color_g{0.4};
  double debug_spheres_marker_color_b{1.0};
  double debug_spheres_marker_scale_multiplier{1.0};  // marker-only scaling (>=1 recommended)
  double debug_spheres_marker_min_diameter{0.02};     // meters, marker-only minimum

  // Debug visualization: 2D slice with distance colormap + gradient arrows
  bool debug_publish_slice{false};
  std::string debug_slice_topic{"map3d/distance_slice"};
  std::string debug_slice_gradient_topic{"map3d/gradient_arrows"};
  double debug_slice_z{0.5};             // meters, Z coordinate of slice in frame_id
  int debug_slice_gradient_stride{4};    // sample gradient every N voxels (>=1)
  double debug_slice_gradient_scale{0.1}; // meters, arrow length scaling
  double debug_slice_max_distance{1.0};  // meters, distances above this are clamped for colormap

  // Debug visualization: occupied voxel wireframe grid
  bool debug_publish_voxel_grid{false};
  std::string debug_voxel_grid_topic{"map3d/voxel_grid"};
  int debug_voxel_grid_stride{1};        // sample every N voxels (>=1), reduce for performance
  double debug_voxel_grid_line_width{0.002};  // meters
  double debug_voxel_grid_alpha{0.6};
  double debug_voxel_grid_color_r{1.0};
  double debug_voxel_grid_color_g{0.5};
  double debug_voxel_grid_color_b{0.0};
};

struct QueryResult
{
  bool valid{false};
  bool inside_bounds{true};

  // Distance to inflated obstacle surface (meters)
  double distance{0.0};

  // Normalized gradient of the distance field (points towards free space), in query frame
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};

  // Closest point on inflated obstacle surface, in query frame (approx via gradient)
  Eigen::Vector3d closest_point{Eigen::Vector3d::Zero()};
};

}  // namespace cartesian_velocity_controller::map3d

