#include "cartesian_velocity_controller/map3d/map3d_manager.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <shape_msgs/SolidPrimitive.h>

#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

namespace cartesian_velocity_controller::map3d
{

static double nowSec()
{
  return ros::WallTime::now().toSec();
}

static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (q.norm() > 1e-12)
    q.normalize();
  else
    q = Eigen::Quaterniond::Identity();
  T.linear() = q.toRotationMatrix();
  return T;
}

Map3DManager::Map3DManager(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , tf_listener_(tf_buffer_)
  , voxelizer_(0.0)
  , grids_{VoxelGrid3D(Map3DConfig{}), VoxelGrid3D(Map3DConfig{})}
{
  loadConfig(pnh_);

  {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    voxelizer_.setObstacleMargin(cfg_.obstacle_margin);
    grids_[0].reset(cfg_);
    grids_[1].reset(cfg_);
  }

  // Debug publishers (optional)
  {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    // Opzione B: debug per-istanza nel namespace privato del nodo
    occupied_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>(cfg_.debug_occupied_cloud_topic, 1);
    // Latched so RViz shows it even if started later
    bounds_marker_pub_ = pnh_.advertise<visualization_msgs::Marker>(cfg_.debug_bounds_marker_topic, 1, true);
    spheres_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(cfg_.debug_spheres_marker_topic, 1);
    slice_image_pub_ = pnh_.advertise<sensor_msgs::Image>(cfg_.debug_slice_topic, 1);
    slice_gradient_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(cfg_.debug_slice_gradient_topic, 1);
    voxel_grid_pub_ = pnh_.advertise<visualization_msgs::Marker>(cfg_.debug_voxel_grid_topic, 1);
  }

  // Query service
  // Opzione B: service per-istanza nel namespace privato (~map3d/query)
  query_srv_ = pnh_.advertiseService("map3d/query", &Map3DManager::queryServiceCb, this);

  // PlanningSceneMonitor: robot_description is required
  {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(cfg_.robot_description_param);
  }

  if (!psm_ || !psm_->getPlanningScene())
  {
    ROS_WARN_NAMED("map3d", "PlanningSceneMonitor not ready (robot_description missing?)");
  }
  else
  {
    std::string joint_topic;
    std::string scene_topic;
    {
      std::lock_guard<std::mutex> lock(cfg_mutex_);
      joint_topic = cfg_.joint_state_topic;
      scene_topic = cfg_.planning_scene_topic;
    }

    psm_->startStateMonitor(joint_topic);
    psm_->startSceneMonitor(scene_topic);
    psm_->startWorldGeometryMonitor();  // for collision objects updates

    sphere_reader_ = std::make_unique<PlanningSceneSphereReader>(cfg_, psm_, &tf_buffer_);
  }
}

Map3DManager::~Map3DManager()
{
  stop();
}

void Map3DManager::loadConfig(ros::NodeHandle& pnh)
{
  std::lock_guard<std::mutex> lock(cfg_mutex_);

  pnh.param("map3d/size_x", cfg_.size_x, cfg_.size_x);
  pnh.param("map3d/size_y", cfg_.size_y, cfg_.size_y);
  pnh.param("map3d/size_z", cfg_.size_z, cfg_.size_z);
  pnh.param("map3d/resolution", cfg_.resolution, cfg_.resolution);
  pnh.param("map3d/frame_id", cfg_.frame_id, cfg_.frame_id);

  std::vector<double> origin_off{cfg_.origin_offset.x(), cfg_.origin_offset.y(), cfg_.origin_offset.z()};
  pnh.getParam("map3d/origin_offset", origin_off);
  if (origin_off.size() == 3)
    cfg_.origin_offset = Eigen::Vector3d(origin_off[0], origin_off[1], origin_off[2]);

  pnh.param("map3d/obstacle_margin", cfg_.obstacle_margin, cfg_.obstacle_margin);
  pnh.param("map3d/min_distance_eps", cfg_.min_distance_eps, cfg_.min_distance_eps);
  pnh.param("map3d/gradient_eps", cfg_.gradient_eps, cfg_.gradient_eps);
  pnh.param("map3d/gradient_clamp_distance", cfg_.gradient_clamp_distance, cfg_.gradient_clamp_distance);

  pnh.param("map3d/update_rate_hz", cfg_.update_rate_hz, cfg_.update_rate_hz);

  pnh.param("map3d/robot_description_param", cfg_.robot_description_param, cfg_.robot_description_param);
  pnh.param("map3d/joint_state_topic", cfg_.joint_state_topic, cfg_.joint_state_topic);
  pnh.param("map3d/planning_scene_topic", cfg_.planning_scene_topic, cfg_.planning_scene_topic);
  pnh.param("map3d/prefer_get_planning_scene_service", cfg_.prefer_get_planning_scene_service,
            cfg_.prefer_get_planning_scene_service);
  pnh.param("map3d/get_planning_scene_service", cfg_.get_planning_scene_service, cfg_.get_planning_scene_service);

  // Debug visualization
  pnh.param("map3d/debug/publish_occupied_cloud", cfg_.debug_publish_occupied_cloud, cfg_.debug_publish_occupied_cloud);
  pnh.param("map3d/debug/occupied_cloud_topic", cfg_.debug_occupied_cloud_topic, cfg_.debug_occupied_cloud_topic);
  pnh.param("map3d/debug/occupied_cloud_stride", cfg_.debug_occupied_cloud_stride, cfg_.debug_occupied_cloud_stride);
  pnh.param("map3d/debug/occupied_cloud_max_points", cfg_.debug_occupied_cloud_max_points, cfg_.debug_occupied_cloud_max_points);

  // Debug: bounds marker (wireframe box)
  pnh.param("map3d/debug/publish_bounds_marker", cfg_.debug_publish_bounds_marker, cfg_.debug_publish_bounds_marker);
  pnh.param("map3d/debug/bounds_marker_topic", cfg_.debug_bounds_marker_topic, cfg_.debug_bounds_marker_topic);
  pnh.param("map3d/debug/bounds_marker_line_width", cfg_.debug_bounds_marker_line_width, cfg_.debug_bounds_marker_line_width);
  pnh.param("map3d/debug/bounds_marker_alpha", cfg_.debug_bounds_marker_alpha, cfg_.debug_bounds_marker_alpha);
  pnh.param("map3d/debug/bounds_marker_color_r", cfg_.debug_bounds_marker_color_r, cfg_.debug_bounds_marker_color_r);
  pnh.param("map3d/debug/bounds_marker_color_g", cfg_.debug_bounds_marker_color_g, cfg_.debug_bounds_marker_color_g);
  pnh.param("map3d/debug/bounds_marker_color_b", cfg_.debug_bounds_marker_color_b, cfg_.debug_bounds_marker_color_b);

  // Debug: spheres marker
  pnh.param("map3d/debug/publish_spheres_marker", cfg_.debug_publish_spheres_marker, cfg_.debug_publish_spheres_marker);
  pnh.param("map3d/debug/spheres_marker_topic", cfg_.debug_spheres_marker_topic, cfg_.debug_spheres_marker_topic);
  pnh.param("map3d/debug/spheres_marker_alpha", cfg_.debug_spheres_marker_alpha, cfg_.debug_spheres_marker_alpha);
  pnh.param("map3d/debug/spheres_marker_color_r", cfg_.debug_spheres_marker_color_r, cfg_.debug_spheres_marker_color_r);
  pnh.param("map3d/debug/spheres_marker_color_g", cfg_.debug_spheres_marker_color_g, cfg_.debug_spheres_marker_color_g);
  pnh.param("map3d/debug/spheres_marker_color_b", cfg_.debug_spheres_marker_color_b, cfg_.debug_spheres_marker_color_b);
  pnh.param("map3d/debug/spheres_marker_scale_multiplier", cfg_.debug_spheres_marker_scale_multiplier,
            cfg_.debug_spheres_marker_scale_multiplier);
  pnh.param("map3d/debug/spheres_marker_min_diameter", cfg_.debug_spheres_marker_min_diameter,
            cfg_.debug_spheres_marker_min_diameter);

  // Debug: 2D slice with distance colormap + gradient arrows
  pnh.param("map3d/debug/publish_slice", cfg_.debug_publish_slice, cfg_.debug_publish_slice);
  pnh.param("map3d/debug/slice_topic", cfg_.debug_slice_topic, cfg_.debug_slice_topic);
  pnh.param("map3d/debug/slice_gradient_topic", cfg_.debug_slice_gradient_topic, cfg_.debug_slice_gradient_topic);
  pnh.param("map3d/debug/slice_z", cfg_.debug_slice_z, cfg_.debug_slice_z);
  pnh.param("map3d/debug/slice_gradient_stride", cfg_.debug_slice_gradient_stride, cfg_.debug_slice_gradient_stride);
  pnh.param("map3d/debug/slice_gradient_scale", cfg_.debug_slice_gradient_scale, cfg_.debug_slice_gradient_scale);
  pnh.param("map3d/debug/slice_max_distance", cfg_.debug_slice_max_distance, cfg_.debug_slice_max_distance);

  // Debug: voxel grid wireframe (occupied cells)
  pnh.param("map3d/debug/publish_voxel_grid", cfg_.debug_publish_voxel_grid, cfg_.debug_publish_voxel_grid);
  pnh.param("map3d/debug/voxel_grid_topic", cfg_.debug_voxel_grid_topic, cfg_.debug_voxel_grid_topic);
  pnh.param("map3d/debug/voxel_grid_stride", cfg_.debug_voxel_grid_stride, cfg_.debug_voxel_grid_stride);
  pnh.param("map3d/debug/voxel_grid_line_width", cfg_.debug_voxel_grid_line_width, cfg_.debug_voxel_grid_line_width);
  pnh.param("map3d/debug/voxel_grid_alpha", cfg_.debug_voxel_grid_alpha, cfg_.debug_voxel_grid_alpha);
  pnh.param("map3d/debug/voxel_grid_color_r", cfg_.debug_voxel_grid_color_r, cfg_.debug_voxel_grid_color_r);
  pnh.param("map3d/debug/voxel_grid_color_g", cfg_.debug_voxel_grid_color_g, cfg_.debug_voxel_grid_color_g);
  pnh.param("map3d/debug/voxel_grid_color_b", cfg_.debug_voxel_grid_color_b, cfg_.debug_voxel_grid_color_b);
}

Map3DConfig Map3DManager::getConfig() const
{
  std::lock_guard<std::mutex> lock(cfg_mutex_);
  return cfg_;
}

void Map3DManager::setDebugSliceZ(double z)
{
  std::lock_guard<std::mutex> lock(cfg_mutex_);
  cfg_.debug_slice_z = z;
}

void Map3DManager::start()
{
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true))
    return;

  worker_ = std::thread(&Map3DManager::updateLoop, this);
}

void Map3DManager::stop()
{
  if (!running_.exchange(false))
    return;

  if (worker_.joinable())
    worker_.join();
}

MapMetadata Map3DManager::getLatestMetadata() const
{
  std::lock_guard<std::mutex> lock(meta_mutex_);
  return last_meta_;
}

bool Map3DManager::transformPoint(const Eigen::Vector3d& p_in,
                                  const std::string& frame_in,
                                  const std::string& frame_out,
                                  Eigen::Vector3d& p_out) const
{
  if (frame_in == frame_out)
  {
    p_out = p_in;
    return true;
  }
  try
  {
    geometry_msgs::TransformStamped tf =
        tf_buffer_.lookupTransform(frame_out, frame_in, ros::Time(0), ros::Duration(0.02));
    const Eigen::Isometry3d T = tf2::transformToEigen(tf);
    p_out = T * p_in;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "map3d", "TF point %s->%s failed: %s",
                            frame_in.c_str(), frame_out.c_str(), ex.what());
    return false;
  }
}

bool Map3DManager::transformVector(const Eigen::Vector3d& v_in,
                                   const std::string& frame_in,
                                   const std::string& frame_out,
                                   Eigen::Vector3d& v_out) const
{
  if (frame_in == frame_out)
  {
    v_out = v_in;
    return true;
  }
  try
  {
    geometry_msgs::TransformStamped tf =
        tf_buffer_.lookupTransform(frame_out, frame_in, ros::Time(0), ros::Duration(0.02));
    const Eigen::Isometry3d T = tf2::transformToEigen(tf);
    v_out = T.linear() * v_in;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "map3d", "TF vector %s->%s failed: %s",
                            frame_in.c_str(), frame_out.c_str(), ex.what());
    return false;
  }
}

QueryResult Map3DManager::queryWorld(const Eigen::Vector3d& p_world, const std::string& world_frame) const
{
  QueryResult r;

  Map3DConfig cfg;
  {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    cfg = cfg_;
  }

  Eigen::Vector3d p_map;
  if (!transformPoint(p_world, world_frame, cfg.frame_id, p_map))
  {
    r.valid = false;
    return r;
  }

  const int idx = front_index_.load(std::memory_order_acquire);
  const VoxelGrid3D& grid = grids_[idx];

  bool inside = true;
  const double d = grid.getDistanceInterpolated(p_map, inside);
  Eigen::Vector3d g = grid.getGradientInterpolated(p_map, inside);

  r.valid = true;
  r.inside_bounds = inside;
  r.distance = d;
  r.gradient = g;
  r.closest_point = p_map - g * d;

  // Convert to world frame for output (vector uses rotation only)
  Eigen::Vector3d g_world;
  Eigen::Vector3d cp_world;
  if (!transformVector(r.gradient, cfg.frame_id, world_frame, g_world) ||
      !transformPoint(r.closest_point, cfg.frame_id, world_frame, cp_world))
  {
    r.valid = false;
    return r;
  }
  r.gradient = g_world;
  r.closest_point = cp_world;

  return r;
}

void Map3DManager::updateLoop()
{
  ros::Rate rate(1.0);
  for (;;)
  {
    if (!running_.load())
      break;

    Map3DConfig cfg;
    {
      std::lock_guard<std::mutex> lock(cfg_mutex_);
      cfg = cfg_;
    }

    if (cfg.update_rate_hz > 1e-6)
      rate = ros::Rate(cfg.update_rate_hz);

    updateOnce();

    if (cfg.update_rate_hz > 1e-6)
      rate.sleep();
  }
}

void Map3DManager::updateOnce()
{
  const double t0 = nowSec();

  Map3DConfig cfg;
  {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    cfg = cfg_;
  }

  const int front = front_index_.load(std::memory_order_relaxed);
  const int back = 1 - front;

  VoxelGrid3D& grid = grids_[back];
  grid.reset(cfg);  // reset is allocation-heavy; we’ll optimize after correctness is stable

  MapMetadata meta;
  meta.stamp = ros::Time::now();

  std::vector<SphereObstacle> spheres;
  const double t_read0 = nowSec();
  if (cfg.prefer_get_planning_scene_service)
  {
    spheres = readSpheresFromGetPlanningScene(cfg);
  }
  else if (sphere_reader_)
  {
    spheres = sphere_reader_->readSpheres(cfg.frame_id);
    // Fallback: if monitor didn't receive world geometry, query move_group directly
    if (spheres.empty())
      spheres = readSpheresFromGetPlanningScene(cfg);
  }
  meta.t_read_scene = nowSec() - t_read0;

  const double t_vox0 = nowSec();
  voxelizer_.setObstacleMargin(cfg.obstacle_margin);
  voxelizer_.voxelize(grid, spheres);
  meta.t_voxelize = nowSec() - t_vox0;

  const double t_edt0 = nowSec();
  EDTCalculator::computeEDT(grid);
  meta.t_edt = nowSec() - t_edt0;

  meta.t_total = nowSec() - t0;

  // Update metadata (stored in grid and manager)
  grid.metadata() = meta;
  {
    std::lock_guard<std::mutex> lock(meta_mutex_);
    meta.update_count = last_meta_.update_count + 1;
    last_meta_ = meta;
  }

  // Swap
  front_index_.store(back, std::memory_order_release);

  // Debug: occupied voxels as PointCloud2
  publishDebugOccupiedCloud(grid, cfg, meta.stamp);
  // Debug: bounds marker (wireframe box), useful even when empty
  publishDebugBoundsMarker(grid, cfg, meta.stamp);
  // Debug: spheres read from scene
  publishDebugSpheresMarker(spheres, cfg, meta.stamp);
  // Debug: 2D slice with distance colormap + gradient arrows
  publishDebugSlice(grid, cfg, meta.stamp);
  // Debug: voxel grid wireframe (occupied cells)
  publishDebugVoxelGrid(grid, cfg, meta.stamp);

  ROS_DEBUG_THROTTLE_NAMED(1.0, "map3d",
                           "Map update %.1f ms (read %.1f, vox %.1f, edt %.1f), spheres=%zu",
                           1000.0 * meta.t_total, 1000.0 * meta.t_read_scene, 1000.0 * meta.t_voxelize,
                           1000.0 * meta.t_edt, spheres.size());
}

std::vector<SphereObstacle> Map3DManager::readSpheresFromGetPlanningScene(const Map3DConfig& cfg)
{
  std::vector<SphereObstacle> out;

  // Lazy-init client (config can change at runtime)
  if (!get_planning_scene_client_ || get_planning_scene_client_.getService() != cfg.get_planning_scene_service)
  {
    get_planning_scene_client_ =
        nh_.serviceClient<moveit_msgs::GetPlanningScene>(cfg.get_planning_scene_service, /*persistent=*/true);
  }

  if (!get_planning_scene_client_)
    return out;

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  if (!get_planning_scene_client_.call(srv))
  {
    ROS_WARN_THROTTLE_NAMED(1.0, "map3d", "Failed to call %s", cfg.get_planning_scene_service.c_str());
    return out;
  }

  const auto& objs = srv.response.scene.world.collision_objects;
  out.reserve(objs.size());

  for (const auto& obj : objs)
  {
    const std::string frame_in = obj.header.frame_id;
    if (frame_in.empty())
      continue;

    const Eigen::Isometry3d T_obj = poseMsgToEigen(obj.pose);

    const std::size_t n = std::min(obj.primitives.size(), obj.primitive_poses.size());
    for (std::size_t i = 0; i < n; ++i)
    {
      const auto& prim = obj.primitives[i];
      if (prim.type != shape_msgs::SolidPrimitive::SPHERE)
        continue;

      if (prim.dimensions.empty())
        continue;

      const double radius = prim.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
      const Eigen::Isometry3d T_prim = poseMsgToEigen(obj.primitive_poses[i]);
      const Eigen::Vector3d c_in = (T_obj * T_prim).translation();

      Eigen::Vector3d c_out;
      if (!transformPoint(c_in, frame_in, cfg.frame_id, c_out))
        continue;

      SphereObstacle s;
      s.id = obj.id;
      s.center = c_out;
      s.radius = radius;
      out.push_back(s);
    }
  }

  return out;
}

void Map3DManager::publishDebugOccupiedCloud(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp)
{
  if (!cfg.debug_publish_occupied_cloud)
    return;
  if (!occupied_cloud_pub_)
    return;
  if (occupied_cloud_pub_.getNumSubscribers() == 0)
    return;

  const int stride = std::max(1, cfg.debug_occupied_cloud_stride);
  const int max_points = std::max(0, cfg.debug_occupied_cloud_max_points);

  std::vector<Eigen::Vector3d> pts;
  if (max_points > 0)
    pts.reserve(static_cast<std::size_t>(max_points));

  const auto& occ = grid.occupancy();
  const std::size_t nx = grid.nx();
  const std::size_t ny = grid.ny();
  const std::size_t nz = grid.nz();

  bool stop = false;
  for (std::size_t iz = 0; iz < nz && !stop; iz += static_cast<std::size_t>(stride))
  {
    for (std::size_t iy = 0; iy < ny && !stop; iy += static_cast<std::size_t>(stride))
    {
      for (std::size_t ix = 0; ix < nx; ix += static_cast<std::size_t>(stride))
      {
        const std::size_t idx = iz * (nx * ny) + iy * nx + ix;
        if (idx >= occ.size())
          continue;
        if (occ[idx] != VoxelGrid3D::kOccupied)
          continue;

        const Eigen::Vector3d p = grid.voxelToWorld(static_cast<int>(ix), static_cast<int>(iy), static_cast<int>(iz));
        pts.push_back(p);

        if (max_points > 0 && static_cast<int>(pts.size()) >= max_points)
        {
          stop = true;
          break;
        }
      }
    }
  }

  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = cfg.frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(pts.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

  for (const auto& p : pts)
  {
    *it_x = static_cast<float>(p.x());
    *it_y = static_cast<float>(p.y());
    *it_z = static_cast<float>(p.z());
    ++it_x; ++it_y; ++it_z;
  }

  occupied_cloud_pub_.publish(cloud);
}

void Map3DManager::publishDebugBoundsMarker(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp)
{
  (void)grid;
  if (!cfg.debug_publish_bounds_marker)
    return;
  if (!bounds_marker_pub_)
    return;

  const Eigen::Vector3d c = cfg.origin_offset;  // center in map frame
  const double hx = 0.5 * cfg.size_x;
  const double hy = 0.5 * cfg.size_y;
  const double hz = 0.5 * cfg.size_z;

  const double x0 = c.x() - hx, x1 = c.x() + hx;
  const double y0 = c.y() - hy, y1 = c.y() + hy;
  const double z0 = c.z() - hz, z1 = c.z() + hz;

  auto p = [](double x, double y, double z) {
    geometry_msgs::Point pt;
    pt.x = x; pt.y = y; pt.z = z;
    return pt;
  };

  const geometry_msgs::Point v000 = p(x0, y0, z0);
  const geometry_msgs::Point v100 = p(x1, y0, z0);
  const geometry_msgs::Point v110 = p(x1, y1, z0);
  const geometry_msgs::Point v010 = p(x0, y1, z0);

  const geometry_msgs::Point v001 = p(x0, y0, z1);
  const geometry_msgs::Point v101 = p(x1, y0, z1);
  const geometry_msgs::Point v111 = p(x1, y1, z1);
  const geometry_msgs::Point v011 = p(x0, y1, z1);

  visualization_msgs::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = cfg.frame_id;
  m.ns = "map3d_bounds";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = std::max(1e-4, cfg.debug_bounds_marker_line_width);
  m.color.a = static_cast<float>(std::clamp(cfg.debug_bounds_marker_alpha, 0.0, 1.0));
  m.color.r = static_cast<float>(std::clamp(cfg.debug_bounds_marker_color_r, 0.0, 1.0));
  m.color.g = static_cast<float>(std::clamp(cfg.debug_bounds_marker_color_g, 0.0, 1.0));
  m.color.b = static_cast<float>(std::clamp(cfg.debug_bounds_marker_color_b, 0.0, 1.0));
  m.lifetime = ros::Duration(0.0);  // latched publisher -> persists

  auto addEdge = [&](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    m.points.push_back(a);
    m.points.push_back(b);
  };

  // Bottom rectangle (z0)
  addEdge(v000, v100);
  addEdge(v100, v110);
  addEdge(v110, v010);
  addEdge(v010, v000);

  // Top rectangle (z1)
  addEdge(v001, v101);
  addEdge(v101, v111);
  addEdge(v111, v011);
  addEdge(v011, v001);

  // Vertical edges
  addEdge(v000, v001);
  addEdge(v100, v101);
  addEdge(v110, v111);
  addEdge(v010, v011);

  bounds_marker_pub_.publish(m);
}

void Map3DManager::publishDebugSpheresMarker(const std::vector<SphereObstacle>& spheres,
                                             const Map3DConfig& cfg,
                                             const ros::Time& stamp)
{
  if (!cfg.debug_publish_spheres_marker)
    return;
  if (!spheres_marker_pub_)
    return;
  if (spheres_marker_pub_.getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray arr;
  arr.markers.reserve(spheres.size() + 1);

  // Clear previous markers (in case count decreases)
  visualization_msgs::Marker clear;
  clear.header.stamp = stamp;
  clear.header.frame_id = cfg.frame_id;
  clear.ns = "map3d_spheres";
  clear.id = 0;
  clear.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(clear);

  int id = 1;
  for (const auto& s : spheres)
  {
    visualization_msgs::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = cfg.frame_id;
    m.ns = "map3d_spheres";
    m.id = id++;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = s.center.x();
    m.pose.position.y = s.center.y();
    m.pose.position.z = s.center.z();
    m.pose.orientation.w = 1.0;
    const double r = std::max(0.0, s.radius);
    const double mul = std::max(0.0, cfg.debug_spheres_marker_scale_multiplier);
    const double d0 = (2.0 * r) * mul;
    const double dmin = std::max(1e-4, cfg.debug_spheres_marker_min_diameter);
    const double d = std::max(dmin, d0);
    m.scale.x = d;
    m.scale.y = d;
    m.scale.z = d;
    m.color.a = static_cast<float>(std::clamp(cfg.debug_spheres_marker_alpha, 0.0, 1.0));
    m.color.r = static_cast<float>(std::clamp(cfg.debug_spheres_marker_color_r, 0.0, 1.0));
    m.color.g = static_cast<float>(std::clamp(cfg.debug_spheres_marker_color_g, 0.0, 1.0));
    m.color.b = static_cast<float>(std::clamp(cfg.debug_spheres_marker_color_b, 0.0, 1.0));
    m.lifetime = ros::Duration(0.5);
    arr.markers.push_back(m);
  }

  spheres_marker_pub_.publish(arr);
}

bool Map3DManager::queryServiceCb(cartesian_velocity_controller::QueryMap3D::Request& req,
                                  cartesian_velocity_controller::QueryMap3D::Response& res)
{
  const Eigen::Vector3d p_world(req.point.x, req.point.y, req.point.z);
  const std::string frame = req.frame_id.empty() ? "world" : req.frame_id;

  QueryResult qr = queryWorld(p_world, frame);

  res.valid = qr.valid;
  res.inside_bounds = qr.inside_bounds;
  res.distance = qr.distance;
  res.gradient.x = qr.gradient.x();
  res.gradient.y = qr.gradient.y();
  res.gradient.z = qr.gradient.z();
  res.closest_point.x = qr.closest_point.x();
  res.closest_point.y = qr.closest_point.y();
  res.closest_point.z = qr.closest_point.z();

  return true;
}

// Helper: distance (0..max_distance) to RGB color (red=near, green=mid, blue=far)
static void distanceToRGB(double d, double max_d, uint8_t& r, uint8_t& g, uint8_t& b)
{
  const double t = std::clamp(d / max_d, 0.0, 1.0);
  // Jet-like colormap: red(0) -> yellow(0.25) -> green(0.5) -> cyan(0.75) -> blue(1)
  if (t < 0.25)
  {
    r = 255;
    g = static_cast<uint8_t>(255 * (t / 0.25));
    b = 0;
  }
  else if (t < 0.5)
  {
    r = static_cast<uint8_t>(255 * (1.0 - (t - 0.25) / 0.25));
    g = 255;
    b = 0;
  }
  else if (t < 0.75)
  {
    r = 0;
    g = 255;
    b = static_cast<uint8_t>(255 * ((t - 0.5) / 0.25));
  }
  else
  {
    r = 0;
    g = static_cast<uint8_t>(255 * (1.0 - (t - 0.75) / 0.25));
    b = 255;
  }
}

void Map3DManager::publishDebugSlice(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp)
{
  if (!cfg.debug_publish_slice)
    return;

  const bool has_image_sub = slice_image_pub_.getNumSubscribers() > 0;
  const bool has_gradient_sub = slice_gradient_pub_.getNumSubscribers() > 0;
  if (!has_image_sub && !has_gradient_sub)
    return;

  const std::size_t nx = grid.nx();
  const std::size_t ny = grid.ny();
  if (nx == 0 || ny == 0)
    return;

  const double slice_z = cfg.debug_slice_z;
  const double max_dist = std::max(0.01, cfg.debug_slice_max_distance);
  const int stride = std::max(1, cfg.debug_slice_gradient_stride);
  const double arrow_scale = cfg.debug_slice_gradient_scale;

  // Create image (RGB8)
  sensor_msgs::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = cfg.frame_id;
  img.height = static_cast<uint32_t>(ny);
  img.width = static_cast<uint32_t>(nx);
  img.encoding = "rgb8";
  img.is_bigendian = false;
  img.step = static_cast<uint32_t>(nx * 3);
  img.data.resize(nx * ny * 3);

  // Gradient arrows
  visualization_msgs::MarkerArray grad_arr;
  visualization_msgs::Marker clear;
  clear.header.stamp = stamp;
  clear.header.frame_id = cfg.frame_id;
  clear.ns = "map3d_slice_grad";
  clear.id = 0;
  clear.action = visualization_msgs::Marker::DELETEALL;
  grad_arr.markers.push_back(clear);

  int arrow_id = 1;

  for (std::size_t iy = 0; iy < ny; ++iy)
  {
    for (std::size_t ix = 0; ix < nx; ++ix)
    {
      // Sample at center of voxel at slice_z
      const Eigen::Vector3d p = grid.voxelToWorld(static_cast<int>(ix), static_cast<int>(iy), 0);
      const Eigen::Vector3d p_slice(p.x(), p.y(), slice_z);

      bool inside = true;
      const double d = grid.getDistanceInterpolated(p_slice, inside);

      // Image pixel (flip Y for image convention: row 0 = top)
      const std::size_t row = ny - 1 - iy;
      const std::size_t pixel_idx = (row * nx + ix) * 3;
      uint8_t r, g, b;
      if (!inside)
      {
        r = g = b = 50;  // dark gray for out-of-bounds
      }
      else
      {
        distanceToRGB(d, max_dist, r, g, b);
      }
      img.data[pixel_idx + 0] = r;
      img.data[pixel_idx + 1] = g;
      img.data[pixel_idx + 2] = b;

      // Gradient arrows (sampled)
      if (has_gradient_sub && (ix % stride == 0) && (iy % stride == 0) && inside && d < max_dist)
      {
        const Eigen::Vector3d grad = grid.getGradientInterpolated(p_slice, inside);

        visualization_msgs::Marker arrow;
        arrow.header.stamp = stamp;
        arrow.header.frame_id = cfg.frame_id;
        arrow.ns = "map3d_slice_grad";
        arrow.id = arrow_id++;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start, end;
        start.x = p_slice.x();
        start.y = p_slice.y();
        start.z = p_slice.z();
        end.x = p_slice.x() + grad.x() * arrow_scale;
        end.y = p_slice.y() + grad.y() * arrow_scale;
        end.z = p_slice.z() + grad.z() * arrow_scale;
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.01;  // shaft diameter
        arrow.scale.y = 0.02;  // head diameter
        arrow.scale.z = 0.0;

        // Color: same as distance
        arrow.color.a = 0.9f;
        arrow.color.r = static_cast<float>(r) / 255.0f;
        arrow.color.g = static_cast<float>(g) / 255.0f;
        arrow.color.b = static_cast<float>(b) / 255.0f;

        arrow.lifetime = ros::Duration(0.5);
        grad_arr.markers.push_back(arrow);
      }
    }
  }

  if (has_image_sub)
    slice_image_pub_.publish(img);
  if (has_gradient_sub)
    slice_gradient_pub_.publish(grad_arr);
}

void Map3DManager::publishDebugVoxelGrid(const VoxelGrid3D& grid, const Map3DConfig& cfg, const ros::Time& stamp)
{
  if (!cfg.debug_publish_voxel_grid)
    return;
  if (!voxel_grid_pub_)
    return;
  if (voxel_grid_pub_.getNumSubscribers() == 0)
    return;

  const int stride = std::max(1, cfg.debug_voxel_grid_stride);
  const double res = grid.resolution();
  const double half_res = res * 0.5;

  visualization_msgs::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = cfg.frame_id;
  m.ns = "map3d_voxel_grid";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = std::max(1e-4, cfg.debug_voxel_grid_line_width);
  m.color.a = static_cast<float>(std::clamp(cfg.debug_voxel_grid_alpha, 0.0, 1.0));
  m.color.r = static_cast<float>(std::clamp(cfg.debug_voxel_grid_color_r, 0.0, 1.0));
  m.color.g = static_cast<float>(std::clamp(cfg.debug_voxel_grid_color_g, 0.0, 1.0));
  m.color.b = static_cast<float>(std::clamp(cfg.debug_voxel_grid_color_b, 0.0, 1.0));
  m.lifetime = ros::Duration(0.5);

  const auto& occ = grid.occupancy();
  const std::size_t nx = grid.nx();
  const std::size_t ny = grid.ny();
  const std::size_t nz = grid.nz();

  // Helper to create a point
  auto pt = [](double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  };

  // Helper to add an edge (two points)
  auto addEdge = [&](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    m.points.push_back(a);
    m.points.push_back(b);
  };

  // Iterate over occupied voxels
  for (std::size_t iz = 0; iz < nz; iz += static_cast<std::size_t>(stride))
  {
    for (std::size_t iy = 0; iy < ny; iy += static_cast<std::size_t>(stride))
    {
      for (std::size_t ix = 0; ix < nx; ix += static_cast<std::size_t>(stride))
      {
        const std::size_t idx = iz * (nx * ny) + iy * nx + ix;
        if (idx >= occ.size())
          continue;
        if (occ[idx] != VoxelGrid3D::kOccupied)
          continue;

        // Get voxel center
        const Eigen::Vector3d c = grid.voxelToWorld(static_cast<int>(ix), static_cast<int>(iy), static_cast<int>(iz));

        // Compute 8 corner vertices of the cube
        const double x0 = c.x() - half_res, x1 = c.x() + half_res;
        const double y0 = c.y() - half_res, y1 = c.y() + half_res;
        const double z0 = c.z() - half_res, z1 = c.z() + half_res;

        const geometry_msgs::Point v000 = pt(x0, y0, z0);
        const geometry_msgs::Point v100 = pt(x1, y0, z0);
        const geometry_msgs::Point v110 = pt(x1, y1, z0);
        const geometry_msgs::Point v010 = pt(x0, y1, z0);
        const geometry_msgs::Point v001 = pt(x0, y0, z1);
        const geometry_msgs::Point v101 = pt(x1, y0, z1);
        const geometry_msgs::Point v111 = pt(x1, y1, z1);
        const geometry_msgs::Point v011 = pt(x0, y1, z1);

        // Add 12 edges of the cube
        // Bottom face (z0)
        addEdge(v000, v100);
        addEdge(v100, v110);
        addEdge(v110, v010);
        addEdge(v010, v000);

        // Top face (z1)
        addEdge(v001, v101);
        addEdge(v101, v111);
        addEdge(v111, v011);
        addEdge(v011, v001);

        // Vertical edges
        addEdge(v000, v001);
        addEdge(v100, v101);
        addEdge(v110, v111);
        addEdge(v010, v011);
      }
    }
  }

  voxel_grid_pub_.publish(m);
}

}  // namespace cartesian_velocity_controller::map3d

