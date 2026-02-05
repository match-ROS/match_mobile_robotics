#include "cartesian_velocity_controller/components/marker_publisher.hpp"

#include <sstream>
#include <iomanip>

namespace cartesian_velocity_controller
{

namespace
{
inline std::string makeMarkerNs(const MarkerPublisherConfig& cfg, const std::string& base_ns)
{
  if (cfg.ns_prefix.empty())
  {
    return base_ns;
  }
  if (base_ns.empty())
  {
    return cfg.ns_prefix;
  }
  return cfg.ns_prefix + "/" + base_ns;
}
}  // namespace

MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, const MarkerPublisherConfig& config)
  : config_(config)
{
  velocity_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("velocity_markers", 10);
  target_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("target_markers", 10);
  command_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("command_markers", 10);
  repulsion_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("repulsion_markers", 10);
  tcp_fk_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("tcp_fk_markers", 10);
}

void MarkerPublisher::setConfig(const MarkerPublisherConfig& config)
{
  config_ = config;
}

void MarkerPublisher::publishVelocityMarkers(const ros::Time& stamp,
                                              const Eigen::Vector3d& origin,
                                              const Eigen::Vector3d& v_goal_linear,
                                              const Eigen::Vector3d& v_goal_angular,
                                              const Eigen::Vector3d& v_obs_linear,
                                              const Eigen::Vector3d& v_link_linear)
{
  visualization_msgs::MarkerArray markers;

  // Helper lambda to create arrow marker
  auto createArrow = [&](int id, const std::string& ns, const Eigen::Vector3d& velocity,
                         float r, float g, float b, double scale_factor = 1.0) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = config_.global_frame;
    arrow.ns = makeMarkerNs(config_, ns);
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(0.5);
    arrow.scale.x = 0.01;   // shaft diameter
    arrow.scale.y = 0.02;   // head diameter
    arrow.scale.z = 0.0;    // auto head length
    arrow.color.a = 0.9;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.pose.orientation.w = 1.0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    geometry_msgs::Point end;
    end.x = origin.x() + velocity.x() * config_.arrow_scale * scale_factor;
    end.y = origin.y() + velocity.y() * config_.arrow_scale * scale_factor;
    end.z = origin.z() + velocity.z() * config_.arrow_scale * scale_factor;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    return arrow;
  };

  // V_goal linear (cyan/teal - attractive)
  if (v_goal_linear.norm() >= kEpsilon)
  {
    markers.markers.push_back(createArrow(0, "v_goal_linear", v_goal_linear, 0.0, 0.8, 0.9));
  }

  // V_goal angular (magenta - attractive angular)
  if (v_goal_angular.norm() >= kEpsilon)
  {
    markers.markers.push_back(createArrow(1, "v_goal_angular", v_goal_angular, 0.9, 0.2, 0.9, 0.5));
  }

  // V_obs linear (red/orange - repulsive TCP)
  if (v_obs_linear.norm() >= kEpsilon)
  {
    markers.markers.push_back(createArrow(2, "v_obs_linear", v_obs_linear, 1.0, 0.3, 0.0));
  }

  // V_link linear (yellow - repulsive links)
  if (v_link_linear.norm() >= kEpsilon)
  {
    markers.markers.push_back(createArrow(3, "v_link_linear", v_link_linear, 1.0, 0.85, 0.0));
  }

  if (!markers.markers.empty())
  {
    velocity_marker_pub_.publish(markers);
    velocity_markers_active_ = true;
  }
}

void MarkerPublisher::publishTargetMarkers(const ros::Time& stamp,
                                            const Eigen::Isometry3d& active_waypoint,
                                            const Eigen::Isometry3d& target_raw,
                                            const Eigen::Isometry3d& target_filtered)
{
  visualization_msgs::MarkerArray markers;
  int id_counter = 0;

  // Helper lambda to create sphere marker
  auto createSphere = [&](const std::string& ns, const Eigen::Vector3d& position,
                          float r, float g, float b, double size) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker sphere;
    sphere.header.stamp = stamp;
    sphere.header.frame_id = config_.global_frame;
    sphere.ns = makeMarkerNs(config_, ns);
    sphere.id = id_counter++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.lifetime = ros::Duration(0.5);
    sphere.scale.x = size;
    sphere.scale.y = size;
    sphere.scale.z = size;
    sphere.color.a = 0.7;
    sphere.color.r = r;
    sphere.color.g = g;
    sphere.color.b = b;
    sphere.pose.position.x = position.x();
    sphere.pose.position.y = position.y();
    sphere.pose.position.z = position.z();
    sphere.pose.orientation.w = 1.0;
    return sphere;
  };

  // Helper lambda to create axis arrow for orientation visualization (XYZ triad)
  auto createAxisArrow = [&](const std::string& ns, const Eigen::Isometry3d& pose,
                             int axis, double length) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = config_.global_frame;
    arrow.ns = makeMarkerNs(config_, ns);
    arrow.id = id_counter++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(0.5);
    arrow.scale.x = 0.006;  // shaft diameter
    arrow.scale.y = 0.012;  // head diameter
    arrow.scale.z = 0.0;
    arrow.color.a = 0.9;
    
    // Set color based on axis: X=red, Y=green, Z=blue
    switch (axis)
    {
      case 0:  // X axis - Red
        arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0;
        break;
      case 1:  // Y axis - Green
        arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 0.0;
        break;
      case 2:  // Z axis - Blue
        arrow.color.r = 0.0; arrow.color.g = 0.0; arrow.color.b = 1.0;
        break;
    }
    arrow.pose.orientation.w = 1.0;

    // Get axis direction from rotation matrix
    Eigen::Vector3d axis_dir = pose.rotation().col(axis);
    Eigen::Vector3d origin = pose.translation();
    Eigen::Vector3d end = origin + axis_dir * length;

    geometry_msgs::Point p_start, p_end;
    p_start.x = origin.x(); p_start.y = origin.y(); p_start.z = origin.z();
    p_end.x = end.x(); p_end.y = end.y(); p_end.z = end.z();

    arrow.points.push_back(p_start);
    arrow.points.push_back(p_end);

    return arrow;
  };

  // Helper lambda to add pose markers (sphere + XYZ triad)
  auto addPoseMarkers = [&](const std::string& ns, const Eigen::Isometry3d& pose,
                            float r, float g, float b, double sphere_size, double axis_length)
  {
    // Sphere for position
    markers.markers.push_back(createSphere(ns, pose.translation(), r, g, b, sphere_size));
    
    // XYZ triad for orientation
    markers.markers.push_back(createAxisArrow(ns + "_axis", pose, 0, axis_length));  // X
    markers.markers.push_back(createAxisArrow(ns + "_axis", pose, 1, axis_length));  // Y
    markers.markers.push_back(createAxisArrow(ns + "_axis", pose, 2, axis_length));  // Z
  };

  // Active waypoint (green sphere + XYZ triad)
  addPoseMarkers("active_waypoint", active_waypoint, 0.0, 1.0, 0.3, 0.04, 0.08);

  // Target raw (orange sphere + XYZ triad)
  addPoseMarkers("target_raw", target_raw, 1.0, 0.5, 0.0, 0.03, 0.06);

  // Target filtered (blue sphere + XYZ triad)
  addPoseMarkers("target_filtered", target_filtered, 0.2, 0.4, 1.0, 0.03, 0.06);

  target_marker_pub_.publish(markers);
  target_markers_active_ = true;
}

void MarkerPublisher::publishCommandMarkers(const ros::Time& stamp,
                                             const Eigen::Vector3d& origin,
                                             const Eigen::Vector3d& cartesian_cmd_linear,
                                             const Eigen::Vector3d& cartesian_cmd_angular)
{
  double linear_speed = cartesian_cmd_linear.norm();
  double angular_speed = cartesian_cmd_angular.norm();

  if (linear_speed < kEpsilon && angular_speed < kEpsilon)
  {
    return;
  }

  visualization_msgs::MarkerArray markers;

  // Linear velocity arrow (white/bright)
  if (linear_speed >= kEpsilon)
  {
    visualization_msgs::Marker linear_arrow;
    linear_arrow.header.stamp = stamp;
    linear_arrow.header.frame_id = config_.global_frame;
    linear_arrow.ns = makeMarkerNs(config_, "cartesian_cmd");
    linear_arrow.id = 0;
    linear_arrow.type = visualization_msgs::Marker::ARROW;
    linear_arrow.action = visualization_msgs::Marker::ADD;
    linear_arrow.lifetime = ros::Duration(0.5);
    linear_arrow.scale.x = 0.012;  // shaft diameter
    linear_arrow.scale.y = 0.024;  // head diameter
    linear_arrow.scale.z = 0.0;
    linear_arrow.color.a = 0.95;
    linear_arrow.color.r = 0.95;
    linear_arrow.color.g = 0.95;
    linear_arrow.color.b = 0.95;
    linear_arrow.pose.orientation.w = 1.0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    geometry_msgs::Point end;
    end.x = origin.x() + cartesian_cmd_linear.x() * config_.arrow_scale;
    end.y = origin.y() + cartesian_cmd_linear.y() * config_.arrow_scale;
    end.z = origin.z() + cartesian_cmd_linear.z() * config_.arrow_scale;

    linear_arrow.points.push_back(start);
    linear_arrow.points.push_back(end);
    markers.markers.push_back(linear_arrow);

    // Text for linear velocity magnitude
    visualization_msgs::Marker linear_text;
    linear_text.header.stamp = stamp;
    linear_text.header.frame_id = config_.global_frame;
    linear_text.ns = makeMarkerNs(config_, "cartesian_cmd");
    linear_text.id = 1;
    linear_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    linear_text.action = visualization_msgs::Marker::ADD;
    linear_text.lifetime = ros::Duration(0.5);
    linear_text.scale.z = 0.025;
    linear_text.color.a = 0.9;
    linear_text.color.r = 0.95;
    linear_text.color.g = 0.95;
    linear_text.color.b = 0.95;
    linear_text.pose.position = end;
    linear_text.pose.position.z += 0.02;
    linear_text.pose.orientation.w = 1.0;
    std::ostringstream ss_lin;
    ss_lin.setf(std::ios::fixed);
    ss_lin.precision(3);
    ss_lin << "|v|=" << linear_speed;
    linear_text.text = ss_lin.str();
    markers.markers.push_back(linear_text);
  }

  // Angular velocity arrow (light purple)
  if (angular_speed >= kEpsilon)
  {
    visualization_msgs::Marker angular_arrow;
    angular_arrow.header.stamp = stamp;
    angular_arrow.header.frame_id = config_.global_frame;
    angular_arrow.ns = makeMarkerNs(config_, "cartesian_cmd");
    angular_arrow.id = 2;
    angular_arrow.type = visualization_msgs::Marker::ARROW;
    angular_arrow.action = visualization_msgs::Marker::ADD;
    angular_arrow.lifetime = ros::Duration(0.5);
    angular_arrow.scale.x = 0.010;
    angular_arrow.scale.y = 0.020;
    angular_arrow.scale.z = 0.0;
    angular_arrow.color.a = 0.85;
    angular_arrow.color.r = 0.8;
    angular_arrow.color.g = 0.6;
    angular_arrow.color.b = 1.0;
    angular_arrow.pose.orientation.w = 1.0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    geometry_msgs::Point end;
    end.x = origin.x() + cartesian_cmd_angular.x() * config_.arrow_scale * 0.5;
    end.y = origin.y() + cartesian_cmd_angular.y() * config_.arrow_scale * 0.5;
    end.z = origin.z() + cartesian_cmd_angular.z() * config_.arrow_scale * 0.5;

    angular_arrow.points.push_back(start);
    angular_arrow.points.push_back(end);
    markers.markers.push_back(angular_arrow);

    // Text for angular velocity magnitude
    visualization_msgs::Marker angular_text;
    angular_text.header.stamp = stamp;
    angular_text.header.frame_id = config_.global_frame;
    angular_text.ns = makeMarkerNs(config_, "cartesian_cmd");
    angular_text.id = 3;
    angular_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    angular_text.action = visualization_msgs::Marker::ADD;
    angular_text.lifetime = ros::Duration(0.5);
    angular_text.scale.z = 0.02;
    angular_text.color.a = 0.85;
    angular_text.color.r = 0.8;
    angular_text.color.g = 0.6;
    angular_text.color.b = 1.0;
    angular_text.pose.position = end;
    angular_text.pose.position.z += 0.015;
    angular_text.pose.orientation.w = 1.0;
    std::ostringstream ss_ang;
    ss_ang.setf(std::ios::fixed);
    ss_ang.precision(3);
    ss_ang << "|w|=" << angular_speed;
    angular_text.text = ss_ang.str();
    markers.markers.push_back(angular_text);
  }

  command_marker_pub_.publish(markers);
  command_markers_active_ = true;
}

void MarkerPublisher::publishRepulsionMarkers(const ros::Time& stamp,
                                               const std::vector<ObstacleInfo>& obstacles,
                                               const std::vector<LinkPOI>& link_pois,
                                               double influence_distance,
                                               const Eigen::Vector3d& tcp_position,
                                               double tcp_radius)
{
  // Always proceed to draw TCP POI (unless everything is empty/disabled, but we assume TCP always exists)
  if (obstacles.empty() && link_pois.empty() && tcp_radius <= 0.0)
  {
    return;
  }

  visualization_msgs::MarkerArray markers;
  int id_counter = 0;

  // Helper lambda to create a sphere marker for POI with actual radius
  auto createPoiSphereWithRadius = [&](const Eigen::Vector3d& position, double poi_radius,
                                        double distance, double influence_dist,
                                        float r, float g, float b, 
                                        const std::string& ns) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker sphere;
    sphere.header.stamp = stamp;
    sphere.header.frame_id = config_.global_frame;
    sphere.ns = makeMarkerNs(config_, ns);
    sphere.id = id_counter++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.lifetime = ros::Duration(0.5);
    
    // Use actual POI radius for size (diameter = 2 * radius)
    double size = poi_radius * 2.0;
    // Ensure minimum visible size
    size = std::max(size, 0.02);
    sphere.scale.x = size;
    sphere.scale.y = size;
    sphere.scale.z = size;
    
    // Alpha based on proximity (closer = more opaque)
    double size_factor = std::clamp(1.0 - distance / influence_dist, 0.1, 1.0);
    sphere.color.a = 0.3 + 0.5 * size_factor;
    sphere.color.r = r;
    sphere.color.g = g;
    sphere.color.b = b;
    
    sphere.pose.position.x = position.x();
    sphere.pose.position.y = position.y();
    sphere.pose.position.z = position.z();
    sphere.pose.orientation.w = 1.0;
    
    return sphere;
  };

  // Helper lambda to create velocity arrow (shows actual velocity vector)
  auto createVelocityArrow = [&](const Eigen::Vector3d& origin, 
                                  const Eigen::Vector3d& velocity,
                                  float r, float g, float b,
                                  const std::string& ns) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = config_.global_frame;
    arrow.ns = makeMarkerNs(config_, ns);
    arrow.id = id_counter++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(0.5);
    arrow.scale.x = 0.01;   // shaft diameter
    arrow.scale.y = 0.02;   // head diameter
    arrow.scale.z = 0.0;
    arrow.color.a = 0.9;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.pose.orientation.w = 1.0;

    geometry_msgs::Point start;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    // Arrow length = velocity magnitude * scale
    geometry_msgs::Point end;
    end.x = origin.x() + velocity.x() * config_.arrow_scale;
    end.y = origin.y() + velocity.y() * config_.arrow_scale;
    end.z = origin.z() + velocity.z() * config_.arrow_scale;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    return arrow;
  };

  // Helper lambda to create text marker for velocity magnitude
  auto createVelocityText = [&](const Eigen::Vector3d& position, 
                                 double velocity_magnitude,
                                 const std::string& poi_name,
                                 float r, float g, float b,
                                 const std::string& ns) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = config_.global_frame;
    text.ns = makeMarkerNs(config_, ns);
    text.id = id_counter++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.lifetime = ros::Duration(0.5);
    text.scale.z = 0.02;  // text height
    text.color.a = 0.9;
    text.color.r = r;
    text.color.g = g;
    text.color.b = b;
    text.pose.position.x = position.x();
    text.pose.position.y = position.y();
    text.pose.position.z = position.z() + 0.05;  // Offset above POI
    text.pose.orientation.w = 1.0;
    
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << poi_name << ": " << velocity_magnitude << " m/s";
    text.text = ss.str();
    
    return text;
  };

  // Helper lambda to create a line marker
  auto createLine = [&](const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                        float r, float g, float b, 
                        const std::string& ns) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = config_.global_frame;
    line.ns = makeMarkerNs(config_, ns);
    line.id = id_counter++;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0.5);
    line.scale.x = 0.005; // Line width
    line.color.a = 0.6;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.pose.orientation.w = 1.0;
    
    geometry_msgs::Point p_start, p_end;
    p_start.x = start.x(); p_start.y = start.y(); p_start.z = start.z();
    p_end.x = end.x(); p_end.y = end.y(); p_end.z = end.z();
    
    line.points.push_back(p_start);
    line.points.push_back(p_end);
    
    return line;
  };

  // Helper lambda for obstacle point sphere
  auto createObstacleSphere = [&](const Eigen::Vector3d& position, 
                                  float r, float g, float b, 
                                  const std::string& ns) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker sphere;
    sphere.header.stamp = stamp;
    sphere.header.frame_id = config_.global_frame;
    sphere.ns = makeMarkerNs(config_, ns);
    sphere.id = id_counter++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.lifetime = ros::Duration(0.5);
    sphere.scale.x = 0.03; // Fixed small size
    sphere.scale.y = 0.03;
    sphere.scale.z = 0.03;
    sphere.color.a = 0.8;
    sphere.color.r = r;
    sphere.color.g = g;
    sphere.color.b = b;
    sphere.pose.position.x = position.x();
    sphere.pose.position.y = position.y();
    sphere.pose.position.z = position.z();
    sphere.pose.orientation.w = 1.0;
    return sphere;
  };

  // Find minimum distance for TCP (for alpha calculation)
  double min_tcp_dist = std::numeric_limits<double>::infinity();
  for (const auto& obs : obstacles)
  {
    if (obs.distance < min_tcp_dist)
    {
      min_tcp_dist = obs.distance;
    }
  }

  // TCP POI Sphere - Always visible using actual TCP position and radius
  markers.markers.push_back(
      createPoiSphereWithRadius(tcp_position, tcp_radius, min_tcp_dist, 
                                influence_distance, 1.0, 0.4, 0.1, "repulsion_tcp"));

  // TCP obstacles interaction lines
  for (const auto& obs : obstacles)
  {
    if (obs.distance < influence_distance)
    {
      // Direction line (Robot TCP -> Obstacle)
      markers.markers.push_back(
          createLine(tcp_position, obs.position, 0.7, 0.7, 0.7, "repulsion_tcp_dir"));
      
      // Obstacle point sphere
      markers.markers.push_back(
          createObstacleSphere(obs.position, 0.5, 0.0, 0.0, "repulsion_tcp_dir"));
    }
  }

  // Link POIs (yellow/gold spheres with actual radius + velocity arrows)
  for (const auto& poi : link_pois)
  {
    // Always show POI sphere (yellow/gold)
    // Intensity (alpha) varies with distance in createPoiSphereWithRadius
    markers.markers.push_back(
        createPoiSphereWithRadius(poi.position_world, poi.poi_radius, 
                                  poi.distance_to_closest_obstacle,
                                  influence_distance, 1.0, 0.8, 0.0, "repulsion_link"));

    if (poi.distance_to_closest_obstacle < influence_distance)
    {
      // Direction line (Robot -> Obstacle)
      Eigen::Vector3d obs_pos = poi.position_world + poi.distance_vector;
      markers.markers.push_back(
          createLine(poi.position_world, obs_pos, 0.7, 0.7, 0.7, "repulsion_link_dir"));

      // Obstacle point sphere
      markers.markers.push_back(
          createObstacleSphere(obs_pos, 0.5, 0.0, 0.0, "repulsion_link_dir"));
      
      // Repulsive velocity arrow (BEFORE Jacobian projection) - RED
      if (poi.repulsive_velocity.norm() > kEpsilon)
      {
        markers.markers.push_back(
            createVelocityArrow(poi.position_world, poi.repulsive_velocity,
                               1.0, 0.2, 0.2, "poi_repulsive_vel"));
        
        // Text showing velocity magnitude
        markers.markers.push_back(
            createVelocityText(poi.position_world, poi.repulsive_velocity_magnitude,
                              poi.point_name, 1.0, 1.0, 1.0, "poi_vel_text"));
      }
    }
  }

  if (!markers.markers.empty())
  {
    repulsion_marker_pub_.publish(markers);
    repulsion_markers_active_ = true;
  }
}

void MarkerPublisher::publishTcpFKMarker(const ros::Time& stamp,
                                          const Eigen::Isometry3d& tcp_pose)
{
  visualization_msgs::MarkerArray markers;
  int id_counter = 0;

  // Helper lambda to create sphere marker
  auto createSphere = [&](const Eigen::Vector3d& position,
                         float r, float g, float b, double size) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker sphere;
    sphere.header.stamp = stamp;
    sphere.header.frame_id = config_.global_frame;
    sphere.ns = makeMarkerNs(config_, "tcp_fk");
    sphere.id = id_counter++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.lifetime = ros::Duration(0.5);
    sphere.scale.x = size;
    sphere.scale.y = size;
    sphere.scale.z = size;
    sphere.color.a = 0.8;
    sphere.color.r = r;
    sphere.color.g = g;
    sphere.color.b = b;
    sphere.pose.position.x = position.x();
    sphere.pose.position.y = position.y();
    sphere.pose.position.z = position.z();
    sphere.pose.orientation.w = 1.0;
    return sphere;
  };

  // Helper lambda to create axis arrow for orientation visualization (XYZ triad)
  auto createAxisArrow = [&](const Eigen::Isometry3d& pose,
                            int axis, double length) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = config_.global_frame;
    arrow.ns = makeMarkerNs(config_, "tcp_fk_axis");
    arrow.id = id_counter++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(0.5);
    arrow.scale.x = 0.008;  // shaft diameter
    arrow.scale.y = 0.016;  // head diameter
    arrow.scale.z = 0.0;
    arrow.color.a = 0.95;
    
    // Set color based on axis: X=red, Y=green, Z=blue
    switch (axis)
    {
      case 0:  // X axis - Red
        arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0;
        break;
      case 1:  // Y axis - Green
        arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 0.0;
        break;
      case 2:  // Z axis - Blue
        arrow.color.r = 0.0; arrow.color.g = 0.0; arrow.color.b = 1.0;
        break;
    }
    arrow.pose.orientation.w = 1.0;

    // Get axis direction from rotation matrix
    Eigen::Vector3d axis_dir = pose.rotation().col(axis);
    Eigen::Vector3d origin = pose.translation();
    Eigen::Vector3d end = origin + axis_dir * length;

    geometry_msgs::Point p_start, p_end;
    p_start.x = origin.x(); p_start.y = origin.y(); p_start.z = origin.z();
    p_end.x = end.x(); p_end.y = end.y(); p_end.z = end.z();

    arrow.points.push_back(p_start);
    arrow.points.push_back(p_end);

    return arrow;
  };

  // TCP FK position sphere (cyan color to distinguish from other target markers)
  markers.markers.push_back(createSphere(tcp_pose.translation(), 0.0, 0.9, 0.9, 0.035));

  // XYZ triad for TCP FK orientation
  markers.markers.push_back(createAxisArrow(tcp_pose, 0, 0.1));  // X
  markers.markers.push_back(createAxisArrow(tcp_pose, 1, 0.1));  // Y
  markers.markers.push_back(createAxisArrow(tcp_pose, 2, 0.1));  // Z

  tcp_fk_marker_pub_.publish(markers);
  tcp_fk_marker_active_ = true;
}

void MarkerPublisher::clear()
{
  if (!velocity_markers_active_ && !target_markers_active_ && 
      !command_markers_active_ && !repulsion_markers_active_ && !tcp_fk_marker_active_)
  {
    return;
  }

  ros::Time now = ros::Time::now();

  // Clear velocity markers
  if (velocity_markers_active_)
  {
    visualization_msgs::MarkerArray velocity_delete;
    for (int id = 0; id <= 3; ++id)
    {
      visualization_msgs::Marker del;
      del.header.stamp = now;
      del.header.frame_id = config_.global_frame;
      del.action = visualization_msgs::Marker::DELETE;
      
      del.ns = makeMarkerNs(config_, "v_goal_linear"); del.id = id;
      velocity_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "v_goal_angular"); del.id = id;
      velocity_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "v_obs_linear"); del.id = id;
      velocity_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "v_link_linear"); del.id = id;
      velocity_delete.markers.push_back(del);
    }
    velocity_marker_pub_.publish(velocity_delete);
    velocity_markers_active_ = false;
  }

  // Clear target markers
  if (target_markers_active_)
  {
    visualization_msgs::MarkerArray target_delete;
    // Each pose now has 4 markers: 1 sphere + 3 axis arrows, total 12 markers
    for (int id = 0; id < 15; ++id)
    {
      visualization_msgs::Marker del;
      del.header.stamp = now;
      del.header.frame_id = config_.global_frame;
      del.action = visualization_msgs::Marker::DELETE;
      
      del.ns = makeMarkerNs(config_, "active_waypoint"); del.id = id;
      target_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "active_waypoint_axis"); del.id = id;
      target_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "target_raw"); del.id = id;
      target_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "target_raw_axis"); del.id = id;
      target_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "target_filtered"); del.id = id;
      target_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "target_filtered_axis"); del.id = id;
      target_delete.markers.push_back(del);
    }
    target_marker_pub_.publish(target_delete);
    target_markers_active_ = false;
  }

  // Clear command markers
  if (command_markers_active_)
  {
    visualization_msgs::MarkerArray command_delete;
    for (int id = 0; id <= 3; ++id)
    {
      visualization_msgs::Marker del;
      del.header.stamp = now;
      del.header.frame_id = config_.global_frame;
      del.ns = makeMarkerNs(config_, "cartesian_cmd");
      del.id = id;
      del.action = visualization_msgs::Marker::DELETE;
      command_delete.markers.push_back(del);
    }
    command_marker_pub_.publish(command_delete);
    command_markers_active_ = false;
  }

  // Clear repulsion markers
  if (repulsion_markers_active_)
  {
    visualization_msgs::MarkerArray repulsion_delete;
    // Clear all possible repulsion marker namespaces with generous ID range
    for (int id = 0; id < 100; ++id)
    {
      visualization_msgs::Marker del;
      del.header.stamp = now;
      del.header.frame_id = config_.global_frame;
      del.action = visualization_msgs::Marker::DELETE;
      
      del.ns = makeMarkerNs(config_, "repulsion_tcp"); del.id = id;
      repulsion_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "repulsion_tcp_dir"); del.id = id;
      repulsion_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "repulsion_link"); del.id = id;
      repulsion_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "repulsion_link_dir"); del.id = id;
      repulsion_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "poi_repulsive_vel"); del.id = id;
      repulsion_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "poi_vel_text"); del.id = id;
      repulsion_delete.markers.push_back(del);
    }
    repulsion_marker_pub_.publish(repulsion_delete);
    repulsion_markers_active_ = false;
  }

  // Clear TCP FK markers
  if (tcp_fk_marker_active_)
  {
    visualization_msgs::MarkerArray tcp_fk_delete;
    // TCP FK has 4 markers: 1 sphere + 3 axis arrows
    for (int id = 0; id < 5; ++id)
    {
      visualization_msgs::Marker del;
      del.header.stamp = now;
      del.header.frame_id = config_.global_frame;
      del.action = visualization_msgs::Marker::DELETE;
      
      del.ns = makeMarkerNs(config_, "tcp_fk"); del.id = id;
      tcp_fk_delete.markers.push_back(del);
      del.ns = makeMarkerNs(config_, "tcp_fk_axis"); del.id = id;
      tcp_fk_delete.markers.push_back(del);
    }
    tcp_fk_marker_pub_.publish(tcp_fk_delete);
    tcp_fk_marker_active_ = false;
  }
}

}  // namespace cartesian_velocity_controller
