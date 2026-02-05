/**
 * @file distance_visualizer.cpp
 * @brief Implementation of RViz marker visualization
 */

#include "scene_builder/distance/distance_visualizer.hpp"

namespace scene_builder
{
namespace distance
{

DistanceVisualizer::DistanceVisualizer(const VisualizationConfig& config)
  : config_(config)
{
}

visualization_msgs::MarkerArray DistanceVisualizer::createMarkers(
    const std::vector<DistanceResult>& distances,
    const std::string& frame_id,
    const ros::Time& stamp)
{
  visualization_msgs::MarkerArray markers;

  // Add clear marker first
  visualization_msgs::Marker clear_marker;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);

  int marker_id = 0;

  for (const auto& dist : distances)
  {
    // Robot point marker (red by default)
    markers.markers.push_back(createPointMarker(
        dist.robot_point,
        marker_id++,
        "nearest_points",
        frame_id,
        stamp,
        config_.robot_point_r,
        config_.robot_point_g,
        config_.robot_point_b,
        config_.robot_point_a));

    // Object point marker (blue by default)
    markers.markers.push_back(createPointMarker(
        dist.object_point,
        marker_id++,
        "nearest_points",
        frame_id,
        stamp,
        config_.object_point_r,
        config_.object_point_g,
        config_.object_point_b,
        config_.object_point_a));

    // Arrow marker showing direction
    const double direction_norm = dist.distance_vector.norm();
    if (direction_norm > 1e-6)
    {
      const Eigen::Vector3d direction_unit = dist.distance_vector / direction_norm;
      Eigen::Vector3d arrow_tail = dist.robot_point - direction_unit * config_.arrow_length;

      markers.markers.push_back(createArrowMarker(
          arrow_tail,
          dist.robot_point,
          marker_id++,
          "nearest_points_arrows",
          frame_id,
          stamp));
    }
  }

  return markers;
}

visualization_msgs::MarkerArray DistanceVisualizer::createClearMarkers()
{
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);
  return markers;
}

visualization_msgs::Marker DistanceVisualizer::createPointMarker(
    const Eigen::Vector3d& point,
    int id,
    const std::string& ns,
    const std::string& frame_id,
    const ros::Time& stamp,
    double r, double g, double b, double a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.w = 1.0;

  const double diameter = 2.0 * config_.point_radius;
  marker.scale.x = diameter;
  marker.scale.y = diameter;
  marker.scale.z = diameter;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.lifetime = ros::Duration(0.0);

  return marker;
}

visualization_msgs::Marker DistanceVisualizer::createArrowMarker(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    int id,
    const std::string& ns,
    const std::string& frame_id,
    const ros::Time& stamp)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = config_.arrow_shaft_diameter;
  marker.scale.y = config_.arrow_head_diameter;
  marker.scale.z = config_.arrow_head_length;

  marker.color.r = config_.arrow_r;
  marker.color.g = config_.arrow_g;
  marker.color.b = config_.arrow_b;
  marker.color.a = config_.arrow_a;

  marker.lifetime = ros::Duration(0.0);

  geometry_msgs::Point tail;
  tail.x = from.x();
  tail.y = from.y();
  tail.z = from.z();
  marker.points.push_back(tail);

  geometry_msgs::Point head;
  head.x = to.x();
  head.y = to.y();
  head.z = to.z();
  marker.points.push_back(head);

  return marker;
}

}  // namespace distance
}  // namespace scene_builder

