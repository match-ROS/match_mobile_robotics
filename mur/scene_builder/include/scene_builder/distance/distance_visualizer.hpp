/**
 * @file distance_visualizer.hpp
 * @brief RViz marker visualization for distance results
 *
 * This file provides the DistanceVisualizer class that creates RViz markers
 * to visualize distance computation results.
 */

#ifndef SCENE_BUILDER_DISTANCE_DISTANCE_VISUALIZER_HPP
#define SCENE_BUILDER_DISTANCE_DISTANCE_VISUALIZER_HPP

#include "scene_builder/distance/distance_calculator.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>

#include <string>
#include <vector>

namespace scene_builder
{
namespace distance
{

/**
 * @brief Configuration for distance visualization
 */
struct VisualizationConfig
{
  double arrow_length = 0.15;           ///< Length of direction arrows (m)
  double arrow_shaft_diameter = 0.01;   ///< Diameter of arrow shaft (m)
  double arrow_head_diameter = 0.02;    ///< Diameter of arrow head (m)
  double arrow_head_length = 0.03;      ///< Length of arrow head (m)
  double point_radius = 0.015;          ///< Radius of point markers (m)

  // Colors (RGBA)
  double robot_point_r = 1.0;           ///< Robot point color - Red
  double robot_point_g = 0.0;           ///< Robot point color - Green
  double robot_point_b = 0.0;           ///< Robot point color - Blue
  double robot_point_a = 1.0;           ///< Robot point color - Alpha

  double object_point_r = 0.0;          ///< Object point color - Red
  double object_point_g = 0.2;          ///< Object point color - Green
  double object_point_b = 1.0;          ///< Object point color - Blue
  double object_point_a = 1.0;          ///< Object point color - Alpha

  double arrow_r = 1.0;                 ///< Arrow color - Red
  double arrow_g = 0.65;                ///< Arrow color - Green
  double arrow_b = 0.0;                 ///< Arrow color - Blue (orange)
  double arrow_a = 1.0;                 ///< Arrow color - Alpha
};

/**
 * @brief Creates RViz markers for distance visualization
 *
 * DistanceVisualizer converts distance computation results into
 * visualization markers for RViz display.
 *
 * Marker types:
 * - Point markers on robot (configurable color, default red)
 * - Point markers on obstacles (configurable color, default blue)
 * - Arrow markers showing distance direction (orange)
 */
class DistanceVisualizer
{
public:
  /**
   * @brief Constructor
   * @param config Visualization configuration
   */
  explicit DistanceVisualizer(const VisualizationConfig& config = {});

  /**
   * @brief Creates markers for distance results
   * @param distances Vector of distance results to visualize
   * @param frame_id Reference frame for markers
   * @param stamp Timestamp for markers
   * @return MarkerArray ready for publishing
   */
  visualization_msgs::MarkerArray createMarkers(
      const std::vector<DistanceResult>& distances,
      const std::string& frame_id,
      const ros::Time& stamp);

  /**
   * @brief Creates a marker array that clears all previous markers
   * @return MarkerArray with DELETEALL action
   */
  visualization_msgs::MarkerArray createClearMarkers();

  /**
   * @brief Sets the visualization configuration
   * @param config New configuration
   */
  void setConfig(const VisualizationConfig& config) { config_ = config; }

  /**
   * @brief Gets the current configuration
   * @return Current visualization configuration
   */
  const VisualizationConfig& getConfig() const { return config_; }

private:
  /**
   * @brief Creates a sphere marker for a point
   * @param point Position of the point
   * @param id Marker ID
   * @param ns Marker namespace
   * @param frame_id Reference frame
   * @param stamp Timestamp
   * @param r Red color component
   * @param g Green color component
   * @param b Blue color component
   * @param a Alpha (transparency)
   * @return Configured marker
   */
  visualization_msgs::Marker createPointMarker(
      const Eigen::Vector3d& point,
      int id,
      const std::string& ns,
      const std::string& frame_id,
      const ros::Time& stamp,
      double r, double g, double b, double a);

  /**
   * @brief Creates an arrow marker
   * @param from Start point of arrow
   * @param to End point (tip) of arrow
   * @param id Marker ID
   * @param ns Marker namespace
   * @param frame_id Reference frame
   * @param stamp Timestamp
   * @return Configured arrow marker
   */
  visualization_msgs::Marker createArrowMarker(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      int id,
      const std::string& ns,
      const std::string& frame_id,
      const ros::Time& stamp);

  VisualizationConfig config_;  ///< Current visualization configuration
};

}  // namespace distance
}  // namespace scene_builder

#endif  // SCENE_BUILDER_DISTANCE_DISTANCE_VISUALIZER_HPP

