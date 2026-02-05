#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "cartesian_velocity_controller/types/config_types.hpp"
#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

/**
 * @brief Component responsible for publishing RViz visualization markers.
 * 
 * Publishes three separate topics:
 * - velocity_markers: 3 linear velocity vectors + 1 angular (v_goal, v_obs, v_link)
 * - target_markers: 3 target points (active_waypoint, target_raw, target_filtered)
 * - command_markers: final cartesian command with magnitude labels
 */
class MarkerPublisher
{
public:
  /// Small value for numerical comparisons
  static constexpr double kEpsilon = 1e-6;

  /**
   * @brief Construct a MarkerPublisher.
   * @param nh NodeHandle for creating the publishers
   * @param config Configuration for marker visualization
   */
  MarkerPublisher(ros::NodeHandle& nh, const MarkerPublisherConfig& config = {});

  /**
   * @brief Update the configuration.
   * @param config New configuration
   */
  void setConfig(const MarkerPublisherConfig& config);

  /**
   * @brief Get the current configuration.
   * @return Current configuration
   */
  const MarkerPublisherConfig& getConfig() const { return config_; }

  /**
   * @brief Publish velocity component markers.
   * 
   * Shows the 3 velocity components that form the desired velocity:
   * V_desired = k_att * V_goal + k_rep_tcp * V_obs + k_rep_links * V_link
   * 
   * @param stamp Timestamp for the markers
   * @param origin Position from which to draw the arrows (TCP position)
   * @param v_goal_linear Attractive linear velocity toward waypoint
   * @param v_goal_angular Attractive angular velocity toward waypoint
   * @param v_obs_linear Repulsive linear velocity from TCP obstacles
   * @param v_link_linear Repulsive linear velocity from link obstacles
   */
  void publishVelocityMarkers(const ros::Time& stamp,
                              const Eigen::Vector3d& origin,
                              const Eigen::Vector3d& v_goal_linear,
                              const Eigen::Vector3d& v_goal_angular,
                              const Eigen::Vector3d& v_obs_linear,
                              const Eigen::Vector3d& v_link_linear);

  /**
   * @brief Publish target point markers.
   * 
   * Shows 3 key target points as spheres:
   * - active_waypoint: current waypoint from global planner
   * - target_raw: raw target from local planner before filtering
   * - target_filtered: filtered target after motion generator
   * 
   * @param stamp Timestamp for the markers
   * @param active_waypoint Current waypoint pose
   * @param target_raw Raw target pose from local planner
   * @param target_filtered Filtered target pose from motion generator
   */
  void publishTargetMarkers(const ros::Time& stamp,
                            const Eigen::Isometry3d& active_waypoint,
                            const Eigen::Isometry3d& target_raw,
                            const Eigen::Isometry3d& target_filtered);

  /**
   * @brief Publish cartesian command markers with magnitude labels.
   * 
   * Shows the final cartesian velocity command sent to inverse kinematics.
   * This is the only marker that displays magnitude values.
   * 
   * @param stamp Timestamp for the markers
   * @param origin Position from which to draw the arrows (TCP position)
   * @param cartesian_cmd_linear Final linear velocity command
   * @param cartesian_cmd_angular Final angular velocity command
   */
  void publishCommandMarkers(const ros::Time& stamp,
                             const Eigen::Vector3d& origin,
                             const Eigen::Vector3d& cartesian_cmd_linear,
                             const Eigen::Vector3d& cartesian_cmd_angular);

  /**
   * @brief Publish repulsive POI markers.
   * 
   * Shows active repulsive Points of Interest with:
   * - Sphere indicating the POI position and influence radius
   * - Arrow showing repulsive direction
   * 
   * @param stamp Timestamp for the markers
   * @param obstacles Active TCP obstacles (ObstacleInfo)
   * @param link_pois Active link POIs (LinkPOI)
   * @param influence_distance Global influence distance for reference
   * @param tcp_position Current TCP position (for always visible visualization)
   * @param tcp_radius Radius of the TCP POI
   */
  void publishRepulsionMarkers(const ros::Time& stamp,
                               const std::vector<ObstacleInfo>& obstacles,
                               const std::vector<LinkPOI>& link_pois,
                               double influence_distance,
                               const Eigen::Vector3d& tcp_position,
                               double tcp_radius);

  /**
   * @brief Publish TCP forward kinematics pose marker.
   * 
   * Shows the current TCP pose calculated via forward kinematics:
   * - Sphere indicating the TCP position
   * - XYZ triad showing the TCP orientation
   * 
   * @param stamp Timestamp for the markers
   * @param tcp_pose Current TCP pose from forward kinematics (in global frame)
   */
  void publishTcpFKMarker(const ros::Time& stamp,
                          const Eigen::Isometry3d& tcp_pose);

  /**
   * @brief Clear all markers.
   * 
   * Sends DELETE actions for all marker IDs to remove them from RViz.
   */
  void clear();

  /**
   * @brief Check if there are currently active markers.
   * @return true if markers are being displayed
   */
  bool hasActiveMarkers() const { return velocity_markers_active_ || target_markers_active_ || command_markers_active_ || repulsion_markers_active_ || tcp_fk_marker_active_; }

private:
  ros::Publisher velocity_marker_pub_;   ///< Publisher for velocity component markers
  ros::Publisher target_marker_pub_;     ///< Publisher for target point markers
  ros::Publisher command_marker_pub_;    ///< Publisher for cartesian command markers
  ros::Publisher repulsion_marker_pub_;  ///< Publisher for repulsion POI markers
  ros::Publisher tcp_fk_marker_pub_;     ///< Publisher for TCP FK pose marker
  
  MarkerPublisherConfig config_;
  
  bool velocity_markers_active_{false};
  bool target_markers_active_{false};
  bool command_markers_active_{false};
  bool repulsion_markers_active_{false};
  bool tcp_fk_marker_active_{false};
};

}  // namespace cartesian_velocity_controller
