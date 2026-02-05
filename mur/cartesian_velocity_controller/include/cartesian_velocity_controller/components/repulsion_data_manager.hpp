#pragma once

/**
 * @file repulsion_data_manager.hpp
 * @brief Manager for repulsive POI data using the local 3D distance map.
 *
 * The RepulsionDataManager:
 * - Loads POI definitions (link + offset) from ROS params
 * - Loads per-POI config (weight, radius, enabled, is_tcp) from ROS params / dynamic_reconfigure
 * - Queries the in-process Map3D (EDT + gradient) to produce ObstacleInfo/LinkPOI for LocalPlanner
 */

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "cartesian_velocity_controller/types/config_types.hpp"
#include "cartesian_velocity_controller/types/pipeline_types.hpp"

namespace cartesian_velocity_controller
{

// Forward declarations
class RobotStateManager;
namespace map3d
{
class Map3DManager;
}

/**
 * @class RepulsionDataManager
 * @brief Manages repulsive POI data and configuration.
 *
 * This class bridges the gap between Map3D (distance field) and the LocalPlanner's
 * repulsion system. It:
 * 1. Computes POI world positions from RobotState (link + offset)
 * 2. Queries Map3D for distance + gradient
 * 3. Converts to ObstacleInfo/LinkPOI based on configuration
 * 4. Allows runtime modification of POI parameters
 */
class RepulsionDataManager
{
public:
  /**
   * @brief Construct a RepulsionDataManager.
   * @param nh NodeHandle for subscriptions
   * @param robot_state Shared pointer to robot state (for position_link computation)
   */
  RepulsionDataManager(ros::NodeHandle& nh, 
                       std::shared_ptr<RobotStateManager> robot_state = nullptr);

  /**
   * @brief Destructor.
   */
  ~RepulsionDataManager() = default;

  // ============== Configuration ==============

  /**
   * @brief Load POI configuration from ROS parameters.
   * @param pnh Private NodeHandle for parameter loading
   */
  void loadConfig(ros::NodeHandle& pnh);

  /**
   * @brief Inject Map3DManager dependency (must outlive this manager).
   */
  void setMap3DManager(std::shared_ptr<map3d::Map3DManager> map_manager);

  /**
   * @brief Set configuration for a specific POI.
   * @param name POI name (e.g., "tcp", "elbow")
   * @param config Configuration to set
   */
  void setPointConfig(const std::string& name, const RepulsivePointConfig& config);

  /**
   * @brief Get configuration for a specific POI.
   * @param name POI name
   * @return Configuration (default if not found)
   */
  RepulsivePointConfig getPointConfig(const std::string& name) const;

  /**
   * @brief Get all configured POI names.
   * @return Vector of POI names
   */
  std::vector<std::string> getPointNames() const;

  // ============== Runtime Parameter Updates ==============

  /**
   * @brief Set weight for a specific POI.
   * @param name POI name
   * @param weight New weight value (0.0 - 2.0)
   */
  void setPointWeight(const std::string& name, double weight);

  /**
   * @brief Set radius for a specific POI.
   * @param name POI name
   * @param radius New radius value in meters
   */
  void setPointRadius(const std::string& name, double radius);

  /**
   * @brief Enable/disable a specific POI.
   * @param name POI name
   * @param enabled Enable state
   */
  void setPointEnabled(const std::string& name, bool enabled);

  // ============== Data Retrieval ==============

  /**
   * @brief Get repulsion data for the LocalPlanner.
   * @param obstacles_out Output vector for TCP/payload obstacles (ObstacleInfo)
   * @param link_pois_out Output vector for link POIs (LinkPOI)
   * @param current_tcp_pose Current TCP pose (for reference)
   * @param global_frame Frame for coordinate system
   *
   * This method processes the latest /robot_points_info message and produces
   * obstacles and link_pois vectors ready for LocalPlanner::compute().
   */
  void getRepulsionData(
      std::vector<ObstacleInfo>& obstacles_out,
      std::vector<LinkPOI>& link_pois_out,
      const Eigen::Isometry3d& current_tcp_pose,
      const std::string& global_frame,
      double dt);

  // ============== Smoothing / Prediction Parameters ==============

  /// 0 = disabled (use raw), (0..1] = EMA weight for raw gradient
  void setGradientFilterAlpha(double alpha);
  double getGradientFilterAlpha() const;

  void setPoiPredictEnabled(bool enabled);
  bool getPoiPredictEnabled() const;

  /// If <= 0, a default based on map update rate is used (clamped internally).
  void setPoiPredictHorizon(double seconds);
  double getPoiPredictHorizon() const;

  /// If <= 0, velocity filtering is disabled (raw finite-difference).
  void setPoiVelocityFilterTau(double seconds);
  double getPoiVelocityFilterTau() const;

  void setPoiVelocityMax(double mps);
  double getPoiVelocityMax() const;

  void setPoiPredictConservativeMinDistance(bool enabled);
  bool getPoiPredictConservativeMinDistance() const;

  // ============== Data Validity ==============

  /**
   * @brief Check if valid data is available.
   * @return true if recent data exists and is not stale
   */
  bool hasValidData() const;

  /**
   * @brief Get timestamp of the last received message.
   * @return ROS time of last message
   */
  ros::Time getLastDataTimestamp() const;

  /**
   * @brief Set the stale timeout.
   * @param timeout Timeout in seconds
   */
  void setStaleTimeout(double timeout);

  /**
   * @brief Get the stale timeout.
   * @return Timeout in seconds
   */
  double getStaleTimeout() const;

private:
  struct PointDefinition
  {
    std::string link_name;
    Eigen::Vector3d offset_link{Eigen::Vector3d::Zero()};  // in link frame
  };

  struct PoiMotionState
  {
    bool has_prev{false};
    Eigen::Vector3d prev_position_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_velocity_world{Eigen::Vector3d::Zero()};
  };

  // ============== ROS ==============
  ros::NodeHandle nh_;

  // ============== Dependencies ==============
  std::shared_ptr<RobotStateManager> robot_state_;
  std::shared_ptr<map3d::Map3DManager> map3d_manager_;

  // ============== POI Configuration ==============
  std::map<std::string, RepulsivePointConfig> point_configs_;
  std::map<std::string, PointDefinition> point_definitions_;
  mutable std::mutex config_mutex_;

  // Gradient stability (fallback near contact)
  mutable std::mutex gradient_mutex_;
  std::map<std::string, Eigen::Vector3d> last_valid_gradients_world_;
  std::map<std::string, Eigen::Vector3d> filtered_gradients_world_;

  // POI motion state (for predictive query)
  mutable std::mutex poi_motion_mutex_;
  std::map<std::string, PoiMotionState> poi_motion_state_;

  // ============== Parameters ==============
  double stale_timeout_{0.3};       ///< Timeout for stale map data (seconds)
  std::string tcp_point_name_{"tcp"};  ///< Name of the POI acting as TCP

  // Gradient smoothing (3.6): alpha <= 0 disables filtering (raw gradient)
  double gradient_filter_alpha_{0.0};

  // Predictive query (3.7) - POI only
  bool poi_predict_enable_{false};
  double poi_predict_horizon_{0.06};         // [s]
  double poi_velocity_filter_tau_{0.05};     // [s]
  double poi_velocity_max_{1.5};             // [m/s]
  bool poi_predict_conservative_min_distance_{true};
};

}  // namespace cartesian_velocity_controller
