/**
 * @file repulsion_data_manager.cpp
 * @brief Implementation of the RepulsionDataManager.
 */

#include "cartesian_velocity_controller/components/repulsion_data_manager.hpp"
#include "cartesian_velocity_controller/components/robot_state_manager.hpp"

#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>

#include "cartesian_velocity_controller/map3d/map3d_manager.hpp"

namespace cartesian_velocity_controller
{

RepulsionDataManager::RepulsionDataManager(
    ros::NodeHandle& nh,
    std::shared_ptr<RobotStateManager> robot_state)
  : nh_(nh)
  , robot_state_(robot_state)
{
  ROS_INFO_NAMED("repulsion_data_manager",
                 "RepulsionDataManager initialized (Map3D-based)");
}

void RepulsionDataManager::loadConfig(ros::NodeHandle& pnh)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  // Load general repulsion parameters
  pnh.param("repulsion/stale_timeout", stale_timeout_, 0.3);
  pnh.param("repulsion/tcp_point_name", tcp_point_name_, std::string("tcp"));

  // 3.6: Gradient smoothing (0 = disabled -> use raw)
  pnh.param("repulsion/gradient_filter_alpha", gradient_filter_alpha_, 0.0);
  gradient_filter_alpha_ = std::clamp(gradient_filter_alpha_, 0.0, 1.0);

  // 3.7: Predictive query (POI only)
  pnh.param("repulsion/poi_predict_enable", poi_predict_enable_, false);
  pnh.param("repulsion/poi_predict_horizon", poi_predict_horizon_, 0.06);
  pnh.param("repulsion/poi_velocity_filter_tau", poi_velocity_filter_tau_, 0.05);
  pnh.param("repulsion/poi_velocity_max", poi_velocity_max_, 1.5);
  pnh.param("repulsion/poi_predict_conservative_min_distance", poi_predict_conservative_min_distance_, true);

  poi_predict_horizon_ = std::max(0.0, poi_predict_horizon_);
  poi_velocity_filter_tau_ = std::max(0.0, poi_velocity_filter_tau_);
  poi_velocity_max_ = std::max(0.0, poi_velocity_max_);

  // Load POI definitions (link + offset)
  // Expected format (same as scene_builder):
  // repulsion/robot_points_of_interest/<name>/link: "tool0"
  // repulsion/robot_points_of_interest/<name>/offset: [x,y,z]
  point_definitions_.clear();
  {
    XmlRpc::XmlRpcValue defs;
    if (pnh.getParam("repulsion/robot_points_of_interest", defs) && defs.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for (auto it = defs.begin(); it != defs.end(); ++it)
      {
        const std::string poi_name = static_cast<std::string>(it->first);
        const XmlRpc::XmlRpcValue& def = it->second;
        if (def.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;

        PointDefinition pd;
        if (def.hasMember("link"))
          pd.link_name = static_cast<std::string>(def["link"]);
        if (def.hasMember("offset") && def["offset"].getType() == XmlRpc::XmlRpcValue::TypeArray &&
            def["offset"].size() == 3)
        {
          pd.offset_link = Eigen::Vector3d(static_cast<double>(def["offset"][0]),
                                           static_cast<double>(def["offset"][1]),
                                           static_cast<double>(def["offset"][2]));
        }
        point_definitions_[poi_name] = pd;
      }
    }
  }

  // Fallback defaults (UR10e) if not provided
  if (point_definitions_.empty())
  {
    point_definitions_["tcp"] = PointDefinition{"tool0", Eigen::Vector3d(0.0, 0.0, 0.0)};
    point_definitions_["elbow"] = PointDefinition{"forearm_link", Eigen::Vector3d(0.0, 0.0, 0.12)};
    point_definitions_["wrist"] = PointDefinition{"wrist_1_link", Eigen::Vector3d(0.0, 0.0, -0.05)};
    point_definitions_["forearm_mid"] = PointDefinition{"forearm_link", Eigen::Vector3d(-0.30, 0.0, 0.035)};
    ROS_WARN_NAMED("repulsion_data_manager",
                   "repulsion/robot_points_of_interest not provided; using UR10e default POI definitions");
  }
  
  // Load POI configurations
  // We expect parameters like:
  // repulsion/points/tcp/weight: 1.0
  // repulsion/points/tcp/radius: 0.05
  // repulsion/points/tcp/enabled: false
  // repulsion/points/tcp/is_tcp: true
  
  const std::vector<std::string> default_pois = {"tcp", "elbow", "wrist", "forearm_mid"};
  
  for (const auto& poi_name : default_pois)
  {
    RepulsivePointConfig config;
    config.name = poi_name;
    
    std::string prefix = "repulsion/points/" + poi_name + "/";
    
    pnh.param(prefix + "weight", config.weight, 1.0);
    pnh.param(prefix + "radius", config.radius, 0.05);
    pnh.param(prefix + "enabled", config.enabled, false);
    pnh.param(prefix + "is_tcp", config.is_tcp, poi_name == "tcp");
    
    // Clamp values
    config.weight = std::clamp(config.weight, 0.0, 2.0);
    config.radius = std::clamp(config.radius, 0.0, 0.5);
    
    point_configs_[poi_name] = config;
    
    ROS_DEBUG_NAMED("repulsion_data_manager",
                    "Loaded POI '%s': weight=%.2f, radius=%.3f, enabled=%d, is_tcp=%d",
                    poi_name.c_str(), config.weight, config.radius, 
                    config.enabled, config.is_tcp);
  }

  ROS_INFO_NAMED("repulsion_data_manager",
                 "Loaded %zu POI configurations from parameters",
                 point_configs_.size());
}

void RepulsionDataManager::setMap3DManager(std::shared_ptr<map3d::Map3DManager> map_manager)
{
  map3d_manager_ = std::move(map_manager);
}

void RepulsionDataManager::setPointConfig(const std::string& name, 
                                          const RepulsivePointConfig& config)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  point_configs_[name] = config;
}

RepulsivePointConfig RepulsionDataManager::getPointConfig(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  auto it = point_configs_.find(name);
  if (it != point_configs_.end())
  {
    return it->second;
  }
  
  // Return default config
  RepulsivePointConfig default_config;
  default_config.name = name;
  return default_config;
}

std::vector<std::string> RepulsionDataManager::getPointNames() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  std::vector<std::string> names;
  names.reserve(point_configs_.size());
  
  for (const auto& pair : point_configs_)
  {
    names.push_back(pair.first);
  }
  
  return names;
}

void RepulsionDataManager::setPointWeight(const std::string& name, double weight)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  auto it = point_configs_.find(name);
  if (it != point_configs_.end())
  {
    it->second.weight = std::clamp(weight, 0.0, 2.0);
  }
}

void RepulsionDataManager::setPointRadius(const std::string& name, double radius)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  auto it = point_configs_.find(name);
  if (it != point_configs_.end())
  {
    it->second.radius = std::clamp(radius, 0.0, 0.5);
  }
}

void RepulsionDataManager::setPointEnabled(const std::string& name, bool enabled)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  auto it = point_configs_.find(name);
  if (it != point_configs_.end())
  {
    it->second.enabled = enabled;
  }
}

void RepulsionDataManager::getRepulsionData(
    std::vector<ObstacleInfo>& obstacles_out,
    std::vector<LinkPOI>& link_pois_out,
    const Eigen::Isometry3d& /*current_tcp_pose*/,
    const std::string& global_frame,
    double dt)
{
  obstacles_out.clear();
  link_pois_out.clear();
  
  // Check for stale data / dependencies
  if (!hasValidData())
  {
    return;
  }
  
  std::map<std::string, RepulsivePointConfig> configs_copy;
  std::map<std::string, PointDefinition> defs_copy;
  
  {
    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
    configs_copy = point_configs_;
    defs_copy = point_definitions_;
  }

  // Robot state snapshot (for link transforms)
  moveit::core::RobotState state_copy = robot_state_->getRobotStateCopy();

  // Process all configured points (no topic-driven filtering)
  for (const auto& pair : configs_copy)
  {
    const std::string& poi_name = pair.first;
    const RepulsivePointConfig& config = pair.second;
    if (!config.enabled)
    {
      continue;
    }
    
    const auto def_it = defs_copy.find(poi_name);
    if (def_it == defs_copy.end() || def_it->second.link_name.empty())
    {
      ROS_WARN_THROTTLE_NAMED(2.0, "repulsion_data_manager",
                              "No POI definition for '%s' (repulsion/robot_points_of_interest/%s)",
                              poi_name.c_str(), poi_name.c_str());
      continue;
    }

    const PointDefinition& def = def_it->second;

    // Compute POI world position from link transform + offset
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    try
    {
      const Eigen::Isometry3d& T_world_link = state_copy.getGlobalLinkTransform(def.link_name);
      position_world = T_world_link * def.offset_link;
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE_NAMED(2.0, "repulsion_data_manager",
                              "Failed to get link transform for '%s' (POI '%s'): %s",
                              def.link_name.c_str(), poi_name.c_str(), e.what());
      continue;
    }

    // Always query at current POI position
    const map3d::QueryResult q_now = map3d_manager_->queryWorld(position_world, global_frame);
    if (!q_now.valid)
    {
      // If current query is invalid, we cannot safely use prediction either (would be frame/TF dependent)
      continue;
    }

    double d_edt_used = q_now.distance;
    Eigen::Vector3d grad = q_now.gradient;  // towards free space (world), normalized

    // 3.7: Predictive query (POI only), conservative distance (min)
    bool do_predict = false;
    bool conservative_min = true;
    double horizon = 0.0;
    double vel_tau = 0.0;
    double vel_max = 0.0;
    {
      std::lock_guard<std::mutex> cfg_lock(config_mutex_);
      do_predict = poi_predict_enable_;
      conservative_min = poi_predict_conservative_min_distance_;
      horizon = poi_predict_horizon_;
      vel_tau = poi_velocity_filter_tau_;
      vel_max = poi_velocity_max_;
    }

    if (do_predict && dt > 0.0)
    {
      PoiMotionState state;
      {
        std::lock_guard<std::mutex> m_lock(poi_motion_mutex_);
        state = poi_motion_state_[poi_name];
      }

      Eigen::Vector3d v_raw = Eigen::Vector3d::Zero();
      if (state.has_prev)
      {
        v_raw = (position_world - state.prev_position_world) / dt;
      }

      // Filter POI velocity (EMA with tau)
      Eigen::Vector3d v_filt = v_raw;
      if (vel_tau > 1e-12)
      {
        const double alpha = std::clamp(dt / (vel_tau + dt), 0.0, 1.0);
        v_filt = (alpha * v_raw) + ((1.0 - alpha) * state.filtered_velocity_world);
      }

      // Clamp POI velocity
      const double v_n = v_filt.norm();
      if (vel_max > 1e-12 && v_n > vel_max)
      {
        v_filt *= (vel_max / v_n);
      }

      // Update motion state
      {
        std::lock_guard<std::mutex> m_lock(poi_motion_mutex_);
        auto& st = poi_motion_state_[poi_name];
        st.has_prev = true;
        st.prev_position_world = position_world;
        st.filtered_velocity_world = v_filt;
      }

      // Horizon selection:
      // - if horizon <= 0: use map update period
      // - clamp to a conservative window
      const auto map_cfg = map3d_manager_->getConfig();
      const double map_T = (map_cfg.update_rate_hz > 1e-6) ? (1.0 / map_cfg.update_rate_hz) : 0.06;
      const double horizon_raw = (horizon > 1e-12) ? horizon : map_T;
      const double horizon_eff = std::clamp(horizon_raw, 0.02, 0.08);

      const Eigen::Vector3d p_pred = position_world + v_filt * horizon_eff;
      const map3d::QueryResult q_pred = map3d_manager_->queryWorld(p_pred, global_frame);
      if (q_pred.valid && conservative_min)
      {
        d_edt_used = std::min(q_now.distance, q_pred.distance);
      }
      // Note: gradient is kept from q_now for geometric consistency at current position.
    }

    // Clamp gradient near contact / degenerate gradient
    const double grad_norm = grad.norm();
    const double clamp_dist = map3d_manager_->getConfig().gradient_clamp_distance;
    const double grad_eps = map3d_manager_->getConfig().gradient_eps;

    bool accept_grad = (grad_norm > grad_eps) && (d_edt_used > clamp_dist);
    if (accept_grad)
    {
      grad /= grad_norm;
    }
    else
    {
      std::lock_guard<std::mutex> g_lock(gradient_mutex_);
      auto itg = last_valid_gradients_world_.find(poi_name);
      if (itg != last_valid_gradients_world_.end())
      {
        grad = itg->second;
      }
      else if (grad_norm > grad_eps)
      {
        grad /= grad_norm;
      }
      // else keep q.gradient fallback (can be zero only if center fallback failed)
    }

    // 3.6: EMA filter on gradient direction (per POI)
    double alpha_raw = 0.0;
    {
      std::lock_guard<std::mutex> cfg_lock(config_mutex_);
      alpha_raw = gradient_filter_alpha_;
    }

    if (alpha_raw > 1e-12)
    {
      const double a = std::clamp(alpha_raw, 0.0, 1.0);
      std::lock_guard<std::mutex> g_lock(gradient_mutex_);
      auto& g_prev = filtered_gradients_world_[poi_name];
      if (g_prev.norm() < grad_eps)
      {
        g_prev = grad;
      }
      Eigen::Vector3d g_mix = (1.0 - a) * g_prev + a * grad;
      const double n = g_mix.norm();
      if (n > grad_eps)
      {
        g_mix /= n;
        grad = g_mix;
        g_prev = g_mix;
      }
    }

    // Store last valid (filtered) gradient for fallback near contact
    if (grad.norm() > grad_eps)
    {
      std::lock_guard<std::mutex> g_lock(gradient_mutex_);
      last_valid_gradients_world_[poi_name] = grad;
    }

    // Effective distance: point robot with POI radius subtraction
    const double min_eps = map3d_manager_->getConfig().min_distance_eps;
    const double d_effective = std::max(min_eps, d_edt_used - config.radius);

    // Closest point on (inflated) obstacle surface (approx)
    const Eigen::Vector3d closest_point_world = position_world - grad * d_edt_used;
    
    if (config.is_tcp)
    {
      // === Generate ObstacleInfo for TCP POI ===
      ObstacleInfo obs;
      obs.id = "map3d";
      obs.position = closest_point_world;  // Closest point on obstacle surface (world)
      obs.distance = d_effective;
      obs.distance_raw = d_edt_used;
      obs.object_characteristic_radius = 0.0;
      obs.poi_radius = config.radius;
      
      // distance_vector: obstacle -> TCP (away from obstacle), magnitude = effective distance
      const Eigen::Vector3d v = position_world - closest_point_world;
      const double n = v.norm();
      if (n > 1e-10)
      {
        obs.distance_vector = (v / n) * d_effective;
      }
      else
      {
        obs.distance_vector = Eigen::Vector3d::Zero();
      }
      
      obstacles_out.push_back(obs);
    }
    else
    {
      // === Generate LinkPOI ===
      LinkPOI poi;
      poi.point_name = poi_name;
      poi.link_name = def.link_name;
      poi.position_world = position_world;
      poi.position_link = def.offset_link;
      poi.distance_to_closest_obstacle = d_effective;
      poi.distance_raw = d_edt_used;
      poi.distance_vector = closest_point_world - position_world;  // POI -> closest point
      poi.closest_obstacle_id = "map3d";
      poi.weight = config.weight;
      poi.poi_radius = config.radius;
      poi.object_characteristic_radius = 0.0;
      
      // Repulsive direction: away from obstacle
      poi.repulsive_direction = grad;
      
      // Note: repulsive_velocity and repulsive_velocity_magnitude will be
      // computed later by LocalPlanner::computeRepulsiveLinkJointVelocity()
      
      link_pois_out.push_back(poi);
    }
  }
  
  ROS_DEBUG_THROTTLE_NAMED(1.0, "repulsion_data_manager",
                           "Generated %zu obstacles, %zu link POIs",
                           obstacles_out.size(), link_pois_out.size());
}

void RepulsionDataManager::setGradientFilterAlpha(double alpha)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  gradient_filter_alpha_ = std::clamp(alpha, 0.0, 1.0);
}

double RepulsionDataManager::getGradientFilterAlpha() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return gradient_filter_alpha_;
}

void RepulsionDataManager::setPoiPredictEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  poi_predict_enable_ = enabled;
}

bool RepulsionDataManager::getPoiPredictEnabled() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return poi_predict_enable_;
}

void RepulsionDataManager::setPoiPredictHorizon(double seconds)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  poi_predict_horizon_ = std::max(0.0, seconds);
}

double RepulsionDataManager::getPoiPredictHorizon() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return poi_predict_horizon_;
}

void RepulsionDataManager::setPoiVelocityFilterTau(double seconds)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  poi_velocity_filter_tau_ = std::max(0.0, seconds);
}

double RepulsionDataManager::getPoiVelocityFilterTau() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return poi_velocity_filter_tau_;
}

void RepulsionDataManager::setPoiVelocityMax(double mps)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  poi_velocity_max_ = std::max(0.0, mps);
}

double RepulsionDataManager::getPoiVelocityMax() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return poi_velocity_max_;
}

void RepulsionDataManager::setPoiPredictConservativeMinDistance(bool enabled)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  poi_predict_conservative_min_distance_ = enabled;
}

bool RepulsionDataManager::getPoiPredictConservativeMinDistance() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return poi_predict_conservative_min_distance_;
}

bool RepulsionDataManager::hasValidData() const
{
  if (!robot_state_ || !robot_state_->isReady())
    return false;
  if (!map3d_manager_)
    return false;

  const auto meta = map3d_manager_->getLatestMetadata();
  if (meta.stamp.isZero())
    return false;

  const double age = (ros::Time::now() - meta.stamp).toSec();
  return age < stale_timeout_;
}

ros::Time RepulsionDataManager::getLastDataTimestamp() const
{
  if (!map3d_manager_)
    return ros::Time(0);
  return map3d_manager_->getLatestMetadata().stamp;
}

void RepulsionDataManager::setStaleTimeout(double timeout)
{
  stale_timeout_ = std::max(0.01, timeout);
}

double RepulsionDataManager::getStaleTimeout() const
{
  return stale_timeout_;
}

}  // namespace cartesian_velocity_controller
