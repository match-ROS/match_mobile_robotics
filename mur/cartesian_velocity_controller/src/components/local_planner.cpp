/**
 * @file local_planner.cpp
 * @brief Implementation of the LocalPlanner component.
 */

#include "cartesian_velocity_controller/components/local_planner.hpp"
#include "cartesian_velocity_controller/components/robot_state_manager.hpp"
#include "cartesian_velocity_controller/components/jacobian_solver.hpp"

#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <map>

namespace cartesian_velocity_controller
{

namespace
{
inline double smoothstep01(double r)
{
  r = std::clamp(r, 0.0, 1.0);
  return (3.0 * r * r) - (2.0 * r * r * r);
}

inline double smootherstep01(double r)
{
  r = std::clamp(r, 0.0, 1.0);
  // 6r^5 - 15r^4 + 10r^3
  const double r2 = r * r;
  const double r3 = r2 * r;
  return r3 * (r * (r * 6.0 - 15.0) + 10.0);
}

inline double repulsiveProfile01(RepulsiveVelocityMode mode, double r)
{
  r = std::clamp(r, 0.0, 1.0);
  switch (mode)
  {
    case RepulsiveVelocityMode::LINEAR:
      return r;
    case RepulsiveVelocityMode::QUADRATIC:
      return r * r;
    case RepulsiveVelocityMode::SMOOTHSTEP:
      return smoothstep01(r);
    case RepulsiveVelocityMode::SMOOTHERSTEP:
      return smootherstep01(r);
    default:
      return r * r;
  }
}
}  // namespace

LocalPlanner::LocalPlanner(std::shared_ptr<RobotStateManager> robot_state,
                           std::shared_ptr<JacobianSolver> solver)
  : robot_state_(std::move(robot_state))
  , solver_(std::move(solver))
{
  ROS_DEBUG_NAMED("local_planner", "LocalPlanner initialized");
}

// ============== Main Computation ==============

LocalPlannerOutput LocalPlanner::compute(
    const Eigen::Isometry3d& current_pose,
    const Eigen::Isometry3d& waypoint,
    const std::vector<ObstacleInfo>& obstacles,
    const std::vector<LinkPOI>& link_pois,
    double dt)
{
  LocalPlannerOutput output;

  // Get parameters
  double k_att, k_rep, k_rep_link;
  double max_lin_vel, max_ang_vel;
  double influence_dist, min_safe_dist;
  double tau_rise, tau_fall;
  double max_acc_rise, max_acc_fall;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    k_att = k_attractive_;
    k_rep = k_repulsive_obstacle_;
    k_rep_link = k_repulsive_link_;
    max_lin_vel = max_linear_velocity_;
    max_ang_vel = max_angular_velocity_;
    influence_dist = influence_distance_;
    min_safe_dist = min_safe_distance_;
    tau_rise = repulsive_filter_tau_rise_;
    tau_fall = repulsive_filter_tau_fall_;
    max_acc_rise = repulsive_max_acc_rise_;
    max_acc_fall = repulsive_max_acc_fall_;
  }

  // Initialize virtual target if not done yet
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_target_)
    {
      target_raw_ = current_pose;
      has_target_ = true;
      ROS_DEBUG_NAMED("local_planner", "Initialized virtual target to current pose");
    }
  }

  // Snapshot of current virtual target (thread-safe)
  Eigen::Isometry3d target_raw_snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_raw_snapshot = target_raw_;
  }

  // ==== 1. Compute Attractive Velocity ====
  Eigen::Vector3d v_attractive_linear = computeAttractiveLinearVelocity(
      target_raw_snapshot.translation(), waypoint.translation());

  Eigen::Vector3d v_attractive_angular = computeAttractiveAngularVelocity(
      Eigen::Quaterniond(target_raw_snapshot.rotation()),
      Eigen::Quaterniond(waypoint.rotation()));

  // Apply attractive gain
  v_attractive_linear *= k_att;
  v_attractive_angular *= k_att;

  output.attractive_linear = v_attractive_linear;
  output.attractive_angular = v_attractive_angular;

  // ==== 2. Compute Repulsive Velocity from TCP/Payload Obstacles ====
  Eigen::Vector3d v_repulsive_obstacle_raw = computeRepulsiveVelocityTotal(
      current_pose.translation(), obstacles);

  // Apply repulsive gain (part of the commanded repulsive component)
  v_repulsive_obstacle_raw *= k_rep;

  // 3.1: EMA smoothing (asymmetric rise/fall), 3.3: rate limiter (asymmetric)
  Eigen::Vector3d v_repulsive_obstacle = v_repulsive_obstacle_raw;
  v_repulsive_obstacle_filtered_ = applyAsymmetricEma(
      v_repulsive_obstacle_raw, v_repulsive_obstacle_filtered_, dt, tau_rise, tau_fall);
  v_repulsive_obstacle = v_repulsive_obstacle_filtered_;
  v_repulsive_obstacle = applyAsymmetricRateLimiter(
      v_repulsive_obstacle, v_repulsive_obstacle_prev_out_, dt, max_acc_rise, max_acc_fall);
  v_repulsive_obstacle_prev_out_ = v_repulsive_obstacle;

  output.repulsive_obstacle_linear = v_repulsive_obstacle;

  // ==== 3. Compute Repulsive Velocity from Link POIs (joint space) ====
  Eigen::VectorXd v_repulsive_links_joint;

  // Create a mutable copy of link_pois to store computed velocities
  std::vector<LinkPOI> link_pois_copy = link_pois;

  if (!link_pois_copy.empty() && robot_state_ && solver_)
  {
    v_repulsive_links_joint = computeRepulsiveLinkJointVelocity(link_pois_copy);
    v_repulsive_links_joint *= k_rep_link;
  }
  else
  {
    // No POIs or missing dependencies - return zero joint velocity
    if (robot_state_)
    {
      v_repulsive_links_joint = Eigen::VectorXd::Zero(robot_state_->getJointCount());
    }
    else
    {
      v_repulsive_links_joint = Eigen::VectorXd::Zero(6);  // Default assumption
    }
  }

  output.repulsive_links_joint = v_repulsive_links_joint;

  // Store POIs with computed velocities for debug/visualization
  output.link_pois_with_velocities = std::move(link_pois_copy);

  // ==== 4. Convert Link POI Joint Velocity to Cartesian for Virtual Point ====
  // According to the user's clarification: the link repulsive joint velocities
  // should be converted to Cartesian velocity via the full Jacobian and added
  // to the virtual point velocity
  Eigen::Vector3d v_repulsive_links_cartesian = Eigen::Vector3d::Zero();

  if (v_repulsive_links_joint.size() > 0 && robot_state_ && robot_state_->isReady())
  {
    // Get TCP Jacobian
    Eigen::MatrixXd jacobian;
    if (robot_state_->getJacobian(robot_state_->getTcpLink(), Eigen::Vector3d::Zero(), jacobian))
    {
      // J * q_dot gives Cartesian velocity at TCP
      if (jacobian.rows() >= 3 && jacobian.cols() == v_repulsive_links_joint.size())
      {
        Eigen::VectorXd cart_vel = jacobian * v_repulsive_links_joint;
        v_repulsive_links_cartesian = cart_vel.head<3>();
      }
    }
  }

  // Apply smoothing/limiting to the link repulsive cartesian component as well (optional but consistent)
  v_repulsive_links_filtered_ = applyAsymmetricEma(
      v_repulsive_links_cartesian, v_repulsive_links_filtered_, dt, tau_rise, tau_fall);
  v_repulsive_links_cartesian = v_repulsive_links_filtered_;
  v_repulsive_links_cartesian = applyAsymmetricRateLimiter(
      v_repulsive_links_cartesian, v_repulsive_links_prev_out_, dt, max_acc_rise, max_acc_fall);
  v_repulsive_links_prev_out_ = v_repulsive_links_cartesian;

  // ==== 5. Combine Velocities ====
  Eigen::Vector3d v_combined_linear = v_attractive_linear + v_repulsive_obstacle + v_repulsive_links_cartesian;
  Eigen::Vector3d v_combined_angular = v_attractive_angular;

  // Apply velocity limits
  v_combined_linear = limitVelocity(v_combined_linear, max_lin_vel);
  v_combined_angular = limitVelocity(v_combined_angular, max_ang_vel);

  output.combined_linear = v_combined_linear;
  output.combined_angular = v_combined_angular;

  // ==== 6. Virtual Target Leash (Soft Leash + Reset) ====
  double scaling_factor = 1.0;
  bool perform_reset = false;
  double dist_virtual_robot = 0.0;
  
  bool leash_active = false;
  double l_start = 0.0, l_stop = 0.0, l_reset = 0.0;

  {
    std::lock_guard<std::mutex> params_lock(params_mutex_);
    leash_active = leash_enabled_;
    l_start = leash_start_dist_;
    l_stop = leash_stop_dist_;
    l_reset = leash_reset_threshold_;
  }

  if (leash_active)
  {
    // Use snapshot to keep computations consistent within this cycle
    dist_virtual_robot = (target_raw_snapshot.translation() - current_pose.translation()).norm();

    if (dist_virtual_robot > l_reset)
    {
      perform_reset = true;
    }
    else if (dist_virtual_robot > l_start)
    {
      double denominator = std::max(1e-4, l_stop - l_start);
      double ratio = (dist_virtual_robot - l_start) / denominator;
      scaling_factor = std::clamp(1.0 - ratio, 0.0, 1.0);
    }
  }

  output.virtual_target_scaling_factor = scaling_factor;

  if (perform_reset)
  {
    resetToPosition(current_pose);
    
    // Zero out velocities to prevent jump
    v_combined_linear.setZero();
    v_combined_angular.setZero();
    
    output.combined_linear.setZero();
    output.combined_angular.setZero();
    output.virtual_target_scaling_factor = 0.0; // Indicate reset state
  }
  else
  {
    // Apply leash in a "non-blocking" way:
    // - Do NOT scale tangential motion
    // - Scale ONLY the outgoing radial component (away from the robot) when beyond start distance
    // - Do NOT scale angular velocity (per design choice)
    Eigen::Vector3d v_integrate_linear = v_combined_linear;
    if (scaling_factor < 1.0 - kEpsilon)
    {
      const Eigen::Vector3d r = target_raw_snapshot.translation() - current_pose.translation();
      const double r_norm = r.norm();
      if (r_norm > kEpsilon)
      {
        const Eigen::Vector3d n = r / r_norm;               // radial direction robot -> target_raw
        const double v_out_scalar = v_integrate_linear.dot(n); // positive = moving further away
        if (v_out_scalar > 0.0)
        {
          const Eigen::Vector3d v_out = v_out_scalar * n;
          const Eigen::Vector3d v_rest = v_integrate_linear - v_out;
          v_integrate_linear = v_rest + scaling_factor * v_out;
        }
      }
    }

    output.combined_linear = v_integrate_linear;
    output.combined_angular = v_combined_angular;

    // ==== 7. Integrate Virtual Target Position ====
    integrateTarget(v_integrate_linear, v_combined_angular, dt);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    output.target_raw = target_raw_;
  }

  // ==== 8. Compute Diagnostic Info ====
  output.distance_to_waypoint = (waypoint.translation() - current_pose.translation()).norm();

  // Angular distance
  Eigen::Quaterniond q_curr(current_pose.rotation());
  Eigen::Quaterniond q_wp(waypoint.rotation());
  if (q_curr.dot(q_wp) < 0.0) q_wp.coeffs() = -q_wp.coeffs();
  Eigen::Quaterniond q_diff = q_curr.inverse() * q_wp;
  output.angular_distance_to_waypoint = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

  // Closest obstacle distance
  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& obs : obstacles)
  {
    if (obs.distance < min_dist)
    {
      min_dist = obs.distance;
    }
  }
  output.closest_obstacle_distance = min_dist;

  return output;
}

// ============== Dynamic Parameters ==============

void LocalPlanner::setAttractiveGain(double k_att)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  k_attractive_ = std::max(0.0, k_att);
}

double LocalPlanner::getAttractiveGain() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return k_attractive_;
}

void LocalPlanner::setRepulsiveObstacleGain(double k_rep)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  k_repulsive_obstacle_ = std::max(0.0, k_rep);
}

double LocalPlanner::getRepulsiveObstacleGain() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return k_repulsive_obstacle_;
}

void LocalPlanner::setRepulsiveLinkGain(double k_rep_link)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  k_repulsive_link_ = std::max(0.0, k_rep_link);
}

double LocalPlanner::getRepulsiveLinkGain() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return k_repulsive_link_;
}

// ============== Velocity Limits ==============

void LocalPlanner::setMaxLinearVelocity(double max_vel)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  max_linear_velocity_ = std::max(0.01, max_vel);
}

double LocalPlanner::getMaxLinearVelocity() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return max_linear_velocity_;
}

void LocalPlanner::setMaxAngularVelocity(double max_vel)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  max_angular_velocity_ = std::max(0.01, max_vel);
}

double LocalPlanner::getMaxAngularVelocity() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return max_angular_velocity_;
}

void LocalPlanner::setIntegrationFreezeThresholds(double linear_thresh, double angular_thresh)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  freeze_linear_threshold_ = std::max(0.0, linear_thresh);
  freeze_angular_threshold_ = std::max(0.0, angular_thresh);
}

// ============== Repulsive Parameters ==============

void LocalPlanner::setInfluenceDistance(double distance)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  influence_distance_ = std::max(0.01, distance);
}

double LocalPlanner::getInfluenceDistance() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return influence_distance_;
}

void LocalPlanner::setMinSafeDistance(double distance)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  min_safe_distance_ = std::max(0.001, distance);
}

double LocalPlanner::getMinSafeDistance() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return min_safe_distance_;
}

void LocalPlanner::setRepulsiveMode(RepulsiveVelocityMode mode)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  repulsive_mode_ = mode;
}

RepulsiveVelocityMode LocalPlanner::getRepulsiveMode() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return repulsive_mode_;
}

void LocalPlanner::setRepulsiveVelocityFilterTaus(double tau_rise, double tau_fall)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  repulsive_filter_tau_rise_ = std::max(0.0, tau_rise);
  repulsive_filter_tau_fall_ = std::max(0.0, tau_fall);
}

void LocalPlanner::getRepulsiveVelocityFilterTaus(double& tau_rise, double& tau_fall) const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  tau_rise = repulsive_filter_tau_rise_;
  tau_fall = repulsive_filter_tau_fall_;
}

void LocalPlanner::setRepulsiveVelocityMaxAccelerations(double max_acc_rise, double max_acc_fall)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  repulsive_max_acc_rise_ = std::max(0.0, max_acc_rise);
  repulsive_max_acc_fall_ = std::max(0.0, max_acc_fall);
}

void LocalPlanner::getRepulsiveVelocityMaxAccelerations(double& max_acc_rise, double& max_acc_fall) const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  max_acc_rise = repulsive_max_acc_rise_;
  max_acc_fall = repulsive_max_acc_fall_;
}

// ============== Virtual Target Leash ==============

void LocalPlanner::setVirtualTargetLeashEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  leash_enabled_ = enabled;
}

void LocalPlanner::setVirtualTargetLeashParams(double start_dist, double stop_dist, double reset_thresh)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  leash_start_dist_ = std::max(0.001, start_dist);
  leash_stop_dist_ = std::max(leash_start_dist_ + 0.001, stop_dist);
  leash_reset_threshold_ = std::max(leash_stop_dist_ + 0.01, reset_thresh);
}

bool LocalPlanner::getVirtualTargetLeashEnabled() const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  return leash_enabled_;
}

void LocalPlanner::getVirtualTargetLeashParams(double& start_dist, double& stop_dist, double& reset_thresh) const
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  start_dist = leash_start_dist_;
  stop_dist = leash_stop_dist_;
  reset_thresh = leash_reset_threshold_;
}

// ============== State Management ==============

void LocalPlanner::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_raw_ = Eigen::Isometry3d::Identity();
  has_target_ = false;
  v_repulsive_obstacle_filtered_.setZero();
  v_repulsive_links_filtered_.setZero();
  v_repulsive_obstacle_prev_out_.setZero();
  v_repulsive_links_prev_out_.setZero();
  ROS_DEBUG_NAMED("local_planner", "LocalPlanner reset");
}

void LocalPlanner::resetToPosition(const Eigen::Isometry3d& pose)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_raw_ = pose;
  has_target_ = true;
  v_repulsive_obstacle_filtered_.setZero();
  v_repulsive_links_filtered_.setZero();
  v_repulsive_obstacle_prev_out_.setZero();
  v_repulsive_links_prev_out_.setZero();
  ROS_DEBUG_NAMED("local_planner", "LocalPlanner reset to position");
}

bool LocalPlanner::isInitialized() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_target_;
}

Eigen::Isometry3d LocalPlanner::getVirtualTarget() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return target_raw_;
}

// ============== Private Methods ==============

Eigen::Vector3d LocalPlanner::computeAttractiveLinearVelocity(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& waypoint_pos) const
{
  Eigen::Vector3d diff = waypoint_pos - current_pos;
  double dist = diff.norm();

  if (dist < kEpsilon)
  {
    return Eigen::Vector3d::Zero();
  }

  // Normalized direction toward waypoint
  // The gain is applied externally, so this returns unit direction
  // scaled by distance (proportional control)
  return diff;  // Note: This is proportional to error, not just normalized
}

Eigen::Vector3d LocalPlanner::applyAsymmetricEma(
    const Eigen::Vector3d& v_raw,
    const Eigen::Vector3d& v_prev_filtered,
    double dt,
    double tau_rise,
    double tau_fall)
{
  const double tau_r = std::max(0.0, tau_rise);
  const double tau_f = std::max(0.0, tau_fall);
  if (tau_r <= 1e-12 && tau_f <= 1e-12)
  {
    return v_raw;  // disabled
  }

  const double v_raw_n = v_raw.norm();
  const double v_prev_n = v_prev_filtered.norm();
  const double tau = (v_raw_n > v_prev_n) ? tau_r : tau_f;

  if (tau <= 1e-12 || dt <= 0.0)
  {
    return v_raw;
  }

  const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
  return (alpha * v_raw) + ((1.0 - alpha) * v_prev_filtered);
}

Eigen::Vector3d LocalPlanner::applyAsymmetricRateLimiter(
    const Eigen::Vector3d& v_target,
    const Eigen::Vector3d& v_prev_out,
    double dt,
    double max_acc_rise,
    double max_acc_fall)
{
  const double a_r = std::max(0.0, max_acc_rise);
  const double a_f = std::max(0.0, max_acc_fall);
  if (a_r <= 1e-12 && a_f <= 1e-12)
  {
    return v_target;  // disabled
  }
  if (dt <= 0.0)
  {
    return v_target;
  }

  const double v_t_n = v_target.norm();
  const double v_p_n = v_prev_out.norm();
  const double a = (v_t_n > v_p_n) ? a_r : a_f;

  if (a <= 1e-12)
  {
    // This side disabled -> allow unconstrained motion for that direction of change.
    return v_target;
  }

  const double max_dv = a * dt;
  Eigen::Vector3d dv = v_target - v_prev_out;
  const double dv_n = dv.norm();
  if (dv_n > max_dv && dv_n > kEpsilon)
  {
    dv *= (max_dv / dv_n);
  }
  return v_prev_out + dv;
}

Eigen::Vector3d LocalPlanner::computeAttractiveAngularVelocity(
    const Eigen::Quaterniond& current_orientation,
    const Eigen::Quaterniond& waypoint_orientation) const
{
  // Ensure normalized
  Eigen::Quaterniond q_curr = current_orientation.normalized();
  Eigen::Quaterniond q_wp = waypoint_orientation.normalized();

  // Take shorter path
  if (q_curr.dot(q_wp) < 0.0)
  {
    q_wp.coeffs() = -q_wp.coeffs();
  }

  // Relative rotation from current to waypoint
  Eigen::Quaterniond q_error = (q_wp * q_curr.inverse()).normalized();

  // Ensure we always use the minimal rotation (angle in [0, pi]).
  // Note: q and -q represent the same rotation, but they yield different
  // angle outputs if you manually extract angle from (w, xyz).
  if (q_error.w() < 0.0)
  {
    q_error.coeffs() = -q_error.coeffs();
  }

  // Robust axis-angle extraction (Eigen guarantees angle in [0, pi])
  const Eigen::AngleAxisd aa(q_error);
  const double angle = aa.angle();
  if (!(std::isfinite(angle)) || angle < kEpsilon)
  {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d axis = aa.axis();
  if (!std::isfinite(axis.x()) || !std::isfinite(axis.y()) || !std::isfinite(axis.z()))
  {
    return Eigen::Vector3d::Zero();
  }

  // Return angular error (proportional to angle difference)
  return angle * axis;
}

Eigen::Vector3d LocalPlanner::computeRepulsiveVelocityFromObstacle(
    const Eigen::Vector3d& point,
    const ObstacleInfo& obstacle) const
{
  double influence_dist, min_safe_dist;
  RepulsiveVelocityMode mode;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    influence_dist = influence_distance_;
    min_safe_dist = min_safe_distance_;
    mode = repulsive_mode_;
  }

  // Use obstacle's own influence distance if specified, otherwise use global
  double effective_influence = (obstacle.influence_distance > 0.0) ?
      obstacle.influence_distance : influence_dist;
  double effective_min_safe = (obstacle.min_safe_distance > 0.0) ?
      obstacle.min_safe_distance : min_safe_dist;

  double distance = obstacle.distance;

  // No repulsion if outside influence range
  if (distance >= effective_influence)
  {
    return Eigen::Vector3d::Zero();
  }

  // Get repulsive direction (away from obstacle)
  Eigen::Vector3d direction = obstacle.getRepulsiveDirection();
  if (direction.norm() < kEpsilon)
  {
    return Eigen::Vector3d::Zero();
  }

  double magnitude = 0.0;

  double max_vel;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    max_vel = max_linear_velocity_;
  }

  if (distance <= effective_min_safe)
  {
    magnitude = max_vel;
  }
  else
  {
    const double denom = effective_influence - effective_min_safe;
    if (denom > kEpsilon)
    {
      const double r = (effective_influence - distance) / denom;  // 0..1 inside influence zone
      magnitude = max_vel * repulsiveProfile01(mode, r);
    }
    else
    {
      magnitude = max_vel;
    }
  }

  return direction * magnitude;
}

Eigen::Vector3d LocalPlanner::computeRepulsiveVelocityTotal(
    const Eigen::Vector3d& point,
    const std::vector<ObstacleInfo>& obstacles) const
{
  Eigen::Vector3d total_repulsive = Eigen::Vector3d::Zero();

  for (const auto& obstacle : obstacles)
  {
    total_repulsive += computeRepulsiveVelocityFromObstacle(point, obstacle);
  }

  return total_repulsive;
}

Eigen::VectorXd LocalPlanner::computeRepulsiveLinkJointVelocity(
    std::vector<LinkPOI>& link_pois) const
{
  if (!robot_state_ || !solver_ || !robot_state_->isReady())
  {
    // Initialize all POIs with zero velocity
    for (auto& poi : link_pois)
    {
      poi.repulsive_velocity = Eigen::Vector3d::Zero();
      poi.repulsive_velocity_magnitude = 0.0;
    }
    return Eigen::VectorXd::Zero(6);
  }

  std::size_t num_joints = robot_state_->getJointCount();
  Eigen::VectorXd total_joint_vel = Eigen::VectorXd::Zero(num_joints);

  double influence_dist, min_safe_dist, max_vel;
  RepulsiveVelocityMode mode;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    influence_dist = influence_distance_;
    min_safe_dist = min_safe_distance_;
    max_vel = max_linear_velocity_;
    mode = repulsive_mode_;
  }

  // Get a copy of the robot state
  moveit::core::RobotState current_state = robot_state_->getRobotStateCopy();
  const moveit::core::JointModelGroup* jmg = robot_state_->getJointModelGroup();

  // Group POIs by name to sum velocities
  struct POIGroup {
    std::vector<LinkPOI*> pois;
    Eigen::Vector3d total_cartesian_velocity = Eigen::Vector3d::Zero();
    std::string link_name;
    Eigen::Vector3d position_link;
  };
  
  std::map<std::string, POIGroup> poi_groups;

  // First pass: Calculate Cartesian velocities and group them
  for (auto& poi : link_pois)
  {
    // Initialize velocity to zero
    poi.repulsive_velocity = Eigen::Vector3d::Zero();
    poi.repulsive_velocity_magnitude = 0.0;

    // Skip if outside influence range
    if (!poi.hasActiveRepulsion(influence_dist))
    {
      continue;
    }
    
    // Add to group
    auto& group = poi_groups[poi.point_name];
    group.pois.push_back(&poi);
    group.link_name = poi.link_name;
    group.position_link = poi.position_link;

    // Compute repulsive magnitude
    double distance = poi.distance_to_closest_obstacle;
    double magnitude = 0.0;

    if (distance <= min_safe_dist)
    {
      magnitude = max_vel;
    }
    else
    {
      const double denom = influence_dist - min_safe_dist;
      if (denom > kEpsilon)
      {
        const double r = (influence_dist - distance) / denom;
        magnitude = max_vel * repulsiveProfile01(mode, r);
      }
      else
      {
        magnitude = max_vel;
      }
    }

    // Apply weight
    magnitude *= poi.weight;

    // Compute Cartesian velocity for this obstacle
    Eigen::Vector3d cart_vel = poi.repulsive_direction * magnitude;
    
    // Update POI info for visualization (this shows individual contribution)
    poi.repulsive_velocity = cart_vel;
    poi.repulsive_velocity_magnitude = cart_vel.norm();

    // Accumulate to group total
    group.total_cartesian_velocity += cart_vel;
  }

  // Second pass: Compute Jacobian and project for each group
  for (const auto& [name, group] : poi_groups)
  {
    if (group.total_cartesian_velocity.norm() < kEpsilon)
    {
      continue;
    }

    // Get link model
    const moveit::core::LinkModel* link_model = current_state.getLinkModel(group.link_name);
    if (!link_model)
    {
      ROS_WARN_THROTTLE_NAMED(5.0, "local_planner",
                              "Link '%s' not found for POI repulsion", group.link_name.c_str());
      continue;
    }

    // Get partial Jacobian for this link/POI
    Eigen::MatrixXd jacobian;
    current_state.getJacobian(jmg, link_model, group.position_link, jacobian);

    if (jacobian.rows() < 3)
    {
      continue;
    }

    // Extract linear part (top 3 rows)
    Eigen::MatrixXd jacobian_linear = jacobian.topRows(3);

    // Remap columns if needed
    const auto& joint_names = robot_state_->getJointNames();
    if (jacobian_linear.cols() != static_cast<int>(joint_names.size()))
    {
      Eigen::MatrixXd remapped = Eigen::MatrixXd::Zero(3, joint_names.size());
      const auto& jacobian_joint_names = jmg->getActiveJointModelNames();
      const auto& joint_index_map = robot_state_->getJointIndexMap();

      for (std::size_t idx = 0; idx < jacobian_joint_names.size(); ++idx)
      {
        auto map_it = joint_index_map.find(jacobian_joint_names[idx]);
        if (map_it != joint_index_map.end())
        {
          remapped.col(map_it->second) = jacobian_linear.col(idx);
        }
      }
      jacobian_linear = remapped;
    }

    // Compute pseudo-inverse (WITHOUT damping, as per plan)
    // J⁺ = Jᵀ * (J * Jᵀ)⁻¹
    Eigen::MatrixXd JJt = jacobian_linear * jacobian_linear.transpose();
    Eigen::MatrixXd JJt_inv = JJt.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_pinv = jacobian_linear.transpose() * JJt_inv;

    // Convert to joint velocity using the SUMMED Cartesian velocity
    Eigen::VectorXd joint_vel = J_pinv * group.total_cartesian_velocity;

    total_joint_vel += joint_vel;
  }

  return total_joint_vel;
}

void LocalPlanner::integrateTarget(const Eigen::Vector3d& velocity_linear,
                                   const Eigen::Vector3d& velocity_angular,
                                   double dt)
{
  double lin_thresh, ang_thresh;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    lin_thresh = freeze_linear_threshold_;
    ang_thresh = freeze_angular_threshold_;
  }

  // Avoid integrating microscopic jitter
  if (velocity_linear.norm() < lin_thresh && velocity_angular.norm() < ang_thresh)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  // Integrate position: P_new = P_old + V * dt
  target_raw_.translation() += velocity_linear * dt;

  // Integrate orientation using exponential map
  double angle = velocity_angular.norm();
  if (angle > kEpsilon)
  {
    Eigen::Vector3d axis = velocity_angular / angle;
    double delta_angle = angle * dt;
    Eigen::Quaterniond delta_rot(Eigen::AngleAxisd(delta_angle, axis));
    Eigen::Quaterniond current_rot(target_raw_.rotation());
    target_raw_.linear() = (delta_rot * current_rot).toRotationMatrix();
  }
}

Eigen::Vector3d LocalPlanner::limitVelocity(const Eigen::Vector3d& velocity, double max_magnitude)
{
  double mag = velocity.norm();
  if (mag > max_magnitude && mag > kEpsilon)
  {
    return velocity * (max_magnitude / mag);
  }
  return velocity;
}

}  // namespace cartesian_velocity_controller

