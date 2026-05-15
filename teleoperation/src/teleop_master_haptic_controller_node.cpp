#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <XmlRpcValue.h>

#include "teleoperation/core/math_utils.hpp"
#include "teleoperation/core/jerk_limiter.hpp"
#include "teleoperation/core/wrench_debug_publisher.hpp"
#include "teleoperation/core/tf_utils.hpp"
#include "teleoperation/core/types.hpp"
#include "teleoperation/core/wrench_utils.hpp"
#include "teleoperation/components/passivity_layer.hpp"

using Wrench3 = teleoperation::Wrench3;

class TeleopMasterHapticController
{
public:
  TeleopMasterHapticController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("master_wrench_topic", master_wrench_topic_, "wrench");
    pnh_.param<std::string>("slave_wrench_topic", slave_wrench_topic_, "");
    pnh_.param<std::string>("coupling_wrench_topic", coupling_wrench_topic_, "");
    node_name_ = ros::this_node::getName();
    resolved_master_wrench_topic_ = nh_.resolveName(master_wrench_topic_);
    resolved_slave_wrench_topic_ = slave_wrench_topic_.empty() ? std::string() : nh_.resolveName(slave_wrench_topic_);
    resolved_coupling_wrench_topic_ = coupling_wrench_topic_.empty() ? std::string() : nh_.resolveName(coupling_wrench_topic_);
    pnh_.param<std::string>("wrench_source_frame_override", wrench_source_frame_override_, "");
    pnh_.param<bool>("use_latest_tf_for_wrench", use_latest_tf_for_wrench_, use_latest_tf_for_wrench_);

    pnh_.param<std::string>("wrench_target_frame", wrench_target_frame_, "base_link");
    pnh_.param<double>("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);

    pnh_.param<std::string>("command_topic", command_topic_, "twist_controller/command");
    pnh_.param<double>("control_rate", control_rate_, control_rate_);

    pnh_.param<double>("mass_linear", mass_linear_, mass_linear_);
    pnh_.param<double>("damping_linear", damping_linear_, damping_linear_);
    pnh_.param<double>("mass_angular", mass_angular_, mass_angular_);
    pnh_.param<double>("damping_angular", damping_angular_, damping_angular_);

    // Dynamic master damping scheduled by slave force (Option A).
    // Params live under the node private namespace, e.g. "~dynamic_damping/enabled".
    pnh_.param<bool>("dynamic_damping/enabled", dyn_damping_enabled_, dyn_damping_enabled_);
    pnh_.param<bool>("dynamic_damping/use_slave_wrench", dyn_damping_use_slave_wrench_, dyn_damping_use_slave_wrench_);
    pnh_.param<std::string>("dynamic_damping/force_metric", dyn_damping_force_metric_, dyn_damping_force_metric_);
    pnh_.param<double>("dynamic_damping/force_start", dyn_damping_force_start_, dyn_damping_force_start_);
    pnh_.param<double>("dynamic_damping/force_stop", dyn_damping_force_stop_, dyn_damping_force_stop_);
    pnh_.param<double>("dynamic_damping/shape_exp", dyn_damping_shape_exp_, dyn_damping_shape_exp_);
    // Optional override: if enabled, angular extras always follow the force schedule.
    // Otherwise, angular extras are scheduled by slave torque when torque_* is provided.
    pnh_.param<bool>("dynamic_damping/angular_schedule_from_force",
                     dyn_damping_angular_schedule_from_force_,
                     dyn_damping_angular_schedule_from_force_);
    const bool has_dyn_damping_torque_start = pnh_.getParam("dynamic_damping/torque_start", dyn_damping_torque_start_);
    const bool has_dyn_damping_torque_stop = pnh_.getParam("dynamic_damping/torque_stop", dyn_damping_torque_stop_);
    (void)pnh_.getParam("dynamic_damping/torque_metric", dyn_damping_torque_metric_);
    dyn_damping_use_torque_schedule_ =
        !dyn_damping_angular_schedule_from_force_ &&
        (has_dyn_damping_torque_start || has_dyn_damping_torque_stop);
    if (dyn_damping_use_torque_schedule_)
    {
      if (!has_dyn_damping_torque_start) dyn_damping_torque_start_ = dyn_damping_force_start_;
      if (!has_dyn_damping_torque_stop) dyn_damping_torque_stop_ = dyn_damping_force_stop_;
    }
    pnh_.param<double>("dynamic_damping/extra_damping_linear", dyn_extra_damping_linear_, dyn_extra_damping_linear_);
    pnh_.param<double>("dynamic_damping/extra_damping_angular", dyn_extra_damping_angular_, dyn_extra_damping_angular_);
    pnh_.param<double>("dynamic_damping/d_extra_lowpass_cutoff_hz", dyn_d_extra_lowpass_cutoff_hz_, dyn_d_extra_lowpass_cutoff_hz_);
    (void)tryGetVector3Param(pnh_, "dynamic_damping/extra_damping_linear_xyz", dyn_extra_damping_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "dynamic_damping/extra_damping_angular_xyz", dyn_extra_damping_angular_xyz_);

    // Dynamic master mass scheduled by slave force (same principles as damping, independent params).
    pnh_.param<bool>("dynamic_mass/enabled", dyn_mass_enabled_, dyn_mass_enabled_);
    pnh_.param<bool>("dynamic_mass/use_slave_wrench", dyn_mass_use_slave_wrench_, dyn_mass_use_slave_wrench_);
    pnh_.param<std::string>("dynamic_mass/force_metric", dyn_mass_force_metric_, dyn_mass_force_metric_);
    pnh_.param<double>("dynamic_mass/force_start", dyn_mass_force_start_, dyn_mass_force_start_);
    pnh_.param<double>("dynamic_mass/force_stop", dyn_mass_force_stop_, dyn_mass_force_stop_);
    pnh_.param<double>("dynamic_mass/shape_exp", dyn_mass_shape_exp_, dyn_mass_shape_exp_);
    // Optional override: if enabled, angular extras always follow the force schedule.
    // Otherwise, angular extras are scheduled by slave torque when torque_* is provided.
    pnh_.param<bool>("dynamic_mass/angular_schedule_from_force",
                     dyn_mass_angular_schedule_from_force_,
                     dyn_mass_angular_schedule_from_force_);
    const bool has_dyn_mass_torque_start = pnh_.getParam("dynamic_mass/torque_start", dyn_mass_torque_start_);
    const bool has_dyn_mass_torque_stop = pnh_.getParam("dynamic_mass/torque_stop", dyn_mass_torque_stop_);
    (void)pnh_.getParam("dynamic_mass/torque_metric", dyn_mass_torque_metric_);
    dyn_mass_use_torque_schedule_ =
        !dyn_mass_angular_schedule_from_force_ &&
        (has_dyn_mass_torque_start || has_dyn_mass_torque_stop);
    if (dyn_mass_use_torque_schedule_)
    {
      if (!has_dyn_mass_torque_start) dyn_mass_torque_start_ = dyn_mass_force_start_;
      if (!has_dyn_mass_torque_stop) dyn_mass_torque_stop_ = dyn_mass_force_stop_;
    }
    pnh_.param<double>("dynamic_mass/extra_mass_linear", dyn_extra_mass_linear_, dyn_extra_mass_linear_);
    pnh_.param<double>("dynamic_mass/extra_mass_angular", dyn_extra_mass_angular_, dyn_extra_mass_angular_);
    pnh_.param<double>("dynamic_mass/m_extra_lowpass_cutoff_hz", dyn_m_extra_lowpass_cutoff_hz_, dyn_m_extra_lowpass_cutoff_hz_);
    (void)tryGetVector3Param(pnh_, "dynamic_mass/extra_mass_linear_xyz", dyn_extra_mass_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "dynamic_mass/extra_mass_angular_xyz", dyn_extra_mass_angular_xyz_);

    pnh_.param<double>("force_reflection_scale", kf_force_, kf_force_);
    pnh_.param<double>("torque_reflection_scale", kf_torque_, kf_torque_);
    pnh_.param<std::string>("force_reflection_gate_topic", force_reflection_gate_topic_, force_reflection_gate_topic_);
    pnh_.param<std::string>("home_return_disable_topic", home_return_disable_topic_, home_return_disable_topic_);
    pnh_.param<bool>("force_reflection_bias/enabled", force_reflection_bias_enabled_, force_reflection_bias_enabled_);
    pnh_.param<std::string>("force_reflection_bias/active_topic",
                            force_reflection_bias_active_topic_,
                            force_reflection_bias_active_topic_);
    pnh_.param<double>("force_reflection_bias/capture_delay_s",
                       force_reflection_bias_capture_delay_s_,
                       force_reflection_bias_capture_delay_s_);
    pnh_.param<double>("force_reflection_bias/capture_window_s",
                       force_reflection_bias_capture_window_s_,
                       force_reflection_bias_capture_window_s_);
    pnh_.param<double>("force_reflection_bias/delta_fade_s",
                       force_reflection_bias_delta_fade_s_,
                       force_reflection_bias_delta_fade_s_);
    if (!std::isfinite(force_reflection_bias_capture_delay_s_) || force_reflection_bias_capture_delay_s_ < 0.0)
      force_reflection_bias_capture_delay_s_ = 0.0;
    if (!std::isfinite(force_reflection_bias_capture_window_s_) || force_reflection_bias_capture_window_s_ < 0.0)
      force_reflection_bias_capture_window_s_ = 0.0;
    if (!std::isfinite(force_reflection_bias_delta_fade_s_) || force_reflection_bias_delta_fade_s_ < 0.0)
      force_reflection_bias_delta_fade_s_ = 0.0;
    pnh_.param<bool>("passivity/enabled", passivity_config_.enabled, passivity_config_.enabled);
    pnh_.param<bool>("passivity/linear_only", passivity_config_.linear_only, passivity_config_.linear_only);
    pnh_.param<double>("passivity/tank_energy_init", passivity_config_.tank_energy_init, passivity_config_.tank_energy_init);
    pnh_.param<double>("passivity/tank_energy_min", passivity_config_.tank_energy_min, passivity_config_.tank_energy_min);
    pnh_.param<double>("passivity/tank_energy_max", passivity_config_.tank_energy_max, passivity_config_.tank_energy_max);
    pnh_.param<double>("passivity/recharge_gain", passivity_config_.recharge_gain, passivity_config_.recharge_gain);
    pnh_.param<double>("passivity/discharge_gain", passivity_config_.discharge_gain, passivity_config_.discharge_gain);
    pnh_.param<double>("passivity/power_deadband", passivity_config_.power_deadband, passivity_config_.power_deadband);
    pnh_.param<double>("passivity/gamma_min", passivity_config_.gamma_min, passivity_config_.gamma_min);
    pnh_.param<double>("passivity/gamma_lowpass_alpha", passivity_config_.gamma_lowpass_alpha, passivity_config_.gamma_lowpass_alpha);
    pnh_.param<double>("passivity/gamma_rate_limit", passivity_config_.gamma_rate_limit, passivity_config_.gamma_rate_limit);
    pnh_.param<bool>("passivity/publish_debug", passivity_publish_debug_, passivity_publish_debug_);
    passivity_layer_.setConfig(passivity_config_);

    // Virtual spring coupling (position-based feedback from slave actual TCP)
    pnh_.param<double>("spring_stiffness_linear", spring_k_lin_, spring_k_lin_);
    pnh_.param<double>("spring_damping_linear", spring_b_lin_, spring_b_lin_);
    pnh_.param<double>("spring_stiffness_angular", spring_k_ang_, spring_k_ang_);
    pnh_.param<double>("spring_damping_angular", spring_b_ang_, spring_b_ang_);
    pnh_.param<double>("max_spring_force", max_spring_force_, max_spring_force_);
    pnh_.param<double>("max_spring_torque", max_spring_torque_, max_spring_torque_);
    pnh_.param<std::string>("slave_actual_pose_topic", slave_actual_pose_topic_, slave_actual_pose_topic_);

    pnh_.param<bool>("use_forces", use_forces_, use_forces_);
    pnh_.param<bool>("use_torques", use_torques_, use_torques_);

    // Filtering
    pnh_.param<double>("wrench_filter_alpha", wrench_filter_alpha_, wrench_filter_alpha_);
    pnh_.param<double>("wrench_filter_cutoff_hz", wrench_filter_cutoff_hz_, wrench_filter_cutoff_hz_);
    pnh_.param<double>("master_wrench_filter_alpha", master_wrench_filter_alpha_, master_wrench_filter_alpha_);
    pnh_.param<double>("master_wrench_filter_cutoff_hz", master_wrench_filter_cutoff_hz_, master_wrench_filter_cutoff_hz_);
    pnh_.param<double>("feedback_wrench_filter_alpha", feedback_wrench_filter_alpha_, feedback_wrench_filter_alpha_);
    pnh_.param<double>("feedback_wrench_filter_cutoff_hz", feedback_wrench_filter_cutoff_hz_, feedback_wrench_filter_cutoff_hz_);

    // Deadzone on norm (soft) + optional hysteresis
    pnh_.param<double>("force_deadband", force_deadband_enter_, force_deadband_enter_);
    pnh_.param<double>("torque_deadband", torque_deadband_enter_, torque_deadband_enter_);
    (void)pnh_.getParam("force_deadband_enter", force_deadband_enter_);
    (void)pnh_.getParam("force_deadband_exit", force_deadband_exit_);
    (void)pnh_.getParam("torque_deadband_enter", torque_deadband_enter_);
    (void)pnh_.getParam("torque_deadband_exit", torque_deadband_exit_);
    if (!(force_deadband_exit_ >= 0.0)) force_deadband_exit_ = force_deadband_enter_;
    if (!(torque_deadband_exit_ >= 0.0)) torque_deadband_exit_ = torque_deadband_enter_;
    pnh_.param<double>("cross_deadband_scale", cross_deadband_scale_, cross_deadband_scale_);

    pnh_.param<double>("max_force", max_force_, max_force_);
    pnh_.param<double>("max_torque", max_torque_, max_torque_);
    pnh_.param<double>("max_force_hand", max_force_hand_, max_force_hand_);
    pnh_.param<double>("max_torque_hand", max_torque_hand_, max_torque_hand_);
    pnh_.param<double>("max_force_feedback", max_force_feedback_, max_force_feedback_);
    pnh_.param<double>("max_torque_feedback", max_torque_feedback_, max_torque_feedback_);

    pnh_.param<double>("max_linear_speed", max_linear_speed_, max_linear_speed_);
    pnh_.param<double>("max_angular_speed", max_angular_speed_, max_angular_speed_);

    // dt sanitization (measured dt from TimerEvent, then clamp/substep).
    pnh_.param<double>("dt_min_factor", dt_min_factor_, dt_min_factor_);
    pnh_.param<double>("dt_max_factor", dt_max_factor_, dt_max_factor_);
    pnh_.param<bool>("dt_use_substepping", dt_use_substepping_, dt_use_substepping_);
    pnh_.param<int>("dt_max_substeps", dt_max_substeps_, dt_max_substeps_);

    // Limits for smoothness (anti-windup coherent with jerk-limited acceleration).
    pnh_.param<double>("max_linear_accel", max_linear_accel_, max_linear_accel_);
    pnh_.param<double>("max_linear_jerk", max_linear_jerk_, max_linear_jerk_);
    pnh_.param<double>("max_angular_accel", max_angular_accel_, max_angular_accel_);
    pnh_.param<double>("max_angular_jerk", max_angular_jerk_, max_angular_jerk_);
    pnh_.param<double>("speed_saturation_eps", speed_saturation_eps_, speed_saturation_eps_);

    // Per-axis parameters (optional): override scalar mass/damping if provided.
    (void)tryGetVector3Param(pnh_, "mass_linear_xyz", mass_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "damping_linear_xyz", damping_linear_xyz_);
    (void)tryGetVector3Param(pnh_, "mass_angular_xyz", mass_angular_xyz_);
    (void)tryGetVector3Param(pnh_, "damping_angular_xyz", damping_angular_xyz_);
    (void)tryGetVector3Param(pnh_, "max_linear_accel_xyz", max_linear_accel_xyz_);
    (void)tryGetVector3Param(pnh_, "max_linear_jerk_xyz", max_linear_jerk_xyz_);
    (void)tryGetVector3Param(pnh_, "max_angular_accel_xyz", max_angular_accel_xyz_);
    (void)tryGetVector3Param(pnh_, "max_angular_jerk_xyz", max_angular_jerk_xyz_);

    pnh_.param<double>("wrench_timeout_s", wrench_timeout_s_, wrench_timeout_s_);
    pnh_.param<bool>("reset_on_stale", reset_on_stale_, reset_on_stale_);

    // Slave target publishing (optional: pose + twist from master to slave)
    pnh_.param<bool>("publish_slave_targets", publish_slave_targets_, publish_slave_targets_);
    pnh_.param<std::string>("slave_target_pose_topic", slave_target_pose_topic_, slave_target_pose_topic_);
    pnh_.param<std::string>("slave_feedforward_twist_topic", slave_ff_twist_topic_, slave_ff_twist_topic_);
    pnh_.param<std::string>("slave_base_frame", slave_base_frame_, slave_base_frame_);
    pnh_.param<std::string>("slave_tcp_frame", slave_tcp_frame_, slave_tcp_frame_);
    pnh_.param<std::string>("slave_frame_id_override", slave_frame_id_override_, slave_frame_id_override_);
    pnh_.param<double>("slave_publish_rate", slave_publish_rate_, slave_publish_rate_);
    if (tryGetVector3Param(pnh_, "slave_pose_twist_rotation_rpy", slave_pose_twist_rotation_rpy_))
    {
      slave_pose_twist_rotation_ = rpyToRotation(slave_pose_twist_rotation_rpy_);
      slave_pose_twist_rotation_q_ = Eigen::Quaterniond(slave_pose_twist_rotation_);
      slave_pose_twist_rotation_q_.normalize();
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Applying static master->slave numeric rotation rpy=[%.6f %.6f %.6f]",
                     slave_pose_twist_rotation_rpy_.x(),
                     slave_pose_twist_rotation_rpy_.y(),
                     slave_pose_twist_rotation_rpy_.z());
    }

    pnh_.param<bool>("motion_scaling/enabled", motion_scaling_enabled_, motion_scaling_enabled_);
    pnh_.param<double>("motion_scaling/translation_scale", motion_translation_scale_, motion_translation_scale_);
    pnh_.param<double>("motion_scaling/rotation_scale", motion_rotation_scale_, motion_rotation_scale_);
    if (!std::isfinite(motion_translation_scale_) || motion_translation_scale_ <= teleoperation::kMathEps)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "motion_scaling/translation_scale must be > 0. Using 1.0.");
      motion_translation_scale_ = 1.0;
    }
    if (!std::isfinite(motion_rotation_scale_) || motion_rotation_scale_ <= teleoperation::kMathEps)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "motion_scaling/rotation_scale must be > 0. Using 1.0.");
      motion_rotation_scale_ = 1.0;
    }

    const bool has_motion_master_pos =
        tryGetVector3Param(pnh_, "motion_scaling/master_neutral_position", motion_master_neutral_pos_);
    const bool has_motion_master_ori =
        tryGetQuaternionParam(pnh_, "motion_scaling/master_neutral_orientation_xyzw", motion_master_neutral_ori_);

    motion_scaling_config_valid_ = has_motion_master_pos && has_motion_master_ori;
    if (motion_scaling_enabled_ && !motion_scaling_config_valid_)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "motion_scaling is enabled but master neutral pose params are incomplete. "
                     "Scaling will stay inactive and the legacy mapping will be used.");
    }
    else if (motion_scaling_enabled_)
    {
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Motion scaling enabled: translation_scale=%.6f rotation_scale=%.6f",
                     motion_translation_scale_, motion_rotation_scale_);
    }

    pnh_.param<bool>("home_return/enabled", home_return_enabled_, home_return_enabled_);
    pnh_.param<double>("home_return/timeout_s", home_return_timeout_s_, home_return_timeout_s_);
    pnh_.param<double>("home_return/force_activity_threshold",
                       home_return_force_activity_threshold_,
                       home_return_force_activity_threshold_);
    pnh_.param<double>("home_return/torque_activity_threshold",
                       home_return_torque_activity_threshold_,
                       home_return_torque_activity_threshold_);
    pnh_.param<double>("home_return/blend_in_time_s", home_return_blend_in_time_s_, home_return_blend_in_time_s_);
    pnh_.param<double>("home_return/stiffness_linear", home_return_k_lin_, home_return_k_lin_);
    pnh_.param<double>("home_return/damping_linear", home_return_d_lin_, home_return_d_lin_);
    pnh_.param<double>("home_return/integral_linear", home_return_i_lin_, home_return_i_lin_);
    pnh_.param<double>("home_return/integral_linear_clamp", home_return_i_lin_clamp_, home_return_i_lin_clamp_);
    pnh_.param<double>("home_return/max_force", home_return_max_force_, home_return_max_force_);
    pnh_.param<double>("home_return/stiffness_angular", home_return_k_ang_, home_return_k_ang_);
    pnh_.param<double>("home_return/damping_angular", home_return_d_ang_, home_return_d_ang_);
    pnh_.param<double>("home_return/integral_angular", home_return_i_ang_, home_return_i_ang_);
    pnh_.param<double>("home_return/integral_angular_clamp", home_return_i_ang_clamp_, home_return_i_ang_clamp_);
    pnh_.param<double>("home_return/max_torque", home_return_max_torque_, home_return_max_torque_);
    pnh_.param<std::string>("home_return/status_topic", home_return_status_topic_, std::string());
    pnh_.param<double>("home_return/status_publish_rate", home_return_status_publish_rate_, home_return_status_publish_rate_);
    pnh_.param<double>("home_return/feedback_timeout_s", home_return_feedback_timeout_s_, home_return_feedback_timeout_s_);
    pnh_.param<double>("home_return/arrival_position_tolerance_m",
                       home_return_arrival_position_tolerance_m_,
                       home_return_arrival_position_tolerance_m_);
    pnh_.param<double>("home_return/arrival_orientation_tolerance_rad",
                       home_return_arrival_orientation_tolerance_rad_,
                       home_return_arrival_orientation_tolerance_rad_);

    Eigen::Vector3d home_target_position_out = Eigen::Vector3d::Zero();
    Eigen::Quaterniond home_target_orientation_out = Eigen::Quaterniond::Identity();
    const bool has_home_target_position =
        tryGetVector3Param(pnh_, "home_return/target_position", home_target_position_out);
    const bool has_home_target_orientation =
        tryGetQuaternionParam(pnh_, "home_return/target_orientation_xyzw", home_target_orientation_out);
    if (has_home_target_position && has_home_target_orientation)
    {
      home_target_pos_ = slave_pose_twist_rotation_.transpose() * home_target_position_out;
      home_target_ori_ = (slave_pose_twist_rotation_q_.conjugate() * home_target_orientation_out).normalized();
      has_home_target_ = true;
    }
    else if (home_return_enabled_)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "home_return is enabled but target_position / target_orientation_xyzw are missing or invalid.");
    }

    // Diagnostics
    pnh_.param<bool>("publish_diagnostics", publish_diagnostics_, publish_diagnostics_);
    pnh_.param<double>("diagnostics_rate", diagnostics_rate_, diagnostics_rate_);

    sub_master_wrench_ = nh_.subscribe(master_wrench_topic_, 1, &TeleopMasterHapticController::masterWrenchCb, this,
                                      ros::TransportHints().tcpNoDelay());

    if (!slave_wrench_topic_.empty())
    {
      sub_slave_wrench_ = nh_.subscribe(slave_wrench_topic_, 1, &TeleopMasterHapticController::slaveWrenchCb, this,
                                        ros::TransportHints().tcpNoDelay());
    }
    else
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller", "slave_wrench_topic is empty: force reflection disabled.");
    }

    if (!coupling_wrench_topic_.empty())
    {
      sub_coupling_wrench_ = nh_.subscribe(coupling_wrench_topic_, 1, &TeleopMasterHapticController::couplingWrenchCb, this,
                                           ros::TransportHints().tcpNoDelay());
    }

    if (!force_reflection_gate_topic_.empty())
    {
      sub_force_reflection_gate_ = nh_.subscribe(force_reflection_gate_topic_, 1,
                                                 &TeleopMasterHapticController::forceReflectionGateCb, this,
                                                 ros::TransportHints().tcpNoDelay());
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Force reflection gate enabled: topic='%s'",
                     force_reflection_gate_topic_.c_str());
    }

    if (!home_return_disable_topic_.empty())
    {
      sub_home_return_disable_ = nh_.subscribe(home_return_disable_topic_, 1,
                                               &TeleopMasterHapticController::homeReturnDisableCb, this,
                                               ros::TransportHints().tcpNoDelay());
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "External home-return disable enabled: topic='%s'",
                     home_return_disable_topic_.c_str());
    }

    if (force_reflection_bias_enabled_)
    {
      if (!force_reflection_bias_active_topic_.empty())
      {
        sub_force_reflection_bias_active_ = nh_.subscribe(force_reflection_bias_active_topic_, 1,
                                                          &TeleopMasterHapticController::forceReflectionBiasActiveCb,
                                                          this,
                                                          ros::TransportHints().tcpNoDelay());
        ROS_INFO_NAMED("teleop_master_haptic_controller",
                       "Force reflection software zero enabled: active_topic='%s' delay=%.3fs window=%.3fs",
                       force_reflection_bias_active_topic_.c_str(),
                       force_reflection_bias_capture_delay_s_,
                       force_reflection_bias_capture_window_s_);
      }
      else
      {
        ROS_WARN_NAMED("teleop_master_haptic_controller",
                       "force_reflection_bias/enabled is true but force_reflection_bias/active_topic is empty.");
      }
    }

    if (!slave_actual_pose_topic_.empty())
    {
      sub_slave_actual_pose_ = nh_.subscribe(slave_actual_pose_topic_, 1,
                                              &TeleopMasterHapticController::slaveActualPoseCb, this,
                                              ros::TransportHints().tcpNoDelay());
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Virtual spring enabled: K_lin=%.2f B_lin=%.2f max=%.1f N, subscribing to '%s'",
                     spring_k_lin_, spring_b_lin_, max_spring_force_,
                     slave_actual_pose_topic_.c_str());
    }
    else if (spring_k_lin_ > 0.0 || spring_k_ang_ > 0.0)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Spring stiffness set but slave_actual_pose_topic is empty: spring disabled.");
    }

    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    pub_cmd_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>(command_topic_ + "_stamped", 1);
    if (!home_return_status_topic_.empty())
    {
      pub_home_return_status_ = nh_.advertise<std_msgs::String>(home_return_status_topic_, 1, true);
      if (home_return_status_publish_rate_ > 0.0)
      {
        home_return_status_timer_ = nh_.createTimer(ros::Duration(1.0 / home_return_status_publish_rate_),
                                                    &TeleopMasterHapticController::homeReturnStatusTick, this);
      }
      publishHomeReturnStatusMessage(home_return_status_);
    }
    if (publish_diagnostics_)
    {
      // Keep all debug topics private so dual-arm runs do not merge left/right streams.
      pub_debug_stats_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/admittance_stats", 1);
      pub_debug_admittance_dynamics_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/admittance_dynamics", 1);
      pub_debug_dt_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/dt_stats", 1);
      pub_debug_v_pre_ = pnh_.advertise<geometry_msgs::TwistStamped>("debug/v_cmd_pre", 1);
      pub_debug_v_post_ = pnh_.advertise<geometry_msgs::TwistStamped>("debug/v_cmd_post", 1);
      if (passivity_publish_debug_)
      {
        // Keep passivity debug scoped to the node instance so left/right controllers
        // publish on different topics in the dual-arm launch.
        pub_debug_passivity_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/passivity_stats", 1);
      }
    }
    debug_master_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                "filtered_master_wrench_topic", "debug/master_wrench_filtered");
    debug_slave_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                               "filtered_slave_wrench_topic", "debug/slave_wrench_filtered");
    debug_coupling_filt_pub_.init(nh_, pnh_, "publish_filtered_wrench_debug",
                                  "filtered_coupling_wrench_topic", "debug/coupling_wrench_filtered");

    if (publish_slave_targets_)
    {
      pub_slave_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(slave_target_pose_topic_, 1);
      pub_slave_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(slave_ff_twist_topic_, 1);

      const double slave_period = (slave_publish_rate_ > 0.0) ? (1.0 / slave_publish_rate_) : 0.01;
      slave_timer_ = nh_.createTimer(ros::Duration(slave_period),
                                     &TeleopMasterHapticController::slaveTargetTick, this);

      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Slave targets enabled: pose='%s', twist='%s' at %.0f Hz, TF %s->%s",
                     slave_target_pose_topic_.c_str(), slave_ff_twist_topic_.c_str(),
                     slave_publish_rate_, slave_base_frame_.c_str(), slave_tcp_frame_.c_str());
    }

    const double period = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopMasterHapticController::tick, this);

    if (passivity_config_.enabled)
    {
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Passivity layer enabled: linear_only=%d E_init=%.3f E_min=%.3f E_max=%.3f deadband=%.3f gamma_min=%.3f",
                     passivity_config_.linear_only ? 1 : 0,
                     passivity_config_.tank_energy_init,
                     passivity_config_.tank_energy_min,
                     passivity_config_.tank_energy_max,
                     passivity_config_.power_deadband,
                     passivity_config_.gamma_min);
    }
    if (home_return_enabled_ && has_home_target_)
    {
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Home return enabled: timeout=%.2fs blend=%.2fs F_thr=%.2fN Tau_thr=%.2fNm",
                     home_return_timeout_s_,
                     home_return_blend_in_time_s_,
                     home_return_force_activity_threshold_,
                     home_return_torque_activity_threshold_);
    }
    if (pub_home_return_status_)
    {
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "Home return status topic enabled: topic='%s' rate=%.2f Hz feedback_timeout=%.3fs pos_tol=%.3fm ori_tol=%.3frad",
                     home_return_status_topic_.c_str(),
                     home_return_status_publish_rate_,
                     home_return_feedback_timeout_s_,
                     home_return_arrival_position_tolerance_m_,
                     home_return_arrival_orientation_tolerance_rad_);
    }
  }

private:
  static bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& v, double& out)
  {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out = static_cast<int>(v);
      return std::isfinite(out);
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      out = static_cast<double>(v);
      return std::isfinite(out);
    }
    return false;
  }

  static bool tryGetVector3Param(ros::NodeHandle& pnh, const std::string& name, Eigen::Vector3d& out)
  {
    if (!pnh.hasParam(name))
    {
      return false;
    }
    XmlRpc::XmlRpcValue v;
    if (!pnh.getParam(name, v))
    {
      return false;
    }
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() != 3)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' exists but is not a 3-element array. Ignoring.", name.c_str());
      return false;
    }

    double x = 0.0, y = 0.0, z = 0.0;
    if (!xmlRpcToDouble(v[0], x) || !xmlRpcToDouble(v[1], y) || !xmlRpcToDouble(v[2], z))
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' array must contain only int/double values. Ignoring.", name.c_str());
      return false;
    }
    out = Eigen::Vector3d(x, y, z);
    return out.allFinite();
  }

  static bool tryGetQuaternionParam(ros::NodeHandle& pnh, const std::string& name, Eigen::Quaterniond& out)
  {
    if (!pnh.hasParam(name))
    {
      return false;
    }
    XmlRpc::XmlRpcValue v;
    if (!pnh.getParam(name, v))
    {
      return false;
    }
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() != 4)
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' exists but is not a 4-element array [x,y,z,w]. Ignoring.", name.c_str());
      return false;
    }

    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
    if (!xmlRpcToDouble(v[0], x) || !xmlRpcToDouble(v[1], y) ||
        !xmlRpcToDouble(v[2], z) || !xmlRpcToDouble(v[3], w))
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' array must contain only int/double values. Ignoring.", name.c_str());
      return false;
    }

    out = Eigen::Quaterniond(w, x, y, z);
    const double q_norm = out.norm();
    if (!(q_norm > teleoperation::kMathEps) || !std::isfinite(q_norm))
    {
      ROS_WARN_NAMED("teleop_master_haptic_controller",
                     "Param '%s' contains an invalid quaternion. Ignoring.", name.c_str());
      return false;
    }
    out.normalize();
    return true;
  }

  static Eigen::Vector3d expandScalarTo3(double x)
  {
    return Eigen::Vector3d(x, x, x);
  }

  static Eigen::Vector3d sanitizePositiveVec(const Eigen::Vector3d& v, double min_val)
  {
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(out[i]) || out[i] < min_val)
      {
        out[i] = min_val;
      }
    }
    return out;
  }

  static Eigen::Vector3d sanitizeNonNegativeVec(const Eigen::Vector3d& v)
  {
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(out[i]) || out[i] < 0.0)
      {
        out[i] = 0.0;
      }
    }
    return out;
  }

  static Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d& rpy)
  {
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  }

  static Eigen::Vector3d quaternionLog(const Eigen::Quaterniond& q_in)
  {
    Eigen::Quaterniond q = q_in.normalized();
    if (q.w() < 0.0)
    {
      q.coeffs() = -q.coeffs();
    }

    const Eigen::AngleAxisd aa(q);
    const double angle = aa.angle();
    if (!std::isfinite(angle) || std::abs(angle) < teleoperation::kMathEps)
    {
      return Eigen::Vector3d::Zero();
    }
    return angle * aa.axis();
  }

  static Eigen::Quaterniond quaternionExp(const Eigen::Vector3d& rotvec)
  {
    const double angle = rotvec.norm();
    if (!std::isfinite(angle) || angle < teleoperation::kMathEps)
    {
      return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotvec / angle)).normalized();
  }

  static Eigen::Quaterniond scaledRelativeOrientation(const Eigen::Quaterniond& reference,
                                                      const Eigen::Quaterniond& current,
                                                      double scale)
  {
    Eigen::Quaterniond q_ref = reference.normalized();
    Eigen::Quaterniond q_curr = current.normalized();
    if (q_ref.dot(q_curr) < 0.0)
    {
      q_curr.coeffs() = -q_curr.coeffs();
    }

    const Eigen::Quaterniond q_delta = (q_ref.conjugate() * q_curr).normalized();
    const Eigen::Vector3d rotvec = quaternionLog(q_delta);
    return (q_ref * quaternionExp(rotvec / scale)).normalized();
  }

  static Eigen::Quaterniond inverseScaledRelativeOrientation(const Eigen::Quaterniond& master_reference,
                                                             const Eigen::Quaterniond& slave_reference,
                                                             const Eigen::Quaterniond& slave_current,
                                                             double scale)
  {
    Eigen::Quaterniond q_slave_ref = slave_reference.normalized();
    Eigen::Quaterniond q_slave_curr = slave_current.normalized();
    if (q_slave_ref.dot(q_slave_curr) < 0.0)
    {
      q_slave_curr.coeffs() = -q_slave_curr.coeffs();
    }

    const Eigen::Quaterniond q_delta = (q_slave_ref.conjugate() * q_slave_curr).normalized();
    const Eigen::Vector3d rotvec = quaternionLog(q_delta);
    return (master_reference.normalized() * quaternionExp(scale * rotvec)).normalized();
  }

  bool motionScalingActive() const
  {
    return motion_scaling_enabled_ && motion_scaling_config_valid_;
  }

  // softDeadzoneNormWithHysteresis is now in teleoperation::math_utils.hpp

  static double computeFilterAlpha(double dt, double alpha_param, double cutoff_hz_param)
  {
    if (cutoff_hz_param > 0.0 && std::isfinite(cutoff_hz_param))
    {
      return teleoperation::lowpassAlphaFromCutoffHz(dt, cutoff_hz_param);
    }
    return std::clamp(alpha_param, 0.0, 1.0);
  }

  static double smoothstep01(double t)
  {
    const double x = std::clamp(t, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  static double applyScheduleShape(double schedule_value, double shape_exp)
  {
    const double x = std::clamp(schedule_value, 0.0, 1.0);
    if (!std::isfinite(shape_exp) || shape_exp <= 0.0)
    {
      return x;
    }
    if (std::abs(shape_exp - 1.0) < 1e-9)
    {
      return x;
    }
    return std::clamp(1.0 - std::pow(1.0 - x, shape_exp), 0.0, 1.0);
  }

  static double computeScheduleSmoothstep(double F_env, double force_start, double force_stop, double shape_exp = 1.0)
  {
    if (!std::isfinite(F_env))
    {
      return 0.0;
    }
    const double f0 = force_start;
    const double f1 = force_stop;
    if (!(f1 > f0))
    {
      // Degenerate config: behave like a step at f0.
      return (F_env > f0) ? 1.0 : 0.0;
    }
    if (F_env <= f0) return 0.0;
    if (F_env >= f1) return 1.0;
    const double u = (F_env - f0) / (f1 - f0);
    return applyScheduleShape(smoothstep01(u), shape_exp);
  }

  void resetForceReflectionBiasState()
  {
    has_force_reflection_bias_ = false;
    force_reflection_bias_capturing_ = false;
    force_reflection_bias_capture_started_ = ros::Time(0);
    force_reflection_bias_captured_time_ = ros::Time(0);
    force_reflection_bias_sample_count_ = 0;
    force_reflection_bias_sum_.f.setZero();
    force_reflection_bias_sum_.tau.setZero();
    force_reflection_bias_.f.setZero();
    force_reflection_bias_.tau.setZero();
    force_reflection_delta_filt_.f.setZero();
    force_reflection_delta_filt_.tau.setZero();
    force_reflection_delta_db_state_ = teleoperation::WrenchDeadbandState{};
  }

  void updateForceReflectionBiasState(bool active,
                                      const ros::Time& active_since,
                                      bool has_slave,
                                      const Wrench3& slave_wrench,
                                      const ros::Time& now)
  {
    if (!force_reflection_bias_enabled_)
    {
      return;
    }

    if (!active)
    {
      force_reflection_bias_last_active_ = false;
      resetForceReflectionBiasState();
      return;
    }

    if (!force_reflection_bias_last_active_)
    {
      resetForceReflectionBiasState();
      force_reflection_bias_last_active_ = true;
    }

    if (has_force_reflection_bias_ || !has_slave)
    {
      return;
    }

    const ros::Time activation_time = active_since.isZero() ? now : active_since;
    if ((now - activation_time).toSec() < force_reflection_bias_capture_delay_s_)
    {
      return;
    }

    if (!force_reflection_bias_capturing_)
    {
      force_reflection_bias_capturing_ = true;
      force_reflection_bias_capture_started_ = now;
      force_reflection_bias_sample_count_ = 0;
      force_reflection_bias_sum_.f.setZero();
      force_reflection_bias_sum_.tau.setZero();
    }

    force_reflection_bias_sum_.f += slave_wrench.f;
    force_reflection_bias_sum_.tau += slave_wrench.tau;
    force_reflection_bias_sample_count_ += 1;

    const double capture_elapsed = (now - force_reflection_bias_capture_started_).toSec();
    if (force_reflection_bias_capture_window_s_ > 0.0 &&
        capture_elapsed < force_reflection_bias_capture_window_s_)
    {
      return;
    }

    const double n = static_cast<double>(std::max(1, force_reflection_bias_sample_count_));
    force_reflection_bias_.f = force_reflection_bias_sum_.f / n;
    force_reflection_bias_.tau = force_reflection_bias_sum_.tau / n;
    has_force_reflection_bias_ = true;
    force_reflection_bias_capturing_ = false;
    force_reflection_bias_captured_time_ = now;
    force_reflection_delta_filt_.f.setZero();
    force_reflection_delta_filt_.tau.setZero();
    force_reflection_delta_db_state_ = teleoperation::WrenchDeadbandState{};

    ROS_INFO_NAMED("teleop_master_haptic_controller",
                   "Captured force reflection software zero: |F|=%.3f N |Tau|=%.3f Nm samples=%d",
                   force_reflection_bias_.f.norm(),
                   force_reflection_bias_.tau.norm(),
                   force_reflection_bias_sample_count_);
  }

  double forceReflectionBiasDeltaGain(const ros::Time& now) const
  {
    if (!has_force_reflection_bias_)
    {
      return 0.0;
    }
    if (force_reflection_bias_delta_fade_s_ <= 0.0 || force_reflection_bias_captured_time_.isZero())
    {
      return 1.0;
    }
    return smoothstep01((now - force_reflection_bias_captured_time_).toSec() /
                        force_reflection_bias_delta_fade_s_);
  }

  void resetControllerState()
  {
    v_lin_cmd_.setZero();
    v_ang_cmd_.setZero();
    a_lin_limiter_.reset();
    a_ang_limiter_.reset();

    has_filtered_master_ = false;
    has_filtered_slave_ = false;
    has_filtered_coupling_ = false;

    master_db_state_ = teleoperation::WrenchDeadbandState{};
    slave_db_state_ = teleoperation::WrenchDeadbandState{};
    coupling_db_state_ = teleoperation::WrenchDeadbandState{};

    has_p_slave_prev_ = false;

    has_dyn_d_extra_filt_ = false;
    dyn_d_extra_lin_filt_.setZero();
    dyn_d_extra_ang_filt_.setZero();

    has_dyn_m_extra_filt_ = false;
    dyn_m_extra_lin_filt_.setZero();
    dyn_m_extra_ang_filt_.setZero();

    resetForceReflectionBiasState();

    passivity_layer_.reset();
    home_integral_lin_.setZero();
    home_integral_ang_.setZero();
    home_blend_start_time_ = ros::Time(0);
  }

  bool lookupMasterPose(Eigen::Vector3d& p_master,
                        Eigen::Quaterniond& q_master,
                        double warn_throttle_s,
                        const std::string& context) const
  {
    geometry_msgs::TransformStamped T;
    try
    {
      T = tf_buffer_.lookupTransform(slave_base_frame_, slave_tcp_frame_, ros::Time(0),
                                     ros::Duration(tf_timeout_s_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(warn_throttle_s, "teleop_master_haptic_controller",
                              "%s TF lookup failed (%s -> %s): %s",
                              context.c_str(), slave_base_frame_.c_str(), slave_tcp_frame_.c_str(), ex.what());
      return false;
    }

    p_master = Eigen::Vector3d(T.transform.translation.x,
                               T.transform.translation.y,
                               T.transform.translation.z);
    q_master = Eigen::Quaterniond(T.transform.rotation.w,
                                  T.transform.rotation.x,
                                  T.transform.rotation.y,
                                  T.transform.rotation.z);
    if (q_master.norm() < teleoperation::kMathEps)
    {
      ROS_WARN_THROTTLE_NAMED(warn_throttle_s, "teleop_master_haptic_controller",
                              "%s TF returned an invalid quaternion. Skipping sample.",
                              context.c_str());
      return false;
    }
    q_master.normalize();
    return true;
  }

  bool wrenchMsgToWrench3(const geometry_msgs::WrenchStamped& msg, Wrench3& out) const
  {
    const Eigen::Vector3d f_src = teleoperation::vector3MsgToEigen(msg.wrench.force);
    const Eigen::Vector3d t_src = teleoperation::vector3MsgToEigen(msg.wrench.torque);

    Eigen::Vector3d f_tgt, t_tgt;
    const std::string src_frame = wrench_source_frame_override_.empty() ? msg.header.frame_id : wrench_source_frame_override_;
    const ros::Time stamp = use_latest_tf_for_wrench_ ? ros::Time(0) : msg.header.stamp;

    if (!teleoperation::rotateVectorToFrame(tf_buffer_, wrench_target_frame_, src_frame, stamp, tf_timeout_s_,
                                            f_src, f_tgt, "teleop_master_haptic_controller"))
    {
      return false;
    }
    if (!teleoperation::rotateVectorToFrame(tf_buffer_, wrench_target_frame_, src_frame, stamp, tf_timeout_s_,
                                            t_src, t_tgt, "teleop_master_haptic_controller"))
    {
      return false;
    }

    out.f = f_tgt;
    out.tau = t_tgt;
    return true;
  }

  void masterWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w))
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "masterWrenchCb: wrenchMsgToWrench3 failed (TF rotation from '%s' to '%s'). Dropping message.",
                              (wrench_source_frame_override_.empty() ? msg->header.frame_id : wrench_source_frame_override_).c_str(),
                              wrench_target_frame_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    master_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    master_wrench_raw_ = w;
    has_master_ = true;
  }

  void slaveWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    slave_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    slave_wrench_raw_ = w;
    has_slave_ = true;
  }

  void couplingWrenchCb(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    Wrench3 w;
    if (!wrenchMsgToWrench3(*msg, w)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    coupling_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    coupling_wrench_raw_ = w;
    has_coupling_ = true;
  }

  void forceReflectionGateCb(const std_msgs::Float64ConstPtr& msg)
  {
    const double gate = std::isfinite(msg->data) ? std::clamp(msg->data, 0.0, 1.0) : 1.0;
    std::lock_guard<std::mutex> lock(mutex_);
    force_reflection_gate_ = gate;
  }

  void forceReflectionBiasActiveCb(const std_msgs::BoolConstPtr& msg)
  {
    const bool active = msg->data;
    std::lock_guard<std::mutex> lock(mutex_);
    if (active && (!force_reflection_bias_active_ || force_reflection_bias_active_since_.isZero()))
    {
      force_reflection_bias_active_since_ = ros::Time::now();
    }
    else if (!active)
    {
      force_reflection_bias_active_since_ = ros::Time(0);
    }
    force_reflection_bias_active_ = active;
  }

  void homeReturnDisableCb(const std_msgs::BoolConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    home_return_externally_disabled_ = msg->data;
  }

  void slaveActualPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    const Eigen::Vector3d raw_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    if (!(q.norm() > teleoperation::kMathEps) || !std::isfinite(q.norm()))
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                              "slaveActualPoseCb received an invalid quaternion. Dropping sample.");
      return;
    }
    const Eigen::Quaterniond raw_ori = q.normalized();

    Eigen::Vector3d spring_pos = raw_pos;
    Eigen::Quaterniond spring_ori = raw_ori;
    if (motionScalingActive())
    {
      spring_pos = motion_master_neutral_pos_ +
                   motion_translation_scale_ * (raw_pos - motion_master_neutral_pos_);
      spring_ori = inverseScaledRelativeOrientation(motion_master_neutral_ori_,
                                                    motion_master_neutral_ori_,
                                                    raw_ori,
                                                    motion_rotation_scale_);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    slave_actual_raw_pos_ = raw_pos;
    slave_actual_raw_ori_ = raw_ori;
    slave_actual_pos_ = spring_pos;
    slave_actual_ori_ = spring_ori;
    slave_actual_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_slave_actual_pose_ = true;
  }

  void publishZero()
  {
    geometry_msgs::Twist cmd;
    pub_cmd_.publish(cmd);
    if (pub_cmd_stamped_)
    {
      geometry_msgs::TwistStamped stamped;
      stamped.header.stamp = ros::Time::now();
      stamped.header.frame_id = wrench_target_frame_;
      stamped.twist = cmd;
      pub_cmd_stamped_.publish(stamped);
    }
  }

  void publishHomeReturnStatusMessage(const std::string& status)
  {
    if (!pub_home_return_status_)
    {
      return;
    }

    std_msgs::String msg;
    msg.data = status;
    pub_home_return_status_.publish(msg);
  }

  void setHomeReturnStatus(const std::string& status)
  {
    const std::string next_status = status.empty() ? std::string("inactive") : status;
    if (home_return_status_ == next_status)
    {
      return;
    }

    home_return_status_ = next_status;
    ROS_INFO_NAMED("teleop_master_haptic_controller",
                   "[%s] Home return status -> %s",
                   node_name_.c_str(),
                   home_return_status_.c_str());
    publishHomeReturnStatusMessage(home_return_status_);
  }

  void homeReturnStatusTick(const ros::TimerEvent& /*ev*/)
  {
    publishHomeReturnStatusMessage(home_return_status_);
  }

  bool getFreshSlaveActualPoseForHome(const ros::Time& now,
                                      Eigen::Vector3d& slave_pos,
                                      Eigen::Quaterniond& slave_ori,
                                      double& feedback_age_s) const
  {
    ros::Time pose_stamp;
    bool has_pose = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_pose = has_slave_actual_pose_;
      slave_pos = slave_actual_raw_pos_;
      slave_ori = slave_actual_raw_ori_;
      pose_stamp = slave_actual_pose_stamp_;
    }

    if (!has_pose)
    {
      feedback_age_s = std::numeric_limits<double>::infinity();
      return false;
    }

    feedback_age_s = std::fabs((now - pose_stamp).toSec());
    if (home_return_feedback_timeout_s_ > 0.0 &&
        feedback_age_s > home_return_feedback_timeout_s_)
    {
      return false;
    }

    return true;
  }

  void slaveTargetTick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

    const std::string frame_out = slave_frame_id_override_.empty()
                                      ? slave_base_frame_
                                      : slave_frame_id_override_;

    Eigen::Vector3d p_master = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_master = Eigen::Quaterniond::Identity();
    if (!lookupMasterPose(p_master, q_master, 1.0, "Slave target"))
    {
      return;
    }

    Eigen::Vector3d p_out = slave_pose_twist_rotation_ * p_master;
    Eigen::Quaterniond q_out = (slave_pose_twist_rotation_q_ * q_master).normalized();
    if (motionScalingActive())
    {
      const Eigen::Vector3d p_master_mapped = p_out;
      const Eigen::Vector3d p_master_neutral_mapped =
          slave_pose_twist_rotation_ * motion_master_neutral_pos_;
      const Eigen::Quaterniond q_master_mapped = q_out;
      const Eigen::Quaterniond q_master_neutral_mapped =
          (slave_pose_twist_rotation_q_ * motion_master_neutral_ori_).normalized();

      p_out = p_master_neutral_mapped +
              (p_master_mapped - p_master_neutral_mapped) / motion_translation_scale_;
      q_out = scaledRelativeOrientation(q_master_neutral_mapped,
                                        q_master_mapped,
                                        motion_rotation_scale_);
    }

    // Publish PoseStamped
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = frame_out;
    pose_msg.pose.position.x = p_out.x();
    pose_msg.pose.position.y = p_out.y();
    pose_msg.pose.position.z = p_out.z();
    pose_msg.pose.orientation.w = q_out.w();
    pose_msg.pose.orientation.x = q_out.x();
    pose_msg.pose.orientation.y = q_out.y();
    pose_msg.pose.orientation.z = q_out.z();
    pub_slave_pose_.publish(pose_msg);

    // Publish commanded twist (already computed by admittance loop)
    Eigen::Vector3d v_lin, v_ang;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v_lin = v_lin_cmd_;
      v_ang = v_ang_cmd_;
    }
    Eigen::Vector3d v_lin_out = slave_pose_twist_rotation_ * v_lin;
    Eigen::Vector3d v_ang_out = slave_pose_twist_rotation_ * v_ang;
    if (motionScalingActive())
    {
      v_lin_out /= motion_translation_scale_;
      v_ang_out /= motion_rotation_scale_;
    }

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = frame_out;
    twist_msg.twist.linear.x = v_lin_out.x();
    twist_msg.twist.linear.y = v_lin_out.y();
    twist_msg.twist.linear.z = v_lin_out.z();
    twist_msg.twist.angular.x = v_ang_out.x();
    twist_msg.twist.angular.y = v_ang_out.y();
    twist_msg.twist.angular.z = v_ang_out.z();
    pub_slave_twist_.publish(twist_msg);
  }

  void tick(const ros::TimerEvent& ev)
  {
    const ros::Time now = ev.current_real.isZero() ? ros::Time::now() : ev.current_real;

    Wrench3 master_raw, slave_raw, coupling_raw;
    ros::Time master_stamp, slave_stamp, coupling_stamp;
    bool has_slave = false;
    bool has_coupling = false;
    double force_reflection_gate = 1.0;
    bool force_reflection_bias_active = false;
    ros::Time force_reflection_bias_active_since;
    bool home_return_externally_disabled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_master_)
      {
        setHomeReturnStatus("inactive");
        publishZero();
        return;
      }
      master_raw = master_wrench_raw_;
      master_stamp = master_stamp_;

      has_slave = has_slave_;
      slave_raw = slave_wrench_raw_;
      slave_stamp = slave_stamp_;

      has_coupling = has_coupling_;
      coupling_raw = coupling_wrench_raw_;
      coupling_stamp = coupling_stamp_;

      force_reflection_gate = force_reflection_gate_;
      force_reflection_bias_active = force_reflection_bias_active_;
      force_reflection_bias_active_since = force_reflection_bias_active_since_;
      home_return_externally_disabled = home_return_externally_disabled_;
    }
    force_reflection_gate = std::clamp(force_reflection_gate, 0.0, 1.0);

    const bool master_stale = ((now - master_stamp).toSec() > wrench_timeout_s_);
    const bool slave_stale = has_slave && ((now - slave_stamp).toSec() > wrench_timeout_s_);
    const bool coupling_stale = has_coupling && ((now - coupling_stamp).toSec() > wrench_timeout_s_);

    const bool any_stale = (master_stale || (has_slave && slave_stale) || (has_coupling && coupling_stale));

    if (any_stale)
    {
      const std::string stale_sources = describeStaleSources(master_stale, slave_stale, coupling_stale);
      if (!stale_active_)
      {
        ROS_WARN_NAMED("teleop_master_haptic_controller",
                       "[%s] Wrench became stale on %s (master=%d slave=%d coupling=%d, timeout=%.3fs). Publishing zero%s.",
                       node_name_.c_str(),
                       stale_sources.c_str(),
                       master_stale,
                       slave_stale,
                       coupling_stale,
                       wrench_timeout_s_,
                       (reset_on_stale_ ? " + resetting internal state (reset_on_stale=true)" : ""));
      }
      else
      {
        ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                                "[%s] Stale wrench on %s (master=%d slave=%d coupling=%d). Publishing zero.",
                                node_name_.c_str(),
                                stale_sources.c_str(),
                                master_stale, slave_stale, coupling_stale);
      }
      setHomeReturnStatus("inactive");
      publishZero();
      if (reset_on_stale_)
      {
        resetControllerState();
      }
      stale_active_ = true;
      last_time_ = now;
      return;
    }
    if (stale_active_)
    {
      ROS_INFO_NAMED("teleop_master_haptic_controller",
                     "[%s] Wrench recovered (no longer stale).",
                     node_name_.c_str());
    }
    stale_active_ = false;

    const double dt_nominal = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.01;
    const double dt_min = std::max(0.0, dt_min_factor_) * dt_nominal;
    const double dt_max = std::max(0.0, dt_max_factor_) * dt_nominal;

    double dt_raw = 0.0;
    if (!ev.last_real.isZero() && !ev.current_real.isZero())
    {
      dt_raw = (ev.current_real - ev.last_real).toSec();
    }
    else if (!last_time_.isZero())
    {
      dt_raw = (now - last_time_).toSec();
    }
    last_time_ = now;

    if (!(dt_raw > 0.0) || !std::isfinite(dt_raw))
    {
      setHomeReturnStatus("inactive");
      publishZero();
      return;
    }

    double dt_used = dt_raw;
    if (dt_min > 0.0 && dt_used < dt_min)
    {
      dt_used = dt_nominal;
    }

    int n_substeps = 1;
    if (dt_max > 0.0 && dt_used > dt_max)
    {
      if (dt_use_substepping_)
      {
        n_substeps = static_cast<int>(std::ceil(dt_used / dt_max));
        n_substeps = std::clamp(n_substeps, 1, std::max(1, dt_max_substeps_));
      }
      else
      {
        dt_used = dt_max;
        n_substeps = 1;
      }
    }
    const double dt_step = dt_used / static_cast<double>(n_substeps);
    if (!(dt_step > 0.0) || !std::isfinite(dt_step))
    {
      setHomeReturnStatus("inactive");
      publishZero();
      return;
    }

    // Filter + soft deadzone (norm, with optional hysteresis) + clamp on norm.
    // NOTE: 3.4 is intentionally rotation-only: wrenchMsgToWrench3 rotates vectors but does not apply p x f.
    const double dt_for_filter = std::clamp(dt_raw, std::max(1e-6, dt_min), (dt_max > 0.0 ? dt_max : dt_raw));
    const double alpha_master =
        computeFilterAlpha(dt_for_filter,
                           (master_wrench_filter_alpha_ >= 0.0 ? master_wrench_filter_alpha_ : wrench_filter_alpha_),
                           (master_wrench_filter_cutoff_hz_ > 0.0 ? master_wrench_filter_cutoff_hz_ : wrench_filter_cutoff_hz_));
    const double alpha_feedback =
        computeFilterAlpha(dt_for_filter,
                           (feedback_wrench_filter_alpha_ >= 0.0 ? feedback_wrench_filter_alpha_ : wrench_filter_alpha_),
                           (feedback_wrench_filter_cutoff_hz_ > 0.0 ? feedback_wrench_filter_cutoff_hz_ : wrench_filter_cutoff_hz_));

    const bool use_master_filter = (alpha_master > 0.0) && (alpha_master < 1.0);
    const bool use_feedback_filter = (alpha_feedback > 0.0) && (alpha_feedback < 1.0);

    // Master wrench: EMA + soft norm deadband with hysteresis + clamp
    if (!has_filtered_master_)
    {
      master_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
          master_raw, master_raw, false, alpha_master,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          (max_force_hand_ > 0.0 ? max_force_hand_ : max_force_),
          (max_torque_hand_ > 0.0 ? max_torque_hand_ : max_torque_),
          use_torques_, cross_deadband_scale_, master_db_state_);
      has_filtered_master_ = true;
    }
    else
    {
      master_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
          master_filt_, master_raw, use_master_filter, alpha_master,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          (max_force_hand_ > 0.0 ? max_force_hand_ : max_force_),
          (max_torque_hand_ > 0.0 ? max_torque_hand_ : max_torque_),
          use_torques_, cross_deadband_scale_, master_db_state_);
    }
    if (!use_forces_)
    {
      master_filt_.f.setZero();
      master_db_state_.f_active = false;
    }

    // Slave wrench
    if (has_slave)
    {
      if (!has_filtered_slave_)
      {
        slave_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
            slave_raw, slave_raw, false, alpha_feedback,
            force_deadband_enter_, force_deadband_exit_,
            torque_deadband_enter_, torque_deadband_exit_,
            (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_),
            (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_),
            use_torques_, cross_deadband_scale_, slave_db_state_);
        has_filtered_slave_ = true;
      }
      else
      {
        slave_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
            slave_filt_, slave_raw, use_feedback_filter, alpha_feedback,
            force_deadband_enter_, force_deadband_exit_,
            torque_deadband_enter_, torque_deadband_exit_,
            (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_),
            (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_),
            use_torques_, cross_deadband_scale_, slave_db_state_);
      }
      if (!use_forces_)
      {
        slave_filt_.f.setZero();
        slave_db_state_.f_active = false;
      }
    }
    else
    {
      slave_filt_.f.setZero();
      slave_filt_.tau.setZero();
      slave_db_state_ = teleoperation::WrenchDeadbandState{};
    }

    // Coupling wrench
    if (has_coupling)
    {
      if (!has_filtered_coupling_)
      {
        coupling_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
            coupling_raw, coupling_raw, false, alpha_feedback,
            force_deadband_enter_, force_deadband_exit_,
            torque_deadband_enter_, torque_deadband_exit_,
            (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_),
            (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_),
            use_torques_, cross_deadband_scale_, coupling_db_state_);
        has_filtered_coupling_ = true;
      }
      else
      {
        coupling_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
            coupling_filt_, coupling_raw, use_feedback_filter, alpha_feedback,
            force_deadband_enter_, force_deadband_exit_,
            torque_deadband_enter_, torque_deadband_exit_,
            (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_),
            (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_),
            use_torques_, cross_deadband_scale_, coupling_db_state_);
      }
      if (!use_forces_)
      {
        coupling_filt_.f.setZero();
        coupling_db_state_.f_active = false;
      }
    }
    else
    {
      coupling_filt_.f.setZero();
      coupling_filt_.tau.setZero();
      coupling_db_state_ = teleoperation::WrenchDeadbandState{};
    }

    updateForceReflectionBiasState(force_reflection_bias_active,
                                   force_reflection_bias_active_since,
                                   has_slave,
                                   slave_filt_,
                                   now);

    debug_master_filt_pub_.publish(master_filt_, now, wrench_target_frame_);
    debug_slave_filt_pub_.publish(slave_filt_, now, wrench_target_frame_);
    debug_coupling_filt_pub_.publish(coupling_filt_, now, wrench_target_frame_);

    const bool any_dyn_from_slave = (dyn_damping_enabled_ || dyn_mass_enabled_);
    if (any_dyn_from_slave)
    {
      // Force metric (currently only norm is supported).
      double F_env = 0.0;
      double Tau_env = 0.0;
      const bool use_slave_for_damping = dyn_damping_enabled_ && dyn_damping_use_slave_wrench_;
      const bool use_slave_for_mass = dyn_mass_enabled_ && dyn_mass_use_slave_wrench_;
      if ((use_slave_for_damping || use_slave_for_mass) && has_slave)
      {
        F_env = slave_filt_.f.norm();
        Tau_env = slave_filt_.tau.norm();
      }

      // ---- Dynamic damping ----
      if (dyn_damping_enabled_)
      {
        if (dyn_damping_force_metric_ != "norm" && dyn_damping_force_metric_ != "Norm")
        {
          ROS_WARN_THROTTLE_NAMED(2.0, "teleop_master_haptic_controller",
                                  "dynamic_damping/force_metric='%s' not supported. Using 'norm'.",
                                  dyn_damping_force_metric_.c_str());
        }

        if (dyn_damping_use_torque_schedule_ &&
            (dyn_damping_torque_metric_ != "norm" && dyn_damping_torque_metric_ != "Norm"))
        {
          ROS_WARN_THROTTLE_NAMED(2.0, "teleop_master_haptic_controller",
                                  "dynamic_damping/torque_metric='%s' not supported. Using 'norm'.",
                                  dyn_damping_torque_metric_.c_str());
        }

        const double s_d_lin = computeScheduleSmoothstep(F_env, dyn_damping_force_start_, dyn_damping_force_stop_, dyn_damping_shape_exp_);
        const double s_d_ang = dyn_damping_use_torque_schedule_
                                   ? computeScheduleSmoothstep(Tau_env, dyn_damping_torque_start_, dyn_damping_torque_stop_, dyn_damping_shape_exp_)
                                   : s_d_lin;

        const Eigen::Vector3d Dextra_lin_max =
            sanitizeNonNegativeVec((dyn_extra_damping_linear_xyz_.allFinite() ? dyn_extra_damping_linear_xyz_ : expandScalarTo3(dyn_extra_damping_linear_)));
        const Eigen::Vector3d Dextra_ang_max =
            sanitizeNonNegativeVec((dyn_extra_damping_angular_xyz_.allFinite() ? dyn_extra_damping_angular_xyz_ : expandScalarTo3(dyn_extra_damping_angular_)));

        const Eigen::Vector3d d_extra_lin_target = s_d_lin * Dextra_lin_max;
        const Eigen::Vector3d d_extra_ang_target = s_d_ang * Dextra_ang_max;

        const bool do_d_filter = (dyn_d_extra_lowpass_cutoff_hz_ > 0.0) && std::isfinite(dyn_d_extra_lowpass_cutoff_hz_);
        const double alpha_d = do_d_filter ? teleoperation::lowpassAlphaFromCutoffHz(dt_for_filter, dyn_d_extra_lowpass_cutoff_hz_) : 1.0;

        if (!has_dyn_d_extra_filt_ || !do_d_filter)
        {
          dyn_d_extra_lin_filt_ = d_extra_lin_target;
          dyn_d_extra_ang_filt_ = d_extra_ang_target;
          has_dyn_d_extra_filt_ = true;
        }
        else
        {
          dyn_d_extra_lin_filt_ = teleoperation::ema3(dyn_d_extra_lin_filt_, d_extra_lin_target, alpha_d);
          dyn_d_extra_ang_filt_ = teleoperation::ema3(dyn_d_extra_ang_filt_, d_extra_ang_target, alpha_d);
        }
      }

      // ---- Dynamic mass ----
      if (dyn_mass_enabled_)
      {
        if (dyn_mass_force_metric_ != "norm" && dyn_mass_force_metric_ != "Norm")
        {
          ROS_WARN_THROTTLE_NAMED(2.0, "teleop_master_haptic_controller",
                                  "dynamic_mass/force_metric='%s' not supported. Using 'norm'.",
                                  dyn_mass_force_metric_.c_str());
        }

        if (dyn_mass_use_torque_schedule_ &&
            (dyn_mass_torque_metric_ != "norm" && dyn_mass_torque_metric_ != "Norm"))
        {
          ROS_WARN_THROTTLE_NAMED(2.0, "teleop_master_haptic_controller",
                                  "dynamic_mass/torque_metric='%s' not supported. Using 'norm'.",
                                  dyn_mass_torque_metric_.c_str());
        }

        const double s_m_lin = computeScheduleSmoothstep(F_env, dyn_mass_force_start_, dyn_mass_force_stop_, dyn_mass_shape_exp_);
        const double s_m_ang = dyn_mass_use_torque_schedule_
                                   ? computeScheduleSmoothstep(Tau_env, dyn_mass_torque_start_, dyn_mass_torque_stop_, dyn_mass_shape_exp_)
                                   : s_m_lin;

        const Eigen::Vector3d Mextra_lin_max =
            sanitizeNonNegativeVec((dyn_extra_mass_linear_xyz_.allFinite() ? dyn_extra_mass_linear_xyz_ : expandScalarTo3(dyn_extra_mass_linear_)));
        const Eigen::Vector3d Mextra_ang_max =
            sanitizeNonNegativeVec((dyn_extra_mass_angular_xyz_.allFinite() ? dyn_extra_mass_angular_xyz_ : expandScalarTo3(dyn_extra_mass_angular_)));

        const Eigen::Vector3d m_extra_lin_target = s_m_lin * Mextra_lin_max;
        const Eigen::Vector3d m_extra_ang_target = s_m_ang * Mextra_ang_max;

        const bool do_m_filter = (dyn_m_extra_lowpass_cutoff_hz_ > 0.0) && std::isfinite(dyn_m_extra_lowpass_cutoff_hz_);
        const double alpha_m = do_m_filter ? teleoperation::lowpassAlphaFromCutoffHz(dt_for_filter, dyn_m_extra_lowpass_cutoff_hz_) : 1.0;

        if (!has_dyn_m_extra_filt_ || !do_m_filter)
        {
          dyn_m_extra_lin_filt_ = m_extra_lin_target;
          dyn_m_extra_ang_filt_ = m_extra_ang_target;
          has_dyn_m_extra_filt_ = true;
        }
        else
        {
          dyn_m_extra_lin_filt_ = teleoperation::ema3(dyn_m_extra_lin_filt_, m_extra_lin_target, alpha_m);
          dyn_m_extra_ang_filt_ = teleoperation::ema3(dyn_m_extra_ang_filt_, m_extra_ang_target, alpha_m);
        }
      }
    }

    // Admittance dynamics (linear + optional angular).
    const Eigen::Vector3d F_hand = master_filt_.f;
    const Eigen::Vector3d Tau_hand = master_filt_.tau;

    // Virtual spring coupling: F_spring = K_s * (p_m - p_s) + B_s * (v_m - v_s)
    Eigen::Vector3d F_spring_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d Tau_spring = Eigen::Vector3d::Zero();
    Eigen::Vector3d F_home = Eigen::Vector3d::Zero();
    Eigen::Vector3d Tau_home = Eigen::Vector3d::Zero();

    const bool spring_enabled = has_slave_actual_pose_ &&
                                 (spring_k_lin_ > 0.0 || spring_k_ang_ > 0.0) &&
                                 publish_slave_targets_;
    const bool home_return_ready = home_return_enabled_ && has_home_target_ && !home_return_externally_disabled;
    const bool need_master_pose = spring_enabled || home_return_ready;
    Eigen::Vector3d p_master = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_master = Eigen::Quaterniond::Identity();
    const bool has_master_tcp = need_master_pose
                                    ? lookupMasterPose(p_master, q_master, 2.0, "Controller")
                                    : false;

    if (spring_enabled && has_master_tcp)
    {
      Eigen::Vector3d p_slave;
      Eigen::Quaterniond q_slave;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        p_slave = slave_actual_pos_;
        q_slave = slave_actual_ori_;
      }

      // Linear spring
      if (spring_k_lin_ > 0.0)
      {
        const Eigen::Vector3d delta_p = p_master - p_slave;
        F_spring_lin = spring_k_lin_ * delta_p;

        // Optional velocity damping via numerical differentiation of slave position
        if (spring_b_lin_ > 0.0 && has_p_slave_prev_)
        {
          const Eigen::Vector3d v_slave_est = (p_slave - p_slave_prev_) / dt_for_filter;
          const Eigen::Vector3d delta_v = v_lin_cmd_ - v_slave_est;
          F_spring_lin += spring_b_lin_ * delta_v;
        }

        F_spring_lin = teleoperation::clampNorm3(F_spring_lin, max_spring_force_);
      }

      // Angular spring
      if (spring_k_ang_ > 0.0 && use_torques_)
      {
        const Eigen::Vector3d delta_o = teleoperation::orientationErrorAxisAngle(q_slave, q_master);
        Tau_spring = spring_k_ang_ * delta_o;
        Tau_spring = teleoperation::clampNorm3(Tau_spring, max_spring_torque_);
      }

      p_slave_prev_ = p_slave;
      has_p_slave_prev_ = true;
    }

    if (last_operator_activity_time_.isZero())
    {
      last_operator_activity_time_ = now;
    }
    const bool operator_active =
        (F_hand.norm() > std::max(0.0, home_return_force_activity_threshold_)) ||
        (use_torques_ && (Tau_hand.norm() > std::max(0.0, home_return_torque_activity_threshold_)));
    if (operator_active)
    {
      last_operator_activity_time_ = now;
    }

    const bool home_should_be_active =
        home_return_ready &&
        ((now - last_operator_activity_time_).toSec() >= std::max(0.0, home_return_timeout_s_));
    if (!home_should_be_active)
    {
      home_integral_lin_.setZero();
      home_integral_ang_.setZero();
      home_blend_start_time_ = ros::Time(0);
    }
    else if (home_blend_start_time_.isZero())
    {
      home_blend_start_time_ = now;
    }

    if (!home_should_be_active)
    {
      setHomeReturnStatus("inactive");
    }
    else
    {
      Eigen::Vector3d slave_pos = Eigen::Vector3d::Zero();
      Eigen::Quaterniond slave_ori = Eigen::Quaterniond::Identity();
      double feedback_age_s = std::numeric_limits<double>::infinity();
      if (getFreshSlaveActualPoseForHome(now, slave_pos, slave_ori, feedback_age_s))
      {
        const double pos_error_norm = (slave_pos - home_target_pos_).norm();
        const double ori_error_norm =
            teleoperation::orientationErrorAxisAngle(home_target_ori_, slave_ori).norm();
        if (pos_error_norm <= std::max(0.0, home_return_arrival_position_tolerance_m_) &&
            ori_error_norm <= std::max(0.0, home_return_arrival_orientation_tolerance_rad_))
        {
          setHomeReturnStatus("home_reached");
        }
        else
        {
          setHomeReturnStatus("active");
        }
      }
      else
      {
        ROS_WARN_THROTTLE_NAMED(1.0, "teleop_master_haptic_controller",
                                "[%s] Home return active but slave feedback is unavailable/stale (timeout=%.3fs age=%.3fs).",
                                node_name_.c_str(),
                                home_return_feedback_timeout_s_,
                                feedback_age_s);
        setHomeReturnStatus("active");
      }
    }

    double home_alpha = 0.0;
    if (home_should_be_active && has_master_tcp)
    {
      if (home_return_blend_in_time_s_ <= 0.0)
      {
        home_alpha = 1.0;
      }
      else
      {
        home_alpha = std::clamp((now - home_blend_start_time_).toSec() / home_return_blend_in_time_s_, 0.0, 1.0);
      }
    }

    if (home_alpha > 0.0 && has_master_tcp)
    {
      const Eigen::Vector3d home_pos_error = p_master - home_target_pos_;
      home_integral_lin_ += home_pos_error * dt_used;
      home_integral_lin_ = teleoperation::clampNorm3(home_integral_lin_, std::max(0.0, home_return_i_lin_clamp_));

      F_home = home_return_k_lin_ * home_pos_error +
               home_return_i_lin_ * home_integral_lin_ +
               home_return_d_lin_ * v_lin_cmd_;
      F_home = home_alpha * teleoperation::clampNorm3(F_home, std::max(0.0, home_return_max_force_));

      if (use_torques_)
      {
        const Eigen::Vector3d home_ori_error = teleoperation::orientationErrorAxisAngle(home_target_ori_, q_master);
        home_integral_ang_ += home_ori_error * dt_used;
        home_integral_ang_ = teleoperation::clampNorm3(home_integral_ang_, std::max(0.0, home_return_i_ang_clamp_));

        Tau_home = home_return_k_ang_ * home_ori_error +
                   home_return_i_ang_ * home_integral_ang_ +
                   home_return_d_ang_ * v_ang_cmd_;
        Tau_home = home_alpha * teleoperation::clampNorm3(Tau_home, std::max(0.0, home_return_max_torque_));
      }
      else
      {
        home_integral_ang_.setZero();
      }
    }

    // Per-axis admittance: M dv + D v = (F_hand - F_feedback)
    const Eigen::Vector3d M_lin_base =
        sanitizePositiveVec((mass_linear_xyz_.allFinite() ? mass_linear_xyz_ : expandScalarTo3(mass_linear_)), 1e-6);
    const Eigen::Vector3d D_lin_base =
        sanitizeNonNegativeVec((damping_linear_xyz_.allFinite() ? damping_linear_xyz_ : expandScalarTo3(damping_linear_)));
    const Eigen::Vector3d M_ang_base =
        sanitizePositiveVec((mass_angular_xyz_.allFinite() ? mass_angular_xyz_ : expandScalarTo3(mass_angular_)), 1e-6);
    const Eigen::Vector3d D_ang_base =
        sanitizeNonNegativeVec((damping_angular_xyz_.allFinite() ? damping_angular_xyz_ : expandScalarTo3(damping_angular_)));

    Eigen::Vector3d M_lin_extra = Eigen::Vector3d::Zero();
    Eigen::Vector3d D_lin_extra = Eigen::Vector3d::Zero();
    Eigen::Vector3d M_ang_extra = Eigen::Vector3d::Zero();
    Eigen::Vector3d D_ang_extra = Eigen::Vector3d::Zero();

    Eigen::Vector3d M_lin = M_lin_base;
    Eigen::Vector3d D_lin = D_lin_base;
    Eigen::Vector3d M_ang = M_ang_base;
    Eigen::Vector3d D_ang = D_ang_base;

    if (dyn_mass_enabled_ && has_dyn_m_extra_filt_)
    {
      M_lin_extra = dyn_m_extra_lin_filt_;
      M_ang_extra = dyn_m_extra_ang_filt_;
      M_lin = sanitizePositiveVec(M_lin_base + M_lin_extra, 1e-6);
      M_ang = sanitizePositiveVec(M_ang_base + M_ang_extra, 1e-6);
    }

    if (dyn_damping_enabled_ && has_dyn_d_extra_filt_)
    {
      D_lin_extra = dyn_d_extra_lin_filt_;
      D_ang_extra = dyn_d_extra_ang_filt_;
      D_lin += D_lin_extra;
      D_ang += D_ang_extra;
    }

    Wrench3 slave_reflection_wrench;
    slave_reflection_wrench.f = (force_reflection_gate * slave_filt_.f).eval();
    slave_reflection_wrench.tau = (force_reflection_gate * slave_filt_.tau).eval();
    if (force_reflection_bias_enabled_ && force_reflection_bias_active && has_force_reflection_bias_)
    {
      Wrench3 slave_delta;
      slave_delta.f = slave_filt_.f - force_reflection_bias_.f;
      slave_delta.tau = slave_filt_.tau - force_reflection_bias_.tau;

      force_reflection_delta_filt_ = teleoperation::filterClampDeadbandWrenchNorm(
          force_reflection_delta_filt_, slave_delta, false, alpha_feedback,
          force_deadband_enter_, force_deadband_exit_,
          torque_deadband_enter_, torque_deadband_exit_,
          (max_force_feedback_ > 0.0 ? max_force_feedback_ : max_force_),
          (max_torque_feedback_ > 0.0 ? max_torque_feedback_ : max_torque_),
          use_torques_, cross_deadband_scale_, force_reflection_delta_db_state_);

      if (!use_forces_)
      {
        force_reflection_delta_filt_.f.setZero();
        force_reflection_delta_db_state_.f_active = false;
      }

      const double delta_gain = forceReflectionBiasDeltaGain(now);
      slave_reflection_wrench.f =
          (delta_gain * force_reflection_delta_filt_.f +
           force_reflection_gate * force_reflection_bias_.f).eval();
      slave_reflection_wrench.tau =
          (delta_gain * force_reflection_delta_filt_.tau +
           force_reflection_gate * force_reflection_bias_.tau).eval();
    }

    // Force reflection: slave FT typically measures the wrench applied *on the slave tool* by the environment.
    // To obtain an opposing reflected contribution at the master, we invert the slave wrench sign here.
    // The spring force K_s*(p_m - p_s) already points in the direction master→slave, so adding it
    // to F_feedback correctly opposes the master's motion when it is ahead.
    Eigen::Vector3d F_reflection = Eigen::Vector3d::Zero();
    Eigen::Vector3d Tau_reflection = Eigen::Vector3d::Zero();
    if (use_forces_)
    {
      F_reflection = (-kf_force_ * slave_reflection_wrench.f).eval();
    }
    if (use_torques_)
    {
      Tau_reflection = (-kf_torque_ * slave_reflection_wrench.tau).eval();
    }
    const teleoperation::PassivityLayerResult passivity_result =
        passivity_layer_.step(F_reflection, Tau_reflection, v_lin_cmd_, v_ang_cmd_, D_lin, D_ang, dt_used);
    const Eigen::Vector3d F_feedback = F_spring_lin + F_home + passivity_result.force_used + coupling_filt_.f;
    const Eigen::Vector3d Tau_feedback = Tau_spring + Tau_home + passivity_result.torque_used + coupling_filt_.tau;

    // Limits (accel/jerk). Scalars are expanded unless *_xyz override exists.
    const Eigen::Vector3d max_a_lin = sanitizeNonNegativeVec((max_linear_accel_xyz_.allFinite() ? max_linear_accel_xyz_ : expandScalarTo3(max_linear_accel_)));
    const Eigen::Vector3d max_j_lin = sanitizeNonNegativeVec((max_linear_jerk_xyz_.allFinite() ? max_linear_jerk_xyz_ : expandScalarTo3(max_linear_jerk_)));
    const Eigen::Vector3d max_a_ang = sanitizeNonNegativeVec((max_angular_accel_xyz_.allFinite() ? max_angular_accel_xyz_ : expandScalarTo3(max_angular_accel_)));
    const Eigen::Vector3d max_j_ang = sanitizeNonNegativeVec((max_angular_jerk_xyz_.allFinite() ? max_angular_jerk_xyz_ : expandScalarTo3(max_angular_jerk_)));

    Eigen::Vector3d v_lin_pre = v_lin_cmd_;
    Eigen::Vector3d v_ang_pre = v_ang_cmd_;

    Eigen::Vector3d a_lin_des_normed = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_ang_des_normed = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_lin_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_ang_cmd = Eigen::Vector3d::Zero();
    bool lin_saturated = false;
    bool ang_saturated = false;

    for (int k = 0; k < n_substeps; ++k)
    {
      const Eigen::Vector3d rhs_lin = (F_hand - F_feedback) - D_lin.cwiseProduct(v_lin_cmd_);
      a_lin_des_normed = rhs_lin.cwiseQuotient(M_lin);

      // 3.1 + 3.2: jerk-limited acceleration + anti-windup against speed saturation.
      const Eigen::Vector3d max_a_lin_eff = (max_a_lin.maxCoeff() > 0.0 ? max_a_lin : expandScalarTo3(1e9));
      const Eigen::Vector3d max_j_lin_eff = (max_j_lin.maxCoeff() > 0.0 ? max_j_lin : expandScalarTo3(1e9));
      a_lin_cmd = a_lin_limiter_.step(a_lin_des_normed, dt_step, max_a_lin_eff, max_j_lin_eff);

      if (max_linear_speed_ > 0.0 && v_lin_cmd_.norm() >= (max_linear_speed_ - std::max(0.0, speed_saturation_eps_)))
      {
        const double vn = v_lin_cmd_.norm();
        if (vn > teleoperation::kMathEps)
        {
          const Eigen::Vector3d u = v_lin_cmd_ / vn;
          const double a_rad = u.dot(a_lin_cmd);
          if (a_rad > 0.0)
          {
            a_lin_cmd = a_lin_cmd - a_rad * u;
          }
        }
      }

      const Eigen::Vector3d v_lin_next = v_lin_cmd_ + a_lin_cmd * dt_step;
      const Eigen::Vector3d v_lin_sat = teleoperation::clampNorm3(v_lin_next, max_linear_speed_);
      lin_saturated = lin_saturated || ((v_lin_next - v_lin_sat).norm() > 1e-12);
      v_lin_cmd_ = v_lin_sat;

      if (use_torques_)
      {
        const Eigen::Vector3d rhs_ang = (Tau_hand - Tau_feedback) - D_ang.cwiseProduct(v_ang_cmd_);
        a_ang_des_normed = rhs_ang.cwiseQuotient(M_ang);

        const Eigen::Vector3d max_a_ang_eff = (max_a_ang.maxCoeff() > 0.0 ? max_a_ang : expandScalarTo3(1e9));
        const Eigen::Vector3d max_j_ang_eff = (max_j_ang.maxCoeff() > 0.0 ? max_j_ang : expandScalarTo3(1e9));
        a_ang_cmd = a_ang_limiter_.step(a_ang_des_normed, dt_step, max_a_ang_eff, max_j_ang_eff);

        if (max_angular_speed_ > 0.0 && v_ang_cmd_.norm() >= (max_angular_speed_ - std::max(0.0, speed_saturation_eps_)))
        {
          const double wn = v_ang_cmd_.norm();
          if (wn > teleoperation::kMathEps)
          {
            const Eigen::Vector3d u = v_ang_cmd_ / wn;
            const double a_rad = u.dot(a_ang_cmd);
            if (a_rad > 0.0)
            {
              a_ang_cmd = a_ang_cmd - a_rad * u;
            }
          }
        }

        const Eigen::Vector3d v_ang_next = v_ang_cmd_ + a_ang_cmd * dt_step;
        const Eigen::Vector3d v_ang_sat = teleoperation::clampNorm3(v_ang_next, max_angular_speed_);
        ang_saturated = ang_saturated || ((v_ang_next - v_ang_sat).norm() > 1e-12);
        v_ang_cmd_ = v_ang_sat;
      }
      else
      {
        v_ang_cmd_.setZero();
        a_ang_limiter_.reset();
      }

      if (!use_forces_)
      {
        v_lin_cmd_.setZero();
        a_lin_limiter_.reset();
      }
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = v_lin_cmd_.x();
    cmd.linear.y = v_lin_cmd_.y();
    cmd.linear.z = v_lin_cmd_.z();
    cmd.angular.x = v_ang_cmd_.x();
    cmd.angular.y = v_ang_cmd_.y();
    cmd.angular.z = v_ang_cmd_.z();
    pub_cmd_.publish(cmd);

    if (pub_cmd_stamped_)
    {
      geometry_msgs::TwistStamped stamped;
      stamped.header.stamp = now;
      stamped.header.frame_id = wrench_target_frame_;
      stamped.twist = cmd;
      pub_cmd_stamped_.publish(stamped);
    }

    // 3.8 diagnostics
    if (publish_diagnostics_)
    {
      // Throttle diagnostics to diagnostics_rate_
      const double diag_period = (diagnostics_rate_ > 0.0) ? (1.0 / diagnostics_rate_) : 0.0;
      if (diag_period <= 0.0 || (now - last_diag_pub_).toSec() >= diag_period)
      {
        last_diag_pub_ = now;

        dt_min_seen_ = (dt_count_ == 0) ? dt_raw : std::min(dt_min_seen_, dt_raw);
        dt_max_seen_ = (dt_count_ == 0) ? dt_raw : std::max(dt_max_seen_, dt_raw);
        dt_count_ += 1;
        dt_mean_ += (dt_raw - dt_mean_) / static_cast<double>(dt_count_);

        std_msgs::Float64MultiArray dt_msg;
        dt_msg.data = {dt_raw, dt_used, dt_step, static_cast<double>(n_substeps), dt_min_seen_, dt_max_seen_, dt_mean_};
        pub_debug_dt_.publish(dt_msg);

        std_msgs::Float64MultiArray st_msg;
        st_msg.data = {
            F_hand.norm(),
            F_feedback.norm(),
            a_lin_des_normed.norm(),
            a_lin_cmd.norm(),
            v_lin_pre.norm(),
            v_lin_cmd_.norm(),
            lin_saturated ? 1.0 : 0.0,
            Tau_hand.norm(),
            Tau_feedback.norm(),
            a_ang_des_normed.norm(),
            a_ang_cmd.norm(),
            v_ang_pre.norm(),
            v_ang_cmd_.norm(),
            ang_saturated ? 1.0 : 0.0};
        pub_debug_stats_.publish(st_msg);

        if (pub_debug_admittance_dynamics_)
        {
          std_msgs::Float64MultiArray admittance_dyn_msg;
          admittance_dyn_msg.data = {
              M_lin_base.x(), M_lin_base.y(), M_lin_base.z(),
              M_lin_extra.x(), M_lin_extra.y(), M_lin_extra.z(),
              M_lin.x(), M_lin.y(), M_lin.z(),
              D_lin_base.x(), D_lin_base.y(), D_lin_base.z(),
              D_lin_extra.x(), D_lin_extra.y(), D_lin_extra.z(),
              D_lin.x(), D_lin.y(), D_lin.z(),
              M_ang_base.x(), M_ang_base.y(), M_ang_base.z(),
              M_ang_extra.x(), M_ang_extra.y(), M_ang_extra.z(),
              M_ang.x(), M_ang.y(), M_ang.z(),
              D_ang_base.x(), D_ang_base.y(), D_ang_base.z(),
              D_ang_extra.x(), D_ang_extra.y(), D_ang_extra.z(),
              D_ang.x(), D_ang.y(), D_ang.z()};
          pub_debug_admittance_dynamics_.publish(admittance_dyn_msg);
        }

        if (pub_debug_passivity_)
        {
          std_msgs::Float64MultiArray passivity_msg;
          passivity_msg.data = {
              passivity_result.energy_before,
              passivity_result.energy_after,
              passivity_result.gamma_raw,
              passivity_result.gamma_applied,
              passivity_result.power_out_requested,
              passivity_result.power_out_applied,
              passivity_result.power_diss,
              F_reflection.norm(),
              passivity_result.force_used.norm(),
              Tau_reflection.norm(),
              passivity_result.torque_used.norm()};
          pub_debug_passivity_.publish(passivity_msg);
        }

        geometry_msgs::TwistStamped vpre, vpost;
        vpre.header.stamp = now;
        vpre.header.frame_id = wrench_target_frame_;
        vpre.twist.linear.x = v_lin_pre.x();
        vpre.twist.linear.y = v_lin_pre.y();
        vpre.twist.linear.z = v_lin_pre.z();
        vpre.twist.angular.x = v_ang_pre.x();
        vpre.twist.angular.y = v_ang_pre.y();
        vpre.twist.angular.z = v_ang_pre.z();
        pub_debug_v_pre_.publish(vpre);

        vpost.header = vpre.header;
        vpost.twist = cmd;
        pub_debug_v_post_.publish(vpost);
      }
    }
  }

private:
  std::string describeStaleSources(bool master_stale, bool slave_stale, bool coupling_stale) const
  {
    std::string sources;
    auto appendSource = [&sources](const std::string& label, const std::string& topic)
    {
      if (!sources.empty())
      {
        sources += ", ";
      }
      sources += label + "=" + (topic.empty() ? std::string("<disabled>") : topic);
    };

    if (master_stale)
    {
      appendSource("master", resolved_master_wrench_topic_);
    }
    if (slave_stale)
    {
      appendSource("slave", resolved_slave_wrench_topic_);
    }
    if (coupling_stale)
    {
      appendSource("coupling", resolved_coupling_wrench_topic_);
    }
    if (sources.empty())
    {
      sources = "unknown source";
    }
    return sources;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_master_wrench_;
  ros::Subscriber sub_slave_wrench_;
  ros::Subscriber sub_coupling_wrench_;
  ros::Subscriber sub_slave_actual_pose_;
  ros::Subscriber sub_force_reflection_gate_;
  ros::Subscriber sub_force_reflection_bias_active_;
  ros::Subscriber sub_home_return_disable_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_cmd_stamped_;
  teleoperation::WrenchDebugPublisher debug_master_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_slave_filt_pub_;
  teleoperation::WrenchDebugPublisher debug_coupling_filt_pub_;
  ros::Publisher pub_debug_stats_;
  ros::Publisher pub_debug_admittance_dynamics_;
  ros::Publisher pub_debug_dt_;
  ros::Publisher pub_debug_v_pre_;
  ros::Publisher pub_debug_v_post_;
  ros::Publisher pub_debug_passivity_;
  ros::Publisher pub_home_return_status_;
  ros::Timer timer_;
  ros::Timer home_return_status_timer_;

  // Slave target publishers
  ros::Publisher pub_slave_pose_;
  ros::Publisher pub_slave_twist_;
  ros::Timer slave_timer_;

  // Params
  std::string node_name_;
  std::string master_wrench_topic_;
  std::string slave_wrench_topic_;
  std::string coupling_wrench_topic_;
  std::string resolved_master_wrench_topic_;
  std::string resolved_slave_wrench_topic_;
  std::string resolved_coupling_wrench_topic_;
  std::string wrench_source_frame_override_;
  bool use_latest_tf_for_wrench_{false};
  std::string wrench_target_frame_;
  double tf_timeout_s_{0.02};

  std::string command_topic_;
  double control_rate_{250.0};

  double mass_linear_{4.0};
  double damping_linear_{40.0};
  double mass_angular_{1.0};
  double damping_angular_{5.0};
  Eigen::Vector3d mass_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d damping_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d mass_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d damping_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};

  // Dynamic damping scheduled by slave force
  bool dyn_damping_enabled_{false};
  bool dyn_damping_use_slave_wrench_{true};
  std::string dyn_damping_force_metric_{"norm"};  // currently only "norm"
  double dyn_damping_force_start_{5.0};
  double dyn_damping_force_stop_{25.0};
  double dyn_damping_shape_exp_{1.0};
  bool dyn_damping_angular_schedule_from_force_{false};
  bool dyn_damping_use_torque_schedule_{false};
  std::string dyn_damping_torque_metric_{"norm"};  // currently only "norm"
  double dyn_damping_torque_start_{5.0};
  double dyn_damping_torque_stop_{25.0};
  double dyn_extra_damping_linear_{0.0};
  double dyn_extra_damping_angular_{0.0};
  double dyn_d_extra_lowpass_cutoff_hz_{10.0};
  Eigen::Vector3d dyn_extra_damping_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d dyn_extra_damping_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  bool has_dyn_d_extra_filt_{false};
  Eigen::Vector3d dyn_d_extra_lin_filt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dyn_d_extra_ang_filt_{Eigen::Vector3d::Zero()};

  // Dynamic mass scheduled by slave force (independent from damping)
  bool dyn_mass_enabled_{false};
  bool dyn_mass_use_slave_wrench_{true};
  std::string dyn_mass_force_metric_{"norm"};  // currently only "norm"
  double dyn_mass_force_start_{5.0};
  double dyn_mass_force_stop_{25.0};
  double dyn_mass_shape_exp_{1.0};
  bool dyn_mass_angular_schedule_from_force_{false};
  bool dyn_mass_use_torque_schedule_{false};
  std::string dyn_mass_torque_metric_{"norm"};  // currently only "norm"
  double dyn_mass_torque_start_{5.0};
  double dyn_mass_torque_stop_{25.0};
  double dyn_extra_mass_linear_{0.0};
  double dyn_extra_mass_angular_{0.0};
  double dyn_m_extra_lowpass_cutoff_hz_{10.0};
  Eigen::Vector3d dyn_extra_mass_linear_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d dyn_extra_mass_angular_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  bool has_dyn_m_extra_filt_{false};
  Eigen::Vector3d dyn_m_extra_lin_filt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dyn_m_extra_ang_filt_{Eigen::Vector3d::Zero()};

  double kf_force_{0.3};
  double kf_torque_{0.0};
  std::string force_reflection_gate_topic_;
  std::string home_return_disable_topic_;
  bool force_reflection_bias_enabled_{false};
  std::string force_reflection_bias_active_topic_;
  double force_reflection_bias_capture_delay_s_{1.0};
  double force_reflection_bias_capture_window_s_{0.25};
  double force_reflection_bias_delta_fade_s_{0.2};
  teleoperation::PassivityLayerConfig passivity_config_;
  bool passivity_publish_debug_{true};
  bool use_forces_{true};
  bool use_torques_{false};

  // Virtual spring coupling
  double spring_k_lin_{0.0};
  double spring_b_lin_{0.0};
  double spring_k_ang_{0.0};
  double spring_b_ang_{0.0};
  double max_spring_force_{20.0};
  double max_spring_torque_{5.0};
  std::string slave_actual_pose_topic_;

  double wrench_filter_alpha_{0.07};
  double wrench_filter_cutoff_hz_{0.0};
  double master_wrench_filter_alpha_{-1.0};
  double master_wrench_filter_cutoff_hz_{0.0};
  double feedback_wrench_filter_alpha_{-1.0};
  double feedback_wrench_filter_cutoff_hz_{0.0};

  double force_deadband_enter_{1.0};
  double force_deadband_exit_{std::numeric_limits<double>::quiet_NaN()};
  double torque_deadband_enter_{0.2};
  double torque_deadband_exit_{std::numeric_limits<double>::quiet_NaN()};
  double cross_deadband_scale_{1.0};

  double max_force_{150.0};
  double max_torque_{20.0};
  double max_force_hand_{-1.0};
  double max_torque_hand_{-1.0};
  double max_force_feedback_{-1.0};
  double max_torque_feedback_{-1.0};

  double max_linear_speed_{0.25};
  double max_angular_speed_{0.4};

  double dt_min_factor_{0.25};
  double dt_max_factor_{2.0};
  bool dt_use_substepping_{true};
  int dt_max_substeps_{10};

  double max_linear_accel_{0.0};
  double max_linear_jerk_{0.0};
  double max_angular_accel_{0.0};
  double max_angular_jerk_{0.0};
  Eigen::Vector3d max_linear_accel_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_linear_jerk_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_angular_accel_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d max_angular_jerk_xyz_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double speed_saturation_eps_{1e-3};

  double wrench_timeout_s_{0.2};
  bool reset_on_stale_{true};
  bool stale_active_{false};

  // Slave target publishing
  bool publish_slave_targets_{false};
  std::string slave_target_pose_topic_{"target_pose"};
  std::string slave_ff_twist_topic_{"feedforward_twist"};
  std::string slave_base_frame_{"base_link"};
  std::string slave_tcp_frame_{"tool0"};
  std::string slave_frame_id_override_;
  double slave_publish_rate_{250.0};
  Eigen::Vector3d slave_pose_twist_rotation_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d slave_pose_twist_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond slave_pose_twist_rotation_q_{Eigen::Quaterniond::Identity()};

  bool motion_scaling_enabled_{false};
  bool motion_scaling_config_valid_{false};
  double motion_translation_scale_{1.0};
  double motion_rotation_scale_{1.0};
  Eigen::Vector3d motion_master_neutral_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond motion_master_neutral_ori_{Eigen::Quaterniond::Identity()};

  bool home_return_enabled_{false};
  bool has_home_target_{false};
  double home_return_timeout_s_{2.0};
  double home_return_force_activity_threshold_{2.0};
  double home_return_torque_activity_threshold_{0.25};
  double home_return_blend_in_time_s_{1.0};
  double home_return_k_lin_{15.0};
  double home_return_d_lin_{18.0};
  double home_return_i_lin_{1.5};
  double home_return_i_lin_clamp_{0.10};
  double home_return_max_force_{25.0};
  double home_return_k_ang_{0.8};
  double home_return_d_ang_{0.25};
  double home_return_i_ang_{0.08};
  double home_return_i_ang_clamp_{0.30};
  double home_return_max_torque_{3.0};
  std::string home_return_status_topic_;
  double home_return_status_publish_rate_{2.0};
  double home_return_feedback_timeout_s_{0.25};
  double home_return_arrival_position_tolerance_m_{0.02};
  double home_return_arrival_orientation_tolerance_rad_{0.10};
  Eigen::Vector3d home_target_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond home_target_ori_{Eigen::Quaterniond::Identity()};

  bool publish_diagnostics_{true};
  double diagnostics_rate_{50.0};

  // Inputs (raw)
  mutable std::mutex mutex_;
  bool has_master_{false};
  bool has_slave_{false};
  bool has_coupling_{false};
  Wrench3 master_wrench_raw_;
  Wrench3 slave_wrench_raw_;
  Wrench3 coupling_wrench_raw_;
  ros::Time master_stamp_{0};
  ros::Time slave_stamp_{0};
  ros::Time coupling_stamp_{0};
  double force_reflection_gate_{1.0};
  bool force_reflection_bias_active_{false};
  ros::Time force_reflection_bias_active_since_{0};
  bool home_return_externally_disabled_{false};

  // Filtered
  bool has_filtered_master_{false};
  bool has_filtered_slave_{false};
  bool has_filtered_coupling_{false};
  Wrench3 master_filt_;
  Wrench3 slave_filt_;
  Wrench3 coupling_filt_;
  bool force_reflection_bias_last_active_{false};
  bool has_force_reflection_bias_{false};
  bool force_reflection_bias_capturing_{false};
  ros::Time force_reflection_bias_capture_started_{0};
  ros::Time force_reflection_bias_captured_time_{0};
  int force_reflection_bias_sample_count_{0};
  Wrench3 force_reflection_bias_sum_;
  Wrench3 force_reflection_bias_;
  Wrench3 force_reflection_delta_filt_;
  teleoperation::WrenchDeadbandState force_reflection_delta_db_state_;

  // Controller state
  ros::Time last_time_{0};
  ros::Time last_diag_pub_{0};
  double dt_min_seen_{0.0};
  double dt_max_seen_{0.0};
  double dt_mean_{0.0};
  uint64_t dt_count_{0};

  Eigen::Vector3d v_lin_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ang_cmd_{Eigen::Vector3d::Zero()};
  teleoperation::PassivityLayer passivity_layer_;
  ros::Time last_operator_activity_time_{0};
  ros::Time home_blend_start_time_{0};
  Eigen::Vector3d home_integral_lin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_integral_ang_{Eigen::Vector3d::Zero()};
  std::string home_return_status_{"inactive"};

  teleoperation::JerkLimiter3 a_lin_limiter_;
  teleoperation::JerkLimiter3 a_ang_limiter_;

  teleoperation::WrenchDeadbandState master_db_state_;
  teleoperation::WrenchDeadbandState slave_db_state_;
  teleoperation::WrenchDeadbandState coupling_db_state_;

  // Virtual spring state
  bool has_slave_actual_pose_{false};
  Eigen::Vector3d slave_actual_raw_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond slave_actual_raw_ori_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d slave_actual_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond slave_actual_ori_{Eigen::Quaterniond::Identity()};
  ros::Time slave_actual_pose_stamp_{0};
  Eigen::Vector3d p_slave_prev_{Eigen::Vector3d::Zero()};
  bool has_p_slave_prev_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_master_haptic_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopMasterHapticController node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_master_haptic_controller failed: %s", ex.what());
    return 1;
  }
  return 0;
}
