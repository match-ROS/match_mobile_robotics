#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <Eigen/Core>

#include <cmath>
#include <mutex>
#include <string>

#include "teleoperation/core/math_utils.hpp"

namespace
{
Eigen::Vector3d pFromPose(const geometry_msgs::PoseStamped& p)
{
  return Eigen::Vector3d(p.pose.position.x, p.pose.position.y, p.pose.position.z);
}

}  // namespace

/**
 * @brief Phase-2 node: bimanual virtual coupling (L/R).
 *
 * Implements (from old doc):
 *   eps_rel = ||x_sL - x_sR|| - ||x_mL - x_mR||
 *   F_couple_L =  Kvirt * eps_rel * u_LtoR
 *   F_couple_R = -Kvirt * eps_rel * u_LtoR
 *
 * It publishes WrenchStamped with ONLY force (no torque).
 *
 * NOTE: This node assumes the input poses for master and slave are expressed in the same
 * "teleop frame" convention (relabel-only or real transform done upstream).
 */
class TeleopBimanualCoupling
{
public:
  TeleopBimanualCoupling(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
  {
    pnh_.param<std::string>("master_left_pose_topic", master_left_pose_topic_, "");
    pnh_.param<std::string>("master_right_pose_topic", master_right_pose_topic_, "");
    pnh_.param<std::string>("slave_left_pose_topic", slave_left_pose_topic_, "");
    pnh_.param<std::string>("slave_right_pose_topic", slave_right_pose_topic_, "");

    pnh_.param<std::string>("coupling_left_wrench_topic", coupling_left_wrench_topic_, "coupling_wrench_left");
    pnh_.param<std::string>("coupling_right_wrench_topic", coupling_right_wrench_topic_, "coupling_wrench_right");
    pnh_.param<std::string>("output_frame_id", output_frame_id_, "base_link");

    pnh_.param<double>("rate", rate_, rate_);
    pnh_.param<double>("pose_timeout", pose_timeout_, pose_timeout_);

    pnh_.param<double>("k_virt", k_virt_, k_virt_);
    pnh_.param<double>("deadband_m", deadband_m_, deadband_m_);
    pnh_.param<double>("max_coupling_force", max_coupling_force_, max_coupling_force_);
    pnh_.param<double>("filter_alpha", filter_alpha_, filter_alpha_);

    if (master_left_pose_topic_.empty() || master_right_pose_topic_.empty() ||
        slave_left_pose_topic_.empty() || slave_right_pose_topic_.empty())
    {
      ROS_WARN_NAMED("teleop_bimanual_coupling",
                    "Some pose topics are empty. Node will run but publish zero coupling until configured.");
    }

    if (!master_left_pose_topic_.empty())
      sub_m_l_ = nh_.subscribe(master_left_pose_topic_, 1, &TeleopBimanualCoupling::mLCb, this);
    if (!master_right_pose_topic_.empty())
      sub_m_r_ = nh_.subscribe(master_right_pose_topic_, 1, &TeleopBimanualCoupling::mRCb, this);
    if (!slave_left_pose_topic_.empty())
      sub_s_l_ = nh_.subscribe(slave_left_pose_topic_, 1, &TeleopBimanualCoupling::sLCb, this);
    if (!slave_right_pose_topic_.empty())
      sub_s_r_ = nh_.subscribe(slave_right_pose_topic_, 1, &TeleopBimanualCoupling::sRCb, this);

    pub_l_ = nh_.advertise<geometry_msgs::WrenchStamped>(coupling_left_wrench_topic_, 1);
    pub_r_ = nh_.advertise<geometry_msgs::WrenchStamped>(coupling_right_wrench_topic_, 1);

    const double period = (rate_ > 0.0) ? (1.0 / rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopBimanualCoupling::tick, this);
  }

private:
  void mLCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    m_l_ = *msg;
    t_m_l_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_m_l_ = true;
  }
  void mRCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    m_r_ = *msg;
    t_m_r_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_m_r_ = true;
  }
  void sLCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    s_l_ = *msg;
    t_s_l_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_s_l_ = true;
  }
  void sRCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    s_r_ = *msg;
    t_s_r_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_s_r_ = true;
  }

  void publishZero()
  {
    const ros::Time now = ros::Time::now();
    geometry_msgs::WrenchStamped wl;
    wl.header.stamp = now;
    wl.header.frame_id = output_frame_id_;
    geometry_msgs::WrenchStamped wr = wl;
    pub_l_.publish(wl);
    pub_r_.publish(wr);
  }

  void tick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();

    geometry_msgs::PoseStamped m_l, m_r, s_l, s_r;
    ros::Time tm_l, tm_r, ts_l, ts_r;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ok = has_m_l_ && has_m_r_ && has_s_l_ && has_s_r_;
      m_l = m_l_;
      m_r = m_r_;
      s_l = s_l_;
      s_r = s_r_;
      tm_l = t_m_l_;
      tm_r = t_m_r_;
      ts_l = t_s_l_;
      ts_r = t_s_r_;
    }

    if (!ok)
    {
      publishZero();
      return;
    }

    if ((now - tm_l).toSec() > pose_timeout_ ||
        (now - tm_r).toSec() > pose_timeout_ ||
        (now - ts_l).toSec() > pose_timeout_ ||
        (now - ts_r).toSec() > pose_timeout_)
    {
      publishZero();
      return;
    }

    const Eigen::Vector3d pm_l = pFromPose(m_l);
    const Eigen::Vector3d pm_r = pFromPose(m_r);
    const Eigen::Vector3d ps_l = pFromPose(s_l);
    const Eigen::Vector3d ps_r = pFromPose(s_r);

    const Eigen::Vector3d d_m = pm_r - pm_l;
    const Eigen::Vector3d d_s = ps_r - ps_l;
    const double dist_m = d_m.norm();
    const double dist_s = d_s.norm();

    if (!std::isfinite(dist_m) || !std::isfinite(dist_s) || dist_m < 1e-6 || dist_s < 1e-6)
    {
      publishZero();
      return;
    }

    const double eps_rel = dist_s - dist_m;  // positive -> slave stretched more than master
    double eps_eff = eps_rel;
    if (std::abs(eps_eff) < deadband_m_) eps_eff = 0.0;

    const Eigen::Vector3d u_l_to_r = d_s.normalized();  // direction on slave geometry
    Eigen::Vector3d F = (k_virt_ * eps_eff) * u_l_to_r; // N (virtual)
    F = teleoperation::clampNorm3(F, max_coupling_force_);

    if (!has_filtered_)
    {
      F_filt_ = F;
      has_filtered_ = true;
    }
    else if (filter_alpha_ > 0.0)
    {
      F_filt_ = teleoperation::ema3(F_filt_, F, filter_alpha_);
    }
    else
    {
      F_filt_ = F;
    }

    geometry_msgs::WrenchStamped wl;
    wl.header.stamp = now;
    wl.header.frame_id = output_frame_id_;
    wl.wrench.force.x = F_filt_.x();
    wl.wrench.force.y = F_filt_.y();
    wl.wrench.force.z = F_filt_.z();

    geometry_msgs::WrenchStamped wr = wl;
    wr.wrench.force.x = -F_filt_.x();
    wr.wrench.force.y = -F_filt_.y();
    wr.wrench.force.z = -F_filt_.z();

    pub_l_.publish(wl);
    pub_r_.publish(wr);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Params
  std::string master_left_pose_topic_;
  std::string master_right_pose_topic_;
  std::string slave_left_pose_topic_;
  std::string slave_right_pose_topic_;

  std::string coupling_left_wrench_topic_;
  std::string coupling_right_wrench_topic_;
  std::string output_frame_id_;

  double rate_{250.0};
  double pose_timeout_{0.2};

  double k_virt_{80.0};            // N/m virtual stiffness on distance mismatch
  double deadband_m_{0.002};       // m
  double max_coupling_force_{40.0}; // N
  double filter_alpha_{0.1};

  ros::Subscriber sub_m_l_;
  ros::Subscriber sub_m_r_;
  ros::Subscriber sub_s_l_;
  ros::Subscriber sub_s_r_;
  ros::Publisher pub_l_;
  ros::Publisher pub_r_;
  ros::Timer timer_;

  // State
  mutable std::mutex mutex_;
  geometry_msgs::PoseStamped m_l_, m_r_, s_l_, s_r_;
  ros::Time t_m_l_{0}, t_m_r_{0}, t_s_l_{0}, t_s_r_{0};
  bool has_m_l_{false}, has_m_r_{false}, has_s_l_{false}, has_s_r_{false};

  bool has_filtered_{false};
  Eigen::Vector3d F_filt_{Eigen::Vector3d::Zero()};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bimanual_coupling");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopBimanualCoupling node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_bimanual_coupling failed: %s", ex.what());
    return 1;
  }
  return 0;
}

