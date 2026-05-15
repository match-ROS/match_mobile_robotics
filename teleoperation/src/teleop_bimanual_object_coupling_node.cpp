#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "teleoperation/core/math_utils.hpp"

namespace
{
enum class Mode
{
  Independent,
  BlendIn,
  Coupled,
  BlendOut,
};

struct CommandPair
{
  Eigen::Isometry3d left_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d left_v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_w{Eigen::Vector3d::Zero()};
};

Eigen::Vector3d translationOf(const geometry_msgs::TransformStamped& T)
{
  return Eigen::Vector3d(T.transform.translation.x,
                         T.transform.translation.y,
                         T.transform.translation.z);
}

Eigen::Quaterniond quaternionOf(const geometry_msgs::TransformStamped& T)
{
  Eigen::Quaterniond q(T.transform.rotation.w,
                       T.transform.rotation.x,
                       T.transform.rotation.y,
                       T.transform.rotation.z);
  if (q.norm() > teleoperation::kMathEps)
  {
    q.normalize();
  }
  return q;
}

Eigen::Isometry3d transformToIsometry(const geometry_msgs::TransformStamped& T)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = translationOf(T);
  const Eigen::Quaterniond q = quaternionOf(T);
  if (q.norm() > teleoperation::kMathEps)
  {
    out.linear() = q.toRotationMatrix();
  }
  return out;
}

double smoothstep01(double t)
{
  const double x = std::clamp(t, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

}  // namespace

class TeleopBimanualObjectCoupling
{
public:
  TeleopBimanualObjectCoupling(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("common_frame", common_frame_, common_frame_);
    pnh_.param<std::string>("left_output_frame", left_output_frame_, left_output_frame_);
    pnh_.param<std::string>("right_output_frame", right_output_frame_, right_output_frame_);
    pnh_.param<std::string>("left_slave_tcp_frame", left_slave_tcp_frame_, left_slave_tcp_frame_);
    pnh_.param<std::string>("right_slave_tcp_frame", right_slave_tcp_frame_, right_slave_tcp_frame_);

    pnh_.param<std::string>("left_input_pose_topic", left_input_pose_topic_, left_input_pose_topic_);
    pnh_.param<std::string>("left_input_twist_topic", left_input_twist_topic_, left_input_twist_topic_);
    pnh_.param<std::string>("right_input_pose_topic", right_input_pose_topic_, right_input_pose_topic_);
    pnh_.param<std::string>("right_input_twist_topic", right_input_twist_topic_, right_input_twist_topic_);
    pnh_.param<std::string>("left_output_pose_topic", left_output_pose_topic_, left_output_pose_topic_);
    pnh_.param<std::string>("left_output_twist_topic", left_output_twist_topic_, left_output_twist_topic_);
    pnh_.param<std::string>("right_output_pose_topic", right_output_pose_topic_, right_output_pose_topic_);
    pnh_.param<std::string>("right_output_twist_topic", right_output_twist_topic_, right_output_twist_topic_);

    pnh_.param<double>("rate", rate_, rate_);
    pnh_.param<double>("input_timeout_s", input_timeout_s_, input_timeout_s_);
    pnh_.param<double>("tf_timeout_s", tf_timeout_s_, tf_timeout_s_);
    pnh_.param<double>("enable_blend_duration_s", enable_blend_duration_s_, enable_blend_duration_s_);
    pnh_.param<double>("disable_blend_duration_s", disable_blend_duration_s_, disable_blend_duration_s_);
    pnh_.param<double>("feedback_fade_duration_s", feedback_fade_duration_s_, feedback_fade_duration_s_);
    pnh_.param<std::string>("average_orientation_mode", average_orientation_mode_, average_orientation_mode_);
    pnh_.param<bool>("publish_debug_frames", publish_debug_frames_, publish_debug_frames_);
    pnh_.param<bool>("use_latest_tf_for_inputs", use_latest_tf_for_inputs_, use_latest_tf_for_inputs_);
    pnh_.param<bool>("auto_enable", auto_enable_, auto_enable_);
    pnh_.param<double>("auto_enable_delay_s", auto_enable_delay_s_, auto_enable_delay_s_);

    sanitizeDurations();

    sub_left_pose_ = nh_.subscribe(left_input_pose_topic_, 1, &TeleopBimanualObjectCoupling::leftPoseCb, this,
                                   ros::TransportHints().tcpNoDelay());
    sub_left_twist_ = nh_.subscribe(left_input_twist_topic_, 1, &TeleopBimanualObjectCoupling::leftTwistCb, this,
                                    ros::TransportHints().tcpNoDelay());
    sub_right_pose_ = nh_.subscribe(right_input_pose_topic_, 1, &TeleopBimanualObjectCoupling::rightPoseCb, this,
                                    ros::TransportHints().tcpNoDelay());
    sub_right_twist_ = nh_.subscribe(right_input_twist_topic_, 1, &TeleopBimanualObjectCoupling::rightTwistCb, this,
                                     ros::TransportHints().tcpNoDelay());

    pub_left_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(left_output_pose_topic_, 1);
    pub_left_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(left_output_twist_topic_, 1);
    pub_right_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(right_output_pose_topic_, 1);
    pub_right_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(right_output_twist_topic_, 1);

    pub_force_gate_ = pnh_.advertise<std_msgs::Float64>("force_reflection_scale", 1, true);
    pub_active_ = pnh_.advertise<std_msgs::Bool>("active", 1, true);
    pub_state_ = pnh_.advertise<std_msgs::String>("state", 1, true);
    if (publish_debug_frames_)
    {
      pub_debug_object_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/object_pose", 1);
      pub_debug_master_avg_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("debug/master_average_pose", 1);
    }

    srv_enable_ = pnh_.advertiseService("enable", &TeleopBimanualObjectCoupling::enableCb, this);
    srv_disable_ = pnh_.advertiseService("disable", &TeleopBimanualObjectCoupling::disableCb, this);

    const double period = (rate_ > 0.0) ? (1.0 / rate_) : 0.01;
    timer_ = nh_.createTimer(ros::Duration(period), &TeleopBimanualObjectCoupling::tick, this);

    if (auto_enable_)
    {
      const double timer_delay_s = std::max(0.001, auto_enable_delay_s_);
      auto_enable_timer_ = nh_.createTimer(ros::Duration(timer_delay_s),
                                           &TeleopBimanualObjectCoupling::autoEnableTick,
                                           this,
                                           true);
      ROS_INFO_NAMED("teleop_bimanual_object_coupling",
                     "Automatic coupling enable scheduled after %.3fs.",
                     auto_enable_delay_s_);
    }

    ROS_INFO_NAMED("teleop_bimanual_object_coupling",
                   "Bimanual object coupling ready. common='%s' outputs L='%s' R='%s' services '~enable'/'~disable'.",
                   common_frame_.c_str(), left_output_frame_.c_str(), right_output_frame_.c_str());
  }

private:
  void sanitizeDurations()
  {
    if (!std::isfinite(enable_blend_duration_s_) || enable_blend_duration_s_ < 0.0)
      enable_blend_duration_s_ = 0.0;
    if (!std::isfinite(disable_blend_duration_s_) || disable_blend_duration_s_ < 0.0)
      disable_blend_duration_s_ = 0.0;
    if (!std::isfinite(feedback_fade_duration_s_) || feedback_fade_duration_s_ < 0.0)
      feedback_fade_duration_s_ = 0.0;
    if (!std::isfinite(input_timeout_s_) || input_timeout_s_ < 0.0)
      input_timeout_s_ = 0.0;
    if (!std::isfinite(tf_timeout_s_) || tf_timeout_s_ < 0.0)
      tf_timeout_s_ = 0.0;
    if (!std::isfinite(auto_enable_delay_s_) || auto_enable_delay_s_ < 0.0)
      auto_enable_delay_s_ = 0.0;
  }

  void leftPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    left_pose_ = *msg;
    left_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_left_pose_ = true;
  }

  void leftTwistCb(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    left_twist_ = *msg;
    left_twist_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_left_twist_ = true;
  }

  void rightPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    right_pose_ = *msg;
    right_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_right_pose_ = true;
  }

  void rightTwistCb(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    right_twist_ = *msg;
    right_twist_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_right_twist_ = true;
  }

  bool isFresh(const ros::Time& stamp, const ros::Time& now) const
  {
    if (stamp.isZero())
    {
      return false;
    }
    if (input_timeout_s_ <= 0.0)
    {
      return true;
    }
    return std::fabs((now - stamp).toSec()) <= input_timeout_s_;
  }

  bool rawInputsFresh(const ros::Time& now, bool warn) const
  {
    const bool ok = has_left_pose_ && has_left_twist_ && has_right_pose_ && has_right_twist_ &&
                    isFresh(left_pose_stamp_, now) && isFresh(left_twist_stamp_, now) &&
                    isFresh(right_pose_stamp_, now) && isFresh(right_twist_stamp_, now);
    if (!ok && warn)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                              "Raw target inputs are missing or stale; holding last coupled target.");
    }
    return ok;
  }

  ros::Time inputLookupTime(const ros::Time& stamp) const
  {
    if (use_latest_tf_for_inputs_ || stamp.isZero())
    {
      return ros::Time(0);
    }
    return stamp;
  }

  static bool poseMsgToIso(const geometry_msgs::PoseStamped& msg, Eigen::Isometry3d& out)
  {
    Eigen::Quaterniond q(msg.pose.orientation.w,
                         msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z);
    if (!(q.norm() > teleoperation::kMathEps) || !std::isfinite(q.norm()))
    {
      return false;
    }
    q.normalize();
    out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z);
    out.linear() = q.toRotationMatrix();
    return true;
  }

  bool transformPoseToCommon(const geometry_msgs::PoseStamped& in, Eigen::Isometry3d& out) const
  {
    geometry_msgs::PoseStamped common_msg;
    if (in.header.frame_id.empty() || in.header.frame_id == common_frame_)
    {
      common_msg = in;
      common_msg.header.frame_id = common_frame_;
    }
    else
    {
      try
      {
        const ros::Time stamp = inputLookupTime(in.header.stamp);
        const geometry_msgs::TransformStamped T =
            tf_buffer_.lookupTransform(common_frame_, in.header.frame_id, stamp, ros::Duration(tf_timeout_s_));
        tf2::doTransform(in, common_msg, T);
        common_msg.header.frame_id = common_frame_;
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                                "TF pose transform failed (%s -> %s): %s",
                                in.header.frame_id.c_str(), common_frame_.c_str(), ex.what());
        return false;
      }
    }

    if (!poseMsgToIso(common_msg, out))
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                              "Received target pose with invalid quaternion.");
      return false;
    }
    return true;
  }

  bool rotateTwistToCommon(const geometry_msgs::TwistStamped& in,
                           Eigen::Vector3d& v_common,
                           Eigen::Vector3d& w_common) const
  {
    Eigen::Vector3d v(in.twist.linear.x, in.twist.linear.y, in.twist.linear.z);
    Eigen::Vector3d w(in.twist.angular.x, in.twist.angular.y, in.twist.angular.z);
    if (in.header.frame_id.empty() || in.header.frame_id == common_frame_)
    {
      v_common = v;
      w_common = w;
      return true;
    }

    try
    {
      const ros::Time stamp = inputLookupTime(in.header.stamp);
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(common_frame_, in.header.frame_id, stamp, ros::Duration(tf_timeout_s_));
      tf2::Quaternion q;
      tf2::fromMsg(T.transform.rotation, q);
      tf2::Matrix3x3 R(q);
      const tf2::Vector3 tv(v.x(), v.y(), v.z());
      const tf2::Vector3 tw(w.x(), w.y(), w.z());
      const tf2::Vector3 tv_out = R * tv;
      const tf2::Vector3 tw_out = R * tw;
      v_common = Eigen::Vector3d(tv_out.x(), tv_out.y(), tv_out.z());
      w_common = Eigen::Vector3d(tw_out.x(), tw_out.y(), tw_out.z());
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                              "TF twist rotation failed (%s -> %s): %s",
                              in.header.frame_id.c_str(), common_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool getRawCommandCommon(CommandPair& out, const ros::Time& now, bool warn) const
  {
    if (!rawInputsFresh(now, warn))
    {
      return false;
    }
    if (!transformPoseToCommon(left_pose_, out.left_pose) ||
        !transformPoseToCommon(right_pose_, out.right_pose) ||
        !rotateTwistToCommon(left_twist_, out.left_v, out.left_w) ||
        !rotateTwistToCommon(right_twist_, out.right_v, out.right_w))
    {
      return false;
    }
    return true;
  }

  bool lookupSlaveTcpPose(const std::string& tcp_frame, Eigen::Isometry3d& out) const
  {
    try
    {
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(common_frame_, tcp_frame, ros::Time(0), ros::Duration(tf_timeout_s_));
      const Eigen::Quaterniond q = quaternionOf(T);
      if (!(q.norm() > teleoperation::kMathEps))
      {
        return false;
      }
      out = transformToIsometry(T);
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_NAMED("teleop_bimanual_object_coupling",
                     "TF TCP lookup failed (%s -> %s): %s",
                     common_frame_.c_str(), tcp_frame.c_str(), ex.what());
      return false;
    }
  }

  Eigen::Quaterniond averageOrientation(const Eigen::Quaterniond& q_left_in,
                                        const Eigen::Quaterniond& q_right_in) const
  {
    Eigen::Quaterniond q_left = q_left_in.normalized();
    Eigen::Quaterniond q_right = q_right_in.normalized();
    if (average_orientation_mode_ == "left")
    {
      return q_left;
    }
    if (average_orientation_mode_ == "right")
    {
      return q_right;
    }

    if (q_left.dot(q_right) < 0.0)
    {
      q_right.coeffs() = -q_right.coeffs();
    }
    return q_left.slerp(0.5, q_right).normalized();
  }

  Eigen::Isometry3d averagePose(const Eigen::Isometry3d& left,
                                const Eigen::Isometry3d& right) const
  {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = 0.5 * (left.translation() + right.translation());
    const Eigen::Quaterniond q_left(left.rotation());
    const Eigen::Quaterniond q_right(right.rotation());
    out.linear() = averageOrientation(q_left, q_right).toRotationMatrix();
    return out;
  }

  Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& a,
                                    const Eigen::Isometry3d& b,
                                    double alpha) const
  {
    const double t = std::clamp(alpha, 0.0, 1.0);
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = (1.0 - t) * a.translation() + t * b.translation();

    Eigen::Quaterniond qa(a.rotation());
    Eigen::Quaterniond qb(b.rotation());
    qa.normalize();
    qb.normalize();
    if (qa.dot(qb) < 0.0)
    {
      qb.coeffs() = -qb.coeffs();
    }
    out.linear() = qa.slerp(t, qb).normalized().toRotationMatrix();
    return out;
  }

  CommandPair interpolateCommand(const CommandPair& a,
                                 const CommandPair& b,
                                 double alpha) const
  {
    const double t = std::clamp(alpha, 0.0, 1.0);
    CommandPair out;
    out.left_pose = interpolatePose(a.left_pose, b.left_pose, t);
    out.right_pose = interpolatePose(a.right_pose, b.right_pose, t);
    out.left_v = (1.0 - t) * a.left_v + t * b.left_v;
    out.left_w = (1.0 - t) * a.left_w + t * b.left_w;
    out.right_v = (1.0 - t) * a.right_v + t * b.right_v;
    out.right_w = (1.0 - t) * a.right_w + t * b.right_w;
    return out;
  }

  bool computeCoupledCommand(const CommandPair& raw, CommandPair& out)
  {
    if (!has_coupling_anchor_)
    {
      return false;
    }

    const Eigen::Isometry3d master_avg = averagePose(raw.left_pose, raw.right_pose);
    const Eigen::Vector3d dp = master_avg.translation() - master_avg0_.translation();

    Eigen::Quaterniond q_avg0(master_avg0_.rotation());
    Eigen::Quaterniond q_avg(master_avg.rotation());
    q_avg0.normalize();
    q_avg.normalize();
    if (q_avg0.dot(q_avg) < 0.0)
    {
      q_avg.coeffs() = -q_avg.coeffs();
    }
    const Eigen::Quaterniond q_delta = (q_avg0.conjugate() * q_avg).normalized();

    Eigen::Quaterniond q_object0(object0_.rotation());
    q_object0.normalize();
    const Eigen::Quaterniond q_object = (q_object0 * q_delta).normalized();

    Eigen::Isometry3d object_des = Eigen::Isometry3d::Identity();
    object_des.translation() = object0_.translation() + dp;
    object_des.linear() = q_object.toRotationMatrix();

    out.left_pose = object_des * object_to_left0_;
    out.right_pose = object_des * object_to_right0_;

    const Eigen::Vector3d v_object = 0.5 * (raw.left_v + raw.right_v);
    const Eigen::Vector3d w_object = 0.5 * (raw.left_w + raw.right_w);
    const Eigen::Vector3d r_left = out.left_pose.translation() - object_des.translation();
    const Eigen::Vector3d r_right = out.right_pose.translation() - object_des.translation();
    out.left_v = v_object + w_object.cross(r_left);
    out.right_v = v_object + w_object.cross(r_right);
    out.left_w = w_object;
    out.right_w = w_object;

    publishDebugPose(pub_debug_master_avg_pose_, master_avg, ros::Time::now(), common_frame_);
    publishDebugPose(pub_debug_object_pose_, object_des, ros::Time::now(), common_frame_);
    return true;
  }

  bool transformPoseFromCommon(const Eigen::Isometry3d& in_common,
                               const std::string& output_frame,
                               Eigen::Isometry3d& out) const
  {
    if (output_frame.empty() || output_frame == common_frame_)
    {
      out = in_common;
      return true;
    }

    try
    {
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(output_frame, common_frame_, ros::Time(0), ros::Duration(tf_timeout_s_));
      out = transformToIsometry(T) * in_common;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                              "TF output pose transform failed (%s -> %s): %s",
                              common_frame_.c_str(), output_frame.c_str(), ex.what());
      return false;
    }
  }

  bool rotateVectorFromCommon(const Eigen::Vector3d& v_common,
                              const std::string& output_frame,
                              Eigen::Vector3d& v_out) const
  {
    if (output_frame.empty() || output_frame == common_frame_)
    {
      v_out = v_common;
      return true;
    }

    try
    {
      const geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(output_frame, common_frame_, ros::Time(0), ros::Duration(tf_timeout_s_));
      tf2::Quaternion q;
      tf2::fromMsg(T.transform.rotation, q);
      tf2::Matrix3x3 R(q);
      const tf2::Vector3 vin(v_common.x(), v_common.y(), v_common.z());
      const tf2::Vector3 vout = R * vin;
      v_out = Eigen::Vector3d(vout.x(), vout.y(), vout.z());
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "teleop_bimanual_object_coupling",
                              "TF output twist rotation failed (%s -> %s): %s",
                              common_frame_.c_str(), output_frame.c_str(), ex.what());
      return false;
    }
  }

  geometry_msgs::PoseStamped makePoseMsg(const Eigen::Isometry3d& pose,
                                         const ros::Time& stamp,
                                         const std::string& frame) const
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.pose.position.x = pose.translation().x();
    msg.pose.position.y = pose.translation().y();
    msg.pose.position.z = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    q.normalize();
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    return msg;
  }

  geometry_msgs::TwistStamped makeTwistMsg(const Eigen::Vector3d& v,
                                           const Eigen::Vector3d& w,
                                           const ros::Time& stamp,
                                           const std::string& frame) const
  {
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.twist.linear.x = v.x();
    msg.twist.linear.y = v.y();
    msg.twist.linear.z = v.z();
    msg.twist.angular.x = w.x();
    msg.twist.angular.y = w.y();
    msg.twist.angular.z = w.z();
    return msg;
  }

  void publishDebugPose(const ros::Publisher& pub,
                        const Eigen::Isometry3d& pose,
                        const ros::Time& stamp,
                        const std::string& frame) const
  {
    if (!pub)
    {
      return;
    }
    pub.publish(makePoseMsg(pose, stamp, frame));
  }

  bool publishCommand(const CommandPair& cmd, const ros::Time& stamp)
  {
    Eigen::Isometry3d left_pose_out;
    Eigen::Isometry3d right_pose_out;
    Eigen::Vector3d left_v_out, left_w_out, right_v_out, right_w_out;
    const bool ok =
        transformPoseFromCommon(cmd.left_pose, left_output_frame_, left_pose_out) &&
        transformPoseFromCommon(cmd.right_pose, right_output_frame_, right_pose_out) &&
        rotateVectorFromCommon(cmd.left_v, left_output_frame_, left_v_out) &&
        rotateVectorFromCommon(cmd.left_w, left_output_frame_, left_w_out) &&
        rotateVectorFromCommon(cmd.right_v, right_output_frame_, right_v_out) &&
        rotateVectorFromCommon(cmd.right_w, right_output_frame_, right_w_out);
    if (!ok)
    {
      return false;
    }

    pub_left_pose_.publish(makePoseMsg(left_pose_out, stamp, left_output_frame_));
    pub_left_twist_.publish(makeTwistMsg(left_v_out, left_w_out, stamp, left_output_frame_));
    pub_right_pose_.publish(makePoseMsg(right_pose_out, stamp, right_output_frame_));
    pub_right_twist_.publish(makeTwistMsg(right_v_out, right_w_out, stamp, right_output_frame_));

    last_command_ = cmd;
    has_last_command_ = true;
    return true;
  }

  void publishRawDirect(const ros::Time& now)
  {
    bool any_stale = false;
    if (has_left_pose_ && has_left_twist_ &&
        isFresh(left_pose_stamp_, now) && isFresh(left_twist_stamp_, now))
    {
      pub_left_pose_.publish(left_pose_);
      pub_left_twist_.publish(left_twist_);
    }
    else
    {
      any_stale = true;
    }

    if (has_right_pose_ && has_right_twist_ &&
        isFresh(right_pose_stamp_, now) && isFresh(right_twist_stamp_, now))
    {
      pub_right_pose_.publish(right_pose_);
      pub_right_twist_.publish(right_twist_);
    }
    else
    {
      any_stale = true;
    }

    CommandPair raw;
    if (getRawCommandCommon(raw, now, false))
    {
      last_command_ = raw;
      has_last_command_ = true;
    }
    else if (any_stale)
    {
      publishHold(now);
    }
  }

  void publishHold(const ros::Time& now)
  {
    if (!has_last_command_)
    {
      return;
    }
    CommandPair hold = last_command_;
    hold.left_v.setZero();
    hold.left_w.setZero();
    hold.right_v.setZero();
    hold.right_w.setZero();
    (void)publishCommand(hold, now);
  }

  bool enableCb(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
  {
    const ros::Time now = ros::Time::now();
    CommandPair raw;
    if (!getRawCommandCommon(raw, now, true))
    {
      res.success = false;
      res.message = "Cannot enable coupling: raw target pose/twist inputs are missing, stale, or not transformable.";
      return true;
    }

    Eigen::Isometry3d actual_left;
    Eigen::Isometry3d actual_right;
    if (!lookupSlaveTcpPose(left_slave_tcp_frame_, actual_left) ||
        !lookupSlaveTcpPose(right_slave_tcp_frame_, actual_right))
    {
      res.success = false;
      res.message = "Cannot enable coupling: slave TCP TF lookup failed.";
      return true;
    }

    master_avg0_ = averagePose(raw.left_pose, raw.right_pose);
    object0_ = averagePose(actual_left, actual_right);
    object_to_left0_ = object0_.inverse() * actual_left;
    object_to_right0_ = object0_.inverse() * actual_right;
    has_coupling_anchor_ = true;

    mode_ = Mode::BlendIn;
    mode_start_time_ = now;
    last_command_ = raw;
    has_last_command_ = true;

    ROS_INFO_NAMED("teleop_bimanual_object_coupling",
                   "Coupling enabled: rigid slave offsets captured in '%s'.",
                   common_frame_.c_str());
    res.success = true;
    res.message = "Coupling enabled.";
    return true;
  }

  bool disableCb(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
  {
    const ros::Time now = ros::Time::now();
    if (mode_ == Mode::Independent)
    {
      res.success = true;
      res.message = "Coupling already disabled.";
      return true;
    }

    if (!has_last_command_)
    {
      CommandPair raw;
      if (getRawCommandCommon(raw, now, false))
      {
        last_command_ = raw;
        has_last_command_ = true;
      }
    }

    if (!has_last_command_)
    {
      mode_ = Mode::Independent;
      has_coupling_anchor_ = false;
      res.success = true;
      res.message = "Coupling disabled without blend because no last command was available.";
      return true;
    }

    blend_out_from_ = last_command_;
    mode_ = Mode::BlendOut;
    mode_start_time_ = now;

    ROS_INFO_NAMED("teleop_bimanual_object_coupling", "Coupling disabling: blending back to independent targets.");
    res.success = true;
    res.message = "Coupling disable requested.";
    return true;
  }

  void autoEnableTick(const ros::TimerEvent& /*ev*/)
  {
    if (mode_ != Mode::Independent)
    {
      ROS_INFO_NAMED("teleop_bimanual_object_coupling",
                     "Automatic coupling enable skipped: coupling is already active.");
      return;
    }

    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    (void)enableCb(req, res);
    if (res.success)
    {
      ROS_INFO_NAMED("teleop_bimanual_object_coupling",
                     "Automatic coupling enable succeeded: %s",
                     res.message.c_str());
    }
    else
    {
      ROS_WARN_NAMED("teleop_bimanual_object_coupling",
                     "Automatic coupling enable failed after %.3fs: %s",
                     auto_enable_delay_s_, res.message.c_str());
    }
  }

  double elapsedModeTime(const ros::Time& now) const
  {
    if (mode_start_time_.isZero())
    {
      return 0.0;
    }
    return std::max(0.0, (now - mode_start_time_).toSec());
  }

  double blendAlpha(double elapsed, double duration) const
  {
    if (duration <= 0.0)
    {
      return 1.0;
    }
    return smoothstep01(elapsed / duration);
  }

  double modeCompletionDuration() const
  {
    if (mode_ == Mode::BlendIn)
    {
      return std::max(enable_blend_duration_s_, feedback_fade_duration_s_);
    }
    if (mode_ == Mode::BlendOut)
    {
      return std::max(disable_blend_duration_s_, feedback_fade_duration_s_);
    }
    return 0.0;
  }

  double forceGateValue(const ros::Time& now) const
  {
    const double elapsed = elapsedModeTime(now);
    if (mode_ == Mode::Independent)
    {
      return 1.0;
    }
    if (mode_ == Mode::Coupled)
    {
      return 0.0;
    }
    if (mode_ == Mode::BlendIn)
    {
      return 1.0 - blendAlpha(elapsed, feedback_fade_duration_s_);
    }
    if (mode_ == Mode::BlendOut)
    {
      return blendAlpha(elapsed, feedback_fade_duration_s_);
    }
    return 1.0;
  }

  std::string modeName() const
  {
    switch (mode_)
    {
      case Mode::Independent:
        return "independent";
      case Mode::BlendIn:
        return "blend_in";
      case Mode::Coupled:
        return "coupled";
      case Mode::BlendOut:
        return "blend_out";
    }
    return "unknown";
  }

  void publishModeTopics(const ros::Time& now)
  {
    std_msgs::Float64 gate_msg;
    gate_msg.data = std::clamp(forceGateValue(now), 0.0, 1.0);
    pub_force_gate_.publish(gate_msg);

    std_msgs::Bool active_msg;
    active_msg.data = (mode_ != Mode::Independent);
    pub_active_.publish(active_msg);

    std_msgs::String state_msg;
    state_msg.data = modeName();
    pub_state_.publish(state_msg);
  }

  void finishTransitionIfNeeded(const ros::Time& now)
  {
    if (mode_ != Mode::BlendIn && mode_ != Mode::BlendOut)
    {
      return;
    }

    const double done_after = modeCompletionDuration();
    if (elapsedModeTime(now) < done_after)
    {
      return;
    }

    if (mode_ == Mode::BlendIn)
    {
      mode_ = Mode::Coupled;
    }
    else
    {
      mode_ = Mode::Independent;
      has_coupling_anchor_ = false;
    }
    mode_start_time_ = now;
  }

  void tick(const ros::TimerEvent& /*ev*/)
  {
    const ros::Time now = ros::Time::now();
    publishModeTopics(now);

    if (mode_ == Mode::Independent)
    {
      publishRawDirect(now);
      return;
    }

    CommandPair raw;
    const bool has_raw = getRawCommandCommon(raw, now, true);
    if (!has_raw)
    {
      publishHold(now);
      finishTransitionIfNeeded(now);
      return;
    }

    const double elapsed = elapsedModeTime(now);
    CommandPair out;

    if (mode_ == Mode::BlendOut)
    {
      const double alpha = blendAlpha(elapsed, disable_blend_duration_s_);
      out = interpolateCommand(blend_out_from_, raw, alpha);
    }
    else
    {
      CommandPair coupled;
      if (!computeCoupledCommand(raw, coupled))
      {
        publishHold(now);
        finishTransitionIfNeeded(now);
        return;
      }

      if (mode_ == Mode::BlendIn)
      {
        const double alpha = blendAlpha(elapsed, enable_blend_duration_s_);
        out = interpolateCommand(raw, coupled, alpha);
      }
      else
      {
        out = coupled;
      }
    }

    (void)publishCommand(out, now);
    finishTransitionIfNeeded(now);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_left_pose_;
  ros::Subscriber sub_left_twist_;
  ros::Subscriber sub_right_pose_;
  ros::Subscriber sub_right_twist_;
  ros::Publisher pub_left_pose_;
  ros::Publisher pub_left_twist_;
  ros::Publisher pub_right_pose_;
  ros::Publisher pub_right_twist_;
  ros::Publisher pub_force_gate_;
  ros::Publisher pub_active_;
  ros::Publisher pub_state_;
  ros::Publisher pub_debug_object_pose_;
  ros::Publisher pub_debug_master_avg_pose_;
  ros::ServiceServer srv_enable_;
  ros::ServiceServer srv_disable_;
  ros::Timer timer_;
  ros::Timer auto_enable_timer_;

  std::string common_frame_{"base_link"};
  std::string left_output_frame_{"base_link"};
  std::string right_output_frame_{"base_link"};
  std::string left_slave_tcp_frame_{"left_tool0"};
  std::string right_slave_tcp_frame_{"right_tool0"};

  std::string left_input_pose_topic_{"left/raw_target_pose"};
  std::string left_input_twist_topic_{"left/raw_feedforward_twist"};
  std::string right_input_pose_topic_{"right/raw_target_pose"};
  std::string right_input_twist_topic_{"right/raw_feedforward_twist"};
  std::string left_output_pose_topic_{"left/target_pose"};
  std::string left_output_twist_topic_{"left/feedforward_twist"};
  std::string right_output_pose_topic_{"right/target_pose"};
  std::string right_output_twist_topic_{"right/feedforward_twist"};

  double rate_{500.0};
  double input_timeout_s_{0.15};
  double tf_timeout_s_{0.02};
  double enable_blend_duration_s_{0.4};
  double disable_blend_duration_s_{1.0};
  double feedback_fade_duration_s_{1.0};
  std::string average_orientation_mode_{"slerp"};
  bool publish_debug_frames_{true};
  bool use_latest_tf_for_inputs_{true};
  bool auto_enable_{false};
  double auto_enable_delay_s_{0.0};

  geometry_msgs::PoseStamped left_pose_;
  geometry_msgs::TwistStamped left_twist_;
  geometry_msgs::PoseStamped right_pose_;
  geometry_msgs::TwistStamped right_twist_;
  ros::Time left_pose_stamp_{0};
  ros::Time left_twist_stamp_{0};
  ros::Time right_pose_stamp_{0};
  ros::Time right_twist_stamp_{0};
  bool has_left_pose_{false};
  bool has_left_twist_{false};
  bool has_right_pose_{false};
  bool has_right_twist_{false};

  Mode mode_{Mode::Independent};
  ros::Time mode_start_time_{0};

  bool has_coupling_anchor_{false};
  Eigen::Isometry3d master_avg0_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d object0_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d object_to_left0_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d object_to_right0_{Eigen::Isometry3d::Identity()};

  bool has_last_command_{false};
  CommandPair last_command_;
  CommandPair blend_out_from_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_bimanual_object_coupling");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    TeleopBimanualObjectCoupling node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("teleop_bimanual_object_coupling failed: %s", ex.what());
    return 1;
  }
  return 0;
}
