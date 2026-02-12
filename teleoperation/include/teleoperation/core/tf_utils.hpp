#pragma once

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>

#include <string>

namespace teleoperation
{

inline bool rotateVectorToFrame(const tf2_ros::Buffer& tf_buffer,
                                const std::string& target_frame,
                                const std::string& source_frame,
                                const ros::Time& stamp,
                                double tf_timeout_s,
                                const Eigen::Vector3d& v_in,
                                Eigen::Vector3d& v_out,
                                const char* node_name)
{
  if (source_frame.empty() || source_frame == target_frame)
  {
    v_out = v_in;
    return true;
  }

  try
  {
    const geometry_msgs::TransformStamped tf =
        tf_buffer.lookupTransform(target_frame, source_frame,
                                  stamp.isZero() ? ros::Time(0) : stamp,
                                  ros::Duration(tf_timeout_s));
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    tf2::Matrix3x3 R(q);
    const tf2::Vector3 vin(v_in.x(), v_in.y(), v_in.z());
    const tf2::Vector3 vout = R * vin;
    v_out = Eigen::Vector3d(vout.x(), vout.y(), vout.z());
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE_NAMED(1.0, node_name,
                            "TF rotate failed (%s -> %s): %s",
                            source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

}  // namespace teleoperation
