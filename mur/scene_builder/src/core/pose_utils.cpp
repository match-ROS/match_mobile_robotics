/**
 * @file pose_utils.cpp
 * @brief Implementation of pose and quaternion utility functions
 */

#include "scene_builder/core/pose_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace scene_builder
{
namespace core
{

geometry_msgs::Pose identityPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

double quaternionNorm(const geometry_msgs::Quaternion& q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

bool isValidQuaternion(const geometry_msgs::Quaternion& q)
{
  return quaternionNorm(q) > 1e-6;
}

void normalizeQuaternion(geometry_msgs::Quaternion& q)
{
  const double norm = quaternionNorm(q);

  if (norm < 1e-6)
  {
    // Quaternion is essentially zero, set to identity
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return;
  }

  // Use tf2 for robust normalization
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf_q.normalize();
  q = tf2::toMsg(tf_q);
}

geometry_msgs::Pose sanitizePose(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose sanitized = pose;
  normalizeQuaternion(sanitized.orientation);
  return sanitized;
}

geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& a,
                                    const geometry_msgs::Pose& b,
                                    double t)
{
  geometry_msgs::Pose result;

  // Linear interpolation for position
  result.position.x = a.position.x + (b.position.x - a.position.x) * t;
  result.position.y = a.position.y + (b.position.y - a.position.y) * t;
  result.position.z = a.position.z + (b.position.z - a.position.z) * t;

  // SLERP for orientation
  tf2::Quaternion qa, qb;
  tf2::fromMsg(a.orientation, qa);
  tf2::fromMsg(b.orientation, qb);
  tf2::Quaternion qinterp = qa.slerp(qb, t);
  result.orientation = tf2::toMsg(qinterp);

  return result;
}

geometry_msgs::Pose integrateTwist(const geometry_msgs::Pose& pose,
                                   const geometry_msgs::Twist& twist,
                                   double dt)
{
  geometry_msgs::Pose integrated = pose;

  // Integrate linear velocity
  integrated.position.x += twist.linear.x * dt;
  integrated.position.y += twist.linear.y * dt;
  integrated.position.z += twist.linear.z * dt;

  // Integrate angular velocity
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Vector3 angular(twist.angular.x, twist.angular.y, twist.angular.z);
  const double angle = angular.length() * dt;

  // If angle is significant, apply rotation
  if (angle > 1e-6)
  {
    tf2::Vector3 axis = angular.normalized();
    tf2::Quaternion delta(axis, angle);
    q = q * delta;
    q.normalize();
  }

  integrated.orientation = tf2::toMsg(q);
  return integrated;
}

}  // namespace core
}  // namespace scene_builder

