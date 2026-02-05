/**
 * @file pose_utils.hpp
 * @brief Utility functions for pose and quaternion operations
 *
 * This file provides common pose manipulation functions used throughout
 * the scene_builder package, including:
 * - Identity pose creation
 * - Quaternion normalization and sanitization
 * - Pose interpolation (SLERP for orientation)
 * - Twist integration
 */

#ifndef SCENE_BUILDER_CORE_POSE_UTILS_HPP
#define SCENE_BUILDER_CORE_POSE_UTILS_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace scene_builder
{
namespace core
{

/**
 * @brief Creates an identity pose (origin position, identity quaternion)
 * @return Pose at origin with quaternion (0, 0, 0, 1)
 */
geometry_msgs::Pose identityPose();

/**
 * @brief Normalizes a pose ensuring the quaternion is valid
 * @param pose Pose to sanitize
 * @return Pose with normalized quaternion
 *
 * If the quaternion norm is too small (< 1e-6), it is replaced with
 * the identity quaternion. Otherwise, it is normalized.
 */
geometry_msgs::Pose sanitizePose(const geometry_msgs::Pose& pose);

/**
 * @brief Linearly interpolates between two poses
 * @param a Starting pose
 * @param b Ending pose
 * @param t Interpolation parameter (0 = a, 1 = b)
 * @return Interpolated pose
 *
 * Position is linearly interpolated, orientation uses SLERP
 * (Spherical Linear Interpolation) for quaternions.
 */
geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& a,
                                    const geometry_msgs::Pose& b,
                                    double t);

/**
 * @brief Integrates a twist (velocity) into a pose over a time interval
 * @param pose Initial pose
 * @param twist Linear and angular velocity
 * @param dt Time interval (seconds)
 * @return Resulting pose after integration
 *
 * Integration method:
 * - Position: linear integration (pose.position + twist.linear * dt)
 * - Orientation: axis-angle integration with quaternion multiplication
 */
geometry_msgs::Pose integrateTwist(const geometry_msgs::Pose& pose,
                                   const geometry_msgs::Twist& twist,
                                   double dt);

/**
 * @brief Computes the norm of a quaternion
 * @param q Quaternion to compute norm of
 * @return Euclidean norm of the quaternion
 */
double quaternionNorm(const geometry_msgs::Quaternion& q);

/**
 * @brief Normalizes a quaternion in place
 * @param q Quaternion to normalize (modified in place)
 *
 * If the quaternion norm is too small (< 1e-6), it is set to identity.
 */
void normalizeQuaternion(geometry_msgs::Quaternion& q);

/**
 * @brief Checks if a quaternion is valid (has non-zero norm)
 * @param q Quaternion to check
 * @return true if the quaternion is valid (norm > 1e-6)
 */
bool isValidQuaternion(const geometry_msgs::Quaternion& q);

}  // namespace core
}  // namespace scene_builder

#endif  // SCENE_BUILDER_CORE_POSE_UTILS_HPP

