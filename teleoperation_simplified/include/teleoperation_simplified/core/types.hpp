#pragma once

#include <Eigen/Core>

namespace teleoperation_simplified
{

struct Wrench3
{
  Eigen::Vector3d f{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tau{Eigen::Vector3d::Zero()};
};

}  // namespace teleoperation_simplified
