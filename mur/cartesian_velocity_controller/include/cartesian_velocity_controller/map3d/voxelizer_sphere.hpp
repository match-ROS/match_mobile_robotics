#pragma once

#include "cartesian_velocity_controller/map3d/map3d_types.hpp"
#include "cartesian_velocity_controller/map3d/voxel_grid_3d.hpp"

#include <vector>

namespace cartesian_velocity_controller::map3d
{

class VoxelizerSphereOnly
{
public:
  explicit VoxelizerSphereOnly(double obstacle_margin = 0.0)
    : obstacle_margin_(obstacle_margin)
  {
  }

  void setObstacleMargin(double m) { obstacle_margin_ = m; }

  void voxelize(VoxelGrid3D& grid, const std::vector<SphereObstacle>& spheres) const;

private:
  double obstacle_margin_{0.0};
};

}  // namespace cartesian_velocity_controller::map3d

