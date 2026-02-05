#include "cartesian_velocity_controller/map3d/voxelizer_sphere.hpp"

#include <algorithm>

namespace cartesian_velocity_controller::map3d
{

void VoxelizerSphereOnly::voxelize(VoxelGrid3D& grid, const std::vector<SphereObstacle>& spheres) const
{
  for (const auto& s : spheres)
  {
    const double r = std::max(0.0, s.radius + obstacle_margin_);
    const Eigen::Vector3d c = s.center;

    // Conservative voxel-space bounding box
    int ix_min, iy_min, iz_min;
    int ix_max, iy_max, iz_max;

    const Eigen::Vector3d pmin = c - Eigen::Vector3d(r, r, r);
    const Eigen::Vector3d pmax = c + Eigen::Vector3d(r, r, r);

    grid.worldToVoxel(pmin, ix_min, iy_min, iz_min);
    grid.worldToVoxel(pmax, ix_max, iy_max, iz_max);

    ix_min = std::clamp(ix_min, 0, static_cast<int>(grid.nx()) - 1);
    iy_min = std::clamp(iy_min, 0, static_cast<int>(grid.ny()) - 1);
    iz_min = std::clamp(iz_min, 0, static_cast<int>(grid.nz()) - 1);
    ix_max = std::clamp(ix_max, 0, static_cast<int>(grid.nx()) - 1);
    iy_max = std::clamp(iy_max, 0, static_cast<int>(grid.ny()) - 1);
    iz_max = std::clamp(iz_max, 0, static_cast<int>(grid.nz()) - 1);

    const double r2 = r * r;
    for (int iz = iz_min; iz <= iz_max; ++iz)
    {
      for (int iy = iy_min; iy <= iy_max; ++iy)
      {
        for (int ix = ix_min; ix <= ix_max; ++ix)
        {
          const Eigen::Vector3d vc = grid.voxelToWorld(ix, iy, iz);
          if ((vc - c).squaredNorm() <= r2)
          {
            grid.setOccupied(ix, iy, iz);
          }
        }
      }
    }
  }
}

}  // namespace cartesian_velocity_controller::map3d

