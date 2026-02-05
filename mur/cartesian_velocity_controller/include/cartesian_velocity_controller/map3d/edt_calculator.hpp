#pragma once

#include "cartesian_velocity_controller/map3d/voxel_grid_3d.hpp"

namespace cartesian_velocity_controller::map3d
{

class EDTCalculator
{
public:
  // Computes Euclidean Distance Transform (EDT) from occupancy into grid.distanceGrid().
  // Occupied voxels -> 0, free -> distance to nearest occupied (meters).
  static void computeEDT(VoxelGrid3D& grid);

private:
  // 1D squared distance transform (Felzenszwalb & Huttenlocher).
  // Input f contains 0 for occupied, +inf for free (or any non-negative costs).
  // Output d contains squared distances (in voxel units squared).
  static void edt1d(const float* f, int n, float* d);
};

}  // namespace cartesian_velocity_controller::map3d

