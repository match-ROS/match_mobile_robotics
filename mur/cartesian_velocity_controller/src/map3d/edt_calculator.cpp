#include "cartesian_velocity_controller/map3d/edt_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cartesian_velocity_controller::map3d
{

// Based on Felzenszwalb & Huttenlocher (2012): Distance transforms of sampled functions.
// Computes lower envelope of parabolas in O(n).
void EDTCalculator::edt1d(const float* f, int n, float* d)
{
  // v: locations of parabolas in lower envelope
  // z: locations of boundaries between parabolas
  std::vector<int> v(static_cast<std::size_t>(n));
  std::vector<float> z(static_cast<std::size_t>(n + 1));

  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<float>::infinity();
  z[1] = std::numeric_limits<float>::infinity();

  auto sq = [](float x) { return x * x; };

  for (int q = 1; q < n; ++q)
  {
    float s = 0.0f;
    for (;;)
    {
      const int p = v[k];
      // Intersection of parabolas from p and q:
      // (f[q] + (q^2)) - (f[p] + (p^2)) / (2q - 2p)
      const float num = (f[q] + sq(static_cast<float>(q))) - (f[p] + sq(static_cast<float>(p)));
      const float den = 2.0f * static_cast<float>(q - p);
      s = num / den;
      if (s > z[static_cast<std::size_t>(k)])
        break;
      --k;
      if (k < 0)
      {
        k = 0;
        break;
      }
    }
    ++k;
    v[k] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = std::numeric_limits<float>::infinity();
  }

  k = 0;
  for (int q = 0; q < n; ++q)
  {
    while (z[static_cast<std::size_t>(k + 1)] < static_cast<float>(q))
      ++k;
    const int p = v[k];
    const float dx = static_cast<float>(q - p);
    d[q] = dx * dx + f[p];
  }
}

void EDTCalculator::computeEDT(VoxelGrid3D& grid)
{
  const std::size_t nx = grid.nx();
  const std::size_t ny = grid.ny();
  const std::size_t nz = grid.nz();
  const std::size_t total = grid.totalVoxels();
  if (total == 0)
    return;

  // Initialize f: 0 for occupied, +inf for free
  const auto& occ = grid.occupancy();
  std::vector<float> f(total);
  constexpr float kInf = 1e20f;
  for (std::size_t i = 0; i < total; ++i)
    f[i] = (occ[i] == VoxelGrid3D::kOccupied) ? 0.0f : kInf;

  // Pass X: for each (y,z) line along x
  std::vector<float> line_in(std::max<std::size_t>(nx, std::max(ny, nz)));
  std::vector<float> line_out(line_in.size());

  std::vector<float> tmp(total);

  for (std::size_t z = 0; z < nz; ++z)
  {
    for (std::size_t y = 0; y < ny; ++y)
    {
      const std::size_t base = z * nx * ny + y * nx;
      for (std::size_t x = 0; x < nx; ++x)
        line_in[x] = f[base + x];
      edt1d(line_in.data(), static_cast<int>(nx), line_out.data());
      for (std::size_t x = 0; x < nx; ++x)
        tmp[base + x] = line_out[x];
    }
  }

  // Pass Y: for each (x,z) line along y
  for (std::size_t z = 0; z < nz; ++z)
  {
    for (std::size_t x = 0; x < nx; ++x)
    {
      for (std::size_t y = 0; y < ny; ++y)
        line_in[y] = tmp[z * nx * ny + y * nx + x];
      edt1d(line_in.data(), static_cast<int>(ny), line_out.data());
      for (std::size_t y = 0; y < ny; ++y)
        tmp[z * nx * ny + y * nx + x] = line_out[y];
    }
  }

  // Pass Z: for each (x,y) line along z
  for (std::size_t y = 0; y < ny; ++y)
  {
    for (std::size_t x = 0; x < nx; ++x)
    {
      for (std::size_t z = 0; z < nz; ++z)
        line_in[z] = tmp[z * nx * ny + y * nx + x];
      edt1d(line_in.data(), static_cast<int>(nz), line_out.data());
      for (std::size_t z = 0; z < nz; ++z)
        tmp[z * nx * ny + y * nx + x] = line_out[z];
    }
  }

  // Convert squared voxel distances to meters
  auto& dist = grid.distanceGrid();
  dist.resize(total);
  const float res = static_cast<float>(grid.resolution());
  for (std::size_t i = 0; i < total; ++i)
  {
    const float v = tmp[i];
    dist[i] = (v >= kInf * 0.5f) ? std::numeric_limits<float>::infinity() : std::sqrt(v) * res;
  }
}

}  // namespace cartesian_velocity_controller::map3d

