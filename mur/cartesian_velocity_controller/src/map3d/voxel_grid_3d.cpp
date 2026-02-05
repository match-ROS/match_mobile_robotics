#include "cartesian_velocity_controller/map3d/voxel_grid_3d.hpp"

#include <algorithm>
#include <cmath>

namespace cartesian_velocity_controller::map3d
{

static std::size_t safeSizeFromMeters(double size_m, double res)
{
  if (res <= 0.0)
    return 0;
  return static_cast<std::size_t>(std::max(1.0, std::floor(size_m / res)));
}

VoxelGrid3D::VoxelGrid3D(const Map3DConfig& cfg)
{
  reset(cfg);
}

void VoxelGrid3D::reset(const Map3DConfig& cfg)
{
  cfg_ = cfg;

  origin_center_ = cfg_.origin_offset;

  nx_ = safeSizeFromMeters(cfg_.size_x, cfg_.resolution);
  ny_ = safeSizeFromMeters(cfg_.size_y, cfg_.resolution);
  nz_ = safeSizeFromMeters(cfg_.size_z, cfg_.resolution);
  total_ = nx_ * ny_ * nz_;

  occupancy_.assign(total_, kFree);
  distance_.assign(total_, std::numeric_limits<float>::infinity());

  meta_ = MapMetadata{};
}

void VoxelGrid3D::clear()
{
  std::fill(occupancy_.begin(), occupancy_.end(), kFree);
  std::fill(distance_.begin(), distance_.end(), std::numeric_limits<float>::infinity());
}

bool VoxelGrid3D::isInsideBounds(const Eigen::Vector3d& p_map) const
{
  const double x = p_map.x();
  const double y = p_map.y();
  const double z = p_map.z();

  return (x >= minX() && x <= (minX() + cfg_.size_x) && y >= minY() && y <= (minY() + cfg_.size_y) &&
          z >= minZ() && z <= (minZ() + cfg_.size_z));
}

bool VoxelGrid3D::worldToVoxel(const Eigen::Vector3d& p_map, int& ix, int& iy, int& iz) const
{
  const double lx = (p_map.x() - minX()) / cfg_.resolution;
  const double ly = (p_map.y() - minY()) / cfg_.resolution;
  const double lz = (p_map.z() - minZ()) / cfg_.resolution;

  ix = static_cast<int>(std::floor(lx));
  iy = static_cast<int>(std::floor(ly));
  iz = static_cast<int>(std::floor(lz));

  return (ix >= 0 && iy >= 0 && iz >= 0 && ix < static_cast<int>(nx_) && iy < static_cast<int>(ny_) &&
          iz < static_cast<int>(nz_));
}

Eigen::Vector3d VoxelGrid3D::voxelToWorld(int ix, int iy, int iz) const
{
  // Center of voxel
  return Eigen::Vector3d(minX() + (static_cast<double>(ix) + 0.5) * cfg_.resolution,
                         minY() + (static_cast<double>(iy) + 0.5) * cfg_.resolution,
                         minZ() + (static_cast<double>(iz) + 0.5) * cfg_.resolution);
}

void VoxelGrid3D::setOccupied(int ix, int iy, int iz)
{
  if (ix < 0 || iy < 0 || iz < 0 || ix >= static_cast<int>(nx_) || iy >= static_cast<int>(ny_) ||
      iz >= static_cast<int>(nz_))
  {
    return;
  }
  occupancy_[voxelToIndex(ix, iy, iz)] = kOccupied;
}

static inline double clamp01(double v)
{
  return std::max(0.0, std::min(1.0, v));
}

double VoxelGrid3D::getDistanceInterpolated(const Eigen::Vector3d& p_map, bool& inside_out) const
{
  inside_out = isInsideBounds(p_map);
  if (!inside_out || total_ == 0)
  {
    return 0.0;
  }

  // Continuous voxel coordinates
  const double fx = (p_map.x() - minX()) / cfg_.resolution - 0.5;
  const double fy = (p_map.y() - minY()) / cfg_.resolution - 0.5;
  const double fz = (p_map.z() - minZ()) / cfg_.resolution - 0.5;

  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const int z0 = static_cast<int>(std::floor(fz));

  const double tx = clamp01(fx - x0);
  const double ty = clamp01(fy - y0);
  const double tz = clamp01(fz - z0);

  auto sample = [&](int xi, int yi, int zi) -> double {
    const int xc = std::clamp(xi, 0, static_cast<int>(nx_) - 1);
    const int yc = std::clamp(yi, 0, static_cast<int>(ny_) - 1);
    const int zc = std::clamp(zi, 0, static_cast<int>(nz_) - 1);
    const float v = distance_[voxelToIndex(xc, yc, zc)];
    return std::isfinite(v) ? static_cast<double>(v) : 0.0;
  };

  const double c000 = sample(x0, y0, z0);
  const double c100 = sample(x0 + 1, y0, z0);
  const double c010 = sample(x0, y0 + 1, z0);
  const double c110 = sample(x0 + 1, y0 + 1, z0);
  const double c001 = sample(x0, y0, z0 + 1);
  const double c101 = sample(x0 + 1, y0, z0 + 1);
  const double c011 = sample(x0, y0 + 1, z0 + 1);
  const double c111 = sample(x0 + 1, y0 + 1, z0 + 1);

  const double c00 = c000 * (1.0 - tx) + c100 * tx;
  const double c10 = c010 * (1.0 - tx) + c110 * tx;
  const double c01 = c001 * (1.0 - tx) + c101 * tx;
  const double c11 = c011 * (1.0 - tx) + c111 * tx;

  const double c0 = c00 * (1.0 - ty) + c10 * ty;
  const double c1 = c01 * (1.0 - ty) + c11 * ty;

  const double c = c0 * (1.0 - tz) + c1 * tz;

  return c;
}

Eigen::Vector3d VoxelGrid3D::getGradientInterpolated(const Eigen::Vector3d& p_map, bool& inside_out) const
{
  inside_out = isInsideBounds(p_map);
  if (!inside_out || total_ == 0)
  {
    // Deterministic "towards center" fallback for out-of-bounds
    const Eigen::Vector3d v = (origin_center_ - p_map);
    const double n = v.norm();
    if (n > 1e-12)
      return Eigen::Vector3d(v / n);
    return Eigen::Vector3d::UnitX();
  }

  const double h = cfg_.resolution;

  bool in_a = true, in_b = true;
  const double dx = (getDistanceInterpolated(p_map + Eigen::Vector3d(h, 0, 0), in_a) -
                     getDistanceInterpolated(p_map - Eigen::Vector3d(h, 0, 0), in_b)) /
                    (2.0 * h);
  const double dy = (getDistanceInterpolated(p_map + Eigen::Vector3d(0, h, 0), in_a) -
                     getDistanceInterpolated(p_map - Eigen::Vector3d(0, h, 0), in_b)) /
                    (2.0 * h);
  const double dz = (getDistanceInterpolated(p_map + Eigen::Vector3d(0, 0, h), in_a) -
                     getDistanceInterpolated(p_map - Eigen::Vector3d(0, 0, h), in_b)) /
                    (2.0 * h);

  Eigen::Vector3d g(dx, dy, dz);
  const double n = g.norm();
  if (n < 1e-12)
  {
    // Fallback deterministic direction
    const Eigen::Vector3d v = (origin_center_ - p_map);
    const double vn = v.norm();
    if (vn > 1e-12)
      return Eigen::Vector3d(v / vn);
    return Eigen::Vector3d::UnitX();
  }
  return g / n;
}

}  // namespace cartesian_velocity_controller::map3d

