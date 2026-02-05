#pragma once

#include "cartesian_velocity_controller/map3d/map3d_types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace cartesian_velocity_controller::map3d
{

class VoxelGrid3D
{
public:
  explicit VoxelGrid3D(const Map3DConfig& cfg);

  void reset(const Map3DConfig& cfg);
  void clear();

  // ============== Geometry / indexing ==============
  bool isInsideBounds(const Eigen::Vector3d& p_map) const;

  bool worldToVoxel(const Eigen::Vector3d& p_map, int& ix, int& iy, int& iz) const;
  Eigen::Vector3d voxelToWorld(int ix, int iy, int iz) const;

  inline std::size_t voxelToIndex(int ix, int iy, int iz) const
  {
    return static_cast<std::size_t>(iz) * (nx_ * ny_) + static_cast<std::size_t>(iy) * nx_ +
           static_cast<std::size_t>(ix);
  }

  // ============== Occupancy ==============
  void setOccupied(int ix, int iy, int iz);

  const std::vector<uint8_t>& occupancy() const { return occupancy_; }
  std::vector<uint8_t>& occupancy() { return occupancy_; }

  const std::vector<float>& distanceGrid() const { return distance_; }
  std::vector<float>& distanceGrid() { return distance_; }

  // ============== Query (map frame) ==============
  // Trilinear interpolation of distance grid.
  double getDistanceInterpolated(const Eigen::Vector3d& p_map, bool& inside_out) const;

  // Gradient computed via finite differences of interpolated distance.
  Eigen::Vector3d getGradientInterpolated(const Eigen::Vector3d& p_map, bool& inside_out) const;

  // ============== Config / meta ==============
  const Map3DConfig& config() const { return cfg_; }
  const MapMetadata& metadata() const { return meta_; }
  MapMetadata& metadata() { return meta_; }

  std::size_t nx() const { return nx_; }
  std::size_t ny() const { return ny_; }
  std::size_t nz() const { return nz_; }
  std::size_t totalVoxels() const { return total_; }

  double resolution() const { return cfg_.resolution; }
  Eigen::Vector3d origin() const { return origin_center_; }

  static constexpr uint8_t kFree = 0;
  static constexpr uint8_t kOccupied = 255;

private:
  Map3DConfig cfg_;

  // Grid dimensions
  std::size_t nx_{0}, ny_{0}, nz_{0}, total_{0};

  // Center of the grid in map frame (origin offset)
  Eigen::Vector3d origin_center_{Eigen::Vector3d::Zero()};

  // Storage
  std::vector<uint8_t> occupancy_;
  std::vector<float> distance_;

  // Metadata
  MapMetadata meta_;

  // Helpers
  inline double minX() const { return origin_center_.x() - 0.5 * cfg_.size_x; }
  inline double minY() const { return origin_center_.y() - 0.5 * cfg_.size_y; }
  inline double minZ() const { return origin_center_.z() - 0.5 * cfg_.size_z; }
};

}  // namespace cartesian_velocity_controller::map3d

