#ifndef ONBOARD_PERCEPTION_OBSTACLE_UTIL_H_
#define ONBOARD_PERCEPTION_OBSTACLE_UTIL_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/params/run_params/proto/run_params.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/perception/obstacle.h"

DECLARE_bool(relax_ground_z_threshold);

namespace qcraft::obstacle_util {

// The maximum height for near ground points. A point with z less than this
// height will be treated as a ground point. We use a larger threshold for
// further points to account for elevation uncertainty.
constexpr float kMinNearGroundHeight = 0.15f;  // m
constexpr float kMidNearGroundHeight = 0.25f;  // m
constexpr float kMaxNearGroundHeight = 0.35f;  // m
constexpr float kMinNearGroundDist = 20.0f;    // m
constexpr float kMidNearGroundDist = 50.0f;    // m
constexpr float kMaxNearGroundDist = 80.0f;    // m

// An obstacle is treated as a overhang one and hence will be ignored if the
// min_z is >= overhang_min_height, which is from an interpolation determined by
// parameters below.
constexpr float kOverhangNearDist = 20.0f;         // m
constexpr float kOverhangFarDist = 120.0f;         // m
constexpr float kOverhangMinAboveAvBuffer = 0.15;  // m
constexpr float kOverhangMaxAboveAvBuffer = 0.2;   // m

constexpr float kObstacleRadius = 0.5 * Obstacle::kDiameter;

class NeighborRange {
 public:
  NeighborRange(int num_neighbors)  // NOLINT
      : num_neighbors_(num_neighbors) {
    QCHECK(num_neighbors_ == 4 || num_neighbors_ == 8);
  }

  template <typename Func>
  NeighborRange(Func condition)  // NOLINT
      : condition_(std::move(condition)) {}

  NeighborRange() = delete;

  std::vector<std::pair<int, int>> GetNeighborOffsets(
      const Obstacle* obstacle = nullptr) const;

 private:
  int num_neighbors_ = 0;
  // A conditional function that compute neighbor radius.
  std::function<int(const Obstacle&)> condition_;
};

using ConnectedCondition = std::function<bool(
    const Obstacle& init, const Obstacle& seed, const Obstacle& neighbor)>;

class LocalObstacleGrid {
 public:
  explicit LocalObstacleGrid(const ObstaclePtrs& obstacle_ptrs);
  LocalObstacleGrid() = delete;
  // No out of bound detection.
  ObstaclePtr ObstacleAt(const int row, const int col) const {
    return local_grid_infos_[ObstacleRcToGridIndex(row, col)].obstacle;
  }
  // No out of bound detection.
  int RawIndexAt(const int row, const int col) const {
    return local_grid_infos_[ObstacleRcToGridIndex(row, col)].raw_index;
  }
  // No out of bound detection.
  std::vector<int> FindNearestInRadius(int row, int col, int radius) const;
  // If have_same_type is true, will only find connected neighbors with the same
  // type of the init obstacle.
  // neighbor_range should be 4 or 8 or a customized condition.
  // | |*| |              |*|*|*|
  // |*|o|*| 4 neighbors  |*|o|*| 8 neighbors
  // | |*| |              |*|*|*|
  std::vector<int> FindConnectedNeighbors(
      int row, int col, const NeighborRange& neighbor_range,
      const ConnectedCondition& condition = {},
      const std::vector<bool>& has_processed = {}) const;

  std::string DebugString() const;

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  struct LocalGridInfo {
    ObstaclePtr obstacle = nullptr;
    int raw_index = -1;
  };

  bool IsLocalRcInRange(const int local_row, const int local_col) const {
    return (local_row >= 0 && local_row < height_ && local_col >= 0 &&
            local_col < width_);
  }

  std::pair<int, int> ObstacleRcToLocalRc(const int row, const int col) const {
    return {row - min_vertex_.first, col - min_vertex_.second};
  }

  int LocalRcToGridIndex(const int local_row, const int local_col) const {
    return local_row * width_ + local_col;
  }

  int ObstacleRcToGridIndex(const int row, const int col) const {
    const auto [local_row, local_col] = ObstacleRcToLocalRc(row, col);
    return LocalRcToGridIndex(local_row, local_col);
  }

 private:
  int num_obstacles_ = 0;
  std::pair<int, int> min_vertex_;  // (row, col)
  std::pair<int, int> max_vertex_;  // (row, col)
  int width_ = 0;
  int height_ = 0;
  std::vector<LocalGridInfo> local_grid_infos_;
};

// Returns true if point in within obstacle range in x-y plane.
// Note that the reason adding a buffer kEpsilon is to tolerate numerical nuance
// brought by FloorToInt in CoordToRC.
inline bool IsInObstacleRangeInXyPlane(const Obstacle& obstacle,
                                       const Vec2f& point) {
  constexpr float kEpsilon = 0.01f;  // m
  return (point.x() <= obstacle.x + kObstacleRadius + kEpsilon &&
          point.x() >= obstacle.x - kObstacleRadius - kEpsilon &&
          point.y() <= obstacle.y + kObstacleRadius + kEpsilon &&
          point.y() >= obstacle.y - kObstacleRadius - kEpsilon);
}

// NOTE(dong): Do not use this function to check if a point in obstacle is
// a real obstacle point. Use IsRealObstaclePoint or IsAboveGroundObstaclePoint
// instead.
// TODO(dong): May need to replace onboard computation with a precomputed lookup
// table, if there is any efficiency problem.
inline float GetObstacleNearGroundMaxZ(const Obstacle& obstacle) {
  const float near_ground_max_z =
      obstacle.points[0].range < kMidNearGroundDist
          ? obstacle.ground_z + kMinNearGroundHeight +
                (kMidNearGroundHeight - kMinNearGroundHeight) *
                    (std::clamp(obstacle.points[0].range, kMinNearGroundDist,
                                kMidNearGroundDist) -
                     kMinNearGroundDist) *
                    (1.0f / (kMidNearGroundDist - kMinNearGroundDist))
          : obstacle.ground_z + kMidNearGroundHeight +
                (kMaxNearGroundHeight - kMidNearGroundHeight) *
                    (std::clamp(obstacle.points[0].range, kMidNearGroundDist,
                                kMaxNearGroundDist) -
                     kMidNearGroundDist) *
                    (1.0f / (kMaxNearGroundDist - kMidNearGroundDist));
  return near_ground_max_z + (FLAGS_relax_ground_z_threshold ? 0.1f : 0.0f);
}

inline float GetObstacleOverhangMinZ(const Obstacle& obstacle,
                                     const float av_height) {
  const float overhang_min_height_near = av_height + kOverhangMinAboveAvBuffer;
  const float overhang_min_height_far = av_height + kOverhangMaxAboveAvBuffer;
  return obstacle.ground_z + overhang_min_height_near +
         (overhang_min_height_far - overhang_min_height_near) *
             (std::clamp(obstacle.points[0].range, kOverhangNearDist,
                         kOverhangFarDist) -
              kOverhangNearDist) *
             (1.0 / (kOverhangFarDist - kOverhangNearDist));
}

// Returns true if point actually falls into obstacle and it's not considered as
// ground or overhanging point.
inline bool IsRealObstaclePoint(const Obstacle& obstacle,
                                const LaserPoint& point,
                                const float av_height) {
  if (!IsInObstacleRangeInXyPlane(obstacle, Vec2f(point.x, point.y))) {
    return false;
  }
  return point.z > GetObstacleNearGroundMaxZ(obstacle) &&
         point.z < GetObstacleOverhangMinZ(obstacle, av_height);
}

inline bool IsAboveGroundObstaclePoint(const Obstacle& obstacle,
                                       const LaserPoint& point) {
  if (!IsInObstacleRangeInXyPlane(obstacle, Vec2f(point.x, point.y))) {
    return false;
  }
  return point.z > GetObstacleNearGroundMaxZ(obstacle);
}

inline std::vector<Vec2d> ComputeContour(const Obstacle& obstacle) {
  return {{obstacle.x - kObstacleRadius, obstacle.y - kObstacleRadius},
          {obstacle.x + kObstacleRadius, obstacle.y - kObstacleRadius},
          {obstacle.x + kObstacleRadius, obstacle.y + kObstacleRadius},
          {obstacle.x - kObstacleRadius, obstacle.y + kObstacleRadius}};
}

inline std::vector<Vec3d> ComputeContourWithZ(const Obstacle& obstacle,
                                              double z) {
  return {{obstacle.x - kObstacleRadius, obstacle.y - kObstacleRadius, z},
          {obstacle.x + kObstacleRadius, obstacle.y - kObstacleRadius, z},
          {obstacle.x + kObstacleRadius, obstacle.y + kObstacleRadius, z},
          {obstacle.x - kObstacleRadius, obstacle.y + kObstacleRadius, z}};
}

inline float ComputeAvHeight(const RunParamsProtoV2& run_params) {
  float av_height = 0.0f;
  for (const auto& lidar_param : run_params.vehicle_params().lidar_params()) {
    av_height = std::max(
        static_cast<float>(lidar_param.installation().extrinsics().z()),
        av_height);
  }
  av_height = std::max(
      static_cast<float>(
          run_params.vehicle_params().vehicle_geometry_params().height()),
      av_height);
  return av_height;
}

std::vector<float> ComputeGaussianKernel(const int kernel_size,
                                         const float sigma = 1.f);

// Create a box region given the center and dist to front/rear/left/right edges.
inline Box2d CreateRegionBox(Vec2d center, double yaw, double front,
                             double rear, double left, double right) {
  Box2d region(center, yaw, front + rear, left + right);
  region.Shift(
      Vec2d((front - rear) * 0.5, (left - right) * 0.5).FastRotate(yaw));
  return region;
}
// Create a box region given the center and dist to front/rear/lateral edges.
inline Box2d CreateRegionBox(Vec2d center, double yaw, double front,
                             double rear, double lateral) {
  return CreateRegionBox(center, yaw, front, rear, lateral, lateral);
}

}  // namespace qcraft::obstacle_util

#endif  // ONBOARD_PERCEPTION_OBSTACLE_UTIL_H_
