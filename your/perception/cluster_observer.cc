#include "onboard/perception/cluster_observer.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"

namespace qcraft {

namespace {

constexpr double kMinPartiallyObservedBoundaryObstacleRatio = 0.1;

constexpr float kBoundaryPointMinRange = 0.5;  // m
// A bound width at the edge of lidar's fov.
constexpr double kPOBZoneBoundWidth = 1.0;  // m
constexpr double kResolutionAzimuth = 0.2;  // angle

bool IsObstacleOnBoundary(
    const absl::flat_hash_map<LidarId, RangeImage>& range_images,
    const Obstacle& obstacle) {
  QCHECK(!obstacle.points.empty());
  const auto& point = obstacle.points[0];
  if (point.range < kBoundaryPointMinRange) return false;
  const double theta = std::asin(kPOBZoneBoundWidth / point.range);
  const int delta_col = RoundToInt(theta * (1.0 / d2r(kResolutionAzimuth)));
  if (const auto* range_image = FindOrNull(range_images, point.lidar_id)) {
    const auto [row, col] =
        range_image->ImagePosAt(point.scan_or_point_index, point.beam_index);
    if (col - range_image->min_col() < delta_col ||
        range_image->max_col() - col < delta_col) {
      return true;
    }
  }

  return false;
}

ObservationState ComputeObservationState(
    const absl::flat_hash_map<LidarId, RangeImage>& range_images,
    const SegmentedCluster& cluster) {
  // BANDAID(dong): Hack here. Only works in the pbq mode.
  if (IsDBQConext()) return OS_UNKNOWN;
  int num_obstacles_on_boundary = 0;
  for (const auto* obstacle : cluster.obstacles()) {
    if (IsObstacleOnBoundary(range_images, *obstacle)) {
      ++num_obstacles_on_boundary;
    }
  }

  if (num_obstacles_on_boundary >
      cluster.NumObstacles() * kMinPartiallyObservedBoundaryObstacleRatio) {
    return OS_PARTIALLY_OBSERVED;
  }

  return OS_UNKNOWN;
}

}  // namespace

void ClusterObserver::Observe(
    const absl::flat_hash_map<LidarId, RangeImage>& range_images,
    SegmentedClusters* clusters) {
  SCOPED_QTRACE_ARG1("ClusterObserver::Observe", "num_clusters",
                     clusters->size());
  ParallelFor(0, clusters->size(), thread_pool_, [&](int i) {
    const ObservationState state =
        ComputeObservationState(range_images, (*clusters)[i]);
    (*clusters)[i].set_observation_state(state);
  });
}

}  // namespace qcraft
