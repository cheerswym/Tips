#ifndef ONBOARD_PERCEPTION_OBSTACLE_MANAGER_H_
#define ONBOARD_PERCEPTION_OBSTACLE_MANAGER_H_

#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/vec.h"
#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/nets/mist_obstacle_net.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/obstacle_grid.h"
#include "onboard/perception/projection_util.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/map_util.h"

DECLARE_bool(enable_obstacle_noisy_filter);
DECLARE_bool(enable_obstacle_mist_filter);
DECLARE_bool(enable_mof_net_filter);
DECLARE_bool(enable_obstacle_semantic_map_filter);

namespace qcraft {
// This class manages the storage of all obstacles and points. All updates to
// obstacles must be done through it.
class ObstacleManager {
 public:
  ObstacleManager(
      ObstacleRefVector obstacles,
      const ObstacleRCCoordConverter& rc_coord_converter,
      const CameraParamsMap& camera_params,
      const std::unordered_map<LidarId, LidarParametersProto>& lidar_params,
      ThreadPool* thread_pool);

  // Disable copy c'tor and copy assignment but keep move c'tor and move
  // assignment.
  ObstacleManager(const ObstacleManager&) = delete;
  ObstacleManager& operator=(const ObstacleManager&) = delete;
  ObstacleManager(ObstacleManager&&) = default;
  ObstacleManager& operator=(ObstacleManager&&) = default;

  Obstacle* mutable_obstacle(int i) { return &obstacles_[i]; }
  Obstacle* mutable_obstacle(int row, int col) {
    ObstaclePtr* res = FindOrNull(rc_to_obstacles_, {row, col});
    if (res == nullptr) {
      return nullptr;
    }
    return mutable_obstacle(obstacles_to_index_[*res]);
  }
  const ObstaclePtrs& obstacle_ptrs() const { return obstacle_ptrs_; }

  ObstaclePtr ObstacleAt(int row, int col) const {
    const int obstacle_index =
        (row - min_row_) * obstacle_grid_width_ + col - min_col_;
    DCHECK_GE(row, min_row_);
    DCHECK_GE(col, min_col_);
    DCHECK_LT(obstacle_index, obstacle_existence_grid_.size());
    if (!obstacle_existence_grid_[obstacle_index]) return nullptr;
    return rc_to_obstacles_.at({row, col});
  }
  ObstaclePtr ObstacleAt(Vec2f coord) const {
    const auto [row, col] = CoordToRC(coord);
    return ObstacleAt(row, col);
  }

  int min_row() const { return min_row_; }
  int min_col() const { return min_col_; }
  int max_row() const { return max_row_; }
  int max_col() const { return max_col_; }

  std::pair<int, int> CoordToRC(Vec2f coord) const {
    return rc_coord_converter_.CoordToRC(coord);
  }

  Vec2d RCToCoord(Vec2i rc) const { return rc_coord_converter_.RCToCoord(rc); }

  const ObstacleRCCoordConverter& RCCoordConverter() {
    return rc_coord_converter_;
  }

  // Obstacle detector ignores offroad obstacles. This function whitelists
  // obstacles that overlap with ped/cyc detections from FEN.
  void WhitelistObstaclesWithDetections(
      const FieryEyeNetClassifier::DetectionResult& fen_result,
      const SemanticSegmentationResults& semantic_results,
      const VehiclePose& pose);

  // Set obstacle's type to kIgnored when it's in filtered clusters.
  void BlacklistObstaclesWithinFilteredClusters(
      const SegmentedClusters& ignored_reflection_clusters,
      const SegmentedClusters& ignored_mist_clusters,
      const SegmentedClusters& ignored_blooming_clusters,
      const SegmentedClusters& ignored_small_clusters);

  // Ignore noisy obstacles (obstacle formed by e.g. dusts in air).
  void FilterObstacleNoise();

  // Rain obstacle filter using mist score.
  void FilterRainObstacleV2(
      const FieryEyeNetClassifier::DetectionResult& fen_result);

  void MofNetRainFilter(
      const VehiclePose& pose,
      const FieryEyeNetClassifier::DetectionResult& fen_result,
      const std::unique_ptr<MistObstacleNet>& mof_net);

  // Ignore ignorance and reflection obstacles labeled from perception zones.
  void FilterWithObstacleSemanticMap(
      const VehiclePose& pose,
      const mapping::SemanticMapManager& semantic_map_manager,
      const CoordinateConverter& coordinate_converter);

  // Rain obstacle filter.
  // TODO(yu): Remove the function in favor of mist score.
  void ObstacleRainFilter(
      const FieryEyeNetClassifier::DetectionResult& fen_result);

  // Set cluster obstacle's type to ingore, and set the source type to
  // segmentation noise
  void BlacklistObstaclesWithinSegmentationNoiseClusters(
      const std::map<ProposerType, SegmentedClusters>& pt_clusters);

 private:
  // Mist obstacle propagation.
  void PropagateMistObstacles(const std::vector<uint8_t>& obstacle_in_bbox);

  std::vector<float> GetPointsAboveGroundWithNeighbor(const VehiclePose& pose,
                                                      const Obstacle& obstacle);

  // Return mist score [0, 1] denoting how likely of the current obstacle mist.
  float ComputeObstacleMistScore(const Obstacle& obstacle);

  // Return if the given obstacle is likely mist (or dust/smoke/etc).
  // TODO(yu): Remove the function in favor of mist score.
  bool ObstacleIsLikelyMist(const Obstacle& obstacle);

 private:
  // Copies of obstacles. Not supposed to be accessed from outside.
  std::vector<Obstacle> obstacles_;

  // Const ptrs to obstacles. To be accessed from outside.
  ObstaclePtrs obstacle_ptrs_;
  // RC Coord converter.
  ObstacleRCCoordConverter rc_coord_converter_;
  // From run params
  CameraParamsMap camera_params_;
  std::unordered_map<LidarId, LidarParametersProto> lidar_params_;

  // A map from row/col to obstacle ptrs, serving as a sparse grid.
  absl::flat_hash_map<std::pair<uint16_t, uint16_t>, ObstaclePtr,
                      absl::Hash<std::pair<uint16_t, uint16_t>>>
      rc_to_obstacles_;

  // Padding for traversing neighbors. Current value is 6.4m.
  static constexpr int kObstacleGridPadding = 32;
  // Min/max row/col of all obstacles.
  int min_row_ = std::numeric_limits<int>::max();
  int min_col_ = std::numeric_limits<int>::max();
  int max_row_ = 0;
  int max_col_ = 0;
  int obstacle_grid_width_ = 0;

  // Store the information whether each cell in the grid defined by min/max
  // row/col contains an obstacle.
  std::vector<bool> obstacle_existence_grid_;

  // A map from obstacle pointer to index.
  absl::flat_hash_map<ObstaclePtr, int> obstacles_to_index_;
  // Thread pool
  ThreadPool* thread_pool_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_MANAGER_H_
