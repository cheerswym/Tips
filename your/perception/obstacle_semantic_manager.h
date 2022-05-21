#ifndef ONBOARD_PERCEPTION_OBSTACLE_SEMANTIC_MANAGER_H_
#define ONBOARD_PERCEPTION_OBSTACLE_SEMANTIC_MANAGER_H_

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/vec.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/obstacle_manager.h"
#include "opencv2/imgproc.hpp"

namespace qcraft {

// Num obstacle types managed in the OSM.
static constexpr int kNumSemanticTypes = 2;

using LogOddsArray = std::array<float, kNumSemanticTypes>;

class ObstacleSemanticGrid {
  struct RowColHash {
    size_t operator()(Vec2i rc) const {
      return std::hash<uint64_t>()(static_cast<uint64_t>(rc.x()) << 32 |
                                   static_cast<uint64_t>(rc.y()));
    }
  };

 public:
  ObstacleSemanticGrid(int width, int height)
      : width_(width), height_(height), info_grid_(width * height) {}

  class GridInfo {
    using iterator = LogOddsArray::iterator;
    using const_iterator = LogOddsArray::const_iterator;

   public:
    const_iterator begin() const noexcept { return log_odds_.begin(); }
    const_iterator end() const noexcept { return log_odds_.end(); }

    bool has_measurement(int channel) const {
      return log_odds_[channel] != 0.f;
    }

    bool is_occupied() const { return occupied_log_odds_ > 0.f; }
    // Each element in the log odds array indicates that the probability of the
    // corresponding type.
    float log_odds(int channel) const { return log_odds_[channel]; }
    void set_log_odds(int channel, float log_odds) {
      log_odds_[channel] = log_odds;
    }
    // Occupied log odds indicates that the occupied probability of a certain
    // grid. It means presence or absence.
    float occupied_log_odds() const { return occupied_log_odds_; }
    void set_occupied_log_odds(float log_odds) {
      occupied_log_odds_ = log_odds;
    }

   private:
    LogOddsArray log_odds_ = {};
    float occupied_log_odds_ = 0.0f;
  };

  GridInfo& operator()(int row, int col) {
    return info_grid_[row * width_ + col];
  }
  const GridInfo& operator()(int row, int col) const {
    return info_grid_[row * width_ + col];
  }
  GridInfo& operator()(const Vec2i& rc) {
    return info_grid_[rc.x() * width_ + rc.y()];
  }
  const GridInfo& operator()(const Vec2i& rc) const {
    return info_grid_[rc.x() * width_ + rc.y()];
  }

  bool is_observed(int row, int col) const {
    return ContainsKey(observed_infos_, Vec2i(row, col));
  }
  bool is_observed(const Vec2i& rc) const {
    return ContainsKey(observed_infos_, rc);
  }

  bool add_observed_info(int row, int col) {
    return observed_infos_.emplace(row, col).second;
  }
  bool add_observed_info(const Vec2i& rc) {
    return observed_infos_.emplace(rc).second;
  }

  const absl::flat_hash_set<Vec2i, RowColHash>& observed_infos() const {
    return observed_infos_;
  }

  void swap(ObstacleSemanticGrid& other) noexcept {
    QCHECK_EQ(width_, other.width_);
    QCHECK_EQ(height_, other.height_);

    std::swap(timestamp_, other.timestamp_);
    std::swap(pose_, other.pose_);

    observed_infos_.swap(other.observed_infos_);
    info_grid_.swap(other.info_grid_);
  }

  void clear() {
    timestamp_ = 0.0;
    pose_ = {};

    for (const auto& rc : observed_infos_) {
      info_grid_[rc.x() * width_ + rc.y()] = {};
    }
    observed_infos_.erase(observed_infos_.begin(), observed_infos_.end());
  }

  int width() const { return width_; }
  int height() const { return height_; }

  double timestamp() const { return timestamp_; }
  void set_timestamp(double timestamp) { timestamp_ = timestamp; }

  const VehiclePose& pose() const { return pose_; }
  void set_pose(const VehiclePose& pose) { pose_ = pose; }

 private:
  const int width_;
  const int height_;

  double timestamp_ = 0.0;
  VehiclePose pose_;
  // Include all grid infos that have been observed.
  absl::flat_hash_set<Vec2i, RowColHash> observed_infos_;

  std::vector<GridInfo> info_grid_;
};
// This class manages obstacle semantics.
// This is a fusion step where multiple outputs from different representation
// and different timestamp could be merged together.
class ObstacleSemanticManager {
 public:
  ObstacleSemanticManager(int width, int height);

  void ClassifyObstacles(const VehiclePose& pose, double timestamp,
                         ObstacleManager* obstacle_manager);

 private:
  // Compute current semantic grid. This should be divided into several stages
  // later if we have multiple obstacle info source.
  void ComputeCurrentGrid(const VehiclePose& pose, double timestamp,
                          const ObstaclePtrs& obstacles);
  void UpdateCurrentGridWithSemanticMap(const ObstaclePtrs& obstacles);
  // TODO(dong): Implement later.
  void UpdateCurrentGridWithSemanticSegmentationResults();
  void UpdateCurrentGridWithPillarSemanticResults();
  // Temporal fusion
  void PredictAndUpdate(const ObstacleManager& obstacle_manager);
  // get classification result.
  void ClassifyBySemanticGrid(ObstacleManager* obstacle_manager);
  // clean up functions
  void CleanUpAndSwap();

  // Obstacle level grid map.
  // Use a dense representation since we intend to describe all grids (not only
  // obstacles) in the future.
  // previous grid
  ObstacleSemanticGrid prev_grid_;
  // current grid
  ObstacleSemanticGrid curr_grid_;
};  // namespace qcraft

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_SEMANTIC_MANAGER_H_
