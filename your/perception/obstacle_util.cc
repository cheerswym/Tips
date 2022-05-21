#include "onboard/perception/obstacle_util.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/math/util.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(relax_ground_z_threshold, false,
            "If true, increase the ground Z threshold by 0.1m. This is only to "
            "temporarily fix short term issues such as calibration error, and "
            "should be deleted in the future");

namespace qcraft::obstacle_util {

std::vector<std::pair<int, int>> NeighborRange::GetNeighborOffsets(
    const Obstacle* obstacle) const {
  if (num_neighbors_ == 4) {
    return std::vector<std::pair<int, int>>{{-1, 0}, {0, -1}, {0, 1}, {1, 0}};
  } else if (num_neighbors_ == 8) {
    return std::vector<std::pair<int, int>>{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  } else {
    QCHECK(condition_ && obstacle);
    const int radius = condition_(*obstacle);
    std::vector<std::pair<int, int>> neighbor_offsets;
    neighbor_offsets.reserve(Sqr(2 * radius + 1));
    for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
      for (int col_offset = -radius; col_offset <= radius; ++col_offset) {
        if (row_offset == 0 && col_offset == 0) continue;
        neighbor_offsets.emplace_back(row_offset, col_offset);
      }
    }
    return neighbor_offsets;
  }
}

LocalObstacleGrid::LocalObstacleGrid(const ObstaclePtrs& obstacle_ptrs)
    : num_obstacles_(obstacle_ptrs.size()),
      min_vertex_(std::numeric_limits<int>::max(),
                  std::numeric_limits<int>::max()),
      max_vertex_(std::numeric_limits<int>::min(),
                  std::numeric_limits<int>::min()) {
  QCHECK(!obstacle_ptrs.empty());
  for (const auto* obstacle_ptr : obstacle_ptrs) {
    min_vertex_.first = std::min<int>(min_vertex_.first, obstacle_ptr->row);
    min_vertex_.second = std::min<int>(min_vertex_.second, obstacle_ptr->col);
    max_vertex_.first = std::max<int>(max_vertex_.first, obstacle_ptr->row);
    max_vertex_.second = std::max<int>(max_vertex_.second, obstacle_ptr->col);
  }
  height_ = max_vertex_.first - min_vertex_.first + 1;
  width_ = max_vertex_.second - min_vertex_.second + 1;
  local_grid_infos_.resize(width_ * height_);
  for (int i = 0; i < obstacle_ptrs.size(); ++i) {
    const int index =
        ObstacleRcToGridIndex(obstacle_ptrs[i]->row, obstacle_ptrs[i]->col);
    local_grid_infos_[index].obstacle = obstacle_ptrs[i];
    local_grid_infos_[index].raw_index = i;
  }
}

std::vector<int> LocalObstacleGrid::FindNearestInRadius(
    const int row, const int col, const int radius) const {
  const auto [local_row, local_col] = ObstacleRcToLocalRc(row, col);
  const int min_row = std::max(local_row - radius, 0);
  const int max_row = std::min(local_row + radius, height_ - 1);
  const int min_col = std::max(local_col - radius, 0);
  const int max_col = std::min(local_col + radius, width_ - 1);

  std::vector<int> neighbor_raw_indices;
  neighbor_raw_indices.reserve(Sqr(2 * radius + 1));
  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      const int index = row * width_ + col;
      const auto& info = local_grid_infos_[index];
      if (info.obstacle != nullptr) {
        neighbor_raw_indices.push_back(info.raw_index);
      }
    }
  }

  return neighbor_raw_indices;
}

std::vector<int> LocalObstacleGrid::FindConnectedNeighbors(
    const int row, const int col, const NeighborRange& neighbor_range,
    const ConnectedCondition& condition,
    const std::vector<bool>& has_processed) const {
  QCHECK_EQ(has_processed.size(), has_processed.empty() ? 0 : num_obstacles_);

  const auto [local_row, local_col] = ObstacleRcToLocalRc(row, col);
  if (!IsLocalRcInRange(local_row, local_col)) {
    return {};
  }
  const auto& info =
      local_grid_infos_[LocalRcToGridIndex(local_row, local_col)];
  if (info.obstacle == nullptr) return {};

  const auto neighbors_offsets =
      neighbor_range.GetNeighborOffsets(info.obstacle);

  std::vector<int> neighbor_raw_indices;
  neighbor_raw_indices.reserve(32);
  std::queue<std::pair<int, int>> queue;
  queue.emplace(local_row, local_col);
  absl::flat_hash_set<int> processed;
  while (!queue.empty()) {
    const auto [lrow, lcol] = queue.front();
    queue.pop();
    const int index = LocalRcToGridIndex(lrow, lcol);
    if (!processed.emplace(index).second) {
      continue;
    }
    const auto& seed_info = local_grid_infos_[index];
    QCHECK_NOTNULL(seed_info.obstacle);
    if (!has_processed.empty() && has_processed[seed_info.raw_index]) {
      continue;
    }
    neighbor_raw_indices.push_back(seed_info.raw_index);
    for (const auto& [row_offset, col_offset] : neighbors_offsets) {
      const int nlrow = lrow + row_offset;
      const int nlcol = lcol + col_offset;
      if (!IsLocalRcInRange(nlrow, nlcol)) {
        continue;
      }
      const int index = LocalRcToGridIndex(nlrow, nlcol);
      if (ContainsKey(processed, index)) {
        continue;
      }
      const auto& neighbor_info = local_grid_infos_[index];
      if (neighbor_info.obstacle == nullptr) {
        continue;
      }
      if (!condition || condition(*info.obstacle, *seed_info.obstacle,
                                  *neighbor_info.obstacle)) {
        queue.emplace(nlrow, nlcol);
      }
    }
  }

  return neighbor_raw_indices;
}

std::string LocalObstacleGrid::DebugString() const {
  return absl::StrFormat(
      "min_vertex: (%d, %d) max_vertex: (%d, %d) width: %d height: %d",
      min_vertex_.first, min_vertex_.second, max_vertex_.first,
      max_vertex_.second, width_, height_);
}

std::vector<float> ComputeGaussianKernel(const int kernel_size,
                                         const float sigma) {
  QCHECK(kernel_size % 2 == 1) << absl::StrFormat(
      "Currently, kernel size(%d) should be an odd number.", kernel_size);
  const float two_sigma_sqr = 2 * Sqr(sigma);
  const int half_kernel_size = (kernel_size - 1) / 2;
  std::vector<float> kernel(kernel_size * kernel_size);
  for (int i = 0; i < kernel_size; ++i) {
    for (int j = 0; j < kernel_size; ++j) {
      const int x = i - half_kernel_size;
      const int y = j - half_kernel_size;
      kernel[i * kernel_size + j] =
          std::exp(-(Sqr(x) + Sqr(y)) / two_sigma_sqr) / (two_sigma_sqr * M_PI);
    }
  }
  const float sum = std::accumulate(kernel.begin(), kernel.end(), 0.f);
  std::transform(kernel.begin(), kernel.end(), kernel.begin(),
                 [sum](const float value) { return value / sum; });
  return kernel;
}

}  // namespace qcraft::obstacle_util
