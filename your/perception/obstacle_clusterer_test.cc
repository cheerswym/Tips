#include "onboard/perception/obstacle_clusterer.h"

#include <time.h>

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/perception/obstacle_manager.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft {

std::pair<int, int> SmallestIndex(const Cluster& c) {
  std::pair<int, int> small_index(INT_MAX, INT_MAX);
  for (const auto* obstacle : c) {
    small_index = std::min(small_index,
                           std::pair<int, int>(obstacle->row, obstacle->col));
  }
  return small_index;
}

bool SmallerIndexCluster(const Cluster& c1, const Cluster& c2) {
  return SmallestIndex(c1) < SmallestIndex(c2);
}

float GetRandom() {
  return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

std::vector<std::pair<int, int>> GetClusterSortIndexs(const Cluster& c) {
  std::vector<std::pair<int, int>> cluster_indexs;
  for (const auto* obstacle : c) {
    cluster_indexs.emplace_back(obstacle->row, obstacle->col);
  }
  std::sort(std::begin(cluster_indexs), std::end(cluster_indexs));

  return cluster_indexs;
}

bool SameCluster(const Cluster& c1, const Cluster& c2) {
  const auto c1_indexs = GetClusterSortIndexs(c1);
  const auto c2_indexs = GetClusterSortIndexs(c2);
  QCHECK_EQ(c1_indexs.size(), c2_indexs.size());
  for (int i = 0; i < c1_indexs.size(); ++i) {
    if (c1_indexs[i] != c2_indexs[i]) {
      LOG(ERROR) << c1_indexs[i].first << ',' << c1_indexs[i].second << ' '
                 << c2_indexs[i].first << ',' << c2_indexs[i].second;
      return false;
    }
  }
  return true;
}

TEST(ObstacleClusterTest, TestObstacleCluster) {
  constexpr int kIterationTimes = 100;

  // Test the dump & set obstacle image success
  const int shift_size = 15.0 / Obstacle::kDiameter;
  for (int iter = 0; iter < kIterationTimes; ++iter) {
    const int height = GetRandom() * 424 + 600;
    const int width = height;
    const int num_obstacles = std::max(height * width / 10000, 10);

    ObstacleRefVector obstacles;
    const auto generate_random_obstacles_ptrs = [&]() {
      for (int i = 0; i < num_obstacles; ++i) {
        const int row = GetRandom() * shift_size + height / 2 - shift_size;
        const int col = GetRandom() * shift_size + width / 2 - shift_size;
        const int size_row = GetRandom() * 10 + 1;
        const int size_col = GetRandom() * 10 + 1;
        for (int j = row - size_row; j < row + size_row; ++j) {
          for (int k = col - size_col; k < col + size_col; ++k) {
            if (j >= 0 && j < height && k >= 0 && k < width) {
              if (GetRandom() > 0.7) {
                auto obstacle = std::make_unique<Obstacle>();
                obstacle->row = j;
                obstacle->col = k;
                obstacle->type = ObstacleProto::DYNAMIC;
                obstacle->points = {LaserPoint()};
                obstacles.push_back(std::move(obstacle));
              }
            }
          }
        }
      }
    };

    generate_random_obstacles_ptrs();
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        auto obstacle = std::make_unique<Obstacle>();
        obstacle->row = height / 2 + i * 10 / Obstacle::kDiameter;
        obstacle->col = width / 2 + j * 10 / Obstacle::kDiameter;
        obstacle->type = ObstacleProto::DYNAMIC;
        obstacle->points = {LaserPoint()};
        obstacles.push_back(std::move(obstacle));
      }
    }

    std::unique_ptr<ObstacleManager> obstacle_manager(new ObstacleManager(
        std::move(obstacles), ObstacleRCCoordConverter(0, 0), {}, {}, nullptr));
    const ObstaclePtrs obstacle_ptrs = obstacle_manager->obstacle_ptrs();

    QCHECK_GE(obstacle_ptrs.size(), 1);

    ObstacleClusterer obstacle_clusterer(width, height, nullptr);

    const auto cluster_compare = [&](const Cluster& c1, const Cluster& c2) {
      return SmallerIndexCluster(c1, c2);
    };

    const auto rc_coord_converter = obstacle_manager->RCCoordConverter();

    ClusterVector naive_clusters = obstacle_clusterer.ClusterObstacles(
        std::move(obstacle_ptrs), rc_coord_converter, VehiclePose(),
        ObstacleClusterer::FloodFilling::kNaive);

    ClusterVector scanline_clusters = obstacle_clusterer.ClusterObstacles(
        obstacle_ptrs, rc_coord_converter, VehiclePose(),
        ObstacleClusterer::FloodFilling::kScanline);

    std::sort(std::begin(naive_clusters), std::end(naive_clusters),
              cluster_compare);
    std::sort(std::begin(scanline_clusters), std::end(scanline_clusters),
              cluster_compare);

    EXPECT_EQ(naive_clusters.size(), scanline_clusters.size());
    EXPECT_GE(naive_clusters.size(), 1);

    for (int i = 0; i < naive_clusters.size(); ++i) {
      const auto naive_cluster = naive_clusters[i];
      const auto scanline_cluster = scanline_clusters[i];
      EXPECT_TRUE(SameCluster(naive_cluster, scanline_cluster));
    }
  }
}

}  // namespace qcraft
