#include <unordered_map>

#include "absl/random/random.h"
#include "benchmark/benchmark.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/obstacle_clusterer.h"
#include "onboard/perception/obstacle_manager.h"

namespace qcraft {

constexpr int kHeight = 1500;
constexpr int kWidth = 1500;

float GetRandom() {
  static std::mt19937 random_generator_;
  static absl::gaussian_distribution<float> normal_random(0.5, 0.05);
  return normal_random(random_generator_);
}

std::unique_ptr<ObstacleManager> GenerateObstacleClusterPtrs() {
  ObstacleRefVector obstacles;
  const int num_samples = 1000;
  std::vector<Vec2i> offsets{
      {0, 0}, {-1, 0}, {0, -1}, {0, 1}, {1, 0},
  };
  for (int i = 0; i < num_samples / offsets.size(); ++i) {
    const float r1 = std::clamp(GetRandom(), 0.0f, 1.0f);
    const float r2 = std::clamp(GetRandom(), 0.0f, 1.0f);
    const int row = std::clamp<int>(r1 * kHeight, 1, kHeight - 2);
    const int col = std::clamp<int>(r2 * kWidth, 1, kWidth - 2);
    for (int k = 0; k < offsets.size(); ++k) {
      auto obstacle = std::make_unique<Obstacle>();
      obstacle->row = row + offsets[k].x();
      obstacle->col = col + offsets[k].y();
      obstacle->type = ObstacleProto::DYNAMIC;
      obstacle->points = {LaserPoint()};
      obstacles.push_back(std::move(obstacle));
    }
  }

  auto obstacle_manager = std::make_unique<ObstacleManager>(
      std::move(obstacles), ObstacleRCCoordConverter(0, 0), CameraParamsMap(),
      std::unordered_map<LidarId, LidarParametersProto>(), nullptr);

  return obstacle_manager;
}

static void BM_ObstacleClusterGaussianBlur(benchmark::State& state) {  // NOLINT
  cv::Mat image(kHeight, kWidth, CV_8UC1, cv::Scalar(0));
  for (auto _ : state) {
    cv::GaussianBlur(image, image, {7, 7}, 0.0);
  }
}

static void BM_ObstacleClusterNaiveFloodFilling(
    benchmark::State& state) {  // NOLINT
  const auto obstacle_manager = GenerateObstacleClusterPtrs();
  const ObstaclePtrs obstacles = obstacle_manager->obstacle_ptrs();
  ObstacleClusterer obstacle_clusterer(kWidth, kHeight, nullptr);
  const auto rc_coord_converter = obstacle_manager->RCCoordConverter();
  for (auto _ : state) {
    obstacle_clusterer.ClusterObstacles(
        obstacles, rc_coord_converter, VehiclePose(),
        ObstacleClusterer::FloodFilling::kNaive);
  }
}

static void BM_ObstacleClusterScanLineFloodFilling(
    benchmark::State& state) {  // NOLINT
  const auto obstacle_manager = GenerateObstacleClusterPtrs();
  const ObstaclePtrs obstacles = obstacle_manager->obstacle_ptrs();
  ObstacleClusterer obstacle_clusterer(kWidth, kHeight, nullptr);
  const auto rc_coord_converter = obstacle_manager->RCCoordConverter();
  for (auto _ : state) {
    obstacle_clusterer.ClusterObstacles(
        obstacles, rc_coord_converter, VehiclePose(),
        ObstacleClusterer::FloodFilling::kScanline);
  }
}

BENCHMARK(BM_ObstacleClusterGaussianBlur);
BENCHMARK(BM_ObstacleClusterNaiveFloodFilling);
BENCHMARK(BM_ObstacleClusterScanLineFloodFilling);

}  // namespace qcraft

BENCHMARK_MAIN();
