#include <random>

#include "benchmark/benchmark.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/perception/obstacle_util.h"

namespace qcraft {
namespace {

using namespace obstacle_util;  // NOLINT

constexpr int kNum = 1000;

std::vector<Obstacle> GenerateRandomObstacle(int n) {
  std::vector<Obstacle> obs;
  obs.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<float> dis(0.f, 100.f);

  for (int i = 0; i < n; ++i) {
    obs.push_back({});
    obs.back().points.push_back({.range = dis(gen)});
  }
  return obs;
}

static std::vector<Obstacle> obstacles = GenerateRandomObstacle(kNum);

void BM_ComputeNearGroundMaxZ(benchmark::State& state) {  // NOLINT
  float sum = 0.f;
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      sum += GetObstacleNearGroundMaxZ(obstacles[i]);
    }
  }
  std::cout << sum << std::endl;
}
BENCHMARK(BM_ComputeNearGroundMaxZ);

void BM_ComputeNearGroundMaxZUsingPiecewiseLinearFunction(
    benchmark::State& state) {  // NOLINT
  PiecewiseLinearFunction piecewise_linear_function(
      std::vector<float>{kMinNearGroundDist, kMidNearGroundDist,
                         kMaxNearGroundDist},
      std::vector<float>{kMinNearGroundHeight, kMidNearGroundHeight,
                         kMaxNearGroundHeight});
  float sum = 0.f;
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      sum += piecewise_linear_function(obstacles[i].points[0].range);
    }
  }
  std::cout << sum << std::endl;
}
BENCHMARK(BM_ComputeNearGroundMaxZUsingPiecewiseLinearFunction);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
