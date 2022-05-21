#include <random>

#include "benchmark/benchmark.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/obstacle_util.h"

namespace qcraft {
namespace {

std::vector<Obstacle> GenerateRandomObstacle(int n) {
  std::vector<Obstacle> obs;
  obs.reserve(n);

  std::set<std::pair<int, int>> row_cols;

  for (int i = 0; i < n; ++i) {
    obs.push_back({});
    int col, row;
    do {
      col = rand() % 1000;  // NOLINT
      row = rand() % 1000;  // NOLINT
      obs.back().col = col;
      obs.back().row = row;
    } while (!row_cols.emplace(col, row).second);
  }
  return obs;
}

static std::vector<Obstacle> obstacles = GenerateRandomObstacle(10000);

void BM_ComputeObstacleManager(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    state.PauseTiming();
    ObstacleRefVector obstacle_refs;
    for (const auto& obs : obstacles) {
      obstacle_refs.push_back(std::make_unique<Obstacle>(obs));
    }
    state.ResumeTiming();
    ObstacleManager obstacle_manager(std::move(obstacle_refs),
                                     ObstacleRCCoordConverter(1000, 1000), {},
                                     {}, nullptr);
  }
}
BENCHMARK(BM_ComputeObstacleManager);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
