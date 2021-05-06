#include "onboard/math/geometry/line_intersection.h"

#include <random>

#include "benchmark/benchmark.h"
#include "gtest/gtest.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace {

std::vector<Vec2d> GenerateRandomVector(int n) {
  std::vector<Vec2d> v;
  v.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(-1.0e5, 1.0e5);

  for (int i = 0; i < n; ++i) {
    v.push_back(Vec2d(dis(gen), dis(gen)));
  }
  return v;
}

void BM_LineIntersectionSolver(benchmark::State &state) {  // NOLINT
  constexpr int kNum = 400;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  double s0;
  double s1;
  for (auto _ : state) {
    for (int i = 0; i < kNum; i += 4) {
      benchmark::DoNotOptimize(FindIntersectionBetweenLinesWithTangents(
          vec[i], vec[i + 1], vec[i + 2], vec[i + 3], &s0, &s1));
    }
  }
}
BENCHMARK(BM_LineIntersectionSolver);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
