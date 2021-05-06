#include <random>

#include "benchmark/benchmark.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/aabox2d.h"

namespace qcraft {
namespace {

std::vector<AABox2d> GenerateRandomAABox(int n) {
  std::vector<AABox2d> v;
  v.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(-1.0e5, 1.0e5);
  std::uniform_real_distribution<> lw_dis(0.0, 100.0);

  for (int i = 0; i < n; ++i) {
    v.push_back(AABox2d(/*half_length=*/lw_dis(gen), /*half_width=*/lw_dis(gen),
                        Vec2d(dis(gen), dis(gen))));
  }
  return v;
}

AABox2d TestCallByCopy(AABox2d v) {
  const AABox2d v1(v.half_length(), v.half_width(), v.center());
  return v1;
}
void BM_AABox2dCopy(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<AABox2d> boxes = GenerateRandomAABox(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByCopy(boxes[i]));
    }
  }
}
BENCHMARK(BM_AABox2dCopy);

AABox2d TestCallByReference(const AABox2d& v) {
  const AABox2d v1(v.half_length(), v.half_width(), v.center());
  return v1;
}
void BM_AABox2dReference(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<AABox2d> boxes = GenerateRandomAABox(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByReference(boxes[i]));
    }
  }
}
BENCHMARK(BM_AABox2dReference);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
