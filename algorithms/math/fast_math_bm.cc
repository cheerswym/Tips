#include "onboard/math/fast_math.h"

#include "benchmark/benchmark.h"

namespace qcraft {
namespace {

constexpr int kNumIters = 100000;
constexpr double kValIncrement = 2 * M_PI / kNumIters;

static void BM_SlowAtan2(benchmark::State& state) {  // NOLINT
  double sum = 0.0;
  for (auto _ : state) {
    double val = 1.0;
    for (int i = 0; i < kNumIters; ++i, val += kValIncrement) {
      sum += std::atan2(i, val);
    }
  }
  CHECK_NE(sum, kNumIters);
}
BENCHMARK(BM_SlowAtan2);

static void BM_FastAtan2(benchmark::State& state) {  // NOLINT
  double sum = 0.0;
  for (auto _ : state) {
    double val = 1.0;
    for (int i = 0; i < kNumIters; ++i, val += kValIncrement) {
      sum += fast_math::Atan2(static_cast<double>(i), val);
    }
  }
  CHECK_NE(sum, kNumIters);
}
BENCHMARK(BM_FastAtan2);

static void BM_SlowSin(benchmark::State& state) {  // NOLINT
  double sum = 0.0;
  for (auto _ : state) {
    double val = 0.0;
    for (int i = 0; i < kNumIters; ++i, val += kValIncrement) {
      sum += std::sin(val);
    }
  }
  CHECK_NE(sum, kNumIters);
}
BENCHMARK(BM_SlowSin);

static void BM_FastSin(benchmark::State& state) {  // NOLINT
  double sum = 0.0;
  for (auto _ : state) {
    double val = 0.0;
    for (int i = 0; i < kNumIters; ++i, val += kValIncrement) {
      sum += fast_math::Sin(NormalizeAngle(val));
    }
  }
  CHECK_NE(sum, kNumIters);
}
BENCHMARK(BM_FastSin);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
