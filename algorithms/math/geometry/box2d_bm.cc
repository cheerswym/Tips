#include "onboard/math/geometry/box2d.h"

#include <random>

#include "benchmark/benchmark.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/segment2d.h"

namespace qcraft {
namespace planner {
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

std::vector<Segment2d> GenerateRandomSegments(int n) {
  const auto starts = GenerateRandomVector(n);
  const auto ends = GenerateRandomVector(n);
  std::vector<Segment2d> segments;
  for (int i = 0; i < n; ++i) {
    const auto end = starts[i] + (ends[i] - starts[i]) / 100.0;
    segments.push_back(Segment2d(starts[i], end));
  }
  return segments;
}

std::vector<Box2d> GenerateRandomBox2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  auto length_width = GenerateRandomVector(n);
  for (auto& lw : length_width) {
    lw /= 100.0;
  }
  std::vector<Box2d> boxes;
  boxes.reserve(n);
  for (int i = 0; i < n; ++i) {
    boxes.push_back(Box2d(centers[i], tangents[i],
                          std::max(0.1, length_width[i].x()),
                          std::max(0.1, length_width[i].y())));
  }
  return boxes;
}

std::vector<AABox2d> GenerateRandomAABox2d(int n) {
  const auto boxes = GenerateRandomBox2d(n);
  std::vector<AABox2d> aaboxes;
  aaboxes.reserve(n);
  for (const auto& box : boxes) {
    aaboxes.push_back(box.GetAABox());
  }
  return aaboxes;
}

void BM_Box2dConstructor(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  auto length_width = GenerateRandomVector(n);
  for (auto& lw : length_width) {
    lw /= 100.0;
  }
  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      benchmark::DoNotOptimize(Box2d(centers[i], tangents[i],
                                     std::max(0.1, length_width[i].x()),
                                     std::max(0.1, length_width[i].y())));
    }
  }
}
BENCHMARK(BM_Box2dConstructor)->Arg(10)->Arg(100)->Arg(1000);

void BM_Box2dOverlapsSegment2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto segments = GenerateRandomSegments(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(boxes[i].HasOverlap(segments[j]));
      }
    }
  }
}
BENCHMARK(BM_Box2dOverlapsSegment2d)->Arg(10)->Arg(100)->Arg(1000);

void BM_Box2dOverlapsBox2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(boxes[i].HasOverlap(boxes[j]));
      }
    }
  }
}
BENCHMARK(BM_Box2dOverlapsBox2d)->Arg(10)->Arg(100)->Arg(1000);

void BM_Box2dOverlapsAABox2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto aaboxes = GenerateRandomAABox2d(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(boxes[i].HasOverlap(aaboxes[j]));
      }
    }
  }
}
BENCHMARK(BM_Box2dOverlapsAABox2d)->Arg(10)->Arg(100)->Arg(1000);
}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
