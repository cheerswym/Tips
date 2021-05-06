#include "onboard/math/geometry/polygon2d.h"

#include <random>

#include "benchmark/benchmark.h"
#include "onboard/math/geometry/box2d.h"

namespace qcraft {
namespace planner {
namespace {

std::vector<Vec2d> GenerateRandomVector(int n) {
  std::vector<Vec2d> v;
  v.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(-5.0, 5.0);

  for (int i = 0; i < n; ++i) {
    v.push_back(Vec2d(dis(gen), dis(gen)));
  }
  return v;
}

// TODO(All): Generate irregular convex and nonconvex polygons.
std::vector<Polygon2d> GenerateRandomPolygon2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  const auto length_width = GenerateRandomVector(n);

  std::vector<Polygon2d> polys;
  polys.reserve(n);
  for (int i = 0; i < n; ++i) {
    polys.push_back(Polygon2d(Box2d(
        centers[i], tangents[i], std::max(0.1, std::fabs(length_width[i].x())),
        std::max(0.1, std::fabs(length_width[i].y())))));
  }
  return polys;
}

bool HasOverlap(const Polygon2d& polygon1, const Polygon2d& polygon2) {
  if (polygon2.max_x() < polygon1.min_x() ||
      polygon2.min_x() > polygon1.max_x() ||
      polygon2.max_y() < polygon1.min_y() ||
      polygon2.min_y() > polygon1.max_y()) {
    return false;
  }
  return polygon1.DistanceTo(polygon2) <= 1e-10;
}

void BM_OldPolygon2dOverlapsPolygon2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto polys = GenerateRandomPolygon2d(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(HasOverlap(polys[i], polys[j]));
      }
    }
  }
}
BENCHMARK(BM_OldPolygon2dOverlapsPolygon2d)->Arg(10)->Arg(100)->Arg(1000);

void BM_NewPolygon2dOverlapsPolygon2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto polys = GenerateRandomPolygon2d(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(polys[i].HasOverlap(polys[j]));
      }
    }
  }
}
BENCHMARK(BM_NewPolygon2dOverlapsPolygon2d)->Arg(10)->Arg(100)->Arg(1000);
}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
