#include <random>

#include "benchmark/benchmark.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/aabox_kdtree2d.h"

namespace qcraft {
namespace {

double RandomDouble(double start, double end) {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(start, end);
  return dis(gen);
}
constexpr double kSize = 1e5;

class Object {
 public:
  Object(double x1, const double y1, const double x2, const double y2,
         const int id)
      : aabox_({x1, y1}, {x2, y2}), id_(id) {}
  const AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const Vec2d &point) const {
    return aabox_.DistanceTo(point);
  }
  double DistanceSquareTo(const Vec2d &point) const {
    return aabox_.SquaredDistanceTo(point);
  }
  int id() const { return id_; }

 private:
  AABox2d aabox_;
  int id_ = 0;
};

std::vector<Object> GenerateObjects(int num_boxes) {
  std::vector<Object> objects;
  objects.reserve(num_boxes);
  for (int i = 0; i < num_boxes; ++i) {
    const double cx = RandomDouble(-kSize, kSize);
    const double cy = RandomDouble(-kSize, kSize);
    const double dx = RandomDouble(-kSize / 50.0, kSize / 50.0);
    const double dy = RandomDouble(-kSize / 50.0, kSize / 50.0);
    objects.emplace_back(cx - dx, cy - dy, cx + dx, cy + dy, i);
  }
  return objects;
}
std::vector<Vec2d> GenerateRandomPoints(int num_points) {
  std::vector<Vec2d> points;
  points.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    points.emplace_back(RandomDouble(-kSize * 1.5, kSize * 1.5),
                        RandomDouble(-kSize * 1.5, kSize * 1.5));
  }
  return points;
}

static void KdTreeBuildArgs(benchmark::internal::Benchmark *b) {
  for (int num_boxes = 10; num_boxes <= 1e4; num_boxes *= 10) {
    for (int depth = 2; depth <= 8; depth += 2) {
      b->Args({num_boxes, depth});
    }
  }
}

void BM_BuildKdTree(benchmark::State &state) {  // NOLINT
  const auto boxes = GenerateObjects(state.range(0));
  AABoxKDTreeParams params;
  params.max_depth = state.range(1);
  params.max_leaf_dimension = kSize / 4.0;
  params.max_leaf_size = 20;
  for (auto _ : state) {
    benchmark::DoNotOptimize(AABoxKDTree2d<Object>(boxes, params));
  }
}
BENCHMARK(BM_BuildKdTree)->Apply(KdTreeBuildArgs);

void BM_QueryKdTree(benchmark::State &state) {  // NOLINT
  const auto boxes = GenerateObjects(state.range(0));
  AABoxKDTreeParams params;
  params.max_depth = 4;
  params.max_leaf_dimension = kSize / 4.0;
  params.max_leaf_size = 20;

  AABoxKDTree2d<Object> tree(boxes, params);
  const auto points = GenerateRandomPoints(1000);
  for (auto _ : state) {
    for (const auto &pt : points) {
      benchmark::DoNotOptimize(tree.GetNearestObject(pt));
    }
  }
}
BENCHMARK(BM_QueryKdTree)->Arg(100)->Arg(1000)->Arg(10000);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
