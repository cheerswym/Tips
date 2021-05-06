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

std::vector<Vec3d> GenerateRandomVector3d(int n) {
  std::vector<Vec3d> v;
  v.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(-1.0e5, 1.0e5);

  for (int i = 0; i < n; ++i) {
    v.push_back(Vec3d(dis(gen), dis(gen), dis(gen)));
  }
  return v;
}

void BM_Vec2dNorm(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 100;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(vec[i].norm());
    }
  }
}
BENCHMARK(BM_Vec2dNorm);

void BM_Vec2dAngle(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 100;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(vec[i].Angle());
    }
  }
}
BENCHMARK(BM_Vec2dAngle);

void BM_Vec2dFastAngle(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 100;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(vec[i].FastAngle());
    }
  }
}
BENCHMARK(BM_Vec2dFastAngle);

Vec2d TestCallByCopy(Vec2d v) { return (v + v) * 2.0; }
void BM_Vec2dCopy(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByCopy(vec[i]));
    }
  }
}
BENCHMARK(BM_Vec2dCopy);

Vec2d TestCallByReference(const Vec2d& v) { return (v + v) * 2.0; }
void BM_Vec2dReference(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<Vec2d> vec = GenerateRandomVector(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByReference(vec[i]));
    }
  }
}
BENCHMARK(BM_Vec2dReference);

Vec3d TestCallByCopy(Vec3d v) { return (v + v) * 2.0; }
void BM_Vec3dCopy(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<Vec3d> vec = GenerateRandomVector3d(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByCopy(vec[i]));
    }
  }
}
BENCHMARK(BM_Vec3dCopy);

Vec3d TestCallByReference(const Vec3d& v) { return (v + v) * 2.0; }
void BM_Vec3dReference(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  std::vector<Vec3d> vec = GenerateRandomVector3d(kNum);
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestCallByReference(vec[i]));
    }
  }
}
BENCHMARK(BM_Vec3dReference);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
