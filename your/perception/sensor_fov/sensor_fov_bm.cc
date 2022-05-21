#include "benchmark/benchmark.h"
#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft::sensor_fov {

void BM_SensorFov_SensorFovToProto(benchmark::State& state) {  // NOLINT
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);
  for (auto _ : state) {
    const auto proto = SensorFov::SensorFovToProto(*sensor_fov);
    benchmark::DoNotOptimize(proto);
  }
}
BENCHMARK(BM_SensorFov_SensorFovToProto);

void BM_SensorFov_ProtoToSensorFov(benchmark::State& state) {  // NOLINT
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);
  const auto proto = SensorFov::SensorFovToProto(*sensor_fov);
  for (auto _ : state) {
    const auto sf = SensorFov::ProtoToSensorFov(proto);
    benchmark::DoNotOptimize(sf);
  }
}
BENCHMARK(BM_SensorFov_ProtoToSensorFov);

void BM_SensorFov_IsOccluded_Box2d(benchmark::State& state) {  // NOLINT
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);
  const Box2d box({40.0, -30.0}, 0.0, 5.0, 2.0);
  for (auto _ : state) {
    const auto status_or = sensor_fov->IsOccluded(box, 1.0);
    benchmark::DoNotOptimize(status_or);
  }
}
BENCHMARK(BM_SensorFov_IsOccluded_Box2d);

void BM_SensorFov_IsOccluded_Polygon2d(benchmark::State& state) {  // NOLINT
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);
  std::vector<Vec2d> points{{40.0, -30.0}, {41.0, -29.9}, {42.0, -29.8},
                            {43.0, -29.9}, {44.0, -30.0}, {44.0, -32.0},
                            {43.0, -32.1}, {42.0, -32.2}, {41.0, -32.1},
                            {40.0, -32.0}};
  const Polygon2d polygon(std::move(points));
  for (auto _ : state) {
    const auto status_or = sensor_fov->IsOccluded(polygon, 1.0);
    benchmark::DoNotOptimize(status_or);
  }
}
BENCHMARK(BM_SensorFov_IsOccluded_Polygon2d);

}  // namespace qcraft::sensor_fov

BENCHMARK_MAIN();
