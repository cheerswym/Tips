#include "benchmark/benchmark.h"
#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft::sensor_fov {

void BM_SensorFovBuilder_Compute(benchmark::State& state) {  // NOLINT
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  for (auto _ : state) {
    const auto sensor_fovs = sensor_fov_builder->Compute(
        obstacle_ptrs, clusters, {0.0, VehiclePose()});
    benchmark::DoNotOptimize(sensor_fovs);
  }
}
BENCHMARK(BM_SensorFovBuilder_Compute);

}  // namespace qcraft::sensor_fov

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags*/ true);
  // Raw BENCHMARK_MAIN() marco.
  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
