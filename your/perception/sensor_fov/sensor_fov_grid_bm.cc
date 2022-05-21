#include <string>

#include "benchmark/benchmark.h"
#include "onboard/perception/sensor_fov/sensor_fov_constants.h"
#include "onboard/perception/sensor_fov/sensor_fov_grid.h"

namespace qcraft::sensor_fov {

void BM_SensorFovGrid_Serialize(benchmark::State& state) {  // NOLINT
  SensorFovGrid<SFPillar> sf_grid(kSFDetectionRangeFront, kSFDetectionRangeRear,
                                  kSFDetectionRangeLateral,
                                  kSFDetectionRangeLateral, kSFGridDiameter);
  for (auto _ : state) {
    std::string serialized_data;
    sf_grid.Serialize(&serialized_data);
  }
}
BENCHMARK(BM_SensorFovGrid_Serialize);

void BM_SensorFovGrid_Deserialize(benchmark::State& state) {  // NOLINT
  SensorFovGrid<SFPillar> sf_grid(kSFDetectionRangeFront, kSFDetectionRangeRear,
                                  kSFDetectionRangeLateral,
                                  kSFDetectionRangeLateral, kSFGridDiameter);
  for (auto _ : state) {
    std::string serialized_data;
    serialized_data.resize(sf_grid.width() * sf_grid.height() *
                           SFPillar::kSerializedSize);
    sf_grid.Deserialize(serialized_data);
  }
}
BENCHMARK(BM_SensorFovGrid_Deserialize);

}  // namespace qcraft::sensor_fov

BENCHMARK_MAIN();
