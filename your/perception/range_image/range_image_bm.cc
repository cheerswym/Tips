#include "benchmark/benchmark.h"
#include "onboard/lidar/spin_reader_test_util.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/range_image/range_image.h"
#include "snappy/snappy.h"

namespace qcraft {

void RangeImageBM(benchmark::State& state) {  // NOLINT
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  QCHECK_NOTNULL(param_manager)->GetRunParams(&run_params);

  const auto lidar_params = run_params.vehicle_params().lidar_params()[0];
  const auto spins = spin_reader::LoadSpinsForTest(LIDAR_PANDAR_64);
  const LidarFrame lidar_frame(
      Spin::CopySpinOnShm(*spins[0], LIDAR_PANDAR_64, LDR_CENTER));

  SemanticSegmentationResults ss_result;
  const auto all_camera_params =
      ComputeAllCameraParams(run_params.vehicle_params());
  for (const auto& [_, camera_param] : all_camera_params) {
    SemanticSegmentationResultProto result;
    constexpr int kWidth = 960;
    constexpr int kHeight = 540;
    result.set_mask_width(kWidth);
    result.set_mask_height(kHeight);
    result.set_sem_seg_output_scale(8.0);
    result.set_compression_format(SemanticSegmentationResultProto::SNAPPY);

    std::string data;
    data.resize(kWidth * kHeight);
    std::string compressed_data;
    compressed_data.resize(kWidth * kHeight);
    size_t compressed_data_size;
    snappy::RawCompress(data.data(), data.size(), compressed_data.data(),
                        &compressed_data_size);
    compressed_data.resize(compressed_data_size);

    result.set_labels(compressed_data);
    result.set_instance_labels(compressed_data);
    result.set_semantic_uncertainty_map(compressed_data);

    ss_result.emplace_back(
        std::make_shared<SemanticSegmentationResult>(result, camera_param));
  }

  for (auto _ : state) {
    const RangeImage ri(lidar_params, lidar_frame, ss_result);
  }
}

static void BM_RangeImage(benchmark::State& state) {  // NOLINT
  RangeImageBM(state);
}

BENCHMARK(BM_RangeImage);

}  // namespace qcraft

BENCHMARK_MAIN();
