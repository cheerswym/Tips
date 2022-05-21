#if Q_CPU_ONLY == 0

#include <cmath>
#include <cstdio>
#include <random>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/range_image/range_image_dense_crf_cpu.h"
#include "onboard/perception/range_image/range_image_dense_crf_gpu.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft {

// Initial const value for testing
constexpr int kW = 10;
constexpr int kH = 10;
constexpr int kNumLabels = SegmentationType_ARRAYSIZE - 1;
constexpr int kNumIterations = 10;
constexpr float kWeightSmoothness = 1.0f;
constexpr float kSmoothnessXYDev = 1.0f;
constexpr float kWeightApp = 10.0f;
constexpr float kAppXYDev = 2.0f;
constexpr float kAppRIDev = 2.0f;
constexpr float kUncertaintyDiffThreshold = 0.05f;

TEST(RangeImageDenseCRFSyncTest, TestSync) {
  // Prepare the data for CPU and GPU
  cv::Mat semantic_image = cv::Mat::zeros(kH, kW, CV_8UC2);
  cv::Mat intensity_image = cv::Mat::zeros(kH, kW, CV_8UC1);
  cv::Mat range_image = cv::Mat::zeros(kH, kW, CV_8UC1);

  // Random generator, uniform, unbiased
  std::mt19937 rng(40);
  std::uniform_int_distribution<int> gen(22, 255);

  // Generate random value to assign with each pixel
  for (int i = 0; i < kW * kH; i++) {
    int r = gen(rng);
    unsigned char c = static_cast<unsigned char>(r);
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[0] = c;
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[1] = c;
    intensity_image.at<uchar>(i / kW, i % kW) = c;
    range_image.at<uchar>(i / kW, i % kW) = c;
  }

  // Initalize both CPU and GPU implementation
  range_image_dense_crf::RangeImageDenseCRFCPU dense_crf_cpu(
      semantic_image, range_image, intensity_image);
  range_image_dense_crf::RangeImageDenseCRFGPU dense_crf_gpu(
      semantic_image, range_image, intensity_image);

  // Process the random generated image separately with two versions
  const absl::Time start_cpu = absl::Now();
  auto cpu_result = dense_crf_cpu.PostProcessRangeImageDenseCRF(
      kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
      kAppXYDev, kAppRIDev);
  LOG(INFO) << "DenseCRF cpu inference time including data preparation time: "
            << (absl::Now() - start_cpu) / absl::Microseconds(1) * 1e-3
            << " ms.";

  const absl::Time start_gpu = absl::Now();
  auto gpu_result = dense_crf_gpu.PostProcessRangeImageDenseCRF(
      kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
      kAppXYDev, kAppRIDev);
  LOG(INFO) << "DenseCRF gpu inference time including data preparation time: "
            << (absl::Now() - start_gpu) / absl::Microseconds(1) * 1e-3
            << " ms.";

  // Compare the mapping result
  for (int i = 0; i < kW * kH; i++) {
    EXPECT_EQ(cpu_result.first[i], gpu_result.first[i]);
  }

  // Compare the uncertainty result
  for (size_t i = 0; i < kW * kH; i++) {
    float prob_sum_cpu = 0.0f;
    float prob_sum_gpu = 0.0f;
    for (size_t j = 0; j < kNumLabels; j++) {
      prob_sum_cpu += cpu_result.second[i * kNumLabels + j] *
                      cpu_result.second[i * kNumLabels + j];
      prob_sum_gpu += gpu_result.second[i * kNumLabels + j] *
                      gpu_result.second[i * kNumLabels + j];
    }
    EXPECT_NEAR(prob_sum_cpu, prob_sum_gpu, kUncertaintyDiffThreshold);
  }
}
}  // namespace qcraft

#endif  // Q_CPU_ONLY == 0
