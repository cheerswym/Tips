#include <cstdio>
#include <random>
#include <utility>
#include <vector>

#include "benchmark/benchmark.h"
#include "onboard/perception/range_image/range_image_dense_crf_cpu.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifndef Q_CPU_ONLY
#include "onboard/perception/range_image/range_image_dense_crf_gpu.h"
#endif

namespace qcraft {

// Initial const value for testing
constexpr int kW = 128;
constexpr int kH = 64;
constexpr int kNumIterations = 10;
constexpr float kWeightSmoothness = 1.0f;
constexpr float kSmoothnessXYDev = 1.0f;
constexpr float kWeightApp = 10.0f;
constexpr float kAppXYDev = 2.0f;
constexpr float kAppRIDev = 2.0f;

void RangeImageDenseCRFCPUBM(benchmark::State& state) {  // NOLINT
  // Initalize both semantic range and intensity image. Be careful that semantic
  // image has two channels including uncertainty
  cv::Mat semantic_image = cv::Mat::zeros(kH, kW, CV_8UC2);
  cv::Mat intensity_image = cv::Mat::zeros(kH, kW, CV_8UC1);
  cv::Mat range_image = cv::Mat::zeros(kH, kW, CV_8UC1);

  // Random generator, uniform, unbiased
  std::mt19937 rng(40);
  std::uniform_int_distribution<int> gen(0, 255);

  // Generate random value to assign with each pixel
  for (int i = 0; i < kW * kH; i++) {
    int r = gen(rng);
    unsigned char c = static_cast<unsigned char>(r);
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[0] = c;
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[1] = c;
    intensity_image.at<uchar>(i / kW, i % kW) = c;
    range_image.at<uchar>(i / kW, i % kW) = c;
  }

  // Initalize the GPU class instance
  range_image_dense_crf::RangeImageDenseCRFCPU dense_crf_cpu(
      semantic_image, range_image, intensity_image);

  for (auto _ : state) {
    auto result = dense_crf_cpu.PostProcessRangeImageDenseCRF(
        kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
        kAppXYDev, kAppRIDev);
  }
}

static void BM_RangeImageCPU(benchmark::State& state) {  // NOLINT
  RangeImageDenseCRFCPUBM(state);
}

BENCHMARK(BM_RangeImageCPU);

#ifndef Q_CPU_ONLY
void RangeImageDenseCRFGPUBM(benchmark::State& state) {  // NOLINT
  // Initalize both semantic range and intensity image. Be careful that semantic
  // image has two channels including uncertainty
  cv::Mat semantic_image = cv::Mat::zeros(kH, kW, CV_8UC2);
  cv::Mat intensity_image = cv::Mat::zeros(kH, kW, CV_8UC1);
  cv::Mat range_image = cv::Mat::zeros(kH, kW, CV_8UC1);

  // Random generator, uniform, unbiased
  std::mt19937 rng(40);
  std::uniform_int_distribution<int> gen(0, 255);

  // Generate random value to assign with each pixel
  for (int i = 0; i < kW * kH; i++) {
    int r = gen(rng);
    unsigned char c = static_cast<unsigned char>(r);
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[0] = c;
    semantic_image.at<cv::Vec2b>(i / kW, i % kW)[1] = c;
    intensity_image.at<uchar>(i / kW, i % kW) = c;
    range_image.at<uchar>(i / kW, i % kW) = c;
  }

  // Initalize the GPU class instance
  range_image_dense_crf::RangeImageDenseCRFGPU dense_crf_gpu(
      semantic_image, range_image, intensity_image);

  for (auto _ : state) {
    auto result = dense_crf_gpu.PostProcessRangeImageDenseCRF(
        kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
        kAppXYDev, kAppRIDev);
  }
}

static void BM_RangeImageGPU(benchmark::State& state) {  // NOLINT
  RangeImageDenseCRFGPUBM(state);
}

BENCHMARK(BM_RangeImageGPU);
#endif
}  // namespace qcraft

BENCHMARK_MAIN();
