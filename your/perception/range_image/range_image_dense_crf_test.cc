#include <cstdio>
#include <random>
#include <utility>
#include <vector>

#include "onboard/perception/range_image/range_image_dense_crf_cpu.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifndef Q_CPU_ONLY
#include "onboard/perception/range_image/range_image_dense_crf_gpu.h"
#endif

#include "gtest/gtest.h"

namespace qcraft {

// Initial const value for testing
constexpr int kW = 8;
constexpr int kH = 8;
constexpr int kNumLabels = SegmentationType_ARRAYSIZE - 1;
constexpr int kNumIterations = 10;
constexpr float kWeightSmoothness = 1.0f;
constexpr float kSmoothnessXYDev = 1.0f;
constexpr float kWeightApp = 10.0f;
constexpr float kAppXYDev = 2.0f;
constexpr float kAppRIDev = 2.0f;

TEST(RangeImageDenseCRF, CPU) {
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

  // Use the CPU implementation
  range_image_dense_crf::RangeImageDenseCRFCPU dense_crf_cpu(
      semantic_image, range_image, intensity_image);
  auto result = dense_crf_cpu.PostProcessRangeImageDenseCRF(
      kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
      kAppXYDev, kAppRIDev);

  // Check if the result looks normal (we do not have ground truth here)
  EXPECT_EQ(result.first.size(), kW * kH);
  EXPECT_EQ(result.second.size(), kW * kH * kNumLabels);
  for (size_t i = 0; i < kW * kH; i++) {
    EXPECT_LT(result.first[i], kNumLabels);
    EXPECT_GE(result.first[i], 0);
  }
  const float min_prob = 1.0 / static_cast<float>(kNumLabels);
  for (size_t i = 0; i < kW * kH; i++) {
    float prob_sum = 0.0f;
    for (size_t j = 0; j < kNumLabels; j++) {
      prob_sum +=
          result.second[i * kNumLabels + j] * result.second[i * kNumLabels + j];
    }
    EXPECT_LE(prob_sum, 1);
    EXPECT_GE(prob_sum, min_prob);
  }
}

#ifndef Q_CPU_ONLY
TEST(RangeImageDenseCRF, GPU) {
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

  // Use the GPU implementation
  range_image_dense_crf::RangeImageDenseCRFGPU dense_crf_gpu(
      semantic_image, range_image, intensity_image);
  auto result = dense_crf_gpu.PostProcessRangeImageDenseCRF(
      kNumIterations, kWeightSmoothness, kSmoothnessXYDev, kWeightApp,
      kAppXYDev, kAppRIDev);

  // Check if the result looks normal (we do not have ground truth here)
  EXPECT_EQ(result.first.size(), kW * kH);
  EXPECT_EQ(result.second.size(), kW * kH * kNumLabels);
  for (size_t i = 0; i < kW * kH; i++) {
    EXPECT_LT(result.first[i], kNumLabels);
    EXPECT_GE(result.first[i], 0);
  }

  const float min_prob = 1.0 / static_cast<float>(kNumLabels);
  for (size_t i = 0; i < kW * kH; i++) {
    float prob_sum = 0.0f;
    for (size_t j = 0; j < kNumLabels; j++) {
      prob_sum +=
          result.second[i * kNumLabels + j] * result.second[i * kNumLabels + j];
    }
    EXPECT_LE(prob_sum, 1);
    EXPECT_GE(prob_sum, min_prob);
  }
}
#endif

}  // namespace qcraft
