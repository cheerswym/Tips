#include "onboard/perception/tracker/track_classifier/visual_feature_extractor.h"

#include <iostream>
#include <limits>

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace qcraft::tracker {
TEST(VisualFeatureExtractorTest, Sample) {
  return;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  QCHECK(param_manager != nullptr);
  std::unique_ptr<VisualFeatureExtractor> visual_feature_extractor =
      VisualFeatureExtractor::Create(*param_manager);
  QCHECK(visual_feature_extractor != nullptr);

  std::vector<cv::Mat> image_batch(
      64, cv::Mat(cv::Size(visual_feature_extractor->GetInputWidth(),
                           visual_feature_extractor->GetInputHeight()),
                  CV_8UC3, cv::Scalar(0)));
  const auto result = visual_feature_extractor->ExtractFeature(image_batch);
  QCHECK_EQ(result.size(), image_batch.size());
}

double cosine_similarity(const std::vector<float>& feat1,
                         const std::vector<float>& feat2) {
  double dot = 0.0, denom_a = 0.0, denom_b = 0.0;
  for (int i = 0; i < feat1.size(); ++i) {
    dot += feat1[i] * feat2[i];
    denom_a += feat1[i] * feat1[i];
    denom_b += feat2[i] * feat2[i];
  }
  return dot / (sqrt(denom_a) * sqrt(denom_b) +
                std::numeric_limits<double>::epsilon());
}

TEST(VisualFeatureExtractorTest, Feature) {
  constexpr float kSimalityThreshold = 0.7;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  QCHECK(param_manager != nullptr);
  std::unique_ptr<VisualFeatureExtractor> visual_feature_extractor =
      VisualFeatureExtractor::Create(*param_manager);
  QCHECK(visual_feature_extractor != nullptr);
  std::vector<std::string> names = {"c1_0.png", "c1_1.png", "h1_0.png",
                                    "h1_1.png", "h2_0.png", "h2_1.png",
                                    "h3_0.png", "h3_1.png"};
  const std::string img_dir = "onboard/nets/data/ped/";
  std::vector<cv::Mat> image_batch;
  image_batch.reserve(names.size());
  for (const auto& name : names) {
    cv::Mat img = cv::imread(img_dir + name);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    image_batch.push_back(img);
  }
  const auto result = visual_feature_extractor->ExtractFeature(image_batch);
  QCHECK_EQ(result.size(), image_batch.size());
  int n = result.size() / 2;
  for (int i = 0; i < result.size(); i += 2) {
    for (int j = 0; j < n; ++j) {
      const double cs = cosine_similarity(result[i], result[2 * j + 1]);
      if (i + 1 == 2 * j + 1) {
        QCHECK_GT(cs, kSimalityThreshold);
      } else {
        QCHECK_LE(cs, kSimalityThreshold);
      }
    }
  }
}
}  // namespace qcraft::tracker
