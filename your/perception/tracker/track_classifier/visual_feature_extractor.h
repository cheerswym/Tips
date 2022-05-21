#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_VISUAL_FEATURE_EXTRACTOR_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_VISUAL_FEATURE_EXTRACTOR_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/nets/tcn_net_classifier.h"
#include "onboard/params/param_manager.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::tracker {
class VisualFeatureExtractor {
 public:
  static std::unique_ptr<VisualFeatureExtractor> Create(
      const ParamManager& param_manager);
  std::vector<std::vector<float>> ExtractFeature(
      const std::vector<cv::Mat>& images);
  int GetInputHeight() const;
  int GetInputWidth() const;
  int MaxBatchSize() const;
  int GetFeatureLength() const;

 private:
  std::unique_ptr<TcnImageNetClassifier> feature_extractor_;
  NetParam deepsort_reid_net_param_;
  explicit VisualFeatureExtractor(const ParamManager& param_manager);
};
}  // namespace qcraft::tracker

// NOLINTNEXTLINE
#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_VISUAL_FEATURE_EXTRACTOR_H_
