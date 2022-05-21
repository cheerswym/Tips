#include "onboard/perception/tracker/track_classifier/visual_feature_extractor.h"

#include <memory>

namespace qcraft::tracker {
std::unique_ptr<VisualFeatureExtractor> VisualFeatureExtractor::Create(
    const ParamManager& param_manager) {
  return std::unique_ptr<VisualFeatureExtractor>(
      new VisualFeatureExtractor(param_manager));
}

VisualFeatureExtractor::VisualFeatureExtractor(
    const ParamManager& param_manager) {
  QLOG(INFO) << "Init VisualFeatureExtractor ...";
  RunParamsProtoV2 run_params;
  param_manager.GetRunParams(&run_params);
  CHECK_OK(param_manager.GetProtoParam("deepsort_reid_net_param",
                                       &deepsort_reid_net_param_));
  feature_extractor_ = std::make_unique<TcnImageNetClassifier>(
      run_params, deepsort_reid_net_param_);
  QLOG(INFO) << "Init VisualFeatureExtractor Done";
}

std::vector<std::vector<float>> VisualFeatureExtractor::ExtractFeature(
    const std::vector<cv::Mat>& images) {
  QCHECK_LE(images.size(), deepsort_reid_net_param_.max_batch_size());
  const int feature_length = feature_extractor_->GetFeatureLength();
  std::vector<std::vector<float>> result(images.size(),
                                         std::vector<float>(feature_length));

  std::vector<float> feature = feature_extractor_->ExtractImageFeature(images);
  int offset = 0;
  for (int i = 0; i < images.size(); ++i) {
    memcpy(result[i].data(), feature.data() + offset,
           sizeof(float) * feature_length);
    offset += feature_length;
  }
  return result;
}

int VisualFeatureExtractor::GetInputHeight() const {
  return feature_extractor_->GetInputHeight();
}

int VisualFeatureExtractor::GetInputWidth() const {
  return feature_extractor_->GetInputWidth();
}

int VisualFeatureExtractor::GetFeatureLength() const {
  return feature_extractor_->GetFeatureLength();
}

int VisualFeatureExtractor::MaxBatchSize() const {
  return deepsort_reid_net_param_.max_batch_size();
}

}  // namespace qcraft::tracker
