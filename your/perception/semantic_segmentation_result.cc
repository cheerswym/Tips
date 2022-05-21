#include "onboard/perception/semantic_segmentation_result.h"

#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "snappy/snappy.h"

namespace qcraft {

using SegmentationInstance = SemanticSegmentationResult::SegmentationInstance;
using SegmentationInstances = SemanticSegmentationResult::SegmentationInstances;

namespace {

constexpr int kMaxNumInstances = 256;

// TODO(dong): SegmentationInstances is aimed to be used in LLNetProposer &
// AddMeasurementsFromCamera(SemanticSegmentationResult). Each instance may
// contain a Box (as visual net) and a mask. As the current instance
// segmentation quality is worse than semantic results, we may need to divide
// the instance results depending on semantic results. This could be what to do
// next.
void GenerateSegmentationInstances(const cv::Mat &instance_mat,
                                   SegmentationInstances *instances) {
  std::array<SegmentationInstance, kMaxNumInstances> max_num_instance_array;
  for (int i = 0; i < instance_mat.rows; ++i) {
    for (int j = 0; j < instance_mat.cols; ++j) {
      const uchar instance_id = instance_mat.at<uchar>(i, j);
      // cv::Point using (x,y) as (column,row)
      max_num_instance_array[instance_id].pixels.emplace_back(j, i);
    }
  }
  for (int i = 0; i < max_num_instance_array.size(); ++i) {
    auto &instance = max_num_instance_array[i];

    if (instance.pixels.empty()) continue;

    instance.id = static_cast<uint8_t>(i);
    instances->emplace_back(std::move(instance));
  }
}

}  // namespace

void SemanticSegmentationResult::Init() {
  // Choose labels or otherwise labels_deprecated for backward compatability.
  const std::string &semantic_mask_string =
      raw_proto_.has_labels() ? raw_proto_.labels()
                              : raw_proto_.labels_deprecated();
  const auto decode_func = [this](const std::string &image_data) {
    switch (raw_proto_.compression_format()) {
      case SemanticSegmentationResultProto::PNG:
        return cv::imdecode(
            {image_data.data(), static_cast<int>(image_data.size())},
            cv::IMREAD_UNCHANGED);
      case SemanticSegmentationResultProto::SNAPPY: {
        cv::Mat image(raw_proto_.mask_height(), raw_proto_.mask_width(),
                      CV_8UC1);
        QCHECK(snappy::RawUncompress(
            reinterpret_cast<const char *>(image_data.data()),
            image_data.size(), reinterpret_cast<char *>(image.data)));
        return image;
      }
    }
  };

  semantic_mat_ = decode_func(semantic_mask_string);

  if (raw_proto_.has_semantic_uncertainty_map()) {
    const std::string &uncertainty_mask_string =
        raw_proto_.semantic_uncertainty_map();
    const std::vector<uchar> uncertainty_mask_array(
        uncertainty_mask_string.begin(), uncertainty_mask_string.end());
    uncertainty_mat_ = decode_func(uncertainty_mask_string);
  } else {
    uncertainty_mat_ =
        cv::Mat(semantic_mat_.rows, semantic_mat_.cols, CV_8UC1, 255);
  }

  if (raw_proto_.has_instance_labels()) {
    const std::string &instance_mask_string = raw_proto_.instance_labels();
    instance_mat_ = decode_func(instance_mask_string);
    GenerateSegmentationInstances(instance_mat_, &instances_);
  }

  pose_ = VehiclePose(raw_proto_.pose());
  smooth_to_camera_transform_ =
      (pose_.ToTransform() *
       camera_param_.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();
}

SemanticSegmentationResult SemanticSegmentationResult::Clone() const {
  SemanticSegmentationResult result = *this;
  result.semantic_mat_ = semantic_mat_.clone();
  result.instance_mat_ = instance_mat_.clone();
  result.uncertainty_mat_ = uncertainty_mat_.clone();
  return result;
}

}  // namespace qcraft
