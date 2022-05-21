#ifndef ONBOARD_PERCEPTION_SEMANTIC_SEGMENTATION_RESULT_H_
#define ONBOARD_PERCEPTION_SEMANTIC_SEGMENTATION_RESULT_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/camera/camera_params.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/map_util.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace qcraft {

class SemanticSegmentationResult {
 public:
  struct SegmentationInstance {
    uint8_t id;
    SegmentationType type;
    uint32_t width;
    uint32_t height;
    std::vector<cv::Point> pixels;
  };

  using SegmentationInstances = std::vector<SegmentationInstance>;

  explicit SemanticSegmentationResult(
      const SemanticSegmentationResultProto& proto,
      const CameraParams& camera_param)
      : raw_proto_(proto), camera_param_(camera_param) {
    Init();
  }
  // NOTE(dong): For some old runs when we have only semantic segmentation
  // results, we don't have instance labels. We need to use the following
  // checker to check if the run has instance labels or not.
  bool HasInstance() const { return raw_proto_.has_instance_labels(); }

  CameraId camera_id() const { return raw_proto_.camera_id(); }
  double image_center_timestamp() const {
    return raw_proto_.image_center_timestamp();
  }
  uint32_t mask_height() const { return raw_proto_.mask_height(); }
  uint32_t mask_width() const { return raw_proto_.mask_width(); }

  SegmentationType GetPixelType(const int x, const int y) const {
    return static_cast<SegmentationType>(
        semantic_mat_.at<uchar>(y, x));  // (row, col) -> (y, x)
  }
  uchar GetPixelId(const int x, const int y) const {
    return instance_mat_.at<uchar>(y, x);  // (row, col) -> (y, x)
  }
  uchar GetPixelUncertainty(const int x, const int y) const {
    return uncertainty_mat_.at<uchar>(y, x);  // (row, col) -> (y, x)
  }
  double sem_seg_output_scale() const {
    if (raw_proto_.has_sem_seg_output_scale()) {
      return raw_proto_.sem_seg_output_scale();
    } else {
      return 1.0;
    }
  }
  const cv::Mat& semantic_mat() const { return semantic_mat_; }
  const cv::Mat& instance_mat() const { return instance_mat_; }
  const cv::Mat& uncertainty_mat() const { return uncertainty_mat_; }
  const VehiclePose& pose() const { return pose_; }
  const CameraParams& camera_param() const { return camera_param_; }
  LidarId ref_lidar_id() const { return camera_param_.ref_lidar(); }
  const AffineTransformation& smooth_to_camera_transform() const {
    return smooth_to_camera_transform_;
  }
  // Deep copy.
  SemanticSegmentationResult Clone() const;

 private:
  void Init();

 private:
  SemanticSegmentationResultProto raw_proto_;
  cv::Mat semantic_mat_;
  cv::Mat instance_mat_;
  cv::Mat uncertainty_mat_;
  SegmentationInstances instances_;
  VehiclePose pose_;
  AffineTransformation smooth_to_camera_transform_;
  CameraParams camera_param_;
};

using SemanticSegmentationResultRef =
    std::shared_ptr<SemanticSegmentationResult>;
using SemanticSegmentationResults = std::vector<SemanticSegmentationResultRef>;

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_SEMANTIC_SEGMENTATION_RESULT_H_
