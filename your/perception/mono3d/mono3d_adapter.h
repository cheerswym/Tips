#ifndef ONBOARD_PERCEPTION_MONO3D_MONO3D_ADAPTER_H_
#define ONBOARD_PERCEPTION_MONO3D_MONO3D_ADAPTER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/camera/camera_params.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/nets/mono3d_detector.h"
#include "opencv2/core.hpp"

namespace qcraft {

class Mono3DAdapter {
 public:
  explicit Mono3DAdapter(const NetParam& net_param);

  MultiCameraMono3dMeasurementsProto GetMono3dMeasurements(
      const CameraParams& camera_param, const CameraImage& image);

  MultiCameraMono3dMeasurementsProto GetMono3dMeasurements(
      CameraId camera_id, const CameraParams& camera_param,
      const cv::Mat& image, int width, int height, const VehiclePose& pose,
      double image_timestamp);

 private:
  static constexpr int kDepthBlockDownSampleRate = 4;
  static constexpr int kDepthBlockMaskWidth =
      Mono3D::kSupportMinImageWidth / kDepthBlockDownSampleRate;
  static constexpr int kDepthBlockMaskHeight =
      Mono3D::kSupportMinImageHeight / kDepthBlockDownSampleRate;
  Eigen::Matrix<int, kDepthBlockMaskHeight, kDepthBlockMaskWidth>
      depth_block_mask_;
  void SetVisibilityScores(std::vector<Mono3D::Mono3DBox>* mono3d_boxes);

  Mat3d EstimateCovFromCameraToSmooth(
      const Mono3D::Mono3DBox& mono3d_box,
      const AffineTransformation& camera_to_vehicle_transform,
      const VehiclePose& vehicle_pose) const;
  std::pair<Vec3d, double> ConvertPointFromCameraToSmooth(
      const Mono3D::Mono3DBox& mono3d_box,
      const AffineTransformation& camera_to_vehicle_transform,
      const VehiclePose& vehicle_pose) const;
  std::vector<Vec2i> GetProjected3dBBoxVertices(
      const Mono3D::Mono3DBox& mono3d_box,
      const CameraIntrinsicsMatrix& camera_matrix);

  const MeasurementType cls_type_[3] = {MT_PEDESTRIAN, MT_CYCLIST, MT_VEHICLE};

  absl::Mutex mono3d_detector_mutex_;
  std::unique_ptr<Mono3DDetector> mono3d_detector_
      GUARDED_BY(mono3d_detector_mutex_);

  // Image buffer to hold resized and padded images for all cameras.
  absl::Mutex padded_images_mutex_;
  std::map<CameraId, cv::Mat> padded_images_ GUARDED_BY(padded_images_mutex_);
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_MONO3D_MONO3D_ADAPTER_H_
