#include "onboard/perception/mono3d/mono3d_adapter.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/math/geometry/util.h"
#include "onboard/nets/net_util.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft {
namespace {
// [RearUpRight, RearUpLeft, RearDownLeft, RearDownRight,
//  FrontUpRight, FrontUpLeft, FrontDownLeft, FrontDownRight]
static const std::vector<qcraft::Vec3d> corners_norm = {
    {-0.5, -1.0, -0.5}, {-0.5, -1.0, 0.5}, {-0.5, 0.0, 0.5}, {-0.5, 0.0, -0.5},
    {0.5, -1.0, -0.5},  {0.5, -1.0, 0.5},  {0.5, 0.0, 0.5},  {0.5, 0.0, -0.5}};
// The flattened resized matrix from raw resolution to model input.
// TODO(zhongying): replace the hard code with the adaptive matrix.
static const std::vector<float> flattened_trans_mat = {4.f, 0.f, 0.f, 0.f, 4.f,
                                                       0.f, 0.f, 0.f, 1.f};
}  // namespace

Mono3DAdapter::Mono3DAdapter(const NetParam& net_param)
    : mono3d_detector_(std::make_unique<Mono3DDetector>(net_param)) {}

MultiCameraMono3dMeasurementsProto Mono3DAdapter::GetMono3dMeasurements(
    const CameraParams& camera_param, const CameraImage& image) {
  return GetMono3dMeasurements(image.camera_id(), camera_param, image.ToMat(),
                               image.width(), image.height(), image.pose(),
                               image.center_timestamp());
}

MultiCameraMono3dMeasurementsProto Mono3DAdapter::GetMono3dMeasurements(
    CameraId camera_id, const CameraParams& camera_param, const cv::Mat& image,
    int width, int height, const VehiclePose& pose, double image_timestamp) {
  SCOPED_QTRACE_ARG1("Mono3DAdapter::GetCamera3dMeasurements", "camera_id",
                     CameraId_Name(camera_id));

  // Compute out ex/in-trinsics.
  const auto camera_to_vehicle_transform =
      camera_param.camera_to_vehicle_extrinsics().ToTransform();
  const auto raw_camera_matrix = camera_param.camera_matrix();

  // Resize-crop the input and adjust intrinsics accordingly.
  // NOTE(zhongying): the current supported resolution is 1024 x 512.
  // TODO(zhongying): move this into vision_module.cc when pinned memory ready.
  auto camera_matrix = raw_camera_matrix;
  const float image_height_width_ratio = static_cast<float>(height) / width;

  // Cropping and adjusting intrinsics if hw ratio does not align with original
  // one.
  constexpr float kConventionalHWRatio =
      static_cast<float>(Mono3D::kSupportMinImageHeight) /
      static_cast<float>(Mono3D::kSupportMinImageWidth);
  const float hwr_ratio = image_height_width_ratio / kConventionalHWRatio;
  int width_padding = 0;
  int height_padding = 0;
  float size_ratio = Mono3D::kSupportMinImageWidth / static_cast<float>(width);

  constexpr int kInputHeight = Mono3D::kSupportMinImageHeight;
  constexpr int kInputWidth = Mono3D::kSupportMinImageWidth;
  cv::Mat* padded_image;
  {
    absl::MutexLock lock(&padded_images_mutex_);
    padded_image = &padded_images_[camera_id];
  }
  if (padded_image->empty()) {
    *padded_image =
        cv::Mat(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(123, 116, 103));
  }

  {
    SCOPED_QTRACE("Mono3DAdapter::ResizeAndPadding");
    if (hwr_ratio > 1.1) {
      // Width smaller than expected.
      const int expected_width =
          RoundToInt(static_cast<float>(width) * kInputHeight / height);
      width_padding = (kInputWidth - expected_width) / 2;
      cv::resize(image,
                 (*padded_image)(
                     cv::Rect(width_padding, 0, expected_width, kInputHeight)),
                 {expected_width, kInputHeight});
      size_ratio = kInputHeight / static_cast<float>(height);
    } else if (hwr_ratio < 0.9) {
      // Height smaller than expected.
      const int expected_height =
          RoundToInt(static_cast<float>(height) * kInputWidth / width);
      height_padding = (kInputHeight - expected_height) / 2;
      cv::resize(image,
                 (*padded_image)(
                     cv::Rect(0, height_padding, kInputWidth, expected_height)),
                 {kInputWidth, expected_height});
      size_ratio = kInputWidth / static_cast<float>(width);
    } else {
      cv::resize(image, *padded_image, {kInputWidth, kInputHeight});
    }
  }

  // Resize and adjusting intrinsics if size disagree.
  camera_matrix.set_cx(camera_matrix.cx() * size_ratio + width_padding);
  camera_matrix.set_cy(camera_matrix.cy() * size_ratio + height_padding);
  camera_matrix.set_fx(camera_matrix.fx() * size_ratio);
  camera_matrix.set_fy(camera_matrix.fy() * size_ratio);

  const auto& vehicle_pose = pose;
  const auto& center_timestamp = image_timestamp;

  // Contruct the flattened camera intrinsics.
  // NOTE(zhongying): inverse op to support ONNX.
  Mat4f intrinsics = Mat4f::Identity();
  intrinsics(0, 0) = camera_matrix.fx();
  intrinsics(1, 1) = camera_matrix.fy();
  intrinsics(0, 2) = camera_matrix.cx();
  intrinsics(1, 2) = camera_matrix.cy();
  auto intrinsics_inverse = intrinsics.inverse();
  std::vector<float> flattened_intrinsics;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      flattened_intrinsics.emplace_back(intrinsics_inverse(i, j));
    }
  }

  std::vector<std::vector<Mono3D::Mono3DBox>> mono3d_output;
  {
    absl::MutexLock lock(&mono3d_detector_mutex_);
    mono3d_output = mono3d_detector_->DetectImages(
        {*padded_image}, {flattened_intrinsics}, {flattened_trans_mat});
  }

  // The only supported batch size is 1.
  QCHECK_EQ(mono3d_output.size(), 1);

  // TODO(zhongying): fix the output nan when fp32 => fp16 in tensorrt.
  std::vector<Mono3D::Mono3DBox> pre_bev_nms_mono3d_boxes;
  for (const auto& mono3d_box : mono3d_output[0]) {
    if (std::isnan(mono3d_box.z)) {
      continue;
    }
    pre_bev_nms_mono3d_boxes.push_back(mono3d_box);
  }

  // BEV Class-agnostic NMS
  std::vector<std::pair<Box2d, float>> bev_box2ds;
  bev_box2ds.reserve(pre_bev_nms_mono3d_boxes.size());
  for (const auto& mono3d_box : pre_bev_nms_mono3d_boxes) {
    bev_box2ds.push_back(mono3d_box.ToBevBoxScore());
  }
  const auto bev_existence_mask =
      net_util::NonMaxSuppressionWithMask(bev_box2ds, 0.2);
  std::vector<Mono3D::Mono3DBox> pre_img_nms_mono3d_boxes;
  pre_img_nms_mono3d_boxes.reserve(pre_bev_nms_mono3d_boxes.size());
  for (int i = 0; i < bev_box2ds.size(); ++i) {
    if (bev_existence_mask[i]) {
      pre_img_nms_mono3d_boxes.push_back(pre_bev_nms_mono3d_boxes[i]);
    }
  }

  // Image Class-agnostic NMS
  std::vector<std::pair<AABox2d, float>> img_box2ds;
  img_box2ds.reserve(pre_img_nms_mono3d_boxes.size());
  for (const auto& mono3d_box : pre_img_nms_mono3d_boxes) {
    img_box2ds.push_back(mono3d_box.ToImgBoxScore());
  }
  const auto img_existence_mask =
      net_util::NonMaxSuppressionWithMask(img_box2ds, 0.8);
  std::vector<Mono3D::Mono3DBox> mono3d_boxes;
  mono3d_boxes.reserve(pre_img_nms_mono3d_boxes.size());
  for (int i = 0; i < img_box2ds.size(); ++i) {
    if (img_existence_mask[i]) {
      mono3d_boxes.push_back(pre_img_nms_mono3d_boxes[i]);
    }
  }

  // Set visibility scores in image.
  SetVisibilityScores(&mono3d_boxes);

  // Multi-camera warpper is more flexible.
  MultiCameraMono3dMeasurementsProto multi_camera_mono3d_measurements;
  auto* single_camera_mono3d_measurements =
      multi_camera_mono3d_measurements.add_single_camera_mono3d_measurements();
  single_camera_mono3d_measurements->set_camera_id(camera_id);

  // Logic to update message.
  Vec3d pos;
  double yaw = 0.0;
  for (int i = 0; i < mono3d_boxes.size(); ++i) {
    std::tie(pos, yaw) = ConvertPointFromCameraToSmooth(
        mono3d_boxes[i], camera_to_vehicle_transform, vehicle_pose);

    const Mat3d pos_cov = EstimateCovFromCameraToSmooth(
        mono3d_boxes[i], camera_to_vehicle_transform, vehicle_pose);

    const auto projected_3d_bbox_vertices =
        GetProjected3dBBoxVertices(mono3d_boxes[i], raw_camera_matrix);

    // Get the 2d box on image.
    float center_2d_x = mono3d_boxes[i].center_2d_x;
    float center_2d_y = mono3d_boxes[i].center_2d_y;
    float width_2d = mono3d_boxes[i].width_2d;
    float height_2d = mono3d_boxes[i].height_2d;
    center_2d_x = (center_2d_x - width_padding) / size_ratio;
    center_2d_y = (center_2d_y - height_padding) / size_ratio;
    width_2d = width_2d / size_ratio;
    height_2d = height_2d / size_ratio;
    int min_x = static_cast<int>(center_2d_x - width_2d / 2);
    int min_y = static_cast<int>(center_2d_y - height_2d / 2);
    int max_x = static_cast<int>(center_2d_x + width_2d / 2);
    int max_y = static_cast<int>(center_2d_y + height_2d / 2);
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    max_x = std::min(width, max_x);
    max_y = std::min(height, max_y);

    // Update the proto.
    auto* measurement = single_camera_mono3d_measurements->add_measurements();
    auto* bbox_2d = measurement->mutable_bbox_2d();
    bbox_2d->set_x(min_x);
    bbox_2d->set_y(min_y);
    bbox_2d->set_width(max_x - min_x);
    bbox_2d->set_height(max_y - min_y);
    for (int i = 0; i < projected_3d_bbox_vertices.size(); ++i) {
      auto* vertex = measurement->add_projected_3d_bbox_vertices();
      vertex->set_x(projected_3d_bbox_vertices[i].x());
      vertex->set_y(projected_3d_bbox_vertices[i].y());
    }
    auto* pos_proto = measurement->mutable_pos();
    pos_proto->set_x(pos.x());
    pos_proto->set_y(pos.y());
    pos_proto->set_z(pos.z());
    measurement->set_heading(yaw);
    measurement->set_width(mono3d_boxes[i].width);
    measurement->set_length(mono3d_boxes[i].length);
    measurement->set_height(mono3d_boxes[i].height);
    measurement->set_camera_id(camera_id);
    measurement->set_timestamp(center_timestamp);
    measurement->set_type(cls_type_[mono3d_boxes[i].label]);
    measurement->set_existence_confidence(mono3d_boxes[i].score);
    measurement->set_visibility_score(mono3d_boxes[i].visibility_score);
    measurement->set_mono3d_depth(mono3d_boxes[i].z);
    measurement->mutable_vehicle_pose()->set_x(vehicle_pose.x);
    measurement->mutable_vehicle_pose()->set_y(vehicle_pose.y);
    measurement->mutable_vehicle_pose()->set_yaw(vehicle_pose.yaw);
    Mat3dToProto(pos_cov, measurement->mutable_pos_covariance());
  }
  return multi_camera_mono3d_measurements;
}

Mat3d Mono3DAdapter::EstimateCovFromCameraToSmooth(
    const Mono3D::Mono3DBox& mono3d_box,
    const AffineTransformation& camera_to_vehicle_transform,
    const VehiclePose& vehicle_pose) const {
  // From kitti to qcraft.
  const Vec3d center = {mono3d_box.z, -mono3d_box.x, -mono3d_box.y};
  Mat3d pos_cov_under_depth = Mat3d::Zero();
  // Assume that 10% depth error corresponds to 3 \sigma.
  double depth_error_percentage = 0.1;
  if (cls_type_[mono3d_box.label] == MeasurementType::MT_CYCLIST) {
    depth_error_percentage = 0.2;
  }
  // Assume that angle error is 0.5 degree.
  constexpr double kCameraAngularResolution = d2r(0.5);  // rad
  // Set a minimal pos noise along ridial/depth direction.
  double min_camera_radial_noise_std = 1.0;  // m
  // Set a minimal pos noise along tangential direction.
  double min_camera_tangential_noise_std = 0.5;  // m
  // If the vehicle is lorry, double the noise.
  constexpr double kLengthToDetermineLorry = 8.0;  // m
  if (mono3d_box.length > kLengthToDetermineLorry) {
    min_camera_radial_noise_std = 2.0;
    min_camera_tangential_noise_std = 1.0;
  }

  // Assume that the height error is constant.
  constexpr double kHeightError = 0.3;

  // Compute noise along radial/depth direction.
  const double depth_error_std =
      std::max(center.block<2, 1>(0, 0).norm() * depth_error_percentage / 3.0,
               min_camera_radial_noise_std);  // m
  pos_cov_under_depth(0, 0) = Sqr(depth_error_std);
  // Compute noise along tangential direction.
  const double tangential_error_std =
      std::max(center.block<2, 1>(0, 0).norm() * kCameraAngularResolution,
               min_camera_tangential_noise_std);  // m
  pos_cov_under_depth(1, 1) = Sqr(tangential_error_std);
  // Compute niose along height direction.
  pos_cov_under_depth(2, 2) = Sqr(kHeightError);

  // Depth to camera.
  const double yaw = atan2(center.y(), center.x());
  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);
  Mat3d rot_depth_to_camera = Mat3d::Zero();
  rot_depth_to_camera << cos_yaw, -sin_yaw, 0, sin_yaw, cos_yaw, 0, 0, 0, 1;
  const Mat3d pos_cov_under_camera = rot_depth_to_camera * pos_cov_under_depth *
                                     rot_depth_to_camera.transpose();

  // Camera to smooth.
  const auto vehicle_to_smooth_transform = vehicle_pose.ToTransform();
  const auto camera_to_smooth_transform =
      (vehicle_to_smooth_transform * camera_to_vehicle_transform).mat();
  return camera_to_smooth_transform.block<3, 3>(0, 0) * pos_cov_under_camera *
         camera_to_smooth_transform.block<3, 3>(0, 0).transpose();
}

std::pair<Vec3d, double> Mono3DAdapter::ConvertPointFromCameraToSmooth(
    const Mono3D::Mono3DBox& mono3d_box,
    const AffineTransformation& camera_to_vehicle_transform,
    const VehiclePose& vehicle_pose) const {
  // Get the transform from camera to smooth.
  const auto vehicle_to_smooth_transform = vehicle_pose.ToTransform();
  const auto camera_to_smooth_transform =
      vehicle_to_smooth_transform * camera_to_vehicle_transform;

  // From kitti to qcraft.
  const Vec3d center = {mono3d_box.z, -mono3d_box.x, -mono3d_box.y};
  double heading = -(mono3d_box.heading + M_PI / 2);

  const double camera_to_vehicle_yaw =
      camera_to_vehicle_transform.GetRotationYawPitchRoll().x();
  double heading_smooth = heading + camera_to_vehicle_yaw + vehicle_pose.yaw;
  heading_smooth = NormalizeAngle(heading_smooth);

  return std::make_pair(camera_to_smooth_transform.TransformPoint(center),
                        heading_smooth);
}

std::vector<Vec2i> Mono3DAdapter::GetProjected3dBBoxVertices(
    const Mono3D::Mono3DBox& mono3d_box,
    const CameraIntrinsicsMatrix& camera_matrix) {
  // Coordinates in camera.
  //
  //                   z front (yaw=-0.5*pi)
  //                  /
  //                 /
  //                0 ------> x right (yaw=0)
  //                |
  //                |
  //                v
  //           down y
  //
  // (x0y0z0, x0y0z1, x0y1z1, x0y1z0, x1y0z0, x1y0z1, x1y1z1, x1y1z0)
  //
  //                 front z
  //                      /
  //                     /
  //       (x0, y0, z1) + -----------  + (x1, y0, z1)
  //                   /|            / |
  //                  / |           /  |
  //    (x0, y0, z0) + ----------- +   + (x1, y1, z1)
  //                 |  /      .   |  /
  //                 | / origin    | /
  //    (x0, y1, z0) + ----------- + -------> x right
  //                 |             (x1, y1, z0)
  //                 |
  //                 v
  //            down y
  //

  // Construct the Y-axis rotation matrix;
  const auto rot_cos = std::cos(mono3d_box.heading);
  const auto rot_sin = std::sin(mono3d_box.heading);
  Mat3d rot_mat = Mat3d::Identity();
  rot_mat(0, 0) = rot_cos;
  rot_mat(0, 2) = rot_sin;
  rot_mat(2, 0) = -rot_sin;
  rot_mat(2, 2) = rot_cos;

  // Construct the camera intrinsics matrix.
  Mat3d intrinsics = Mat3d::Identity();
  intrinsics(0, 0) = camera_matrix.fx();
  intrinsics(1, 1) = camera_matrix.fy();
  intrinsics(0, 2) = camera_matrix.cx();
  intrinsics(1, 2) = camera_matrix.cy();

  // Denorm => Rotate => Project
  std::vector<Vec2i> projected_3d_bbox_vertices;
  Vec3d locs = {mono3d_box.x, mono3d_box.y + mono3d_box.height / 2,
                mono3d_box.z};
  Vec3d dims = {mono3d_box.length, mono3d_box.height, mono3d_box.width};
  for (int i = 0; i < corners_norm.size(); ++i) {
    auto corner_3d = rot_mat * (corners_norm[i].cwiseProduct(dims)) + locs;
    auto corner_pj = intrinsics * corner_3d;
    projected_3d_bbox_vertices.emplace_back(
        Vec2i{static_cast<int>(corner_pj.x() / corner_pj.z()),
              static_cast<int>(corner_pj.y() / corner_pj.z())});
  }
  return projected_3d_bbox_vertices;
}

void Mono3DAdapter::SetVisibilityScores(
    std::vector<Mono3D::Mono3DBox>* mono3d_boxes) {
  SCOPED_QTRACE("Mono3DAdapter::SetVisibilityScores");
  std::vector<float> depths;
  depths.reserve(mono3d_boxes->size());
  for (const auto& mono3d_box : *mono3d_boxes) {
    depths.push_back(mono3d_box.z);
  }
  std::vector<int> sorted_indices(depths.size());
  iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&depths](int i, int j) { return depths[i] < depths[j]; });
  depth_block_mask_.setZero();
  for (const int& index : sorted_indices) {
    const auto box = std::get<AABox2d>((*mono3d_boxes)[index].ToImgBoxScore());
    int min_x = FloorToInt(box.min_x() / kDepthBlockDownSampleRate);
    int max_x = CeilToInt(box.max_x() / kDepthBlockDownSampleRate);
    int min_y = FloorToInt(box.min_y() / kDepthBlockDownSampleRate);
    int max_y = CeilToInt(box.max_y() / kDepthBlockDownSampleRate);
    min_x = std::max(0, min_x);
    max_x = std::min(kDepthBlockMaskWidth - 1, max_x);
    min_y = std::max(0, min_y);
    max_y = std::min(kDepthBlockMaskHeight - 1, max_y);
    int total_area = 0;
    int visibility_area = 0;
    for (int i = min_y; i <= max_y; ++i) {
      for (int j = min_x; j <= max_x; ++j) {
        ++total_area;
        if (depth_block_mask_(i, j) == 0) {
          ++visibility_area;
          depth_block_mask_(i, j) = 1;
        }
      }
    }
    const float visibility_score =
        static_cast<float>(visibility_area) /
        (total_area + std::numeric_limits<float>::epsilon());
    (*mono3d_boxes)[index].visibility_score = visibility_score;
  }
}

}  // namespace qcraft
