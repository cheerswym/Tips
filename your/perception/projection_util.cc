#include "onboard/perception/projection_util.h"

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"

namespace qcraft::projection_util {

double FindRefLidarMidTimestamp(const CameraParams& camera_param,
                                const std::vector<LidarFrame>& lidar_frames) {
  if (camera_param.has_ref_lidar()) {
    for (const auto& lidar_frame : lidar_frames) {
      if (lidar_frame.lidar_id() == camera_param.ref_lidar()) {
        return lidar_frame.MidTimestamp();
      }
    }
  }
  return std::accumulate(lidar_frames.begin(), lidar_frames.end(), 0.0,
                         [](double sum, const LidarFrame& lidar_frame) {
                           return sum + lidar_frame.MidTimestamp();
                         }) /
         lidar_frames.size();
}

AffineTransformation GetSmoothToCameraCoordTransform(
    const CameraParams& camera_param,
    const LidarParametersProto& ref_lidar_params,
    const AffineTransformation& pose_transform) {
  const auto camera_to_lidar =
      camera_param.camera_to_ref_lidar_extrinsics().ToTransform();
  const auto& lidar_to_vehicle = AffineTransformation::FromExtrinsics(
      ref_lidar_params.installation().extrinsics());
  AffineTransformation point_to_camera = camera_to_lidar.Inverse() *
                                         lidar_to_vehicle.Inverse() *
                                         pose_transform.Inverse();
  return point_to_camera;
}

AffineTransformation GetSmoothToCameraCoordTransform(
    const CameraParams& camera_param,
    const LidarParametersProto& ref_lidar_params, const VehiclePose& pose) {
  return GetSmoothToCameraCoordTransform(camera_param, ref_lidar_params,
                                         pose.ToTransform());
}

std::optional<Vec2i> SmoothPointToImagePos(
    const Vec3d& point, const AffineTransformation& smooth_to_camera_transform,
    const CameraParams& camera_params, bool out_of_image_coord) {
  // Convert point from smooth to camera frame.
  const Vec3d point_in_camera_frame =
      smooth_to_camera_transform.TransformPoint(point);
  return CameraPointToImagePos(point_in_camera_frame, camera_params,
                               out_of_image_coord);
}

std::optional<Vec2i> CameraPointToImagePos(const Vec3d& point,
                                           const CameraParams& camera_params,
                                           bool out_of_image_coord) {
  const auto& camera_matrix = camera_params.camera_matrix();
  if (point.x() > 0) {
    const double cx = camera_matrix.cx();
    const double cy = camera_matrix.cy();
    const double fx = camera_matrix.fx();
    const double fy = camera_matrix.fy();
    const double point_x_inv = 1.0 / point.x();
    const int x = FloorToInt(-point.y() * point_x_inv * fx + cx);
    const int y = FloorToInt(-point.z() * point_x_inv * fy + cy);
    if (out_of_image_coord ||
        (!out_of_image_coord && x >= 0 && x < camera_params.width() && y >= 0 &&
         y < camera_params.height())) {
      return Vec2i(x, y);
    }
  }
  return std::nullopt;
}

Vec3d ImagePosToVehiclePoint(const Vec2i& pos,
                             const CameraParams& camera_params) {
  // Under flat-world assumption, find 3D posiiton of a pixel position in image
  // assume it falls on vehicle plane
  const auto& camera_matrix = camera_params.camera_matrix();
  // pseudo_y and pseudo_z are lateral axis and vertical axis coordinates in
  // camera frame, with left and up as positive direction
  const double pseudo_y = (camera_matrix.cx() - pos.x()) / camera_matrix.fx();
  const double pseudo_z = (camera_matrix.cy() - pos.y()) / camera_matrix.fy();
  const Mat4d& extrinsic_matrix =
      camera_params.camera_to_vehicle_extrinsics_matrix_inv();
  const Mat2d& A =
      extrinsic_matrix.block<2, 2>(1, 0) -
      Vec2d(pseudo_y, pseudo_z) * extrinsic_matrix.block<1, 2>(0, 0);
  const Vec2d b(pseudo_y * extrinsic_matrix(0, 3) - extrinsic_matrix(1, 3),
                pseudo_z * extrinsic_matrix(0, 3) - extrinsic_matrix(2, 3));
  const double det_A_inv = 1.0 / (A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0));
  return Vec3d(det_A_inv * (A(1, 1) * b.x() - A(0, 1) * b.y()),
               det_A_inv * (A(0, 0) * b.y() - A(1, 0) * b.x()), 0.0);
}

Vec3d GetLidarViewPointInVehicleCoord(
    const std::unordered_map<LidarId, LidarParametersProto>& lidar_params) {
  // Find a 3d position for lidar sensors. Note that there might be multiple
  // lidar sensor so we use different logics in different scenarios.
  // 1. Use the position if LDR_CENTER presents.
  // 2. Use the position if LDR_FRONT presents.
  // 3. Use the mid point if LDR_FRONT_LEFT and LDR_FRONT_RIGHT presents.
  // 4. Use the first lidar otherwise.

  QCHECK(!lidar_params.empty());
  if (const auto* param = FindOrNull(lidar_params, LDR_CENTER)) {
    const auto& lidar_extrinsics = param->installation().extrinsics();
    return {lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()};
  } else if (const auto* param = FindOrNull(lidar_params, LDR_FRONT)) {
    const auto& lidar_extrinsics = param->installation().extrinsics();
    return {lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()};
  } else if (const auto* fl_param = FindOrNull(lidar_params, LDR_FRONT_LEFT),
             *fr_param = FindOrNull(lidar_params, LDR_FRONT_RIGHT);
             fl_param != nullptr && fr_param != nullptr) {
    const auto& left_lidar_extrinsics = fl_param->installation().extrinsics();
    const Vec3d left_pos = {left_lidar_extrinsics.x(),
                            left_lidar_extrinsics.y(),
                            left_lidar_extrinsics.z()};
    const auto& right_lidar_extrinsics = fr_param->installation().extrinsics();
    const Vec3d right_pos = {right_lidar_extrinsics.x(),
                             right_lidar_extrinsics.y(),
                             right_lidar_extrinsics.z()};
    return (left_pos + right_pos) * 0.5;
  } else {
    const auto& lidar_extrinsics =
        lidar_params.begin()->second.installation().extrinsics();
    return {lidar_extrinsics.x(), lidar_extrinsics.y(), lidar_extrinsics.z()};
  }
}

std::optional<AffineTransformation> FindClosestLocalToCameraTransformation(
    const boost::circular_buffer<CameraImageWithTransform>&
        camera_image_with_t_buffer,
    const double target_center_timestamp) {
  // Binary search to find transformation.
  auto it = std::lower_bound(
      camera_image_with_t_buffer.begin(), camera_image_with_t_buffer.end(),
      target_center_timestamp,
      [](const CameraImageWithTransform& camera_image_with_t,
         const double target_center_timestamp) {
        return std::get<CameraImage>(camera_image_with_t).center_timestamp() <
               target_center_timestamp;
      });
  constexpr double kDoubleMinDiff = 1e-6;
  if (it == camera_image_with_t_buffer.end() ||
      std::abs(std::get<CameraImage>(*it).center_timestamp() -
               target_center_timestamp) > kDoubleMinDiff) {
    return std::nullopt;
  } else {
    return std::get<AffineTransformation>(*it);
  }
}

}  // namespace qcraft::projection_util
