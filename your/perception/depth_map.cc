#include "onboard/perception/depth_map.h"

#include "glog/logging.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/params/utils/param_util.h"
#include "onboard/perception/projection_util.h"

namespace qcraft {

void ComputeDepthMap(const LidarFrame& lidar_frame,
                     const LidarExtrinsicsProto& lidar_extrinsics,
                     const CameraParams& camera_params,
                     const VehiclePose& image_top_pose,
                     const VehiclePose& image_bottom_pose,
                     DepthMapProto* depth_map_proto) {
  QCHECK_NOTNULL(depth_map_proto);

  const auto camera_matrix = camera_params.camera_params_proto()
                                 .inherent()
                                 .intrinsics()
                                 .camera_matrix();
  const auto& camera_to_vehicle_extrinsics =
      camera_params.camera_to_vehicle_extrinsics();
  // Compute the horizontal FOV with a buffer of 30 degrees.
  constexpr double kCameraHFovBuffer = d2r(30.);  // 30 degrees.
  const double hfov_half =
      fast_math::Atan2(camera_matrix.cx(), camera_matrix.fx()) +
      kCameraHFovBuffer;

  const VehiclePose image_mid_pose(
      (image_top_pose.x + image_bottom_pose.x) * 0.5,
      (image_top_pose.y + image_bottom_pose.y) * 0.5,
      (image_top_pose.z + image_bottom_pose.z) * 0.5,
      (image_top_pose.yaw + image_bottom_pose.yaw) * 0.5,
      (image_top_pose.pitch + image_bottom_pose.pitch) * 0.5,
      (image_top_pose.roll + image_bottom_pose.roll) * 0.5);
  if (lidar_frame.is_spin()) {
    for (const auto& scan : *lidar_frame.spin()) {
      // Check if the scan match the given camera.
      const double scan_yaw =
          M_PI - d2r(scan.azimuth_in_degree) + lidar_extrinsics.yaw();
      const double yaw_diff =
          std::abs(NormalizeAngle(scan_yaw - camera_to_vehicle_extrinsics.yaw));
      if (yaw_diff > hfov_half) continue;
      if (yaw_diff < d2r(0.5)) {
        depth_map_proto->set_timestamp(scan.timestamp);
      }

      const auto smooth_to_camera_transform =
          (image_mid_pose.ToTransform() *
           camera_to_vehicle_extrinsics.ToTransform())
              .Inverse();
      for (const auto& shot : scan) {
        for (int i = 0; i < shot.num_returns; ++i) {
          const float range = shot.calibrated_returns[i].range;
          const auto image_pos = projection_util::SmoothPointToImagePos(
              shot.calibrated_returns[i].coord().cast<double>(),
              smooth_to_camera_transform, camera_params);
          if (image_pos) {
            // Correct pose.
            // Compute the pose when the image_pos->y()'th row is captured.
            const double ratio =
                LerpFactor<double>(0, camera_params.height(), image_pos->y());
            const VehiclePose new_pose(
                Lerp(image_top_pose.x, image_bottom_pose.x, ratio),
                Lerp(image_top_pose.y, image_bottom_pose.y, ratio),
                Lerp(image_top_pose.z, image_bottom_pose.z, ratio),
                Lerp(image_top_pose.yaw, image_bottom_pose.yaw, ratio),
                Lerp(image_top_pose.pitch, image_bottom_pose.pitch, ratio),
                Lerp(image_top_pose.roll, image_bottom_pose.roll, ratio));

            // Recompute the transformation.
            const auto smooth_to_camera_transform =
                (new_pose.ToTransform() *
                 camera_to_vehicle_extrinsics.ToTransform())
                    .Inverse();
            const auto image_pos = projection_util::SmoothPointToImagePos(
                shot.calibrated_returns[i].coord().cast<double>(),
                smooth_to_camera_transform, camera_params);
            if (image_pos) {
              auto* pixel = depth_map_proto->add_pixels();
              pixel->set_row(image_pos->y());
              pixel->set_col(image_pos->x());
              pixel->set_depth_cm(RoundToInt(range * 100.0f));
              pixel->set_intensity(shot.calibrated_returns[i].intensity);
            }
          }
        }
      }
    }
  } else if (lidar_frame.is_point_cloud()) {
    const auto smooth_to_camera_transform =
        (image_mid_pose.ToTransform() *
         camera_to_vehicle_extrinsics.ToTransform())
            .Inverse();
    for (int i = 0; i < lidar_frame.point_cloud()->num_points(); ++i) {
      const PointCloud::Point& p = lidar_frame.point_cloud()->point(i);
      const auto image_pos = projection_util::SmoothPointToImagePos(
          Vec3d(p.x, p.y, p.z), smooth_to_camera_transform, camera_params);
      if (image_pos) {
        const double ratio =
            LerpFactor<double>(0, camera_params.height(), image_pos->y());
        const VehiclePose new_pose(
            Lerp(image_top_pose.x, image_bottom_pose.x, ratio),
            Lerp(image_top_pose.y, image_bottom_pose.y, ratio),
            Lerp(image_top_pose.z, image_bottom_pose.z, ratio),
            Lerp(image_top_pose.yaw, image_bottom_pose.yaw, ratio),
            Lerp(image_top_pose.pitch, image_bottom_pose.pitch, ratio),
            Lerp(image_top_pose.roll, image_bottom_pose.roll, ratio));

        const auto smooth_to_camera_transform =
            (new_pose.ToTransform() *
             camera_to_vehicle_extrinsics.ToTransform())
                .Inverse();
        const auto image_pos = projection_util::SmoothPointToImagePos(
            Vec3d(p.x, p.y, p.z), smooth_to_camera_transform, camera_params);
        if (image_pos) {
          auto* pixel = depth_map_proto->add_pixels();
          pixel->set_row(image_pos->y());
          pixel->set_col(image_pos->x());
          pixel->set_depth_cm(p.range() * 100);
          pixel->set_intensity(p.intensity);
        }
      }
    }
  }
}

}  // namespace qcraft
