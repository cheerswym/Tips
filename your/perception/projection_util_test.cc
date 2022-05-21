#include "onboard/perception/projection_util.h"

#include <utility>

#include "gtest/gtest.h"
#include "onboard/params/utils/param_util.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {

constexpr char kCameraParamsStr[] = R"(
    camera_id: CAM_L_FRONT
    intrinsics {
      calibration_time: "2021-02-01 03:55:32"
      distort_coeffs {
        k1: -0.57424205759189
        k2: 0.31831740392514846
        p1: -0.0008512964244029186
        p2: 0.00062087693060185146
        k3: -0.043437049513538
      }
      camera_matrix {
        fx: 1965.3796218545585
        fy: 1964.846193822493
        cx: 950.56417472563055
        cy: 562.506912999145
      }
      serial_no: "H60-D03050562"
      calibration_engineer: "monica"
      rms: 0.36545475837100794
    }
    extrinsics {
      calibration_time: "2021-06-07 06:50:16"
      x: 0.077
      y: 0
      z: -0.097
      yaw: 0.0030019663134302492
      pitch: 0.0019373154697137074
      roll: -0.0032288591161895117
      calibration_engineer: "zhihui"
    }
    width: 1920
    height: 1080
    max_fps: 22
    expected_fps: 10
    mid_row_time_since_trigger: 0.07
    rolling_elapse_time: 0.033
    format {
      output_format: YUV
    }
    ref_lidar_id: LDR_FRONT_LEFT
    camera_info {
      camera_vendor: SENSING
      sensor_type: AR0231
    }
    device_path: "/dev/video0"
    hardware_trigger: true
  )";

constexpr char kLidarParamsStr[] = R"(
    extrinsics {
      x: 1.0733368396759033
      y: -0.039019051939249039
      z: 2.0887777805328369
      roll: 0.035022657364606857
      pitch: -0.037045884877443314
      yaw: -0.046073541
      calibration_time: "2021-07-16 20:39:40"
      calibration_engineer: "Meng"
    }
    serial_no: "PA6438C75D9238C75E"
  )";

CameraParams SetupCameraParams() {
  CameraParamsProto v1_camera;
  TextToProto(kCameraParamsStr, &v1_camera);
  LidarParamsProto v1_lidar;
  const auto v2_lidar = param_util::ConvertLidarParams(v1_lidar);
  CameraParams camera_params(param_util::ConvertCameraParams(v1_camera),
                             &v2_lidar);
  return camera_params;
}

CameraParams SetupCameraWithLidarRefParams() {
  CameraParamsProto v1_camera;
  TextToProto(kCameraParamsStr, &v1_camera);
  LidarParamsProto v1_lidar;
  TextToProto(kLidarParamsStr, &v1_lidar);
  const auto v2_lidar = param_util::ConvertLidarParams(v1_lidar);
  CameraParams camera_params(param_util::ConvertCameraParams(v1_camera),
                             &v2_lidar);
  return camera_params;
}

TEST(ProjectionUtilTest, SmoothPointToImagePosTestValidPoint) {
  const auto& camera_params = SetupCameraParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d lidar_point(100, 0, 0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      lidar_point, smooth_to_camera_transform, camera_params);
  EXPECT_TRUE(image_pos);
  // QCHECK_EQ(*image_pos, Vec2i(975, 612));
}

TEST(ProjectionUtilTest, SmoothPointToImagePosTestOutOfImagePointNoReturn) {
  const auto& camera_params = SetupCameraParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d lidar_point(100, -10000, 0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      lidar_point, smooth_to_camera_transform, camera_params);
  EXPECT_FALSE(image_pos);
}

TEST(ProjectionUtilTest, SmoothPointToImagePosTestOutOfImagePointReturn) {
  const auto& camera_params = SetupCameraParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d lidar_point(100, 10000, 0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      lidar_point, smooth_to_camera_transform, camera_params, true);
  EXPECT_TRUE(image_pos);
  // QCHECK_EQ(*image_pos, Vec2i(-217399, 612));
}

TEST(ProjectionUtilTest, SmoothPointToImagePosTestPointFromBehindNoReturn) {
  const auto& camera_params = SetupCameraParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d lidar_point(-100, 0, 0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      lidar_point, smooth_to_camera_transform, camera_params, true);
  EXPECT_FALSE(image_pos);
}

TEST(ProjectionUtilTest, SmoothPointToImagePosTestPointFromBehindReturn) {
  const auto& camera_params = SetupCameraParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d lidar_point(-100, 0, 0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      lidar_point, smooth_to_camera_transform, camera_params);
  EXPECT_FALSE(image_pos);
}

TEST(ProjectionUtilTest, FindClosestLocalToCameraTransformation) {
  boost::circular_buffer<CameraImageWithTransform> buffer(5);
  const auto& affine_trans0 =
      projection_util::FindClosestLocalToCameraTransformation(buffer, 0.0);
  QCHECK_EQ(affine_trans0.has_value(), false);
  for (int i = 0; i < 5; ++i) {
    ImageInfo image_info;
    image_info.set_trigger_timestamp(i * 0.1);
    image_info.set_exposure_time_ms(i * 0.1);
    std::unique_ptr<ShmMessage> shm_decoded_msg =
        CameraImageBuffer::MakeCameraImageOnShm(256, 256, "decoded_image", "");
    auto* decoded_image_meta =
        shm_decoded_msg->mutable_shm_msg_metadata()->MutableExtension(
            DecodedImageMetadata::decoded_image_meta);
    decoded_image_meta->set_camera_id(CameraId::CAM_L_FRONT);
    *decoded_image_meta->mutable_image_info() = std::move(image_info);
    CameraParametersProto param;
    auto* inherent = param.mutable_inherent();
    inherent->set_mid_row_time_since_trigger(0.0);
    CameraImage image(std::move(shm_decoded_msg), CameraParams(param, nullptr));
    buffer.push_back({std::move(image), AffineTransformation()});
  }

  const auto& affine_trans1 =
      projection_util::FindClosestLocalToCameraTransformation(buffer, 0.07);
  EXPECT_EQ(affine_trans1.has_value(), false);
  const auto& affine_trans2 =
      projection_util::FindClosestLocalToCameraTransformation(buffer, 0.00);
  EXPECT_EQ(affine_trans2.has_value(), true);
  const auto& affine_trans3 =
      projection_util::FindClosestLocalToCameraTransformation(buffer, 0.15);
  EXPECT_EQ(affine_trans3.has_value(), false);
}

TEST(ProjectionUtilTest, ImagePosToVehiclePoint) {
  const auto& camera_params = SetupCameraWithLidarRefParams();
  const auto smooth_to_camera_transform =
      camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
  const Vec3d point_3d(30.0, -2.0, 0.0);
  const auto image_pos = projection_util::SmoothPointToImagePos(
      point_3d, smooth_to_camera_transform, camera_params);
  EXPECT_TRUE(image_pos);
  const Vec3d projected_point =
      projection_util::ImagePosToVehiclePoint(image_pos.value(), camera_params);
  EXPECT_LT(projected_point.DistanceSquareTo(point_3d), 0.1);
}

}  // namespace qcraft
