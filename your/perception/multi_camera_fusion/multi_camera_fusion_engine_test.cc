#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine.h"

#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine_friend.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/perception/test_util/track_builder.h"
#include "onboard/utils/file_util.h"

namespace qcraft::multi_camera_fusion {

const std::set<CameraId> cameras_for_mono3d = {
    CAM_PBQ_FRONT_LEFT, CAM_PBQ_FRONT_RIGHT, CAM_PBQ_REAR_LEFT,
    CAM_PBQ_REAR_RIGHT, CAM_PBQ_REAR};

TEST(MultiCameraFusionEngine, TrackCameraObjectsTest) {
  RunParamsProto run_params_proto;
  const std::string param_file_path =
      "onboard/perception/multi_camera_fusion/test_data/test_car_param.pb.txt";
  QCHECK(file_util::TextFileToProto(param_file_path, &run_params_proto));
  const auto params_proto_v2 = RunParamsProtoV2::construct(run_params_proto);
  ThreadPool multi_camera_fusion_thread_pool(1);
  SemanticMapManager semantic_map_manager;
  MultiCameraFusionEngine multi_camera_fusion_engine = MultiCameraFusionEngine(
      nullptr, &multi_camera_fusion_thread_pool, &semantic_map_manager,
      cameras_for_mono3d, params_proto_v2);
  MultiCameraFusionEngineFriend multi_camera_fusion_engine_friend(
      &multi_camera_fusion_engine);
  // Construct camera measruements.
  for (const auto cam_id : cameras_for_mono3d) {
    auto m_camera = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                          Camera3dMeasurementBuilder().Build());

    std::shared_ptr<MeasurementsProto> measurements_proto =
        std::make_shared<MeasurementsProto>();
    *measurements_proto->add_measurements() = m_camera;

    measurements_proto->set_min_timestamp(0.1);
    measurements_proto->set_max_timestamp(0.1);
    measurements_proto->set_group_type(MeasurementsProto::CAMERA);

    multi_camera_fusion_engine_friend.TrackSingleCameraObjects(
        cam_id, VehiclePose(), 0.1, measurements_proto);
  }

  // Single camera track test.
  for (const auto cam_id : cameras_for_mono3d) {
    const auto& tracks =
        multi_camera_fusion_engine_friend.SingleCameraTracks(cam_id);
    EXPECT_EQ(tracks.size(), 1);
  }

  // Multi camera track test.
  std::vector<CameraId> processed_cameras;

  std::shared_ptr<MeasurementsProto> output_measurements =
      std::make_shared<MeasurementsProto>();
  for (const auto cam_id : cameras_for_mono3d) {
    processed_cameras.push_back(cam_id);
  }
  multi_camera_fusion_engine_friend.FuseMultiCameraTracks(
      processed_cameras, 0.1, {}, output_measurements.get());
}

// TODO(zheng): Add more test by using true scenario.

}  // namespace qcraft::multi_camera_fusion
