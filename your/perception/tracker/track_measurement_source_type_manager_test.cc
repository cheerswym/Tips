#include "onboard/perception/tracker/track_measurement_source_type_manager.h"

#include <string>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/global/run_context.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/test_util.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/perception/test_util/track_builder.h"

DECLARE_string(q_run_context);
namespace qcraft::tracker::track_measurement_source_type_manager {

using TMST = TrackMeasurementSourceType;
using TMSTM =
    track_measurement_source_type_manager::TrackMeasurementSourceTypeManager;

TEST(TrackMeasurementSourceTypeManager, ComputeTrackMeasurementSourceTypeTest) {
  Track<TrackState> track;
  // Construct measurement.
  const auto m_laser = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             LaserMeasurementBuilder().Build());
  const auto m_radar = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             RadarMeasurementBuilder().Build());
  const auto m_camera = BuildMeasurementProto(
      0.1, MT_VEHICLE, MTS_LL_NET, Camera3dMeasurementBuilder().Build());

  // Test LO to LO, LR, LV trans.
  track.track_state.measurement_source_type = TMST::TMST_LO;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LO);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_LR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_LV);

  // Test VO to LV, VR, VO trans.
  track.track_state.measurement_source_type = TMST::TMST_VO;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LV);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_VR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_VO);

  // Test RO to LR, VR, RO trans.
  track.track_state.measurement_source_type = TMST::TMST_RO;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_RO);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_VR);

  // Test LV to LV, LVR trans.
  track.track_state.measurement_source_type = TMST::TMST_LV;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LV);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_LVR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_LV);

  // Test LR to LR, LVR trans.
  track.track_state.measurement_source_type = TMST::TMST_LR;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_LR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_LVR);

  // Test VR to LVR, VR trans.
  track.track_state.measurement_source_type = TMST::TMST_VR;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LVR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_VR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_VR);

  // Test LVR to LVR trans.
  track.track_state.measurement_source_type = TMST::TMST_LVR;
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_laser),
            TMST::TMST_LVR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_radar),
            TMST::TMST_LVR);
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceType(track, m_camera),
            TMST::TMST_LVR);
}
TEST(TrackMeasurementSourceTypeManager, UseMeasurementHistoryTest) {
  Track<TrackState> track;
  // Construct measurement.
  const auto m_laser = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             LaserMeasurementBuilder().Build());
  const auto m_radar = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             RadarMeasurementBuilder().Build());
  const auto m_camera = BuildMeasurementProto(
      0.1, MT_VEHICLE, MTS_LL_NET, Camera3dMeasurementBuilder().Build());

  // Test LO.
  constexpr double kMaxMeasurementHistoryBufferLength = 1.2;  // 1.2s
  for (int i = 0; i < 1; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_laser, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_LO);
  // Test LR.
  for (int i = 1; i < 2; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_radar, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_LR);
  // Test LVR
  for (int i = 2; i < 5; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_camera, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_LVR);
  // Test VO.
  for (int i = 5; i < 10; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_camera, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_VO);
  // Test RO
  for (int i = 10; i < 15; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_radar, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_RO);
  // Test VR.
  for (int i = 15; i < 20; ++i) {
    const double timestamp = i * 0.1;
    track.measurement_history.PushBackAndClearStale(
        timestamp, &m_camera, kMaxMeasurementHistoryBufferLength);
    track.track_state.state_timestamp = timestamp;
  }
  EXPECT_EQ(TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track),
            TMST::TMST_VR);
}
TEST(TrackMeasurementSourceTypeManager, HasLidarMeasurementsTest) {
  Track<TrackState> track;
  // Test LO
  track.track_state.measurement_source_type = TMST::TMST_LO;
  EXPECT_TRUE(TMSTM::HasLidarMeasurements(track));

  // Test VO
  track.track_state.measurement_source_type = TMST::TMST_VO;
  EXPECT_FALSE(TMSTM::HasLidarMeasurements(track));

  // Test RO
  track.track_state.measurement_source_type = TMST::TMST_RO;
  EXPECT_FALSE(TMSTM::HasLidarMeasurements(track));

  // Test LV
  track.track_state.measurement_source_type = TMST::TMST_LV;
  EXPECT_TRUE(TMSTM::HasLidarMeasurements(track));
  // Test LR
  track.track_state.measurement_source_type = TMST::TMST_LR;
  EXPECT_TRUE(TMSTM::HasLidarMeasurements(track));
  // Test VR
  track.track_state.measurement_source_type = TMST::TMST_VR;
  EXPECT_FALSE(TMSTM::HasLidarMeasurements(track));
  // Test LVR
  track.track_state.measurement_source_type = TMST::TMST_LVR;
  EXPECT_TRUE(TMSTM::HasLidarMeasurements(track));
}
TEST(TrackMeasurementSourceTypeManager, ShouldCreateTrackFromRadarTest) {
  auto m_radar = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                       RadarMeasurementBuilder().Build());

  const auto m_laser = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             LaserMeasurementBuilder().Build());
  VehiclePose pose(100.0, 100.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(TMSTM::ShouldCreateTrackFromRadar(m_laser, pose, false));
  EXPECT_FALSE(TMSTM::ShouldCreateTrackFromRadar(m_radar, pose, true));

  m_radar.mutable_radar_measurement()->mutable_body_pos()->set_x(-100.0);
  m_radar.mutable_radar_measurement()->mutable_body_pos()->set_y(.0);
  EXPECT_FALSE(TMSTM::ShouldCreateTrackFromRadar(m_radar, pose, false));

  m_radar.mutable_radar_measurement()->mutable_body_pos()->set_x(300.0);
  m_radar.mutable_radar_measurement()->mutable_body_pos()->set_y(.0);
  m_radar.mutable_radar_measurement()->mutable_vel()->set_x(20.0);
  m_radar.mutable_radar_measurement()->mutable_vel()->set_y(.0);
  EXPECT_TRUE(TMSTM::ShouldCreateTrackFromRadar(m_radar, pose, false));
}
TEST(TrackMeasurementSourceTypeManager, ShouldCreateTrackFromVisionTest) {
  RunParamsProto run_params_proto;
  const std::string param_file_path =
      "onboard/perception/tracker/test_data/test_car_param.pb.txt";
  QCHECK(file_util::TextFileToProto(param_file_path, &run_params_proto));
  const auto params_proto_v2 = RunParamsProtoV2::construct(run_params_proto);
  TMSTM tmstm(params_proto_v2);
  FLAGS_enable_context_test = 1;
  FLAGS_q_run_context = "pbqv1";
  VehiclePose pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto pose_inv = pose.ToTransform().Inverse();
  const auto m_laser = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                             LaserMeasurementBuilder().Build());
  auto m_camera = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                        Camera3dMeasurementBuilder().Build());

  EXPECT_FALSE(
      tmstm.ShouldCreateTrackFromVision(m_laser, pose, pose_inv, false));
  // Test in M1 view field.
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_x(50.0);
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_y(.0);
  EXPECT_FALSE(
      tmstm.ShouldCreateTrackFromVision(m_camera, pose, pose_inv, false));

  // Test beyond detection range.
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_x(250.0);
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_y(.0);
  EXPECT_TRUE(
      tmstm.ShouldCreateTrackFromVision(m_camera, pose, pose_inv, false));

  // Test out of M1 view field.
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_x(50.0);
  m_camera.mutable_camera3d_measurement()->mutable_pos()->set_y(100.0);
  EXPECT_TRUE(
      tmstm.ShouldCreateTrackFromVision(m_camera, pose, pose_inv, false));
}

}  // namespace qcraft::tracker::track_measurement_source_type_manager
