#include "onboard/perception/multi_camera_fusion/tracker_util.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/math/geometry/util.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/perception/test_util/track_builder.h"
#include "onboard/utils/file_util.h"

namespace qcraft::multi_camera_fusion {
TEST(TrackerUtilTest, TrackToMeasurementTest) {
  tracker::Track<MultiCameraTrackState> track;
  // Construct measurement.
  auto m_camera = BuildMeasurementProto(0.1, MT_VEHICLE, MTS_LL_NET,
                                        Camera3dMeasurementBuilder().Build());
  auto* camera3d_m = m_camera.mutable_camera3d_measurement();
  Vec3dToProto(Vec3d(10.0, 10.0, 2.0), camera3d_m->mutable_pos());
  camera3d_m->set_heading(M_PI);
  auto* m_bbox_2d = camera3d_m->mutable_bbox_2d();
  m_bbox_2d->set_x(50.0);
  m_bbox_2d->set_y(50.0);
  m_bbox_2d->set_width(6.0);
  m_bbox_2d->set_height(6.0);
  track.measurement_history.PushBackAndClearStale(0.1, &m_camera, 1.0);

  // Construct track state.
  track.track_state.state_timestamp = 1.0;
  track.track_state.type = MT_VEHICLE;
  track.track_state.camera_id = CAM_FRONT_LEFT;
  track.track_state.id = 1;
  track.track_state.first_timestamp = 0.1;
  track.track_state.last_timestamp = 0.1;
  track.track_state.state_timestamp = 0.1;

  tracker::CarState state;
  state.x() = 10.0;
  state.y() = 10.0;
  state.yaw() = M_PI;
  state.vel() = 1.0;

  MotionFilterParamProto param3d;
  const std::string param3d_path =
      "onboard/perception/multi_camera_fusion/test_data/car_model.pb.txt";

  QCHECK(file_util::TextFileToProto(param3d_path, &param3d));
  param3d.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{param3d};
  track.track_state.estimator_3d =
      tracker::Estimator(tracker::ToImmProto(v, {1.0}));
  track.track_state.estimator_3d.Init(tracker::StateData(state), 0.1);
  tracker::BBoxState bbox_meas;
  bbox_meas.length() = 5.0;
  bbox_meas.width() = 2.0;
  track.track_state.estimator_3d.InitExtent(bbox_meas);

  track.track_state.pos = Vec3d(10.0, 10.0, 2.0);
  track.track_state.heading = M_PI;
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/multi_camera_fusion/test_data/img_2d_model.pb.txt",
      &params));
  tracker::Img2DState img_2d_state;
  img_2d_state << 50.0, 50.0, 1.0, 6.0, 0.0, 0.0, 0.0, 0.0;
  tracker::Img2DKalmanFilter<tracker::Img2DModelCV> img_2d_kf(
      params, img_2d_state, 1.0);
  track.track_state.motion_filter_2d = img_2d_kf;
  // Test output value.
  MeasurementProto m = TrackToMeasurementProto(track);

  const auto& output_camera3d_m = m.camera3d_measurement();
  const auto& output_bbox_2d = output_camera3d_m.bbox_2d();

  // Test 2d info.
  EXPECT_DOUBLE_EQ(output_bbox_2d.x(), 50.0);
  EXPECT_DOUBLE_EQ(output_bbox_2d.y(), 50.0);
  EXPECT_DOUBLE_EQ(output_bbox_2d.width(), 6.0);
  EXPECT_DOUBLE_EQ(output_bbox_2d.height(), 6.0);

  // Test 3d info.
  EXPECT_DOUBLE_EQ(output_camera3d_m.pos().x(), 10.0);
  EXPECT_DOUBLE_EQ(output_camera3d_m.pos().y(), 10.0);
  EXPECT_DOUBLE_EQ(output_camera3d_m.vel().x(), -1.0);
  EXPECT_DOUBLE_EQ(output_camera3d_m.heading(), M_PI);

  EXPECT_DOUBLE_EQ(output_camera3d_m.length(), 5.0);
  EXPECT_DOUBLE_EQ(output_camera3d_m.width(), 2.0);
  EXPECT_DOUBLE_EQ(output_camera3d_m.timestamp(), 0.1);
}
}  // namespace qcraft::multi_camera_fusion
