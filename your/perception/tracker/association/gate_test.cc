#include "onboard/perception/tracker/association/gate.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {
namespace association {

tracker::Estimator BuildCarModelEstimator() {
  // Create track.
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{params, params};

  // track1
  tracker::CarState state;
  state.x() = 0.0;
  state.y() = 0.0;
  state.vel() = 0.0;
  state.yaw() = 0.0;
  state.yawd() = 0.0;
  state.acc() = 0.0;
  tracker::Estimator estimator(tracker::ToImmProto(v, {0.5, 0.5}));
  estimator.Init(/*StateData*/ tracker::StateData(state), /*timestamp*/ 0.0);

  return estimator;
}

TEST(CircleGate, TestGating) {
  // DBQ
  Track<TrackState> track1, track2;
  track1.track_state.contour =
      Polygon2d({{3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}});
  track2.track_state.contour =
      Polygon2d({{10.0, 10.0}, {11.0, 10.0}, {11.0, 11.0}, {10.0, 11.0}});
  MeasurementProto measurement1;
  std::vector<Vec2d> measurement1_contour = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (size_t i = 0; i < measurement1_contour.size(); ++i) {
    measurement1.mutable_laser_measurement()->add_contour();
    measurement1.mutable_laser_measurement()->mutable_contour(i)->set_x(
        measurement1_contour[i].x());
    measurement1.mutable_laser_measurement()->mutable_contour(i)->set_y(
        measurement1_contour[i].y());
  }
  // Gate
  auto gate_ptr = BuildGateFromType<Track<TrackState>, MeasurementProto>(
      GatesProto::CIRCLE, {Sqr(7.0)});
  EXPECT_EQ(gate_ptr->Type(), GatesProto::CIRCLE);
  EXPECT_TRUE(gate_ptr->Gating(measurement1, track1).first);
  EXPECT_FALSE(gate_ptr->Gating(measurement1, track2).first);

  // PBQ not implemented.
}

TEST(TypeCircleGate, TestGating) {
  // DBQ
  Track<TrackState> track1, track2;
  track1.track_state.type = MT_PEDESTRIAN;
  track1.track_state.contour =
      Polygon2d({{3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}});
  track2.track_state.type = MT_VEHICLE;
  track2.track_state.contour =
      Polygon2d({{3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}});
  MeasurementProto measurement1;
  measurement1.set_type(MT_UNKNOWN);
  std::vector<Vec2d> measurement1_contour = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (size_t i = 0; i < measurement1_contour.size(); ++i) {
    measurement1.mutable_laser_measurement()->add_contour();
    measurement1.mutable_laser_measurement()->mutable_contour(i)->set_x(
        measurement1_contour[i].x());
    measurement1.mutable_laser_measurement()->mutable_contour(i)->set_y(
        measurement1_contour[i].y());
  }
  // Gate
  auto gate_ptr = BuildGateFromType<Track<TrackState>, MeasurementProto>(
      GatesProto::TYPE_CIRCLE, {Sqr(1.0), Sqr(7.0)});
  EXPECT_FALSE(gate_ptr->Gating(measurement1, track1).first);
  EXPECT_TRUE(gate_ptr->Gating(measurement1, track2).first);

  // PBQ not implemented.
}

TEST(BevIouGate, TestGating) {
  // measurement
  MeasurementProto measurement;
  measurement.set_timestamp(0.1);
  measurement.set_type(MT_VEHICLE);
  measurement.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_x(1.0);
  measurement.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_y(0.1);
  measurement.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_heading(0.0);
  measurement.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_width(2.3);
  measurement.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_length(5.1);

  MeasurementProto m1;
  m1.mutable_laser_measurement();
  Track<TrackState> track1;
  track1.track_state.state_timestamp = 0.0;
  track1.track_state.estimator_3d = BuildCarModelEstimator();
  track1.track_state.type = MT_VEHICLE;
  track1.track_state.bounding_box = Box2d({0.0, 0.0}, 0.0, 5.0, 2.5);
  track1.track_state.measurement = &m1;
  track1.checkpoints.push_back(0.0, track1.track_state);

  // Gate
  BevIouGate<Track<TrackState>, MeasurementProto> gate({1e-8});
  EXPECT_TRUE(gate.Gating(measurement, track1).first);
}

TEST(SpeedGate, TestGating) {
  Track<TrackState> track1, track2;
  track1.track_state.type = MT_PEDESTRIAN;
  TrackState track_state1;
  std::vector<Vec2d> contour1 = {
      {3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}};
  track_state1.contour = Polygon2d(contour1);
  track_state1.state_timestamp = 1.0;
  track1.checkpoints.push_back(track_state1.state_timestamp, track_state1);
  MeasurementProto m1;
  m1.set_timestamp(1.0);
  for (size_t i = 0; i < contour1.size(); ++i) {
    m1.mutable_laser_measurement()->add_contour();
    m1.mutable_laser_measurement()->mutable_contour(i)->set_x(contour1[i].x());
    m1.mutable_laser_measurement()->mutable_contour(i)->set_y(contour1[i].y());
  }
  track1.measurement_history.push_back(1.0, &m1);

  track2.track_state.type = MT_CYCLIST;
  TrackState track_state2;
  std::vector<Vec2d> contour2 = {
      {10.0, 10.0}, {11.0, 10.0}, {11.0, 11.0}, {10.0, 11.0}};
  track_state2.contour = Polygon2d(contour2);
  track_state2.state_timestamp = 1.0;
  track2.checkpoints.push_back(track_state2.state_timestamp, track_state1);
  MeasurementProto m2;
  m2.set_timestamp(1.0);
  for (size_t i = 0; i < contour2.size(); ++i) {
    m2.mutable_laser_measurement()->add_contour();
    m2.mutable_laser_measurement()->mutable_contour(i)->set_x(contour1[i].x());
    m2.mutable_laser_measurement()->mutable_contour(i)->set_y(contour1[i].y());
  }
  track2.measurement_history.push_back(1.0, &m2);

  MeasurementProto measurement;
  measurement.set_timestamp(1.1);
  measurement.set_type(MT_PEDESTRIAN);
  std::vector<Vec2d> measurement1_contour = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (size_t i = 0; i < measurement1_contour.size(); ++i) {
    measurement.mutable_laser_measurement()->add_contour();
    measurement.mutable_laser_measurement()->mutable_contour(i)->set_x(
        measurement1_contour[i].x());
    measurement.mutable_laser_measurement()->mutable_contour(i)->set_y(
        measurement1_contour[i].y());
  }
  // Gate
  SpeedGate<Track<TrackState>, MeasurementProto> gate({0.0});
  EXPECT_FALSE(gate.Gating(measurement, track1).first);
  EXPECT_FALSE(gate.Gating(measurement, track2).first);
}

TEST(TypeGate, TestGating) {
  // Dimension
  TypeGate<Track<TrackState>, MeasurementProto> gate_dim({0.5});
  for (int mt = MeasurementType_MIN; mt <= MeasurementType_MAX; ++mt) {
    QCHECK_LT(mt, gate_dim.Dim());
  }
  QCHECK_EQ(MT_UNKNOWN, 0);
  QCHECK_EQ(MT_VEHICLE, 1);
  QCHECK_EQ(MT_MOTORCYCLIST, 2);
  QCHECK_EQ(MT_PEDESTRIAN, 3);
  QCHECK_EQ(MT_CYCLIST, 4);
  QCHECK_EQ(MT_FOD, 5);
  QCHECK_EQ(MT_STATIC_OBJECT, 6);
  QCHECK_EQ(MT_VEGETATION, 7);
  QCHECK_EQ(MT_BARRIER, 8);
  QCHECK_EQ(MT_ROAD, 9);
  QCHECK_EQ(MT_CONE, 10);
  QCHECK_EQ(MT_MIST, 11);
  QCHECK_EQ(MT_FLYING_BIRD, 12);
  QCHECK_EQ(MT_WARNING_TRIANGLE, 13);

  Track<TrackState> track1, track2, track3;
  track1.track_state.type = MT_VEHICLE;
  track2.track_state.type = MT_PEDESTRIAN;
  track3.track_state.type = MT_CYCLIST;
  MeasurementProto measurement1, measurement2, measurement3;
  measurement1.set_type(MT_VEHICLE);
  measurement1.set_type_source(MTS_FIERY_EYE_NET);
  measurement2.set_type(MT_PEDESTRIAN);
  measurement2.set_type_source(MTS_FIERY_EYE_NET);
  measurement3.set_type(MT_VEGETATION);
  measurement3.set_type_source(MTS_SEMANTIC_MAP_ZONE);
  // Gate
  TypeGate<Track<TrackState>, MeasurementProto> gate({0.5});
  EXPECT_TRUE(gate.Gating(measurement1, track1).first);
  EXPECT_FALSE(gate.Gating(measurement1, track2).first);
  EXPECT_TRUE(gate.Gating(measurement1, track3).first);
  EXPECT_FALSE(gate.Gating(measurement2, track1).first);
  EXPECT_TRUE(gate.Gating(measurement2, track2).first);
  EXPECT_TRUE(gate.Gating(measurement2, track3).first);
  EXPECT_FALSE(gate.Gating(measurement3, track1).first);
}

TEST(ElipseGate, TestGating) {}

TEST(ImageIouGate, TestGating) {
  // PBQ
  Track<multi_camera_fusion::MultiCameraTrackState> track_pbq;
  // track_pbq.track_state.estimator_2d = BuildCarModelEstimator();
  MeasurementProto mea_history;
  mea_history.set_timestamp(0.1);
  auto* mea_history_bbox2d =
      mea_history.mutable_camera3d_measurement()->mutable_bbox_2d();
  mea_history_bbox2d->set_x(0);
  mea_history_bbox2d->set_y(2);
  mea_history_bbox2d->set_width(4);
  mea_history_bbox2d->set_height(4);
  track_pbq.measurement_history.push_back(0.0, &mea_history);

  MeasurementProto mea_pbq;
  mea_pbq.set_timestamp(0.1);
  auto* mea_bbox2d = mea_pbq.mutable_camera3d_measurement()->mutable_bbox_2d();
  mea_bbox2d->set_x(2);
  mea_bbox2d->set_y(0);
  mea_bbox2d->set_width(4);
  mea_bbox2d->set_height(4);

  ImageIouGate<Track<multi_camera_fusion::MultiCameraTrackState>,
               MeasurementProto>
      gate({0.1});
  EXPECT_TRUE(gate.Gating(mea_pbq, track_pbq).first);
  EXPECT_DOUBLE_EQ(gate.Gating(mea_pbq, track_pbq).second, 4.0 / 28.0);
}

}  // namespace association
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
