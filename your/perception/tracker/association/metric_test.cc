#include "onboard/perception/tracker/association/metric.h"

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

TEST(EuclideanMetric, TestSimilarity) {
  // DBQ

  // weighted_obstacle_centroid = (3.5, 3.5)
  std::vector<Vec2d> m_laser_near_contour = {
      {3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}};
  MeasurementProto m_laser_near;
  for (size_t i = 0; i < m_laser_near_contour.size(); ++i) {
    auto* obstacle_info = m_laser_near.mutable_laser_measurement()
                              ->mutable_cluster_measurement()
                              ->add_obstacle_info();
    obstacle_info->mutable_center()->set_x(m_laser_near_contour[i].x());
    obstacle_info->mutable_center()->set_x(m_laser_near_contour[i].y());
    obstacle_info->set_num_points(1);
  }

  // weighted_obstacle_centroid = (10.5, 10.5)
  std::vector<Vec2d> m_laser_away_contour = {
      {10.0, 10.0}, {11.0, 10.0}, {11.0, 11.0}, {10.0, 11.0}};
  MeasurementProto m_laser_away;
  for (size_t i = 0; i < m_laser_away_contour.size(); ++i) {
    auto* obstacle_info = m_laser_away.mutable_laser_measurement()
                              ->mutable_cluster_measurement()
                              ->add_obstacle_info();
    obstacle_info->mutable_center()->set_x(m_laser_away_contour[i].x());
    obstacle_info->mutable_center()->set_x(m_laser_away_contour[i].y());
    obstacle_info->set_num_points(1);
  }

  Track<TrackState> track1;
  track1.track_state.state_timestamp = 0.0;
  track1.track_state.estimator_3d = BuildCarModelEstimator();
  track1.track_state.measurement = &m_laser_near;
  track1.checkpoints.push_back(0.0, track1.track_state);

  Track<TrackState> track2;
  track2.track_state.state_timestamp = 0.0;
  track2.track_state.estimator_3d = BuildCarModelEstimator();
  track2.track_state.measurement = &m_laser_away;
  track2.checkpoints.push_back(0.0, track2.track_state);

  MeasurementProto measurement1;
  measurement1.set_timestamp(0.1);
  std::vector<Vec2d> measurement1_contour = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (size_t i = 0; i < measurement1_contour.size(); ++i) {
    auto* obstacle_info = measurement1.mutable_laser_measurement()
                              ->mutable_cluster_measurement()
                              ->add_obstacle_info();
    obstacle_info->mutable_center()->set_x(measurement1_contour[i].x());
    obstacle_info->mutable_center()->set_x(measurement1_contour[i].y());
    obstacle_info->set_num_points(1);
  }

  // Similarity
  EuclideanMetric<Track<TrackState>, MeasurementProto> metric({100.0});
  const double s1 = metric.Similarity(measurement1, track1);
  EXPECT_LE(s1, 1.0);
  EXPECT_GE(s1, 0.0);
  const double s2 = metric.Similarity(measurement1, track2);
  EXPECT_LE(s2, 1.0);
  EXPECT_GE(s2, 0.0);
  EXPECT_GT(s1, s2);

  // PBQ
  Track<multi_camera_fusion::MultiCameraTrackState> track3;
  track3.track_state.estimator_3d = BuildCarModelEstimator();
  auto s = track3.track_state.estimator_3d.GetStateData();
  s.car().state().x() = 10.0;
  s.car().state().y() = 10.0;
  s.car().state().yaw() = M_PI_2;
  track3.track_state.estimator_3d.SetStateData(s);
  track3.track_state.type = MT_VEHICLE;

  // measurement
  MeasurementProto measurement2;
  measurement2.mutable_camera3d_measurement()->set_timestamp(0.1);
  measurement2.mutable_camera3d_measurement()->mutable_pos()->set_x(11.0);
  measurement2.mutable_camera3d_measurement()->mutable_pos()->set_y(11.0);

  EuclideanMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                  MeasurementProto>
      metric2({100.0});
  const double s3 = metric2.Similarity(measurement2, track3);
  EXPECT_DOUBLE_EQ(s3, 1.0 - 2. / 100.);
}

TEST(VehicleIouMetric, TestSimilarity) {
  Track<TrackState> track1, track2;
  track1.track_state.type = MT_VEHICLE;
  track1.track_state.bounding_box = Box2d({0.0, 0.0}, 0.0, 5.0, 2.5);
  track2.track_state.type = MT_VEHICLE;
  track2.track_state.bounding_box = Box2d({10.0, 10.0}, M_PI_2, 5.0, 2.5);

  MeasurementProto measurement1;
  measurement1.set_type(MT_VEHICLE);
  measurement1.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_x(1.0);
  measurement1.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_y(0.1);
  measurement1.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_heading(0.0);
  measurement1.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_width(2.3);
  measurement1.mutable_laser_measurement()
      ->mutable_detection_bounding_box()
      ->set_length(5.1);

  // Similarity
  VehicleIouMetric<Track<TrackState>, MeasurementProto> metric({0.1});
  EXPECT_EQ(metric.Similarity(measurement1, track1), kMaxSimilarityValue);
  EXPECT_EQ(metric.Similarity(measurement1, track2), 0.0);
}

// TODO(jingwei)
TEST(PValueMetric, TestSimilarity) {}

// TODO(jingwei)
TEST(RadarMahalanobisMetric, TestSimilarity) {}

TEST(ImageIouMetric, TestSimilarity) {
  // DBQ
  Track<TrackState> track_dbq;
  MeasurementProto mea_dbq;
  ImageIouMetric<Track<TrackState>, MeasurementProto> metric({0.5});
  EXPECT_DOUBLE_EQ(metric.Similarity(mea_dbq, track_dbq), 0.0);

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
  ImageIouMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                 MeasurementProto>
      metric_pbq({0.5});
  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_pbq, track_pbq), 0.0);
}

TEST(ImageAppearanceFeatureMetric, TestSimilarity) {
  std::vector<float> feature1(128, sqrt(1. / 128.));
  std::vector<float> feature2(128, 0.);
  std::vector<float> feature3(128, -sqrt(1. / 128.));
  // DBQ
  MeasurementProto mea_dbq_same;
  *mea_dbq_same.mutable_camera3d_measurement()
       ->mutable_reid_feature_embeddings() = {feature1.begin(), feature1.end()};

  MeasurementProto mea_dbq_zero;
  *mea_dbq_zero.mutable_camera3d_measurement()
       ->mutable_reid_feature_embeddings() = {feature2.begin(), feature2.end()};

  MeasurementProto mea_dbq_inv;
  *mea_dbq_inv.mutable_camera3d_measurement()
       ->mutable_reid_feature_embeddings() = {feature3.begin(), feature3.end()};

  Track<TrackState> track_dbq;
  track_dbq.measurement_history.push_back(0.0, &mea_dbq_same);

  ImageAppearanceFeatureMetric<Track<TrackState>, MeasurementProto> metric_dbq(
      {0.6});

  EXPECT_DOUBLE_EQ(metric_dbq.Similarity(mea_dbq_same, track_dbq), 1.0);
  EXPECT_DOUBLE_EQ(metric_dbq.Similarity(mea_dbq_zero, track_dbq), 0.0);
  EXPECT_DOUBLE_EQ(metric_dbq.Similarity(mea_dbq_inv, track_dbq), 0.0);

  // PBQ
  Track<multi_camera_fusion::MultiCameraTrackState> track_pbq;
  track_pbq.measurement_history.push_back(0.0, &mea_dbq_same);

  ImageAppearanceFeatureMetric<
      Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>
      metric_pbq({0.6});

  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_dbq_same, track_pbq), 1.0);
  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_dbq_zero, track_pbq), 0.0);
  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_dbq_inv, track_pbq), 0.0);
}

TEST(CameraTrackerIdMetric, TestSimilarity) {
  // DBQ
  MeasurementProto mea_dbq_id_10;
  mea_dbq_id_10.mutable_camera3d_measurement()->set_track_id(10);

  MeasurementProto mea_dbq_id_20;
  mea_dbq_id_20.mutable_camera3d_measurement()->set_track_id(20);

  Track<TrackState> track_dbq_id_10;
  track_dbq_id_10.measurement_history.push_back(0.0, &mea_dbq_id_10);

  CameraTrackerIdMetric<Track<TrackState>, MeasurementProto> metric_dbq({});

  EXPECT_DOUBLE_EQ(metric_dbq.Similarity(mea_dbq_id_10, track_dbq_id_10), 1.0);
  EXPECT_DOUBLE_EQ(metric_dbq.Similarity(mea_dbq_id_20, track_dbq_id_10), 0.0);

  // PBQ
  Track<multi_camera_fusion::MultiCameraTrackState> track_pbq_id_10;
  track_pbq_id_10.measurement_history.push_back(0.0, &mea_dbq_id_10);

  CameraTrackerIdMetric<Track<multi_camera_fusion::MultiCameraTrackState>,
                        MeasurementProto>
      metric_pbq({});

  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_dbq_id_10, track_pbq_id_10), 1.0);
  EXPECT_DOUBLE_EQ(metric_pbq.Similarity(mea_dbq_id_20, track_pbq_id_10), 0.0);
}

TEST(BevIouMetric, TestSimilarity) {
  Track<TrackState> track;
  track.track_state.bounding_box = Box2d({0.0, 0.0}, 0.0, 5.0, 2.5);
  track.track_state.contour = Polygon2d(track.track_state.bounding_box.value());
  track.track_state.estimator_3d = BuildCarModelEstimator();

  MeasurementProto laser_measurement;
  Polygon2d laser_contour(Box2d({0.0, 0.0}, 0.0, 2.3, 1.0));
  const auto points = laser_contour.points();
  for (int i = 0; i < points.size(); ++i) {
    Vec2dToProto(points[i],
                 laser_measurement.mutable_laser_measurement()->add_contour());
  }

  MeasurementProto cam_measurement;
  Vec3dToProto({0.0, 0.0, 0.0},
               cam_measurement.mutable_camera3d_measurement()->mutable_pos());
  cam_measurement.mutable_camera3d_measurement()->set_heading(0.0);
  cam_measurement.mutable_camera3d_measurement()->set_width(2.0);
  cam_measurement.mutable_camera3d_measurement()->set_length(4.5);
  cam_measurement.mutable_camera3d_measurement()->set_height(2.0);

  MeasurementProto radar_measurement_1, radar_measurement_2;
  Vec2dToProto({10.0, 10.0},
               radar_measurement_1.mutable_radar_measurement()->mutable_pos());
  Vec2dToProto({0.0, 0.0},
               radar_measurement_2.mutable_radar_measurement()->mutable_pos());

  // Similarity
  BevIouMetric<Track<TrackState>, MeasurementProto> metric({0.1, 0.1});
  EXPECT_EQ(metric.Similarity(laser_measurement, track), 1.0);
  EXPECT_EQ(metric.Similarity(cam_measurement, track), 1.0);
  EXPECT_EQ(metric.Similarity(radar_measurement_1, track), 0.0);
  EXPECT_EQ(metric.Similarity(radar_measurement_2, track), 1.0);
}

}  // namespace association
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
