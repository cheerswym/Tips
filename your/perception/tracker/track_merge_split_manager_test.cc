#include "onboard/perception/tracker/track_merge_split_manager.h"

#include <utility>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/test_util.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/perception/test_util/track_builder.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::tracker {
namespace {
std::vector<TrackRef> DuplicateTracks(const std::vector<TrackRef>& tracks) {
  std::vector<TrackRef> dup_tracks;
  for (const auto& track : tracks) {
    dup_tracks.push_back(std::make_unique<Track<TrackState>>(*track));
  }
  return dup_tracks;
}
}  // namespace

Track<TrackState> CreateCarModelTrack(
    const Polygon2d& init_poly, const Vec2d& velocity, int track_id,
    bool is_offroad, MeasurementType type, double init_timestamp,
    MotionFilterParamProto::Type motion_filter_type) {
  TrackBuilder track_builder;
  track_builder.set_contour(init_poly);
  track_builder.set_id(track_id);
  track_builder.set_type(type);
  track_builder.set_offroad(is_offroad);

  // Create estimator.
  const Vec2d centroid = tracker_util::ComputeCentroid(init_poly.points());
  MotionFilterParamProto params;
  CHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(motion_filter_type);
  std::vector<MotionFilterParamProto> v{params, params};
  tracker::Estimator estimator(tracker::ToImmProto(v, {0.5, 0.5}));
  EXPECT_TRUE(estimator.IsCarModel());
  EXPECT_FALSE(estimator.IsPointModel());

  tracker::CarState state;
  state.x() = centroid.x();
  state.y() = centroid.y();
  state.vel() = velocity.norm();
  const auto init_state = tracker::StateData(state);
  estimator.Init(init_state, init_timestamp);
  track_builder.set_estimator(estimator);
  return track_builder.Build();
}

void UpdateTrackExtents(const LaserMeasurementProto& laser_measurement,
                        Track<TrackState>* track) {
  track->track_state.contour =
      geometry_util::ToPolygon2d(laser_measurement.contour());
  track->track_state.contour =
      geometry_util::ToPolygon2d(laser_measurement.contour());
  if (laser_measurement.has_cluster_measurement()) {
    track->track_state.anchor_point =
        tracker_util::ComputeWeightedObstacleCentroid(
            laser_measurement.cluster_measurement());
  }

  if (laser_measurement.has_detection_bounding_box()) {
    track->track_state.bounding_box =
        Box2d(laser_measurement.detection_bounding_box());
    track->track_state.refined_bounding_box =
        Box2d(laser_measurement.refined_bounding_box());
  } else {
    track->track_state.bounding_box = std::nullopt;
    track->track_state.refined_bounding_box = std::nullopt;
  }

  // Sync min_z, max_z and ground_z value.
  if (laser_measurement.has_cluster_measurement()) {
    const auto& cluster = laser_measurement.cluster_measurement();
    track->track_state.min_z = cluster.min_z();
    track->track_state.max_z = cluster.max_z();
    track->track_state.ground_z = cluster.ground_z();
  }
}

void UpdateTrackFromLaserMeasurement(const MeasurementProto* measurement,
                                     Track<TrackState>* track) {
  QCHECK(measurement->has_laser_measurement());
  const auto& laser_measurement = measurement->laser_measurement();
  QCHECK_NOTNULL(track);
  // Update contour, obstacle centers and bounding box.
  UpdateTrackExtents(laser_measurement, track);
  const Vec2d m_centroid = tracker_util::ComputeCentroid(
      geometry_util::ToPolygon2d(laser_measurement.contour()).points());
  const tracker::PositionMeasurement pos(m_centroid.x(), m_centroid.y());
  track->track_state.estimator_3d.Predict(measurement->timestamp());
  track->track_state.estimator_3d.Update(pos);

  track->measurement_history.PushBackAndClearStale(measurement->timestamp(),
                                                   measurement, 1.0);
  const auto s = track->track_state.estimator_3d.GetStateData();
  track->track_state.vel = s.GetVel();
  track->track_state.heading = s.GetYaw();
  track->track_state.last_timestamp = measurement->timestamp();
  track->track_state.state_timestamp = measurement->timestamp();
  track->track_state.last_laser_timestamp = measurement->timestamp();

  // Save checkpoints.
  TrackState checkpoint = track->track_state;
  track->checkpoints.PushBackAndClearStale(checkpoint.last_timestamp,
                                           std::move(checkpoint),
                                           kMaxCheckPointHistoryBufferLength);
}

std::vector<MeasurementProto> CreatTrackLaserMeasurements(
    int start_index, int end_index, double time_interval, double init_timestamp,
    MeasurementType type, const Vec2d& velocity, const Polygon2d& init_poly) {
  std::vector<MeasurementProto> measurements;
  for (int i = start_index; i < end_index; ++i) {
    const double timestamp = i * time_interval + init_timestamp;
    const double delta_time = i * time_interval - init_timestamp;  // s
    const Vec2d offset = velocity * delta_time;
    const auto curr_poly =
        tracker_util::ShiftPoints(offset, init_poly.points());
    auto laser_measurement =
        LaserMeasurementBuilder().set_contour(curr_poly).Build();

    auto measurement = BuildMeasurementProto(timestamp, type, MTS_LL_NET,
                                             std::move(laser_measurement));
    measurements.emplace_back(measurement);
  }
  return measurements;
}

TEST(TrackMergeSplitManagerTest, MergeTest) {
  const Polygon2d poly(Box2d::CreateAABox({0, 10}, {3, 13}));
  const double init_timestamp = 0.0;
  const Vec2d velocity = Vec2d(2.0, 0.0);
  const auto measurements0 = CreatTrackLaserMeasurements(
      1, 5, 0.1, init_timestamp, MT_VEHICLE, velocity, poly);
  const auto measurements1 = CreatTrackLaserMeasurements(
      1, 5, 0.1, init_timestamp, MT_VEHICLE, velocity, poly);

  std::vector<TrackRef> base_tracks;
  // Create and update tracks[0].
  base_tracks.emplace_back(std::make_unique<Track<TrackState>>(
      CreateCarModelTrack(poly, Vec2d(2.0, 0.0), 0, false, MT_VEHICLE,
                          init_timestamp, MotionFilterParamProto::CAR_CV)));
  for (int i = 0; i < 3; ++i) {
    UpdateTrackFromLaserMeasurement(&measurements0[i], base_tracks[0].get());
  }

  // Create and update tracks[1].
  base_tracks.emplace_back(std::make_unique<Track<TrackState>>(
      CreateCarModelTrack(poly, Vec2d(2.0, 0.0), 0, false, MT_VEHICLE,
                          init_timestamp, MotionFilterParamProto::CAR_CTRV)));
  for (int i = 0; i < 2; ++i) {
    UpdateTrackFromLaserMeasurement(&measurements1[i], base_tracks[1].get());
  }

  ThreadPool thread_pool(1);
  TrackMergeSplitManager tmsm(&thread_pool);
  // Merge test, track0 has laser measurement, track1 has no
  // laser measurement in current frame.
  std::vector<TrackRef> tracks0 = DuplicateTracks(base_tracks);
  tmsm.MergeAndSplitTracks(0.3, &tracks0);
  EXPECT_FALSE(tracks0[0]->track_state.merged_by_other_track);
  EXPECT_TRUE(tracks0[1]->track_state.merged_by_other_track);

  // Type blacklist test.
  std::vector<TrackRef> tracks1 = DuplicateTracks(base_tracks);
  tracks1[0]->track_state.type = MT_PEDESTRIAN;
  tmsm.MergeAndSplitTracks(0.3, &tracks1);
  EXPECT_FALSE(tracks1[0]->track_state.merged_by_other_track);
  EXPECT_FALSE(tracks1[1]->track_state.merged_by_other_track);
  // Unknown type test.
  std::vector<TrackRef> tracks2 = DuplicateTracks(base_tracks);
  tracks2[0]->track_state.type = MT_UNKNOWN;
  tracks2[1]->track_state.type = MT_STATIC_OBJECT;
  tmsm.MergeAndSplitTracks(0.3, &tracks2);
  EXPECT_FALSE(tracks2[0]->track_state.merged_by_other_track);
  EXPECT_TRUE(tracks2[1]->track_state.merged_by_other_track);
  // State sync test.
  std::vector<TrackRef> tracks3 = DuplicateTracks(base_tracks);
  tracks3[0]->track_state.type = MT_UNKNOWN;
  tracks3[1]->track_state.type = MT_VEHICLE;
  tracks3[0]->track_state.life_state = TrackLifeState::kConfirmed;
  tracks3[1]->track_state.life_state = TrackLifeState::kInit;
  EXPECT_GT(
      tracks3[1]->track_state.estimator_3d.GetStateData().car().state().vel(),
      1.0);
  auto source_s = tracks3[0]->track_state.estimator_3d.GetStateData();
  source_s.car().state().vel() = 0.0;
  // source_s.car().state_cov() =
  //     0.01 * tracker::Covariance<tracker::CarState>::Identity();
  tracks3[0]->track_state.estimator_3d.SetStateData(source_s);
  tmsm.MergeAndSplitTracks(0.3, &tracks3);
  EXPECT_EQ(tracks3[1]->track_state.life_state, TrackLifeState::kConfirmed);
  EXPECT_TRUE(tracks3[0]->track_state.merged_by_other_track);
  EXPECT_FALSE(tracks3[1]->track_state.merged_by_other_track);
  EXPECT_LT(
      tracks3[1]->track_state.estimator_3d.GetStateData().car().state().vel(),
      0.1);
}

TEST(TrackMergeSplitManagerTest, SplitTest) {
  const Polygon2d poly(Box2d::CreateAABox({0, 10}, {3, 13}));
  const Polygon2d split_poly0(Box2d::CreateAABox({0, 10}, {1.5, 11.5}));
  const Polygon2d split_poly1(Box2d::CreateAABox({1.5, 11.5}, {3, 13}));
  const double init_timestamp = 0.0;
  const Vec2d velocity = Vec2d(2.0, 0.0);
  const auto measurements0 = CreatTrackLaserMeasurements(
      1, 10, 0.1, init_timestamp, MT_VEHICLE, velocity, poly);
  const auto measurements1 = CreatTrackLaserMeasurements(
      1, 10, 0.1, init_timestamp, MT_VEHICLE, velocity, split_poly0);
  const auto measurements2 = CreatTrackLaserMeasurements(
      1, 10, 0.1, init_timestamp, MT_VEHICLE, velocity, split_poly1);

  std::vector<TrackRef> base_tracks;
  // Create and update tracks[0].
  base_tracks.emplace_back(std::make_unique<Track<TrackState>>(
      CreateCarModelTrack(poly, Vec2d(2.0, 0.0), 0, false, MT_VEHICLE,
                          init_timestamp, MotionFilterParamProto::CAR_CV)));
  for (int i = 0; i < 4; ++i) {
    UpdateTrackFromLaserMeasurement(&measurements0[i], base_tracks[0].get());
  }
  // Use splited measurement to update track.
  int split_index = 4;
  UpdateTrackFromLaserMeasurement(&measurements1[split_index],
                                  base_tracks[0].get());

  // Create and update tracks[1].
  const auto splited_meaurement_contour = geometry_util::ToPolygon2d(
      measurements2[split_index].laser_measurement().contour());
  base_tracks.emplace_back(
      std::make_unique<Track<TrackState>>(CreateCarModelTrack(
          splited_meaurement_contour, Vec2d(0.0, 0.0), 0, false, MT_VEHICLE,
          init_timestamp, MotionFilterParamProto::CAR_CTRV)));
  UpdateTrackFromLaserMeasurement(&measurements2[split_index],
                                  base_tracks[1].get());

  ThreadPool thread_pool(1);
  TrackMergeSplitManager tmsm(&thread_pool);
  std::vector<TrackRef> tracks0 = DuplicateTracks(base_tracks);
  tmsm.MergeAndSplitTracks(0.5, &tracks0);
  // Do not split track if the tracks are not confirmed.
  EXPECT_FALSE(tracks0[0]->track_state.merged_by_other_track);
  EXPECT_FALSE(tracks0[1]->track_state.merged_by_other_track);
  EXPECT_FALSE(tracks0[0]->track_state.split_from_other_track);
  EXPECT_FALSE(tracks0[1]->track_state.split_from_other_track);
  std::vector<TrackRef> tracks1 = DuplicateTracks(base_tracks);
  tracks1[0]->track_state.life_state = TrackLifeState::kConfirmed;
  tmsm.MergeAndSplitTracks(0.5, &tracks1);
  EXPECT_FALSE(tracks1[0]->track_state.split_from_other_track);
  EXPECT_TRUE(tracks1[1]->track_state.split_from_other_track);
  // Type test.
  std::vector<TrackRef> tracks2 = DuplicateTracks(base_tracks);
  tracks2[0]->track_state.life_state = TrackLifeState::kConfirmed;
  tracks2[0]->track_state.type = MT_PEDESTRIAN;
  tmsm.MergeAndSplitTracks(0.5, &tracks2);
  EXPECT_FALSE(tracks2[0]->track_state.split_from_other_track);
  EXPECT_FALSE(tracks2[1]->track_state.split_from_other_track);
  std::vector<TrackRef> tracks3 = DuplicateTracks(base_tracks);
  tracks3[0]->track_state.life_state = TrackLifeState::kConfirmed;
  tracks3[0]->track_state.type = MT_UNKNOWN;
  tmsm.MergeAndSplitTracks(0.5, &tracks3);
  EXPECT_FALSE(tracks3[0]->track_state.split_from_other_track);
  EXPECT_TRUE(tracks3[1]->track_state.split_from_other_track);

  // State sync test.
  std::vector<TrackRef> tracks4 = DuplicateTracks(base_tracks);
  tracks4[0]->track_state.life_state = TrackLifeState::kConfirmed;
  tracks4[0]->track_state.type = MT_VEHICLE;
  tracks4[1]->track_state.type = MT_UNKNOWN;
  EXPECT_LT(
      tracks4[1]->track_state.estimator_3d.GetStateData().car().state().vel(),
      0.1);
  tmsm.MergeAndSplitTracks(0.5, &tracks4);
  EXPECT_FALSE(tracks4[0]->track_state.split_from_other_track);
  EXPECT_TRUE(tracks4[1]->track_state.split_from_other_track);
  EXPECT_GT(
      tracks4[1]->track_state.estimator_3d.GetStateData().car().state().vel(),
      1.0);
  EXPECT_EQ(tracks4[1]->track_state.type, MT_VEHICLE);
  EXPECT_EQ(tracks4[1]->track_state.motion_filter_type,
            tracks4[0]->track_state.motion_filter_type);
}

}  // namespace qcraft::tracker
