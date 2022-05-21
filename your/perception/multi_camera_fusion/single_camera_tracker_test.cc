
#include "onboard/perception/multi_camera_fusion/single_camera_tracker.h"

#include "gtest/gtest.h"
#include "onboard/perception/multi_camera_fusion/single_camera_tracker_friend.h"
#include "onboard/utils/file_util.h"

namespace qcraft::multi_camera_fusion {

TEST(SingleCameraTrackerTest, AssociateCameraMeasurementsAndUpdateTracksTest) {
  ThreadPool tracker_thread_pool(2);
  SingleCameraTracker tracker(CAM_L_FRONT, &tracker_thread_pool);
  SingleCameraTrackerFriend tracker_friend(&tracker);
  auto tracks = tracker_friend.tracks();
  EXPECT_EQ(tracks.size(), 0);
  constexpr int m_num = 3;

  MeasurementsProto measurements_group;
  for (int i = 0; i < m_num; ++i) {
    Camera3dMeasurementProto m;
    m.set_type(MT_VEHICLE);
    m.set_timestamp(0.1);
    m.set_heading(0.0);
    auto* m_pos = m.mutable_pos();
    m_pos->set_x(i * 10);
    m_pos->set_y(i * 10);

    auto* camera_m = measurements_group.add_measurements();
    *camera_m->mutable_camera3d_measurement() = m;
  }
  tracker_friend.AssociateCameraMeasurementsAndUpdateTracks(0.1,
                                                            measurements_group);

  EXPECT_EQ(tracker_friend.tracks().size(), m_num);
  // TODO(zheng): Add Update test after we add motion filter.
}

TEST(SingleCameraTrackerTest, RemoveExpiredTracksTest) {
  ThreadPool tracker_thread_pool(2);
  SingleCameraTracker tracker(CAM_L_FRONT, &tracker_thread_pool);
  SingleCameraTrackerFriend tracker_friend(&tracker);
  auto& tracks = tracker_friend.mutable_tracks();
  for (int i = 1; i < 10; ++i) {
    tracks.emplace_back(
        std::make_unique<tracker::Track<MultiCameraTrackState>>());
    auto& track = tracks.back();
    track->track_state.id = i;
    track->track_state.last_timestamp = i * 0.1;
  }
  EXPECT_EQ(tracks.size(), 9);
  tracker_friend.RemoveExpiredTracks(0.3);
  EXPECT_EQ(tracks.size(), 9);
  tracker_friend.RemoveExpiredTracks(0.6);
  EXPECT_EQ(tracks.size(), 7);
  tracker_friend.RemoveExpiredTracks(1.5);
  EXPECT_EQ(tracks.size(), 0);
}

TEST(SingleCameraTrackerTest, ShouldUseCarModelTest) {
  ThreadPool tracker_thread_pool(2);
  SingleCameraTracker tracker(CAM_L_FRONT, &tracker_thread_pool);
  SingleCameraTrackerFriend tracker_friend(&tracker);
  tracker::Track<MultiCameraTrackState> track;
  track.track_state.type = MT_VEHICLE;
  EXPECT_TRUE(tracker_friend.ShouldUseCarModel(track));

  track.track_state.type = MT_PEDESTRIAN;
  EXPECT_FALSE(tracker_friend.ShouldUseCarModel(track));

  track.track_state.type = MT_CYCLIST;
  EXPECT_TRUE(tracker_friend.ShouldUseCarModel(track));

  track.track_state.type = MT_MOTORCYCLIST;
  EXPECT_TRUE(tracker_friend.ShouldUseCarModel(track));

  track.track_state.type = MT_CONE;
  EXPECT_FALSE(tracker_friend.ShouldUseCarModel(track));
}
}  // namespace qcraft::multi_camera_fusion
