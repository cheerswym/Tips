#ifndef MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_FRIEND_H_
#define MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_FRIEND_H_

#include <memory>
#include <vector>

#include "onboard/perception/multi_camera_fusion/single_camera_tracker.h"

namespace qcraft::multi_camera_fusion {

class SingleCameraTrackerFriend {
 public:
  explicit SingleCameraTrackerFriend(
      SingleCameraTracker* single_camera_tracker) {
    single_camera_tracker_ = single_camera_tracker;
  }
  ~SingleCameraTrackerFriend() = default;

 public:
  inline void AssociateCameraMeasurementsAndUpdateTracks(
      const double timestamp,
      const MeasurementsProto& measurements_group) const {
    single_camera_tracker_->AssociateCameraMeasurementsAndUpdateTracks(
        timestamp, measurements_group);
  }

  inline void RemoveExpiredTracks(double timestamp) const {
    single_camera_tracker_->RemoveExpiredTracks(timestamp);
  }

  bool ShouldUseCarModel(
      const tracker::Track<MultiCameraTrackState>& track) const {
    return single_camera_tracker_->ShouldUseCarModel(track);
  }

  std::vector<TrackRef>& mutable_tracks() {
    return single_camera_tracker_->tracks_;
  }

  const std::vector<TrackRef>& tracks() {
    return single_camera_tracker_->tracks();
  }

 private:
  SingleCameraTracker* single_camera_tracker_;
};
}  // namespace qcraft::multi_camera_fusion

#endif  // MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_FRIEND_H_
