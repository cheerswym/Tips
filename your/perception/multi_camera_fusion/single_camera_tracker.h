#ifndef ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_H_
#define ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "boost/circular_buffer.hpp"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/multi_camera_fusion/tracker_constants.h"
#include "onboard/perception/tracker/association/onboard_associator.h"
#include "onboard/utils/history_buffer.h"
namespace qcraft::multi_camera_fusion {

struct TrackerInputVariable {
  CameraId camera_id;
  VehiclePose pose;
  double center_timestamp;
  std::shared_ptr<MeasurementsProto> camera_measurements;
};

class SingleCameraTracker {
 public:
  explicit SingleCameraTracker(const CameraId camera_id,
                               ThreadPool* thread_pool)
      : camera_id_(camera_id),
        thread_pool_(thread_pool),
        camera_associator_(thread_pool) {
    id_counter_ = kMaxTrackNumForOneCamera * static_cast<int>(camera_id);
    max_allowed_track_id_value_ =
        kMaxTrackNumForOneCamera * (1 + static_cast<int>(camera_id)) - 1;
    camera_associator_.Init(
        "onboard/perception/tracker/association/config/"
        "pbq_camera_associator_assemble_config.pb.txt",
        "PBQ_CAMERA");
  }

  SingleCameraTracker(const SingleCameraTracker&) = delete;
  SingleCameraTracker& operator=(const SingleCameraTracker&) const = delete;

  void TrackObjects(
      double timestamp, const VehiclePose& pose,
      std::shared_ptr<const MeasurementsProto> measurements_group);
  // Sometimes the camera may lost in a process cycle, in this
  // case we should reset all track life state to lost.
  void ResetLifeStateToLost();
  const VehiclePose& pose() const { return pose_; }

  double latest_tracked_timestamp() const { return latest_tracked_timestamp_; }

  const std::vector<TrackRef>& tracks() { return tracks_; }

  int size() const { return tracks_.size(); }

  CameraId camera_id() const { return camera_id_; }

  ThreadPool* thread_pool() { return thread_pool_; }

  void RemoveExpiredTracks(double timestamp);

  std::vector<TrackRef> GetConfirmedTracks();

 private:
  void AssociateCameraMeasurementsAndUpdateTracks(
      const double timestamp, const MeasurementsProto& measurements_group);

  bool ShouldUpdateTrackBy2DMotionTrendAnalysis(
      const tracker::Track<MultiCameraTrackState>& track,
      const MeasurementProto& m);
  void UpdateTracksFromMeasurements(const double timestamp,
                                    const std::vector<const MeasurementProto*>&
                                        matched_measurements_per_track);
  void UpdateTrackFromCamera3DMeasurement(
      const MeasurementProto& measurement,
      tracker::Track<MultiCameraTrackState>* track);
  void UpdateTrackWithoutMeasurement(
      const double timestamp, tracker::Track<MultiCameraTrackState>* track);
  uint32_t GenerateNewTrackId() {
    return id_counter_++;
    QCHECK_LE(id_counter_, max_allowed_track_id_value_);
  }
  // NOTE(zheng): We may filter out some unstable camera measurement
  // such as offroad measurements.
  std::vector<const MeasurementProto*> GetValidMeasurements(
      const MeasurementsProto& measurements_group);
  // Create  new tracks.
  void CreateNewTracksFromMeasurements(
      const std::vector<const MeasurementProto*>& measurements);
  void ClassifyAndAdjustTrackAfterUpdate();

  bool ShouldUseCarModel(
      const tracker::Track<MultiCameraTrackState>& track) const;
  // TODO(zheng, keyan): Add Cerate 2d estimator API.
  tracker::Estimator Create3DEstimator(const bool should_use_car_model);

  void SaveCheckPoints(tracker::Track<MultiCameraTrackState>* track);

  // For test.
  friend class SingleCameraTrackerFriend;

 private:
  CameraId camera_id_;
  uint32_t id_counter_ = 0;
  uint32_t max_allowed_track_id_value_;
  double latest_tracked_timestamp_ = 0.0;
  ThreadPool* thread_pool_;
  VehiclePose pose_;
  std::vector<TrackRef> tracks_;

  static constexpr double kMaxMeasurementHistoryBufferLength = 1.2;  // 1.2s
  static constexpr double kMaxAllowedNoMeasurementUpdateTime = 0.3;  // s
  static constexpr int kNumUpdatesToPromote = 3;
  HistoryBuffer<std::shared_ptr<const MeasurementsProto>>
      measurement_history_groups_;

  tracker::association::OnboardAssociator<tracker::Track<MultiCameraTrackState>,
                                          MeasurementProto>
      camera_associator_;
};

}  // namespace qcraft::multi_camera_fusion

#endif  // ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_SINGLE_CAMERA_TRACKER_H_
