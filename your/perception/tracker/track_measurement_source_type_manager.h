#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_MEASUREMENT_SOURCE_TYPE_MANAGER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_MEASUREMENT_SOURCE_TYPE_MANAGER_H_

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "onboard/params/param_manager.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker::track_measurement_source_type_manager {

using TMST = TrackMeasurementSourceType;
using MP = MeasurementProto;

class TrackMeasurementSourceTypeManager {
 public:
  TrackMeasurementSourceTypeManager() = default;
  explicit TrackMeasurementSourceTypeManager(
      const RunParamsProtoV2& run_params);
  ~TrackMeasurementSourceTypeManager() = default;

 public:
  // Compute track measurement source type after updated by measurement.
  static TMST ComputeTrackMeasurementSourceType(const Track<TrackState>& track,
                                                const MeasurementProto& m);
  static TMST ComputeTrackMeasurementSourceTypeByMeasurementHistory(
      const Track<TrackState>& track);
  static bool HasLidarMeasurements(const Track<TrackState>& track);
  static bool HasLidarMeasurements(TMST source_type);
  static bool HasCameraMeasurements(const Track<TrackState>& track);
  // Judge if we should create a track from radar measurement.
  static bool ShouldCreateTrackFromRadar(const MeasurementProto& m,
                                         const VehiclePose& pose,
                                         const bool is_first_frame);

  // Determine the transition from LVR to VR
  static bool IsSwitchToNoLidarTrack(const TMST prev, const TMST curr);

  // Determine the transition from VR to LVR
  static bool IsSwitchToLidarTrack(const TMST prev, const TMST curr);

  // Judge if we should create a track from vision measurement.
  bool ShouldCreateTrackFromVision(const MeasurementProto& m,
                                   const VehiclePose& pose,
                                   const AffineTransformation& pose_inv,
                                   const bool is_first_frame);

 private:
  static const std::map<std::pair<TMST, MP::MeasurementCase>, TMST>
      kMeasurementSourceTypeTransMap;

  static const std::map<int, TMST> kMeasurementSourceTypeInMeasurementHistory;
  static const std::unordered_set<LidarModel> kPbqLidarList;
  std::unordered_map<LidarId, AffineTransformation> pbq_vehicle2lidar_trans_;
  static constexpr double kTimeDurationForSourceTypeComputing = 0.5;  // s
};

}  // namespace qcraft::tracker::track_measurement_source_type_manager

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_MEASUREMENT_SOURCE_TYPE_MANAGER_H_
