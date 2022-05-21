#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_LIFE_MANAGER_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_LIFE_MANAGER_

#include <string>
#include <unordered_map>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/tracker/track.h"

namespace qcraft::tracker::track_life_manager {

// TODO(zheng): Add track state and track state transfer mechanism.
class TrackLifeManager {
 public:
  TrackLifeManager() = default;
  ~TrackLifeManager() = default;

  static bool ShouldBeAtConfirmedState(const Track<TrackState>& track,
                                       const VehiclePose& pose,
                                       double current_timestamp);
  static bool ShouldBeAtIdleState(
      const Track<TrackState>& track, const Vec2d& ego_pose,
      double current_timestamp,
      double max_allowed_no_update_time_for_idle_state);
  static bool ShouldBeAtLostState(
      const Track<TrackState>& track, double current_timestamp,
      double max_allowed_no_update_time_for_lost_state);
  static void ComputeMaxAllowedNoUpdatedTime(
      const Track<TrackState>& track, double life_time_buffer,
      bool is_track_in_lidar_occluded_area,
      double* max_allowed_no_update_time_for_lost_state,
      double* max_allowed_no_update_time_for_idle_state);

 private:
  static bool HasOnlyRadarMeasurementAfterTime(const Track<TrackState>& track,
                                               double timestamp);
  static int GetNumUpdatesForPromote(const Track<TrackState>& track,
                                     const VehiclePose& pose);
  static bool ShouldPromoteImmediately(const Track<TrackState>& track,
                                       const VehiclePose& pose);

  static int GetConfirmendMeasurementNumInAllowedDuration(
      const Track<TrackState>& track, const VehiclePose& pose,
      double current_timestamp);
  static int GetMeasurementNum(const Track<TrackState>& track,
                               double current_timestamp,
                               double allowed_duration);
};

}  // namespace qcraft::tracker::track_life_manager

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_LIFE_MANAGER_
