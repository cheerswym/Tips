#include "onboard/perception/tracker/track_life_manager.h"

#include <string>
#include <vector>

#include "onboard/perception/tracker/tracker_constant.h"
#include "onboard/perception/tracker/tracker_util.h"

namespace qcraft::tracker::track_life_manager {
// For different region, we use different num updates for pormotion.
int TrackLifeManager::GetNumUpdatesForPromote(const Track<TrackState>& track,
                                              const VehiclePose& pose) {
  const double dist2 =
      (tracker_util::ComputeCentroid(track.track_state.contour.points()) -
       Vec2d(pose.x, pose.y))
          .squaredNorm();
  if (dist2 <= Sqr(kNearRegionMaxDistance)) {
    return kNumUpdatesToPromoteForNearRegion;
  } else if (dist2 <= Sqr(kMiddleRegionMaxDistance)) {
    return kNumUpdatesToPromoteForMiddleRegion;
  } else {
    return kNumUpdatesToPromoteForFarRegion;
  }
  return kNumUpdatesToPromoteForNearRegion;
}

bool TrackLifeManager::ShouldPromoteImmediately(const Track<TrackState>& track,
                                                const VehiclePose& pose) {
  if (track.track_state.is_certain_static_track) {
    return true;
  }

  if (track.track_state.measurement_source_type ==
          TrackMeasurementSourceType::TMST_VO ||
      track.track_state.measurement_source_type ==
          TrackMeasurementSourceType::TMST_VR) {
    return false;
  }
  // BANDAID(zheng): When the big vehicle segmentation is unstable,
  // the tracker may lost association, to handle this kind of case
  // we promote the big vehicle immediately when the vehicle is close
  // to av.
  constexpr double kMaxSqrDistanceForImmediatelyPromote = Sqr(30.0);  // m^2
  constexpr double kMinAreaToImmediatelyPromote = 12.0;               // 2m * 6m

  const double dist2 =
      (track.track_state.ref_point.pos - Vec2d(pose.x, pose.y)).squaredNorm();

  if (track.track_state.type == MT_VEHICLE &&
      dist2 < kMaxSqrDistanceForImmediatelyPromote) {
    const double area = track.track_state.contour.area();
    // Note(zheng): If the big truck is on left or right of the av the polygon
    // area may be small, for this kind of case, we can use the min area bbox
    // length to judge if the vehicle is a big truck.
    const auto bbox = track.track_state.contour.BoundingBoxWithHeading(
        track.track_state.heading);
    const double length = bbox.length();
    constexpr double kMinLengthToImmediatelyPromote = 6.0;  // m
    if (area > kMinAreaToImmediatelyPromote ||
        length > kMinLengthToImmediatelyPromote) {
      return true;
    }
  }
  return false;
}

int TrackLifeManager::GetConfirmendMeasurementNumInAllowedDuration(
    const Track<TrackState>& track, const VehiclePose& pose,
    double current_timestamp) {
  const int num_measurements = track.measurement_history.size();

  // Note(zheng): Do not use the camera or radar measurement to judge if
  // the track should be confirmed.
  const auto& m_history = track.measurement_history;
  int valid_measurement_num = 0;
  const double allowed_duration_for_valid_measurement = 2.0;  // s

  for (int j = m_history.GetIndexWithTimeAtLeast(
           current_timestamp - allowed_duration_for_valid_measurement);
       j < num_measurements; ++j) {
    const auto* m = m_history.value(j);

    if (m->has_camera3d_measurement() || m->has_laser_measurement() ||
        (track.track_state.measurement_source_type ==
             TrackMeasurementSourceType::TMST_RO &&
         m->has_radar_measurement())) {
      ++valid_measurement_num;
    }
  }

  return valid_measurement_num;
}

int TrackLifeManager::GetMeasurementNum(const Track<TrackState>& track,
                                        double current_timestamp,
                                        double allowed_duration) {
  const auto& m_history = track.measurement_history;
  return m_history.size() - m_history.GetIndexWithTimeAtLeast(
                                current_timestamp - allowed_duration);
}

void TrackLifeManager::ComputeMaxAllowedNoUpdatedTime(
    const Track<TrackState>& track, double life_time_buffer,
    const bool is_track_in_lidar_occluded_area,
    double* max_allowed_no_update_time_for_lost_state,
    double* max_allowed_no_update_time_for_idle_state) {
  double max_track_life_without_update = kMaxOnroadTrackLifeWithoutUpdate;
  if (track.track_state.offroad) {
    max_track_life_without_update = kMaxOffroadTrackLifeWithoutUpdate;
  } else if (track.track_state.is_certain_static_track) {
    max_track_life_without_update = kMaxCertainStaticTrackLifeWithoutUpdate;
  }
  max_track_life_without_update += life_time_buffer;
  *max_allowed_no_update_time_for_lost_state = max_track_life_without_update;
  // TODO(zheng, jingwei): Set different max exist time for different type.
  // TODO(zheng, jingwei): Open depromoted mode after running some scenario
  // tests.
  double max_exist_time_from_lost_to_idle =
      is_track_in_lidar_occluded_area ? 0.5 : 0.0;  // s
  *max_allowed_no_update_time_for_idle_state =
      max_track_life_without_update + max_exist_time_from_lost_to_idle;
}

bool TrackLifeManager::HasOnlyRadarMeasurementAfterTime(
    const Track<TrackState>& track, double timestamp) {
  for (int i = 0; i < track.measurement_history.size(); ++i) {
    const auto& [ts, m] = track.measurement_history[i];
    if (ts <= timestamp) continue;
    if (!m->has_radar_measurement()) return false;
  }
  return true;
}

bool TrackLifeManager::ShouldBeAtIdleState(
    const Track<TrackState>& track, const Vec2d& ego_pose,
    double current_timestamp,
    double max_allowed_no_update_time_for_idle_state) {
  // NOTE: We set delta time bigger than
  // max_allowed_no_update_time_for_idle_state as idle state.
  const bool is_idle_by_time =
      current_timestamp - track.track_state.last_timestamp >
      max_allowed_no_update_time_for_idle_state;
  // NOTE: We set merged track as idle state.
  const bool is_idle_by_merge = track.track_state.merged_by_other_track;
  // NOTE: We set track from other source to RO as idle state.
  constexpr double kTimeRange = 0.64;           // 0.64s
  constexpr double kDistanceRange = Sqr(80.0);  // 80m
  const bool is_idle_by_ro =
      HasOnlyRadarMeasurementAfterTime(
          track, track.track_state.state_timestamp - kTimeRange) &&
      (track.track_state.estimator_3d.GetStateData().GetStatePos() - ego_pose)
              .squaredNorm() < kDistanceRange;
  return is_idle_by_time || is_idle_by_merge || is_idle_by_ro;
}

bool TrackLifeManager::ShouldBeAtLostState(
    const Track<TrackState>& track, double current_timestamp,
    double max_allowed_no_update_time_for_lost_state) {
  return current_timestamp - track.track_state.last_timestamp >
         max_allowed_no_update_time_for_lost_state;
}

bool TrackLifeManager::ShouldBeAtConfirmedState(const Track<TrackState>& track,
                                                const VehiclePose& pose,
                                                double current_timestamp) {
  // TODO(zheng): Remove this BANDAGE.
  if (ShouldPromoteImmediately(track, pose)) {
    return true;
  }
  switch (track.track_state.life_state) {
    case TrackLifeState::kInit: {
      const int num_update_to_promote = GetNumUpdatesForPromote(track, pose);
      const int valid_measurement_num =
          GetConfirmendMeasurementNumInAllowedDuration(track, pose,
                                                       current_timestamp);
      return valid_measurement_num >= num_update_to_promote;
    }
    case TrackLifeState::kLost: {
      // Count latest 0.3s measurements.
      constexpr double kDurationForMeasurementCount = 0.3;  // s
      const int measurement_num = GetMeasurementNum(
          track, current_timestamp, kDurationForMeasurementCount);
      return measurement_num > 0;
    }
    case TrackLifeState::kConfirmed:
      return true;
    case TrackLifeState::kIdle:
      return false;
  }
  return false;
}

}  // namespace qcraft::tracker::track_life_manager
