#include "onboard/perception/tracker/shooter_killer.h"

#include <algorithm>

#include "onboard/eval/qevent.h"

namespace qcraft::tracker::shooter_killer {

std::optional<Vec2d> GetBboxCenter(
    const LaserMeasurementProto& laser_measurement, bool use_min_area_bbox) {
  if (!use_min_area_bbox && !laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  } else if (use_min_area_bbox &&
             !laser_measurement.has_min_area_bounding_box()) {
    return std::nullopt;
  }
  const auto& box2d = use_min_area_bbox
                          ? laser_measurement.min_area_bounding_box()
                          : laser_measurement.detection_bounding_box();
  return Vec2d(box2d.x(), box2d.y());
}

bool ShooterKiller::IsShooterByMaxAccCheck(
    const Track<TrackState>& track) const {
  // 1. Max acc sanity check: we suppose the true moving vechile acc is below
  // a threshold, change moving state directly to static if acc is invalid.
  constexpr double kMaxAccAllowedForVehicle = 5.0;  // m/s^2
  // Use filtered speed difference to compute acc, and get the max acc in a
  // window.
  double max_acc = 0.0;
  // Window size for getting acc statistics.
  constexpr int kWindowSize = 5;  // 5 frames.
  // We skip first kMinCheckPointsNumForMovingStateJudgement checkpoints,
  // because the speed is not stable at first.
  constexpr int kMinCheckPointsNumForMovingStateJudgement = 3;
  const int checkpoints_num = track.checkpoints.size();
  const int checkpoints_begin_index = std::max(
      kMinCheckPointsNumForMovingStateJudgement, checkpoints_num - kWindowSize);
  for (int j = checkpoints_begin_index; j < checkpoints_num - 1; ++j) {
    const TrackState& t0 = track.checkpoints.value(j);
    for (int k = j + 1; k < checkpoints_num; ++k) {
      const TrackState& t1 = track.checkpoints.value(k);
      const double time_diff = t0.state_timestamp - t1.state_timestamp;
      // A bigger time interval is more robust.
      if (fabs(time_diff) < 0.2) {
        continue;
      }
      if (t0.estimator_3d.IsCarModel() && t1.estimator_3d.IsCarModel()) {
        const auto s0 = t0.estimator_3d.GetStateData();
        const auto s1 = t1.estimator_3d.GetStateData();
        const double acc =
            (s0.GetVel() - s1.GetVel()) /
            (std::fabs(t0.state_timestamp - t1.state_timestamp) + DBL_EPSILON);
        max_acc = std::max(std::fabs(acc), max_acc);
        break;
      }
    }
  }
  if (max_acc > kMaxAccAllowedForVehicle) {
    QEVENT("jingwei", "IsShooterByMaxAccCheck", [=](QEvent* qevent) {
      qevent->AddField("max_acc", max_acc)
          .AddField("acc_threshold", kMaxAccAllowedForVehicle)
          .AddField("track.track_state.id",
                    static_cast<int>(track.track_state.id))
          .AddField("ckp_num", checkpoints_num);
    });
    return true;
  }

  return false;
}

bool ShooterKiller::IsShooterByBboxCenterAverageMovingSantityCheck(
    const Track<TrackState>& track) const {
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  if (num_m == 0) return false;

  constexpr double kDurationAsValidMeasurement = 1.0;  // s
  int oldest_laser_measurement_index = 0;
  int latest_laser_measurement_index = num_m - 1;
  const double last_m_timestamp = m_history.back_time();
  // Get oldest laser measurement index.
  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidMeasurement);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      oldest_laser_measurement_index = j;
      break;
    }
  }
  // Get latest laser measurement index.
  for (int j = num_m - 1;
       j >= m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                              kDurationAsValidMeasurement);
       --j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      latest_laser_measurement_index = j;
      break;
    }
  }
  if (oldest_laser_measurement_index == latest_laser_measurement_index)
    return false;
  // Get oldest laser measurement detection bbox center and min area bbox
  // center.
  const auto& oldest_laser_m =
      m_history.value(oldest_laser_measurement_index)->laser_measurement();
  const auto& oldest_laser_m_timestamp =
      m_history.time(oldest_laser_measurement_index);
  const auto oldest_min_area_ref_point_or =
      GetBboxCenter(oldest_laser_m,
                    /*use_min_area_bbox=*/true);
  const auto oldest_detection_ref_point_or =
      GetBboxCenter(oldest_laser_m,
                    /*use_min_area_bbox*/ false);

  // Get latest laser measurement detection bbox center and min area bbox
  // center.
  const auto& latest_laser_m =
      m_history.value(latest_laser_measurement_index)->laser_measurement();
  const auto& latest_laser_m_timestamp =
      m_history.time(latest_laser_measurement_index);
  const auto latest_min_area_ref_point_or =
      GetBboxCenter(latest_laser_m,
                    /*use_min_area_bbox=*/true);
  const auto latest_detection_ref_point_or =
      GetBboxCenter(latest_laser_m,
                    /*use_min_area_bbox*/ false);

  constexpr double kMaxSpeedMeasurementNoise = 0.125;  // m/s
  // Compute detection bbox center moving average speed by using the movement.
  if (latest_detection_ref_point_or && oldest_detection_ref_point_or) {
    const auto detection_ref_point_velocity_measurement =
        (*latest_detection_ref_point_or - *oldest_detection_ref_point_or) /
        (latest_laser_m_timestamp - oldest_laser_m_timestamp + DBL_EPSILON);

    // If the speed is below than threshold, we cap the
    // velocity to zero.
    if (detection_ref_point_velocity_measurement.norm() <
        kMaxSpeedMeasurementNoise) {
      QEVENT("jingwei", "IsShooterByBboxCenterAverageMovingSantityCheck",
             [=](QEvent* qevent) {
               qevent
                   ->AddField("detection_ref_point_velocity_measurement",
                              detection_ref_point_velocity_measurement.norm())
                   .AddField("threshold", kMaxSpeedMeasurementNoise)
                   .AddField("track.track_state.type",
                             static_cast<int>(track.track_state.type))
                   .AddField("track.track_state.id",
                             static_cast<int>(track.track_state.id));
             });
      return true;
    }
  }
  // Compute min area bbox center moving average speed by using the movement.
  if (latest_min_area_ref_point_or && oldest_min_area_ref_point_or) {
    const auto min_area_ref_point_velocity_measurement =
        (*latest_min_area_ref_point_or - *oldest_min_area_ref_point_or) /
        (latest_laser_m_timestamp - oldest_laser_m_timestamp + DBL_EPSILON);

    // If the speed is below than kMaxStationaryVehicleSpeed, we cap the
    // velocity to zero.
    if (min_area_ref_point_velocity_measurement.norm() <
        kMaxSpeedMeasurementNoise) {
      QEVENT("jingwei", "IsShooterByBboxCenterAverageMovingSantityCheck",
             [=](QEvent* qevent) {
               qevent
                   ->AddField("min_area_ref_point_velocity_measurement",
                              min_area_ref_point_velocity_measurement.norm())
                   .AddField("threshold", kMaxSpeedMeasurementNoise)
                   .AddField("track.track_state.type",
                             static_cast<int>(track.track_state.type))
                   .AddField("track.track_state.id",
                             static_cast<int>(track.track_state.id));
             });
      return true;
    }
  }

  return false;
}

bool ShooterKiller::IsShooterByDetBboxCenterMovingConsistencyCheck(
    const Track<TrackState>& track) const {
  // We suppose the detection bbox center movement is consistent when the object
  // is true moving.
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  const double last_m_timestamp = m_history.back_time();
  int oldest_laser_measurement_index = 0;
  int latest_laser_measurement_index = num_m - 1;
  // Get oldest laser measurement index.
  constexpr double kDurationAsValidMeasurement = 1.0;  // s
  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidMeasurement);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      oldest_laser_measurement_index = j;
      break;
    }
  }
  // Get latest laser measurement index.
  for (int j = num_m - 1;
       j >= m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                              kDurationAsValidMeasurement);
       --j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_min_area_bounding_box() &&
        m->laser_measurement().has_detection_bounding_box()) {
      latest_laser_measurement_index = j;
      break;
    }
  }

  const auto& latest_laser_m =
      m_history.value(latest_laser_measurement_index)->laser_measurement();
  const auto latest_detection_ref_point_or =
      GetBboxCenter(latest_laser_m,
                    /*use_min_area_bbox*/ false);
  const auto& oldest_laser_m =
      m_history.value(oldest_laser_measurement_index)->laser_measurement();
  const auto oldest_detection_ref_point_or =
      GetBboxCenter(oldest_laser_m,
                    /*use_min_area_bbox*/ false);

  // TODO(zheng): Handle the detection lost situation.
  constexpr double kMinTimeInterval = 0.5;
  if (latest_detection_ref_point_or && oldest_detection_ref_point_or) {
    // The center movement from oldest laser measurement to latest laser
    // measuremnet.
    const Vec2d oldest2latest_center_movement =
        *latest_detection_ref_point_or - *oldest_detection_ref_point_or;
    const double oldest2latest_center_moving_dist =
        oldest2latest_center_movement.norm();

    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement() &&
          m->laser_measurement().has_detection_bounding_box()) {
        const auto laser_m = m->laser_measurement();
        // Compute current frame laser measurement detection bbox center.
        const auto curr_detection_ref_point_or =
            GetBboxCenter(laser_m,
                          /*use_min_area_bbox*/ false);
        if (curr_detection_ref_point_or) {
          // Compute current frame detection bbox center movement.
          const Vec2d latest2current_center_movement =
              *curr_detection_ref_point_or - *oldest_detection_ref_point_or;
          const double latest2current_center_moving_dist =
              latest2current_center_movement.norm();
          // Only the two measurements have enough time interval, we perform
          // measurement moving coincident sanity check.
          const double time_diff =
              m->timestamp() - m_history.time(oldest_laser_measurement_index);
          if (time_diff > kMinTimeInterval) {
            if (oldest2latest_center_moving_dist < DBL_EPSILON ||
                latest2current_center_moving_dist < DBL_EPSILON) {
              QEVENT("jingwei",
                     "IsShooterByDetBboxCenterMovingConsistencyCheck",
                     [=](QEvent* qevent) {
                       qevent
                           ->AddField("oldest2latest_center_moving_dist",
                                      oldest2latest_center_moving_dist)
                           .AddField("latest2current_center_moving_dist",
                                     latest2current_center_moving_dist)
                           .AddField("track.track_state.type",
                                     static_cast<int>(track.track_state.type))
                           .AddField("track.track_state.id",
                                     static_cast<int>(track.track_state.id));
                     });

              return true;
            }

            // Compute the angle between the center movement, if the angle
            // greater than a threshold, we suppose the speed is a shooter,
            // because for vehicle/cyc the movement should not change so
            // large.
            constexpr double kMaxAngleChange = M_PI / 2.0;
            const auto angle = std::acos(oldest2latest_center_movement.dot(
                                             latest2current_center_movement) /
                                         (oldest2latest_center_moving_dist *
                                          latest2current_center_moving_dist));

            if (angle > kMaxAngleChange) {
              QEVENT("jingwei",
                     "IsShooterByDetBboxCenterMovingConsistencyCheck",
                     [=](QEvent* qevent) {
                       qevent->AddField("angle", angle)
                           .AddField("threshold", kMaxAngleChange)
                           .AddField("track.track_state.type",
                                     static_cast<int>(track.track_state.type))
                           .AddField("track.track_state.id",
                                     static_cast<int>(track.track_state.id));
                     });

              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool ShooterKiller::IsShooterByHugeUnknownCheck(
    const Track<TrackState>& track) const {
  // BANDAID(yu): Set the speed to zero for huge unknown objects (which are
  // normally shooters on offroad objects).
  if (track.track_state.type == MT_UNKNOWN) {
    constexpr double kMinAreaForUnknownToConsiderSpeedRemoving = 50.0;  // m^2
    const auto area = track.track_state.contour.area();
    if (area > kMinAreaForUnknownToConsiderSpeedRemoving) {
      QEVENT("jingwei", "IsShooterByHugeUnknownCheck", [=](QEvent* qevent) {
        qevent->AddField("area", area)
            .AddField("threshold", kMinAreaForUnknownToConsiderSpeedRemoving)
            .AddField("track.track_state.type",
                      static_cast<int>(track.track_state.type))
            .AddField("track.track_state.id",
                      static_cast<int>(track.track_state.id));
      });
      return true;
    }
  }
  return false;
}

bool ShooterKiller::IsShooterByHugeSpeed(const Track<TrackState>& track) const {
  const auto state = track.track_state.estimator_3d.GetStateData();
  constexpr double kFastestCarSpeedOnRoad = 55.;  // 55m/s is 198km/h
  if (state.GetVel() > kFastestCarSpeedOnRoad) {
    QEVENT("jingwei", "IsShooterByHugeSpeed", [=](QEvent* qevent) {
      qevent->AddField("track.track_state.vel", state.GetVel())
          .AddField("threshold", kFastestCarSpeedOnRoad)
          .AddField("track.track_state.id",
                    static_cast<int>(track.track_state.id));
    });
    return true;
  }

  return false;
}

// NOTE(zheng): 1. We only process the vehicle/cyc shooters, the unknown
// object's shooter case are handled by const position motion filter,
// the pedestrian's shooter case is a todo list.
// 2. Based on observation, most shooters' moving state is static in history,
// so to avoid classify the true moving object to static state, we only
// process the objects which are static in history.
ShooterKiller::ShooterDebugResult ShooterKiller::IsShooter(
    const VehiclePose& pose, const Track<TrackState>& track) const {
  // TODO(jingwei) This are old code, very confusing, try to remove.
  if (track.track_state.moving_state != TrackState::MovingState::kStatic ||
      !track.track_state.estimator_3d.IsCarModel()) {
    return {false, kNotInShootRangeOfShooterKiller};
  }

  if (IsShooterByHugeSpeed(track)) {
    return {true, kIsShooterByHugeSpeed};
  }

  if (IsShooterByMaxAccCheck(track)) {
    return {true, kIsShooterByMaxAccCheck};
  }
  if (IsShooterByBboxCenterAverageMovingSantityCheck(track)) {
    return {true, kIsShooterByBboxCenterAverageMovingSantityCheck};
  }
  if (IsShooterByDetBboxCenterMovingConsistencyCheck(track)) {
    return {true, kIsShooterByDetBboxCenterMovingConsistencyCheck};
  }
  if (IsShooterByHugeUnknownCheck(track)) {
    return {true, kIsShooterByHugeUnknownCheck};
  }

  return {false, kNoEvidenceAsShooter};
}

}  // namespace qcraft::tracker::shooter_killer
