#include "onboard/perception/tracker/motion_static_classifier.h"

#include <algorithm>
#include <limits>

#include "onboard/math/geometry/util.h"
#include "onboard/perception/tracker/tracker_constant.h"
#include "onboard/perception/tracker/tracker_util.h"

namespace qcraft::tracker::motion_static_classifier {

bool IsSlowMovingVehicle(const Track<TrackState>& track, double timestamp) {
  if (track.track_state.type != MT_VEHICLE) {
    return false;
  }
  // Use icp velocity to check slow moving.
  const auto& m_history = track.measurement_history;
  const double last_m_timestamp = m_history.back_time();
  for (int j = m_history.size() - 1;
       j > m_history.GetIndexWithTimeAtLeast(last_m_timestamp - 0.3); --j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_icp_measurement() &&
        m->laser_measurement().icp_measurement().has_vel()) {
      const Vec2d icp_vel =
          Vec2dFromProto(m->laser_measurement().icp_measurement().vel());
      constexpr double kMaxStaticObjectIcpSpeenNoiseSqr = Sqr(0.9);
      constexpr double kMaxSlowMovingIcpSpeedSqr = Sqr(1.5);
      if (icp_vel.squaredNorm() > kMaxStaticObjectIcpSpeenNoiseSqr &&
          icp_vel.squaredNorm() < kMaxSlowMovingIcpSpeedSqr) {
        return true;
      }
    }
  }

  // We can use the ref point measurement velocity to judge if the vehicle is
  // slow moving.
  double max_ref_point_speed = std::numeric_limits<double>::lowest();
  double min_ref_point_speed = std::numeric_limits<double>::max();
  const int checkpoints_num = track.checkpoints.size();
  int valid_num = 0;
  constexpr double kValidCheckPointsDuration = 0.5;  // s
  for (int j = track.checkpoints.GetIndexWithTimeAtLeast(
           timestamp - kValidCheckPointsDuration);
       j < checkpoints_num; ++j) {
    const TrackState& t0 = track.checkpoints.value(j);
    if (t0.ref_point_vel) {
      const auto speed = t0.ref_point_vel->dot(
          Vec2d::FastUnitFromAngle(track.track_state.heading));
      max_ref_point_speed = std::max(max_ref_point_speed, speed);
      min_ref_point_speed = std::min(min_ref_point_speed, speed);
      ++valid_num;
    }
  }
  // If the vehicle is slow moving, it has some features as follows:
  // 1. The speed should be in one direction in the duration.
  // 2. All speed should  greater than the noise threshold.
  // 3. All speed should less than a threshold.
  constexpr double kMaxStationaryVehicleSpeedNoise = 0.25;  // m/s
  constexpr double kMaxSpeedForSlowMovingVehicle = 1.5;     // m/s
  const bool is_in_one_direction =
      max_ref_point_speed * min_ref_point_speed > 0.0;
  const bool is_true_moving =
      std::fabs(max_ref_point_speed) > kMaxStationaryVehicleSpeedNoise &&
      std::fabs(min_ref_point_speed) > kMaxStationaryVehicleSpeedNoise;
  const bool is_slowing_moving =
      std::fabs(max_ref_point_speed) < kMaxSpeedForSlowMovingVehicle &&
      std::fabs(min_ref_point_speed) < kMaxSpeedForSlowMovingVehicle;
  if (valid_num > 1 && is_in_one_direction && is_true_moving &&
      is_slowing_moving) {
    return true;
  }
  return false;
}

bool MotionStaticClassifier::IsStaticByStaticObjectZone(
    const Track<TrackState>& track,
    const SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter) const {
  return tracker_util::IsInSpecialZone(
      semantic_map_manager, tracker_util::GetContour(track),
      coordinate_converter, mapping::PerceptionZoneProto::STATIC_OBJECT,
      kStaticObjectZoneOverlapMinRatio);
}

bool MotionStaticClassifier::IsStaticByType(
    const Track<TrackState>& track) const {
  return track.track_state.type == MT_STATIC_OBJECT ||
         track.track_state.type == MT_VEGETATION ||
         track.track_state.type == MT_BARRIER ||
         track.track_state.type == MT_CONE ||
         track.track_state.type == MT_MIST || track.track_state.type == MT_ROAD;
}

bool MotionStaticClassifier::IsStaticByNaiveTooSlowIsStaticPhilosophy(
    const Track<TrackState>& track) const {
  // If the vel of car model is less than kMaxStationaryVehicleSpeed, or the vel
  // of point model is less than kMaxStationaryNonVehicleSpeed, is static.
  constexpr double kMaxStationaryVehicleSpeed = 0.25;     // m/s
  constexpr double kMaxStationaryNonVehicleSpeed = 0.15;  // m/s
  const auto s = track.track_state.estimator_3d.GetStateData();
  if (track.track_state.estimator_3d.IsCarModel()) {
    // if (track.track_state.estimator.IsCarModel()) {
    if (std::fabs(s.GetVel()) < kMaxStationaryVehicleSpeed) {
      return true;
    }
  } else {
    if (s.GetVel() < kMaxStationaryNonVehicleSpeed) {
      return true;
    }
  }
  return false;
}

bool MotionStaticClassifier::IsMovingByNaiveRadarMeasurement(
    const VehiclePose& pose, const Track<TrackState>& track) const {
  // If the the track has reliable radar measurement velocity,
  // we classify static by radar velocity.
  const auto centroid = track.track_state.contour.centroid();
  const auto pose_transform_inv = pose.ToTransform().Inverse();
  const double kMinDistX = 5.0;  // m
  const auto obj_pos =
      pose_transform_inv.TransformPoint(Vec3d(centroid.x(), centroid.y(), 0.0));
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  const double last_m_timestamp = m_history.back_time();

  if (track.track_state.type == MT_VEHICLE &&
      std::abs(obj_pos(0)) > kMinDistX) {
    constexpr double kDurationAsValidMeasurement = 0.3;  // s
    constexpr double kOneQuartersPi = 0.25 * M_PI;
    constexpr double kRadarReliableAreaAngle = 0.5 * kOneQuartersPi;

    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto& m = m_history.value(j);
      if (m->has_radar_measurement()) {
        // NOTE(zheng): If the radar measurement is right side or left side of
        // the av, the radar measurement velocity is not accurate.
        const double obj_body_pos_angle =
            Vec2dFromProto(m->radar_measurement().radar_coord_pos())
                .FastAngle();
        const auto radar_vel =
            Vec2dFromProto(m->radar_measurement().vel()).squaredNorm();
        constexpr double kMaxStationaryVehicleSpeed = 0.25;  // m/s
        if (radar_vel > kMaxStationaryVehicleSpeed &&
            std::abs(obj_body_pos_angle) > kRadarReliableAreaAngle &&
            std::abs(obj_body_pos_angle) < (M_PI - kRadarReliableAreaAngle)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool MotionStaticClassifier::IsStaticByNaiveFenVelocity(
    const Track<TrackState>& track) const {
  // NOTE(jingwei) This function works especially when bbox jumps.
  // Fen velocity is accurate when its value is zero except
  // slowing moving vehicles. To avoid we classify the slow moving vehicle to
  // static, we should judge if the vehicle is slow moving.
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  const double last_m_timestamp = m_history.back_time();
  if (!IsSlowMovingVehicle(track, last_m_timestamp)) {
    constexpr double kMaxStationaryVehicleSpeed = 0.25;  // m/s
    constexpr double kMaxStaticObjectFenSpeenNoiseSqr =
        Sqr(kMaxStationaryVehicleSpeed);                      // m/s
    constexpr double kValidFenVelMeasurementDuration = 0.30;  // s
    for (int j = m_history.GetIndexWithTimeAtLeast(
             last_m_timestamp - kValidFenVelMeasurementDuration);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement() &&
          m->laser_measurement().has_fen_velocity()) {
        const Vec2d fen_vel =
            Vec2dFromProto(m->laser_measurement().fen_velocity());
        // TODO(jingwei) remove using track.track_state.moving_state.
        if (track.track_state.moving_state ==
                TrackState::MovingState::kStatic &&
            fen_vel.squaredNorm() < kMaxStaticObjectFenSpeenNoiseSqr) {
          return true;
        }
      }
    }
  }
  return false;
}

MotionStaticClassifier::MovingStateDebug MotionStaticClassifier::MovingOrStatic(
    const VehiclePose& pose,
    const mapping::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter,
    const Track<TrackState>& track) const {
  if (IsStaticByStaticObjectZone(track, semantic_map_manager,
                                 coordinate_converter)) {
    return {STATIC, kIsStaticByStaticObjectZone};
  }
  if (IsStaticByType(track)) {
    return {STATIC, kIsStaticByType};
  }

  // We can not judge the moving state by by using track sequence info
  // at the beginning of the track, so we set the moving state to unknown.
  constexpr int kMinCheckPointsNumForMovingStateJudgement = 3;
  if (track.checkpoints.size() < kMinCheckPointsNumForMovingStateJudgement &&
      track.track_state.moving_state == TrackState::MovingState::kUnKnown) {
    return {UNKNOWN, kIsUnknownByTrackIsAtBegining};
  }

  if (IsStaticByNaiveTooSlowIsStaticPhilosophy(track)) {
    return {STATIC, kIsStaticByNaiveTooSlowIsStaticPhilosophy};
  }
  if (IsMovingByNaiveRadarMeasurement(pose, track)) {
    return {MOVING, kIsMovingByNaiveRadarMeasurementa};
  }
  if (IsStaticByNaiveFenVelocity(track)) {
    return {STATIC, kIsStaticByNaiveFenVelocity};
  }

  return {UNKNOWN, kIsUnknownByDoNotHaveEnoughInformation};
}

}  // namespace qcraft::tracker::motion_static_classifier
