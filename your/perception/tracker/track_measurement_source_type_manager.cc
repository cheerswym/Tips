#include "onboard/perception/tracker/track_measurement_source_type_manager.h"

#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/obstacle_constants.h"
#include "onboard/perception/tracker/tracker_constant.h"
#include "onboard/utils/map_util.h"

namespace qcraft::tracker::track_measurement_source_type_manager {

// If the trans map don't contain a key, means the type needn't to be
// transformed.
const std::map<std::pair<TMST, MP::MeasurementCase>, TMST>
    TrackMeasurementSourceTypeManager::kMeasurementSourceTypeTransMap = {
        {{TMST::TMST_LO, MP::kCamera3DMeasurement}, TMST::TMST_LV},
        {{TMST::TMST_LO, MP::kCameraMeasurement}, TMST::TMST_LV},
        {{TMST::TMST_LO, MP::kRadarMeasurement}, TMST::TMST_LR},
        {{TMST::TMST_VO, MP::kLaserMeasurement}, TMST::TMST_LV},
        {{TMST::TMST_VO, MP::kRadarMeasurement}, TMST::TMST_VR},
        {{TMST::TMST_RO, MP::kLaserMeasurement}, TMST::TMST_LR},
        {{TMST::TMST_RO, MP::kCamera3DMeasurement}, TMST::TMST_VR},
        {{TMST::TMST_RO, MP::kCameraMeasurement}, TMST::TMST_VR},
        {{TMST::TMST_LV, MP::kRadarMeasurement}, TMST::TMST_LVR},
        {{TMST::TMST_LR, MP::kCamera3DMeasurement}, TMST::TMST_LVR},
        {{TMST::TMST_LR, MP::kCameraMeasurement}, TMST::TMST_LVR},
        {{TMST::TMST_VR, MP::kLaserMeasurement}, TMST::TMST_LVR}};

const std::map<int, TMST> TrackMeasurementSourceTypeManager::
    kMeasurementSourceTypeInMeasurementHistory{
        {0b0100, TMST::TMST_LO},  {0b0110, TMST::TMST_LV},
        {0b0111, TMST::TMST_LVR}, {0b0101, TMST::TMST_LR},
        {0b0010, TMST::TMST_VO},  {0b0011, TMST::TMST_VR},
        {0b0001, TMST::TMST_RO}};

const std::unordered_set<LidarModel>
    TrackMeasurementSourceTypeManager::kPbqLidarList = {LIDAR_RS_M1,
                                                        LIDAR_PANDAR_AT128};

TrackMeasurementSourceTypeManager::TrackMeasurementSourceTypeManager(
    const RunParamsProtoV2& run_params) {
  const auto& lidar_params = run_params.vehicle_params().lidar_params();
  for (const auto& lidar_param : lidar_params) {
    if (kPbqLidarList.find(lidar_param.model()) != kPbqLidarList.end()) {
      const auto lidar2vehicle_transform = AffineTransformation::FromExtrinsics(
          lidar_param.installation().extrinsics());
      pbq_vehicle2lidar_trans_[lidar_param.installation().lidar_id()] =
          lidar2vehicle_transform.Inverse();
    }
  }
}

TMST TrackMeasurementSourceTypeManager::ComputeTrackMeasurementSourceType(
    const Track<TrackState>& track, const MeasurementProto& m) {
  // NOTE(zheng): We may only need to consider the latest 1s measurement
  // history.
  const auto key = std::make_pair(track.track_state.measurement_source_type,
                                  m.Measurement_case());
  if (kMeasurementSourceTypeTransMap.find(key) ==
      kMeasurementSourceTypeTransMap.end()) {
    return track.track_state.measurement_source_type;
  } else {
    return kMeasurementSourceTypeTransMap.at(key);
  }
}

TMST TrackMeasurementSourceTypeManager::
    ComputeTrackMeasurementSourceTypeByMeasurementHistory(
        const Track<TrackState>& track) {
  QCHECK(!track.measurement_history.empty());
  int has_lidar_measurement = 0b0000;   // 0b0100 means true;
  int has_vision_measurement = 0b0000;  // 0b0010 means true;
  int has_radar_measurement = 0b0000;   // 0b0001 means true;
  for (const auto& [ts, m] : track.measurement_history) {
    const double time_diff = track.track_state.state_timestamp - ts;
    if (time_diff > kTimeDurationForSourceTypeComputing) {
      continue;
    }
    if (m->has_laser_measurement()) {
      has_lidar_measurement = 0b0100;
    } else if (m->has_camera_measurement()) {
      has_vision_measurement = 0b0010;
    } else if (m->has_camera3d_measurement()) {
      has_vision_measurement = 0b0010;
    } else if (m->has_radar_measurement()) {
      has_radar_measurement = 0b0001;
    }
  }
  const int source_type_key =
      has_lidar_measurement | has_vision_measurement | has_radar_measurement;
  return FindOrDie(kMeasurementSourceTypeInMeasurementHistory, source_type_key);
}
// Determine the transition from LVR to VR
bool TrackMeasurementSourceTypeManager::IsSwitchToNoLidarTrack(TMST prev,
                                                               TMST curr) {
  return HasLidarMeasurements(prev) && (!HasLidarMeasurements(curr));
}

// Determine the transition from VR to LVR
bool TrackMeasurementSourceTypeManager::IsSwitchToLidarTrack(TMST prev,
                                                             TMST curr) {
  return (!HasLidarMeasurements(prev)) && HasLidarMeasurements(curr);
}

bool TrackMeasurementSourceTypeManager::HasLidarMeasurements(TMST source_type) {
  return source_type == TMST::TMST_LO || source_type == TMST::TMST_LV ||
         source_type == TMST::TMST_LR || source_type == TMST::TMST_LVR;
}

bool TrackMeasurementSourceTypeManager::HasLidarMeasurements(
    const Track<TrackState>& track) {
  const auto source_type = track.track_state.measurement_source_type;
  return HasLidarMeasurements(source_type);
}

bool TrackMeasurementSourceTypeManager::HasCameraMeasurements(
    const Track<TrackState>& track) {
  const auto source_type = track.track_state.measurement_source_type;
  return source_type == TMST::TMST_VO || source_type == TMST::TMST_LV ||
         source_type == TMST::TMST_VR || source_type == TMST::TMST_LVR;
}

bool TrackMeasurementSourceTypeManager::ShouldCreateTrackFromRadar(
    const MeasurementProto& m, const VehiclePose& pose,
    const bool is_first_frame) {
  // Do not create radar only track in first frame.
  if (is_first_frame || !m.has_radar_measurement()) {
    return false;
  }
  const auto& radar_obj = m.radar_measurement();
  // NOTE(zheng):Nowadays, we only create radar only track when the
  // radar measurement statisfy all the 3 conditions:
  // 1. the radar measurement is right behind or right front of the av.
  // 2. the radar measurement's speed is greater than a
  // threshold and it velocity direction has a small angle
  // with av x axis.
  // 3. the radar measurement is out of the obstacle detection range.
  constexpr double kMinRadarVelToCreateTrack = 5.0;  // m/s
  if (Vec2dFromProto(radar_obj.vel()).squaredNorm() <
      Sqr(kMinRadarVelToCreateTrack)) {
    return false;
  }

  const double vel_angle_with_av_dir =
      NormalizeAngle(Vec2dFromProto(radar_obj.vel()).FastAngle() - pose.yaw);
  if (std::abs(vel_angle_with_av_dir) > kOneQuartersPi &&
      std::abs(vel_angle_with_av_dir) < kThreeQuartersPi) {
    return false;
  }

  // Note(zheng): If the radar measurement is right side or left side of the
  // av, the radar measurement velocity is not accurate.
  const Vec2d obj_body_pos = Vec2dFromProto(radar_obj.body_pos());
  const double obj_body_pos_angle = NormalizeAngle(obj_body_pos.FastAngle());
  if (std::abs(obj_body_pos_angle) < kThreeQuartersPi &&
      std::abs(obj_body_pos_angle) > kOneQuartersPi) {
    return false;
  }

  // Don't create radar only track in obstacle detection range.
  if (obj_body_pos.x() < kObstacleDetectionFrontDist &&
      obj_body_pos.x() > -kObstacleDetectionRearDist) {
    return false;
  }

  return true;
}

bool TrackMeasurementSourceTypeManager::ShouldCreateTrackFromVision(
    const MeasurementProto& m, const VehiclePose& pose,
    const AffineTransformation& pose_inv, const bool is_first_frame) {
  if (is_first_frame || !m.has_camera3d_measurement()) {
    return false;
  }

  const Vec2d m_pos_smooth = Vec2dFromProto(m.camera3d_measurement().pos());
  const auto& m_pos_vehicle =
      pose_inv.TransformPoint({m_pos_smooth.x(), m_pos_smooth.y(), pose.z});

  if (!IsDBQConext()) {
    constexpr double kMaxRsM1LidarViewFiledAngle = M_PI * 0.3333;
    for (const auto& [_, trans] : pbq_vehicle2lidar_trans_) {
      const auto m_pos_lidar = trans.TransformPoint(m_pos_vehicle);
      const double obj_lidar_pos_angle =
          NormalizeAngle(m_pos_lidar.xy().FastAngle());
      if (std::abs(obj_lidar_pos_angle) < kMaxRsM1LidarViewFiledAngle &&
          m_pos_lidar.x() < kObstacleDetectionFrontDist &&
          m_pos_lidar.x() > -kObstacleDetectionRearDist) {
        return false;
      }
    }
  } else {
    // TODO(zheng): Add DBQ camera track create strategy.
    return false;
  }
  return true;
}

}  // namespace qcraft::tracker::track_measurement_source_type_manager
