#include "onboard/perception/tracker/track.h"

#include <utility>

#include "absl/strings/str_format.h"
#include "onboard/math/coordinate_converter.h"

namespace qcraft::tracker {
std::string TrackState::DebugString() const {
  const bool is_radar_only =
      measurement_source_type == TrackMeasurementSourceType::TMST_RO;
  std::string debug_string = absl::StrFormat(
      "\nTrack:\n id: %d\n type: %s\n ref_point: %s\n last time: %.4f\n "
      "offroad: %d\n vel: %f\n "
      "park: %d\n life_state: %s\n merged: %d\n splited: %d\n"
      "estimator: %s\n "
      "motion filter param path: %s\n "
      "moving_state: %s\n"
      "is_certain_static_track: %d\n"
      "is_radar_only: %d\n"
      "measurement_source_type: %s\n ",
      id, MeasurementType_Name(type), RefPointTypeToString(ref_point.type),
      last_timestamp, offroad, vel, in_parking_area,
      TrackLifeState_Name(life_state), merged_by_other_track,
      split_from_other_track, MotionFilterTypeToString(motion_filter_type),
      GetMotionFilterParamPathByType(motion_filter_param_type),
      MovingStateToString(moving_state), is_certain_static_track, is_radar_only,
      TrackMeasurementSourceType_Name(measurement_source_type));

  return debug_string;
}

std::string TrackDebugInfo::DebugString() const {
  std::string state_pos_reset_info =
      state_pos_reset_offset
          ? absl::StrFormat("state pos reset offset: %.2f, %.2f \n",
                            state_pos_reset_offset->x(),
                            state_pos_reset_offset->y())
          : "No state pos reset. \n";

  std::string debug_string = absl::StrFormat(
      "use_camera: %d\n"
      "use_icp_vel: %d\n"
      "use_radar_vel: %d\n"
      "use_fen_vel: %d\n"
      "%s \n",
      updated_with_camera, updated_with_icp_velocity,
      updated_with_radar_velocity, updated_with_fen_velocity,
      state_pos_reset_info);
  return debug_string;
}

std::string RefPointTypeToString(const TrackState::RefPoint::Type type) {
  switch (type) {
    case TrackState::RefPoint::kNone:
      return "kNone";
    case TrackState::RefPoint::kBBCenter:
      return "kBBCenter";
    case TrackState::RefPoint::kContourCentroid:
      return "kContourCentroid";
    case TrackState::RefPoint::kTopRightCorner:
      return "kTopRightCorner";
    case TrackState::RefPoint::kTopLeftCorner:
      return "kTopLeftCorner";
    case TrackState::RefPoint::kBottomLeftCorner:
      return "kBottomLeftCorner";
    case TrackState::RefPoint::kBottomRightCorner:
      return "kBottomRightCorner";
    case TrackState::RefPoint::kFrontFaceCenter:
      return "kFrontFaceCenter";
    case TrackState::RefPoint::kLeftFaceCenter:
      return "kLeftFaceCenter";
    case TrackState::RefPoint::kRearFaceCenter:
      return "kRearFaceCenter";
    case TrackState::RefPoint::kRightFaceCenter:
      return "kRightFaceCenter";
    case TrackState::RefPoint::kNearestObstacleCenter:
      return "kNearestObstacleCenter";
    case TrackState::RefPoint::kWeightedObstacleCentroid:
      return "kNearestObstacleCenter";
    default:
      return "Unknown ref point type.";
  }
}

std::string MovingStateToString(const TrackState::MovingState moving_state) {
  switch (moving_state) {
    case TrackState::MovingState::kUnKnown:
      return "kUnKnown";
    case TrackState::MovingState::kStatic:
      return "kStatic";
    case TrackState::MovingState::kMoving:
      return "kMoving";
    default:
      return "Unknown moving state.";
  }
}

std::string MotionFilterTypeToString(const TrackState::MotionFilterType type) {
  switch (type) {
    case TrackState::MotionFilterType::kCarModel:
      return "car model";
    case TrackState::MotionFilterType::kPointModelCA:
      return "point model CA";
    case TrackState::MotionFilterType::kPointModelCP:
      return "point model CP";
    default:
      return "unknown motion filter type";
  }
}

// NOTE(jiawei): Temporary solution for output motion filter params path.
std::string GetMotionFilterParamPathByType(const MotionFilterParamType type) {
  switch (type) {
    // Compute given type corner ref point.
    case MFPT_HIGHWAY_CAR_MODEL_CTRA_CAR:
      return "highway_car_model_ctra_car.pb.txt";
    case MFPT_HIGHWAY_CAR_MODEL_CTRA_CYC:
      return "highway_car_model_ctra_cyc.pb.txt";
    case MFPT_URBAN_CAR_MODEL_CTRA_CAR:
      return "urban_car_model_ctra_car.pb.txt";
    case MFPT_URBAN_CAR_MODEL_CTRA_CYC:
      return "urban_car_model_ctra_cyc.pb.txt";
    case MFPT_POINT_MODEL_CA:
      return "point_model.pb.txt";
    case MFPT_POINT_MODEL_CP:
      return "point_model.pb.txt";
    default:
      return "unknown motion filter param type";
  }
}
}  // namespace qcraft::tracker
