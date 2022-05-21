#include "onboard/perception/tracker/tracker_output_guarder.h"

namespace qcraft::tracker::tracker_output_guarder {

TrackState::MovingState TrackerOutputGuarder::Guard(
    const VehiclePose& pose,
    const mapping::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& coordinate_converter,
    const Track<TrackState>& track,
    TrackerDebugProto::OutputGuarderDebugInfoProto* guarder_debug_proto) const {
  // We do not guard tracks that are from split or merge.
  if (track.track_state.merged_by_other_track ||
      track.track_state.split_from_other_track) {
    if (guarder_debug_proto) {
      guarder_debug_proto->set_moving_state_debug(
          motion_static_classifier::MotionStaticClassifier::DebugToString(
              motion_static_classifier::MotionStaticClassifier::kNoJudge));
      guarder_debug_proto->set_shooter_state_debug(
          shooter_killer::ShooterKiller::DebugToString(
              shooter_killer::ShooterKiller::kNoJudge));
    }
    return track.track_state.moving_state;
  }
  const auto [moving_state, moving_state_debug] =
      motion_static_classifier_.MovingOrStatic(pose, semantic_map_manager,
                                               coordinate_converter, track);
  if (guarder_debug_proto) {
    guarder_debug_proto->set_moving_state_debug(
        motion_static_classifier::MotionStaticClassifier::DebugToString(
            moving_state_debug));
  }

  if (motion_static_classifier::STATIC == moving_state) {
    return TrackState::MovingState::kStatic;
  } else {
    // Moving or Unknown
    const auto [is_shooter, shooter_debug] =
        shooter_killer_.IsShooter(pose, track);
    if (guarder_debug_proto) {
      guarder_debug_proto->set_shooter_state_debug(
          shooter_killer::ShooterKiller::DebugToString(shooter_debug));
    }
    if (is_shooter) {
      return TrackState::MovingState::kStatic;
    } else {
      return TrackState::MovingState::kMoving;
    }
  }
}

}  // namespace qcraft::tracker::tracker_output_guarder
