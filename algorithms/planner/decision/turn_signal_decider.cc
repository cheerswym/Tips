#include "onboard/planner/decision/turn_signal_decider.h"

#include <algorithm>
#include <optional>

#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kPreviewDirectionDist = 15.0;  // m.

bool HasReachedDestination(const PlannerSemanticMapManager &psmm,
                           const RouteSections &route_sections,
                           const mapping::LanePath &lane_path) {
  if (lane_path.lane_ids().back() != route_sections.destination().lane_id()) {
    return false;
  }

  constexpr double kEpsilon = 0.5;  // m.
  const auto &last_sec = route_sections.back();
  const double last_sec_length =
      psmm.FindSectionInfoOrDie(last_sec.id).average_length;
  return last_sec_length *
             std::abs(last_sec.end_fraction - lane_path.end_fraction()) <
         kEpsilon;
}

bool HasMapDictatedTurnSignal(const PlannerSemanticMapManager &psmm,
                              const mapping::LanePath &lane_path,
                              TurnSignal *signal) {
  const mapping::ElementId current_lane_id = lane_path.front().lane_id();
  const double current_lane_fraction = lane_path.front().fraction();
  const mapping::LaneProto &current_lane_proto =
      psmm.FindLaneByIdOrDie(current_lane_id);

  const bool left_on =
      current_lane_proto.has_require_left_turn_signal() &&
      (!current_lane_proto.require_left_turn_signal().has_start_fraction() ||
       current_lane_fraction >=
           current_lane_proto.require_left_turn_signal().start_fraction()) &&
      (!current_lane_proto.require_left_turn_signal().has_end_fraction() ||
       current_lane_fraction <=
           current_lane_proto.require_left_turn_signal().end_fraction());
  const bool right_on =
      current_lane_proto.has_require_right_turn_signal() &&
      (!current_lane_proto.require_right_turn_signal().has_start_fraction() ||
       current_lane_fraction >=
           current_lane_proto.require_right_turn_signal().start_fraction()) &&
      (!current_lane_proto.require_right_turn_signal().has_end_fraction() ||
       current_lane_fraction <=
           current_lane_proto.require_right_turn_signal().end_fraction());

  if (left_on && right_on) {
    *signal = TURN_SIGNAL_EMERGENCY;
    return true;
  } else if (left_on) {
    *signal = TURN_SIGNAL_LEFT;
    return true;
  } else if (right_on) {
    *signal = TURN_SIGNAL_RIGHT;
    return true;
  }

  return false;
}

std::optional<TurnSignal> ComputeDirectionSignal(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &lane_path) {
  const auto &preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist).lane_id();
  const mapping::LaneProto &preview_lane_proto =
      psmm.FindLaneByIdOrDie(preview_lane_id);
  // There is no direction signal
  if (!preview_lane_proto.has_direction()) return std::nullopt;

  const auto &direction = preview_lane_proto.direction();

  switch (direction) {
    case mapping::LaneProto::STRAIGHT:
      return std::nullopt;
    case mapping::LaneProto::LEFT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
    case mapping::LaneProto::RIGHT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
    case mapping::LaneProto::UTURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }
  return std::nullopt;
}

bool HasDirectionSignalAhead(const PlannerSemanticMapManager &psmm,
                             const mapping::LanePath &lane_path) {
  constexpr double preview_dist = 200.0;  // m.
  const auto lane_path_in_horizon =
      lane_path.AfterArclength(kPreviewDirectionDist)
          .BeforeArclength(preview_dist);
  for (const auto &seg : lane_path_in_horizon) {
    const mapping::LaneProto &lane_proto = psmm.FindLaneByIdOrDie(seg.lane_id);
    const auto direction = lane_proto.direction();

    if (direction == mapping::LaneProto::LEFT_TURN ||
        direction == mapping::LaneProto::RIGHT_TURN ||
        direction == mapping::LaneProto::UTURN) {
      return true;
    }
  }
  return false;
}

TurnSignal DecideLaneChangingTurnSignal(const FrenetBox &ego_sl_box,
                                        const DrivePassage &drive_passage) {
  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(ego_sl_box.center().s);
  const double lane_boundary_right_offset =
      std::max(boundaries.first->lat_offset, -kMaxHalfLaneWidth);
  const double lane_boundary_left_offset =
      std::min(boundaries.second->lat_offset, kMaxHalfLaneWidth);

  const double ego_center_l = ego_sl_box.center().l;
  if (lane_boundary_right_offset < ego_center_l &&
      ego_center_l < lane_boundary_left_offset) {
    return TURN_SIGNAL_NONE;
  }

  return ego_center_l <= lane_boundary_right_offset ? TURN_SIGNAL_LEFT
                                                    : TURN_SIGNAL_RIGHT;
}

}  // namespace

// Planner 3.0 turn signal decider
TurnSignalResult DecideTurnSignal(const PlannerSemanticMapManager &psmm,
                                  TurnSignal route_signal,
                                  const RouteSections &route_sections,
                                  const mapping::LanePath &current_lane_path,
                                  const LaneChangeStateProto &lc_state,
                                  const TeleopState &teleop_state,
                                  const DrivePassage &drive_passage,
                                  const FrenetBox &ego_sl_box) {
  // Teleop override turn signal.
  if (teleop_state.IsOverrideEmergencyBlinkerOn() ||
      (teleop_state.IsOverrideLeftBlinkerOn() &&
       teleop_state.IsOverrideRightBlinkerOn())) {
    return {TURN_SIGNAL_EMERGENCY, TELEOP_TURN_SIGNAL};
  }
  if (teleop_state.IsOverrideLeftBlinkerOn()) {
    return {TURN_SIGNAL_LEFT, TELEOP_TURN_SIGNAL};
  }
  if (teleop_state.IsOverrideRightBlinkerOn()) {
    return {TURN_SIGNAL_RIGHT, TELEOP_TURN_SIGNAL};
  }

  if (route_signal == TURN_SIGNAL_LEFT) {
    return {route_signal, STARTING_TURN_SIGNAL};
  }

  constexpr double kStopSignalDistanceAhead = 30.0;  // m.
  if (HasReachedDestination(psmm, route_sections, current_lane_path) &&
      current_lane_path.length() < kStopSignalDistanceAhead) {
    return {TURN_SIGNAL_RIGHT, PULL_OVER_TURN_SIGNAL};
  }

  // Map dictated turn signal
  TurnSignal map_signal;
  if (HasMapDictatedTurnSignal(psmm, current_lane_path, &map_signal)) {
    return {map_signal, MAP_DICTATED_TURN_SIGNAL};
  }

  // Direction signal
  std::optional<TurnSignal> direction_signal = std::nullopt;
  direction_signal = ComputeDirectionSignal(psmm, current_lane_path);
  if (direction_signal.has_value()) {
    return {direction_signal.value(), TURNING_TURN_SIGNAL};
  }

  // Lane change signal
  if (lc_state.stage() == LCS_PAUSE) {
    return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
            LANE_CHANGE_TURN_SIGNAL};
  }
  const auto lane_changing_signal =
      DecideLaneChangingTurnSignal(ego_sl_box, drive_passage);
  if (lane_changing_signal != TURN_SIGNAL_NONE) {
    return {lane_changing_signal, LANE_CHANGE_TURN_SIGNAL};
  }

  // Turn on turn signal in advance to prepare for lane change.
  if (lc_state.has_turn_signal() &&
      lc_state.turn_signal() != TURN_SIGNAL_NONE &&
      !HasDirectionSignalAhead(psmm, current_lane_path)) {
    return {lc_state.turn_signal(), LANE_CHANGE_TURN_SIGNAL};
  }

  return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
}

}  // namespace planner
}  // namespace qcraft
