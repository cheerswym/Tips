#ifndef ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_

#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft {
namespace planner {

struct TurnSignalResult {
  TurnSignal signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason reason = TurnSignalReason::TURN_SIGNAL_OFF;
};

// Planner 3.0
TurnSignalResult DecideTurnSignal(const PlannerSemanticMapManager &psmm,
                                  TurnSignal route_signal,
                                  const RouteSections &route_sections,
                                  const mapping::LanePath &current_lane_path,
                                  const LaneChangeStateProto &lc_state,
                                  const TeleopState &teleop_state,
                                  const DrivePassage &drive_passage,
                                  const FrenetBox &ego_sl_box);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
