#include "onboard/planner/decision/door_open_decider.h"

#include "absl/time/time.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

DoorDecision ComputeDoorDecision(absl::Time now, double last_override_door_time,
                                 bool override_door, bool override_door_open,
                                 bool end_of_route) {
  const bool should_override_door_to_open = override_door && override_door_open;
  if (FLAGS_planner_open_door_at_route_end) {
    const bool driver_override_expired =
        FLAGS_planner_open_door_at_route_end &&
        (ToUnixDoubleSeconds(now) >
         (last_override_door_time +
          FLAGS_planner_door_state_override_waiting_time));
    bool door_open = should_override_door_to_open;
    if (end_of_route && driver_override_expired) {
      door_open = true;
    }
    DoorDecision door_decision;
    door_decision.set_door_state(door_open ? DoorDecision::DOOR_OPEN
                                           : DoorDecision::DOOR_CLOSE);
    if (door_open) {
      door_decision.set_reason(DoorDecision::ARRIVED_AT_STATION);
    } else {
      door_decision.set_reason(DoorDecision::DEFAULT_CLOSE);
    }
    return door_decision;
  } else {
    DoorDecision door_decision;
    door_decision.set_door_state(should_override_door_to_open
                                     ? DoorDecision::DOOR_OPEN
                                     : DoorDecision::DOOR_CLOSE);
    door_decision.set_reason(DoorDecision::TELEOP_OVERRIDE_DOOR_STATE);
    return door_decision;
  }
}

}  // namespace planner
}  // namespace qcraft
