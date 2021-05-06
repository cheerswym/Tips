#ifndef ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_

#include "absl/time/time.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

DoorDecision ComputeDoorDecision(absl::Time now, double override_door_time,
                                 bool override_door, bool override_door_open,
                                 bool stopped_at_end_of_route);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_
