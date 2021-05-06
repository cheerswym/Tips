#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_

#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

// Creates a lane change state proto from a lane change stage. The rest fields
// use default settings.
LaneChangeStateProto MakeNoneLaneChangeState();

LaneChangeStateProto MakeLaneChangeState(const DrivePassage &drive_passage,
                                         const FrenetBox &ego_frenet_box);

void ToSchedulerOutputProto(const SchedulerOutput &output,
                            SchedulerOutputProto *proto);

double ComputeLCExecutingAggressiveness(double rest_distance,
                                        const FrenetBox &ego_frenet_box,
                                        bool lc_left);

double ComputeLCPreparingAggressiveness(double rest_distance);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
