#ifndef ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_
#define ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_

#include "absl/time/time.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

void CompleteTrajectoryPastPoints(double trajectory_time_step,
                                  TrajectoryProto *trajectory);

// Update manual trajectory iterationally.
void ShiftPreviousTrajectory(const absl::Duration shift_duration,
                             TrajectoryProto *trajectory);

void CompleteTrajectoryPastPoints(TrajectoryProto *trajectory);

// Recalculate trajectory point's acceleration.
void UpdateTrajectoryPointAccel(TrajectoryProto *trajectory);

void ConvertGlobalTrajectoryToSmooth(
    const CoordinateConverter &coordinate_converter,
    TrajectoryProto *trajectory);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_
