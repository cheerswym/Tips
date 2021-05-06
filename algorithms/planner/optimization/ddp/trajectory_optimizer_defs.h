#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_

#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {
namespace optimizer {

using Mfob = MixedFourthOrderBicycle<kDdpTrajectorySteps>;

constexpr int kSampleStep =
    static_cast<int>(kDdpTrajectoryTimeStep / kTrajectoryTimeStep + 0.5);
constexpr int kFreeIndex = static_cast<int>(
    (kTrajectorySteps - 1) * kTrajectoryTimeStep / kDdpTrajectoryTimeStep);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
