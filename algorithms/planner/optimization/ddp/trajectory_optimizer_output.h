#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_

#include <vector>

#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct TrajectoryOptimizerOutput {
  std::vector<TrajectoryPoint> trajectory;
  std::vector<ApolloTrajectoryPointProto> trajectory_proto;
  AccumulatedDiscountedCostsProto feature_costs;
  AccumulatedDiscountedCostsProto expert_costs;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
