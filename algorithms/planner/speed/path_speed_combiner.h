#ifndef ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_
#define ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_

#include <vector>

#include "absl/status/status.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

absl::Status CombinePathAndSpeed(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_PATH_SPEED_COMBINER_H_
