#ifndef ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
#define ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

// Transform a trajectory to a path which extends the last trajectory point by
// an arc with a given minimum length, while considering deceleration distance.
absl::StatusOr<DiscretizedPath> ExtendPathAndDeleteUnreasonablePart(
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
    double min_length, double max_curvature);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
