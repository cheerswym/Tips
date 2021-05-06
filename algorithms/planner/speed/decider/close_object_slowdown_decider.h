#ifndef ONBOARD_PLANNER_SPEED_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_

#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/st_graph.h"

namespace qcraft {
namespace planner {

void MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const DrivePassage& drive_passage, const DiscretizedPath& path_points,
    const PathSlBoundary& path_sl_boundary, ConstraintManager* constraint_mgr);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
