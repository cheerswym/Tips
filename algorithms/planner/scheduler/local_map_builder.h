#ifndef ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner {
// This function will return a lane paths vector sorted from left to right.
// Output lane paths num less than the lane num of first route section. The
// input param "route_sections" must start from AV pos.
absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager &psmm, const RouteSections &route_sections,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes);
}  // namespace qcraft::planner

#endif
