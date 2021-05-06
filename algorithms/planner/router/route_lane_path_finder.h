#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_LANE_PATH_FINDER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_LANE_PATH_FINDER_H_

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_section_sequence.h"

namespace qcraft::planner {

struct RouteLanePathFinderInput {
  const RouteSectionSequence *section_sequence;
  absl::flat_hash_set<mapping::ElementId> lane_id_blacklist;
  mapping::LanePoint start_point;
  mapping::LanePoint destination_point;
  bool empty_if_must_cross_solid_boundary = false;
};

// Find a lane level route from start point to destination on a given route
// section sequence.
// How to define the best route lane path:
//  - less lane change
//  - later lane change(route lane path should mark the latest lane change point
//  and let planner to decide when to execute lane change)
//  - avoiding lane in blacklist
absl::StatusOr<CompositeLanePath> FindRouteLanePathOnSectionSequence(
    const RouteLanePathFinderInput &input,
    const SemanticMapManager &semantic_map_manager);

namespace internal {
std::vector<mapping::ElementId> MinLaneChangeBlacklist(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &lane_point, bool is_origin);
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_LANE_PATH_FINDER_H_
