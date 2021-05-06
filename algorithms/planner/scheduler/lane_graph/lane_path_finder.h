#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_

#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph.h"

namespace qcraft::planner {

std::vector<LanePathInfo> FindBestLanePathsFromStart(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, const LaneGraph &lane_graph,
    ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_
