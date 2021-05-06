#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph.h"

namespace qcraft::planner {

void SendLaneGraphToCanvas(const LaneGraph &lane_graph,
                           const SemanticMapManager &semantic_map_manager,
                           const RouteSectionsInfo &sections_info,
                           const std::string &topic);

void SendLanePathInfoToCanvas(const LanePathInfo &lp_info,
                              const SemanticMapManager &semantic_map_manager,
                              const std::string &topic);

void SendLocalMapToCanvas(const std::vector<mapping::LanePath> &lane_paths,
                          const SemanticMapManager &semantic_map_manager,
                          const std::string &topic);

void SendAvSlTrajectoryToCanvas(
    const DrivePassage &drive_passage,
    const PiecewiseLinearFunction<double, double> &traj_l_s,
    const std::string &topic);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_PLOT_UTIL_H_
