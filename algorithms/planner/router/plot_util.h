#ifndef ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_

#include <string>
#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft {
namespace planner {

void SendDrivePassageToCanvas(const DrivePassage &drive_passages,
                              const std::string &channel);

void SendRouteLanePathToCanvas(const SemanticMapManager &semantic_map_manager,
                               const CompositeLanePath &route_lane_path,
                               const std::string &topic);

std::vector<Polygon2d> SampleRouteSectionsPolygons(
    const SemanticMapManager *semantic_map_manager,
    const RouteSections &route_sections);

void SendRouteSectionsAreaToCanvas(
    const SemanticMapManager *semantic_map_manager,
    const RouteSections &route_sections, const std::string &channel);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_PLOT_UTIL_H_
