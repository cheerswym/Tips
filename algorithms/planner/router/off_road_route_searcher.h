#ifndef ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_
#define ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

std::vector<mapping::LanePoint> FindCrossingLanePointsFromPose(
    const SemanticMapManager &smm, const PoseProto &pose, double radius);

struct OffRoadRouteSearcherOutput {
  mapping::LanePoint link_lane_point;
  RouteSectionSequence route_section_sequence;
  CompositeLanePath on_road_route_lane_path;
  int next_stop_index;
};

absl::StatusOr<OffRoadRouteSearcherOutput> SearchForRouteFromOffRoad(
    const SemanticMapManager &smm, const PoseProto &pose,
    const MultipleStopsRequest &route_request, double search_radius);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_
