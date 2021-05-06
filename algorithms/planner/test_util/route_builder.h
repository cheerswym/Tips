#ifndef ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_

#include <string>

#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {
RouteParamProto CreateDefaultRouteParam();
CompositeLanePath RoutingToNameSpot(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    std::string name_spot);

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const CompositeLanePath::LanePoint &lane_point);

absl::StatusOr<CompositeLanePath> RoutingToGlobalPoint(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const Vec2d &global_point);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_
