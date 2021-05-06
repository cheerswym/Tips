#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

struct RouteTrackerOutput {
  double travel_distance;
  CompositeLanePath::CompositeIndex composite_index = {0, 0};
  mapping::LanePoint lane_point = {mapping::kInvalidElementId, 0.0};
};

// The result of a point(vehicle) project to a route.
struct PointToCompositeLanePath {
  bool point_on_route_lane = false;
  // An map matched index of the route composite lane path
  CompositeLanePath::CompositeIndex composite_index = {0, 0};
  // The lane id which on the route other than the vehicle is on
  mapping::ElementId route_lane_id;
};

absl::StatusOr<PointToCompositeLanePath> FindLaneFromLastCompositeIndex(
    const CompositeLanePath &composite_lane_path,
    const CompositeLanePath::CompositeIndex &last_composite_index,
    absl::Span<const mapping::ElementId> section_lane_ids,
    mapping::ElementId to_find_lane_id);

absl::StatusOr<RouteTrackerOutput> GetTrackerInfoOnRoute(
    const RouteManagerState &rm_state, const SemanticMapManager &smm,
    const PoseProto &pose);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_
