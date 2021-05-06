#ifndef ONBOARD_PLANNER_ROUTER_MANAGER_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_MANAGER_UTIL_H_

#include <utility>
#include <vector>

#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/route_engine.h"

namespace qcraft {
namespace planner {

constexpr static auto kLookFutureRestrictDuration = absl::Minutes(5);

absl::StatusOr<int> FindNextDestinationIndexViaLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const MultipleStopsRequest &multi_stops_request,
    const std::function<bool(const RouteSectionSequence &)>
        &point_on_route_func);

absl::StatusOr<MultipleStopsRequest> RecoverRoutingRequest(
    const RecordedRouteProto &recorded_route,
    const SemanticMapManager &semantic_map_manager);

absl::StatusOr<Route> RebuildRouteFromRouteLanePath(
    const SemanticMapManager *semantic_map_manager, int update_id,
    const RoutingRequestProto &routing_request, const CompositeLanePath &rlp,
    bool is_bus, const absl::flat_hash_set<mapping::ElementId> &avoid_lanes);

/// @brief  Change the blacklist with bus lane
/// @return the change list.
std::vector<mapping::ElementId> MayUpdateAvoidLanesByRestict(
    const SemanticMapManager &semantic_map_manager,
    const RouteSectionSequence &section_seq, const absl::Time &begin,
    const absl::Time &end, absl::flat_hash_set<mapping::ElementId> *blacklist);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_MANAGER_UTIL_H_
