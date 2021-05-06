#include "onboard/planner/router/route_reroute.h"

#include <utility>
#include <vector>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/map_match.h"
#include "onboard/planner/router/route_engine.h"
#include "onboard/planner/router/route_error.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner::route {
namespace {

constexpr double kPoseOverDueSecs = 5.0;

bool IsMotorWay(const mapping::LaneInfo &lane_info) {
  return lane_info.IsPassengerVehicleAvoidLaneType();
}

absl::Status CheckReroutePrecondition(const RerouteInput &input) {
  if (input.semantic_map_manager == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot reroute, semantic_map_manager cannot be null.");
  }
  if (input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot reroute, pose cannot be null.");
  }
  if (input.origin_routing_request == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot reroute, origin_routing_request cannot be null.");
  }
  if (input.multi_stops == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot reroute, multi_stops cannot be null.");
  }
  if (MicroSecondsToSeconds(input.cur_time_micros -
                            input.pose->header().timestamp()) >
      kPoseOverDueSecs) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Pose is overdue. Should not exceed ", kPoseOverDueSecs, " secs."));
  }
  return absl::OkStatus();
}

absl::StatusOr<RerouteOutput> GenerateRouteOutput(
    const SemanticMapManager &semantic_map_manager,
    const MultipleStopsRequest &multi_stops, RoutingResultProto response,
    bool is_bus) {
  VLOG(3) << "avoid lanes:" << absl::StrJoin(multi_stops.avoid_lanes(), "|")
          << ", is_bus:" << is_bus;
  auto route_or = RebuildRouteFromRouteLanePath(
      &semantic_map_manager, response.update_id(), response.routing_request(),
      CompositeLanePath(&semantic_map_manager, response.lane_path()), is_bus,
      multi_stops.avoid_lanes());

  if (!route_or.ok()) {
    return route_or.status();
  }
  auto route = std::make_unique<Route>(std::move(route_or.value()));
  auto route_from_current = std::make_unique<Route>(
      &semantic_map_manager,
      std::make_unique<CompositeLanePath>(route->lane_path()),
      route->section_sequence(),
      ConvertRoutingRequestToGlobalPoint(semantic_map_manager,
                                         route->routing_request()),
      route->update_id(), route->avoid_lanes());
  return RerouteOutput{.routing_result_proto = std::move(response),
                       .route = std::move(route),
                       .route_from_current = std::move(route_from_current)};
}

}  // namespace

absl::StatusOr<RerouteOutput> RerouteFromPose(const RerouteInput &input) {
  // (0) Check input precondition
  RETURN_IF_ERROR(CheckReroutePrecondition(input));

  const auto &semantic_map_manager = *input.semantic_map_manager;
  // (1) Map matching
  ASSIGN_OR_RETURN(const auto point_to_lane,
                   map_match::GetNearestLaneOnDriving(
                       semantic_map_manager, *input.pose,
                       *input.route_param_proto, input.cur_time_micros));

  // (2) Handle start point (match lane, add lane cost)
  // 2.1 fail if on  bicyle lane
  const mapping::LaneInfo *lane_info = point_to_lane.lane_info;
  if (!IsMotorWay(*lane_info)) {
    return absl::FailedPreconditionError("Must on motorway when reroute.");
  }

  // 2.2 Set start cost.TODO(xiang): should set lane cost before search a route.
  const mapping::LanePoint origin = {point_to_lane.lane_info->id,
                                     point_to_lane.fraction};
  // (3) Call Route engine.
  absl::flat_hash_set<mapping::ElementId> blacklist(
      input.origin_routing_request->avoid_lanes().begin(),
      input.origin_routing_request->avoid_lanes().end());

  ASSIGN_OR_RETURN(const auto sections_lanes_pair,
                   SearchForRouteLanePathFromLanePoint(
                       semantic_map_manager, origin,
                       *input.origin_routing_request, blacklist, true));

  // (4) Generate route navi output.
  RoutingResultProto routing_result_proto;
  RETURN_IF_ERROR(CheckRouteValidity(sections_lanes_pair.second));
  sections_lanes_pair.second.ToProto(routing_result_proto.mutable_lane_path());
  routing_result_proto.set_success(true);
  routing_result_proto.set_update_id(input.next_request_id);
  *routing_result_proto.mutable_routing_request() =
      *input.origin_routing_request;
  return GenerateRouteOutput(*input.semantic_map_manager, *input.multi_stops,
                             std::move(routing_result_proto), input.is_bus);
}

}  // namespace qcraft::planner::route
