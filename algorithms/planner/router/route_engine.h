#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_ENGINE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_ENGINE_H_

#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_lane_path_finder.h"
#include "onboard/planner/router/route_section_sequence.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

// Search for section sequence.
// TODO(zuowei): Add blacklist in searching for RouteSectionSequence.
// In case all the lanes in a section are in blacklist.
absl::StatusOr<RouteSectionSequence> SearchForRouteSectionSequenceFromLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RoutingRequestProto &routing_request, bool use_time);

absl::StatusOr<std::pair<RouteSectionSequence, mapping::LanePoint>>
SearchForRouteSectionSequenceFromPose(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto, const PoseProto &pose,
    const RoutingRequestProto &routing_request, bool use_time);

absl::StatusOr<RouteSectionSequence>
SearchForRouteSectionSequenceFromDestination(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto &destination_proto,
    const RoutingRequestProto &routing_request, bool use_time);

absl::StatusOr<RouteSectionSequence>
SearchForRouteSectionSequenceAlongLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations, bool use_time);

// Search for route lane path and section sequence.
// Since RoutingRequestProto has avoid lanes and avoid regions, you
// should pre-process them and send them all to blacklist. They will
// not be processed here, we suppose them are all in blacklist.
absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time);

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromPose(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto, const PoseProto &pose,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time);

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromDestination(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto &destination_proto,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time);

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathAlongLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time);

// Search for route lane path from section sequence.
absl::StatusOr<CompositeLanePath> SearchForRouteLanePathFromSectionSequence(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RouteSectionSequence &section_sequence,
    const absl::flat_hash_set<mapping::ElementId> &blacklist);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_ENGINE_H_
