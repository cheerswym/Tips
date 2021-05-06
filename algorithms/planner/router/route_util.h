#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto proto);

absl::StatusOr<std::vector<mapping::LanePoint>>
ParseDestinationProtoToLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const RoutingRequestProto &routing_request);

mapping::LaneNeighborInfo GetLaneNeighborInfo(
    const mapping::LaneInfo &this_lane, mapping::ElementId next_lane_id,
    bool *left, bool accept_opppsite = false);

// ---- Based on lane aligned semantic map
mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath &target);

mapping::LanePath ForwardExtendRoutePath(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath &target,
    double max_length = std::numeric_limits<double>::infinity(),
    bool broken_boundary_must = false);

mapping::LanePath AlignSourcePathWithTargetPathEndPoint(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePath &target, const mapping::LanePath &source);

CompositeLanePath RearrangeTransitionsOnRoutePath(
    const SemanticMapManager &semantic_map_manager,
    const CompositeLanePath &raw_route_path);

absl::Status CheckRouteValidity(const CompositeLanePath &route_lane_path);

RoutingRequestProto ConvertRoutingRequestToGlobalPoint(
    const SemanticMapManager &semantic_map_manager,
    const RoutingRequestProto &raw_routing_request);

mapping::LanePath BackwardExtendLanePath(
    const mapping::SemanticMapManager &smm,
    const mapping::LanePath &raw_lane_path, double extend_len,
    const std::function<bool(const mapping::LaneInfo &)>
        *nullable_should_stop_and_avoid_extend = nullptr);

mapping::LanePath ForwardExtendLanePath(const mapping::SemanticMapManager &smm,
                                        const mapping::LanePath &raw_lane_path,
                                        double extend_len);

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, mapping::ElementId start_lane_id,
    double extend_len);

absl::StatusOr<RoutingRequestProto> ConvertAllRouteFlagsToRoutingRequestProto();

absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
    const RoutingRequestProto &routing_request,
    const SemanticMapManager &semantic_map_manager);

absl::flat_hash_set<mapping::ElementId> FindAvoidBusOnlyLanes(
    const SemanticMapManager &semantic_map_manager);

absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
    const SemanticMapManager &semantice_map_manager,
    const CompositeLanePath &composite_lane_path, double max_distance_meters);

double SqrDistanceToNextStop(
    const RouteManagerOutputProto &route_out_proto, const PoseProto &pose,
    const LocalizationTransformProto &localization_transform_proto);

bool HasOnlyOneOffroadRequest(const RoutingRequestProto &routing_request);

inline bool IsOffRoadDestination(
    const MultipleStopsRequestProto::StopProto &next_stop) {
  return next_stop.stop_point().has_off_road();
}

inline bool IsValidRouteOutput(const RouteManagerOutputProto &route_out_proto) {
  if (!route_out_proto.has_destination_stop()) return false;
  if (IsOffRoadDestination(route_out_proto.destination_stop())) return true;

  return route_out_proto.has_route_from_current();
}
std::string PoseDebugString(const SemanticMapManager &semantic_map_manager,
                            const PoseProto &pose);

double TravelDistanceBetween(const SemanticMapManager &semantic_map_manager,
                             const CompositeLanePath &composite_lane_path,
                             const CompositeLanePath::CompositeIndex &first,
                             double first_fraction,
                             const CompositeLanePath::CompositeIndex &second,
                             double second_fraction);

bool IsTravelMatchedValid(const CompositeLanePath &composite_lane_path,
                          const CompositeLanePath::CompositeIndex &first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex &second,
                          double second_fraction, int64 last_time_micros,
                          int64 cur_time_micros, double distance, double speed,
                          double reroute_threshold_meters);

struct SectionLanesQueueItem {
  mapping::ElementId id = mapping::kInvalidElementId;
  mapping::LaneBoundaryProto::Type type =
      mapping::LaneBoundaryProto::UNKNOWN_TYPE;
  SectionLanesQueueItem *prev = nullptr;
};

/// @brief return the lane sequence from the src lane(included) to the targe
/// lane(included) in the same section according to the specified direction,
/// NotFoundError if cannot reach.
absl::StatusOr<std::vector<SectionLanesQueueItem>> SearchConnectLaneInSection(
    const SemanticMapManager &semantic_map_manager,
    mapping::ElementId scr_lane_id, mapping::ElementId target_lane_id,
    bool search_direction_left);

/// @brief true if all lanes passed is seperated with broke-white or unkonwn
/// type. The two lanes no need to be adjacent and ignore the fraction. We
/// assume all lanes are almost with same length, better if project to the
/// center line of the path boundary.
bool IsTwoLaneConnectedWithBrokenWhites(
    const SemanticMapManager &semantic_map_manager, mapping::ElementId src,
    mapping::ElementId target);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_
