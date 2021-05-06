#include "onboard/planner/router/route_util.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "absl/strings/str_split.h"
#include "boost/geometry.hpp"
#include "boost/geometry/algorithms/distance.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/register/linestring.hpp"
#include "boost/geometry/geometries/register/point.hpp"
#include "boost/geometry/strategies/strategies.hpp"
#include "common/proto/map_geometry.pb.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_params.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/time_util.h"

BOOST_GEOMETRY_REGISTER_POINT_2D(
    qcraft::Vec2d, double,
    ::boost::geometry::cs::cartesian, operator[](0), operator[](1));
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<qcraft::Vec2d>);

namespace qcraft {
namespace planner {

namespace {

constexpr double kMaxRetrogradeMeters = -1.0;
constexpr double kMinForwadSpeed = 0.5;

// Convert different types of routing destination to global point
// representation.
mapping::GeoPointProto ConvertToGlobal(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto &dest) {
  mapping::GeoPointProto global_point;
  if (dest.has_global_point()) {
    global_point = dest.global_point();
  } else if (dest.has_lane_point()) {
    const auto lane_pt = mapping::LanePoint(dest.lane_point());
    const auto &lane_info =
        semantic_map_manager.FindLaneInfoOrDie(lane_pt.lane_id());
    const auto global =
        semantic_map_manager.coordinate_converter().SmoothToGlobal(
            Vec3d(lane_info.LerpPointFromFraction(lane_pt.fraction()), 0.0));
    global_point.set_longitude(global.x());
    global_point.set_latitude(global.y());
    global_point.set_altitude(global.z());

  } else {
    QCHECK(dest.has_named_spot());
    const mapping::NamedSpotProto *named_spot = nullptr;
    for (const auto &spot : semantic_map_manager.semantic_map().named_spots()) {
      if (spot.name() == dest.named_spot()) {
        named_spot = &spot;
        break;
      }
    }
    QCHECK(named_spot != nullptr)
        << "Could not find the named spot " << dest.named_spot()
        << ", please check your semantic map version.";

    global_point = named_spot->point();
  }
  return global_point;
}

}  // namespace

absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto destination_proto) {
  if (destination_proto.has_off_road()) {
    return mapping::LanePoint{mapping::kInvalidElementId, 0.0};
  }
  if (destination_proto.has_global_point()) {
    const Vec3d global(destination_proto.global_point().longitude(),
                       destination_proto.global_point().latitude(),
                       destination_proto.global_point().altitude());
    const Vec3d smooth =
        semantic_map_manager.coordinate_converter().GlobalToSmooth(global);
    const auto level = semantic_map_manager.InferLevelIdFromNearbyLanes(smooth);
    auto result_or = mapping::FindClosestLanePointToSmoothPointAtLevel(
        level, semantic_map_manager, Vec2d(smooth.x(), smooth.y()));
    if (!result_or.ok()) {
      return absl::NotFoundError(absl::StrCat(
          "Could not found lane id from destination proto: ",
          destination_proto.ShortDebugString(),
          ", Smooth:", smooth.DebugString(), ", Level:", level, ", Transform:",
          semantic_map_manager.coordinate_converter()
              .localization_transform()
              .ShortDebugString()));
    } else {
      return result_or;
    }
  } else if (destination_proto.has_lane_point()) {
    const auto lp = mapping::LanePoint(destination_proto.lane_point());
    if (!lp.Valid()) {
      return absl::InvalidArgumentError(
          absl::StrCat("The provided lane point is invalid: ",
                       destination_proto.ShortDebugString()));
    } else {
      return lp;
    }
  } else if (destination_proto.has_named_spot()) {
    const auto &map = semantic_map_manager.semantic_map();
    const mapping::NamedSpotProto *named_spot = nullptr;
    for (const auto &spot : map.named_spots()) {
      if (spot.name() == destination_proto.named_spot()) {
        named_spot = &spot;
      }
    }
    if (named_spot == nullptr) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Could not find the named spot ", destination_proto.named_spot()));
    }
    const Vec3d global(named_spot->point().longitude(),
                       named_spot->point().latitude(),
                       named_spot->point().altitude());
    const Vec3d smooth =
        semantic_map_manager.coordinate_converter().GlobalToSmooth(global);
    const auto level = semantic_map_manager.InferLevelIdFromNearbyLanes(smooth);
    const auto lp_or = mapping::FindClosestLanePointToSmoothPointAtLevel(
        level, semantic_map_manager, Vec2d(smooth.x(), smooth.y()));
    if (!lp_or.ok()) {
      return absl::NotFoundError(absl::StrCat(
          "Could not found lane if of named spot from destination proto: ",
          destination_proto.ShortDebugString()));
    } else {
      return lp_or;
    }
  } else {
    QLOG(FATAL) << "should not reach here.";
  }
}

absl::StatusOr<std::vector<mapping::LanePoint>>
ParseDestinationProtoToLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const RoutingRequestProto &routing_request) {
  std::vector<mapping::LanePoint> destinations;
  destinations.reserve(routing_request.destinations_size());
  for (const RoutingDestinationProto &destination_proto :
       routing_request.destinations()) {
    const auto lane_point_or = ParseDestinationProtoToLanePoint(
        semantic_map_manager, destination_proto);
    if (!lane_point_or.ok()) return lane_point_or.status();
    destinations.push_back(lane_point_or.value());
  }
  if (destinations.empty()) {
    return absl::NotFoundError(
        absl::StrCat("No route destination found from request: ",
                     routing_request.ShortDebugString()));
  }
  return destinations;
}

mapping::LaneNeighborInfo GetLaneNeighborInfo(
    const mapping::LaneInfo &this_lane, mapping::ElementId next_lane_id,
    bool *left, bool accept_opposite) {
  for (const auto &neighbor : this_lane.lane_neighbors_on_right) {
    if (neighbor.other_id == next_lane_id &&
        (accept_opposite || !neighbor.opposite)) {
      *left = false;
      return neighbor;
    }
  }

  for (const auto &neighbor : this_lane.lane_neighbors_on_left) {
    if (neighbor.other_id == next_lane_id &&
        (accept_opposite || !neighbor.opposite)) {
      *left = true;
      return neighbor;
    }
  }
  QLOG(FATAL) << "This lane and next lane are not neighbors";
  return {};
}

mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath &target) {
  const auto &start_lane =
      semantic_map_manager.FindLaneInfoOrDie(start_point.lane_id());
  const auto start_range = mapping::GetNeighborRange(left, target, start_lane);
  const double min_fraction_error = 1.0 / start_lane.length();

  if (start_point.fraction() > start_range.second + min_fraction_error) {
    return mapping::LanePath(&semantic_map_manager);
  }

  mapping::LanePath extended_path(&semantic_map_manager, {start_lane.id},
                                  start_range.first, start_point.fraction());

  while (extended_path.start_fraction() == 0.0) {
    bool extended = false;
    for (const auto idx :
         semantic_map_manager.FindLaneInfoOrDie(extended_path.front().lane_id())
             .incoming_lane_indices) {
      const auto &in_lane = semantic_map_manager.lane_info()[idx];
      const auto range = mapping::GetNeighborRange(left, target, in_lane);
      if (range.second == 1.0) {
        extended_path = mapping::LanePath(&semantic_map_manager, {in_lane.id},
                                          range.first, 1.0)
                            .Connect(extended_path);
        extended = true;
        break;
      }
    }
    if (!extended) break;
  }

  return extended_path;
}

mapping::LanePath ForwardExtendRoutePath(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath &target,
    double max_length, bool broken_boundary_must) {
  QCHECK_GT(max_length, 0.0);

  const auto &start_lane =
      semantic_map_manager.FindLaneInfoOrDie(start_point.lane_id());
  mapping::LaneNeighborInfo neighbor;
  const auto start_range =
      mapping::GetNeighborRange(left, target, start_lane, &neighbor);
  const double min_fraction_error = 1.0 / start_lane.length();

  if (start_point.fraction() < start_range.first - min_fraction_error) {
    return mapping::LanePath(&semantic_map_manager);
  }

  // TODO(luzou, xiang): consider BROKEN_LEFT_DOUBLE_WHITE and
  // BROKEN_RIGHT_DOUBLE_WHITE
  if (broken_boundary_must &&
      neighbor.lane_boundary_type != mapping::LaneBoundaryProto::BROKEN_WHITE) {
    return mapping::LanePath(&semantic_map_manager);
  }

  mapping::LanePath extended_path(&semantic_map_manager, {start_lane.id},
                                  start_point.fraction(), start_range.second);

  while (extended_path.end_fraction() == 1.0) {
    bool extended = false;
    for (const auto idx :
         semantic_map_manager.FindLaneInfoOrDie(extended_path.back().lane_id())
             .outgoing_lane_indices) {
      const auto &out_lane = semantic_map_manager.lane_info()[idx];

      const auto range =
          mapping::GetNeighborRange(left, target, out_lane, &neighbor);

      // TODO(luzou, xiang): consider BROKEN_LEFT_DOUBLE_WHITE and
      // BROKEN_RIGHT_DOUBLE_WHITE
      if (broken_boundary_must &&
          neighbor.lane_boundary_type !=
              mapping::LaneBoundaryProto::BROKEN_WHITE) {
        continue;
      }

      QCHECK(range.first == 0.0);
      if (range.second > 0.0) {
        extended_path = extended_path.Connect(mapping::LanePath(
            &semantic_map_manager, {out_lane.id}, 0.0, range.second));
        extended = true;

        if (extended_path.length() > max_length) {
          return extended_path;
        }

        break;
      }
    }
    if (!extended) break;
  }

  return extended_path;
}

mapping::LanePath AlignSourcePathWithTargetPathEndPoint(
    const SemanticMapManager &semantic_map_manager, bool left,
    const mapping::LanePath &target, const mapping::LanePath &source) {
  if (!HasNeighborPointOnPath(semantic_map_manager, left, source.back(),
                              target)) {
    mapping::LanePoint neighbor_point;

    QCHECK(HasNeighborPointOnPath(semantic_map_manager, !left, target.back(),
                                  source, &neighbor_point))
        << source.DebugString() << "\n"
        << target.DebugString();
    return source.BeforeLastOccurrenceOfLanePoint(neighbor_point);
  }
  // Try forward extending target path
  const auto extend_path =
      ForwardExtendRoutePath(semantic_map_manager, left, source.back(), target);
  return source.Connect(extend_path);

  // Neighbor point is on backward extend path: should not happen
}

CompositeLanePath RearrangeTransitionsOnRoutePath(
    const SemanticMapManager &semantic_map_manager,
    const CompositeLanePath &raw_route_path) {
  // (1) Narrow transitions to last lane change point in order to avoid overlap
  // between two transitions.
  // (2) Leave enough space for consecutive lane change
  //
  // Before:
  // o------------------------------------------------------>
  //            |                                           |
  //            o-------------------------------------------------->
  //                        |                                      |
  //                        o-------------------------------------->
  //
  // After:
  // o------------------------>
  //                          |
  //                          o----------------------->
  //                                                  |
  //                                                  o----------------------->

  if (raw_route_path.num_lane_paths() <= 1) return raw_route_path;

  auto lane_paths = raw_route_path.lane_paths();
  auto transitions = raw_route_path.transitions();

  for (int i = transitions.size(); i > 0; --i) {
    const auto &target_path = lane_paths[i];

    // Align previous path with target_path's end point
    lane_paths[i - 1] = AlignSourcePathWithTargetPathEndPoint(
        semantic_map_manager, transitions[i - 1].lc_left, target_path,
        lane_paths[i - 1]);

    // Cut lane change target path overlap part
    mapping::LanePoint neighbor_point;
    QCHECK(HasNeighborPointOnPath(
        semantic_map_manager, transitions[i - 1].lc_left,
        lane_paths[i - 1].back(), lane_paths[i], &neighbor_point))
        << "seq:" << i
        << ", lane_point:" << lane_paths[i - 1].back().DebugString()
        << ", lane_paths:" << lane_paths[i].DebugString()
        << ", raw_route_path:" << raw_route_path.DebugString();

    lane_paths[i] =
        lane_paths[i].AfterFirstOccurrenceOfLanePoint(neighbor_point);

    transitions[i - 1].overlap_length = 0.0;
  }

  return CompositeLanePath(lane_paths, transitions);
}

absl::Status CheckRouteValidity(const CompositeLanePath &route_lane_path) {
  if (route_lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Empty route lane path");
  }

  if (route_lane_path.num_lane_paths() > 1 &&
      route_lane_path.lane_paths().front().length() <
          PlannerParams::Get().lc_decision_params().min_lane_change_length()) {
    return absl::InvalidArgumentError(
        "Current lane path is too short for a routed lane change");
  }

  return absl::OkStatus();
}

RoutingRequestProto ConvertRoutingRequestToGlobalPoint(
    const SemanticMapManager &semantic_map_manager,
    const RoutingRequestProto &raw_routing_request) {
  RoutingRequestProto proto;
  *proto.mutable_avoid_lanes() = raw_routing_request.avoid_lanes();
  *proto.mutable_lane_cost_modifiers() =
      raw_routing_request.lane_cost_modifiers();

  if (raw_routing_request.has_multi_stops()) {
    proto.mutable_multi_stops()->set_infinite_loop(
        raw_routing_request.multi_stops().infinite_loop());

    for (const auto &raw_stop : raw_routing_request.multi_stops().stops()) {
      MultipleStopsRequestProto::StopProto stop_global;
      stop_global.set_stop_name(raw_stop.stop_name());
      if (raw_stop.has_stop_point()) {
        *stop_global.mutable_stop_point()->mutable_global_point() =
            ConvertToGlobal(semantic_map_manager, raw_stop.stop_point());
      } else {
        RoutingDestinationProto tmp_dest;
        *tmp_dest.mutable_named_spot() = raw_stop.stop_name();
        *stop_global.mutable_stop_point()->mutable_global_point() =
            ConvertToGlobal(semantic_map_manager, tmp_dest);
      }

      for (const auto &via_p : raw_stop.via_points()) {
        *stop_global.add_via_points()->mutable_global_point() =
            ConvertToGlobal(semantic_map_manager, via_p);
      }

      *proto.mutable_multi_stops()->add_stops() = std::move(stop_global);
    }
  }

  for (const auto &dest : raw_routing_request.destinations()) {
    *proto.add_destinations()->mutable_global_point() =
        ConvertToGlobal(semantic_map_manager, dest);
  }

  return proto;
}

mapping::LanePath BackwardExtendLanePath(
    const SemanticMapManager &smm, const mapping::LanePath &raw_lane_path,
    double extend_len,
    const std::function<bool(const mapping::LaneInfo &)>
        *nullable_should_stop_and_avoid_extend) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.front();
  mapping::LanePath backward_path(&smm, start_lp);
  while (extend_len > 0.0) {
    const auto &lane_info = smm.FindLaneInfoOrDie(start_lp.lane_id());
    if (nullable_should_stop_and_avoid_extend != nullptr &&
        (*nullable_should_stop_and_avoid_extend)(lane_info)) {
      break;
    }
    if (start_lp.fraction() == 0.0) {
      if (lane_info.incoming_lane_indices.empty()) break;
      if (lane_info.incoming_lane_indices.size() == 1) {
        const auto incoming_lane_idx = lane_info.incoming_lane_indices.front();
        start_lp =
            mapping::LanePoint(smm.lane_info()[incoming_lane_idx].id, 1.0);
      } else {
        constexpr double kSampleLen = 4.0;  // m.
        const Vec2d origin_pt = lane_info.points_smooth.front();
        const Vec2d next_pt = lane_info.LerpPointFromFraction(
            std::min(1.0, kSampleLen / lane_info.length()));
        const Vec2d heading = (next_pt - origin_pt).normalized();

        double max_projection = std::numeric_limits<double>::lowest();
        auto opt_incoming_index = lane_info.incoming_lane_indices.front();

        for (int i = 0; i < lane_info.incoming_lane_indices.size(); ++i) {
          const auto &tmp_lane_info =
              smm.lane_info()[lane_info.incoming_lane_indices[i]];
          const Vec2d prev_pt = tmp_lane_info.LerpPointFromFraction(
              std::max(0.0, 1.0 - kSampleLen / tmp_lane_info.length()));

          const Vec2d tmp_heading = (origin_pt - prev_pt).normalized();

          const double proj = heading.Dot(tmp_heading);

          if (proj > max_projection) {
            max_projection = proj;
            opt_incoming_index = lane_info.incoming_lane_indices[i];
          }
        }
        start_lp =
            mapping::LanePoint(smm.lane_info()[opt_incoming_index].id, 1.0);
      }

    } else {
      const double len = lane_info.length();
      if (len * start_lp.fraction() > extend_len) {
        const double fraction = start_lp.fraction() - extend_len / len;
        backward_path = mapping::LanePath(&smm, {lane_info.id}, fraction,
                                          start_lp.fraction())
                            .Connect(backward_path);
        break;
      } else {
        extend_len -= len * start_lp.fraction();
        backward_path =
            mapping::LanePath(&smm, {lane_info.id}, 0.0, start_lp.fraction())
                .Connect(backward_path);
        start_lp = mapping::LanePoint(start_lp.lane_id(), 0.0);
      }
    }
  }

  return backward_path.Connect(raw_lane_path);
}

mapping::LanePath ForwardExtendLanePath(const SemanticMapManager &smm,
                                        const mapping::LanePath &raw_lane_path,
                                        double extend_len) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path(&smm, start_lp);
  while (extend_len > 0.0) {
    const auto &lane_info = smm.FindLaneInfoOrDie(start_lp.lane_id());
    if (start_lp.fraction() == 1.0) {
      if (lane_info.outgoing_lane_indices.empty()) break;
      const auto out_lane_idx = lane_info.outgoing_lane_indices.front();
      start_lp = mapping::LanePoint(smm.lane_info()[out_lane_idx].id, 0.0);

    } else {
      const double len = lane_info.length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        forward_path = forward_path.Connect(mapping::LanePath(
            &smm, {lane_info.id}, start_lp.fraction(), fraction));
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        forward_path = forward_path.Connect(
            mapping::LanePath(&smm, {lane_info.id}, start_lp.fraction(), 1.0));
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }

  return raw_lane_path.Connect(forward_path);
}

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, mapping::ElementId start_lane_id,
    double extend_len) {
  if (extend_len <= 0.0) return {};

  const auto &sections = sections_info.section_segments();
  std::vector<mapping::ElementId> lane_ids;
  mapping::ElementId cur_lane_id = start_lane_id;
  double cur_frac = sections_info.front().start_fraction, end_frac = 1.0;
  int sec_idx = 0;
  while (extend_len > 0.0) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(cur_lane_id);
    if (cur_frac == 1.0) {
      const auto &next_sec = sections[++sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      double max_driving_dist = 0.0;
      for (const auto lane_idx : lane_info.outgoing_lane_indices) {
        const auto next_lane_id = psmm.lane_info()[lane_idx].id;
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          const double driving_dist = next_sec.driving_distance[FindOrDie(
              next_sec.id_idx_map, next_lane_id)];
          if (driving_dist > max_driving_dist) {
            outgoing_lane_id = next_lane_id;
            max_driving_dist = driving_dist;
          }
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      cur_frac = 0.0;
    } else {
      lane_ids.emplace_back(cur_lane_id);
      const double len = lane_info.length();
      if (len * (1.0 - cur_frac) > extend_len) {
        end_frac = extend_len / len + cur_frac;
        break;
      } else {
        extend_len -= len * (1.0 - cur_frac);
        cur_frac = 1.0;
      }
    }
  }
  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           sections_info.front().start_fraction, end_frac);
}

absl::StatusOr<RoutingRequestProto>
ConvertAllRouteFlagsToRoutingRequestProto() {
  RoutingRequestProto routing_request;
  StringToProtoLogCollector log_collector;

  if (!FLAGS_route.empty()) {
    QCHECK(
        StringToProtoLogDetail(FLAGS_route, &routing_request, &log_collector))
        << "Failed to parse routing request, "
        << log_collector.GetLastParseError().DebugString();
    QLOG(INFO) << "Create default routing request by " << FLAGS_route;
    return routing_request;
  }

  if (!FLAGS_multi_stops_route.empty()) {
    MultipleStopsRequestProto multiple_stops_request;
    QCHECK(StringToProtoLogDetail(FLAGS_multi_stops_route,
                                  &multiple_stops_request, &log_collector))
        << "Failed to parse routing request, "
        << log_collector.GetLastParseError().DebugString();

    QLOG(INFO) << "Create default routing request by "
               << FLAGS_multi_stops_route;
    *routing_request.mutable_multi_stops() = multiple_stops_request;
    return routing_request;
  }

  if (!FLAGS_route_str.empty()) {
    QLOG(INFO) << "Create default routing request by " << FLAGS_route_str;
    const auto tokens = absl::StrSplit(FLAGS_route_str, ',');
    std::vector<std::string> named_spots(tokens.begin(), tokens.end());
    if (FLAGS_random_route) {
      // Read random seed from /dev/random.
      std::random_device rd("/dev/random");
      std::mt19937 g(rd());
      std::shuffle(named_spots.begin(), named_spots.end(), g);
    }
    for (const auto &named_spot : named_spots) {
      if (named_spot.empty()) continue;
      routing_request.add_destinations()->set_named_spot(
          std::string(named_spot));
    }
    return routing_request;
  }

  return absl::NotFoundError(
      "Routing request is empty or invalid, please check it again.");
}

absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
    const RoutingRequestProto &routing_request,
    const SemanticMapManager &semantic_map_manager) {
  if (routing_request.avoid_regions().empty()) return {};

  absl::flat_hash_set<mapping::ElementId> avoid_lanes_in_regions;

  std::vector<Polygon2d> polygons;
  polygons.reserve(routing_request.avoid_regions_size());
  for (const auto &avoid_region : routing_request.avoid_regions()) {
    polygons.push_back(SmoothPolygon2dFromGeoPolygonProto(
        avoid_region, semantic_map_manager.coordinate_converter()));
  }
  for (const auto &lane_info : semantic_map_manager.lane_info()) {
    for (const auto &polygon : polygons) {
      bool lane_in_polygon = true;
      for (const auto &smooth_point : lane_info.points_smooth) {
        if (!polygon.IsPointIn(smooth_point)) {
          lane_in_polygon = false;
          break;
        }
      }
      if (lane_in_polygon) {
        avoid_lanes_in_regions.insert(lane_info.id);
      }
    }
  }
  return avoid_lanes_in_regions;
}

absl::flat_hash_set<mapping::ElementId> FindAvoidBusOnlyLanes(
    const SemanticMapManager &semantic_map_manager) {
  absl::flat_hash_set<mapping::ElementId> bus_only_lanes;
  for (const auto &lane_info : semantic_map_manager.lane_info()) {
    if (lane_info.Type() == mapping::LaneProto::BUS_ONLY) {
      bus_only_lanes.insert(lane_info.id);
    }
  }
  return bus_only_lanes;
}

/// @brief Cannot keep self-intersect,max_distance(epsilon,unit=meter), use
/// boost:geometry doglas-peuker,
/// @return Global coordinate  points seq (rad)
/// @see mapping::SampleLanePathPoints
absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
    const SemanticMapManager &semantic_map_manager,
    const CompositeLanePath &composite_lane_path, double max_distance_meters) {
  if (max_distance_meters < 0.0) {
    return absl::InvalidArgumentError("distance must GE 0");
  }
  std::vector<Vec2d> points;

  constexpr double kApproximateMeterToRadRatio = 1.11E-5 * M_PI / 180;
  for (const auto &lane_path : composite_lane_path.lane_paths()) {
    const std::vector<Vec2d> &smooth_points =
        SampleLanePathPoints(semantic_map_manager, lane_path);
    std::vector<Vec2d> source_points;
    if (smooth_points.empty()) {
      continue;
    }
    source_points.reserve(smooth_points.size());
    std::for_each(
        smooth_points.begin(), smooth_points.end(),
        [&semantic_map_manager, &source_points](const auto &point) {
          source_points.push_back(
              semantic_map_manager.coordinate_converter().SmoothToGlobal(
                  point));
        });

    std::vector<Vec2d> simplified;
    boost::geometry::simplify(
        source_points, simplified,
        max_distance_meters * kApproximateMeterToRadRatio);
    const int simplified_size = simplified.size();
    points.insert(points.end(), simplified.begin(), simplified.end());
    VLOG(2) << " Simplify, source points num:" << source_points.size()
            << ", after:" << simplified_size
            << ", total points:" << points.size();
  }
  return points;
}

double SqrDistanceToNextStop(
    const RouteManagerOutputProto &route_out_proto, const PoseProto &pose,
    const LocalizationTransformProto &localization_transform_proto) {
  const MultipleStopsRequestProto::StopProto &destination_stop =
      route_out_proto.destination_stop();
  const RoutingDestinationProto &stop_point = destination_stop.stop_point();
  const mapping::GeoPointProto &stop_point_pos = stop_point.global_point();
  const Vec3d stop_pos_global =
      Vec3(stop_point_pos.longitude(), stop_point_pos.latitude(),
           stop_point_pos.altitude());
  const CoordinateConverter coordinate_converter =
      CoordinateConverter::FromLocalizationTransform(
          localization_transform_proto, /*correct_yaw=*/false);
  const Vec3d stop_pos_smooth =
      coordinate_converter.GlobalToSmooth(stop_pos_global);
  // ignore z currently
  return Sqr(stop_pos_smooth.x() - pose.pos_smooth().x()) +
         Sqr(stop_pos_smooth.y() - pose.pos_smooth().y());
}

bool HasOnlyOneOffroadRequest(const RoutingRequestProto &routing_request) {
  return (routing_request.destinations_size() == 1 &&
          routing_request.destinations(0).has_off_road()) ||
         (routing_request.has_multi_stops() &&
          routing_request.multi_stops().stops_size() == 1 &&
          IsOffRoadDestination(routing_request.multi_stops().stops(0)));
}
std::string PoseDebugString(const SemanticMapManager &semantic_map_manager,
                            const PoseProto &pose) {
  const Vec3d pos = Vec3dFromProto(pose.pos_smooth());
  const Vec3d global =
      semantic_map_manager.coordinate_converter().SmoothToGlobal(pos);
  return absl::StrFormat("{x: %.8f, y: %.8f, heading: %.2f, speed: %.2f}",
                         global.x(), global.y(), pose.yaw(),
                         pose.vel_body().x());
}

double TravelDistanceBetween(const SemanticMapManager &semantic_map_manager,
                             const CompositeLanePath &composite_lane_path,
                             const CompositeLanePath::CompositeIndex &first,
                             double first_fraction,
                             const CompositeLanePath::CompositeIndex &second,
                             double second_fraction) {
  // reversing here allowed,so distance may be minus.
  const auto &first_lane_path =
      composite_lane_path.lane_path(first.lane_path_index);
  const auto &last_lane_path =
      composite_lane_path.lane_path(second.lane_path_index);

  double travel_distance = 0;
  const auto &first_lane = semantic_map_manager.FindLaneInfoOrDie(
      first_lane_path.lane_id(first.lane_index));
  const auto &last_lane = semantic_map_manager.FindLaneInfoOrDie(
      last_lane_path.lane_id(second.lane_index));

  if (first.lane_path_index == second.lane_path_index) {
    if (first.lane_index == second.lane_index) {
      travel_distance =
          first_lane.length() * (second_fraction - first_fraction);
      return travel_distance;
    }

    travel_distance += first_lane.length() *
                       (first.lane_index == first_lane_path.back().lane_id()
                            ? first_lane_path.end_fraction()
                            : 1.0 - first_fraction);
    travel_distance += last_lane.length() * second_fraction;
    for (int j = first.lane_index + 1; j < second.lane_index; ++j) {
      const auto &lane_path =
          composite_lane_path.lane_path(second.lane_path_index);
      const auto &lane =
          semantic_map_manager.FindLaneInfoOrDie(lane_path.lane_id(j));
      travel_distance += lane.length();
    }
  } else {
    travel_distance += first_lane.length() *
                       (first.lane_index == first_lane_path.back().lane_id()
                            ? first_lane_path.end_fraction()
                            : 1.0 - first_fraction);
    travel_distance += last_lane.length() * second_fraction;
    double first_lane_path_distance = 0;
    // first lane path
    for (int j = first.lane_index + 1; j < first_lane_path.size(); j++) {
      const auto &lane =
          semantic_map_manager.FindLaneInfoOrDie(first_lane_path.lane_id(j));
      travel_distance += (j + 1 == first_lane_path.size())
                             ? lane.length() * first_lane_path.end_fraction()
                             : lane.length();
    }
    travel_distance += first_lane_path_distance;
    // last lane path
    for (int j = 0; j < second.lane_index; j++) {
      const auto &lane =
          semantic_map_manager.FindLaneInfoOrDie(last_lane_path.lane_id(j));
      travel_distance +=
          (j == 0) ? lane.length() * (1 - last_lane_path.start_fraction())
                   : lane.length();
    }
    // all mid lane path
    for (int i = first.lane_path_index + 1; i < second.lane_path_index; i++) {
      const auto &lane_path = composite_lane_path.lane_path(i);
      for (int j = 0; j < lane_path.size(); j++) {
        double end_fraction =
            (j + 1 == lane_path.size()) ? first_lane_path.end_fraction() : 1.0;
        double start_fraction = (j == 0) ? lane_path.start_fraction() : 0.0;
        const auto &lane =
            semantic_map_manager.FindLaneInfoOrDie(lane_path.lane_id(j));
        travel_distance += lane.length() * (end_fraction - start_fraction);
      }
    }
  }
  return travel_distance;
}

bool IsTravelMatchedValid(const CompositeLanePath &composite_lane_path,
                          const CompositeLanePath::CompositeIndex &first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex &second,
                          double second_fraction, int64 last_time_micros,
                          int64 cur_time_micros, double distance, double speed,
                          double reroute_threshold_meters) {
  VLOG(2) << "first" << first.DebugString() << ", second"
          << second.DebugString() << ", first_fraction:" << first_fraction
          << ", second_fraction:" << second_fraction
          << ", last_time_micros:" << last_time_micros
          << ", cur_time_micros:" << cur_time_micros;
  if (last_time_micros == -1) {
    return true;
  }
  // Retrograde is not allowed, forward direction
  if (speed >= kMinForwadSpeed && distance <= kMaxRetrogradeMeters) {
    VLOG(2) << "Retrograde is not allowed.distance:" << distance
            << ", max:" << kMaxRetrogradeMeters << ", speed:" << speed;
    return false;
  }
  constexpr double max_speed = Kph2Mps(120.0);
  constexpr double drift_allowed = 3.5 * 2;
  double reroute_threshold = std::min(
      reroute_threshold_meters,
      MicroSecondsToSeconds(cur_time_micros - last_time_micros) * max_speed);
  VLOG(2) << "RerouteThresholdMeters:" << reroute_threshold_meters
          << ", drift_allowed: " << drift_allowed << ", distance:" << distance;
  return distance <= std::max(reroute_threshold, drift_allowed);
}

absl::StatusOr<std::vector<SectionLanesQueueItem>> SearchConnectLaneInSection(
    const SemanticMapManager &semantic_map_manager,
    mapping::ElementId scr_lane_id, mapping::ElementId target_lane_id,
    bool search_direction_left) {
  std::vector<std::unique_ptr<SectionLanesQueueItem>> allocated_items;
  std::deque<SectionLanesQueueItem *> queue;
  auto root = std::make_unique<SectionLanesQueueItem>();
  root->id = scr_lane_id;
  root->prev = nullptr;
  root->type = mapping::LaneBoundaryProto::UNKNOWN_TYPE;
  allocated_items.push_back(std::move(root));
  queue.push_back(allocated_items.back().get());
  bool found = false;
  SectionLanesQueueItem *cur = nullptr;
  while (!queue.empty()) {
    cur = queue.front();
    mapping::ElementId cur_id = cur->id;
    queue.pop_front();
    if (cur_id != target_lane_id) {
      const mapping::LaneInfo &lane_info =
          semantic_map_manager.FindLaneInfoOrDie(cur_id);
      const auto &neighbors = search_direction_left
                                  ? lane_info.lane_neighbors_on_left
                                  : lane_info.lane_neighbors_on_right;
      for (const mapping::LaneNeighborInfo &neighbour : neighbors) {
        auto new_item = std::make_unique<SectionLanesQueueItem>();
        new_item->id = (neighbour.this_id == cur_id ? neighbour.other_id
                                                    : neighbour.this_id);
        new_item->prev = cur;
        new_item->type = neighbour.lane_boundary_type;
        allocated_items.push_back(std::move(new_item));
        queue.push_back(allocated_items.back().get());
      }
    } else {
      found = true;
      break;
    }
  }
  if (!found) {
    return absl::NotFoundError("Cannot find connect lane");
  }
  std::vector<SectionLanesQueueItem> results;
  for (; cur != nullptr; cur = cur->prev) {
    results.push_back(*cur);
  }
  return results;
}

bool IsTwoLaneConnectedWithBrokenWhites(
    const SemanticMapManager &semantic_map_manager, mapping::ElementId src,
    mapping::ElementId target) {
  const auto connect_left_or = SearchConnectLaneInSection(
      semantic_map_manager, src, target, /*search_direction_left=*/true);
  if (connect_left_or.ok()) {
    for (const SectionLanesQueueItem &item : connect_left_or.value()) {
      if (item.type != mapping::LaneBoundaryProto::UNKNOWN_TYPE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_WHITE) {
        VLOG(2) << "id:" << item.id << ",type:" << item.type << ",left";
        return false;
      }
    }
    return true;
  }
  const auto connect_right_or = SearchConnectLaneInSection(
      semantic_map_manager, src, target, /*search_direction_left=*/false);
  if (connect_right_or.ok()) {
    for (const SectionLanesQueueItem &item : connect_right_or.value()) {
      if (item.type != mapping::LaneBoundaryProto::UNKNOWN_TYPE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_WHITE) {
        VLOG(2) << "id:" << item.id << ",type:" << item.type << ",right";
        return false;
      }
    }
    return true;
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
