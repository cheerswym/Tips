#include "onboard/planner/router/route_engine.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/route_searcher.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/status_macros.h"

DEFINE_bool(use_route_searcher_for_section_seq, true,
            "use previous method to search for section sequence");

namespace qcraft {
namespace planner {

namespace {

absl::StatusOr<std::vector<mapping::ElementId>>
SearchForRouteSectionsToOneLanePoint(
    const mapping::LanePoint &origin, const mapping::LanePoint &destination,
    const SemanticMapManager &semantic_map_manager,
    absl::Span<double> section_costs, bool use_time) {
  QCOUNTER_FUNC();
  const int num_sections = semantic_map_manager.semantic_map().sections_size();

  // A* search, all processes are based on idx
  std::vector<int> closed(num_sections, 0);
  // Least cumulative cost for reaching current section
  std::vector<double> cum_costs(num_sections,
                                std::numeric_limits<double>::infinity());
  // The predecessor section of current section
  std::vector<int> from(num_sections, mapping::kInvalidElementId);

  // find origin section idx and destination section idx
  const auto &origin_lane_info =
      semantic_map_manager.FindLaneInfoOrDie(origin.lane_id());
  const int origin_section_idx =
      semantic_map_manager.FindSectionInfoOrDie(origin_lane_info.section_id)
          .idx;
  const auto &destination_lane_info =
      semantic_map_manager.FindLaneInfoOrDie(destination.lane_id());
  const int destination_section_idx =
      semantic_map_manager
          .FindSectionInfoOrDie(destination_lane_info.section_id)
          .idx;

  // current section idx, predecessor section idx
  using SemanticMapSectionStep = std::pair<int, int>;
  // A* heuristic cost estimate: the same as route search, choose the Euclidean
  // distance from the endpoint of an arbitrary lane (e.g. the first)
  // in a section to the start of an arbitrary lane in final section.
  const Vec2d dest_section_point =
      semantic_map_manager
          .GetLaneControlPointsOrDie(
              semantic_map_manager.section_info()[destination_section_idx]
                  .lane_ids[0])
          .front();
  const auto h = [&](int section_idx) {
    if (section_idx == destination_section_idx) return 0.0;
    const double dist =
        (semantic_map_manager
             .GetLaneControlPointsOrDie(
                 semantic_map_manager.section_info()[section_idx].lane_ids[0])
             .back() -
         dest_section_point)
            .norm();
    if (use_time) {
      constexpr double kMaxSpeedLimit = 65.0;  // mph.
      return dist / Mph2Mps(kMaxSpeedLimit);
    } else {
      return dist;
    }
  };

  // A* cost: actual least cumulative cost plus estimated cost to final section.
  // cost = pre + now + heuristic
  const auto cost_fn = [&section_costs, &h,
                        &cum_costs](SemanticMapSectionStep step) {
    const double section_cost = section_costs[step.first];
    return cum_costs[step.first] + section_cost + h(step.first);
  };

  const auto cost_comp = [&cost_fn](SemanticMapSectionStep step0,
                                    SemanticMapSectionStep step1) {
    return cost_fn(step0) > cost_fn(step1);
  };

  using OpenQueue = std::priority_queue<SemanticMapSectionStep,
                                        std::vector<SemanticMapSectionStep>,
                                        decltype(cost_comp)>;

  OpenQueue open_queue(cost_comp);

  open_queue.emplace(origin_section_idx, mapping::kInvalidElementId);
  bool destination_section_reached = false;
  bool need_loop = (origin_section_idx == destination_section_idx &&
                    origin.fraction() > destination.fraction());

  while (!open_queue.empty()) {
    const SemanticMapSectionStep current_best_step = open_queue.top();
    open_queue.pop();
    const int current_section_idx = current_best_step.first;
    const int predecessor_section_idx = current_best_step.second;

    if (closed[current_section_idx] &&
        current_section_idx != destination_section_idx) {
      continue;
    }

    closed[current_section_idx] = 1;

    if (current_section_idx == destination_section_idx) {
      if (need_loop) {
        need_loop = false;
      } else {
        destination_section_reached = true;
        from[destination_section_idx] = predecessor_section_idx;
        break;
      }
    }

    const auto &current_section_info =
        semantic_map_manager.section_info()[current_section_idx];
    const double prev_cum_cost = (current_section_idx == origin_section_idx)
                                     ? 0.0
                                     : cum_costs[current_section_idx];

    const double cost_on_section = section_costs[current_section_idx];

    const double new_cost = prev_cum_cost + cost_on_section;
    for (const auto next_section_idx : current_section_info.outgoing_sections) {
      if (cum_costs[next_section_idx] > new_cost) {
        cum_costs[next_section_idx] = new_cost;
        open_queue.emplace(next_section_idx, current_section_idx);
        from[next_section_idx] = current_section_idx;
      }
    }
  }

  if (!destination_section_reached) {
    return absl::NotFoundError(
        absl::StrFormat("Could not find a section sequence from %s to %s.",
                        destination.DebugString(), origin.DebugString()));
  }

  std::vector<mapping::ElementId> section_ids;
  section_ids.push_back(destination_lane_info.section_id);
  while (
      from[semantic_map_manager.FindSectionInfoOrDie(section_ids.back()).idx] !=
      mapping::kInvalidElementId) {
    mapping::ElementId prev_section_id =
        semantic_map_manager
            .section_info()[from[semantic_map_manager
                                     .FindSectionInfoOrDie(section_ids.back())
                                     .idx]]
            .id;
    section_ids.push_back(prev_section_id);

    if (section_ids.back() == origin_lane_info.section_id) break;
  }

  std::reverse(section_ids.begin(), section_ids.end());

  return section_ids;
}

absl::StatusOr<RouteSectionSequence> SearchForRouteSectionSequence(
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    const SemanticMapManager &semantic_map_manager, bool use_time) {
  QCOUNTER_FUNC();
  // Since use A star algorithm to search for route_section_sequence still have
  // some problems due to maps, we use older method to find section sequence.
  if (FLAGS_use_route_searcher_for_section_seq) {
    CompositeLanePath route_lane_path;
    // TODO(zuowei): Change APIs in route_searcher.h to absl::StatusOr<>.
    std::vector<mapping::LanePoint> vec_destinations;
    vec_destinations.reserve(destinations.size());
    for (const auto &destination : destinations) {
      vec_destinations.push_back(destination);
    }
    RouteSearcher route_searcher(&semantic_map_manager, /*use_time=*/true);
    bool find_route_lane_path = route_searcher.SearchForRoutePathToLanePoints(
        origin, vec_destinations, &route_lane_path);
    if (!find_route_lane_path) {
      return absl::UnavailableError(absl::StrFormat(
          "Search for route lane path failed from %s to %s.",
          origin.DebugString(),
          absl::StrJoin(
              destinations, "\n",
              [](std::string *out, const mapping::LanePoint &destination) {
                absl::StrAppend(out, destination.DebugString());
              })));
    }
    return RouteSectionSequence(route_lane_path, &semantic_map_manager);
  }

  std::vector<mapping::ElementId> section_ids;
  // Create default section costs.
  const int num_sections = semantic_map_manager.semantic_map().sections_size();
  std::vector<double> section_costs;
  section_costs.resize(num_sections, 0.0);

  for (int i = 0; i < num_sections; ++i) {
    const auto &section_info = semantic_map_manager.section_info()[i];
    // Cost from average section traversal time at speed limit.
    section_costs[i] = use_time ? section_info.average_length /
                                      section_info.average_speed_limit
                                : section_info.average_length;
  }

  // Search for section ids.
  mapping::LanePoint current_origin = origin;
  for (const mapping::LanePoint &destination : destinations) {
    QCHECK(destination.Valid())
        << "Invalid destination: " << destination.DebugString();
    const auto current_section_ids = SearchForRouteSectionsToOneLanePoint(
        current_origin, destination, semantic_map_manager,
        absl::MakeSpan(section_costs), use_time);
    if (!current_section_ids.ok()) {
      return absl::NotFoundError(absl::StrFormat(
          "Could not find a section sequence from %s to %s",
          current_origin.DebugString(), destination.DebugString()));
    }
    current_origin = destination;
    if (!section_ids.empty()) {
      section_ids.pop_back();
    }
    section_ids.insert(section_ids.end(), current_section_ids.value().begin(),
                       current_section_ids.value().end());
  }

  // Get RouteSectionSequence from section ids.
  return RouteSectionSequence(absl::MakeSpan(section_ids), semantic_map_manager,
                              origin, destinations.back());
}

absl::StatusOr<CompositeLanePath>
SearchForCompositeLanePathFromSectionsViaLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const RouteSectionSequence &route_section_sequence,
    const absl::flat_hash_set<mapping::ElementId> &blacklist,
    absl::Span<const mapping::LanePoint> destinations,
    const mapping::LanePoint &origin) {
  CompositeLanePath route_lane_path;
  mapping::LanePoint current_origin = origin;
  for (const auto &destination : destinations) {
    const auto composite_lane_path_or = FindRouteLanePathOnSectionSequence(
        RouteLanePathFinderInput{.section_sequence = &route_section_sequence,
                                 .lane_id_blacklist = blacklist,
                                 .start_point = current_origin,
                                 .destination_point = destination},
        semantic_map_manager);

    if (!composite_lane_path_or.ok()) {
      return absl::UnavailableError(absl::StrFormat(
          "Could not find a composite lane path from %s to %s.",
          current_origin.DebugString(), destination.DebugString()));
    }
    if (route_lane_path.IsEmpty()) {
      route_lane_path = composite_lane_path_or.value();
    } else {
      CompositeLanePath tmp_lane_path =
          route_lane_path.Connect(composite_lane_path_or.value());
      route_lane_path = tmp_lane_path;
    }
    current_origin = destination;
  }
  return route_lane_path;
}

absl::StatusOr<CompositeLanePath>
SearchForCompositeLanePathFromSectionsViaRoutingRequest(
    const SemanticMapManager &semantic_map_manager,
    const RouteSectionSequence &route_section_sequence,
    const absl::flat_hash_set<mapping::ElementId> &blacklist,
    const RoutingRequestProto &routing_request,
    const mapping::LanePoint &origin) {
  ASSIGN_OR_RETURN(
      const auto destinations,
      ParseDestinationProtoToLanePoints(semantic_map_manager, routing_request));

  return SearchForCompositeLanePathFromSectionsViaLanePoint(
      semantic_map_manager, route_section_sequence, blacklist,
      absl::MakeSpan(destinations), origin);
}
}  // namespace

// Search for RouteSectionSequence.
absl::StatusOr<RouteSectionSequence> SearchForRouteSectionSequenceFromLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RoutingRequestProto &routing_request, bool use_time) {
  QCOUNTER_FUNC();
  ASSIGN_OR_RETURN(
      const auto destinations,
      ParseDestinationProtoToLanePoints(semantic_map_manager, routing_request));

  for (int i = 0; i < destinations.size(); ++i) {
    const auto &destination = destinations[i];
    if (destination.lane_id() == mapping::kInvalidElementId) {
      if (routing_request.destinations(i).has_off_road()) {
        QLOG(INFO) << "Destination " << i << " is off road end."
                   << " Now  not support  off road on route.";
      }
      return absl::UnavailableError(
          absl::StrFormat("Can not locate destination {%s} on map!",
                          routing_request.destinations(i).DebugString()));
    }
  }
  return SearchForRouteSectionSequence(origin, absl::MakeSpan(destinations),
                                       semantic_map_manager, use_time);
}

absl::StatusOr<std::pair<RouteSectionSequence, mapping::LanePoint>>
SearchForRouteSectionSequenceFromPose(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto, const PoseProto &pose,
    const RoutingRequestProto &routing_request, bool use_time) {
  QCOUNTER_FUNC();
  // get multi lane point in radius and headings
  const double radius_error =
      route_param_proto.on_driving_param().filter().radius_error();
  const double heading_error =
      route_param_proto.on_driving_param().filter().heading_error();

  VLOG(3) << "radius_error:" << radius_error
          << ", heading_error:" << heading_error;
  constexpr double kPredictTime = 0.1;
  constexpr double kMaxAdditionLength = 2.0;
  // future car smooth
  Vec2d addition_smooth =
      kPredictTime * Vec2d(pose.vel_smooth().x(), pose.vel_smooth().y());
  if (addition_smooth.squaredNorm() > Sqr(kMaxAdditionLength)) {
    addition_smooth = kMaxAdditionLength * addition_smooth.Unit();
  }
  Vec2d car_smooth =
      Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()) + addition_smooth;
  VLOG(3) << "car smooth="
          << Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()).DebugString()
          << " + " << addition_smooth.DebugString() << " = "
          << car_smooth.DebugString();

  double car_heading = pose.yaw();
  const Vec2d car_heading_vec = Vec2d::FastUnitFromAngle(car_heading);
  VLOG(3) << "origin point:" << car_smooth.DebugString()
          << " heading=" << car_heading;
  const auto level_id = semantic_map_manager.GetLevel();
  const auto point_to_lanes = mapping::PointToNearLanesWithHeadingAtLevel(
      level_id, semantic_map_manager, car_smooth, radius_error, car_heading,
      heading_error);
  if (point_to_lanes.empty()) {
    return absl::OutOfRangeError(
        absl::StrFormat("Could not find lane near pose: %s at level: %d",
                        car_smooth.DebugString(), level_id));
  }
  std::vector<const mapping::LaneInfo *> lane_infos;
  lane_infos.reserve(point_to_lanes.size());
  for (const auto &point_to_lane : point_to_lanes) {
    lane_infos.push_back(point_to_lane.lane_info);
  }
  std::vector<mapping::LanePath> candidate_lane_paths;
  absl::flat_hash_set<mapping::ElementId> section_ids;
  for (const auto &lane_info : lane_infos) {
    VLOG(3) << "candidate lane:" << lane_info->id;
    // black list
    if (route_param_proto.on_driving_param().filter().motoway_only() &&
        mapping::IsPassengerVehicleAvoidLaneType(lane_info->proto->type()))
      continue;
    const auto pair_it = section_ids.insert(lane_info->section_id);
    if (pair_it.second) {
      candidate_lane_paths.emplace_back(
          mapping::LanePath(&semantic_map_manager, {lane_info->id}, 0.0, 1.0));
    }
  }
  if (candidate_lane_paths.empty()) {
    return absl::UnavailableError(
        absl::StrFormat("Cannot find candidate lane paths near pose: %s",
                        car_smooth.DebugString()));
  }

  std::vector<std::pair<RouteSectionSequence, mapping::LanePoint>>
      candidate_section_sequence;
  for (const auto &lane_path : candidate_lane_paths) {
    auto lane_point_or = mapping::
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            level_id, semantic_map_manager, car_smooth, lane_path, car_heading);
    if (!lane_point_or.ok()) {
      QLOG(WARNING) << "Found invalid id from pose: "
                    << car_smooth.DebugString()
                    << " with heading: " << car_heading
                    << " on lane path: " << lane_path.DebugString();
      continue;
    }
    const auto this_sectionsequence_or =
        SearchForRouteSectionSequenceFromLanePoint(
            semantic_map_manager, *lane_point_or, routing_request, use_time);
    if (!this_sectionsequence_or.ok()) {
      QLOG(WARNING) << "Failed to find path from "
                    << lane_point_or->DebugString()
                    << " for request : " << routing_request.ShortDebugString();
      continue;
    }
    candidate_section_sequence.push_back(
        std::make_pair(std::move(this_sectionsequence_or).value(),
                       std::move(lane_point_or).value()));
  }

  if (candidate_section_sequence.empty()) {
    return absl::UnavailableError(absl::StrFormat(
        "Could not find a section sequence due to closest lane point near "
        "pose: %s",
        pose.ShortDebugString()));
  }

  double total_distance = 0.0;
  double total_heading_error = 0.0;
  double total_length = 0.0;
  double total_deviate_cost = 0.0;

  constexpr double kMaxDistance = 3.0;
  constexpr double kExtraCost = 3.0;

  constexpr double w1 = 1.5;
  constexpr double w2 = 1.5;
  constexpr double w3 = 1.0;
  constexpr double w4 = 0.2;

  const auto distance_fun = [](double distance) {
    return std::max(0.2, distance);
  };

  const auto length_fun = [](double length) {
    return std::min(1e8, std::exp(0.015 * length));
  };

  const auto heading_error_fun = [](double cos_theta) {
    return std::max(0.01, -0.5 * cos_theta + 0.5);
  };

  const auto deviate_cost_fun = [&heading_error_fun](Vec2d car_heading_vec,
                                                     Vec2d addition_smooth,
                                                     Vec2d deviate_vec) {
    const Vec2d deviate_vec_unit = deviate_vec.Unit();
    return heading_error_fun(car_heading_vec.Dot(deviate_vec_unit)) +
           (addition_smooth == Vec2d::Zero()
                ? 0.0
                : heading_error_fun(
                      addition_smooth.Unit().Dot(deviate_vec_unit)));
  };

  const auto sections_distance = [](const RouteSectionSequence &section_seq) {
    double distance = 0.0;
    for (const auto &section : section_seq.sections()) {
      distance += section.length();
    }
    return distance;
  };

  for (const auto &section_seq : candidate_section_sequence) {
    const auto &this_lane_point = section_seq.second;

    total_distance += distance_fun(car_smooth.DistanceTo(
        this_lane_point.ComputePos(semantic_map_manager)));

    total_heading_error += heading_error_fun(car_heading_vec.Dot(
        this_lane_point.ComputeTangent(semantic_map_manager)));

    total_length += length_fun(sections_distance(section_seq.first));
    const Vec2d deviate_vec =
        this_lane_point.ComputePos(semantic_map_manager) - car_smooth;
    total_deviate_cost +=
        deviate_cost_fun(car_heading_vec, addition_smooth, deviate_vec);
  }
  // debug, do not remove!
  const auto debug_cost = [&]() {
    std::string debug_str;
    for (const auto &section_seq : candidate_section_sequence) {
      const auto &this_lane_point = section_seq.second;

      double distance = distance_fun(car_smooth.DistanceTo(
          this_lane_point.ComputePos(semantic_map_manager)));

      double heading_error = heading_error_fun(car_heading_vec.Dot(
          this_lane_point.ComputeTangent(semantic_map_manager)));

      double path_length = length_fun(sections_distance(section_seq.first));

      const Vec2d deviate_vec =
          this_lane_point.ComputePos(semantic_map_manager) - car_smooth;
      double deviation =
          deviate_cost_fun(car_heading_vec, addition_smooth, deviate_vec);

      absl::StrAppend(&debug_str, "lane_point:", this_lane_point.DebugString(),
                      " distance=", distance, " heading_error=", heading_error,
                      " path length=", path_length, " deviation=", deviation,
                      "\n");

      double cost = w1 * (distance) / total_distance +
                    w2 * (heading_error) / total_heading_error +
                    w3 * (path_length) / total_length +
                    w4 * deviation / total_deviate_cost +
                    (distance > kMaxDistance ? kExtraCost : 0.0);
      absl::StrAppend(&debug_str, "cost: ", w1 * (distance) / total_distance,
                      " + ", w2 * (heading_error) / total_heading_error, " + ",
                      w3 * (path_length) / total_length, " + ",
                      w4 * deviation / total_deviate_cost, " = ", cost, "\n");
    }
    return debug_str;
  };

  VLOG(3) << "------debug-------\n" << debug_cost();
  const auto &it = std::min_element(
      candidate_section_sequence.begin(), candidate_section_sequence.end(),
      [&](const std::pair<RouteSectionSequence, mapping::LanePoint>
              &section_seq1,
          const std::pair<RouteSectionSequence, mapping::LanePoint>
              &section_seq2) {
        const auto &cost =
            [&](const std::pair<RouteSectionSequence, mapping::LanePoint>
                    &section_seq) {
              const auto &lane_point = section_seq.second;
              // distance
              double distance = distance_fun(car_smooth.DistanceTo(
                  lane_point.ComputePos(semantic_map_manager)));

              // heading
              double heading_error = heading_error_fun(car_heading_vec.Dot(
                  lane_point.ComputeTangent(semantic_map_manager)));

              // length
              double length = length_fun(sections_distance(section_seq.first));
              // deviation
              const Vec2d deviate_vec =
                  lane_point.ComputePos(semantic_map_manager) - car_smooth;
              double deviation = deviate_cost_fun(car_heading_vec,
                                                  addition_smooth, deviate_vec);
              // cost
              return w1 * distance / total_distance +
                     w2 * heading_error / total_heading_error +
                     w3 * length / total_length +
                     w4 * deviation / total_deviate_cost +
                     (distance > kMaxDistance ? kExtraCost : 0.0);
            };
        return cost(section_seq1) < cost(section_seq2);
      });

  return *it;
}

absl::StatusOr<RouteSectionSequence>
SearchForRouteSectionSequenceFromDestination(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto &destination_proto,
    const RoutingRequestProto &routing_request, bool use_time) {
  QCOUNTER_FUNC();
  const auto lane_point =
      ParseDestinationProtoToLanePoint(semantic_map_manager, destination_proto);
  if (!lane_point.ok()) {
    return lane_point.status();
  }
  return SearchForRouteSectionSequenceFromLanePoint(
      semantic_map_manager, lane_point.value(), routing_request, use_time);
}

absl::StatusOr<RouteSectionSequence>
SearchForRouteSectionSequenceAlongLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations, bool use_time) {
  return SearchForRouteSectionSequence(origin, destinations,
                                       semantic_map_manager, use_time);
}

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromLanePoint(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time) {
  QCOUNTER_FUNC();
  ASSIGN_OR_RETURN(
      const auto section_seq,
      SearchForRouteSectionSequenceFromLanePoint(semantic_map_manager, origin,
                                                 routing_request, use_time));

  ASSIGN_OR_RETURN(const auto composite_lane_path,
                   SearchForCompositeLanePathFromSectionsViaRoutingRequest(
                       semantic_map_manager, section_seq, blacklist,
                       routing_request, origin));

  return std::make_pair(std::move(section_seq), std::move(composite_lane_path));
}

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromPose(
    const SemanticMapManager &semantic_map_manager,
    const RouteParamProto &route_param_proto, const PoseProto &pose,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time) {
  QCOUNTER_FUNC();
  ASSIGN_OR_RETURN(const auto section_seq,
                   SearchForRouteSectionSequenceFromPose(
                       semantic_map_manager, route_param_proto, pose,
                       routing_request, use_time));

  ASSIGN_OR_RETURN(const auto composite_lane_path,
                   SearchForCompositeLanePathFromSectionsViaRoutingRequest(
                       semantic_map_manager, section_seq.first, blacklist,
                       routing_request, section_seq.second));

  return std::make_pair(std::move(section_seq).first,
                        std::move(composite_lane_path));
}

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathFromDestination(
    const SemanticMapManager &semantic_map_manager,
    const RoutingDestinationProto &destination_proto,
    const RoutingRequestProto &routing_request,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time) {
  QCOUNTER_FUNC();
  ASSIGN_OR_RETURN(
      const auto section_seq,
      SearchForRouteSectionSequenceFromDestination(
          semantic_map_manager, destination_proto, routing_request, use_time));

  ASSIGN_OR_RETURN(const auto lane_point,
                   ParseDestinationProtoToLanePoint(semantic_map_manager,
                                                    destination_proto));

  ASSIGN_OR_RETURN(const auto composite_lane_path,
                   SearchForCompositeLanePathFromSectionsViaRoutingRequest(
                       semantic_map_manager, section_seq, blacklist,
                       routing_request, lane_point));

  return std::make_pair(std::move(section_seq), std::move(composite_lane_path));
}

absl::StatusOr<std::pair<RouteSectionSequence, CompositeLanePath>>
SearchForRouteLanePathAlongLanePoints(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    const absl::flat_hash_set<mapping::ElementId> &blacklist, bool use_time) {
  QCOUNTER_FUNC();
  ASSIGN_OR_RETURN(const auto section_seq,
                   SearchForRouteSectionSequence(
                       origin, destinations, semantic_map_manager, use_time));
  ASSIGN_OR_RETURN(
      const auto composite_lane_path,
      SearchForCompositeLanePathFromSectionsViaLanePoint(
          semantic_map_manager, section_seq, blacklist, destinations, origin));

  return std::make_pair(std::move(section_seq), std::move(composite_lane_path));
}

absl::StatusOr<CompositeLanePath> SearchForRouteLanePathFromSectionSequence(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin,
    const RouteSectionSequence &section_sequence,
    const absl::flat_hash_set<mapping::ElementId> &blacklist) {
  QCOUNTER_FUNC();
  // In the end section, only the destination lane has a driving distance.
  mapping::ElementId destination_lane_id;
  for (const auto &lane : section_sequence.sections().back().lanes) {
    if (lane.second > 0.0) destination_lane_id = lane.first;
  }
  mapping::LanePoint destination(
      destination_lane_id, section_sequence.sections().back().end_fraction);

  return FindRouteLanePathOnSectionSequence(
      RouteLanePathFinderInput{.section_sequence = &section_sequence,
                               .lane_id_blacklist = blacklist,
                               .start_point = origin,
                               .destination_point = std::move(destination)},

      semantic_map_manager);
}

}  // namespace planner
}  // namespace qcraft
