#include "onboard/planner/router/route_searcher.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <string>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

using mapping::LanePath;
using mapping::LanePoint;

namespace {
double GetInfeasibleTurnCost(const SemanticMapManager &semantic_map_manager,
                             const mapping::LaneInfo &prev,
                             const mapping::LaneInfo &cur,
                             double infeasible_u_turn_cost = 200.0,
                             double min_radius = 5.0) {
  if (!FLAGS_route_turn_cost_enable) {
    return 0.0;
  }
  switch (cur.direction) {
    case mapping::LaneProto::UTURN:
      if (!cur.outgoing_lane_indices.empty()) {
        const mapping::LaneInfo &target_lane =
            semantic_map_manager.lane_info()[cur.outgoing_lane_indices[0]];
        const auto &section_info =
            semantic_map_manager.FindSectionInfoOrDie(target_lane.section_id);
        const int lane_num = section_info.lane_ids.size();
        const double road_width = kDefaultLaneWidth * lane_num;
        return (road_width < 2 * min_radius) ? infeasible_u_turn_cost : 0.0;
      }
      break;
    case mapping::LaneProto::STRAIGHT:
    case mapping::LaneProto::LEFT_TURN:
    case mapping::LaneProto::RIGHT_TURN:
      return 0.0;
  }
  return 0.0;
}

}  // namespace

bool RouteSearcher::SearchForRouteToDestination(
    LanePoint origin, LanePoint destination,
    const absl::flat_hash_map<mapping::ElementId, double> &lane_costs,
    LanePath *lane_path) const {
  SCOPED_QTRACE("search_for_route");
  QCHECK(destination.Valid()) << destination.DebugString();

  const int num_lanes = semantic_map_manager_->semantic_map().lanes_size();

  // A* search.
  absl::flat_hash_map<mapping::ElementId, bool> closed;
  // Best cumulative cost for reaching the start of the given lane.
  absl::flat_hash_map<mapping::ElementId, double> cum_costs;
  // The predecessor lane in the best route to each lane.
  absl::flat_hash_map<mapping::ElementId, mapping::ElementId> from;
  for (int i = 0; i < num_lanes; ++i) {
    const mapping::ElementId id =
        semantic_map_manager_->semantic_map().lanes(i).id();
    QCHECK_NE(id, mapping::kInvalidElementId);
    closed[id] = false;
    cum_costs[id] = std::numeric_limits<double>::infinity();
    from[id] = mapping::kInvalidElementId;
  }

  // (current lane point, predecessor lane).
  using SemanticMapPointStep = std::pair<LanePoint, mapping::ElementId>;
  // A* heuristic cost estimate: the Euclidean distance from the lane
  // endpoint to destination lane start, plus the lane arclength between the
  // LanePoint and the lane endpoint.
  const auto h = [this, &destination](LanePoint lane_point) {
    if (lane_point.lane_id() == destination.lane_id()) return 0.0;
    const double dist =
        (semantic_map_manager_->GetLaneControlPointsOrDie(lane_point.lane_id())
             .back() -
         semantic_map_manager_->GetLaneControlPointsOrDie(destination.lane_id())
             .front())
            .norm() +
        semantic_map_manager_->GetLaneLengthOrDie(lane_point.lane_id()) *
            (1.0 - lane_point.fraction());
    if (is_use_time()) {
      constexpr double kMaxSpeedLimit = 65.0;  // mph.
      return dist / Mph2Mps(kMaxSpeedLimit);
    } else {
      return dist;
    }
  };

  // A* cost: actual best cumulative cost plus estimated cost to destination.
  const auto cost = [&lane_costs, &cum_costs, &h](SemanticMapPointStep step) {
    const double lane_cost = FindOrDie(lane_costs, step.first.lane_id());
    return FindOrDie(cum_costs, step.first.lane_id()) +
           (lane_cost == std::numeric_limits<double>::infinity()
                ? std::numeric_limits<double>::infinity()
                : lane_cost * step.first.fraction()) +
           h(step.first);
  };

  const auto cost_comp = [&cost](SemanticMapPointStep step0,
                                 SemanticMapPointStep step1) {
    // We want the smallest cost first, so this is a greater comparator.
    return cost(step0) > cost(step1);
  };

  using OpenQueue = std::priority_queue<SemanticMapPointStep,
                                        std::vector<SemanticMapPointStep>,
                                        decltype(cost_comp)>;

  ////////////////////////// Debug String //////////////////////////////
  const auto open_queue_debug_string = [&cost](const OpenQueue &queue) {
    std::string output;
    OpenQueue tmp = queue;
    while (!tmp.empty()) {
      absl::StrAppendFormat(
          &output, "step: lane point: %s, predecessor: %d value: %f; ",
          tmp.top().first.DebugString(), tmp.top().second, cost(tmp.top()));
      tmp.pop();
    }
    return output;
  };
  const auto closed_set_debug_string =
      [](const absl::flat_hash_map<mapping::ElementId, bool> &closed) {
        std::string output;
        for (const auto &kv : closed) {
          if (kv.second) {
            absl::StrAppend(&output, " ", kv.first);
          }
        }
        return output;
      };
  const auto lane_ids_debug_string =
      [this](absl::Span<const mapping::ElementId> lane_ids) {
        std::string output;
        for (auto it = lane_ids.begin(); it != lane_ids.end(); ++it) {
          if (it != lane_ids.begin()) {
            const auto &lane = semantic_map_manager_->FindLaneByIdOrDie(*it);
            if (std::find(lane.incoming_lanes().begin(),
                          lane.incoming_lanes().end(),
                          *(it - 1)) == lane.incoming_lanes().end()) {
              absl::StrAppend(&output, "(lanechange)");
            }
          }
          absl::StrAppend(&output, *it, "->");
        }
        return output;
      };
  //////////////////////////////////////////////////////////////////////

  OpenQueue open_queue(cost_comp);

  open_queue.emplace(origin, mapping::kInvalidElementId);

  bool destination_reached = false;
  while (!open_queue.empty()) {
    const SemanticMapPointStep current_best_step = open_queue.top();
    const LanePoint current_point = current_best_step.first;
    const mapping::ElementId current_predecessor = current_best_step.second;
    VLOG(2) << "Current best step: " << current_point.DebugString()
            << " predessor: " << current_predecessor
            << " cum_cost: " << cum_costs[current_point.lane_id()]
            << " in-lane cost: "
            << (FindOrDie(lane_costs, current_point.lane_id()) ==
                        std::numeric_limits<double>::infinity()
                    ? std::numeric_limits<double>::infinity()
                    : FindOrDie(lane_costs, current_point.lane_id()) *
                          current_point.fraction())
            << " h: " << h(current_point);
    VLOG(2) << "Current open queue: " << open_queue_debug_string(open_queue);
    VLOG(2) << "Current closed_set: " << closed_set_debug_string(closed);
    open_queue.pop();
    // Since std::priority_queue does not have an operation to adjust an
    // existing element's priority, we push the element again with the new
    // priority each time we need to adjust an element. This means a same
    // element could occur multiple times in the queue; the one that pops
    // up first is the one with the most recent adjustment, and the rest
    // should be discarded upon pop.

    if (current_point.lane_id() == destination.lane_id()) {
      if (current_point.fraction() == destination.fraction()) {
        VLOG(2) << "Destination reached (destination: " << destination.lane_id()
                << ":" << destination.fraction() << ")";
        destination_reached = true;
        from[destination.lane_id()] = current_predecessor;
        break;
      } else if (current_point.fraction() < destination.fraction()) {
        VLOG(2) << "Reaching destination (destination: "
                << destination.lane_id() << ":" << destination.fraction()
                << ")";
        open_queue.push(std::make_pair(destination, current_predecessor));
        continue;
      }
    }

    const auto &current_lane_info =
        semantic_map_manager_->FindLaneInfoOrDie(current_point.lane_id());
    const double prev_cum_cost =
        current_point.lane_id() == origin.lane_id()
            ? 0.0
            : FindOrDie(cum_costs, current_point.lane_id());

    // Go to next lane from current point
    const double cost_on_lane = FindOrDie(lane_costs, current_point.lane_id()) *
                                (1.0 - current_point.fraction());
    const double new_cost = prev_cum_cost + cost_on_lane;
    for (const auto next_lane_idx : current_lane_info.outgoing_lane_indices) {
      const auto &nex_lane_info =
          semantic_map_manager_->lane_info()[next_lane_idx];
      const mapping::ElementId next_lane_id = nex_lane_info.id;
      const double turn_cost = GetInfeasibleTurnCost(
          *semantic_map_manager_, current_lane_info, nex_lane_info, 200.0, 5.0);
      QCHECK_NE(next_lane_id, mapping::kInvalidElementId);

      // The second condition is for some special cases when origin and
      // destination are neighbours but destination is behind origin. So maybe
      // we need choose destination lane twice.
      if (FindOrDie(cum_costs, next_lane_id) > new_cost ||
          next_lane_id == destination.lane_id()) {
        cum_costs[next_lane_id] = new_cost + turn_cost;
        open_queue.emplace(LanePoint(next_lane_id, 0.0), current_lane_info.id);
        from[next_lane_id] = current_lane_info.id;
        VLOG(2) << current_lane_info.id << " go to next lane " << next_lane_id;
      }
    }

    // Lane change from current point
    if (FLAGS_allow_routed_lane_change) {
      const auto neighbors_info = SearchForNeighborsInfo(current_point, from);

      for (const auto &[neighbor_info, lc_cost] : neighbors_info) {
        const double dis_on_this_lane =
            current_lane_info.length() *
            std::max(current_point.fraction(),
                     neighbor_info.this_start_fraction);
        const double cum_cost_till_lc_start =
            prev_cum_cost +
            (is_use_time() ? (dis_on_this_lane / current_lane_info.speed_limit)
                           : dis_on_this_lane);

        const auto &neighbor_lane_info =
            semantic_map_manager_->FindLaneInfoOrDie(neighbor_info.other_id);

        double other_start_fraction = neighbor_info.other_start_fraction;
        if (current_point.fraction() > neighbor_info.this_start_fraction) {
          other_start_fraction +=
              current_lane_info.length() *
              (current_point.fraction() - neighbor_info.this_start_fraction) /
              neighbor_lane_info.length();
        }

        const double dis_on_neighbor_lane =
            neighbor_lane_info.length() * other_start_fraction;
        const double neighbor_cum_cost =
            cum_cost_till_lc_start + lc_cost -
            (is_use_time()
                 ? dis_on_neighbor_lane / neighbor_lane_info.speed_limit
                 : dis_on_neighbor_lane);

        if (FindOrDie(cum_costs, neighbor_lane_info.id) > neighbor_cum_cost ||
            neighbor_lane_info.id == destination.lane_id()) {
          cum_costs[neighbor_lane_info.id] = neighbor_cum_cost;
          open_queue.emplace(
              LanePoint(neighbor_lane_info.id, other_start_fraction),
              current_lane_info.id);
          VLOG(2) << current_point.lane_id() << " lane change to "
                  << neighbor_lane_info.id;
          from[neighbor_lane_info.id] = current_lane_info.id;
        }
      }
    }
  }

  if (!destination_reached) {
    QLOG(ERROR) << "Failed to route from " << origin.DebugString()
                << " to destination " << destination.DebugString();
    return false;
  }

  std::vector<mapping::ElementId> lane_ids;
  lane_ids.push_back(destination.lane_id());
  while (from[lane_ids.back()] != mapping::kInvalidElementId) {
    lane_ids.push_back(from[lane_ids.back()]);

    if (lane_ids.back() == origin.lane_id()) break;
  }
  std::reverse(lane_ids.begin(), lane_ids.end());

  VLOG(2) << lane_ids_debug_string(lane_ids);

  *lane_path = LanePath(semantic_map_manager_, std::move(lane_ids),
                        origin.fraction(), destination.fraction());
  return true;
}

bool RouteSearcher::SearchForRouteBySections(
    LanePoint origin, LanePoint destination,
    absl::Span<const double> section_costs,
    std::vector<mapping::ElementId> *section_ids) const {
  SCOPED_QTRACE("search_for_route_by_sections");

  const int num_sections =
      semantic_map_manager_->semantic_map().sections_size();

  // A* search, all processes are based on idx
  std::vector<bool> closed(num_sections, false);
  // Least cumulative cost for reaching current section
  std::vector<double> cum_costs(num_sections,
                                std::numeric_limits<double>::infinity());
  // The predecessor section of current section
  std::vector<int> from(num_sections, mapping::kInvalidElementId);

  // find origin section idx and destination section idx
  const auto &origin_lane_info =
      semantic_map_manager_->FindLaneInfoOrDie(origin.lane_id());
  const int origin_section_idx =
      semantic_map_manager_->FindSectionInfoOrDie(origin_lane_info.section_id)
          .idx;
  const auto &destination_lane_info =
      semantic_map_manager_->FindLaneInfoOrDie(destination.lane_id());
  const int destination_section_idx =
      semantic_map_manager_
          ->FindSectionInfoOrDie(destination_lane_info.section_id)
          .idx;

  // current section idx, predecessor section idx
  using SemanticMapSectionStep = std::pair<int, int>;
  // A* heuristic cost estimate: the same as route search, choose the Eucliden
  // distance from the endpoint of an arbitrary lane (e.g. the first)
  // in a section to the start of an arbitrary lane in final section.
  const Vec2d dest_section_point =
      semantic_map_manager_
          ->GetLaneControlPointsOrDie(
              semantic_map_manager_->section_info()[destination_section_idx]
                  .lane_ids[0])
          .front();
  const auto h = [this, &destination_section_idx,
                  &dest_section_point](int section_idx) {
    if (section_idx == destination_section_idx) return 0.0;
    const double dist =
        (semantic_map_manager_
             ->GetLaneControlPointsOrDie(
                 semantic_map_manager_->section_info()[section_idx].lane_ids[0])
             .back() -
         dest_section_point)
            .norm();
    if (is_use_time()) {
      constexpr double kMaxSpeedLimit = 65.0;  // mph.
      return dist / Mph2Mps(kMaxSpeedLimit);
    } else {
      return dist;
    }
  };

  // A* cost: actual least cumulative cost plus estimated cost to final section.
  // cost = pre + now + heuristic
  const auto cost = [&section_costs, &h,
                     &cum_costs](SemanticMapSectionStep step) {
    const double section_cost = section_costs[step.first];
    return cum_costs[step.first] + section_cost + h(step.first);
  };

  const auto cost_comp = [&cost](SemanticMapSectionStep step0,
                                 SemanticMapSectionStep step1) {
    return cost(step0) > cost(step1);
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

    closed[current_section_idx] = true;

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
        semantic_map_manager_->section_info()[current_section_idx];
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
    return false;
  }

  section_ids->emplace_back(destination_lane_info.section_id);
  while (from[semantic_map_manager_->FindSectionInfoOrDie(section_ids->back())
                  .idx] != mapping::kInvalidElementId) {
    mapping::ElementId prev_section_id =
        semantic_map_manager_
            ->section_info()[from[semantic_map_manager_
                                      ->FindSectionInfoOrDie(
                                          section_ids->back())
                                      .idx]]
            .id;
    section_ids->emplace_back(prev_section_id);

    if (section_ids->back() == origin_lane_info.section_id) break;
  }

  std::reverse(section_ids->begin(), section_ids->end());

  return true;
}

bool RouteSearcher::SearchForRouteSections(
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    absl::Span<const double> section_costs,
    std::vector<mapping::ElementId> *section_ids) const {
  mapping::LanePoint current_origin = origin;
  for (const mapping::LanePoint &destination : destinations) {
    QCHECK(destination.Valid())
        << "Invalid destination: " << destination.DebugString();
    std::vector<mapping::ElementId> current_section_sequence;
    if (!SearchForRouteBySections(current_origin, destination, section_costs,
                                  &current_section_sequence)) {
      return false;
    }
    current_origin = destination;
    if (!section_ids->empty()) {
      section_ids->pop_back();
    }
    section_ids->insert(section_ids->end(), current_section_sequence.begin(),
                        current_section_sequence.end());
  }

  return true;
}

bool RouteSearcher::SearchForRoutePathWithLaneCosts(
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    const absl::flat_hash_map<mapping::ElementId, double> &lane_costs,
    CompositeLanePath *route_path) const {
  std::vector<mapping::ElementId> lane_sequence;
  mapping::LanePoint leg_origin = origin;
  for (const mapping::LanePoint &destination : destinations) {
    VLOG(2) << "Routing from " << leg_origin.DebugString() << " to destination "
            << destination.DebugString();
    if (!destination.Valid()) {
      QLOG(ERROR) << "destination is invalid: " << destination.DebugString();
      return false;
    }
    LanePath lane_path(semantic_map_manager_);
    if (!SearchForRouteToDestination(leg_origin, destination, lane_costs,
                                     &lane_path)) {
      return false;
    }
    VLOG(2) << "Lane path found: " << lane_path.DebugString();

    leg_origin = destination;
    if (!lane_sequence.empty()) lane_sequence.pop_back();
    lane_sequence.insert(lane_sequence.end(), lane_path.lane_ids().begin(),
                         lane_path.lane_ids().end());
  }

  *route_path = BuildCompositeLanePathFromLaneIds(
      lane_sequence, origin.fraction(), destinations.back().fraction());

  return true;
}

bool RouteSearcher::SearchForRoutePathToLanePoints(
    const mapping::LanePoint &origin,
    absl::Span<const mapping::LanePoint> destinations,
    CompositeLanePath *route_path) const {
  for (const auto &destination : destinations) {
    if (!destination.Valid()) {
      QLOG(ERROR) << "destination is invalid: " << destination.DebugString();
      return false;
    }
  }
  absl::flat_hash_map<mapping::ElementId, double> lane_costs;
  CreateDefaultLaneCosts(&lane_costs);
  return SearchForRoutePathWithLaneCosts(origin, destinations, lane_costs,
                                         route_path);
}

bool RouteSearcher::SearchForRoute(
    mapping::LanePoint origin,
    absl::Span<const mapping::LanePoint> destinations,
    RouteProto *route_proto) const {
  route_proto->Clear();
  for (const auto destination : destinations) {
    if (!destination.Valid()) {
      QLOG(ERROR) << "destination is invalid: " << destination.DebugString();
      return false;
    }
  }
  CompositeLanePath route_path;
  if (!SearchForRoutePathToLanePoints(origin, destinations, &route_path)) {
    return false;
  }

  for (const auto &seg : route_path.lane_paths()) {
    for (const auto id : seg.lane_ids()) {
      route_proto->mutable_lane_ids()->Add(id);
    }
  }
  route_proto->set_start_fraction(route_path.front().fraction());
  route_proto->set_end_fraction(route_path.back().fraction());
  route_path.ToProto(route_proto->mutable_lane_path());
  return true;
}

void RouteSearcher::CreateDefaultLaneCosts(
    absl::flat_hash_map<mapping::ElementId, double> *lane_costs) const {
  const int num_lanes = semantic_map_manager_->semantic_map().lanes_size();

  for (int i = 0; i < num_lanes; ++i) {
    const mapping::ElementId id =
        semantic_map_manager_->semantic_map().lanes(i).id();
    const auto &lane_info = semantic_map_manager_->FindLaneInfoOrDie(id);

    // Cost from lane traversal time at speed limit.
    (*lane_costs)[id] = is_use_time()
                            ? lane_info.length() / lane_info.speed_limit
                            : lane_info.length();
  }
}

void RouteSearcher::CreateDefaultSectionCosts(
    std::vector<double> *section_costs) const {
  const int num_sections =
      semantic_map_manager_->semantic_map().sections_size();

  for (int i = 0; i < num_sections; ++i) {
    const auto &section_info = semantic_map_manager_->section_info()[i];

    // Cost from average section traversal time at speed limit.
    (*section_costs)[i] = is_use_time() ? section_info.average_length /
                                              section_info.average_speed_limit
                                        : section_info.average_length;
  }
}

CompositeLanePath RouteSearcher::BuildCompositeLanePathFromLaneIds(
    absl::Span<const mapping::ElementId> lane_ids, double start_fraction,
    double end_fraction) const {
  std::vector<mapping::LanePath> lane_paths;
  std::vector<CompositeLanePath::TransitionInfo> transitions;
  std::vector<mapping::ElementId> tmp_lane_ids;

  double now_start_fraction = start_fraction;
  for (auto it = lane_ids.begin(); it != lane_ids.end(); ++it) {
    tmp_lane_ids.push_back(*it);

    // Handle last lane
    if (it == lane_ids.end() - 1) {
      lane_paths.emplace_back(semantic_map_manager_, tmp_lane_ids,
                              now_start_fraction, end_fraction);
      break;
    }

    const auto &current_lane = semantic_map_manager_->FindLaneInfoOrDie(*it);
    const auto &out_lane_ids = current_lane.outgoing_lane_indices;

    // Lanes are not connected, split them with a transition
    if (std::find_if(
            out_lane_ids.begin(), out_lane_ids.end(), [this, &it](auto idx) {
              const auto &out_lane = semantic_map_manager_->lane_info()[idx];
              return out_lane.id == *(it + 1);
            }) == out_lane_ids.end()) {
      bool lc_left;
      const auto overlap =
          GetLaneNeighborInfo(current_lane, *(it + 1), &lc_left);

      const double overlap_len =
          current_lane.length() *
          (overlap.this_end_fraction - overlap.this_start_fraction);
      lane_paths.emplace_back(semantic_map_manager_, tmp_lane_ids,
                              now_start_fraction, overlap.this_end_fraction);
      VLOG(3) << "Split lane " << *it << " and lane " << *(it + 1) << ". left "
              << lc_left;

      transitions.emplace_back(
          CompositeLanePath::TransitionInfo{.overlap_length = overlap_len,
                                            .transition_point_fraction = 0.5,
                                            .lc_left = lc_left});

      //  Align neighbor start fraction with current lane's start fraction
      //
      //  o-------------------------------------> next_lane
      //  |     delete      |
      //  o-------------------------------------> current_lane
      //  |     delete      |
      //                    o-------------------> prev_lane
      //                    ^
      //                    |

      if (tmp_lane_ids.size() == 1 &&
          now_start_fraction > overlap.this_start_fraction) {
        // NOTE: lanes are fully aligned in new semantic map. We can assume
        // this/other_start_fraction = 0.0 && this/other_end_fraction = 1.0
        // now_start_fraction inherts from last value.

      } else {
        now_start_fraction = overlap.other_start_fraction;
      }

      tmp_lane_ids.clear();
    }
  }

  QCHECK(lane_paths.size() == transitions.size() + 1);

  return RearrangeTransitionsOnRoutePath(
      *semantic_map_manager_,
      CompositeLanePath(std::move(lane_paths), std::move(transitions)));
}

std::vector<std::pair<mapping::LaneNeighborInfo, double>>
RouteSearcher::SearchForNeighborsInfo(
    const mapping::LanePoint &start_point,
    const absl::flat_hash_map<mapping::ElementId, mapping::ElementId> &from)
    const {
  const auto compute_lane_change_cost =
      [start_point](const mapping::LaneInfo &this_lane,
                    const mapping::LaneNeighborInfo &neighbor, bool use_time) {
        constexpr double kMinLaneChangeCost = 100.0;
        constexpr double kLaneChangeCost = 200.0;
        constexpr double kIllegalLaneChangeCost = 10000.0;
        const double overlap_len =
            this_lane.length() *
            (neighbor.this_end_fraction -
             std::max(neighbor.this_start_fraction, start_point.fraction()));

        // TODO(luzou, xiang): consider BROKEN_LEFT_DOUBLE_WHITE and
        // BROKEN_RIGHT_DOUBLE_WHITE
        const double distance_cost = std::fmax(
            kMinLaneChangeCost,
            -overlap_len + ((neighbor.lane_boundary_type ==
                                 mapping::LaneBoundaryProto::BROKEN_YELLOW ||
                             neighbor.lane_boundary_type ==
                                 mapping::LaneBoundaryProto::BROKEN_WHITE)
                                ? kLaneChangeCost
                                : kIllegalLaneChangeCost));

        return use_time ? (distance_cost / this_lane.speed_limit)
                        : distance_cost;
      };

  const mapping::LaneInfo &this_lane =
      semantic_map_manager_->FindLaneInfoOrDie(start_point.lane_id());

  auto neighbors = this_lane.lane_neighbors_on_left;
  neighbors.insert(neighbors.end(), this_lane.lane_neighbors_on_right.begin(),
                   this_lane.lane_neighbors_on_right.end());

  std::vector<std::pair<mapping::LaneNeighborInfo, double>> neighbors_info;

  for (const auto &neighbor : neighbors) {
    if (neighbor.opposite) continue;

    if (neighbor.this_end_fraction > start_point.fraction() &&
        FindOrDie(from, this_lane.id) != neighbor.other_id) {
      neighbors_info.emplace_back(
          neighbor, compute_lane_change_cost(this_lane, neighbor, use_time_));
    }
  }

  return neighbors_info;
}

}  // namespace planner
}  // namespace qcraft
