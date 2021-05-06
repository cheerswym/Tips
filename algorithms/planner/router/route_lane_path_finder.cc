#include "onboard/planner/router/route_lane_path_finder.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <stack>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(route_allow_solid_line_lane_change, false,
            "Whether to allow to lane change in solid line.");

namespace qcraft::planner {
namespace internal {
// PLANNER-346 If start/end lane is too short,  other peer lanes to blacklist
// Only use the whitelist strategy properly other than this blacklist strategy
// also works.
[[maybe_unused]] std::vector<mapping::ElementId> MinLaneChangeBlacklist(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &lane_point, bool is_origin) {
  std::vector<mapping::ElementId> blacklist;
  const mapping::LaneInfo &lane_info =
      semantic_map_manager.FindLaneInfoOrDie(lane_point.lane_id());
  double fraction =
      is_origin ? (1.0 - lane_point.fraction()) : lane_point.fraction();
  const double lane_pass_length = lane_info.length() * fraction;
  if (lane_pass_length < kMinLcLaneLength) {
    const mapping::SectionInfo &section =
        semantic_map_manager.FindSectionInfoOrDie(lane_info.section_id);
    for (auto lane_id : section.lane_ids) {
      if (lane_id != lane_point.lane_id()) {
        blacklist.push_back(lane_id);
      }
    }
  }
  return blacklist;
}

// TODO(luzou, xiang): decide which side we are considering.
bool CanLaneChange(
    const mapping::LaneNeighborInfo &neighbor_info,
    const mapping::BoundarySide side = mapping::BoundarySide::kAny) {
  return neighbor_info.lane_boundary_type ==
             mapping::LaneBoundaryProto::BROKEN_WHITE ||
         neighbor_info.lane_boundary_type ==
             mapping::LaneBoundaryProto::UNKNOWN_TYPE ||
         (neighbor_info.lane_boundary_type ==
              mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE &&
          side != mapping::BoundarySide::kLeft) ||
         (neighbor_info.lane_boundary_type ==
              mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE &&
          side != mapping::BoundarySide::kRight);
}
}  // namespace internal
namespace {

constexpr double kBlacklistCost = 1000.0;
constexpr double kLaneChangeCost = 100.0;
constexpr double kLaneChangeExtensionCost = 1.0;
constexpr double kLaneLengthCost = 1e-3;     // per meter.
constexpr double kIgnoreTailLength = 30.0;   // meter.
constexpr double kIgnoreStartLength = 15.0;  // meter.

struct RouteLaneIndex {
  int route_section_index = -1;
  int lane_index_in_section = -1;
};
struct RouteLaneInfo {
  double cum_cost = std::numeric_limits<double>::infinity();
  int cum_lane_change_cnt = 0;
  RouteLaneIndex prev_lane_index;
};

// If the origin lane or destination lane is in blacklist, and it is short,
// we take them out the blacklist.
void InsertLanesToWhitelist(
    const absl::flat_hash_set<mapping::ElementId> &blacklist,
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &start_search_point,
    const mapping::LaneInfo &start_lane_info, double ignore_length,
    bool is_start, absl::flat_hash_set<mapping::ElementId> *whitelist) {
  const auto func_next_lanes = [&semantic_map_manager](
                                   const mapping::ElementId &current_lane_id,
                                   bool is_start) {
    return is_start ? semantic_map_manager.FindLaneInfoOrDie(current_lane_id)
                          .outgoing_lane_indices
                    : semantic_map_manager.FindLaneInfoOrDie(current_lane_id)
                          .incoming_lane_indices;
  };

  std::stack<std::pair<mapping::ElementId, double>> white_lane;
  if (is_start) {
    white_lane.push(std::make_pair(
        start_search_point.lane_id(),
        start_lane_info.length() * (1.0 - start_search_point.fraction())));
  } else {
    white_lane.push(std::make_pair(
        start_search_point.lane_id(),
        start_lane_info.length() * start_search_point.fraction()));
  }

  while (!white_lane.empty()) {
    const auto current_lane = white_lane.top();
    white_lane.pop();
    if (blacklist.contains(current_lane.first)) {
      whitelist->insert(current_lane.first);
    }
    if (current_lane.second > ignore_length) {
      continue;
    }
    for (const auto &next_lane_index :
         func_next_lanes(current_lane.first, is_start)) {
      const auto &next_lane_info =
          semantic_map_manager.lane_info()[next_lane_index];
      white_lane.push(std::make_pair(
          next_lane_info.id, next_lane_info.length() + current_lane.second));
    }
  }
}

absl::flat_hash_set<mapping::ElementId> BuildWhitelistFromBlacklist(
    const absl::flat_hash_set<mapping::ElementId> &blacklist,
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin, const mapping::LanePoint &destination) {
  absl::flat_hash_set<mapping::ElementId> whitelist;
  // Process start lane.
  const auto &origin_info =
      semantic_map_manager.FindLaneInfoOrDie(origin.lane_id());
  if (origin_info.length() * (1.0 - origin.fraction()) < kIgnoreStartLength) {
    InsertLanesToWhitelist(blacklist, semantic_map_manager, origin, origin_info,
                           kIgnoreStartLength, /*is_start=*/true, &whitelist);
  }

  // Process destination lane.
  const auto &destination_info =
      semantic_map_manager.FindLaneInfoOrDie(destination.lane_id());
  if (destination_info.length() * destination.fraction() < kIgnoreTailLength) {
    InsertLanesToWhitelist(blacklist, semantic_map_manager, destination,
                           destination_info, kIgnoreTailLength,
                           /*is_start=*/false, &whitelist);
  }
  return whitelist;
}

bool NeedBlacklistCost(const absl::flat_hash_set<mapping::ElementId> &blacklist,
                       const absl::flat_hash_set<mapping::ElementId> &whitelist,
                       const mapping::ElementId &lane_id) {
  return blacklist.contains(lane_id) && !whitelist.contains(lane_id);
}

// Calculate cost if the current lane and the previous lane are connected.
RouteLaneInfo ConnectionLaneCost(
    const RouteLaneIndex &current_idx, const RouteLaneIndex &prev_idx,
    const RouteLanePathFinderInput &input,
    const absl::flat_hash_set<mapping::ElementId> &whitelist,
    absl::Span<const RouteSectionSequence::RouteSection> route_sections,
    const std::vector<std::vector<RouteLaneInfo>> &cost_table,
    const SemanticMapManager &semantic_map_manager) {
  double lane_current_cost = 0.0;
  const auto prev_lane_info =
      cost_table[prev_idx.route_section_index][prev_idx.lane_index_in_section];

  // 1. LaneLaneth cost.
  lane_current_cost +=
      semantic_map_manager
          .FindLaneInfoOrDie(route_sections[current_idx.route_section_index]
                                 .lanes[current_idx.lane_index_in_section]
                                 .first)
          .length() *
      (route_sections[current_idx.route_section_index].end_fraction -
       route_sections[current_idx.route_section_index].start_fraction) *
      kLaneLengthCost;

  // 2. Blacklist cost.
  const bool current_lane_in_blacklist =
      NeedBlacklistCost(input.lane_id_blacklist, whitelist,
                        route_sections[current_idx.route_section_index]
                            .lanes[current_idx.lane_index_in_section]
                            .first);
  const bool prev_lane_in_blacklist =
      NeedBlacklistCost(input.lane_id_blacklist, whitelist,
                        route_sections[prev_idx.route_section_index]
                            .lanes[prev_idx.lane_index_in_section]
                            .first);
  if (whitelist.find(route_sections[current_idx.route_section_index]
                         .lanes[current_idx.lane_index_in_section]
                         .first) != whitelist.end()) {
    // Do not add kBlacklistcost.
  } else if (prev_lane_in_blacklist || current_lane_in_blacklist) {
    lane_current_cost += kBlacklistCost;
  }

  // 3. ExtensionCost
  lane_current_cost +=
      prev_lane_info.cum_lane_change_cnt * kLaneChangeExtensionCost;

  // 4. Previous lane cost.
  lane_current_cost += prev_lane_info.cum_cost;

  // Update cost_table.
  auto current_route_lane_info = cost_table[current_idx.route_section_index]
                                           [current_idx.lane_index_in_section];

  if (lane_current_cost < current_route_lane_info.cum_cost) {
    current_route_lane_info.cum_cost = lane_current_cost;
    current_route_lane_info.cum_lane_change_cnt =
        prev_lane_info.cum_lane_change_cnt;
    current_route_lane_info.prev_lane_index = prev_idx;
  }

  return current_route_lane_info;
}

// Calculate cost if the current lane and the previous lane are the same
// section.
void SectionLaneChangeCost(const RouteLaneIndex &current_idx,
                           const RouteLaneIndex &prev_idx,
                           std::vector<RouteLaneInfo> *section_cost_table) {
  const auto prev_cost = section_cost_table->at(prev_idx.lane_index_in_section);

  // 1. LC cost.
  // 2. Plus previous lane cost.
  const double lane_current_cost = kLaneChangeCost + prev_cost.cum_cost;

  // Update cost_table.
  auto &current_route_lane_info =
      section_cost_table->at(current_idx.lane_index_in_section);

  if (lane_current_cost < current_route_lane_info.cum_cost) {
    current_route_lane_info.cum_cost = lane_current_cost;
    current_route_lane_info.cum_lane_change_cnt =
        prev_cost.cum_lane_change_cnt + 1;
    current_route_lane_info.prev_lane_index = prev_idx;
  }
}

// Update cost in the same section.
// We should the minimum cost lane indices due to connection relationship and
// update other lanes depends on them.
void UpdateSectionCost(const int &section_idx, std::queue<int> *min_cost_index,
                       std::vector<RouteLaneInfo> *section_cost_table) {
  const int lane_size = section_cost_table->size();
  if (lane_size == min_cost_index->size()) return;

  std::vector<bool> need_update(lane_size, true);

  while (!min_cost_index->empty()) {
    const int index = min_cost_index->front();
    min_cost_index->pop();
    RouteLaneIndex base_lane_index = {section_idx, index};
    need_update[index] = false;

    if (index > 0 && need_update[index - 1]) {
      RouteLaneIndex left_lane_index = {section_idx, index - 1};
      SectionLaneChangeCost(left_lane_index, base_lane_index,
                            section_cost_table);
      min_cost_index->push(index - 1);
    }
    if (index + 1 < lane_size && need_update[index + 1]) {
      RouteLaneIndex right_lane_index = {section_idx, index + 1};
      SectionLaneChangeCost(right_lane_index, base_lane_index,
                            section_cost_table);
      min_cost_index->push(index + 1);
    }
  }
}

absl::StatusOr<CompositeLanePath> PostProcessForRouteLanePath(
    const SemanticMapManager &semantic_map_manager,
    const std::vector<CompositeLanePath::LanePath> &original_lane_paths,
    const std::vector<CompositeLanePath::TransitionInfo> &original_transitions,
    bool empty_if_must_cross_solid_boundary) {
  std::vector<CompositeLanePath::LanePath> lane_paths = original_lane_paths;
  std::vector<CompositeLanePath::TransitionInfo> transitions =
      original_transitions;

  const auto get_lc_neighbor = [&semantic_map_manager](
                                   const mapping::ElementId &current_lane_id,
                                   bool turn_left) {
    const auto &lane_info =
        semantic_map_manager.FindLaneInfoOrDie(current_lane_id);
    const auto &lane_neighbors = turn_left ? lane_info.lane_neighbors_on_left
                                           : lane_info.lane_neighbors_on_right;
    return lane_neighbors.empty() ? std::nullopt
                                  : std::optional(lane_neighbors.front());
  };

  const auto find_extend_lane_ids_fn = [&semantic_map_manager](
                                           const mapping::ElementId &start_id,
                                           const mapping::ElementId &end_id,
                                           int extend_times) {
    std::vector<mapping::ElementId> extend_lane_ids;
    // Current_lane_id, next_lane_id.
    VLOG(3) << "start_id:" << start_id << ",end_id:" << end_id;
    absl::flat_hash_map<mapping::ElementId, mapping::ElementId> from;
    std::queue<std::pair<mapping::ElementId, int>> que;
    que.push(std::make_pair(start_id, 1));
    while (!que.empty()) {
      const auto current_id = que.front();
      que.pop();
      if (current_id.second > extend_times) break;
      for (const auto &next_lane_idx :
           semantic_map_manager.FindLaneInfoOrDie(current_id.first)
               .outgoing_lane_indices) {
        const auto next_lane_id =
            semantic_map_manager.lane_info()[next_lane_idx].id;
        from[next_lane_id] = current_id.first;
        if (next_lane_id == end_id) {
          const auto *id = FindOrNull(from, end_id);
          while (id != nullptr) {
            extend_lane_ids.push_back(*id);
            id = FindOrNull(from, *id);
          }
          std::reverse(extend_lane_ids.begin(), extend_lane_ids.end());
          break;
        }
        que.push(std::make_pair(next_lane_id, current_id.second + 1));
      }
    }
    return extend_lane_ids;
  };

  for (int i = 0; i + 1 < original_lane_paths.size(); ++i) {
    const auto lc_neighbor_original = get_lc_neighbor(
        lane_paths[i].lane_ids().back(), transitions[i].lc_left);
    QCHECK(lc_neighbor_original.has_value())
        << "No lane neighbor found, plz check the corresponding lane info!";
    if (internal::CanLaneChange(*lc_neighbor_original)) {
      continue;
    }
    // While do not allow lc in solid lane boundary, we post process
    // lc point to broken white.
    bool post_process_success = false;
    // Store lane_ids after processing.
    std::vector<mapping::ElementId> processed_current_lane_ids;
    std::vector<mapping::ElementId> processed_target_lane_ids;

    for (int j = lane_paths[i].size() - 1; j >= 0; --j) {
      const auto lc_neighbor =
          get_lc_neighbor(lane_paths[i].lane_id(j), transitions[i].lc_left);
      if (lc_neighbor.has_value() && internal::CanLaneChange(*lc_neighbor)) {
        // 1. Process current LanePath.
        processed_current_lane_ids.insert(
            processed_current_lane_ids.begin(),
            lane_paths[i].lane_ids().begin(),
            lane_paths[i].lane_ids().begin() + j + 1);

        // 2. Process target LanePath.
        const auto target_lane_initial_id =
            lane_paths[i + 1].lane_ids().front();
        const auto processed_target_lane_start_id =
            get_lc_neighbor(processed_current_lane_ids.back(),
                            transitions[i].lc_left)
                ->other_id;
        // Backward extend lane ids for target lane path.
        processed_target_lane_ids = find_extend_lane_ids_fn(
            processed_target_lane_start_id, target_lane_initial_id,
            lane_paths[i].size() - 1 - j);
        if (!processed_target_lane_ids.empty()) {
          post_process_success = true;
        }
        // Connect initial target lane ids.
        processed_target_lane_ids.insert(processed_target_lane_ids.end(),
                                         lane_paths[i + 1].lane_ids().begin(),
                                         lane_paths[i + 1].lane_ids().end());
      }

      if (post_process_success) break;
    }

    if (post_process_success) {
      lane_paths[i] = CompositeLanePath::LanePath(
          &semantic_map_manager, std::move(processed_current_lane_ids),
          original_lane_paths[i].start_fraction(), 1.0);
      transitions[i] = CompositeLanePath::TransitionInfo{
          .overlap_length = 0.0,
          .lc_left = original_transitions[i].lc_left,
          .lc_section_id =
              semantic_map_manager
                  .FindLaneInfoOrDie(lane_paths[i].lane_ids().back())
                  .section_id};
      lane_paths[i + 1] = CompositeLanePath::LanePath(
          &semantic_map_manager, std::move(processed_target_lane_ids), 1.0,
          original_lane_paths[i + 1].end_fraction());
    } else if (empty_if_must_cross_solid_boundary) {
      return absl::NotFoundError("Must cross solid boundary");
    }
  }
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

absl::StatusOr<CompositeLanePath> BuildCompositeLanePathFromLanesSequence(
    const std::vector<RouteLaneIndex> &lanes_sequence,
    const std::vector<RouteSectionSequence::RouteSection> &route_sections,
    const SemanticMapManager &semantic_map_manager,
    const RouteLaneIndex &start_route_lane_idx, double start_fraction,
    double end_fraction, int destination_section_idx,
    bool empty_if_must_cross_solid_boundary) {
  std::vector<CompositeLanePath::LanePath> lane_paths;
  std::vector<CompositeLanePath::TransitionInfo> transitions;

  const auto get_lane_id = [&route_sections](RouteLaneIndex index) {
    return route_sections[index.route_section_index]
        .lanes[index.lane_index_in_section]
        .first;
  };

  // As a lc point is the end_fraction of last section, the start_fraction is
  // the end of last section except start lane.
  const auto lanepath_end_fraction = [&end_fraction, &destination_section_idx](
                                         const RouteLaneIndex &lane_idx) {
    return (lane_idx.route_section_index == destination_section_idx)
               ? end_fraction
               : 1.0;
  };

  std::vector<mapping::ElementId> lane_ids;
  lane_ids.push_back(get_lane_id(start_route_lane_idx));
  double lanepath_start_fraction = start_fraction;
  for (int i = 1; i < lanes_sequence.size(); ++i) {
    if (mapping::IsOutgoingLane(semantic_map_manager,
                                semantic_map_manager.FindLaneInfoOrDie(
                                    get_lane_id(lanes_sequence[i - 1])),
                                get_lane_id(lanes_sequence[i]))) {
      lane_ids.push_back(get_lane_id(lanes_sequence[i]));
    } else {
      lane_paths.emplace_back(&semantic_map_manager, lane_ids,
                              lanepath_start_fraction,
                              lanepath_end_fraction(lanes_sequence[i - 1]));

      lane_ids.clear();
      // Process LC.
      const bool lc_left = lanes_sequence[i - 1].lane_index_in_section >
                           lanes_sequence[i].lane_index_in_section;

      transitions.emplace_back(CompositeLanePath::TransitionInfo{
          .overlap_length = 0.0,
          .lc_left = lc_left,
          .lc_section_id =
              route_sections[lanes_sequence[i].route_section_index].id});
      lane_ids.push_back(get_lane_id(lanes_sequence[i]));
      lanepath_start_fraction =
          lanes_sequence[i].route_section_index == destination_section_idx
              ? end_fraction
              : route_sections[lanes_sequence[i].route_section_index]
                    .end_fraction;
    }
  }

  if (!lane_ids.empty()) {
    lane_paths.emplace_back(&semantic_map_manager, lane_ids,
                            lanepath_start_fraction,
                            lanepath_end_fraction(lanes_sequence.back()));
  }

  if (!FLAGS_route_allow_solid_line_lane_change) {
    return PostProcessForRouteLanePath(semantic_map_manager, lane_paths,
                                       transitions,
                                       empty_if_must_cross_solid_boundary);
  }

  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

absl::StatusOr<CompositeLanePath> DpFindRouteLanePath(
    const RouteLanePathFinderInput &input,
    const SemanticMapManager &semantic_map_manager) {
  const auto &route_sections = input.section_sequence->sections();

  std::vector<std::vector<RouteLaneInfo>> cost_table;

  // Find start section if exists.
  const auto start_lane_id = input.start_point.lane_id();
  int start_section_idx = 0;
  for (; start_section_idx < route_sections.size(); ++start_section_idx) {
    bool found = false;
    for (const auto &lane : route_sections[start_section_idx].lanes) {
      if (start_lane_id == lane.first) {
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }

  if (start_section_idx == route_sections.size()) {
    QLOG(ERROR) << "Cannot find start section.";
    return absl::UnknownError("start section cannot be found.");
  }

  // Find destination section if exists.
  const auto destination_lane_id = input.destination_point.lane_id();
  int destination_section_idx = route_sections.size() - 1;
  for (; destination_section_idx >= 0; --destination_section_idx) {
    bool found = false;
    for (const auto &lane : route_sections[destination_section_idx].lanes) {
      if (destination_lane_id == lane.first) {
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }

  if (destination_section_idx == -1) {
    QLOG(ERROR) << "Cannot find destination section.";
    return absl::UnknownError("destination section cannot be found.");
  }

  if (destination_section_idx < start_section_idx) {
    QLOG(ERROR) << "Destination is behind origin!";
    return absl::UnknownError("destination is behind origin");
  }

  // Find origin lane and destination lane.
  RouteLaneIndex start_route_lane_idx;
  start_route_lane_idx.route_section_index = start_section_idx;
  start_route_lane_idx.lane_index_in_section =
      FindOrDie(route_sections[start_section_idx].id_idx_map,
                input.start_point.lane_id());

  RouteLaneIndex destination_route_lane_idx;
  destination_route_lane_idx.route_section_index = destination_section_idx;
  destination_route_lane_idx.lane_index_in_section =
      FindOrDie(route_sections[destination_section_idx].id_idx_map,
                input.destination_point.lane_id());

  // When OD is too short for lane change, add peer to blacklist
  absl::flat_hash_set<mapping::ElementId> blacklist(input.lane_id_blacklist);
  auto origin_peer_blacks = internal::MinLaneChangeBlacklist(
      semantic_map_manager, input.start_point, true);
  blacklist.insert(origin_peer_blacks.begin(), origin_peer_blacks.end());
  auto destination_peer_blacks = internal::MinLaneChangeBlacklist(
      semantic_map_manager, input.destination_point, false);
  blacklist.insert(destination_peer_blacks.begin(),
                   destination_peer_blacks.end());
  VLOG(3) << "origin peer blacks(lane change):"
          << absl::StrJoin(origin_peer_blacks, ",")
          << ", destination peer blacks(lane change):"
          << absl::StrJoin(destination_peer_blacks, ",");

  // When last part of the lanepath is short (< 30m) and in blacklist,
  // we ignore the lanes.
  absl::flat_hash_set<mapping::ElementId> whitelist =
      BuildWhitelistFromBlacklist(input.lane_id_blacklist, semantic_map_manager,
                                  input.start_point, input.destination_point);

  // Build cost table.
  cost_table.resize(route_sections.size());
  for (int i = 0; i < route_sections.size(); ++i) {
    cost_table[i].resize(route_sections[i].lanes.size());
  }

  // Initialize start section.
  cost_table[start_section_idx][start_route_lane_idx.lane_index_in_section]
      .cum_cost = (ContainsKey(blacklist, input.start_point.lane_id()))
                      ? kBlacklistCost
                      : 0.0;

  std::queue<int> start_min_cost_index;
  start_min_cost_index.push(start_route_lane_idx.lane_index_in_section);
  UpdateSectionCost(start_section_idx, &start_min_cost_index,
                    &cost_table[start_section_idx]);

  // Update other sections.
  for (int i = start_section_idx + 1; i <= destination_section_idx; ++i) {
    const auto &prev_section_cost = cost_table[i - 1];

    // Update cost due to connection relationship
    // and find the minimum cost.
    double minimum_cost_in_current_section =
        std::numeric_limits<double>::infinity();
    for (int j = 0; j < prev_section_cost.size(); ++j) {
      RouteLaneIndex prev_lane_idx = {i - 1, j};
      for (const int &lane_idx_in_current_section :
           route_sections[i - 1].outgoing_lanes_index_table[j]) {
        RouteLaneIndex current_lane_idx = {i, lane_idx_in_current_section};
        cost_table[i][lane_idx_in_current_section] = ConnectionLaneCost(
            current_lane_idx, prev_lane_idx, input, whitelist,
            absl::MakeSpan(route_sections), cost_table, semantic_map_manager);
        minimum_cost_in_current_section =
            std::min(minimum_cost_in_current_section,
                     cost_table[i][lane_idx_in_current_section].cum_cost);
      }
    }

    // Find the minimum cost lanes idx in current section and
    // update other lanes via them.
    std::queue<int> min_cost_lanes_idx;
    for (int k = 0; k < cost_table[i].size(); ++k) {
      if (std::fabs(cost_table[i][k].cum_cost -
                    minimum_cost_in_current_section) < 1e-4) {
        min_cost_lanes_idx.push(k);
      }
    }
    UpdateSectionCost(i, &min_cost_lanes_idx, &cost_table[i]);
  }

  // Get lane sequence due to cost_table.
  std::vector<RouteLaneIndex> lanes_sequence;
  lanes_sequence.emplace_back(destination_route_lane_idx);

  const auto get_route_lane_cost =
      [&cost_table](RouteLaneIndex index) -> RouteLaneInfo & {
    return cost_table[index.route_section_index][index.lane_index_in_section];
  };

  while (get_route_lane_cost(lanes_sequence.back())
             .prev_lane_index.route_section_index != -1) {
    lanes_sequence.emplace_back(
        get_route_lane_cost(lanes_sequence.back()).prev_lane_index);
  }

  std::reverse(lanes_sequence.begin(), lanes_sequence.end());

  // Build CompositeLanePath.
  return BuildCompositeLanePathFromLanesSequence(
      lanes_sequence, route_sections, semantic_map_manager,
      start_route_lane_idx, input.start_point.fraction(),
      input.destination_point.fraction(), destination_section_idx,
      input.empty_if_must_cross_solid_boundary);
}

}  // namespace

absl::StatusOr<CompositeLanePath> FindRouteLanePathOnSectionSequence(
    const RouteLanePathFinderInput &input,
    const SemanticMapManager &semantic_map_manager) {
  return DpFindRouteLanePath(input, semantic_map_manager);
}

}  // namespace qcraft::planner
