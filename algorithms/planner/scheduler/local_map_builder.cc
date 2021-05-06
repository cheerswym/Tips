#include "onboard/planner/scheduler/local_map_builder.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/planner/router/route_sections_info.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

// The max length of lane path is consistent with the effective range of
// perception temporarily.
constexpr double kDefaultLanePathLength = 150.0;

std::vector<mapping::ElementId> FindRoutingTargetLaneIdsOnRouteSection(
    const RouteSectionsInfo::RouteSectionSegmentInfo &section) {
  double max_dist = std::numeric_limits<double>::lowest();

  for (const auto &dist : section.driving_distance) {
    max_dist = std::max(max_dist, dist);
  }

  std::vector<mapping::ElementId> ids;
  for (int i = 0, n = section.driving_distance.size(); i < n; ++i) {
    if (section.driving_distance[i] == max_dist) {
      ids.push_back(section.lane_ids[i]);
    }
  }
  return ids;
}

// TODO(weijun): Add unit test.
absl::StatusOr<mapping::ElementId> SelectBestLaneOnRouteSection(
    const SemanticMapManager &smm,
    const RouteSectionsInfo::RouteSectionSegmentInfo &section,
    const std::vector<mapping::ElementId> &candidate_lanes) {
  std::vector<mapping::ElementId> filtered_candidate_lanes;
  for (const auto &id : candidate_lanes) {
    if (std::find_if(section.lane_ids.begin(), section.lane_ids.end(),
                     [&id](mapping::ElementId key) { return id == key; }) !=
        section.lane_ids.end()) {
      filtered_candidate_lanes.emplace_back(id);
    }
  }

  if (filtered_candidate_lanes.empty()) return absl::NotFoundError("Not found");

  const auto targets = FindRoutingTargetLaneIdsOnRouteSection(section);

  const auto &id_idx_map = section.id_idx_map;

  const auto index_gap_to_targets = [&targets,
                                     &id_idx_map](mapping::ElementId id) {
    const int index = FindOrDie(id_idx_map, id);
    int gap = std::numeric_limits<int>::max();
    for (const auto &target_id : targets) {
      const int target_index = FindOrDie(id_idx_map, target_id);
      gap = std::min(gap, std::abs(index - target_index));
    }
    return gap;
  };

  // TODO(weijun): Maybe we can compute distance and gap in advance to
  // accelerate computation.
  const auto it = std::min_element(
      filtered_candidate_lanes.begin(), filtered_candidate_lanes.end(),
      [&smm, &id_idx_map, &section, &index_gap_to_targets](
          mapping::ElementId lhs, mapping::ElementId rhs) {
        const double lhs_distance =
            section.driving_distance[FindOrDie(id_idx_map, lhs)];
        const double rhs_distance =
            section.driving_distance[FindOrDie(id_idx_map, rhs)];

        if (lhs_distance > rhs_distance) return true;
        if (rhs_distance > lhs_distance) return false;

        const int lhs_gap = index_gap_to_targets(lhs);
        const int rhs_gap = index_gap_to_targets(rhs);

        if (lhs_gap < rhs_gap) return true;
        if (rhs_gap < lhs_gap) return false;

        return smm.FindLaneInfoOrDie(lhs).length() <
               smm.FindLaneInfoOrDie(rhs).length();
      });

  return *it;
}

// Always select the longest driving distance next lane to extend.
mapping::LanePath BuildLanePathAlongRoute(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePoint &start_point,
    const RouteSectionsInfo &sections_info) {
  const auto &current_section_info = sections_info.front();
  auto lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), {start_point.lane_id()},
      start_point.fraction(), current_section_info.end_fraction);

  for (int i = 1, n = sections_info.size(); i < n; ++i) {
    std::vector<mapping::ElementId> candidate_lanes;
    const auto &back_lane_info =
        psmm.FindLaneInfoOrDie(lane_path.back().lane_id());
    for (const auto idx : back_lane_info.outgoing_lane_indices) {
      candidate_lanes.push_back(psmm.lane_info()[idx].id);
    }

    const auto next_section = sections_info.section_segment(i);
    const auto best_next_lane_id_or = SelectBestLaneOnRouteSection(
        *psmm.semantic_map_manager(), next_section, candidate_lanes);

    if (!best_next_lane_id_or.ok()) return lane_path;

    const auto best_next_lane = mapping::LanePath(
        psmm.semantic_map_manager(), {*best_next_lane_id_or},
        next_section.start_fraction, next_section.end_fraction);

    lane_path = lane_path.Connect(best_next_lane);

    if (lane_path.length() >= kDefaultLanePathLength) {
      return lane_path.BeforeArclength(kDefaultLanePathLength);
    }
  }

  return lane_path.BeforeArclength(kDefaultLanePathLength);
}

}  // namespace

absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager &psmm, const RouteSections &route_sections,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes) {
  const RouteSectionsInfo route_sections_info(psmm, &route_sections,
                                              avoid_lanes);
  if (route_sections_info.empty() ||
      route_sections_info.front().lane_ids.empty()) {
    return absl::NotFoundError("Route section is empty.");
  }

  std::vector<mapping::LanePath> lane_paths;
  const auto &current_section_info = route_sections_info.front();
  lane_paths.reserve(current_section_info.lane_ids.size());

  // Build lane paths start from left most lane to right most lane in current
  // route section.
  for (const auto lane_id : current_section_info.lane_ids) {
    lane_paths.emplace_back(BuildLanePathAlongRoute(
        psmm, mapping::LanePoint(lane_id, current_section_info.start_fraction),
        route_sections_info));
  }
  return lane_paths;
}
}  // namespace qcraft::planner
