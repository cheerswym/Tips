#include "onboard/planner/router/route_sections_info.h"

#include <utility>

#include "onboard/global/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

RouteSectionsInfo::RouteSectionsInfo(
    const PlannerSemanticMapManager &psmm, const RouteSections *sections,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes) {
  route_sections_ = QCHECK_NOTNULL(sections);
  if (sections->empty()) return;

  section_segments_.reserve(route_sections_->size());
  for (int i = 0; i < route_sections_->size(); ++i) {
    const auto &section_info =
        psmm.FindSectionInfoOrDie(route_sections_->section_ids()[i]);
    section_segments_.emplace_back(RouteSectionSegmentInfo{
        .id = route_sections_->section_ids()[i],
        .start_fraction = 0.0,
        .end_fraction = 1.0,
        .average_length = section_info.average_length,
        .lane_ids = section_info.lane_ids,
        .driving_distance =
            std::vector<double>(section_info.lane_ids.size(), 0.0)});
  }

  // Refine first and last section fraction.
  section_segments_.front().start_fraction = route_sections_->start_fraction();
  section_segments_.back().end_fraction = route_sections_->end_fraction();

  // Build id_idx_map.
  for (auto &seg : section_segments_) {
    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
    for (int i = 0; i < seg.lane_ids.size(); ++i) {
      id_idx_map.insert({seg.lane_ids[i], i});
    }
    seg.id_idx_map = std::move(id_idx_map);
  }

  // Compute driving distance from last segment to first segment.
  constexpr double kDestinationBonus = 10.0;  // m.
  auto &last_sec = section_segments_.back();
  for (int i = 0; i < last_sec.lane_ids.size(); ++i) {
    last_sec.driving_distance[i] = last_sec.length();
    if (last_sec.lane_ids[i] == route_sections_->destination().lane_id()) {
      last_sec.driving_distance[i] += kDestinationBonus;
    }
  }
  for (auto it = section_segments_.rbegin() + 1; it != section_segments_.rend();
       ++it) {
    const auto &next_segment = *(it - 1);
    auto &this_segment = *it;

    for (int i = 0; i < this_segment.lane_ids.size(); ++i) {
      const auto &lane_info = psmm.FindLaneInfoOrDie(this_segment.lane_ids[i]);
      if (avoid_lanes.contains(this_segment.lane_ids[i]) ||
          lane_info.IsPassengerVehicleAvoidLaneType()) {
        this_segment.driving_distance[i] = 0.0;
        continue;
      }

      this_segment.driving_distance[i] = this_segment.length();
      for (int j = 0; j < next_segment.lane_ids.size(); ++j) {
        if (mapping::IsOutgoingLane(*psmm.semantic_map_manager(), lane_info,
                                    next_segment.lane_ids[j])) {
          this_segment.driving_distance[i] = std::max(
              this_segment.driving_distance[i],
              next_segment.driving_distance[j] + this_segment.length());
        }
      }
    }
  }
  planning_horizon_ = sections->planning_horizon(psmm);
}

}  // namespace qcraft::planner
