#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner {

class RouteSectionsInfo {
 public:
  explicit RouteSectionsInfo(
      const PlannerSemanticMapManager &psmm, const RouteSections *sections,
      const absl::flat_hash_set<mapping::ElementId> &avoid_lanes);

  struct RouteSectionSegmentInfo {
    mapping::ElementId id;
    double start_fraction;
    double end_fraction;
    double average_length;
    std::vector<mapping::ElementId> lane_ids;
    std::vector<double> driving_distance;  // The same size with lane_ids

    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;

    double length() const {
      return average_length * (end_fraction - start_fraction);
    }
    bool contains(const mapping::LanePoint &lane_pt) const {
      return id_idx_map.contains(lane_pt.lane_id()) &&
             start_fraction <= lane_pt.fraction() &&
             lane_pt.fraction() <= end_fraction;
    }
  };

  bool empty() const { return section_segments_.empty(); }
  int size() const { return section_segments_.size(); }
  mapping::LanePoint destination() const {
    return route_sections_->destination();
  }
  const RouteSectionSegmentInfo &front() const {
    return section_segments_.front();
  }
  const RouteSectionSegmentInfo &back() const {
    return section_segments_.back();
  }
  double planning_horizon() const { return planning_horizon_; }

  double length_between(int start_idx, int end_idx) const {
    CHECK_GE(start_idx, 0);
    CHECK_LE(end_idx, section_segments_.size());
    double len = 0.0;
    for (int i = start_idx; i < end_idx; ++i) {
      len += section_segments_[i].length();
    }
    return len;
  }
  double length() const { return length_between(0, section_segments_.size()); }

  const RouteSections *route_sections() const { return route_sections_; }

  absl::Span<const RouteSectionSegmentInfo> section_segments() const {
    return section_segments_;
  }
  const RouteSectionSegmentInfo &section_segment(int index) const {
    return section_segments_[index];
  }

  const RouteSectionSegmentInfo *FindSegmentContainingLanePointOrNull(
      const mapping::LanePoint &lane_pt) const {
    for (const auto &section : section_segments_) {
      if (section.id_idx_map.contains(lane_pt.lane_id())) {
        return section.start_fraction <= lane_pt.fraction() &&
                       lane_pt.fraction() <= section.end_fraction
                   ? &section
                   : nullptr;
      }
    }
    return nullptr;
  }

 private:
  const RouteSections *route_sections_;
  std::vector<RouteSectionSegmentInfo> section_segments_;
  double planning_horizon_ = 0.0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_INFO_H_
