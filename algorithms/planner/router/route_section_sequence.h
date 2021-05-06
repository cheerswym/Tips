#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTION_SEQUENCE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTION_SEQUENCE_H_

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {

// A RouteSectionSequence is a sequence of connected sections on route. It only
// indicates road connectivity instead of giving a path of lanes as
// CompositeLanePath does.

//                                            o
//                                            |
//                                            |  section_4
//                                            |
//   ------------->|------------>|------------^  (turn left)
//   ------------->|------------>|
//   ------------->|------------>|
//      section_1     section_2     section_3
//
//
//  Each section contains a group of aligned lanes on semantic map. If there is
//  at least one pair of connected lanes between two sections, we consider these
//  two sections are connected

class RouteSectionSequence {
 public:
  struct RouteSection {
    int64_t id;
    double start_fraction;
    double end_fraction;
    double max_driving_dis_on_route = 0.0;

    // <lane id, max_driving_distance_on_route>
    // The second indicates how far we can drive from this lane without lc on a
    // given route
    std::vector<std::pair<mapping::ElementId, double>> lanes;

    // Outgoing lanes index in next RouteSection
    std::vector<std::vector<int>> outgoing_lanes_index_table;

    // id: lane_id.   idx: lane index in lanes vector above.
    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;

    double avg_length;

    double length() const {
      return avg_length * (end_fraction - start_fraction);
    }

    bool contains(const mapping::LanePoint lane_pt) const {
      return id_idx_map.contains(lane_pt.lane_id()) &&
             start_fraction <= lane_pt.fraction() &&
             lane_pt.fraction() <= end_fraction;
    }

    std::string DebugString() const {
      return absl::StrFormat(
          "{section_id: %d, start_fraction: %.6f, end_fraction: %.6f}", id,
          start_fraction, end_fraction);
    }
  };

  RouteSectionSequence() = default;

  explicit RouteSectionSequence(
      absl::Span<const mapping::ElementId> section_ids,
      const SemanticMapManager &semantic_map_manager,
      const mapping::LanePoint &origin, const mapping::LanePoint &destination) {
    FromSectionIds(section_ids, semantic_map_manager, origin, destination);
  }

  explicit RouteSectionSequence(RouteSection section)
      : sections_({std::move(section)}) {}

  explicit RouteSectionSequence(std::vector<RouteSection> sections)
      : sections_(std::move(sections)) {}

  explicit RouteSectionSequence(
      const CompositeLanePath &route_lane_path,
      const SemanticMapManager *semantic_map_manager) {
    FromCompositeLanePath(*semantic_map_manager, route_lane_path);
  }

  bool empty() const { return sections_.empty(); }

  int size() const { return sections_.size(); }

  const std::vector<RouteSection> &sections() const { return sections_; }

  double length_between(int start_idx, int end_idx) const {
    CHECK_GE(start_idx, 0);
    CHECK_LE(end_idx, sections_.size());
    double len = 0.0;
    for (int i = start_idx; i < end_idx; ++i) len += sections_[i].length();
    return len;
  }

  double length() const { return length_between(0, sections_.size()); }

  std::string DebugString() const {
    return absl::StrJoin(sections_, "\n",
                         [](std::string *out, const RouteSection &section) {
                           absl::StrAppend(out, section.DebugString());
                         });
  }

  std::string ShortDebugString() const {
    return absl::StrJoin(sections_, ";",
                         [](std::string *out, const RouteSection &section) {
                           absl::StrAppend(out, section.DebugString());
                         });
  }

  bool IsPointOnSections(const SemanticMapManager &semantic_map_manager,
                         Vec2d query_point, double heading,
                         double lat_dist_thres = kMaxHalfLaneWidth,
                         double lon_dist_offset = 0.0,
                         double cosine_angle = /*cos(M_PI/4)*/ 0.70717) const;

  bool IsLanePointOnSections(const mapping::LanePoint &lane_point) const;

  bool ContainsSection(int64_t section_id) const;
  int FirstOccurrenceOfSectionToIndex(int64_t section_id) const;

  void ToProto(RouteSectionSequenceProto *section_proto) const;

 private:
  void FromCompositeLanePath(const SemanticMapManager &semantic_map_manager,
                             const CompositeLanePath &route_lane_path);

  void FromSectionIds(absl::Span<const mapping::ElementId> section_ids,
                      const SemanticMapManager &semantic_map_manager,
                      const mapping::LanePoint &origin,
                      const mapping::LanePoint &destination);

  void BuildOutgoingLanes(const SemanticMapManager &semantic_map_manager);
  void BuildIdIdxMap();

 private:
  std::vector<RouteSection> sections_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SECTION_SEQUENCE_H_
