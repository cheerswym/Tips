#include "onboard/planner/router/route_section_sequence.h"

#include <deque>
#include <unordered_set>

#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

void RouteSectionSequence::BuildOutgoingLanes(
    const SemanticMapManager &semantic_map_manager) {
  if (sections_.size() <= 1) return;
  for (int i = 0; i < sections_.size(); ++i) {
    const int lanes_size = sections_[i].lanes.size();
    sections_[i].outgoing_lanes_index_table.resize(lanes_size, {});
    if (i + 1 != sections_.size()) {
      for (int j = 0; j < lanes_size; ++j) {
        std::vector<int> outgoing_lanes;
        const auto &current_lane_info =
            semantic_map_manager.FindLaneInfoOrDie(sections_[i].lanes[j].first);
        for (int k = 0; k < sections_[i + 1].lanes.size(); ++k) {
          if (mapping::IsOutgoingLane(semantic_map_manager, current_lane_info,
                                      sections_[i + 1].lanes[k].first)) {
            outgoing_lanes.emplace_back(k);
          }
        }
        sections_[i].outgoing_lanes_index_table[j] = std::move(outgoing_lanes);
      }
    }
  }
}

void RouteSectionSequence::BuildIdIdxMap() {
  for (int i = 0; i < sections_.size(); ++i) {
    for (int j = 0; j < sections_[i].lanes.size(); ++j) {
      sections_[i].id_idx_map[sections_[i].lanes[j].first] = j;
    }
  }
}

void RouteSectionSequence::FromSectionIds(
    absl::Span<const mapping::ElementId> section_ids,
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePoint &origin, const mapping::LanePoint &destination) {
  sections_.clear();
  sections_.reserve(section_ids.size());

  for (int i = 0; i < section_ids.size(); ++i) {
    if (!sections_.empty() && sections_.back().id == section_ids[i]) continue;

    const auto &section_info =
        semantic_map_manager.FindSectionInfoOrDie(section_ids[i]);
    auto &section = sections_.emplace_back();
    section.id = section_info.id;
    section.start_fraction = 0.0;
    section.end_fraction = 1.0;

    section.lanes.reserve(section_info.lane_ids.size());
    double sum_len = .0;
    for (int i = 0; i < section_info.lane_ids.size(); ++i) {
      // Distance to be calculated later
      section.lanes.emplace_back(section_info.lane_ids[i], 0.0);
      sum_len +=
          semantic_map_manager.FindLaneInfoOrDie(section_info.lane_ids[i])
              .length();
    }

    section.avg_length = sum_len / section_info.lane_ids.size();
  }

  // Refine first and last section fraction.
  const auto origin_section_id =
      semantic_map_manager.FindLaneInfoOrDie(origin.lane_id()).section_id;
  for (auto &section : sections_) {
    if (section.id == origin_section_id) {
      section.start_fraction = origin.fraction();
      break;
    }
  }
  sections_.back().end_fraction = destination.fraction();

  for (auto it = sections_.rbegin() + 1; it != sections_.rend(); ++it) {
    const auto &next_section = *(it - 1);
    auto &this_section = *it;

    for (auto &lane : this_section.lanes) {
      const auto &lane_info =
          semantic_map_manager.FindLaneInfoOrDie(lane.first);

      for (const auto &next_lane : next_section.lanes) {
        if (mapping::IsOutgoingLane(semantic_map_manager, lane_info,
                                    next_lane.first)) {
          lane.second =
              std::max(lane.second, next_lane.second + next_section.length());
        }
      }
    }
  }

  // Store outgoing lanes in next RouteSection.
  BuildOutgoingLanes(semantic_map_manager);

  // Build id-idx map.
  BuildIdIdxMap();

  for (auto &lane : sections_.back().lanes) {
    if (lane.first == destination.lane_id()) {
      lane.second += 1.0;
    }
  }
}

void RouteSectionSequence::FromCompositeLanePath(
    const SemanticMapManager &semantic_map_manager,
    const CompositeLanePath &route_lane_path) {
  for (const auto &lane_path : route_lane_path.lane_paths()) {
    // Drop overlap sections
    int i = 0;
    bool have_overlap = false;
    if (!sections_.empty()) {
      for (; i < lane_path.size(); i++) {
        const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(
            lane_path.lane_segment(i).lane_id);
        if (lane_info.section_id == sections_.back().id) {
          i++;
          have_overlap = true;
          break;
        }
      }
    }

    if (!have_overlap) {
      i = 0;
    }

    for (; i < lane_path.size(); i++) {
      const auto &lane_seg = lane_path.lane_segment(i);
      const auto &lane_info =
          semantic_map_manager.FindLaneInfoOrDie(lane_seg.lane_id);
      const auto &section_info =
          semantic_map_manager.FindSectionInfoOrDie(lane_info.section_id);

      if (!sections_.empty() && sections_.back().id == section_info.id)
        continue;

      sections_.emplace_back();
      auto &section = sections_.back();
      section.id = section_info.id;
      section.start_fraction = 0.0;
      section.end_fraction = 1.0;

      section.lanes.reserve(section_info.lane_ids.size());
      double sum_len = .0;
      for (int i = 0; i < section_info.lane_ids.size(); ++i) {
        // Distance to be calculated later
        section.lanes.emplace_back(section_info.lane_ids[i], 0.0);
        sum_len +=
            semantic_map_manager.FindLaneInfoOrDie(section_info.lane_ids[i])
                .length();
      }

      section.avg_length = sum_len / section_info.lane_ids.size();
    }
  }

  // Refine first and last section fraction
  sections_.front().start_fraction = route_lane_path.front().fraction();
  sections_.back().end_fraction = route_lane_path.back().fraction();

  for (auto it = sections_.rbegin() + 1; it != sections_.rend(); ++it) {
    const auto &next_section = *(it - 1);
    auto &this_section = *it;

    for (auto &lane : this_section.lanes) {
      const auto &lane_info =
          semantic_map_manager.FindLaneInfoOrDie(lane.first);

      for (const auto &next_lane : next_section.lanes) {
        if (mapping::IsOutgoingLane(semantic_map_manager, lane_info,
                                    next_lane.first)) {
          lane.second =
              std::max(lane.second, next_lane.second + next_section.length());
        }
      }
    }
  }

  // Store outgoing lanes in next RouteSection.
  BuildOutgoingLanes(semantic_map_manager);

  // Build id-idx map.
  BuildIdIdxMap();

  // Give destination lane a bonus
  for (auto &lane : sections_.back().lanes) {
    if (lane.first == route_lane_path.back().lane_id()) {
      lane.second += 1.0;
    }
  }
}

bool RouteSectionSequence::IsPointOnSections(
    const SemanticMapManager &semantic_map_manager, Vec2d query_point,
    double heading, double lat_dist_thres, double lon_dist_offset,
    double cosine_angle) const {
  // BANDAID(boqian): to deal with backward pose jumping on reset.
  constexpr double kResetTolerance = 8.0;
  for (const auto &sec : sections_) {
    const auto &map_section = semantic_map_manager.FindSectionInfoOrDie(sec.id);
    for (const auto lane_id : map_section.lane_ids) {
      if (sec.start_fraction == sec.end_fraction) continue;
      const auto points = mapping::ResampleLanePoints(
          semantic_map_manager.FindLaneInfoOrDie(lane_id), sec.start_fraction,
          sec.end_fraction);

      ASSIGN_OR_CONTINUE(const auto ff, BuildBruteForceFrenetFrame(points));
      const FrenetCoordinate sl = ff.XYToSL(query_point);
      const double lon_offset = std::min(ff.length(), lon_dist_offset);
      const double lane_theta = ff.InterpolateTangentByS(sl.s).FastAngle();
      if (std::cos(lane_theta - heading) < cosine_angle) {
        continue;
      }

      if (std::abs(sl.l) < lat_dist_thres && sl.s < ff.end_s() - lon_offset &&
          sl.s > ff.start_s() - lon_offset - kResetTolerance) {
        return true;
      }
    }
  }
  return false;
}

bool RouteSectionSequence::IsLanePointOnSections(
    const mapping::LanePoint &lane_point) const {
  constexpr double kDistanceErrorThreshold = 1.0;  // m.
  for (const auto &section : sections_) {
    for (const auto &[id, max_dd] : section.lanes) {
      const double start_fraction =
          std::max(0.0, section.start_fraction -
                            kDistanceErrorThreshold / section.length());
      if (lane_point.lane_id() == id &&
          lane_point.fraction() >= start_fraction &&
          lane_point.fraction() <= section.end_fraction) {
        return true;
      }
    }
  }
  return false;
}

int RouteSectionSequence::FirstOccurrenceOfSectionToIndex(
    int64_t section_id) const {
  for (int i = 0; i < sections_.size(); ++i) {
    if (sections_[i].id == section_id) return i;
  }
  QLOG(FATAL) << "Can not find section " << section_id << "on "
              << ShortDebugString();
  return -1;
}

bool RouteSectionSequence::ContainsSection(int64_t section_id) const {
  for (const auto &sec : sections_) {
    if (sec.id == section_id) return true;
  }
  return false;
}

void RouteSectionSequence::ToProto(
    RouteSectionSequenceProto *section_proto) const {
  section_proto->set_start_fraction(sections_.front().start_fraction);
  section_proto->set_end_fraction(sections_.back().end_fraction);

  section_proto->mutable_section_id()->Reserve(sections_.size());
  for (const auto &route_section : sections_) {
    section_proto->add_section_id(route_section.id);
  }
}

}  // namespace planner
}  // namespace qcraft
