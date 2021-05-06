#include "onboard/planner/router/route_sections_util.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

absl::StatusOr<RouteSections> AlignRouteSections(
    const RouteSections &global_sections, const RouteSections &local_sections) {
  if (local_sections.empty() || global_sections.empty()) {
    return absl::InvalidArgumentError("empty section is not allowed.");
  }

  const auto local_first_index_or =
      global_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = local_sections.front().id,
          .start_fraction = local_sections.front().start_fraction,
          .end_fraction = local_sections.front().start_fraction});

  if (local_first_index_or.ok()) {
    return RouteSections(
        local_sections.start_fraction(), global_sections.end_fraction(),
        std::vector<mapping::ElementId>(global_sections.section_ids().begin() +
                                            local_first_index_or.value(),
                                        global_sections.section_ids().end()),
        global_sections.destination());
  }

  const auto global_first_index_or =
      local_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = global_sections.front().id,
          .start_fraction = global_sections.front().start_fraction,
          .end_fraction = global_sections.front().start_fraction});

  if (!global_first_index_or.ok()) {
    return absl::NotFoundError(absl::StrCat(
        "No overlap between sections:", "global:",
        global_sections.DebugString(), "local:", local_sections.DebugString()));
  }

  // Backward extend global sections along local sections.
  std::vector<mapping::ElementId> new_section_ids(
      local_sections.section_ids().begin(),
      local_sections.section_ids().begin() + global_first_index_or.value());

  for (int i = 0; i < global_sections.size(); ++i) {
    new_section_ids.push_back(global_sections.section_ids()[i]);
  }

  return RouteSections(
      local_sections.start_fraction(), global_sections.end_fraction(),
      std::move(new_section_ids), global_sections.destination());
}

RouteSections ConvertRouteSectionSequenceToRouteSections(
    const RouteSectionSequence &route_section_sequence) {
  std::vector<mapping::ElementId> section_ids;
  section_ids.reserve(route_section_sequence.size());
  for (const auto &section : route_section_sequence.sections()) {
    section_ids.push_back(section.id);
  }

  mapping::LanePoint destination;
  for (const auto &[id, distance] :
       route_section_sequence.sections().back().lanes) {
    if (distance > 0.0) {
      destination = mapping::LanePoint(
          id, route_section_sequence.sections().back().end_fraction);
    }
  }

  return RouteSections(route_section_sequence.sections().front().start_fraction,
                       route_section_sequence.sections().back().end_fraction,
                       std::move(section_ids), destination);
}

RouteSections RouteSectionsFromCompositeLanePath(const SemanticMapManager &smm,
                                                 const CompositeLanePath &clp) {
  std::vector<mapping::ElementId> all_sec_ids;
  for (const auto &lane_path : clp.lane_paths()) {
    for (const auto lane_id : lane_path.lane_ids()) {
      const auto &lane_info = smm.FindLaneInfoOrDie(lane_id);
      const auto sec_id = lane_info.section_id;
      if (!all_sec_ids.empty() && sec_id == all_sec_ids.back()) continue;

      all_sec_ids.push_back(sec_id);
    }
  }
  return RouteSections(clp.front().fraction(), clp.back().fraction(),
                       all_sec_ids, clp.back());
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByLateralOffset(
    const PlannerSemanticMapManager &psmm,
    const RouteSectionsInfo &sections_info, Vec2d query_point,
    double lat_dist_thres) {
  // To deal with numerical error in frenet projection.
  constexpr double kEpsilon = 0.1;  // m.

  const auto &sections = sections_info.section_segments();
  double accum_s = 0.0;
  for (int sec_idx = 0; sec_idx < sections.size(); ++sec_idx) {
    const auto &sec = sections[sec_idx];
    if (sec.start_fraction == sec.end_fraction) continue;

    for (const auto lane_id : sec.lane_ids) {
      const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
      const auto points = mapping::ResampleLanePoints(
          lane_info, sec.start_fraction, sec.end_fraction);

      ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(points));
      const FrenetCoordinate sl = ff.XYToSL(query_point);
      if (ff.start_s() - kEpsilon < sl.s && sl.s < ff.end_s() &&
          std::abs(sl.l) < lat_dist_thres) {
        const double frac = std::max(sl.s / lane_info.length(), 0.0);
        return PointOnRouteSections{
            .accum_s = accum_s + sec.average_length * frac,
            .section_idx = sec_idx,
            .fraction = std::clamp(frac + sec.start_fraction, 0.0, 1.0),
            .lane_id = lane_id};
      }
    }
    accum_s += sec.length();
  }
  return absl::NotFoundError(absl::StrCat(
      "FindSmoothPointOnRouteSectionsByLateralOffset:Point(", query_point.x(),
      ",", query_point.y(), ")is not on route sections:",
      sections_info.route_sections()->DebugString()));
}

absl::StatusOr<mapping::LanePath>
FindClosestLanePathOnRouteSectionsToSmoothPoint(
    const PlannerSemanticMapManager &psmm, const RouteSections &sections,
    Vec2d query_point, double *proj_s) {
  ASSIGN_OR_RETURN(const auto lane_paths,
                   CollectAllLanePathOnRouteSections(psmm, sections));

  double ego_s, min_lat_error = std::numeric_limits<double>::infinity();
  const mapping::LanePath *nearest_lane_path = nullptr;
  for (const auto &lane_path : lane_paths) {
    const auto points =
        mapping::SampleLanePathPoints(*psmm.semantic_map_manager(), lane_path);
    ASSIGN_OR_CONTINUE(const auto ff, BuildBruteForceFrenetFrame(points));

    const FrenetCoordinate sl = ff.XYToSL(query_point);
    constexpr double kEpsilon = 0.1;  // m.

    if (ff.start_s() - kEpsilon < sl.s && sl.s < ff.end_s()) {
      if (std::abs(sl.l) < min_lat_error) {
        min_lat_error = std::abs(sl.l);
        ego_s = sl.s;

        nearest_lane_path = &lane_path;
      }
    }
  }
  if (nearest_lane_path == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("FindClosestLanePathOnRouteSectionsToSmoothPoint:Point (",
                     query_point.x(), ",", query_point.y(),
                     ") is not on route sections:", sections.DebugString()));
  }

  if (proj_s != nullptr) *proj_s = ego_s;
  return *nearest_lane_path;
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByDrivePassage(
    const PlannerSemanticMapManager &psmm, const RouteSections &sections,
    Vec2d query_point) {
  ASSIGN_OR_RETURN(const auto nearest_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, sections, query_point));

  const double step_s = std::min(1.0, 0.5 * nearest_lane_path.length());
  const auto drive_passage = BuildDrivePassageFromLanePath(
      *psmm.semantic_map_manager(), nearest_lane_path, step_s,
      /*avoid_loop=*/false);

  ASSIGN_OR_RETURN(const auto sl,
                   drive_passage.QueryFrenetCoordinateAt(query_point),
                   _ << "Fail to project ego box on drive passage.");

  const auto start_lane_point = nearest_lane_path.AfterArclength(sl.s).front();
  for (int i = 0; i < sections.size(); ++i) {
    const auto &section_info =
        psmm.FindSectionInfoOrDie(sections.section_ids()[i]);

    if (std::find(section_info.lane_ids.begin(), section_info.lane_ids.end(),
                  start_lane_point.lane_id()) != section_info.lane_ids.end()) {
      return PointOnRouteSections{.accum_s = sl.s,
                                  .section_idx = i,
                                  .fraction = start_lane_point.fraction(),
                                  .lane_id = start_lane_point.lane_id()};
    }
  }

  return absl::NotFoundError(
      absl::StrCat("FindSmoothPointOnRouteSectionsByDrivePassage:Point (",
                   query_point.x(), ",", query_point.y(),
                   ") is not on route sections:", sections.DebugString()));
}

absl::StatusOr<RouteSections> ClampRouteSectionsBeforeArcLength(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &raw_route_sections, double len) {
  if (len < 0.0) {
    return absl::NotFoundError("Arc length is negative");
  }

  std::vector<mapping::ElementId> section_ids;
  double accum_s = 0.0;
  for (int i = 0; i < raw_route_sections.size(); ++i) {
    const auto &section_seg = raw_route_sections.route_section_segment(i);
    const auto &section_info = psmm.FindSectionInfoOrDie(section_seg.id);

    const double this_len =
        section_info.average_length *
        (section_seg.end_fraction - section_seg.start_fraction);

    section_ids.push_back(section_seg.id);
    if (accum_s + this_len > len) {
      return RouteSections(
          raw_route_sections.start_fraction(),
          std::clamp(section_seg.end_fraction - (accum_s + this_len - len) /
                                                    section_info.average_length,
                     0.0, 1.0),
          std::move(section_ids), raw_route_sections.destination());
    }

    accum_s += this_len;
  }

  return RouteSections(
      raw_route_sections.start_fraction(), raw_route_sections.end_fraction(),
      std::move(section_ids), raw_route_sections.destination());
}

absl::StatusOr<RouteSections> ClampRouteSectionsAfterArcLength(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &raw_route_sections, double len) {
  double accum_s = 0.0;
  int i = 0;
  double start_fraction = raw_route_sections.start_fraction();
  for (; i < raw_route_sections.size(); ++i) {
    const auto &section_seg = raw_route_sections.route_section_segment(i);
    const auto &section_info = psmm.FindSectionInfoOrDie(section_seg.id);
    const double this_len =
        section_info.average_length *
        (section_seg.end_fraction - section_seg.start_fraction);

    if (accum_s + this_len > len) {
      start_fraction =
          std::clamp(section_seg.end_fraction - (accum_s + this_len - len) /
                                                    section_info.average_length,
                     0.0, 1.0);
      break;
    }
    accum_s += this_len;
  }

  if (i == raw_route_sections.size()) {
    return absl::NotFoundError("Arc length is larger than sections length");
  }

  return RouteSections(start_fraction, raw_route_sections.end_fraction(),
                       std::vector<mapping::ElementId>(
                           raw_route_sections.section_ids().begin() + i,
                           raw_route_sections.section_ids().end()),
                       raw_route_sections.destination());
}

absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager &psmm,
                            const RouteSections &route_sections, Vec2d pos,
                            double projection_range,
                            double keep_behind_length) {
  SCOPED_QTRACE("ProjectPointToRouteSections");

  ASSIGN_OR_RETURN(const auto project_route_sections,
                   ClampRouteSectionsBeforeArcLength(psmm, route_sections,
                                                     projection_range));

  ASSIGN_OR_RETURN(auto point_proj,
                   FindSmoothPointOnRouteSectionsByDrivePassage(
                       psmm, project_route_sections, pos));

  std::vector<mapping::ElementId> sec_ids;
  for (int i = point_proj.section_idx; i < project_route_sections.size(); ++i) {
    sec_ids.push_back(project_route_sections.route_section_segment(i).id);
  }
  const RouteSections projected_route_sections_from_start(
      point_proj.fraction, project_route_sections.end_fraction(),
      std::move(sec_ids), project_route_sections.destination());

  ASSIGN_OR_RETURN(
      auto sections_from_start,
      AlignRouteSections(route_sections, projected_route_sections_from_start));

  RouteSections sections_with_behind;

  if (point_proj.accum_s > keep_behind_length) {
    ASSIGN_OR_RETURN(
        sections_with_behind,
        ClampRouteSectionsAfterArcLength(
            psmm, route_sections, point_proj.accum_s - keep_behind_length));
  } else {
    sections_with_behind = route_sections;
  }

  return std::make_tuple(std::move(sections_from_start),
                         std::move(sections_with_behind),
                         std::move(point_proj));
}

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSections(const PlannerSemanticMapManager &psmm,
                                  const RouteSections &route_sections) {
  std::vector<mapping::LanePath> final_results;
  std::vector<mapping::LanePath> lane_path_vec;

  const auto &first_section_info =
      psmm.FindSectionInfoOrDie(route_sections.front().id);
  for (const auto lane_id : first_section_info.lane_ids) {
    lane_path_vec.emplace_back(
        mapping::LanePath(psmm.semantic_map_manager(), {lane_id},
                          route_sections.front().start_fraction,
                          route_sections.front().end_fraction));
  }

  for (int i = 1; i < route_sections.size(); ++i) {
    const auto &route_section_seg = route_sections.route_section_segment(i);
    const auto &section_info = psmm.FindSectionInfoOrDie(route_section_seg.id);
    std::vector<mapping::LanePath> new_lane_path_vec;

    for (const auto &lane_path : lane_path_vec) {
      bool has_outgoing = false;
      const auto &last_lane_info =
          psmm.FindLaneInfoOrDie(lane_path.back().lane_id());
      for (const auto lane_id : section_info.lane_ids) {
        if (mapping::IsOutgoingLane(*psmm.semantic_map_manager(),
                                    last_lane_info, lane_id)) {
          has_outgoing = true;
          std::vector<mapping::ElementId> lane_ids = lane_path.lane_ids();
          lane_ids.push_back(lane_id);
          new_lane_path_vec.emplace_back(
              psmm.semantic_map_manager(), std::move(lane_ids),
              lane_path.front().fraction(), route_section_seg.end_fraction);
        }
      }

      if (!has_outgoing) {
        final_results.push_back(lane_path);
      }
    }

    lane_path_vec = std::move(new_lane_path_vec);
  }

  for (auto &lane_path : lane_path_vec) {
    final_results.emplace_back(std::move(lane_path));
  }

  return final_results;
}

RouteSections BackwardExtendRouteSections(const PlannerSemanticMapManager &psmm,
                                          const RouteSections &raw_sections,
                                          double extend_len) {
  if (extend_len <= 0.0) {
    return raw_sections;
  }

  std::vector<mapping::ElementId> extend_section_ids;
  double start_fraction = raw_sections.start_fraction();
  mapping::ElementId section_id = raw_sections.front().id;
  while (extend_len > 0.0) {
    const auto &section_info = psmm.FindSectionInfoOrDie(section_id);

    if (start_fraction == 0.0) {
      if (section_info.incoming_section_ids.empty()) {
        break;
      }
      const auto prev_id = section_info.incoming_section_ids.front();
      const auto &prev_section_info = psmm.FindSectionInfoOrDie(prev_id);
      extend_section_ids.push_back(prev_id);
      if (extend_len <= prev_section_info.average_length) {
        start_fraction = 1.0 - extend_len / prev_section_info.average_length;
        break;
      }

      extend_len -= prev_section_info.average_length;
      section_id = prev_id;
      continue;
    }

    const double rest_len = section_info.average_length * start_fraction;
    if (rest_len > extend_len) {
      start_fraction = std::max(
          0.0, start_fraction - extend_len / section_info.average_length);
      break;
    } else {
      start_fraction = 0.0;
      extend_len -= rest_len;
    }
  }

  std::reverse(extend_section_ids.begin(), extend_section_ids.end());
  extend_section_ids.insert(extend_section_ids.end(),
                            raw_sections.section_ids().begin(),
                            raw_sections.section_ids().end());

  return RouteSections(start_fraction, raw_sections.end_fraction(),
                       std::move(extend_section_ids),
                       raw_sections.destination());
}

absl::StatusOr<RouteSections> BackwardExtendRouteSectionsFromPos(
    const PlannerSemanticMapManager &psmm, const RouteSections &raw_sections,
    Vec2d pos, double extend_len) {
  ASSIGN_OR_RETURN(const auto proj_sections,
                   ClampRouteSectionsBeforeArcLength(
                       psmm, raw_sections, kMaxTravelDistanceBetweenFrames));

  ASSIGN_OR_RETURN(const auto lane_paths,
                   CollectAllLanePathOnRouteSections(psmm, proj_sections));
  double min_lat_error = std::numeric_limits<double>::infinity();
  const mapping::LanePath *nearest_lane_path = nullptr;
  for (const auto &lane_path : lane_paths) {
    const auto points =
        mapping::SampleLanePathPoints(*psmm.semantic_map_manager(), lane_path);
    ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(points));

    const FrenetCoordinate sl = ff.XYToSL(pos);
    constexpr double kEpsilon = 5.0;  // m.
    if (ff.start_s() - kEpsilon < sl.s && sl.s < ff.end_s()) {
      if (std::abs(sl.l) < min_lat_error) {
        min_lat_error = std::abs(sl.l);
        nearest_lane_path = &lane_path;
      }
    }
  }
  if (nearest_lane_path == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "BackwardExtendRouteSectionsFromPos:Point (", pos.x(), ",", pos.y(),
        ") is not on route sections:", proj_sections.DebugString()));
  }

  const auto extend_lp = BackwardExtendLanePath(*psmm.semantic_map_manager(),
                                                *nearest_lane_path, extend_len);

  const auto extended_sections =
      RouteSections::BuildFromLanePath(psmm, extend_lp);
  return AlignRouteSections(raw_sections, extended_sections);
}

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager &psmm, const RouteSections &route_sections,
    const mapping::LanePath &raw_lane_path, double extend_len) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  const auto &start_lp = raw_lane_path.back();
  // No avoid lane since we don't use driving distance here.
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});
  const auto &sections = sections_info.section_segments();
  // Find start section if exists.
  int sec_idx = 0;
  for (; sec_idx < sections.size(); ++sec_idx) {
    if (sections[sec_idx].contains(start_lp)) {
      break;
    }
  }
  if (sec_idx == sections.size()) {
    return absl::NotFoundError("Start section not found.");
  }

  std::vector<mapping::ElementId> lane_ids(raw_lane_path.lane_ids());
  auto cur_lane_id = start_lp.lane_id();
  double end_fraction = start_lp.fraction();
  while (extend_len > 0.0) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(cur_lane_id);
    if (end_fraction == 1.0) {
      if (++sec_idx == sections.size()) break;

      const auto &id_map = sections[sec_idx].id_idx_map;
      cur_lane_id = mapping::kInvalidElementId;
      for (const auto lane_idx : lane_info.outgoing_lane_indices) {
        const auto lane_id = psmm.lane_info()[lane_idx].id;
        if (id_map.contains(lane_id)) {
          cur_lane_id = lane_id;
          break;
        }
      }
      if (cur_lane_id == mapping::kInvalidElementId) break;

      lane_ids.push_back(cur_lane_id);
      end_fraction = 0.0;
    } else {
      const double len = lane_info.length();
      if (len * (1.0 - end_fraction) >= extend_len) {
        end_fraction += extend_len / len;
        break;
      } else {
        extend_len -= len * (1.0 - end_fraction);
        end_fraction = 1.0;
      }
    }
  }

  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           raw_lane_path.start_fraction(), end_fraction);
}

absl::StatusOr<mapping::LanePath> FindClosestTargetLanePathOnReset(
    const PlannerSemanticMapManager &psmm, const RouteSections &prev_sections,
    Vec2d ego_pos) {
  ASSIGN_OR_RETURN(
      const auto project_route_sections,
      ClampRouteSectionsBeforeArcLength(
          psmm, prev_sections,
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength));

  double ego_proj_s;
  ASSIGN_OR_RETURN(const auto ego_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, project_route_sections, ego_pos, &ego_proj_s));
  const double local_horizon =
      prev_sections.planning_horizon(psmm) + kLocalMapExtension;
  ASSIGN_OR_RETURN(
      const auto local_route_sections,
      ClampRouteSectionsBeforeArcLength(
          psmm, prev_sections, local_horizon + kDrivePassageKeepBehindLength));

  return ForwardExtendLanePathOnRouteSections(
      psmm, local_route_sections, ego_lane_path,
      local_horizon - (ego_lane_path.length() - ego_proj_s));
}

}  // namespace qcraft::planner
