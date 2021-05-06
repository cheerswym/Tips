#include "onboard/planner/speed/st_overlap_analyzer.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

namespace {

struct LaneInteraction {
  double fraction;
  mapping::LanePoint other_lane_point;
  StOverlapMetaProto::OverlapPriority priority;
  mapping::LaneInteractionProto::GeometricConfiguration geo_config;
  mapping::LaneProto::Type other_lane_type;

  LaneInteraction(
      double frac, mapping::LanePoint other_lane_pt,
      StOverlapMetaProto::OverlapPriority prio,
      mapping::LaneInteractionProto::GeometricConfiguration geo_conf,
      mapping::LaneProto::Type other_lane_tp)
      : fraction(frac),
        other_lane_point(other_lane_pt),
        priority(prio),
        geo_config(geo_conf),
        other_lane_type(other_lane_tp) {}

  static StOverlapMetaProto::OverlapPriority ReactionRuleToPriority(
      mapping::LaneInteractionProto::ReactionRule reaction_rule) {
    switch (reaction_rule) {
      case mapping::LaneInteractionProto::YIELD:
      case mapping::LaneInteractionProto::YIELD_ON_RED:
      case mapping::LaneInteractionProto::YIELD_ON_GREEN_CIRCLE:
      case mapping::LaneInteractionProto::YIELD_MERGE:
      case mapping::LaneInteractionProto::STOP: {
        return StOverlapMetaProto::LOW;
      }
      case mapping::LaneInteractionProto::PROCEED_MERGE:
      case mapping::LaneInteractionProto::PROCEED: {
        return StOverlapMetaProto::HIGH;
      }
      case mapping::LaneInteractionProto::FFA:
      case mapping::LaneInteractionProto::BOTH_STOP: {
        return StOverlapMetaProto::EQUAL;
      }
    }
  }
};

using LaneInteractionMap =
    absl::flat_hash_map<mapping::ElementId, std::vector<LaneInteraction>>;

// Only run overlap analyzer for this object type of st boundaries.
bool RunStOverlapAnalzyerByStBoundaryObjectType(
    StBoundaryProto::ObjectType object_type) {
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN:
      return true;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::UNKNOWN_OBJECT:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
      return false;
  }
}

// Only run overlap analyzer for this source type of st boundaries.
bool RunStOverlapnalyzerByStBoundarySourceType(StBoundary::SourceType source) {
  switch (source) {
    case StBoundary::SourceType::ST_OBJECT:
      return true;
    case StBoundary::SourceType::UNKNOWN:
    case StBoundary::SourceType::VIRTUAL:
    case StBoundary::SourceType::IMPASSABLE_BOUNDARY:
      return false;
  }
}

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  const auto& overlap_infos = st_boundary->overlap_infos();
  QCHECK(!overlap_infos.empty());

  const auto& first_overlap_info = overlap_infos.front();
  const auto& last_overlap_info = overlap_infos.back();

  const int state_num = st_traj.states().size();
  const int path_num = path.size();

  // If first object contour before overlap is behind the first AV box on path,
  // we consider it to be on the path at the beginning; if first object contour
  // after overlap is ahead of the last AV box on path, we consider it to be
  // on the path at the end.
  enum class RelativeLonPosition {
    BEHIND = 0,
    AHEAD = 1,
    INTERSECT = 2,
  };
  const auto compute_obj_relative_position_with_av_box =
      [&path, &st_traj, &vehicle_geometry_params](
          int av_path_idx, int obj_traj_idx) -> RelativeLonPosition {
    const auto& obj_contour = st_traj.states()[obj_traj_idx].contour;
    const auto& av_path_point = path[av_path_idx];
    const auto av_path_dir = Vec2d::FastUnitFromAngle(av_path_point.theta());
    Vec2d front, back;
    obj_contour.ExtremePoints(av_path_dir, &back, &front);
    const Vec2d av_path_pos = ToVec2d(av_path_point);
    if ((back - av_path_pos).Dot(av_path_dir) >
        vehicle_geometry_params.front_edge_to_center() +
            st_traj.required_lateral_gap()) {
      return RelativeLonPosition::AHEAD;
    } else if ((front - av_path_pos).Dot(av_path_dir) <
               -vehicle_geometry_params.back_edge_to_center() -
                   st_traj.required_lateral_gap()) {
      return RelativeLonPosition::BEHIND;
    } else {
      return RelativeLonPosition::INTERSECT;
    }
  };

  if (first_overlap_info.obj_idx == 0 ||
      compute_obj_relative_position_with_av_box(
          0, first_overlap_info.obj_idx - 1) == RelativeLonPosition::BEHIND) {
    // If first object contour out of path is in front of the last AV box, we
    // also consider it to be of type STAY.
    if (last_overlap_info.obj_idx == state_num - 1 ||
        compute_obj_relative_position_with_av_box(
            path_num - 1, last_overlap_info.obj_idx + 1) ==
            RelativeLonPosition::AHEAD) {
      return StOverlapMetaProto::STAY;
    } else {
      return StOverlapMetaProto::LEAVE;
    }
  }

  // If first object contour out of path is in front of the last AV box, we
  // also consider it to be of type ENTER.
  if (last_overlap_info.obj_idx == state_num - 1 ||
      compute_obj_relative_position_with_av_box(
          path_num - 1, last_overlap_info.obj_idx + 1) ==
          RelativeLonPosition::AHEAD) {
    return StOverlapMetaProto::ENTER;
  } else {
    // True for left side, false for right side.
    const auto get_side_on_path = [&path, &st_traj](int av_path_idx,
                                                    int obj_traj_idx) -> bool {
      const auto& av_path_point = path[av_path_idx];
      const auto& obj_traj_point = *st_traj.states()[obj_traj_idx].traj_point;
      const Vec2d ref = obj_traj_point.pos() - ToVec2d(av_path_point);
      const bool is_on_path_left =
          Vec2d::FastUnitFromAngle(av_path_point.theta()).CrossProd(ref) >= 0.0;
      return is_on_path_left;
    };

    const bool first_overlap_side = get_side_on_path(
        (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2,
        first_overlap_info.obj_idx);
    const bool last_overlap_side = get_side_on_path(
        (last_overlap_info.av_start_idx + last_overlap_info.av_end_idx) / 2,
        last_overlap_info.obj_idx);
    if (first_overlap_side == last_overlap_side) {
      return StOverlapMetaProto::INTERFERE;
    } else {
      return StOverlapMetaProto::CROSS;
    }
  }
}

struct OverlapSourcePriority {
  StOverlapMetaProto::OverlapSource source = StOverlapMetaProto::UNKNOWN_SOURCE;
  StOverlapMetaProto::OverlapPriority priority =
      StOverlapMetaProto::UNKNOWN_PRIORITY;
  std::string priority_reason;
  std::optional<double> time_to_lc_complete;
};

bool MatchOverlapWithLaneInteraction(StBoundaryProto::ObjectType object_type,
                                     const Vec2d& first_overlap_obj_pos,
                                     double first_overlap_obj_heading,
                                     const LaneInteraction& lane_interaction,
                                     const PlannerSemanticMapManager& psmm) {
  // TODO(renjie): Consider the object's current position.
  QCHECK(object_type == StBoundaryProto::VEHICLE ||
         object_type == StBoundaryProto::CYCLIST);

  const auto is_vehicle_drivable_lane_type =
      [](mapping::LaneProto::Type lane_type) {
        return lane_type != mapping::LaneProto::BICYCLE_ONLY &&
               lane_type != mapping::LaneProto::WALKING_STREET;
      };
  // TODO(renjie): Delete `lane_type == VIRTUAL` after it is deprecated.
  const auto is_cyclist_drivable_lane_type =
      [](mapping::LaneProto::Type lane_type) {
        return lane_type == mapping::LaneProto::BICYCLE_ONLY ||
               lane_type == mapping::LaneProto::MIXED_WITH_CYCLIST ||
               lane_type == mapping::LaneProto::VIRTUAL;
      };

  // We only match drivable other lanes for corresponding object type.
  if ((object_type == StBoundaryProto::VEHICLE &&
       !is_vehicle_drivable_lane_type(lane_interaction.other_lane_type)) ||
      (object_type == StBoundaryProto::CYCLIST &&
       !is_cyclist_drivable_lane_type(lane_interaction.other_lane_type))) {
    return false;
  }

  VLOG(4) << "Match interaction with lane: ("
          << lane_interaction.other_lane_point.lane_id() << ", "
          << lane_interaction.other_lane_point.fraction() << "), priority: "
          << StOverlapMetaProto::OverlapPriority_Name(lane_interaction.priority)
          << ", geo config: "
          << mapping::LaneInteractionProto::GeometricConfiguration_Name(
                 lane_interaction.geo_config);
  double fraction = 0.0;
  double min_dist = 0.0;
  if (!psmm.GetLaneProjectionAtLevel(
          psmm.GetLevel(), first_overlap_obj_pos,
          lane_interaction.other_lane_point.lane_id(), &fraction,
          /*point=*/nullptr, &min_dist)) {
    return false;
  }
  constexpr double MatchLaneInteractionDistThres = 2.0;  // m.
  VLOG(4) << "Object pos: " << first_overlap_obj_pos.DebugString()
          << ", distance to other lane: " << min_dist
          << ", closest other lane fraction: " << fraction;
  if (min_dist < MatchLaneInteractionDistThres) {
    // Check heading diff.
    const mapping::LanePoint closest_other_lane_point(
        lane_interaction.other_lane_point.lane_id(), fraction);
    const double closest_other_lane_point_heading =
        closest_other_lane_point.ComputeLerpTheta(*psmm.semantic_map_manager());
    constexpr double MatchLaneInteractionHeadingThres = M_PI / 6.0;
    const double heading_diff = std::abs(NormalizeAngle(
        first_overlap_obj_heading - closest_other_lane_point_heading));
    VLOG(4) << "Object heading diff with closest other lane point: "
            << heading_diff;
    if (heading_diff < MatchLaneInteractionHeadingThres) {
      // Matched lane interaction.
      return true;
    }
  }
  return false;
}

OverlapSourcePriority AnalyzeOverlapSourceAndPriority(
    const StBoundaryRef& st_boundary,
    StOverlapMetaProto::OverlapPattern overlap_pattern,
    const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const LaneInteractionMap& lane_interaction_map, double init_v) {
  VLOG(4) << "Analyze overlap source and priority for st-boundary "
          << st_boundary->id();

  OverlapSourcePriority res;
  // Only analyze source and priority for overlap of pattern ENTER, CROSS and
  // INTERFERE.
  if (overlap_pattern != StOverlapMetaProto::ENTER &&
      overlap_pattern != StOverlapMetaProto::CROSS &&
      overlap_pattern != StOverlapMetaProto::INTERFERE) {
    return res;
  }

  // Only analyze source and priority for overlap happening at a future path
  // point.
  if (st_boundary->bottom_left_point().s() <= 0.0) {
    return res;
  }

  // For an overlap of pattern ENTER, CROSS and INTERFERE, it should have a
  // positive min_t.
  if (st_boundary->min_t() <= 0.0) {
    QEVENT("renjie", "unexpected_nonnegative_min_t", [&](QEvent* qevent) {
      qevent->AddField("st_boundary_id", st_boundary->id())
          .AddField("pattern",
                    StOverlapMetaProto::OverlapPattern_Name(overlap_pattern))
          .AddField("min_t", st_boundary->min_t());
    });
  }

  // Low priority for all pedestrian overlaps.
  if (st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
    res.source = StOverlapMetaProto::OTHER;
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason = absl::StrCat(
        "LOW priority for object type ",
        StBoundaryProto::ObjectType_Name(st_boundary->object_type()));
    return res;
  }

  const auto& overlap_infos = st_boundary->overlap_infos();
  // In this function, 'fo' denotes 'first_overlap'.
  const auto& fo_info = overlap_infos.front();

  enum class LaneChangeSemantic { NONE = 0, LEFT = 1, RIGHT = 2 };
  LaneChangeSemantic fo_av_lane_change_semantic = LaneChangeSemantic::NONE;
  absl::flat_hash_set<mapping::ElementId> fo_av_lane_id_set;
  constexpr double kLaneChangeCheckLookAheadDist = 20.0;  // m.
  // Chech whether AV is changing lane during the first overlap up to a
  // lookahead distance. This is to compensate for the fact the AV front wheels
  // would enter target lane before its rac point.
  std::optional<std::vector<int>> lc_lane_path_id_history = std::nullopt;
  const double check_lc_end_path_s =
      path[fo_info.av_end_idx].s() + kLaneChangeCheckLookAheadDist;
  for (int i = fo_info.av_start_idx;
       i < path_semantics.size() && path[i].s() <= check_lc_end_path_s; ++i) {
    const auto& lane_path_id_history = path_semantics[i].lane_path_id_history;
    if (fo_av_lane_change_semantic == LaneChangeSemantic::NONE &&
        lane_path_id_history.back() != 0) {
      QCHECK_GT(lane_path_id_history.size(), 1);
      QCHECK_NE(lane_path_id_history.back(),
                lane_path_id_history[lane_path_id_history.size() - 2]);
      if (lane_path_id_history.back() >
          lane_path_id_history[lane_path_id_history.size() - 2]) {
        fo_av_lane_change_semantic = LaneChangeSemantic::LEFT;
      } else {
        fo_av_lane_change_semantic = LaneChangeSemantic::RIGHT;
      }
      lc_lane_path_id_history = lane_path_id_history;
    }
    if (i <= fo_info.av_end_idx) {
      fo_av_lane_id_set.insert(path_semantics[i].closest_lane_point.lane_id());
    }
  }

  const auto* fo_obj_point = st_traj.states()[fo_info.obj_idx].traj_point;
  const auto& fo_obj_pos = fo_obj_point->pos();
  if (fo_av_lane_change_semantic != LaneChangeSemantic::NONE) {
    // The overlap happens during AV lane change. Check if the object is on
    // the corresponding side of path to see if it is being cut in by AV.
    const auto& fo_av_path_point =
        path[(fo_info.av_start_idx + fo_info.av_end_idx) / 2];
    const bool is_fo_obj_on_path_left =
        Vec2d::FastUnitFromAngle(fo_av_path_point.theta())
            .CrossProd(fo_obj_pos - ToVec2d(fo_av_path_point)) >= 0.0;
    if ((fo_av_lane_change_semantic == LaneChangeSemantic::LEFT &&
         is_fo_obj_on_path_left) ||
        (fo_av_lane_change_semantic == LaneChangeSemantic::RIGHT &&
         !is_fo_obj_on_path_left)) {
      res.source = StOverlapMetaProto::AV_CUTIN;
      res.priority = StOverlapMetaProto::LOW;
      res.priority_reason = "LOW priority for AV cutting in object";
      QCHECK(lc_lane_path_id_history.has_value());
      for (int i = 0; i < path_semantics.size(); ++i) {
        if (path_semantics[i].lane_path_id_history ==
            *lc_lane_path_id_history) {
          res.time_to_lc_complete = path[i].s() / (init_v + 1e-6);
          break;
        }
      }
      QCHECK(res.time_to_lc_complete.has_value());
      return res;
    }
  } else {
    // The overlap happends during AV lane keeping.
    const auto fo_obj_heading = fo_obj_point->theta();
    // Check if first overlap can be matched to a lane interaction.
    for (const auto lane_id : fo_av_lane_id_set) {
      const auto* lane_interactions = FindOrNull(lane_interaction_map, lane_id);
      if (lane_interactions == nullptr) continue;
      for (const auto& lane_interaction : *lane_interactions) {
        if (MatchOverlapWithLaneInteraction(st_boundary->object_type(),
                                            fo_obj_pos, fo_obj_heading,
                                            lane_interaction, psmm)) {
          if (lane_interaction.geo_config ==
              mapping::LaneInteractionProto::MERGE) {
            res.source = StOverlapMetaProto::LANE_MERGE;
            res.priority = lane_interaction.priority;
            res.priority_reason = absl::StrCat(
                StOverlapMetaProto::OverlapPriority_Name(
                    lane_interaction.priority),
                " priority for AV lane ", lane_id, " merging object lane ",
                lane_interaction.other_lane_point.lane_id());
          } else {
            res.source = StOverlapMetaProto::LANE_CROSS;
            res.priority = lane_interaction.priority;
            res.priority_reason = absl::StrCat(
                StOverlapMetaProto::OverlapPriority_Name(
                    lane_interaction.priority),
                " priority for AV lane ", lane_id, " crossing object lane ",
                lane_interaction.other_lane_point.lane_id());
          }
          // TODO(renjie): Consider to match the most likely interaction instead
          // of early exit.
          return res;
        }
      }
    }
  }

  // Other cases (AV is changing lane but object is not cut in by AV, or AV is
  // keeping lane but the overlap is not matched to a lane interaction), we
  // consider the object is cutting AV.
  res.source = StOverlapMetaProto::OBJECT_CUTIN;
  if (st_boundary->object_type() == StBoundaryProto::CYCLIST &&
      path_semantics[(fo_info.av_start_idx + fo_info.av_end_idx) / 2]
              .lane_semantic == LaneSemantic::INTERSECTION_RIGHT_TURN) {
    res.priority = StOverlapMetaProto::LOW;
    res.priority_reason =
        "LOW priority for AV being cut in by cyclist during right turn";
    return res;
  }
  res.priority = StOverlapMetaProto::HIGH;
  res.priority_reason = "HIGH priority for AV being cut in by object";
  return res;
}

bool IsDrivingParallel(const DiscretizedPath& path,
                       const SpacetimeObjectTrajectory& st_traj) {
  constexpr double kReflectCurrentDrivingPatternDist = 1.5;  // m.
  std::vector<double> av_headings;
  av_headings.reserve(15);
  for (const auto& path_point : path) {
    if (path_point.s() > kReflectCurrentDrivingPatternDist) {
      continue;
    }
    av_headings.push_back(path_point.theta());
  }
  std::vector<double> agent_headings;
  agent_headings.reserve(15);
  for (const auto& state : st_traj.states()) {
    if (state.traj_point->s() > kReflectCurrentDrivingPatternDist) {
      continue;
    }
    agent_headings.push_back(state.traj_point->theta());
  }
  const double av_mean_heading =
      std::accumulate(av_headings.begin(), av_headings.end(), 0.0) /
      av_headings.size();
  const double agent_mean_heading =
      std::accumulate(agent_headings.begin(), agent_headings.end(), 0.0) /
      agent_headings.size();

  constexpr double kDrivingParallelHeadingThres = M_PI / 6.0;
  return std::abs(NormalizeAngle(av_mean_heading - agent_mean_heading)) <
         kDrivingParallelHeadingThres;
}

StOverlapMetaProto::ModificationType AnalyzeOverlapModificationType(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    StOverlapMetaProto::OverlapPattern overlap_pattern,
    StOverlapMetaProto::OverlapSource overlap_source,
    StOverlapMetaProto::OverlapPriority overlap_priority) {
  if (overlap_priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
    // An overlap without priority must be non-interactive.
    return StOverlapMetaProto::NON_MODIFIABLE;
  }
  QCHECK(st_boundary->object_type() == StBoundaryProto::VEHICLE ||
         st_boundary->object_type() == StBoundaryProto::CYCLIST ||
         st_boundary->object_type() == StBoundaryProto::PEDESTRIAN);

  // For vehicles/cyclists.
  if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
      st_boundary->object_type() == StBoundaryProto::CYCLIST) {
    switch (overlap_source) {
      case StOverlapMetaProto::LANE_MERGE:
      case StOverlapMetaProto::LANE_CROSS:
      case StOverlapMetaProto::AV_CUTIN: {
        return StOverlapMetaProto::LON_MODIFIABLE;
      }
      case StOverlapMetaProto::OBJECT_CUTIN: {
        if (IsDrivingParallel(path, st_traj)) {
          return StOverlapMetaProto::LON_LAT_MODIFIABLE;
        } else {
          return StOverlapMetaProto::LON_MODIFIABLE;
        }
      }
      case StOverlapMetaProto::UNKNOWN_SOURCE:
      case StOverlapMetaProto::OTHER: {
        return StOverlapMetaProto::NON_MODIFIABLE;
      }
    }
  }

  // For pedestrians.
  return StOverlapMetaProto::NON_MODIFIABLE;
}

StOverlapMetaProto AnalyzeStOverlap(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const LaneInteractionMap& lane_interaction_map,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double init_v) {
  StOverlapMetaProto overlap_meta;
  // Step 1: Analyze overlap pattern.
  overlap_meta.set_pattern(AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                 vehicle_geometry_params));
  // Step 2: Analyze overlap source and priority.
  const auto source_priority = AnalyzeOverlapSourceAndPriority(
      st_boundary, overlap_meta.pattern(), st_traj, psmm, path, path_semantics,
      lane_interaction_map, init_v);
  overlap_meta.set_source(source_priority.source);
  overlap_meta.set_priority(source_priority.priority);
  overlap_meta.set_priority_reason(std::move(source_priority.priority_reason));
  if (source_priority.time_to_lc_complete.has_value()) {
    overlap_meta.set_time_to_lc_complete(*source_priority.time_to_lc_complete);
  }

  // Step 3: Analyze modification type.
  overlap_meta.set_modification_type(AnalyzeOverlapModificationType(
      st_boundary, st_traj, path, overlap_meta.pattern(), overlap_meta.source(),
      overlap_meta.priority()));

  return overlap_meta;
}

}  // namespace

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary) {
  if (!RunStOverlapnalyzerByStBoundarySourceType(st_boundary->source_type())) {
    return false;
  }
  if (!RunStOverlapAnalzyerByStBoundaryObjectType(st_boundary->object_type())) {
    return false;
  }
  if (st_boundary->is_stationary()) return false;
  return true;
}

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params, double init_v,
    std::vector<StBoundaryRef>* st_boundaries) {
  SCOPED_QTRACE("AnalyzeStOverlaps");

  // Generate lane interaction map.
  LaneInteractionMap path_lane_interaction_map;
  for (const auto& path_semantic : path_semantics) {
    const auto& current_closest_lane_point =
        path_semantics[0].closest_lane_point;
    const auto& closest_lane_point = path_semantic.closest_lane_point;
    if (path_lane_interaction_map.contains(closest_lane_point.lane_id())) {
      continue;
    }
    const auto& lane_proto =
        psmm.FindLaneByIdOrDie(closest_lane_point.lane_id());
    path_lane_interaction_map[closest_lane_point.lane_id()] = {};
    for (const auto& interaction : lane_proto.interactions()) {
      const auto& other_lane_proto =
          psmm.FindLaneByIdOrDie(interaction.other_lane_id());
      // Only add lane interactions beyond current AV position.
      if (closest_lane_point.lane_id() ==
              current_closest_lane_point.lane_id() &&
          interaction.this_lane_fraction() <
              current_closest_lane_point.fraction()) {
        continue;
      }
      path_lane_interaction_map[closest_lane_point.lane_id()].emplace_back(
          interaction.this_lane_fraction(),
          mapping::LanePoint(interaction.other_lane_id(),
                             interaction.other_lane_fraction()),
          LaneInteraction::ReactionRuleToPriority(interaction.reaction_rule()),
          interaction.geometric_configuration(), other_lane_proto.type());
    }
  }

  if (VLOG_IS_ON(4)) {
    for (const auto& [lane_id, lane_interactions] : path_lane_interaction_map) {
      VLOG(4) << "AV path lane: " << lane_id;
      for (const auto& lane_interaction : lane_interactions) {
        VLOG(4) << "Fraction: " << lane_interaction.fraction
                << ", other lane point: "
                << lane_interaction.other_lane_point.DebugString()
                << ", AV precedence: "
                << StOverlapMetaProto::OverlapPriority_Name(
                       lane_interaction.priority);
      }
    }
  }

  // TODO(renjie): See if need to parallelize this loop.
  for (auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    // All st-object-generated st-boundaries must have a traj_id.
    QCHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));

    StOverlapMetaProto overlap_meta = AnalyzeStOverlap(
        st_boundary, *traj, psmm, path, path_semantics,
        path_lane_interaction_map, vehicle_geometry_params, init_v);

    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}

}  // namespace planner
}  // namespace qcraft
