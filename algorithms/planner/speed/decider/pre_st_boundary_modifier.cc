#include "onboard/planner/speed/decider/pre_st_boundary_modifier.h"

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"

namespace qcraft {
namespace planner {
namespace {

std::optional<StBoundaryModificationResult> ModifyOncomingStBoundary(
    const StGraph& st_graph, const StBoundaryWithDecision& st_boundary_wd,
    const SpacetimeObjectTrajectory& st_traj, double current_v,
    const DiscretizedPath& path) {
  // An st-boundary is considered to be ONCOMING if:
  // 1. Its overlap source is OBJECT_CUTIN;
  // 2. Its first-overlap heading diff is beyond certain threshold;
  // 3. Its first-overlap s_lower is larger than last-overlap s_lower.
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  if (st_boundary.overlap_meta()->source() !=
      StOverlapMetaProto::OBJECT_CUTIN) {
    return std::nullopt;
  }

  if (st_boundary.bottom_left_point().s() <=
      st_boundary.bottom_right_point().s()) {
    return std::nullopt;
  }

  const auto& overlap_infos = st_boundary.overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto* first_overlap_obj_point =
      st_traj.states()[first_overlap_info.obj_idx].traj_point;
  const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
  const auto first_overlap_av_middle_heading =
      path[(first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) /
           2]
          .theta();
  constexpr double kOnComingThreshold = 5.0 * M_PI / 6.0;
  if (std::abs(NormalizeAngle(first_overlap_obj_heading -
                              first_overlap_av_middle_heading)) <
      kOnComingThreshold) {
    return std::nullopt;
  }

  VLOG(2) << "St-boundary " << st_boundary_wd.id()
          << " is considered to be ONCOMING.";

  // Only modify the oncoming prediction if it would cause uncomfortable brake.
  const double const_speed_s = current_v * st_boundary.bottom_right_point().t();
  if (st_boundary.bottom_right_point().s() > const_speed_s) {
    // No brake is needed.
    return std::nullopt;
  }
  const double estimated_av_decel =
      2.0 * (const_speed_s - st_boundary.bottom_right_point().s()) /
      Sqr(st_boundary.bottom_right_point().t());
  constexpr double kUncomfortableDecel = 1.0;  // m/s^2.
  if (estimated_av_decel < kUncomfortableDecel) {
    return std::nullopt;
  }

  // Modify oncoming spacetime trajectory.
  VLOG(2) << "Modify ONCOMING st-boundary " << st_boundary.id();
  constexpr double kOncomingHumanDelay = 0.5;   // s.
  constexpr double kOncomingHumanDecel = -2.5;  // s.
  constexpr double kStandoff = 1.0;             // m.

  const double first_overlap_s = first_overlap_obj_point->s();
  const double start_s = st_traj.states().front().traj_point->s();
  const double start_v = st_traj.states().front().traj_point->v();
  const auto decel_to_stop = ComputeSafeStopDecelWithDelayedAction(
      first_overlap_s - start_s - kStandoff, start_v, /*a0=*/0.0,
      kOncomingHumanDelay);
  const double oncoming_obj_decel =
      decel_to_stop.has_value() ? std::max(*decel_to_stop, kOncomingHumanDecel)
                                : kOncomingHumanDecel;

  // Make new spacetime trajectory.
  // FIXME(renjie): The current modification type for OBJECT_CUT_IN is
  // LON_LAT_MODIFIABLE but we modifies oncoming st-boundaries longitudinally
  // just to be in consistent with the old logic. The modification method should
  // be subject to the modification type given in overlap meta.
  auto new_st_traj = CreateSpacetimeTrajectoryByDecelAfterDelay(
      st_traj, kOncomingHumanDelay, oncoming_obj_decel);

  // Generate new st_boundaries.
  auto new_st_boundaries = st_graph.MapMovingSpacetimeObject(new_st_traj);
  std::vector<StBoundaryWithDecision> st_boundaries_wd;
  st_boundaries_wd.reserve(new_st_boundaries.size());
  const auto modifier_type = StBoundaryModifierProto::ONCOMING;
  for (auto& st_boundary : new_st_boundaries) {
    st_boundary->set_id(absl::StrCat(st_boundary->id(), "|m"));
    if (st_boundary_wd.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundaries_wd.emplace_back(std::move(st_boundary),
                                    st_boundary_wd.decision_type(),
                                    st_boundary_wd.decision_reason());
    } else {
      st_boundaries_wd.emplace_back(
          std::move(st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(),
          /*pass_time=*/0.0, /*yield_time=*/0.0);
    }
  }

  return StBoundaryModificationResult(
      {.newly_generated_st_boundaries_wd = std::move(st_boundaries_wd),
       .processed_st_traj = std::move(new_st_traj),
       .modifier_type = modifier_type});
}

std::optional<StBoundaryModificationResult> PreModifyStBoundary(
    const PreStboundaryModifierInput& input,
    const StBoundaryWithDecision& st_boundary_wd) {
  std::optional<StBoundaryModificationResult> res = std::nullopt;
  const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
  // Only pre-modify st-boundaries having overlap meta.
  if (!st_boundary.overlap_meta().has_value()) return res;

  if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE) return res;

  QCHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
         st_boundary.object_type() == StBoundaryProto::CYCLIST ||
         st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  const auto& traj_id = st_boundary.traj_id();
  QCHECK(traj_id.has_value());

  const auto* traj =
      QCHECK_NOTNULL(input.st_traj_mgr->FindTrajectoryById(*traj_id));

  // Modify oncoming predictions that would cause uncomfortable brake.
  res = ModifyOncomingStBoundary(*input.st_graph, st_boundary_wd, *traj,
                                 input.current_v, *input.path);
  if (res.has_value()) {
    return res;
  }

  // TODO(renjie): Implement other pre st-boundary modification functions.
  return res;
}

}  // namespace

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects) {
  QCHECK_NOTNULL(input.st_graph);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.path);

  ModifyAndUpdateStBoundaries(
      input,
      std::function<std::optional<StBoundaryModificationResult>(
          const PreStboundaryModifierInput&, const StBoundaryWithDecision&)>(
          PreModifyStBoundary),
      processed_st_objects, st_boundaries_wd);
}

}  // namespace planner
}  // namespace qcraft
