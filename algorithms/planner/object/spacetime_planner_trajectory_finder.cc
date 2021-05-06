#include "onboard/planner/object/spacetime_planner_trajectory_finder.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace qcraft {
namespace planner {
namespace {

// Desired lateral distance for nudge moving objects.
constexpr double kLateralDistanceNudgeMovingObs = 0.5;  // m.
// Desired lateral movement for considering a moving object as
// lateral object.
constexpr double kLateralMovingObjectThreshold = 0.75;  // m.
// Following params are used for hysteresis to ensure decisions stability.
// Minimal lateral distance for nudge moving objects.
constexpr double kHysteresisLateralDistanceNudgeMovingObs = 0.30;  // m.
// Maximally allowed lateral movement for considering a moving object as
// lateral object.
constexpr double kHysteresisLateralMovingObjectThreshold = 1.0;  // m.

constexpr double kCloseMovingObjThreshold = 0.75;  // m.

struct LateralRelationInfo {
  double obj_lmin = std::numeric_limits<double>::lowest();
  double obj_lmax = std::numeric_limits<double>::max();
  double ref_lmin = std::numeric_limits<double>::lowest();
  double ref_lmax = std::numeric_limits<double>::max();
};

bool CanOvertake(absl::Span<const LateralRelationInfo> lat_infos,
                 double av_width, double lat_nudge_dist,
                 double lat_obj_movement) {
  // Check if we can nudge from left.
  bool has_enough_space_on_left = true;
  double l_trace_min = std::numeric_limits<double>::max();
  for (const auto& lat_info : lat_infos) {
    l_trace_min = std::min(l_trace_min, lat_info.obj_lmax);
    const double space = lat_info.ref_lmax - lat_info.obj_lmax - av_width;
    const double obj_trend = lat_info.obj_lmax - l_trace_min;
    // If we don't have enough space for overtaking at left or if object is
    // moving towards center.
    if (space < lat_nudge_dist || obj_trend > lat_obj_movement) {
      has_enough_space_on_left = false;
      break;
    }
  }
  // Check if we can nudge from right.
  bool has_enough_space_on_right = true;
  double l_trace_max = std::numeric_limits<double>::lowest();
  for (const auto& lat_info : lat_infos) {
    l_trace_max = std::max(l_trace_max, lat_info.obj_lmin);
    const double space = lat_info.obj_lmin - lat_info.ref_lmin - av_width;
    const double obj_trend = l_trace_max - lat_info.obj_lmin;
    // If we don't have enough space for overtaking at right or if object is
    // moving towards center.
    if (space < lat_nudge_dist || obj_trend > lat_obj_movement) {
      has_enough_space_on_right = false;
      break;
    }
  }
  if (has_enough_space_on_left || has_enough_space_on_right) {
    return true;
  }

  return false;
}

bool IsFrontObject(const DrivePassage& drive_passage,
                   const FrenetCoordinate& av_sl, const Box2d& bbox,
                   double av_length, bool prev_st_planner_obj) {
  const auto obj_frenet_box_or = drive_passage.QueryFrenetBoxAt(bbox);
  // No projection, pass.
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  // Object's current s is less than ego s (behind ego vehicle), do not consider
  // in space time planner.
  if (prev_st_planner_obj) {
    if (av_sl.s < obj_frenet_box_or->s_max) {
      return true;
    }
  } else {
    if (av_sl.s + av_length * 0.5 < obj_frenet_box_or->s_max) {
      return true;
    }
  }
  return false;
}

bool IsFrontOrSideObject(const DrivePassage& drive_passage,
                         const FrenetCoordinate& av_sl, double av_length,
                         const Box2d& bbox) {
  const auto obj_frenet_box_or = drive_passage.QueryFrenetBoxAt(bbox);
  // No projection, pass.
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  // Object's current front s is less than ego s (behind ego vehicle), do not
  // consider in space time planner.
  if (av_sl.s + av_length * 0.5 < obj_frenet_box_or->s_max) {
    return true;
  }
  return false;
}
bool IsFrontOrSideObjectForEmergencySituation(const DrivePassage& drive_passage,
                                              const FrenetBox& av_sl_box,
                                              const Box2d& bbox) {
  const auto obj_frenet_box_or = drive_passage.QueryFrenetBoxAt(bbox);
  // No projection, pass.
  if (!obj_frenet_box_or.ok()) {
    return false;
  }
  // Object's current front s is less than ego s (behind ego vehicle), do not
  // consider in space time planner.
  if (av_sl_box.s_max < obj_frenet_box_or->s_max) {
    return true;
  }
  return false;
}
}  // namespace

SpacetimePlannerTrajectoryReason::Type
StationarySpacetimePlannerTrajectoryFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (traj.is_stationary()) {
    return SpacetimePlannerTrajectoryReason::STATIONARY;
  }
  return SpacetimePlannerTrajectoryReason::NONE;
}

FrontSideMovingSpacetimePlannerTrajectoryFinder::
    FrontSideMovingSpacetimePlannerTrajectoryFinder(
        const DrivePassage* drive_passage, const PathSlBoundary* sl_boundary,
        const ApolloTrajectoryPointProto* plan_start_point,
        const SpacetimePlannerTrajectories* prev_st_trajs, double av_length,
        double av_width)
    : drive_passage_(drive_passage),
      path_sl_boundary_(sl_boundary),
      plan_start_point_(plan_start_point),
      av_length_(av_length),
      av_width_(av_width) {
  const Vec2d av_pos(plan_start_point_->path_point().x(),
                     plan_start_point_->path_point().y());
  auto av_sl_or = drive_passage_->QueryFrenetCoordinateAt(av_pos);
  QCHECK(av_sl_or.ok());
  av_sl_ = std::move(*av_sl_or);
  for (const auto& st_traj_proto : prev_st_trajs->trajectory()) {
    if (!prev_st_planner_obj_decisions_.contains(st_traj_proto.id()) ||
        st_traj_proto.reason() != SpacetimePlannerTrajectoryReason::NONE) {
      prev_st_planner_obj_decisions_[st_traj_proto.id()] =
          st_traj_proto.reason();
    }
  }
}

SpacetimePlannerTrajectoryReason::Type
FrontSideMovingSpacetimePlannerTrajectoryFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  // Don't consider stationary object here.
  if (traj.is_stationary()) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  const auto& states = traj.states();
  const auto* ptr_reason =
      FindOrNull(prev_st_planner_obj_decisions_, traj.planner_object()->id());
  const bool prev_st_planner_obj =
      (ptr_reason != nullptr) &&
      (*ptr_reason != SpacetimePlannerTrajectoryReason::NONE);
  if (!IsFrontObject(*drive_passage_, av_sl_, states[0].box, av_length_,
                     prev_st_planner_obj)) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  std::vector<LateralRelationInfo> vec_lat_relation_info;
  vec_lat_relation_info.reserve(states.size());
  for (const auto& state : states) {
    const auto frenet_box_or = drive_passage_->QueryFrenetBoxAt(state.box);
    if (!frenet_box_or.ok()) {
      continue;
    }
    const auto l_boundaries =
        path_sl_boundary_->QueryBoundaryL(frenet_box_or->s_min);
    vec_lat_relation_info.push_back({.obj_lmin = frenet_box_or->l_min,
                                     .obj_lmax = frenet_box_or->l_max,
                                     .ref_lmin = l_boundaries.first,
                                     .ref_lmax = l_boundaries.second});
  }
  if (vec_lat_relation_info.empty()) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  double lat_space = kLateralDistanceNudgeMovingObs;
  double lat_allowed_mov = kLateralMovingObjectThreshold;
  // If previously spacetime planner trajectory.
  if (ptr_reason != nullptr &&
      *ptr_reason != SpacetimePlannerTrajectoryReason::NONE) {
    lat_space = kHysteresisLateralDistanceNudgeMovingObs;
    lat_allowed_mov = kHysteresisLateralMovingObjectThreshold;
  }
  if (CanOvertake(vec_lat_relation_info, av_width_, lat_space,
                  lat_allowed_mov)) {
    return SpacetimePlannerTrajectoryReason::SIDE;
  }
  return SpacetimePlannerTrajectoryReason::NONE;
}

DangerousSideMovingSpacetimePlannerTrajectoryFinder::
    DangerousSideMovingSpacetimePlannerTrajectoryFinder(
        const Box2d& av_box, const DrivePassage* drive_passage)
    : av_box_(av_box), drive_passage_(drive_passage), av_sl_box_(std::nullopt) {
  auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = std::move(*av_sl_box_or);
  }
}
SpacetimePlannerTrajectoryReason::Type
DangerousSideMovingSpacetimePlannerTrajectoryFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  // Don't consider stationary object here.
  if (traj.is_stationary()) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  if (av_box_.DistanceTo(traj.states()[0].box) > kCloseMovingObjThreshold) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObjectForEmergencySituation(*drive_passage_, *av_sl_box_,
                                               traj.states()[0].box)) {
    return SpacetimePlannerTrajectoryReason::EMERGENCY_AVOIDANCE;
  }
  return SpacetimePlannerTrajectoryReason::NONE;
}

FrontMovingSpacetimePlannerTrajectoryFinder::
    FrontMovingSpacetimePlannerTrajectoryFinder(
        const DrivePassage* drive_passage,
        const ApolloTrajectoryPointProto* plan_start_point, double av_length)
    : drive_passage_(drive_passage),
      plan_start_point_(plan_start_point),
      av_length_(av_length) {
  const Vec2d av_pos(plan_start_point_->path_point().x(),
                     plan_start_point_->path_point().y());
  auto av_sl_or = drive_passage_->QueryFrenetCoordinateAt(av_pos);
  QCHECK(av_sl_or.ok());
  av_sl_ = std::move(*av_sl_or);
}

SpacetimePlannerTrajectoryReason::Type
FrontMovingSpacetimePlannerTrajectoryFinder::Find(
    const SpacetimeObjectTrajectory& traj) const {
  // Don't consider stationary obstacle here.
  if (traj.is_stationary()) {
    return SpacetimePlannerTrajectoryReason::NONE;
  }
  if (IsFrontOrSideObject(*drive_passage_, av_sl_, av_length_,
                          traj.states()[0].box)) {
    return SpacetimePlannerTrajectoryReason::FRONT;
  }
  return SpacetimePlannerTrajectoryReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
