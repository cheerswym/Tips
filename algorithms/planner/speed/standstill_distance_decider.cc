#include "onboard/planner/speed/standstill_distance_decider.h"

#include <optional>
#include <utility>

#include "onboard/lite/logging.h"

namespace qcraft {
namespace planner {
namespace {

inline bool IsStBoundaryStalledObject(
    const StBoundary& st_boundary,
    const absl::flat_hash_set<std::string>& stalled_object_ids) {
  QCHECK(st_boundary.traj_id().has_value());
  const auto obj_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
      *st_boundary.traj_id());
  return stalled_object_ids.contains(obj_id);
}

std::optional<Box2d> BuildBoomBarrierBoxOr(const mapping::LanePath& lane_path) {
  constexpr double kForwardDistance = 50.0;      // m.
  constexpr double kBoomBarrierBoxLength = 1.0;  // m.
  constexpr double kBoomBarrierBoxWidth = 2.0;   // m.
  const auto lanes_info =
      lane_path.BeforeArclength(kForwardDistance).GetLanesInfo();
  for (const auto* lane_info : lanes_info) {
    if (lane_info->endpoint_toll) {
      QCHECK_GE(lane_info->points_smooth.size(), 2);
      const auto& points = lane_info->points_smooth;
      const Vec2d end_vec(points.back() - points[points.size() - 2]);
      return Box2d(points.back(), end_vec.FastAngle(), kBoomBarrierBoxLength,
                   kBoomBarrierBoxWidth);
    }
  }
  return std::nullopt;
}

// BANDAID(ping): This is a hack to identify a gate boom barrier.
bool IsStaticStBoundaryBoomBarrier(
    const StBoundary& st_boundary, const mapping::LanePath& lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  const auto barrier_box = BuildBoomBarrierBoxOr(lane_path);
  if (!barrier_box.has_value()) return false;
  QCHECK(st_boundary.traj_id().has_value());
  const auto* obj =
      QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*st_boundary.traj_id()));
  return obj->contour().HasOverlap(*barrier_box);
}

// return: {follow_standstill_distance, lead_standstill_distance}.
std::pair<double, double> GetStBoundaryStandStillDistance(
    const StBoundary& st_boundary,
    const SpeedFinderParamsProto& speed_finder_params,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const mapping::LanePath* lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ConstraintManager& constraint_mgr) {
  double follow_standstill_distance = 0.0;
  double lead_standstill_distance = 0.0;
  switch (st_boundary.object_type()) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN: {
      if (st_boundary.is_stationary()) {
        if (IsStBoundaryStalledObject(st_boundary, stalled_object_ids)) {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance_for_static_obj();
        } else {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance();
        }
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance();
      }
      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::STATIC: {
      if (lane_path != nullptr &&
          IsStaticStBoundaryBoomBarrier(st_boundary, *lane_path, st_traj_mgr)) {
        constexpr double kTollStandstillDist = 1.5;  // m.
        follow_standstill_distance = kTollStandstillDist;
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance_for_static_obj();
      }
      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::IMPASSABLE_BOUNDARY: {
      follow_standstill_distance =
          speed_finder_params.follow_standstill_distance_for_curb();
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::VIRTUAL: {
      const auto it = std::find_if(
          constraint_mgr.StopLine().begin(), constraint_mgr.StopLine().end(),
          [&st_boundary](const ConstraintProto::StopLineProto& stopline) {
            return stopline.id() == st_boundary.id();
          });
      QCHECK(it != constraint_mgr.StopLine().end());
      follow_standstill_distance = it->standoff();
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::UNKNOWN_OBJECT: {
      QLOG(FATAL) << "Unpexted "
                  << StBoundaryProto::ObjectType_Name(st_boundary.object_type())
                  << " st-boundary " << st_boundary.id();
      break;
    }
  }

  return std::make_pair(follow_standstill_distance, lead_standstill_distance);
}

}  // namespace

void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDeciderInput& input,
    StBoundaryWithDecision* st_boundary_wd) {
  QCHECK_NOTNULL(input.speed_finder_params);
  QCHECK_NOTNULL(input.stalled_object_ids);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.constraint_mgr);
  QCHECK_NOTNULL(st_boundary_wd);
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  const auto standstill_distance = GetStBoundaryStandStillDistance(
      st_boundary, *input.speed_finder_params, *input.stalled_object_ids,
      input.lane_path, *input.st_traj_mgr, *input.constraint_mgr);
  st_boundary_wd->set_follow_standstill_distance(standstill_distance.first);
  st_boundary_wd->set_lead_standstill_distance(standstill_distance.second);

  return;
}

}  // namespace planner
}  // namespace qcraft
