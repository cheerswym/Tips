#include "onboard/planner/decision/cautious_brake_decider.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
namespace qcraft {
namespace planner {
namespace {
constexpr double kCrosswalkAssociationDistance = 2.0;
constexpr double kEpsilon = 0.1;
constexpr double kDrivePassageEndPointEpsilon = 0.1;
constexpr double kIntersectionSpeedReductionRatio = 0.75;
constexpr double kEmptyCrossWalkSpeedReductionRatio = 0.9;
constexpr double kVRUNearCrossWalkSpeedReductionRatio = 0.8;
constexpr double kVRUOnCrossWalkSpeedReductionRatio = 0.7;

constexpr double kMinCautiousSpeed = 3.0;  // m/s

bool IsObjectOnCrosswalk(const mapping::CrosswalkInfo &cw,
                         const SpacetimeObjectTrajectory &traj) {
  return cw.polygon_smooth.DistanceSquareTo(
             traj.planner_object()->pose().pos()) < Sqr(kEpsilon);
}

bool IsObjectNearCrosswalk(const mapping::CrosswalkInfo &cw,
                           const SpacetimeObjectTrajectory &traj) {
  return cw.polygon_smooth.DistanceSquareTo(
             traj.planner_object()->pose().pos()) <
         Sqr(kCrosswalkAssociationDistance);
}

bool IsAnyVruOnCrosswalk(const mapping::CrosswalkInfo &cw,
                         const SpacetimeTrajectoryManager &st_mgr) {
  for (const auto &traj : st_mgr.trajectories()) {
    if (IsVulnerableRoadUserType(traj->planner_object()->type())) {
      if (IsObjectOnCrosswalk(cw, *traj)) {
        return true;
      }
    }
  }
  return false;
}

bool IsAnyVruNearCrosswalk(const mapping::CrosswalkInfo &cw,
                           const SpacetimeTrajectoryManager &st_mgr) {
  for (const auto &traj : st_mgr.trajectories()) {
    if (IsVulnerableRoadUserType(traj->planner_object()->type())) {
      if (IsObjectNearCrosswalk(cw, *traj)) {
        return true;
      }
    }
  }
  return false;
}

absl::StatusOr<ConstraintProto::SpeedRegionProto> ProcessMapElement(
    const mapping::LanePath &lane_path,
    const mapping::LanePath::LaneSegment &seg,
    const mapping::LaneInfo &lane_info, const std::pair<int, Vec2d> &element,
    double passage_min_s, double passage_max_s, double lane_speed_limit,
    double speed_reduction_ratio) {
  if (seg.start_fraction > element.second.y()) {
    return absl::NotFoundError("Map Element not on current seg.");
  }

  const auto start_point = lane_info.LerpPointFromFraction(element.second.x());
  // End point.
  const auto end_point = lane_info.LerpPointFromFraction(element.second.y());
  ConstraintProto::SpeedRegionProto cautious_brake_constraint;
  start_point.ToProto(cautious_brake_constraint.mutable_start_point());
  end_point.ToProto(cautious_brake_constraint.mutable_end_point());

  const double start_point_s = lane_path.LaneIndexPointToArclength(
      /*lane_index_point=*/{seg.lane_index, element.second.x()});
  const double end_point_s = lane_path.LaneIndexPointToArclength(
      /*lane_index_point=*/{seg.lane_index, element.second.y()});

  if (start_point_s >= passage_max_s - kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element beyond current drive passage. "
                        "start_point_s: %f, passage_max_s:%f",
                        start_point_s, passage_max_s));
  } else if (end_point_s <= passage_min_s + kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element behind current drive passage. "
                        "end_point_s:%f, drive passage_min_s: %f",
                        end_point_s, passage_min_s));
  } else if (start_point_s + kDrivePassageEndPointEpsilon >= end_point_s) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element range on drive passage is too short. "
                        "start_pont_s: %f, end_point_s: %f",
                        start_point_s, end_point_s));
  }
  cautious_brake_constraint.set_start_s(std::max(start_point_s, passage_min_s));
  cautious_brake_constraint.set_end_s(std::min(end_point_s, passage_max_s));
  const auto speed_limit =
      std::max(kMinCautiousSpeed, lane_speed_limit * speed_reduction_ratio);
  cautious_brake_constraint.set_max_speed(speed_limit);

  return cautious_brake_constraint;
}
}  // namespace

std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const DrivePassage &passage, const SpacetimeTrajectoryManager &st_mgr) {
  absl::flat_hash_map<std::string,
                      std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  const auto &lane_path = passage.lane_path();
  for (const auto &seg : lane_path) {
    const auto &lane_info =
        planner_semantic_map_manager.FindLaneInfoOrDie(seg.lane_id);
    double speed_limit =
        planner_semantic_map_manager.QueryLaneSpeedLimitById(seg.lane_id);
    // Intersection.
    for (const auto &intersection : lane_info.intersections) {
      const int id =
          planner_semantic_map_manager.IntersectionAt(intersection.first).id;
      std::string str_id =
          absl::StrFormat("cautious_brake_intersection_%d", id);
      auto cautious_brake_constraint = ProcessMapElement(
          lane_path, seg, lane_info, intersection, passage.front_s(),
          passage.end_s(), speed_limit, kIntersectionSpeedReductionRatio);
      if (!cautious_brake_constraint.ok()) {
        continue;
      }
      cautious_brake_constraint->mutable_source()
          ->mutable_intersection()
          ->set_id(id);
      cautious_brake_constraint->set_id(str_id);
      id_element_map[str_id].push_back(std::move(*cautious_brake_constraint));
    }
    // Crosswalk.
    for (const auto &cw_idx : lane_info.crosswalks) {
      const auto &cw = planner_semantic_map_manager.CrosswalkAt(cw_idx.first);
      const int id = cw.id;
      std::string str_id = absl::StrFormat("cautious_brake_crosswalk_%d", id);
      double reduction_ratio = kEmptyCrossWalkSpeedReductionRatio;
      if (IsAnyVruOnCrosswalk(cw, st_mgr)) {
        reduction_ratio = kVRUOnCrossWalkSpeedReductionRatio;
      } else if (IsAnyVruNearCrosswalk(cw, st_mgr)) {
        reduction_ratio = kVRUNearCrossWalkSpeedReductionRatio;
      }
      auto cautious_brake_constraint = ProcessMapElement(
          lane_path, seg, lane_info, cw_idx, passage.front_s(), passage.end_s(),
          speed_limit, reduction_ratio);
      if (!cautious_brake_constraint.ok()) {
        continue;
      }
      cautious_brake_constraint->mutable_source()
          ->mutable_intersection()
          ->set_id(id);
      cautious_brake_constraint->set_id(str_id);
      id_element_map[str_id].push_back(std::move(*cautious_brake_constraint));
    }
  }
  std::vector<ConstraintProto::SpeedRegionProto> cautious_brake_constraints;
  cautious_brake_constraints.reserve(id_element_map.size());
  for (const auto &pair : id_element_map) {
    cautious_brake_constraints.push_back(MergeSameElement(pair.second));
  }
  return cautious_brake_constraints;
}

}  // namespace planner
}  // namespace qcraft
