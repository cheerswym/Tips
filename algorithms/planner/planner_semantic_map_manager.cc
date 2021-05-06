#include "onboard/planner/planner_semantic_map_manager.h"

#include <algorithm>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/planner_flags.h"

namespace qcraft::planner {

PlannerSemanticMapManager::PlannerSemanticMapManager(
    const SemanticMapManager *semantic_map_manager)
    : semantic_map_manager_(semantic_map_manager) {}

PlannerSemanticMapManager::PlannerSemanticMapManager(
    const SemanticMapManager *semantic_map_manager,
    PlannerSemanticMapModification modifier)
    : semantic_map_manager_(semantic_map_manager),
      modifier_(std::move(modifier)) {}

double PlannerSemanticMapManager::QueryLaneSpeedLimitById(
    mapping::ElementId id) const {
  const auto *lane_info_ptr = FindLaneInfoOrNull(id);
  if (lane_info_ptr == nullptr) {
    return modifier_.max_speed_limit;
  }
  const auto &lane_info = *lane_info_ptr;
  const double lane_speed_limit =
      lane_info.speed_limit *
      (1.0 + FLAGS_planner_increase_lane_speed_limit_fraction);
  double speed_limit = std::min(modifier_.max_speed_limit, lane_speed_limit);
  if (const auto it = modifier_.lane_speed_limit_map.find(id);
      it != modifier_.lane_speed_limit_map.end()) {
    speed_limit = std::min(it->second, speed_limit);
  }

  return speed_limit;
}

const PlannerSemanticMapModification &
PlannerSemanticMapManager::GetSemanticMapModifier() const {
  return modifier_;
}

// Other APIs, wrap the original SemanticMapManager APIs.
const SemanticMapManager *PlannerSemanticMapManager::semantic_map_manager()
    const {
  return semantic_map_manager_;
}

const mapping::SemanticMapProto &PlannerSemanticMapManager::semantic_map_proto()
    const {
  return semantic_map_manager_->semantic_map();
}

const CoordinateConverter &PlannerSemanticMapManager::coordinate_converter()
    const {
  return semantic_map_manager_->coordinate_converter();
}

mapping::SemanticLevelId PlannerSemanticMapManager::GetLevel() const {
  return semantic_map_manager_->GetLevel();
}

mapping::SemanticLevelId PlannerSemanticMapManager::InferLevelIdFromNearbyLanes(
    Vec3d smooth_coord) const {
  return semantic_map_manager_->InferLevelIdFromNearbyLanes(smooth_coord);
}

std::vector<const mapping::LaneInfo *>
PlannerSemanticMapManager::GetLanesInfoAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord,
    double radius) const {
  return semantic_map_manager_->GetLanesInfoAtLevel(level_id, smooth_coord,
                                                    radius);
}

const mapping::LaneInfo *
PlannerSemanticMapManager::GetNearestLaneInfoWithHeadingAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord, double theta,
    double radius, double max_heading_diff) const {
  return semantic_map_manager_->GetNearestLaneInfoWithHeadingAtLevel(
      level_id, smooth_coord, theta, radius, max_heading_diff);
}

bool PlannerSemanticMapManager::GetLaneProjectionAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord,
    mapping::ElementId lane_id, double *const fraction, Vec2d *const point,
    double *const min_dist, Segment2d *const segment) const {
  return semantic_map_manager_->GetLaneProjectionAtLevel(
      level_id, smooth_coord, lane_id, fraction, point, min_dist, segment);
}

const mapping::LaneProto &PlannerSemanticMapManager::FindLaneByIdOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindLaneByIdOrDie(id);
}

const mapping::LaneProto *PlannerSemanticMapManager::FindLaneByIdOrNull(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindLaneByIdOrNull(id);
}

double PlannerSemanticMapManager::GetLaneLengthOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->GetLaneLengthOrDie(id);
}

const std::vector<Vec2d> &PlannerSemanticMapManager::GetLaneControlPointsOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->GetLaneControlPointsOrDie(id);
}

const mapping::LaneInfo &PlannerSemanticMapManager::FindLaneInfoOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindLaneInfoOrDie(id);
}

const mapping::LaneInfo *PlannerSemanticMapManager::FindLaneInfoOrNull(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindLaneInfoOrNull(id);
}

const mapping::IntersectionInfo &PlannerSemanticMapManager::IntersectionAt(
    int index) const {
  return semantic_map_manager_->IntersectionAt(index);
}

const mapping::CrosswalkInfo &PlannerSemanticMapManager::CrosswalkAt(
    int index) const {
  return semantic_map_manager_->CrosswalkAt(index);
}

const mapping::LaneVector<mapping::LaneInfo>
    &PlannerSemanticMapManager::lane_info() const {
  return semantic_map_manager_->lane_info();
}

std::vector<const mapping::LaneBoundaryInfo *>
PlannerSemanticMapManager::GetLaneBoundariesInfoAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord,
    double radius) const {
  return semantic_map_manager_->GetLaneBoundariesInfoAtLevel(
      level_id, smooth_coord, radius);
}

std::vector<const mapping::LaneInfo *>
PlannerSemanticMapManager::GetLanesInfoWithHeadingAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord, double theta,
    double radius, double max_heading_diff) const {
  return semantic_map_manager_->GetLanesInfoWithHeadingAtLevel(
      level_id, smooth_coord, theta, radius, max_heading_diff);
}

const mapping::TrafficLightProto &
PlannerSemanticMapManager::FindTrafficLightByIdOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindTrafficLightByIdOrDie(id);
}

const mapping::SectionInfo &PlannerSemanticMapManager::FindSectionInfoOrDie(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindSectionInfoOrDie(id);
}

const std::vector<mapping::SectionInfo>
    &PlannerSemanticMapManager::section_info() const {
  return semantic_map_manager_->section_info();
}

std::vector<Segment2d>
PlannerSemanticMapManager::GetImpassableBoundariesAtLevel(
    mapping::SemanticLevelId level_id, Vec2d smooth_coord,
    double radius) const {
  return semantic_map_manager_->GetImpassableBoundariesAtLevel(
      level_id, smooth_coord, radius);
}

const mapping::ParkingSpotInfo *
PlannerSemanticMapManager::FindParkingSpotByIdOrNull(
    mapping::ElementId id) const {
  return semantic_map_manager_->FindParkingSpotByIdOrNull(id);
}

}  // namespace qcraft::planner
