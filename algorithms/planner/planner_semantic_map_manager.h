#ifndef ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_
#define ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/maps/semantic_map_manager.h"

namespace qcraft::planner {

constexpr double kDefaultMaxSpeedLimit = 50;  // m/s

struct PlannerSemanticMapModification {
  absl::flat_hash_map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = kDefaultMaxSpeedLimit;

  bool IsEmpty() const {
    return max_speed_limit >= kDefaultMaxSpeedLimit &&
           lane_speed_limit_map.empty();
  }
};

class PlannerSemanticMapManager {
 public:
  explicit PlannerSemanticMapManager(
      const SemanticMapManager *semantic_map_manager);

  explicit PlannerSemanticMapManager(
      const SemanticMapManager *semantic_map_manager,
      PlannerSemanticMapModification modifier);

  // Returned value unit is m/s.
  double QueryLaneSpeedLimitById(mapping::ElementId lane_id) const;

  const PlannerSemanticMapModification &GetSemanticMapModifier() const;

  // Should called before GetXXX
  void SetSemanticMapModifier(PlannerSemanticMapModification modifier) {
    modifier_ = std::move(modifier);
  }

  const SemanticMapManager *semantic_map_manager() const;

  // Other APIs, wrap the original SemanticMapManager APIs.
  const mapping::SemanticMapProto &semantic_map_proto() const;

  const CoordinateConverter &coordinate_converter() const;

  mapping::SemanticLevelId GetLevel() const;

  mapping::SemanticLevelId InferLevelIdFromNearbyLanes(
      Vec3d smooth_coord) const;

  // TODO(zixuan): Change function naming following the style guide.
  std::vector<const mapping::LaneInfo *> GetLanesInfoAtLevel(
      mapping::SemanticLevelId level_id, Vec2d smooth_coord,
      double radius) const;

  const mapping::LaneInfo *GetNearestLaneInfoWithHeadingAtLevel(
      mapping::SemanticLevelId level_id, Vec2d smooth_coord, double theta,
      double radius, double max_heading_diff) const;

  bool GetLaneProjectionAtLevel(mapping::SemanticLevelId level_id,
                                Vec2d smooth_coord, mapping::ElementId lane_id,
                                double *const fraction = nullptr,
                                Vec2d *const point = nullptr,
                                double *const min_dist = nullptr,
                                Segment2d *const segment = nullptr) const;

  // TODO(zixuan): Return StatusOr instead of die directly.
  const mapping::LaneProto &FindLaneByIdOrDie(mapping::ElementId id) const;

  const mapping::LaneProto *FindLaneByIdOrNull(mapping::ElementId id) const;

  double GetLaneLengthOrDie(mapping::ElementId id) const;

  const std::vector<Vec2d> &GetLaneControlPointsOrDie(
      mapping::ElementId id) const;

  const mapping::LaneInfo &FindLaneInfoOrDie(mapping::ElementId id) const;

  const mapping::LaneInfo *FindLaneInfoOrNull(mapping::ElementId id) const;

  const mapping::IntersectionInfo &IntersectionAt(int index) const;

  const mapping::CrosswalkInfo &CrosswalkAt(int index) const;

  const mapping::LaneVector<mapping::LaneInfo> &lane_info() const;

  std::vector<const mapping::LaneBoundaryInfo *> GetLaneBoundariesInfoAtLevel(
      mapping::SemanticLevelId level_id, Vec2d smooth_coord,
      double radius) const;

  std::vector<const mapping::LaneInfo *> GetLanesInfoWithHeadingAtLevel(
      mapping::SemanticLevelId level_id, Vec2d smooth_coord, double theta,
      double radius, double max_heading_diff) const;

  const mapping::TrafficLightProto &FindTrafficLightByIdOrDie(
      mapping::ElementId id) const;

  const mapping::TrafficLightProto *FindTrafficLightByIdOrNull(
      mapping::ElementId id) const {
    return semantic_map_manager_->FindTrafficLightByIdOrNull(id);
  }

  const mapping::SectionInfo &FindSectionInfoOrDie(mapping::ElementId id) const;

  // TODO(zixuan): Replace std::vector with absl::Span.
  const std::vector<mapping::SectionInfo> &section_info() const;

  std::vector<Segment2d> GetImpassableBoundariesAtLevel(
      mapping::SemanticLevelId level_id, Vec2d smooth_coord,
      double radius) const;

  const mapping::ParkingSpotInfo *FindParkingSpotByIdOrNull(
      mapping::ElementId id) const;

 private:
  const SemanticMapManager *semantic_map_manager_;
  // Modifier contains speed limits, unit is m/s.
  PlannerSemanticMapModification modifier_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_SEMANTIC_MAP_MANAGER_H_
