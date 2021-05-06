#include "onboard/planner/freespace/freespace_constraint_builder.h"

#include <algorithm>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

DEFINE_bool(
    use_parking_spot_line_soft_constraints, true,
    "Whether to add cost to trajectory when AV drives on parking spot edge.");

namespace qcraft {
namespace planner {
namespace {

bool IsMapRegionContainsAv(const VehicleGeometryParamsProto &vehicle_geom,
                           const PoseProto &ego_pose,
                           const AABox2d &map_aabox) {
  if (ego_pose.pos_smooth().x() < map_aabox.min_x() ||
      ego_pose.pos_smooth().x() > map_aabox.max_x() ||
      ego_pose.pos_smooth().y() < map_aabox.min_y() ||
      ego_pose.pos_smooth().y() > map_aabox.max_y()) {
    return false;
  }
  return true;
}

absl::StatusOr<ConstraintProto::StopLineProto> AddEndOfLocalPathConstraint(
    const VehicleGeometryParamsProto &veh_geo_params,
    const DirectionalPath &path) {
  if (path.path.empty()) {
    return absl::InternalError("Local path empty.");
  }
  const auto &last_path_point = path.path.back();
  const auto unit = Vec2d::FastUnitFromAngle(last_path_point.theta());
  const auto perp = unit.Perp();
  const Vec2d pos(last_path_point.x(), last_path_point.y());
  const Vec2d center =
      pos + unit * (path.forward ? veh_geo_params.front_edge_to_center()
                                 : veh_geo_params.back_edge_to_center());

  constexpr double kHalfPlaneHalfWidth = 3.0;  // m.
  const HalfPlane halfplane(center - perp * kHalfPlaneHalfWidth,
                            center + perp * kHalfPlaneHalfWidth);

  ConstraintProto::StopLineProto stop_line;
  // For end_of_local_path stop line, its s means the path s for rac.
  stop_line.set_s(last_path_point.s());
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id("end_of_local_path");
  stop_line.mutable_source()->mutable_end_of_local_path()->set_reason(
      "End of local path.");
  return stop_line;
}

bool IsLaneBoundaryNearParkingSpot(
    const mapping::ParkingSpotInfo *parking_spot_info,
    const Segment2d &segment) {
  constexpr double kNearSegmentsDist = 0.3;
  if (parking_spot_info == nullptr) return false;
  const Polygon2d &spot = parking_spot_info->polygon();
  if (spot.min_x() > segment.max_x() + kNearSegmentsDist ||
      spot.min_y() > segment.max_y() + kNearSegmentsDist ||
      spot.max_x() < segment.min_x() - kNearSegmentsDist ||
      spot.max_y() < segment.min_y() - kNearSegmentsDist) {
    return false;
  }
  return spot.DistanceTo(segment) < kNearSegmentsDist;
}

}  // namespace

absl::StatusOr<FreespaceMap> ConstructFreespaceMap(
    FreespaceTaskProto::TaskType task_type,
    const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerSemanticMapManager &psmm, const PoseProto &ego_pose,
    const mapping::ParkingSpotInfo *parking_spot_info, const PathPoint &goal) {
  SCOPED_QTRACE("ConstructParkingFreespaceMap");
  FreespaceMap freespace_map;
  using mapping::ParkingSpotInfo;
  // Construct freespace region.
  constexpr double kFreespaceHalfWidthForParking = 20.0;  // m.
  constexpr double kFreespaceHalfWidthForDriving = 50.0;  // m.
  double freespace_half_width = 0.0;
  if (parking_spot_info == nullptr) {
    freespace_half_width = kFreespaceHalfWidthForDriving;
  } else {
    freespace_half_width = kFreespaceHalfWidthForParking;
  }

  const auto goal_pos = ToVec2d(goal);
  freespace_map.region =
      AABox2d(freespace_half_width, freespace_half_width, goal_pos);
  if (!IsMapRegionContainsAv(vehicle_geom, ego_pose, freespace_map.region)) {
    // TODO(yumeng): Consider extend region.
    const double dist_to_goal = goal_pos.DistanceTo(
        Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));
    return absl::InternalError(
        absl::StrFormat("Too Far away from goal: %f, %f, distance: %f",
                        goal_pos.x(), goal_pos.y(), dist_to_goal));
  }

  const double radius = Hypot(freespace_half_width, freespace_half_width);
  // Add map boundaries.
  const std::vector<const mapping::LaneBoundaryInfo *> boundaries_info =
      psmm.GetLaneBoundariesInfoAtLevel(psmm.GetLevel(), goal_pos, radius);
  // Assumes a parking spot provides at most three boundaries.
  freespace_map.boundaries.reserve(boundaries_info.size() + 3);
  const Box2d region_box(
      freespace_map.region.half_length() + vehicle_geom.length(),
      freespace_map.region.half_width() + vehicle_geom.length(),
      freespace_map.region.center(), /*heading=*/0.0);
  int boundary_index = 1;
  // Add parking spot boundaries.
  if (parking_spot_info != nullptr) {
    for (const auto side : {ParkingSpotInfo::LEFT, ParkingSpotInfo::REAR,
                            ParkingSpotInfo::RIGHT, ParkingSpotInfo::FRONT}) {
      if (parking_spot_info->IsSideEnterable(side)) continue;
      Segment2d segment = parking_spot_info->GetEdge(side);
      // Ignore part of spot line of parallel parking.
      if (task_type == FreespaceTaskProto::PARALLEL_PARKING &&
          (side == ParkingSpotInfo::FRONT || side == ParkingSpotInfo::REAR)) {
        // Get enter side.
        Segment2d enter_side_segment;
        if (parking_spot_info->IsSideEnterable(ParkingSpotInfo::LEFT)) {
          enter_side_segment =
              parking_spot_info->GetEdge(ParkingSpotInfo::LEFT);
        } else {
          enter_side_segment =
              parking_spot_info->GetEdge(ParkingSpotInfo::RIGHT);
        }
        // Get new spot line.
        constexpr double kEpsilon = 1e-6;
        constexpr double kIgnoreSpotLineRatio = 0.35;
        auto start = segment.start();
        auto end = segment.end();
        double sign = 1.0;
        if (enter_side_segment.DistanceTo(start) > kEpsilon) {
          std::swap(start, end);
          sign = -1.0;
        }
        const auto new_start = start + kIgnoreSpotLineRatio * sign *
                                           segment.length() *
                                           segment.unit_direction();
        segment = Segment2d(new_start, end);
      }
      // If enable FLAG_use_parking_spot_line_soft_constraints, which means we
      // treat slot egdes as soft constraints, push it to special
      // boundaries. Otherwise, do it to freespace boundaries.
      if (FLAGS_use_parking_spot_line_soft_constraints) {
        freespace_map.special_boundaries.push_back(
            {.id = "s" + std::to_string(boundary_index),
             .segment = std::move(segment),
             .type = SpecialBoundaryType::SOFT_PARKING_SPOT_LINE});
      } else {
        freespace_map.boundaries.push_back(
            {.id = "b" + std::to_string(boundary_index),
             .type = FreespaceMapProto::PARKING_SPOT,
             .segment = std::move(segment),
             .near_parking_spot = false});
      }
      ++boundary_index;
    }
  }

  const auto add_map_boundary =
      [&freespace_map, &parking_spot_info,
       &boundary_index = std::as_const(boundary_index)](
          const Segment2d &segment,
          const FreespaceMapProto::BoundaryType &type) {
        freespace_map.boundaries.push_back(
            {.id = "b" + std::to_string(boundary_index),
             .type = type,
             .segment = segment,
             .near_parking_spot =
                 IsLaneBoundaryNearParkingSpot(parking_spot_info, segment)});
      };

  const auto add_map_special_boundary =
      [&freespace_map, &boundary_index = std::as_const(boundary_index)](
          const Segment2d &segment, const SpecialBoundaryType &type) {
        freespace_map.special_boundaries.push_back(
            {.id = "d" + std::to_string(boundary_index),
             .segment = segment,
             .type = type});
      };

  // Add map boundaries.
  for (const mapping::LaneBoundaryInfo *lane_boundary_info : boundaries_info) {
    for (int i = 0; i < lane_boundary_info->points_smooth.size() - 1; ++i) {
      Segment2d boundary(lane_boundary_info->points_smooth[i],
                         lane_boundary_info->points_smooth[i + 1]);
      // Filter boundaries not in map region.
      if (!region_box.HasOverlap(boundary)) {
        continue;
      }
      switch (lane_boundary_info->type) {
        case mapping::LaneBoundaryProto::CURB:
          add_map_boundary(boundary, FreespaceMapProto::CURB);
          break;
        case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
        case mapping::LaneBoundaryProto::SOLID_YELLOW:
          if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
              task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
            add_map_boundary(boundary, FreespaceMapProto::YELLOW_SOLID_LANE);
          }
          break;
        case mapping::LaneBoundaryProto::BROKEN_YELLOW:
          if (task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
            add_map_boundary(boundary, FreespaceMapProto::YELLOW_DASHED_LANE);
          }
          if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
              task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
            add_map_special_boundary(boundary,
                                     SpecialBoundaryType::CROSSABLE_LANE_LINE);
          }
          break;
        case mapping::LaneBoundaryProto::SOLID_WHITE:
          if (task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
            add_map_boundary(boundary, FreespaceMapProto::WHITE_SOLID_LANE);
          } else if (task_type == FreespaceTaskProto::THREE_POINT_TURN) {
            add_map_special_boundary(boundary,
                                     SpecialBoundaryType::CROSSABLE_LANE_LINE);
          }
          break;
        case mapping::LaneBoundaryProto::BROKEN_WHITE:
        // TODO(luzou, renjie): for double white, with one of the sides being
        // broken, we treat it as passable. This can be the case where we are
        // doing U-Turn
        case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
        case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
          if (task_type == FreespaceTaskProto::THREE_POINT_TURN ||
              task_type == FreespaceTaskProto::DRIVING_TO_LANE) {
            add_map_special_boundary(boundary,
                                     SpecialBoundaryType::CROSSABLE_LANE_LINE);
          }
          break;
        case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
          add_map_boundary(boundary, FreespaceMapProto::OTHER);
          break;
      }
      ++boundary_index;
    }
  }
  return freespace_map;
}

void AddUTurnBoundary(const PlannerSemanticMapManager &psmm,
                      const mapping::LanePath *lane_path,
                      const VehicleGeometryParamsProto &vehicle_geom,
                      FreespaceMap *freespace_map) {
  // Judge left U-turn or right U-turn, currently no right U-turn in China.
  const Vec2d start_point =
      lane_path->front().ComputePos(*psmm.semantic_map_manager());
  const Vec2d start_dir =
      lane_path->front().ComputeTangent(*psmm.semantic_map_manager());
  const Vec2d end_point =
      lane_path->back().ComputePos(*psmm.semantic_map_manager());
  const bool left_turn = (start_dir.CrossProd(end_point - start_point) > 0.0);

  constexpr double kNeighborLaneBoundaryLength = 40.0;
  constexpr double kDefaultLaneWidth = 3.75;
  // Add boundary of neighbor lane and ego lane.
  double left_width = 0.0;
  double right_width = 0.0;
  const auto level = psmm.semantic_map_manager()->GetLevel();
  if (psmm.semantic_map_manager()->GetLaneWidthAtLevel(
          level, start_point, &left_width, &right_width)) {
    left_width = std::min(kDefaultLaneWidth * 0.5, left_width);
    right_width = -std::max(kDefaultLaneWidth * 0.5, right_width);
    if (!left_turn) std::swap(left_width, right_width);
    // Add boundary of neightbor lane.
    const Vec2d start_normal = start_dir.Perp();
    const Vec2d neigbor_boundary_start =
        start_point + right_width * start_normal;
    const Vec2d neigbor_boundary_end =
        neigbor_boundary_start + kNeighborLaneBoundaryLength * start_dir;
    freespace_map->boundaries.push_back(
        {.id = "uturn_neighbor_boundary",
         .type = FreespaceMapProto::VIRTUAL,
         .segment = Segment2d(neigbor_boundary_start, neigbor_boundary_end),
         .near_parking_spot = false});
    // Add boundary of ego lane.
    const Vec2d ego_boundary_start = start_point + left_width * start_normal;
    const Vec2d ego_boundary_end =
        ego_boundary_start + kNeighborLaneBoundaryLength * start_dir;
    freespace_map->special_boundaries.push_back(
        {.id = "uturn_reverse_stopper",
         .segment = Segment2d(ego_boundary_start, ego_boundary_end),
         .type = SpecialBoundaryType::GEAR_REVERSE_STOPPER});
  }
  return;
}

absl::StatusOr<ConstraintManager> BuildFreespacePlannerConstraint(
    const VehicleGeometryParamsProto &veh_geo_params,
    const DirectionalPath &path) {
  ConstraintManager constraint_manager;
  ASSIGN_OR_RETURN(auto end_of_path_constraint,
                   AddEndOfLocalPathConstraint(veh_geo_params, path));
  constraint_manager.AddStopLine(std::move(end_of_path_constraint));

  return constraint_manager;
}

}  // namespace planner
}  // namespace qcraft
