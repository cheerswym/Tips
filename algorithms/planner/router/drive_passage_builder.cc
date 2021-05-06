#include "onboard/planner/router/drive_passage_builder.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft::planner {

namespace {

constexpr double kVehicleWidth = 2.02;
constexpr double kDrivePassageCutOffAngleDiff = 1.1 * M_PI;
constexpr double kDrivePassageMaxForwardExtendLength = 10.0;  // m.

constexpr double kFarStationHorizonRatio = 1.0 - 0.3;
constexpr double kFarStationStepRatio = 2.0;

StationBoundaryType MapBoundaryTypeToStationBoundaryType(
    mapping::LaneBoundaryProto::Type map_type) {
  switch (map_type) {
    case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
      return StationBoundaryType::UNKNOWN_TYPE;
    case mapping::LaneBoundaryProto::BROKEN_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
      return StationBoundaryType::BROKEN_WHITE;
    case mapping::LaneBoundaryProto::SOLID_WHITE:
      return StationBoundaryType::SOLID_WHITE;
    case mapping::LaneBoundaryProto::BROKEN_YELLOW:
      return StationBoundaryType::BROKEN_YELLOW;
    case mapping::LaneBoundaryProto::SOLID_YELLOW:
      return StationBoundaryType::SOLID_YELLOW;
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
      return StationBoundaryType::SOLID_DOUBLE_YELLOW;
    case mapping::LaneBoundaryProto::CURB:
      return StationBoundaryType::CURB;
  }
}

// Returns true if type1 is strictly lower than type2.
bool LowerType(StationBoundaryType type1, StationBoundaryType type2) {
  switch (type1) {
    case StationBoundaryType::UNKNOWN_TYPE:
      return true;
    case StationBoundaryType::BROKEN_WHITE:
      return type2 == StationBoundaryType::SOLID_WHITE ||
             type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_WHITE:
      return type2 == StationBoundaryType::BROKEN_YELLOW ||
             type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::BROKEN_YELLOW:
      return type2 == StationBoundaryType::SOLID_YELLOW ||
             type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_YELLOW:
      return type2 == StationBoundaryType::SOLID_DOUBLE_YELLOW ||
             type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      return type2 == StationBoundaryType::CURB ||
             type2 == StationBoundaryType::VIRTUAL_CURB;
    case StationBoundaryType::CURB:
    case StationBoundaryType::VIRTUAL_CURB:
      return false;
  }
}

std::pair<double, double> ComputeLateralLimits(
    const std::vector<StationCenter>& centers,
    const StationCenter& current_center) {
  if (centers.empty()) return {-kMaxLateralOffset, kMaxLateralOffset};

  const double d_theta = NormalizeAngle(current_center.tangent.FastAngle() -
                                        centers.back().tangent.FastAngle());
  const double ds = current_center.accum_s - centers.back().accum_s;
  if (std::abs(d_theta) * kMaxLateralOffset < ds)
    return {-kMaxLateralOffset, kMaxLateralOffset};

  constexpr double kVehicleMinTurnRadius = 5.05;  // m.
  const double turn_radius =
      std::max(kVehicleMinTurnRadius, ds / std::abs(d_theta));

  if (d_theta < 0.0) {
    // Turn right
    return {-turn_radius, kMaxLateralOffset};
  }
  // Turn left
  return {-kMaxLateralOffset, turn_radius};
}

std::vector<StationBoundary> CollectStationBoundaries(
    const SemanticMapManager& smm, const StationCenter& center,
    double max_right_offset, double max_left_offset) {
  const Segment2d normal_line(center.lat_point(max_right_offset),
                              center.lat_point(max_left_offset));

  const auto candidate_boundaries = smm.GetLaneBoundariesInfoAtLevel(
      smm.GetLevel(), center.xy, kMaxLateralOffset);
  std::vector<StationBoundary> station_boundaries;
  bool has_left_curb = false;
  bool has_right_curb = false;
  for (const auto& boundary : candidate_boundaries) {
    // Virtual lane, only keep curbs.
    if ((center.is_virtual || center.is_merging) &&
        boundary->type != mapping::LaneBoundaryProto::CURB) {
      continue;
    }
    // Compute cross point of boundary and normal line
    Vec2d closest_intersection;
    double min_sqr_dis = std::numeric_limits<double>::max();
    for (int k = 0; k + 1 < boundary->points_smooth.size(); ++k) {
      const Vec2d& p0 = boundary->points_smooth[k];
      const Vec2d& p1 = boundary->points_smooth[k + 1];
      const Segment2d boundary_segment(p0, p1);

      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = center.xy.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_intersection = intersection;
      }
    }
    if (min_sqr_dis < std::numeric_limits<double>::max()) {
      station_boundaries.push_back(
          {MapBoundaryTypeToStationBoundaryType(boundary->type),
           center.lat_offset(closest_intersection)});
      if (station_boundaries.back().type == StationBoundaryType::CURB) {
        station_boundaries.back().lat_offset > 0 ? has_left_curb = true
                                                 : has_right_curb = true;
      }
    }
  }
  if (!has_left_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_left_offset});
  if (!has_right_curb)
    station_boundaries.push_back(
        {StationBoundaryType::VIRTUAL_CURB, max_right_offset});

  return station_boundaries;
}

void PostProcessStationBoundaries(
    const StationCenter& center,
    std::vector<StationBoundary>* mutable_boundaries) {
  std::sort(mutable_boundaries->begin(), mutable_boundaries->end(),
            [](const StationBoundary& lhs, const StationBoundary& rhs) {
              return lhs.lat_offset < rhs.lat_offset;
            });
  std::vector<StationBoundary> remaining_boundaries;

  auto current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (auto it = mutable_boundaries->rbegin(); it != mutable_boundaries->rend();
       ++it) {
    // From center to right.
    if (it->lat_offset > 0.0 || LowerType(it->type, current_top_type)) {
      continue;
    }
    remaining_boundaries.push_back(*it);
    if (it->type == StationBoundaryType::CURB ||
        it->type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = it->type;
  }
  std::reverse(remaining_boundaries.begin(), remaining_boundaries.end());

  current_top_type = StationBoundaryType::UNKNOWN_TYPE;
  for (const auto& bound : *mutable_boundaries) {
    // From center to left.
    if (bound.lat_offset < 0.0 || LowerType(bound.type, current_top_type)) {
      continue;
    }

    remaining_boundaries.push_back(bound);
    if (bound.type == StationBoundaryType::CURB ||
        bound.type == StationBoundaryType::VIRTUAL_CURB) {
      break;
    }
    current_top_type = bound.type;
  }

  // We can only borrow one lane at most if crossed yellow lane boundary.
  const double borrow_road_width = kDefaultLaneWidth + kVehicleWidth * 0.5;
  double left_solid_yellow_line_offset = std::numeric_limits<double>::max();
  double right_solid_yellow_line_offset = std::numeric_limits<double>::lowest();
  for (const auto& bound : remaining_boundaries) {
    if (bound.type == StationBoundaryType::SOLID_YELLOW ||
        bound.type == StationBoundaryType::SOLID_DOUBLE_YELLOW) {
      bound.lat_offset < 0.0 ? right_solid_yellow_line_offset = bound.lat_offset
                             : left_solid_yellow_line_offset = bound.lat_offset;
    }
  }
  if (left_solid_yellow_line_offset + borrow_road_width <
      remaining_boundaries.back().lat_offset) {
    remaining_boundaries.back().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.back().lat_offset =
        left_solid_yellow_line_offset + borrow_road_width;
  }
  if (right_solid_yellow_line_offset - borrow_road_width >
      remaining_boundaries.front().lat_offset) {
    remaining_boundaries.front().type = StationBoundaryType::VIRTUAL_CURB;
    remaining_boundaries.front().lat_offset =
        right_solid_yellow_line_offset - borrow_road_width;
  }

  *mutable_boundaries = std::move(remaining_boundaries);
}

struct DrivePassageData {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
};
DrivePassageData SampleLanePathWithSemanticMapMgr(
    const SemanticMapManager& smm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step, bool avoid_loop) {
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n = CeilToInt((end_s - start_s) / step);
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  for (double sample_s = start_s; sample_s <= end_s; sample_s += step) {
    const auto sample_lane_point = lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle = sample_lane_point.ComputeTangent(smm).FastAngle();
    } else {
      const auto cur_station_angle =
          sample_lane_point.ComputeTangent(smm).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if ((max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
         min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) &&
        avoid_loop) {
      break;
    }
    station_idx++;

    const auto& lane_info = smm.FindLaneInfoOrDie(sample_lane_point.lane_id());
    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = sample_lane_point.ComputePos(smm),
        .tangent = sample_lane_point.ComputeTangent(smm),
        .accum_s = sample_s,
        .speed_limit = smm.QueryLaneSpeedLimitById(sample_lane_point.lane_id()),
        .is_virtual = lane_info.IsVirtual(),
        .is_merging = lane_info.proto->is_merging(),
        .is_in_intersection = lane_info.is_in_intersection,
        .direction = lane_info.direction};

    // Station boundaries:
    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    auto current_station_boundaries = CollectStationBoundaries(
        smm, center, right_lat_offset, left_lat_offset);
    PostProcessStationBoundaries(center, &current_station_boundaries);

    centers.emplace_back(std::move(center));
    stations_boundaries.emplace_back(std::move(current_station_boundaries));
  }
  return DrivePassageData{
      .centers = std::move(centers),
      .stations_boundaries = std::move(stations_boundaries)};
}
}  // namespace

DrivePassage BuildDrivePassageFromLanePath(const SemanticMapManager& smm,
                                           const mapping::LanePath& lane_path,
                                           double step_s, bool avoid_loop) {
  auto dp_data = SampleLanePathWithSemanticMapMgr(
      smm, lane_path, 0.0, lane_path.length(), step_s, avoid_loop);
  // Create station vector.
  QCHECK_EQ(dp_data.centers.size(), dp_data.stations_boundaries.size());
  StationVector<Station> stations;
  stations.reserve(dp_data.centers.size());
  for (int i = 0; i < dp_data.centers.size(); ++i) {
    stations.emplace_back(std::move(dp_data.centers[i]),
                          std::move(dp_data.stations_boundaries[i]));
  }

  return DrivePassage(nullptr, std::move(stations), lane_path, lane_path,
                      /*lane_path_start_s=*/0.0);
}

absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    double keep_behind_len, bool all_lanes_virtual) {
  SCOPED_QTRACE("BuildDrivePassage");
  const auto& smm = *psmm.semantic_map_manager();
  auto lane_path_in_horizon =
      lane_path_from_pose.BeforeArclength(planning_horizon);

  // Forward and backward extend lane path. Forward for projection of objects
  // that are slightly beyond the current lane path's end, and backward for plan
  // start point projection of the next frame.
  const double forward_extend_len =
      std::min(planning_horizon - lane_path_from_pose.length(),
               kDrivePassageMaxForwardExtendLength);
  const auto forward_extended_lane_path =
      ForwardExtendLanePath(smm, lane_path_in_horizon, forward_extend_len);
  // TODO(weijun): Record past route lane path instead of random extension.
  auto ref_lane_path =
      BackwardExtendLanePath(smm, forward_extended_lane_path, keep_behind_len);

  const double ref_ego_s = ref_lane_path.FirstOccurrenceOfLanePointToArclength(
      lane_path_from_pose.front());
  const double ref_anchor_s =
      ref_lane_path.ContainsLanePoint(anchor_point)
          ? ref_lane_path.FirstOccurrenceOfLanePointToArclength(anchor_point)
          : ref_ego_s;
  const double ref_neutral_s =
      ref_anchor_s +
      RoundToInt((ref_ego_s - ref_anchor_s) / kRouteStationUnitStep) *
          kRouteStationUnitStep;
  const double start_station_accum_s =
      -FloorToInt(ref_neutral_s / kRouteStationUnitStep) *
      kRouteStationUnitStep;
  const double start_station_ref_s = ref_neutral_s + start_station_accum_s;

  const double ref_path_len = ref_lane_path.length();
  std::vector<StationCenter> centers;
  std::vector<std::vector<StationBoundary>> stations_boundaries;
  const int n =
      CeilToInt((ref_path_len - start_station_ref_s) / kRouteStationUnitStep);
  centers.reserve(n);
  stations_boundaries.reserve(n);

  int station_idx = 0;
  double prev_station_angle = 0.0;
  double max_accumulated_angle_diff = 0.0;
  double min_accumulated_angle_diff = 0.0;

  const double far_station_thres =
      kFarStationHorizonRatio * planning_horizon + ref_ego_s;
  for (double sample_s = start_station_ref_s; sample_s <= ref_path_len;) {
    const auto sample_lane_point = ref_lane_path.ArclengthToLanePoint(sample_s);

    // Ensure we do not create a loop in drive passage. Planner does not support
    // looped passage.
    if (station_idx == 0) {
      prev_station_angle = sample_lane_point.ComputeTangent(smm).FastAngle();
    } else {
      const auto cur_station_angle =
          sample_lane_point.ComputeTangent(smm).FastAngle();
      const double angle_diff =
          NormalizeAngle(cur_station_angle - prev_station_angle);
      max_accumulated_angle_diff =
          std::max(max_accumulated_angle_diff + angle_diff, angle_diff);
      min_accumulated_angle_diff =
          std::min(min_accumulated_angle_diff + angle_diff, angle_diff);
      prev_station_angle = cur_station_angle;
    }
    if (max_accumulated_angle_diff >= kDrivePassageCutOffAngleDiff ||
        min_accumulated_angle_diff <= -kDrivePassageCutOffAngleDiff) {
      break;
    }
    station_idx++;

    const auto& lane_info = psmm.FindLaneInfoOrDie(sample_lane_point.lane_id());
    // Station center:
    StationCenter center{
        .lane_id = sample_lane_point.lane_id(),
        .fraction = sample_lane_point.fraction(),
        .xy = sample_lane_point.ComputePos(smm),
        .tangent = sample_lane_point.ComputeTangent(smm),
        .accum_s = sample_s - ref_neutral_s,
        .speed_limit =
            psmm.QueryLaneSpeedLimitById(sample_lane_point.lane_id()),
        .is_virtual = (all_lanes_virtual || lane_info.IsVirtual()),
        .is_merging = lane_info.proto->is_merging(),
        .is_in_intersection = lane_info.is_in_intersection,
        .direction = lane_info.direction};

    // Station boundaries:
    const auto [right_lat_offset, left_lat_offset] =
        ComputeLateralLimits(centers, center);
    auto current_station_boundaries = CollectStationBoundaries(
        smm, center, right_lat_offset, left_lat_offset);
    PostProcessStationBoundaries(center, &current_station_boundaries);

    centers.emplace_back(std::move(center));
    stations_boundaries.emplace_back(std::move(current_station_boundaries));

    sample_s += sample_s >= far_station_thres
                    ? kFarStationStepRatio * kRouteStationUnitStep
                    : kRouteStationUnitStep;
  }

  // Create station vector.
  QCHECK_EQ(centers.size(), stations_boundaries.size());
  StationVector<Station> stations;
  stations.reserve(centers.size());
  for (int i = 0; i < centers.size(); ++i) {
    stations.emplace_back(std::move(centers[i]),
                          std::move(stations_boundaries[i]));
  }

  return DrivePassage(&psmm, std::move(stations),
                      std::move(lane_path_in_horizon), std::move(ref_lane_path),
                      /*lane_path_start_s=*/ref_ego_s - ref_neutral_s);
}

}  // namespace qcraft::planner
