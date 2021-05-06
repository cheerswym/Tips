#include "onboard/planner/speed/speed_limit_generator.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
using Sfp = SpeedFinderParamsProto;
using SpeedLimitProto = Sfp::SpeedLimitParamsProto;
using SpeedLimitType = Sfp::SpeedLimitType;
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
constexpr double kEps = 1e-6;
// Speed limit diff less than this value will be ignored.
constexpr double kApproxSpeedLimitEps = 1.0;

double GetCurvatureSpeedLimitByPathPoints(
    const DiscretizedPath& discretized_points, int point_idx,
    double max_allowed_kappa, double max_speed_limit,
    const SpeedLimitProto& speed_limit_config) {
  // TODO(ping): Optimize this function. There is some redundant computation.
  QCHECK_LT(point_idx, discretized_points.size());

  const auto& path_point = discretized_points[point_idx];
  double max_kappa = std::fabs(path_point.kappa());

  const double start_s =
      path_point.s() - speed_limit_config.max_curvature_consider_radius();

  const double end_s =
      path_point.s() + speed_limit_config.max_curvature_consider_radius();

  for (int i = point_idx - 1; i >= 0 && i < discretized_points.size(); --i) {
    const auto& p = discretized_points[i];
    if (p.s() < start_s) break;
    max_kappa = std::max(max_kappa, std::fabs(p.kappa()));
  }

  for (int i = point_idx + 1; i >= 0 && i < discretized_points.size(); ++i) {
    const auto& p = discretized_points[i];
    if (p.s() > end_s) break;
    max_kappa = std::max(max_kappa, std::fabs(p.kappa()));
  }

  double speed_limit =
      speed_limit_config.curvature_bias() +
      1.0 / (max_kappa + kEps) * speed_limit_config.curvature_gain();

  if (max_kappa > max_allowed_kappa) {
    speed_limit = std::min(
        speed_limit, speed_limit_config.speed_limit_for_large_curvature());
  }

  return std::min(speed_limit, max_speed_limit);
}

SpeedLimit GenerateCurvatureSpeedLimit(
    const DiscretizedPath& path_points,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    const SpeedLimitProto& speed_limit_config) {
  const double max_allowed_kappa =
      GetCenterMaxCurvature(veh_geo_params, veh_drive_params);
  std::vector<SpeedLimitRange> speed_limit_ranges;
  const double init_limit = GetCurvatureSpeedLimitByPathPoints(
      path_points, /*point_idx=*/0, max_allowed_kappa, max_speed_limit,
      speed_limit_config);
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), init_limit);
  for (int i = 1; i < path_points.size(); ++i) {
    const double curr_speed_limit = GetCurvatureSpeedLimitByPathPoints(
        path_points, i, max_allowed_kappa, max_speed_limit, speed_limit_config);
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = Sfp::SpeedLimitType_Name(Sfp::CURVATURE)});
      prev_speed_limit_point =
          std::make_pair(path_points[i].s(), curr_speed_limit);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges, max_speed_limit);
}

SpeedLimit GenerateLaneSpeedLimit(const DiscretizedPath& path_points,
                                  double max_speed_limit,
                                  const DrivePassage& drive_passage) {
  const auto get_speed_limit = [&drive_passage,
                                max_speed_limit](const PathPoint& path_point) {
    const auto speed_limit =
        drive_passage.QuerySpeedLimitAt(ToVec2d(path_point));
    return speed_limit.ok() ? *speed_limit : max_speed_limit;
  };

  std::vector<SpeedLimitRange> speed_limit_ranges;
  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  for (int i = 1; i < path_points.size(); ++i) {
    const double curr_speed_limit = get_speed_limit(path_points[i]);
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path_points.size() - 1) {
      QCHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = Sfp::SpeedLimitType_Name(Sfp::LANE)});
      prev_speed_limit_point =
          std::make_pair(path_points[i].s(), curr_speed_limit);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges, max_speed_limit);
}

std::string TypeCaseToString(const SourceProto::TypeCase type) {
  switch (type) {
    case SourceProto::TypeCase::kCloseObject:
      return "CLOSE_OBJECT";
    case SourceProto::TypeCase::kSpeedBump:
      return "SPEED_BUMP";
    case SourceProto::TypeCase::kIntersection:
      return "INTERSECTION";
    case SourceProto::TypeCase::kLcEndOfCurrentLane:
      return "LC_END_OF_CURRENT_LANE";
    case SourceProto::TypeCase::kBeyondLengthAlongRoute:
      return "BEYOND_LENGTH_ALONE_ROUTE";
    case SourceProto::TypeCase::kPedestrianObject:
      return "PEDESTRIAN_OBJECT";
    case SourceProto::TypeCase::kCrosswalk:
      return "CROSSWALK";
    case SourceProto::TypeCase::kToll:
      return "TOLL";
    case SourceProto::TypeCase::kTrafficLight:
      return "TRAFFIC_LIGHT";
    case SourceProto::TypeCase::kNoBlock:
      return "NO_BLOCK";
    case SourceProto::TypeCase::kEndOfPathBoundary:
      return "END_OF_PATH_BOUNDARY";
    case SourceProto::TypeCase::kEndOfCurrentLanePath:
      return "END_OF_CURRENT_LANE_PATH";
    case SourceProto::TypeCase::kParkingBrakeRelease:
      return "PARKING_BRAKERELEASE";
    case SourceProto::TypeCase::kBlockingStaticObject:
      return "BLOCKING_STATIC_OBJECT";
    case SourceProto::TypeCase::kStandby:
      return "STANDBY";
    case SourceProto::TypeCase::kStopSign:
      return "STOP_SIGN";
    case SourceProto::TypeCase::kStandstill:
      return "STANDSTILL";
    case SourceProto::TypeCase::kPullOver:
      return "PULL_OVER";
    case SourceProto::TypeCase::kBrakeToStop:
      return "BRAKE_TO_STOP";
    case SourceProto::TypeCase::kEndOfLocalPath:
      return "END_OF_LOCAL_PATH";
    case SourceProto::TypeCase::TYPE_NOT_SET:
      return "TYPE_NOT_SET";
  }
}

std::optional<SpeedLimit> GenerateExternalSpeedLimit(
    const DiscretizedPath& path_points, const ConstraintManager& constraint_mgr,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& path_point : path_points) {
    points.emplace_back(path_point.x(), path_point.y());
  }
  ASSIGN_OR_DIE(const auto frenet_path, BuildBruteForceFrenetFrame(points));
  constexpr double kSpeedRegionBuffer = 0.5;
  std::vector<SpeedLimitRange> speed_limit_ranges;
  for (const ConstraintProto::SpeedRegionProto& speed_region :
       constraint_mgr.SpeedRegion()) {
    const auto type_case = speed_region.source().type_case();
    // Only consider upper bound speed limit.
    if (type_case == SourceProto::TypeCase::kNoBlock) continue;
    const double start_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.start_point())).s -
        veh_geo_params.front_edge_to_center() - kSpeedRegionBuffer;
    const double end_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.end_point())).s +
        veh_geo_params.back_edge_to_center() + kSpeedRegionBuffer;
    if (start_s >= end_s) continue;
    QCHECK_GT(end_s, start_s);
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(speed_region.max_speed(), max_speed_limit),
         .info = TypeCaseToString(speed_region.source().type_case())});
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges, max_speed_limit);
}

SpeedLimit GenerateCombinationSpeedLimit(
    const absl::flat_hash_map<SpeedLimitType, SpeedLimit>& speed_limit_map,
    double max_speed_limit) {
  std::vector<SpeedLimitRange> speed_limit_ranges;
  for (const auto& [_, speed_limit] : speed_limit_map) {
    for (const auto& range : speed_limit.merged_ranges()) {
      speed_limit_ranges.push_back(range);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges, max_speed_limit);
}

}  // namespace

absl::flat_hash_map<SpeedLimitType, SpeedLimit> GetSpeedLimitMap(
    const DiscretizedPath& discretized_points, double max_speed_limit,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage& drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedLimitProto& speed_limit_config) {
  SCOPED_QTRACE("GetSpeedLimitMap");

  absl::flat_hash_map<SpeedLimitType, SpeedLimit> speed_limit_map;
  SpeedLimit lane_speed_limit = GenerateLaneSpeedLimit(
      discretized_points, max_speed_limit, drive_passage);
  speed_limit_map.emplace(Sfp::LANE, std::move(lane_speed_limit));

  SpeedLimit curvature_speed_limit = GenerateCurvatureSpeedLimit(
      discretized_points, veh_drive_params, veh_geo_params, max_speed_limit,
      speed_limit_config);
  speed_limit_map.emplace(Sfp::CURVATURE, std::move(curvature_speed_limit));

  const auto external_speed_limit = GenerateExternalSpeedLimit(
      discretized_points, constraint_mgr, veh_geo_params, max_speed_limit);
  if (external_speed_limit.has_value()) {
    speed_limit_map.emplace(Sfp::EXTERNAL, std::move(*external_speed_limit));
  }

  SpeedLimit combination_speed_limit =
      GenerateCombinationSpeedLimit(speed_limit_map, max_speed_limit);
  speed_limit_map.emplace(Sfp::COMBINATION, std::move(combination_speed_limit));

  return speed_limit_map;
}

}  // namespace qcraft::planner
