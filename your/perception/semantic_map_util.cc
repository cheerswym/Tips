#include "onboard/perception/semantic_map_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

DEFINE_bool(perception_semantic_map_util_cvs, false,
            "Enable perception semantic map util cvs.");
namespace qcraft::perception_semantic_map_util {

namespace {

static constexpr double kMaxNearZonesRange = 200.0;

void MaybeRenderLaneHeadingAtPos(const double lane_heading, const Vec2d p0,
                                 const Vec2d p1, const Vec2d pos) {
  if (FLAGS_perception_semantic_map_util_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/semantic_map_util_cvs");
    canvas.DrawLine(Vec3d(pos, 0.0),
                    Vec3d(pos + Vec2d::UnitFromAngle(lane_heading), 0.0),
                    vis::Color::kYellow, 1);
    canvas.DrawPoint(Vec3d(pos, 0.0), vis::Color::kSkyBlue, 3);
    canvas.DrawPoint(Vec3d(p0, 0.0), vis::Color::kGreen, 3);
    canvas.DrawPoint(Vec3d(p1, 0.0), vis::Color::kRed, 3);
  }
}
}  // namespace

std::vector<Polygon2d> CollectNearPerceptionZones(
    const mapping::SemanticMapManager& semantic_map_manager,
    mapping::PerceptionZoneProto::Type type, const VehiclePose& pose,
    const CoordinateConverter& coordinate_converter) {
  const auto perception_zones = semantic_map_manager.GetPerceptionZonesAtLevel(
      semantic_map_manager.GetLevel(), {pose.x, pose.y}, kMaxNearZonesRange);
  std::vector<Polygon2d> near_zones;
  near_zones.reserve(perception_zones.size());
  for (const auto* perception_zone : perception_zones) {
    if (perception_zone->type() == type) {
      near_zones.emplace_back(SmoothPolygon2dFromGeoPolygonProto(
          perception_zone->polygon(), coordinate_converter));
    }
  }
  return near_zones;
}

std::vector<Polygon2d> CollectNearDriveways(
    const mapping::SemanticMapManager& semantic_map_manager,
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter) {
  const auto driveways = semantic_map_manager.GetDrivewaysAtLevel(
      semantic_map_manager.GetLevel(), {pose.x, pose.y}, kMaxNearZonesRange);
  std::vector<Polygon2d> near_zones;
  near_zones.reserve(driveways.size());
  for (const auto* driveway : driveways) {
    near_zones.emplace_back(SmoothPolygon2dFromGeoPolygonProto(
        driveway->polygon(), coordinate_converter));
  }
  return near_zones;
}

std::vector<Polygon2d> CollectNearParkingAreas(
    const mapping::SemanticMapManager& semantic_map_manager,
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter) {
  const auto parking_areas = semantic_map_manager.GetParkingAreasAtLevel(
      semantic_map_manager.GetLevel(), {pose.x, pose.y}, kMaxNearZonesRange);
  std::vector<Polygon2d> near_zones;
  near_zones.reserve(parking_areas.size());
  for (const auto* parking_area : parking_areas) {
    near_zones.emplace_back(SmoothPolygon2dFromGeoPolygonProto(
        parking_area->polygon(), coordinate_converter));
  }
  return near_zones;
}

std::optional<double> ComputeLaneHeadingAtPos(
    const SemanticMapManager& semantic_map_manager, const Vec2d pos) {
  const auto level_id = semantic_map_manager.GetLevel();
  const auto* lane_info =
      semantic_map_manager.GetNearestLaneInfoAtLevel(level_id, pos);
  if (lane_info == nullptr) return std::nullopt;
  const auto& points_smooth = lane_info->points_smooth;
  if (points_smooth.size() < 2) return std::nullopt;
  int min_dist_index = 0;
  int second_min_dist_index = 1;
  if ((points_smooth[min_dist_index] - pos).squaredNorm() >
      (points_smooth[second_min_dist_index] - pos).squaredNorm()) {
    std::swap(min_dist_index, second_min_dist_index);
  }
  for (int i = 2; i < points_smooth.size(); ++i) {
    const auto& point = points_smooth[i];
    const double dist_sqr = (point - pos).squaredNorm();
    const double min_dist_sqr =
        (points_smooth[min_dist_index] - pos).squaredNorm();
    const double second_min_dist_sqr =
        (points_smooth[second_min_dist_index] - pos).squaredNorm();
    if (dist_sqr <= min_dist_sqr) {
      second_min_dist_index = min_dist_index;
      min_dist_index = i;
    } else if (dist_sqr < second_min_dist_sqr) {
      second_min_dist_index = i;
    }
  }
  QCHECK_NE(min_dist_index, second_min_dist_index);
  const double lane_heading = min_dist_index > second_min_dist_index
                                  ? (points_smooth[min_dist_index] -
                                     points_smooth[second_min_dist_index])
                                        .FastAngle()
                                  : (points_smooth[second_min_dist_index] -
                                     points_smooth[min_dist_index])
                                        .FastAngle();

  MaybeRenderLaneHeadingAtPos(lane_heading, points_smooth[min_dist_index],
                              points_smooth[second_min_dist_index], pos);

  return lane_heading;
}
}  // namespace qcraft::perception_semantic_map_util
