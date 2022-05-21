#ifndef ONBOARD_PERCEPTION_SEMANTIC_MAP_UTIL_H_
#define ONBOARD_PERCEPTION_SEMANTIC_MAP_UTIL_H_

#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/util.h"

namespace qcraft::perception_semantic_map_util {

inline bool PointInZones(const Vec2d point,
                         const std::vector<Polygon2d>& zones) {
  for (const auto& zone : zones) {
    if (zone.IsPointIn({point.x(), point.y()})) {
      return true;
    }
  }
  return false;
}

std::vector<Polygon2d> CollectNearPerceptionZones(
    const mapping::SemanticMapManager& semantic_map_manager,
    mapping::PerceptionZoneProto::Type type, const VehiclePose& pose,
    const CoordinateConverter& coordinate_converter);

std::vector<Polygon2d> CollectNearDriveways(
    const mapping::SemanticMapManager& semantic_map_manager,
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter);

std::vector<Polygon2d> CollectNearParkingAreas(
    const mapping::SemanticMapManager& semantic_map_manager,
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter);

std::optional<double> ComputeLaneHeadingAtPos(
    const SemanticMapManager& semantic_map_manager, const Vec2d pos);
}  // namespace qcraft::perception_semantic_map_util

#endif  // ONBOARD_PERCEPTION_SEMANTIC_MAP_UTIL_H_
