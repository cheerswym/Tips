#include "onboard/perception/segmentation/types.h"

#include <vector>

#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/semantic_map_util.h"

namespace qcraft::segmentation {
namespace {
static constexpr double kMaxNearZonesRange = 200.0;
}

ProposerEnvInfo::ProposerEnvInfo(
    const VehiclePose& pose, const ObstacleManager& obstacle_manager,
    const SemanticMapManager& semantic_map_manager,
    const LocalImagery& local_imagery,
    const CoordinateConverter& coordinate_converter,
    const RunParamsProtoV2& run_params, const Context& context)
    : pose_(pose),
      obstacle_manager_(obstacle_manager),
      semantic_map_manager_(semantic_map_manager),
      local_imagery_(local_imagery),
      coordinate_converter_(coordinate_converter),
      run_params_(run_params),
      context_(context) {
  for (const auto& lidar_params :
       run_params_.get().vehicle_params().lidar_params()) {
    InsertOrDie(&lidar_params_map_, lidar_params.installation().lidar_id(),
                lidar_params);
  }

  camera_params_ = ComputeAllCameraParams(run_params_.get().vehicle_params());

  CollectNearPerceptionZones();
  ComputeAvBox();
  ComputeAvHeight();
}

void ProposerEnvInfo::CollectNearPerceptionZones() {
  const auto perception_zones =
      semantic_map_manager().GetPerceptionZonesAtLevel(
          semantic_map_manager().GetLevel(), {pose().x, pose().y},
          kMaxNearZonesRange);
  for (const auto* perception_zone : perception_zones) {
    auto& perception_zones =
        perception_zones_array_[static_cast<int>(perception_zone->type())];
    Polygon2d zone_polygon = SmoothPolygon2dFromGeoPolygonProto(
        perception_zone->polygon(), coordinate_converter());
    perception_zones.push_back(std::move(zone_polygon));
  }
}

bool ProposerEnvInfo::PointInZones(
    const Vec2d& point, const mapping::PerceptionZoneProto::Type type) const {
  return perception_semantic_map_util::PointInZones(
      point, perception_zones_array_[static_cast<int>(type)]);
}

void ProposerEnvInfo::ComputeAvBox() {
  QCHECK(run_params().vehicle_params().has_vehicle_geometry_params());

  const auto& geometry_params =
      run_params().vehicle_params().vehicle_geometry_params();
  const double av_width = geometry_params.width();
  const double av_length = geometry_params.length();
  const double center_to_rac_dist =
      geometry_params.front_edge_to_center() - av_length * 0.5;
  av_box_ =
      Box2d({pose().x, pose().y}, pose().yaw, av_length * 1.1, av_width * 1.1);
  av_box_.Shift(Vec2d(center_to_rac_dist, 0.0).FastRotate(pose().yaw));
}

void ProposerEnvInfo::ComputeAvHeight() {
  av_height_ = obstacle_util::ComputeAvHeight(run_params_);
}

}  // namespace qcraft::segmentation
