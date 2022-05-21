#include "onboard/perception/sensor_fov/sensor_fov.h"

#include <algorithm>
#include <atomic>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/utils/map_util.h"
#include "snappy/snappy.h"

namespace qcraft::sensor_fov {

namespace {

bool IsPillarOccluded(const SFPillar& pillar, const float height) {
  return pillar.state == SF_OCCLUDED &&
         pillar.max_invisable_elevation >= height &&
         pillar.min_invisable_elevation <= height;
}

std::pair<Vec2i, Vec2i> GetGridIndexRange(
    const SensorFovGrid<SFPillar>& sf_grid, const AABox2d& aabox) {
  auto [min_row, min_col] =
      sf_grid.VehicleCoordToRC({aabox.max_x(), aabox.max_y()});
  min_row = std::clamp(min_row, 0, sf_grid.height() - 1);
  min_col = std::clamp(min_col, 0, sf_grid.width() - 1);
  auto [max_row, max_col] =
      sf_grid.VehicleCoordToRC({aabox.min_x(), aabox.min_y()});
  max_row = std::clamp(max_row, 0, sf_grid.height() - 1);
  max_col = std::clamp(max_col, 0, sf_grid.width() - 1);
  return {Vec2i(min_row, min_col), Vec2i(max_row, max_col)};
}

}  // namespace

SensorFovProto SensorFov::SensorFovToProto(const SensorFov& sensor_fov) {
  SCOPED_QTRACE("SensorFov::SensorFovToProto");
  const auto& sf_grid = sensor_fov.sf_grid_;
  SensorFovProto proto;
  proto.set_timestamp(sensor_fov.pose_.timestamp);
  *proto.mutable_pose() = sensor_fov.pose_.vehicle_pose.ToVehiclePoseProto();
  for (const auto& [sensor_name, ref_sensor] : sensor_fov.ref_sensors_) {
    auto& ref_sensor_proto = (*proto.mutable_ref_sensors())[sensor_name];
    ref_sensor_proto.set_x(ref_sensor.x());
    ref_sensor_proto.set_y(ref_sensor.y());
    ref_sensor_proto.set_z(ref_sensor.z());
  }
  proto.set_view_type(sensor_fov.view_type_);
  // Build sf grid proto.
  auto* sf_grid_proto = proto.mutable_sf_grid();
  *sf_grid_proto->mutable_detection_range() =
      sf_grid.detection_range().ToProto();
  sf_grid_proto->set_diameter(sf_grid.diameter());
  sf_grid_proto->set_elevation_resolution(kSerializedElevationResolution);
  std::string serialized_data;
  sf_grid.Serialize(&serialized_data);
  QCHECK(snappy::Compress(serialized_data.data(), serialized_data.size(),
                          sf_grid_proto->mutable_serialized_data()));

  return proto;
}

SensorFov SensorFov::ProtoToSensorFov(const SensorFovProto& proto) {
  SCOPED_QTRACE("SensorFov::ProtoToSensorFov");
  QCHECK_NEAR(proto.sf_grid().elevation_resolution(),
              kSerializedElevationResolution, 1e-5);
  std::map<std::string, Vec3d> ref_sensors;
  for (const auto& [sensor_name, ref_sensor_proto] : proto.ref_sensors()) {
    ref_sensors.emplace(sensor_name,
                        Vec3d(ref_sensor_proto.x(), ref_sensor_proto.y(),
                              ref_sensor_proto.z()));
  }
  const auto& detection_range = proto.sf_grid().detection_range();
  SensorFov sensor_fov(
      detection_range.front(), detection_range.behind(), detection_range.left(),
      detection_range.right(), proto.sf_grid().diameter(), proto.view_type(),
      ref_sensors, {proto.timestamp(), VehiclePose(proto.pose())});
  // Build sf grid.
  std::string serialized_data;
  QCHECK(snappy::Uncompress(proto.sf_grid().serialized_data().data(),
                            proto.sf_grid().serialized_data().size(),
                            &serialized_data));
  sensor_fov.sf_grid_.Deserialize(serialized_data);

  return sensor_fov;
}

absl::StatusOr<bool> SensorFov::IsOccluded(const Vec2d& point,
                                           const float height) const {
  if (!sf_grid_.IsSmoothCoordInRange(point)) {
    return absl::OutOfRangeError("Point is out of sensor fov detection range.");
  }
  const auto [row, col] = sf_grid_.SmoothCoordToRC(point);
  const auto& pillar = sf_grid_(row, col);
  return IsPillarOccluded(pillar, height);
}

absl::StatusOr<bool> SensorFov::IsOccluded(const Box2d& box,
                                           const float height) const {
  const auto box_in_vehicle = geometry_util::ConvertBox2dSmoothToVehicle(
      box, pose_.vehicle_pose, smooth_to_vehicle_);
  for (const auto& corner : box_in_vehicle.GetCornersCounterClockwise()) {
    if (!sf_grid_.IsVehicleCoordInRange(corner)) {
      return absl::OutOfRangeError("Box is out of sensor fov detection range.");
    }
  }
  const auto [min_vertex, max_vertex] =
      GetGridIndexRange(sf_grid_, box_in_vehicle.GetAABox());
  int num_unvisable_pillars = 0;
  int num_pillars_in_box = 0;
  for (int row = min_vertex.x(); row <= max_vertex.x(); ++row) {
    for (int col = min_vertex.y(); col <= max_vertex.y(); ++col) {
      const auto& coord = sf_grid_.RCToVehicleCoord({row, col});
      if (!box_in_vehicle.IsPointIn(coord)) continue;
      const auto& pillar = sf_grid_(row, col);
      if (IsPillarOccluded(pillar, height)) {
        ++num_unvisable_pillars;
      }
      ++num_pillars_in_box;
    }
  }
  return view_type_ == VT_LIDAR
             ? num_unvisable_pillars >
                   num_pillars_in_box * kLidarViewMinBoxOccludedRatio
             : num_unvisable_pillars >
                   num_pillars_in_box * kCameraViewMinBoxOccludedRatio;
}

absl::StatusOr<bool> SensorFov::IsOccluded(const Polygon2d& polygon,
                                           const float height) const {
  const auto polygon_in_vehicle =
      geometry_util::ConvertPolygon2dSmoothToVehicle(polygon,
                                                     smooth_to_vehicle_);
  for (const auto& point : polygon_in_vehicle.points()) {
    if (!sf_grid_.IsVehicleCoordInRange(point)) {
      return absl::OutOfRangeError(
          "Polygon is out of sensor fov detection range.");
    }
  }
  const auto [min_vertex, max_vertex] =
      GetGridIndexRange(sf_grid_, polygon_in_vehicle.AABoundingBox());
  int num_unvisable_pillars = 0;
  int num_pillars_in_box = 0;
  for (int row = min_vertex.x(); row <= max_vertex.x(); ++row) {
    for (int col = min_vertex.y(); col <= max_vertex.y(); ++col) {
      const auto& coord = sf_grid_.RCToVehicleCoord({row, col});
      if (!polygon_in_vehicle.IsPointIn(coord)) continue;
      const auto& pillar = sf_grid_(row, col);
      if (IsPillarOccluded(pillar, height)) {
        ++num_unvisable_pillars;
      }
      ++num_pillars_in_box;
    }
  }
  return view_type_ == VT_LIDAR
             ? num_unvisable_pillars >
                   num_pillars_in_box * kLidarViewMinPolygonOccludedRatio
             : num_unvisable_pillars >
                   num_pillars_in_box * kCameraViewMinPolygonOccludedRatio;
}

CameraId SensorFov::camera_id() const {
  QCHECK_EQ(view_type_, VT_CAMERA);
  QCHECK_EQ(ref_sensors_.size(), 1);
  CameraId camera_id;
  QCHECK(CameraId_Parse(ref_sensors_.begin()->first, &camera_id));
  return camera_id;
}

SensorFov BuildLidarViewSensorFov(const SensorFovsProto& sensor_fovs_proto) {
  const SensorFovProto* lidar_view_sensor_fov_proto = nullptr;
  for (const auto& sensor_fov_proto : sensor_fovs_proto.sensor_fovs()) {
    if (sensor_fov_proto.view_type() == VT_LIDAR) {
      lidar_view_sensor_fov_proto = &sensor_fov_proto;
      break;
    }
  }
  return SensorFov::ProtoToSensorFov(
      *QCHECK_NOTNULL(lidar_view_sensor_fov_proto));
}

std::map<CameraId, SensorFov> BuildCameraViewSensorFovs(
    const SensorFovsProto& sensor_fovs_proto) {
  std::map<CameraId, SensorFov> camera_view_sensor_fovs;
  for (const auto& sensor_fov_proto : sensor_fovs_proto.sensor_fovs()) {
    if (sensor_fov_proto.view_type() == VT_CAMERA) {
      QCHECK_EQ(sensor_fov_proto.ref_sensors().size(), 1);
      CameraId camera_id;
      QCHECK(CameraId_Parse(sensor_fov_proto.ref_sensors().begin()->first,
                            &camera_id));
      camera_view_sensor_fovs.emplace(
          camera_id, SensorFov::ProtoToSensorFov(sensor_fov_proto));
    }
  }
  return camera_view_sensor_fovs;
}

SensorFovRef GetLidarViewSensorFov(const SensorFovRefs& sensor_fovs) {
  SensorFovRef lidar_view_sensor_fov;
  for (const auto& sensor_fov : sensor_fovs) {
    if (sensor_fov->view_type() == VT_LIDAR) {
      lidar_view_sensor_fov = sensor_fov;
      break;
    }
  }
  return QCHECK_NOTNULL(lidar_view_sensor_fov);
}

std::map<CameraId, SensorFovRef> GetCameraViewSensorFovs(
    const SensorFovRefs& sensor_fovs) {
  std::map<CameraId, SensorFovRef> camera_view_sensor_fovs;
  for (const auto& sensor_fov : sensor_fovs) {
    if (sensor_fov->view_type() == VT_CAMERA) {
      camera_view_sensor_fovs.emplace(sensor_fov->camera_id(), sensor_fov);
    }
  }
  return camera_view_sensor_fovs;
}

}  // namespace qcraft::sensor_fov
