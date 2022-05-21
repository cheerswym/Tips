#include "onboard/perception/sensor_fov/sensor_fov.h"

#include <cmath>
#include <map>
#include <string>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/global/run_context.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/test_util/obstacle_builder.h"
#include "snappy/snappy.h"

namespace qcraft::sensor_fov {

TEST(SensorFovTest, TestSensorFovProtoConvertion) {
  FLAGS_enable_context_test = 1;
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  // Compute sensor fov
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov_0 = GetLidarViewSensorFov(sensor_fovs);
  const auto sf_proto_0 = SensorFov::SensorFovToProto(*sensor_fov_0);
  const auto sensor_fov_1 = SensorFov::ProtoToSensorFov(sf_proto_0);
  const auto sf_proto_1 = SensorFov::SensorFovToProto(sensor_fov_1);

  EXPECT_EQ(sf_proto_0.sf_grid().serialized_data().size(),
            sf_proto_1.sf_grid().serialized_data().size());

  SensorFovGrid<SFPillar> sf_grid_0(
      sf_proto_0.sf_grid().detection_range().front(),
      sf_proto_0.sf_grid().detection_range().behind(),
      sf_proto_0.sf_grid().detection_range().left(),
      sf_proto_0.sf_grid().detection_range().right(),
      sf_proto_0.sf_grid().diameter());
  std::string serialized_data_0;
  QCHECK(snappy::Uncompress(sf_proto_0.sf_grid().serialized_data().data(),
                            sf_proto_0.sf_grid().serialized_data().size(),
                            &serialized_data_0));
  sf_grid_0.Deserialize(serialized_data_0);
  sf_grid_0.InitializeWithPose(VehiclePose(sf_proto_0.pose()));
  SensorFovGrid<SFPillar> sf_grid_1(
      sf_proto_1.sf_grid().detection_range().front(),
      sf_proto_1.sf_grid().detection_range().behind(),
      sf_proto_1.sf_grid().detection_range().left(),
      sf_proto_1.sf_grid().detection_range().right(),
      sf_proto_1.sf_grid().diameter());
  std::string serialized_data_1;
  QCHECK(snappy::Uncompress(sf_proto_1.sf_grid().serialized_data().data(),
                            sf_proto_1.sf_grid().serialized_data().size(),
                            &serialized_data_1));
  sf_grid_1.Deserialize(serialized_data_1);
  sf_grid_1.InitializeWithPose(VehiclePose(sf_proto_1.pose()));
  EXPECT_EQ(sf_grid_0.width(), sf_grid_1.width());
  EXPECT_EQ(sf_grid_0.height(), sf_grid_1.height());
  std::map<SensorFovState, int> num_state_map_0;
  std::map<SensorFovState, int> num_state_map_1;
  for (int row = 0; row < sf_grid_0.height(); ++row) {
    for (int col = 0; col < sf_grid_0.width(); ++col) {
      const auto& info_0 = sf_grid_0(row, col);
      const auto& info_1 = sf_grid_1(row, col);
      EXPECT_EQ(info_0.state, info_1.state);
      EXPECT_NEAR(info_0.max_invisable_elevation,
                  info_1.max_invisable_elevation, 1e-6);
      ++num_state_map_0[info_0.state];
      ++num_state_map_1[info_1.state];
    }
  }
  EXPECT_EQ(num_state_map_0[SF_OCCUPIED], num_state_map_1[SF_OCCUPIED]);
  EXPECT_GT(num_state_map_0[SF_OCCUPIED], 0);
  EXPECT_EQ(num_state_map_0[SF_OCCLUDED], num_state_map_1[SF_OCCLUDED]);
  EXPECT_GT(num_state_map_0[SF_OCCLUDED], 0);
}

TEST(SensorFovTest, TestIsOccluded) {
  auto sensor_fov_builder = BuildSensorFovBuilder();
  const VehiclePose pose(4444.0, 5555.0, 0.0, M_PI_4, 0.0, 0.0);
  const auto& [clusters, obstacle_refs] = BuildSegmentedClusters(pose);
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  // Compute sensor fov
  const auto sensor_fovs =
      sensor_fov_builder->Compute(obstacle_ptrs, clusters, {0.0, pose});
  const auto sensor_fov = GetLidarViewSensorFov(sensor_fovs);
  const AffineTransformation vehicle_to_smooth = pose.ToTransform();
  // Test IsOccluded Point
  {
    const Vec2d point_in_vehicle(41.0, -31.0);
    const Vec2d point_in_smooth =
        vehicle_to_smooth.TransformPoint({point_in_vehicle, 0}).head<2>();
    const auto status_or = sensor_fov->IsOccluded(point_in_smooth, 1.0);
    EXPECT_EQ(status_or.ok(), true);
    EXPECT_EQ(status_or.value(), true);
  }
  {
    const Vec2d point_in_vehicle(41.0, -31.0);
    const Vec2d point_in_smooth =
        vehicle_to_smooth.TransformPoint({point_in_vehicle, 0}).head<2>();
    const auto status_or = sensor_fov->IsOccluded(point_in_smooth, 1.5);
    EXPECT_EQ(status_or.ok(), true);
    EXPECT_EQ(status_or.value(), false);
  }
  {
    const Vec2d point_in_vehicle(1000.0, 1000.0);
    const Vec2d point_in_smooth =
        vehicle_to_smooth.TransformPoint({point_in_vehicle, 0}).head<2>();
    const auto status_or = sensor_fov->IsOccluded(point_in_smooth, 1.5);
    EXPECT_EQ(status_or.ok(), false);
  }
  // Test IsOccluded Box2d
  {
    const Box2d box_in_vehicle({45.0, -33.0}, 0.0, 4.5, 2.0);
    const Box2d box_in_smooth = geometry_util::ConvertBox2dVehicleToSmooth(
        box_in_vehicle, pose, vehicle_to_smooth);
    const auto status_or = sensor_fov->IsOccluded(box_in_smooth, 0.8);
    EXPECT_EQ(status_or.ok(), true);
    EXPECT_EQ(status_or.value(), true);
  }
  {
    const Box2d box_in_vehicle({10.0, 10.0}, 0.0, 4.5, 2.0);
    const Box2d box_in_smooth = geometry_util::ConvertBox2dVehicleToSmooth(
        box_in_vehicle, pose, vehicle_to_smooth);
    const auto status_or = sensor_fov->IsOccluded(box_in_smooth, 0.8);
    EXPECT_EQ(status_or.ok(), true);
    EXPECT_EQ(status_or.value(), false);
  }
  {
    const Box2d box_in_vehicle({150.0, -80.0}, 0.0, 4.5, 2.0);
    const Box2d box_in_smooth = geometry_util::ConvertBox2dVehicleToSmooth(
        box_in_vehicle, pose, vehicle_to_smooth);
    const auto status_or = sensor_fov->IsOccluded(box_in_smooth, 0.8);
    EXPECT_EQ(status_or.ok(), false);
  }
  // Test IsOccluded Polygon2d
  {
    std::vector<Vec2d> points{{-5.0, -5.0}, {-6.0, -5.0}, {-5.5, -6.0}};
    const Polygon2d polygon_in_vehicle(std::move(points));
    const Polygon2d polygon_in_smooth =
        geometry_util::ConvertPolygon2dVehicleToSmooth(polygon_in_vehicle,
                                                       vehicle_to_smooth);
    const auto status_or = sensor_fov->IsOccluded(polygon_in_smooth, 0.8);
    EXPECT_EQ(status_or.ok(), true);
    EXPECT_EQ(status_or.value(), false);
  }
  {
    std::vector<Vec2d> points{{-25.0, -35.0}, {-26.0, -34.0}, {-27.0, -35.0}};
    const Polygon2d polygon_in_vehicle(std::move(points));
    const Polygon2d polygon_in_smooth =
        geometry_util::ConvertPolygon2dVehicleToSmooth(polygon_in_vehicle,
                                                       vehicle_to_smooth);
    const auto status_or = sensor_fov->IsOccluded(polygon_in_smooth, 0.8);
    EXPECT_EQ(status_or.ok(), false);
  }
}

}  // namespace qcraft::sensor_fov
