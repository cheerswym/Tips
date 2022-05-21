#include "onboard/perception/test_util/measurement_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace perception {
namespace {

TEST(LaserMeasurementBuilderTest, Build) {
  const auto laser_m = LaserMeasurementBuilder().Build();
  EXPECT_THAT(laser_m.contour(0), ProtoEqText("x: 0.0, y: 0.0"));
  EXPECT_EQ(laser_m.cluster_measurement().height(), 3.);
}

TEST(LaserMeasurementBuilderTest, SetContour) {
  const auto laser_m =
      LaserMeasurementBuilder()
          .set_contour(Polygon2d({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}))
          .Build();
  std::vector<Vec2d> points;
  for (const auto& vertex : laser_m.contour()) {
    points.emplace_back(vertex);
  }
  EXPECT_THAT(points,
              testing::ElementsAre(Vec2dEqXY(1.0, 0.0), Vec2dEqXY(1.0, 1.0),
                                   Vec2dEqXY(0.0, 1.0)));
}

TEST(LaserMeasurementBuilderTest, SetClusterMeasurement) {
  const auto laser_m =
      LaserMeasurementBuilder()
          .set_contour(Polygon2d({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}))
          .set_cluster_measurement(
              ClusterMeasurementBuilder()
                  .set_obstacle_info(
                      {ObstacleInfoBuilder().set_center({1.0, 0.0}).Build(),
                       ObstacleInfoBuilder().set_center({1.0, 1.0}).Build(),
                       ObstacleInfoBuilder().set_center({0.0, 1.0}).Build()})
                  .Build())
          .Build();
  EXPECT_EQ(laser_m.cluster_measurement().obstacle_info_size(), 3);
}

TEST(LaserMeasurementBuilderTest, SetIcpMeasurement) {
  const auto laser_m =
      LaserMeasurementBuilder()
          .set_contour(Polygon2d({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}))
          .set_icp_measurement(
              IcpMeasurementBuilder().set_vel({1.0, 2.0}).Build())
          .Build();
  EXPECT_THAT(laser_m.icp_measurement().vel(), ProtoEqText("x: 1.0, y: 2.0"));
}

TEST(LaserMeasurementBuilderTest, AdjustClusterMeasurementGivenContour) {
  const auto laser_m =
      LaserMeasurementBuilder()
          .set_contour(Polygon2d({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}))
          .AdjustClusterMeasurementGivenContour()
          .Build();
  EXPECT_EQ(laser_m.cluster_measurement().obstacle_info_size(), 3);
  EXPECT_THAT(laser_m.cluster_measurement().obstacle_info(0).center(),
              Vec2dEqXY(1.0, 0.0));
  EXPECT_THAT(laser_m.cluster_measurement().obstacle_info(1).center(),
              Vec2dEqXY(1.0, 1.0));
  EXPECT_THAT(laser_m.cluster_measurement().obstacle_info(2).center(),
              Vec2dEqXY(0.0, 1.0));
}

TEST(RadarMeasurementBuilderTest, Build) {
  const auto radar_m = RadarMeasurementBuilder().Build();
  EXPECT_EQ(radar_m.id(), 0);
}

TEST(RadarMeasurementBuilderTest, SetId) {
  const auto radar_m = RadarMeasurementBuilder().set_id(1).Build();
  EXPECT_EQ(radar_m.id(), 1);
}

TEST(Camera3DMeasurementBuilderTest, Build) {
  const auto camera_m = Camera3dMeasurementBuilder().Build();
  EXPECT_EQ(camera_m.width(), 1.0);
}

}  // namespace
}  // namespace perception
}  // namespace qcraft
