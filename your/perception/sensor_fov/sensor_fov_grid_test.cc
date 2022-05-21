#include "onboard/perception/sensor_fov/sensor_fov_grid.h"

#include <algorithm>
#include <random>

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::sensor_fov {

constexpr float kRangeFront = 20.f;    // m
constexpr float kRangeRear = 10.f;     // m
constexpr float kRangeLateral = 10.f;  // m
constexpr float kDiameter = 0.1f;      // m

TEST(SensorFovGridTest, TestRCCoordConvertion) {
  struct GridInfo {};
  SensorFovGrid<GridInfo> sensor_fov_grid(
      kRangeFront, kRangeRear, kRangeLateral, kRangeLateral, kDiameter);
  const auto [r0, c0] =
      sensor_fov_grid.VehicleCoordToRC({kRangeFront, kRangeLateral});
  EXPECT_EQ(r0, 0);
  EXPECT_EQ(c0, 0);

  const auto [r1, c1] =
      sensor_fov_grid.VehicleCoordToRC({-kRangeRear, -kRangeLateral});
  EXPECT_EQ(r1, sensor_fov_grid.height() - 1);
  EXPECT_EQ(c1, sensor_fov_grid.width() - 1);

  const auto [r2, c2] =
      sensor_fov_grid.VehicleCoordToRC({kRangeFront * 2, kRangeLateral * 2});
  EXPECT_LT(r2, 0);
  EXPECT_LT(c2, 0);

  const auto [r3, c3] =
      sensor_fov_grid.VehicleCoordToRC({-kRangeRear * 2, -kRangeLateral * 2});
  EXPECT_GT(r3, sensor_fov_grid.height());
  EXPECT_GT(c3, sensor_fov_grid.width());
}

TEST(SensorFovGridTest, TestSerializeAndDeserialize) {
  std::mt19937 e;
  std::uniform_real_distribution<float> u(0, 2.f);
  SensorFovGrid<SFPillar> sensor_fov_grid(
      kRangeFront, kRangeRear, kRangeLateral, kRangeLateral, kDiameter);
  std::vector<float> elevations(sensor_fov_grid.height() *
                                sensor_fov_grid.width());
  for (float x = -5.f; x <= 10.f; x += 0.1f) {
    for (float y = -5.f; y <= 5.f; y += 0.1f) {
      const auto [row, col] = sensor_fov_grid.VehicleCoordToRC({x, y});
      EXPECT_GE(row, 0);
      EXPECT_GE(col, 0);
      EXPECT_LT(row, sensor_fov_grid.height());
      EXPECT_LT(col, sensor_fov_grid.width());
      auto& info = sensor_fov_grid(row, col);
      info.state = SF_OCCUPIED;
      const float elevation = u(e);
      info.max_invisable_elevation = elevation;
      elevations[row * sensor_fov_grid.width() + col] = elevation;
    }
  }

  std::string serialized_data;
  sensor_fov_grid.Serialize(&serialized_data);
  SensorFovGrid<SFPillar> sensor_fov_grid_deserialized(
      kRangeFront, kRangeRear, kRangeLateral, kRangeLateral, kDiameter);
  sensor_fov_grid_deserialized.Deserialize(serialized_data);
  for (float x = -5.f; x <= 10.f; x += 0.1f) {
    for (float y = -5.f; y <= 5.f; y += 0.1f) {
      const auto [row, col] =
          sensor_fov_grid_deserialized.VehicleCoordToRC({x, y});
      const auto& info = sensor_fov_grid(row, col);
      EXPECT_EQ(info.state, SF_OCCUPIED);
      EXPECT_NEAR(info.max_invisable_elevation,
                  elevations[row * sensor_fov_grid_deserialized.width() + col],
                  kSerializedElevationResolution);
    }
  }
}

}  // namespace qcraft::sensor_fov
