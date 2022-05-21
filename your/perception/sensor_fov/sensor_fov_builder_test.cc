#include "onboard/perception/sensor_fov/sensor_fov_builder.h"

#include <map>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/sensor_fov/line_iterator.h"
#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"

namespace qcraft::sensor_fov {

TEST(SensorFovBuilderTest, TestAllRaysShouldCoverSensorFovGrid) {
  SensorFovGrid<uint8_t> sensor_fov_grid(
      kSFDetectionRangeFront, kSFDetectionRangeRear, kSFDetectionRangeLateral,
      kSFDetectionRangeLateral, kSFGridDiameter);
  const int height = sensor_fov_grid.height();
  const int width = sensor_fov_grid.width();
  std::vector<std::vector<Vec2i>> contour_lines;
  contour_lines.reserve(4);
  contour_lines.emplace_back(
      ComputeAllLinePoints({0, 0}, {height - 1, 0}, /*connectivity*/ 4));
  contour_lines.emplace_back(ComputeAllLinePoints(
      {height - 1, 0}, {height - 1, width - 1}, /*connectivity*/ 4));
  contour_lines.emplace_back(ComputeAllLinePoints(
      {height - 1, width - 1}, {0, width - 1}, /*connectivity*/ 4));
  contour_lines.emplace_back(
      ComputeAllLinePoints({0, width - 1}, {0, 0}, /*connectivity*/ 4));
  const auto [sensor_row, sensor_col] =
      sensor_fov_grid.VehicleCoordToRC({4.0, 1.2});
  const Vec2i start_point(sensor_row, sensor_col);
  for (const auto& contour_line : contour_lines) {
    for (const auto& line_point : contour_line) {
      const auto line_points =
          ComputeAllLinePoints(start_point, line_point, /*connectivity*/ 8);
      for (const auto& line_point : line_points) {
        sensor_fov_grid(line_point.x(), line_point.y()) = 1;
      }
    }
  }
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      EXPECT_EQ(sensor_fov_grid(row, col), 1);
    }
  }
}

}  // namespace qcraft::sensor_fov
