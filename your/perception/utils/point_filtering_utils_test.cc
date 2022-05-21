#include "onboard/perception/utils/point_filtering_utils.h"

#include <random>

#include "gtest/gtest.h"

namespace qcraft::point_filtering_util {

TEST(PointFilteringTest, FilterPedPoints) {
  LaserPoint p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = -1;
  LaserPoint p2;
  p2.x = 0;
  p2.y = 10000;
  p2.z = 1;
  LaserPoint p3;
  p3.x = 100;
  p3.y = 0;
  p3.z = 100;
  LaserPoint p4;
  p4.x = 0;
  p4.y = 100;
  p4.z = -100;
  const std::vector<LaserPointGroundZ> raw_points = {
      std::make_pair(p1, 0.5), std::make_pair(p2, 0.1), std::make_pair(p3, 0),
      std::make_pair(p4, 0)};
  const auto filtered_output =
      FilterPedPoints(raw_points, /*max_distance_to_ground_z=*/2.0);
  EXPECT_EQ(filtered_output.size(), 1);
}

}  // namespace qcraft::point_filtering_util
