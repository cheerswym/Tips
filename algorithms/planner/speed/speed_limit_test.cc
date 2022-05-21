#include "onboard/planner/speed/speed_limit.h"

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

using Range = SpeedLimit::SpeedLimitRange;
constexpr double kDefaultSpeedLimit = 20.0;

//
//
//    |    1                        8
//    |    o-----------------------o     speed_limit: 5
//    |       2     4    5    6
//    |       o-----o    o----o          speed_limit: 4
//    |         3   4
//    |         o---o                    speed_limit: 3
//    |------------------------------------->s
//
TEST(SpeedLimitTest, Test_1) {
  std::vector<Range> ranges;
  ranges.reserve(4);
  ranges.push_back(
      {.start_s = 1.0, .end_s = 8.0, .speed_limit = 5.0, .info = "range_1"});
  ranges.push_back(
      {.start_s = 2.0, .end_s = 4.0, .speed_limit = 4.0, .info = "range_2"});
  ranges.push_back(
      {.start_s = 5.0, .end_s = 6.0, .speed_limit = 4.0, .info = "range_3"});
  ranges.push_back(
      {.start_s = 3.0, .end_s = 4.0, .speed_limit = 3.0, .info = "range_4"});
  SpeedLimit st_speed_limit(ranges, kDefaultSpeedLimit);

  const auto speed_limit1 = st_speed_limit.GetSpeedLimitRangeByS(2.3);
  EXPECT_EQ(speed_limit1->speed_limit, 4.0);
  EXPECT_EQ(speed_limit1->info, "range_2");

  const auto speed_limit2 = st_speed_limit.GetSpeedLimitRangeByS(4.5);
  EXPECT_EQ(speed_limit2->speed_limit, 5.0);
  EXPECT_EQ(speed_limit2->info, "range_1");

  const auto speed_limit3 = st_speed_limit.GetSpeedLimitByS(7.5);
  EXPECT_EQ(speed_limit3, 5.0);

  const auto speed_limit4 = st_speed_limit.GetSpeedLimitByS(8.0);
  EXPECT_EQ(speed_limit4, 5.0);

  const auto speed_limit5 = st_speed_limit.GetSpeedLimitByS(8.1);
  EXPECT_EQ(speed_limit5, 20.0);

  const auto speed_limit6 = st_speed_limit.GetSpeedLimitRangeByS(3.9);
  EXPECT_EQ(speed_limit6->speed_limit, 3.0);
  EXPECT_EQ(speed_limit6->info, "range_4");

  const auto speed_limit_range = st_speed_limit.GetSpeedLimitRangeByS(1.1);
  EXPECT_EQ(speed_limit_range->speed_limit, 5.0);
  EXPECT_EQ(speed_limit_range->info, "range_1");
}

//
//
//    |                  3     4
//    |                  o-----o     speed_limit: 3
//    |     1     2
//    |     o-----o                  speed_limit: 2
//    |
//    |
//    |------------------------------------->s
//
TEST(SpeedLimitTest, Test_2) {
  std::vector<Range> ranges;
  ranges.reserve(2);
  ranges.push_back({.start_s = 1.0, .end_s = 2.0, .speed_limit = 2.0});
  ranges.push_back({.start_s = 3.0, .end_s = 4.0, .speed_limit = 3.0});
  SpeedLimit st_speed_limit(ranges, kDefaultSpeedLimit);

  const auto speed_limit1 = st_speed_limit.GetSpeedLimitByS(3.5);
  EXPECT_EQ(speed_limit1, 3.0);

  const auto speed_limit2 = st_speed_limit.GetSpeedLimitByS(2);
  EXPECT_EQ(speed_limit2, 2.0);

  const auto speed_limit3 = st_speed_limit.GetSpeedLimitByS(1.0);
  EXPECT_EQ(speed_limit3, 2);

  const auto speed_limit4 = st_speed_limit.GetSpeedLimitByS(3.0);
  EXPECT_EQ(speed_limit4, 3);

  const auto speed_limit5 = st_speed_limit.GetSpeedLimitByS(4.0);
  EXPECT_EQ(speed_limit5, 3);

  const auto speed_limit6 = st_speed_limit.GetSpeedLimitByS(2.5);
  EXPECT_EQ(speed_limit6, 20.0);
}

//
//
//    |     1                     10
//    |     o---------------------o        speed_limit: 3
//    |     1                     10
//    |     o---------------------o        speed_limit: 2
//    |
//    |
//    |------------------------------------->s
//
TEST(SpeedLimitTest, Test_3) {
  std::vector<Range> ranges;
  ranges.reserve(2);
  ranges.push_back({.start_s = 1.0, .end_s = 10.0, .speed_limit = 3.0});
  ranges.push_back({.start_s = 1.0, .end_s = 10.0, .speed_limit = 2.0});

  SpeedLimit st_speed_limit(ranges, kDefaultSpeedLimit);

  const auto speed_limit1 = st_speed_limit.GetSpeedLimitByS(2.0);
  EXPECT_EQ(speed_limit1, 2.0);

  const auto speed_limit2 = st_speed_limit.GetSpeedLimitByS(1.0);
  EXPECT_EQ(speed_limit2, 2.0);

  const auto speed_limit3 = st_speed_limit.GetSpeedLimitByS(10.0);
  EXPECT_EQ(speed_limit3, 2.0);
}

//
//
//    |     1                     10
//    |     o---------------------o      speed_limit: 3
//    |          3          6
//    |          o----------o            speed_limit: 2
//    |
//    |
//    |------------------------------------->s
//
TEST(SpeedLimitTest, Test_4) {
  std::vector<Range> ranges;
  ranges.reserve(2);
  ranges.push_back({.start_s = 1.0, .end_s = 10.0, .speed_limit = 3.0});
  ranges.push_back({.start_s = 3.0, .end_s = 6.0, .speed_limit = 2.0});
  SpeedLimit st_speed_limit(ranges, kDefaultSpeedLimit);

  const auto speed_limit1 = st_speed_limit.GetSpeedLimitByS(2.0);
  EXPECT_EQ(speed_limit1, 3.0);

  const auto speed_limit2 = st_speed_limit.GetSpeedLimitByS(4.0);
  EXPECT_EQ(speed_limit2, 2.0);

  const auto speed_limit3 = st_speed_limit.GetSpeedLimitByS(8.0);
  EXPECT_EQ(speed_limit3, 3.0);
}

//
//
//    |     1      3     5        8
//    |     o------o     o--------o         speed_limit: 4
//    |         2         6
//    |     o---x--o------x                 speed_limit: 3
//    |
//    |     o------o     o--------o         speed_limit: 2
//    |
//    |------------------------------------->s
//
TEST(SpeedLimitTest, Test_5) {
  std::vector<Range> ranges;
  ranges.reserve(2);
  ranges.push_back({.start_s = 1.0, .end_s = 3.0, .speed_limit = 3.0});
  ranges.push_back({.start_s = 2.0, .end_s = 6.0, .speed_limit = 3.0});
  ranges.push_back({.start_s = 5.0, .end_s = 8.0, .speed_limit = 2.0});
  ranges.push_back({.start_s = 1.0, .end_s = 3.0, .speed_limit = 2.0});
  ranges.push_back({.start_s = 1.0, .end_s = 3.0, .speed_limit = 4.0});
  ranges.push_back({.start_s = 5.0, .end_s = 8.0, .speed_limit = 4.0});

  SpeedLimit st_speed_limit(ranges, kDefaultSpeedLimit);

  const auto speed_limit = st_speed_limit.GetSpeedLimitByS(1.0);
  EXPECT_EQ(speed_limit, 2.0);

  const auto speed_limit1 = st_speed_limit.GetSpeedLimitByS(2.0);
  EXPECT_EQ(speed_limit1, 2.0);

  const auto speed_limit2 = st_speed_limit.GetSpeedLimitByS(3.5);
  EXPECT_EQ(speed_limit2, 3.0);

  const auto speed_limit3 = st_speed_limit.GetSpeedLimitByS(6.0);
  EXPECT_EQ(speed_limit3, 2.0);

  const auto speed_limit4 = st_speed_limit.GetSpeedLimitByS(8.0);
  EXPECT_EQ(speed_limit4, 2.0);
}

}  // namespace
}  // namespace qcraft::planner
