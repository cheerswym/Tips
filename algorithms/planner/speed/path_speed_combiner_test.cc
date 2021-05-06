#include "onboard/planner/speed/path_speed_combiner.h"

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(PathSpeedCombinerTest, SimpleTest) {
  constexpr int path_size = 50;
  constexpr int speed_size = 60;
  constexpr int trajectory_size = 590;
  std::vector<ApolloTrajectoryPointProto> trajectory_data;

  std::vector<PathPoint> path_points;
  std::vector<SpeedPoint> speed_points;
  for (int i = 0; i < path_size; ++i) {
    PathPoint temp;
    temp.set_x(0.2 * i);
    temp.set_y(0.0);
    temp.set_s(0.2 * i);
    path_points.push_back(temp);
  }
  for (int j = 0; j < speed_size; ++j) {
    SpeedPoint temp;
    temp.set_s(0.15 * j);
    temp.set_t(j);
    temp.set_v(0.1);
    temp.set_a(0.1);
    temp.set_j(0.1);
    speed_points.push_back(temp);
  }

  DiscretizedPath path_data(path_points);
  SpeedVector speed_data(speed_points);

  EXPECT_EQ(path_data.size(), path_size);
  EXPECT_EQ(speed_data.size(), speed_size);

  absl::Status status = CombinePathAndSpeed(path_data, /*forward=*/true,
                                            speed_data, &trajectory_data);

  EXPECT_TRUE(status.ok());
  EXPECT_EQ(trajectory_data.size(), trajectory_size);
}

}  // namespace
}  // namespace qcraft::planner
