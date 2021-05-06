#include "onboard/planner/smooth_lane.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace planner {

TEST(SmoothLaneTest, DISABLED_InterpolationConsistencyTest) {
  mapping::ElementId id = 69;
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  SmoothLane lane(semantic_map_manager, id);
  const std::vector<double> smooth_s = lane.smooth_s();
  const std::vector<Vec3d> smooth_points = lane.smooth_points();
  const std::vector<Vec3d> newly_interpolated_points = lane.Sample(smooth_s);
  ASSERT_EQ(smooth_s.size(), smooth_points.size());
  ASSERT_EQ(smooth_s.size(), newly_interpolated_points.size());
  for (int i = 0; i < smooth_s.size(); ++i) {
    EXPECT_EQ(smooth_points[i], newly_interpolated_points[i]);
  }
}

}  // namespace planner
}  // namespace qcraft
