#include "onboard/planner/test_util/route_builder.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"

namespace qcraft {
namespace planner {

const PoseProto car_pose(double smooth_x = 0.0, double smooth_y = 0.0,
                         double heading = 0.0, double smooth_v_x = 0.0,
                         double smooth_v_y = 0.0) {
  PoseProto car_pose;
  car_pose.mutable_pos_smooth()->set_x(smooth_x);
  car_pose.mutable_pos_smooth()->set_y(smooth_y);
  car_pose.set_yaw(heading);
  car_pose.mutable_vel_smooth()->set_x(smooth_v_x);
  car_pose.mutable_vel_smooth()->set_y(smooth_v_y);
  return car_pose;
}

bool route_is_expected(const CompositeLanePath &this_route,
                       const CompositeLanePath &expected_route) {
  bool route_correct = false;
  constexpr double kFractionThreshold = 0.03;
  LOG(INFO) << "this route:" << this_route.DebugString();
  LOG(INFO) << "expected route:" << expected_route.DebugString();
  if (this_route.lane_paths().size() != expected_route.lane_paths().size()) {
    LOG(INFO) << " this route size:" << this_route.lane_paths().size()
              << " expected route size:" << expected_route.lane_paths().size();
  } else {
    for (int i = 0; i < expected_route.lane_paths().size(); ++i) {
      if (expected_route.lane_path(i).size() != this_route.lane_path(i).size())
        break;
      if (std::fabs(expected_route.lane_path(i).start_fraction() -
                        this_route.lane_path(i).start_fraction() >
                    kFractionThreshold) ||
          std::fabs(expected_route.lane_path(i).end_fraction() -
                        this_route.lane_path(i).end_fraction() >
                    kFractionThreshold))
        break;
      for (int j = 0; j < expected_route.lane_path(i).size(); ++j) {
        if (expected_route.lane_path(i).lane_id(j) !=
            this_route.lane_path(i).lane_id(j))
          break;
        route_correct = true;
      }
    }
  }
  return route_correct;
}

TEST(RouteBuilderTest, RouteToNameSpot) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  // route to name spot "8a_n2"
  CompositeLanePath expected_route_path(
      {mapping::LanePath(&semantic_map_manager, {96, 2, 938, 940, 2470},
                         0.143225, 1.0),
       mapping::LanePath(&semantic_map_manager, {2471, 53, 187, 114, 136, 142},
                         1.0, 0.499336)},
      CompositeLanePath::kDefaultTransition);

  const auto route = RoutingToNameSpot(semantic_map_manager,
                                       car_pose(-33.015, 3.423), "8a_n2");

  EXPECT_TRUE(route_is_expected(route, expected_route_path))
      << "route is not correct";
}

}  // namespace planner
}  // namespace qcraft
