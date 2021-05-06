#include "onboard/planner/router/route_searcher.h"

#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/router/route.h"

namespace qcraft {
namespace planner {
namespace {

/*
CompositeLanePath TestSearchForRouteLanePath(
    const mapping::LanePoint &start, const mapping::LanePoint &dest_pt) {
  RoutingRequestProto request;
  RoutingDestinationProto dest;
  dest.mutable_lane_point()->set_lane_id(dest_pt.lane_id());
  dest.mutable_lane_point()->set_fraction(dest_pt.fraction());
  request.mutable_destinations()->Add(std::move(dest));

  CompositeLanePath result;
  const bool success = Route::SearchForRoutePath(start, request, &result);
  LOG(INFO) << result.DebugString();
  EXPECT_TRUE(success);

  return result;
}
*/

TEST(RouteSearcherTest, RoutedLCReachability) {
  /*
    SetMap("dojo");
    SemanticMapManager::Instance().Reload();
    FLAGS_allow_routed_lane_change = true;

    CompositeLanePath route_lane_path;

    // --------------  TEST 1 -----------------------//
    LOG(INFO) << "Forced lane change from end of a lane";
    // route_lane_path = TestSearchForRouteLanePath(mapping::LanePoint(1694,
    0.9),
    // mapping::LanePoint(1847, 0.0));

    // --------------  TEST 2 -----------------------//
    LOG(INFO) << "Destination is behind start point on the same lane";
    route_lane_path = TestSearchForRouteLanePath(mapping::LanePoint(1503, 0.9),
                                                 mapping::LanePoint(1503, 0.0));

    // --------------  TEST 3 -----------------------//
    LOG(INFO) << "Destination is ahead of start point on the same lane";
    route_lane_path = TestSearchForRouteLanePath(mapping::LanePoint(1503, 0.1),
                                                 mapping::LanePoint(1503, 0.5));

    // --------------  TEST 4 -----------------------//
    LOG(INFO) << "Expect a lane change to reach destination rather than making a
    " "detour"; route_lane_path =
    TestSearchForRouteLanePath(mapping::LanePoint(1525, 0.0),
                                                 mapping::LanePoint(1523, 1.0));
    EXPECT_LT(route_lane_path.length(), 100.0);
  */
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
