#include "onboard/planner/router/route_util.h"

#include <regex>

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/router/route_searcher.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteUtilTest, ForwardExtendLanePathTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const mapping::LanePath lp(&semantic_map_manager, {14}, 0.0, 1.0);
  const auto forward = ForwardExtendLanePath(semantic_map_manager, lp, 60.0);
  EXPECT_EQ(forward.size(), 3);
  EXPECT_EQ(forward.lane_id(0), 14);
  EXPECT_EQ(forward.lane_id(1), 59);
  EXPECT_EQ(forward.lane_id(2), 2476);
}

TEST(RouteUtilTest, FindLanePathFromLaneAlongRouteSections) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapManager psmm(&semantic_map_manager,
                                 PlannerSemanticMapModification());
  const RouteSections route_sections(0.0, 1.0,
                                     {12437, 12439, 12438, 12215, 12211},
                                     mapping::LanePoint(246, 1.0));
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});
  const auto lane_path =
      FindLanePathFromLaneAlongRouteSections(psmm, sections_info, 155, 100.0);
  EXPECT_NEAR(lane_path.length(), 100.0, 1e-3);
  EXPECT_EQ(lane_path.back().lane_id(), 246);
}

TEST(RouteUtilTest, TravelDistanceBetween) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  mapping::LanePath lane_path0(&smm, {169, 130}, 0.0, 0.5);
  mapping::LanePath lane_path1(&smm, {129}, 0.5, 0.9);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0,
      .lc_left = false,
      .lc_section_id = 3047,
  };
  CompositeLanePath clp({lane_path0, lane_path1}, {trans1});

  const double cal_distance =
      TravelDistanceBetween(smm, clp, CompositeLanePath::CompositeIndex(0, 0),
                            0.0, CompositeLanePath::CompositeIndex(1, 0), 0.5);
  const double expected_length = smm.FindLaneInfoOrDie(169).length() +
                                 smm.FindLaneInfoOrDie(130).length() * 0.5 +
                                 smm.FindLaneInfoOrDie(129).length() * 0.5;
  EXPECT_TRUE(fabs(cal_distance - expected_length) < 0.1);
}

TEST(RouteUtilTest, IsTravelMatchedValid) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  mapping::LanePath lane_path0(&smm, {169, 130}, 0.0, 0.5);
  mapping::LanePath lane_path1(&smm, {129}, 0.5, 0.9);
  CompositeLanePath::TransitionInfo trans1 = {
      .overlap_length = 0.0,
      .lc_left = false,
      .lc_section_id = 3047,
  };
  CompositeLanePath clp({lane_path0, lane_path1}, {trans1});

  EXPECT_TRUE(IsTravelMatchedValid(clp, CompositeLanePath::CompositeIndex(0, 0),
                                   0.0, CompositeLanePath::CompositeIndex(1, 0),
                                   0.5, 1000000, 6000000, 63.2644, 23, 200));
}

TEST(RouteUtilTest, SearchConnectLaneInSection) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const auto connect_lanes_in_section_or =
      SearchConnectLaneInSection(smm, 2, 3, false);
  std::vector<mapping::ElementId> res;
  if (!connect_lanes_in_section_or.ok()) {
    EXPECT_TRUE(false);
  } else {
    for (const auto &item : connect_lanes_in_section_or.value()) {
      res.push_back(item.id);
    }
  }
  EXPECT_THAT(std::vector<mapping::ElementId>({3, 2448, 2}),
              testing::ElementsAreArray(res));
}

TEST(RouteUtilTest, IsTwoLaneConnectedWithBrokenWhites) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  EXPECT_TRUE(IsTwoLaneConnectedWithBrokenWhites(smm, 2, 3));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
