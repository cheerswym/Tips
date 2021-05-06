#include "onboard/planner/common/lane_path_info.h"

#include "gtest/gtest.h"

namespace qcraft::planner {

namespace {

TEST(LanePathInfo, LanePathInfoQueryTest) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const LanePathInfo lane_path_info(mapping::LanePath(&smm, {2471}, 0.0, 1.0),
                                    /*len_along_route=*/0.0, /*path_cost=*/0.0,
                                    smm);
  const auto sl =
      lane_path_info.ProjectionSLInRange(Vec2d(116.041, -3.524), 0.0, 10.0);

  EXPECT_NEAR(sl.s, 0.5, 0.1);
  EXPECT_NEAR(sl.l, -3.5, 0.1);
}

}  // namespace

}  // namespace qcraft::planner
