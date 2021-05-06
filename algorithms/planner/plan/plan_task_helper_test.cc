#include "onboard/planner/plan/plan_task_helper.h"

#include "gtest/gtest.h"

namespace qcraft::planner {

namespace {

TEST(SplitCruiseByUturnTask, BasicTest) {
  SetMap("dojo");
  auto smm = std::make_unique<mapping::SemanticMapManager>();
  smm->LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(smm.get(), psmm_modifier);

  {
    const mapping::LanePath lane_path(smm.get(), {6564, 6697, 6643}, 0.8, 1.0);
    const mapping::LanePoint dest(6643, 1.0);

    const auto new_tasks_or = SplitCruiseByUturnTask(psmm, lane_path, dest);

    EXPECT_OK(new_tasks_or);
    EXPECT_EQ(new_tasks_or->size(), 3);
    EXPECT_EQ((*new_tasks_or)[0].type(), ON_ROAD_CRUISE_PLAN);
    EXPECT_EQ((*new_tasks_or)[1].type(), UTURN_PLAN);
    EXPECT_EQ((*new_tasks_or)[2].type(), ON_ROAD_CRUISE_PLAN);

    EXPECT_EQ((*new_tasks_or)[1]
                  .destination_info()
                  .dest.lane_points->front()
                  .lane_id(),
              6643);
    EXPECT_TRUE(
        (*new_tasks_or)[1].destination_info().uturn_ref_lane_path.has_value());
  }

  {
    // Destination is on uturn
    const mapping::LanePath lane_path(smm.get(), {6564, 6697}, 0.8, 0.7);
    const mapping::LanePoint dest(6697, 0.7);
    const auto new_tasks_or = SplitCruiseByUturnTask(psmm, lane_path, dest);
    EXPECT_OK(new_tasks_or);
    EXPECT_EQ(new_tasks_or->size(), 2);
    EXPECT_EQ((*new_tasks_or)[1].destination_info().dest.lane_points->front(),
              lane_path.back());
  }
}

}  // namespace

}  // namespace qcraft::planner
