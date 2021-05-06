#include "onboard/planner/scheduler/smooth_reference_line_result.h"

#include "gtest/gtest.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"

namespace qcraft::planner {
namespace {

TEST(SmoothedReferenceLineResultMap, SmoothResultTest) {
  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  SmoothedReferenceLineResultMap results;

  {
    const mapping::ElementId id = 7814;
    const std::vector<mapping::ElementId> lane_ids = {id};
    const auto smoothed_line_or = SmoothLanePathByLaneIds(
        planner_semantic_map_manager, lane_ids, /*half_av_width=*/0.0);

    EXPECT_OK(smoothed_line_or);
    EXPECT_TRUE(!results.Contains(lane_ids));
    results.AddResult(lane_ids, *smoothed_line_or);
    EXPECT_TRUE(results.Contains(lane_ids));
    results.DeleteResult(lane_ids);
    EXPECT_TRUE(!results.Contains(lane_ids));
  }

  {
    const mapping::ElementId id = 7806;
    const std::vector<mapping::ElementId> lane_ids = {id};
    const auto smoothed_line_or = SmoothLanePathByLaneIds(
        planner_semantic_map_manager, lane_ids, /*half_av_width=*/0.0);

    EXPECT_OK(smoothed_line_or);
    EXPECT_TRUE(!results.Contains(lane_ids));
    results.AddResult(lane_ids, *smoothed_line_or);
    EXPECT_TRUE(results.Contains(lane_ids));
    results.DeleteResult(lane_ids);
    EXPECT_TRUE(!results.Contains(lane_ids));
  }
}

}  // namespace
}  // namespace qcraft::planner
