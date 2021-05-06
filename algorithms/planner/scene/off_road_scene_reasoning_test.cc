#include "onboard/planner/scene/off_road_scene_reasoning.h"

#include "gtest/gtest.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
TEST(OffRoadSceneReasoning, ObjectReasoningTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&semantic_map_manager, psmm_modifier);

  // Build stall objects.
  ObjectsProto perception;
  PerceptionObjectBuilder perception_object_builder;
  auto object_cone =
      perception_object_builder.set_id("agent 1").set_type(OT_CONE).Build();
  *perception.add_objects() = std::move(object_cone);

  auto object_barrier =
      perception_object_builder.set_id("agent 2").set_type(OT_BARRIER).Build();
  *perception.add_objects() = std::move(object_barrier);

  auto object_warning_triangle = perception_object_builder.set_id("agent 3")
                                     .set_type(OT_WARNING_TRIANGLE)
                                     .Build();
  *perception.add_objects() = std::move(object_warning_triangle);

  auto planner_objects =
      BuildPlannerObjects(&perception, /*prediction*/ nullptr,
                          /*aligntime*/ std::nullopt);

  PlannerObjectManager object_mgr(planner_objects);

  ASSIGN_OR_DIE(const auto scene_reasoning,
                RunOffRoadSceneReasoning(OffRoadSceneReasoningInput{
                    .psmm = &psmm, .object_mgr = &object_mgr}));

  EXPECT_EQ(scene_reasoning.objects_annotation_size(), 3);
}
}  // namespace
}  // namespace qcraft::planner
