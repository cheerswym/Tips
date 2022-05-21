#include "onboard/planner/object/planner_object_manager.h"

#include <optional>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/container/strong_vector.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PlannerObjectManager, Empty) {
  PlannerObjectManager mgr;
  ASSERT_EQ(mgr.size(), 0);
}

TEST(PlannerObjectManager, OneObject) {
  const auto object = PerceptionObjectBuilder().set_id("o").Build();
  ObjectsProto perception_objects;
  *perception_objects.add_objects() = object;

  auto planner_objects =
      BuildPlannerObjects(&perception_objects, /*prediction=*/nullptr,
                          /*align_time=*/std::nullopt);

  FilteredTrajectories filtered_trajs;
  const auto mgr = PlannerObjectManagerBuilder()
                       .set_planner_objects(std::move(planner_objects))
                       .Build(&filtered_trajs)
                       .value();
  ASSERT_EQ(mgr.size(), 1);
  const auto index = ObjectIndex(0);
  EXPECT_EQ(mgr.planner_object(index).id(), "o");
}

TEST(PlannerObjectManager, TwoObjects) {
  ObjectsProto perception_objects;
  const auto object1 = PerceptionObjectBuilder().set_id("1").Build();
  *perception_objects.add_objects() = object1;
  const auto object2 = PerceptionObjectBuilder().set_id("2").Build();
  *perception_objects.add_objects() = object2;

  auto planner_objects =
      BuildPlannerObjects(&perception_objects, /*prediction=*/nullptr,
                          /*align_time=*/std::nullopt);

  FilteredTrajectories filtered_trajs;
  auto mgr = PlannerObjectManagerBuilder()
                 .set_planner_objects(std::move(planner_objects))
                 .Build(&filtered_trajs)
                 .value();
  ASSERT_EQ(mgr.size(), 2);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
