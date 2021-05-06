#include "onboard/planner/object/planner_object_manager_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/container/strong_vector.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PlannerObjectManagerBuilder, MergePerceptionAndPrediction) {
  ObjectsProto perception_objects;
  const auto object1 = PerceptionObjectBuilder()
                           .set_type(OT_FOD)
                           .set_id("1")
                           .set_timestamp(1.0)
                           .Build();
  *perception_objects.add_objects() = object1;
  const auto object2 = PerceptionObjectBuilder()
                           .set_id("2")
                           .set_type(OT_VEHICLE)
                           .set_timestamp(1.1)
                           .Build();
  *perception_objects.add_objects() = object2;

  ObjectPredictionBuilder pred_builder;
  auto object3 = object2;
  object3.set_timestamp(0.9);
  pred_builder.set_object(object3)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  const auto object_pred = pred_builder.Build();
  ObjectsPredictionProto objects_pred;
  object_pred.ToProto(objects_pred.add_objects());

  FilteredTrajectories filtered_trajs;
  auto planner_objects = BuildPlannerObjects(&perception_objects, &objects_pred,
                                             /*align_time=*/1.2);
  const auto mgr = PlannerObjectManagerBuilder()
                       .set_planner_objects(std::move(planner_objects))
                       .Build(&filtered_trajs);
  EXPECT_OK(mgr) << mgr.status().message();
  EXPECT_EQ(mgr->size(), 2);
  EXPECT_EQ(mgr->stationary_objects().size(), 1);
  for (const auto& planner_object : mgr->planner_objects()) {
    ASSERT_EQ(planner_object.object_proto().timestamp(), 1.2);
    ASSERT_EQ(planner_object.prediction().perception_object().timestamp(), 1.2);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
