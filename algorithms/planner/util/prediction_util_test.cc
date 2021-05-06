#include "onboard/planner/util/prediction_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {
TEST(AlignPredictionTime, Works) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(1.0, 0.0))
                                  .set_timestamp(10.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();
  ObjectPredictionBuilder builder;
  builder.set_object(perception_obj)
      .add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d(1.0, 0.0), Vec2d(10.0, 0.0),
                         /*init_v=*/10.0, /*last_v=*/10.0);
  builder.add_predicted_trajectory()->set_probability(0.3).set_straight_line(
      Vec2d::Zero(), Vec2d(0.0, 10.0),
      /*init_v=*/0.2, /*last_v=*/0.2);

  const auto object_prediction = builder.Build();

  ObjectPredictionProto pred_proto;
  object_prediction.ToProto(&pred_proto);
  ASSERT_EQ(pred_proto.trajectories().size(), 2);
  EXPECT_EQ(pred_proto.trajectories()[0].points_size(), 10);
  EXPECT_EQ(pred_proto.trajectories()[1].points_size(), 500);
  const int num_trajs = pred_proto.trajectories().size();
  ObjectPredictionProto& aligned_proto = pred_proto;
  ASSERT_OK(AlignPredictionTime(/*current_time=*/10.1, &aligned_proto));
  EXPECT_EQ(aligned_proto.perception_object().timestamp(), 10.1);

  ASSERT_EQ(aligned_proto.trajectories().size(), num_trajs);

  const auto& first_aligned_traj = aligned_proto.trajectories()[0];
  ASSERT_EQ(first_aligned_traj.points_size(), 9);

  for (int i = 0; i < aligned_proto.trajectories().size(); ++i) {
    const auto& aligned_traj = aligned_proto.trajectories()[i];
    ASSERT_EQ(aligned_traj.points(0).s(), 0.0);
    for (int j = 0; j < aligned_traj.points().size(); ++j) {
      ASSERT_NEAR(aligned_traj.points(j).t(), kTrajectoryTimeStep * j, 1e-8);
    }
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
