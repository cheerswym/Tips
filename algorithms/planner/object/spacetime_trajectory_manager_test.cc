#include "onboard/planner/object/spacetime_trajectory_manager.h"

#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/util/prediction_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner {
namespace {

TEST(SpacetimeTrajectoryManagerTest, CreateSpacetimeObject) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("abc")
                            .set_type(OT_VEHICLE)
                            .set_timestamp(1.0)
                            .set_velocity(2.0)
                            .set_yaw(0.0)
                            .set_length_width(4.0, 2.0)
                            .set_box_center(Vec2d::Zero())
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  objects.push_back(builder.Build());

  perception_obj.set_id("def");
  objects.push_back(PlannerObjectBuilder()
                        .set_object(perception_obj)
                        .set_stationary(true)
                        .Build());

  SpacetimeTrajectoryManager mgr(objects);

  // TODO(lidong): How to draw the planner object and spacetime objects in unit
  // test?

  const auto& st_trajs = mgr.trajectories();
  ASSERT_EQ(st_trajs.size(), 3);

  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("abc").size(), 2);
  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("def").size(), 1);
  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("hig").size(), 0);
  EXPECT_EQ(mgr.FindTrajectoryById("abc-idx1")->traj_id(), "abc-idx1");
  EXPECT_EQ(mgr.FindTrajectoryById("abc-def"), nullptr);
  EXPECT_EQ(mgr.FindObjectByObjectId("abc")->id(), "abc");
  EXPECT_EQ(mgr.FindObjectByObjectId("xyz"), nullptr);

  const auto& first_states = st_trajs[0]->states();
  for (int i = 0; i < first_states.size(); ++i) {
    const auto& state = first_states[i];
    ASSERT_NEAR(state.traj_point->theta(), 0.0, 1e-8);
    ASSERT_NEAR(state.box.heading(), 0.0, 1e-8);
    ASSERT_NEAR(state.box.length(), 4.0, 1e-8);
    ASSERT_NEAR(state.box.width(), 2.0, 1e-8);
    ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
  }

  const auto& second_states = st_trajs[1]->states();
  for (int i = 0; i < second_states.size(); ++i) {
    const auto& state = second_states[i];
    ASSERT_NEAR(state.traj_point->theta(), M_PI_2, 1e-8);
    ASSERT_NEAR(state.box.heading(), M_PI_2, 1e-8);
    ASSERT_NEAR(state.box.length(), 4.0, 1e-8);
    ASSERT_NEAR(state.box.width(), 2.0, 1e-8);
    ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
  }

  // This is a stationary object.
  const auto& third_states = st_trajs[2]->states();
  ASSERT_EQ(third_states.size(), 100);
  const auto& state = third_states[0];
  ASSERT_EQ(state.box.heading(), 0.0);
  ASSERT_EQ(state.box.length(), 4.0);
  ASSERT_EQ(state.box.width(), 2.0);
  ASSERT_THAT(state.box.center(), Vec2dEqXY(0.0, 0.0));
  ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
}
}  // namespace
}  // namespace qcraft::planner
