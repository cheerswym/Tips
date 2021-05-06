#include "onboard/planner/initializer/ref_speed_table.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"

namespace qcraft::planner {
namespace {

TEST(RefSpeedTable, RefSpeedVecTest) {
  FLAGS_planner_increase_lane_speed_limit_fraction = 0.0;

  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  const TestRouteResult route_result =
      CreateAContinuousLaneChangeRouteWithSolidInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());

  ASSIGN_OR_DIE(
      const auto drive_passage,
      BuildDrivePassage(psmm, route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        /*keep_behind_len=*/10.0));
  SendDrivePassageToCanvas(drive_passage, "passage_straight");

  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("Phantom0")
                            .set_type(OT_VEHICLE)
                            .set_pos(Vec2d(160.0, 3.5))
                            .set_yaw(0.0)
                            .set_timestamp(0.0)
                            .set_velocity(5.0)
                            .set_length_width(4.0, 2.0)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.7)
      .set_straight_line(Vec2d(160.0, 3.5), Vec2d(210.0, 3.5),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  objects.push_back(builder.Build());

  ObjectVector<PlannerObject> obj_vec;
  obj_vec.push_back(objects.front());
  DrawPlannerObjectManagerToCanvas(PlannerObjectManager(obj_vec),
                                   "leading_vehicle", vis::Color::kLightGreen);
  const SpacetimeTrajectoryManager st_traj_mgr(objects);

  ConstraintProto::LeadingObjectProto leading_obj;
  *leading_obj.mutable_traj_id() = "Phantom0-idx0";
  leading_obj.set_reason(
      ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE);

  ConstraintManager c_mgr;
  c_mgr.AddLeadingObject(std::move(leading_obj));

  const RefSpeedTable ref_speed_table(c_mgr, st_traj_mgr, drive_passage,
                                      /*stop_s=*/{60.0});

  SendRefSpeedTableToCanvas(ref_speed_table, drive_passage);

  const auto lookup_0 = ref_speed_table.LookUpRefSpeed(0.0, 20.0);
  EXPECT_NEAR(lookup_0.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_0.second, 6.5, 0.1);

  const auto lookup_1 = ref_speed_table.LookUpRefSpeed(0.0, 30.0);
  EXPECT_NEAR(lookup_1.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_1.second, 5.0, 0.1);

  const auto lookup_2 = ref_speed_table.LookUpRefSpeed(3.0, 20.0);
  EXPECT_NEAR(lookup_2.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_2.second, 8.5, 0.1);

  const auto lookup_3 = ref_speed_table.LookUpRefSpeed(3.0, 30.0);
  EXPECT_NEAR(lookup_3.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_3.second, 7.2, 0.1);

  const auto lookup_4 = ref_speed_table.LookUpRefSpeed(4.0, 40.0);
  const auto lookup_5 = ref_speed_table.LookUpRefSpeed(9.0, 40.0);
  EXPECT_NEAR(lookup_4.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_5.first, 16.7, 0.1);
  EXPECT_NEAR(lookup_4.second, lookup_5.second, 1e-6);
}

}  // namespace
}  // namespace qcraft::planner
