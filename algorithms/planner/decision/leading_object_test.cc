#include "onboard/planner/decision/leading_object.h"

#include "gtest/gtest.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from `dojo.planner.leading_object_1.pb.txt`
// Description: Agents not on right most lane, leading us.
TEST(FindLeadingObjectsTest, AgentsNotOnRightMostLane) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(4);
  for (int i = 0; i < 4; ++i) {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj =
        perception_builder.set_id(absl::StrFormat("Agent%d", i))
            .set_type(OT_VEHICLE)
            .set_timestamp(1.0)
            .set_velocity(0.0)
            .set_yaw(0.0)
            .set_length_width(4.5, 2.0)
            .set_pos(Vec2d(160.6 + i * 10.0, 66.6))
            .set_box_center(Vec2d(160.6 + i * 10.0, 66.6))
            .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(120.0);
  pose.mutable_pos_smooth()->set_y(66.5);
  pose.mutable_vel_smooth()->set_x(5.0);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"a8_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto& passage = *drive_passage;
  const auto path_sl_boundary = CreateFakePathSlBoundary(passage);
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  ApolloTrajectoryPointProto plan_start_point;
  // Start position must in the range of drive passage.
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x() + 10.0);
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());

  // Calculate av frenet box.
  const Box2d av_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box, passage.QueryFrenetBoxAt(av_box));

  const auto leading_objects =
      FindLeadingObjects(vehicle_geometry_params, planner_semantic_map_manager,
                         /*stalled_objects=*/{},
                         /*scene reasoning=*/{}, passage, path_sl_boundary,
                         std::nullopt, st_traj_mgr, plan_start_point,
                         av_frenet_box, /*borrow_lane_boundary=*/false);

  EXPECT_EQ(leading_objects.size(), 4);
  EXPECT_EQ(leading_objects[0].reason(),
            ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE);
}

// Data generated from `dojo.planner.leading_object_2.pb.txt`
// Description: Agents waiting for traffic light, leading us.
TEST(FindLeadingObjectsTest, AgentsWaitingForTL) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(5);
  for (int i = 0; i < 5; ++i) {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj =
        perception_builder.set_id(absl::StrFormat("Agent%d", i))
            .set_type(OT_VEHICLE)
            .set_timestamp(1.0)
            .set_velocity(0.0)
            .set_yaw(0.0)
            .set_length_width(4.5, 2.0)
            .set_pos(Vec2d(160.6 + i * 10.0, -3.5))
            .set_box_center(Vec2d(160.6 + i * 10.0, -3.5))
            .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(119.7);
  pose.mutable_pos_smooth()->set_y(-3.5);
  pose.mutable_vel_smooth()->set_x(11.1);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"a8_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto& passage = *drive_passage;
  const auto path_sl_boundary = CreateFakePathSlBoundary(passage);

  ApolloTrajectoryPointProto plan_start_point;
  // Start position must in the range of drive passage.
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x() + 10.0);
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());

  // Calculate av frenet box.
  const Box2d av_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box, passage.QueryFrenetBoxAt(av_box));

  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  const auto leading_objects =
      FindLeadingObjects(vehicle_geometry_params, planner_semantic_map_manager,
                         /*stalled_objects=*/{},
                         /*scene reasoning=*/{}, passage, path_sl_boundary,
                         std::nullopt, st_traj_mgr, plan_start_point,
                         av_frenet_box, /*borrow_lane_boundary=*/false);
  EXPECT_EQ(leading_objects.size(), 5);
  for (int i = 0; i < leading_objects.size(); ++i) {
    const std::string traj_id = SpacetimeObjectTrajectory::MakeTrajectoryId(
        absl::StrFormat("Agent%d", i), 0);
    EXPECT_EQ(leading_objects[i].traj_id(), traj_id);
  }
}

// Data generated from `dojo.planner.leading_object_3.pb.txt`
// Description: Agent0~agent3 blocking. Agent4 is not stationary, leading us.
TEST(FindLeadingObjectsTest, AgentNotStationary) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  absl::flat_hash_set<std::string> stalled_objects;

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(5);
  for (int i = 0; i < 4; ++i) {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj =
        perception_builder.set_id(absl::StrFormat("Agent%d", i))
            .set_type(OT_VEHICLE)
            .set_timestamp(1.0)
            .set_velocity(0.0)
            .set_yaw(0.0)
            .set_length_width(4.5, 2.0)
            .set_pos(Vec2d(-342.9 + i * 10.0, 206.5))
            .set_box_center(Vec2d(-342.9 + i * 10.0, 206.5))
            .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
    stalled_objects.insert(perception_obj.id());
  }
  {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("Agent4")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(1.0)
                                    .set_yaw(0.0)
                                    .set_length_width(4.5, 2.0)
                                    .set_pos(Vec2d(-231.7, 206.5))
                                    .set_box_center(Vec2d(-231.7, 206.5))
                                    .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  ASSERT_EQ(st_traj_mgr.trajectories().size(), 5);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(-390.3);
  pose.mutable_pos_smooth()->set_y(206.7);
  pose.mutable_vel_smooth()->set_x(11.1);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"hw_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto& passage = *drive_passage;
  const auto path_sl_boundary = CreateFakePathSlBoundary(passage);

  ApolloTrajectoryPointProto plan_start_point;
  // Start position must in the range of drive passage.
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x() + 10.0);
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());

  // Calculate av frenet box.
  const Box2d av_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box, passage.QueryFrenetBoxAt(av_box));

  const auto leading_objects = FindLeadingObjects(
      vehicle_geometry_params, planner_semantic_map_manager, stalled_objects,
      /*scene reasoning=*/{}, passage, path_sl_boundary, std::nullopt,
      st_traj_mgr, plan_start_point, av_frenet_box,
      /*borrow_lane_boundary=*/false);
  ASSERT_EQ(leading_objects.size(), 1);
  const std::string traj_id =
      SpacetimeObjectTrajectory::MakeTrajectoryId("Agent4", 0);
  EXPECT_EQ(leading_objects[0].traj_id(), traj_id);
}

// Data generated from `dojo.planner.leading_object_4.pb.txt`
// Description: Agent1 occupy av lane less than kAllowedOccupyDistance. Not
// leading.
TEST(FindLeadingObjectsTest, AgentOccupyAvLaneSlightly) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(1);
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj =
      perception_builder.set_id(absl::StrFormat("Agent%d", 1))
          .set_type(OT_VEHICLE)
          .set_timestamp(1.0)
          .set_velocity(0.0)
          .set_yaw(0.0)
          .set_length_width(4.5, 2.0)
          .set_pos(Vec2d(170.5, 64.2))
          .set_box_center(Vec2d(170.5, 64.2))
          .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                            perception_obj.yaw())
      .set_probability(0.5);

  PlannerObject object = builder.Build();
  planner_objects.push_back(std::move(object));

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(120.0);
  pose.mutable_pos_smooth()->set_y(66.5);
  pose.mutable_vel_smooth()->set_x(11.1);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"a8_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto& passage = *drive_passage;
  const auto path_sl_boundary = CreateFakePathSlBoundary(passage);

  ApolloTrajectoryPointProto plan_start_point;
  // Start position must in the range of drive passage.
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x() + 10.0);
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());

  // Calculate av frenet box.
  const Box2d av_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box, passage.QueryFrenetBoxAt(av_box));

  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  const auto leading_objects =
      FindLeadingObjects(vehicle_geometry_params, planner_semantic_map_manager,
                         /*stalled_objects=*/{},
                         /*scene reasoning=*/{}, passage, path_sl_boundary,
                         std::nullopt, st_traj_mgr, plan_start_point,
                         av_frenet_box, /*borrow_lane_boundary=*/false);
  EXPECT_EQ(leading_objects.size(), 0);
}

// Data generated from `dojo.planner.leading_object_5.pb.txt`
// Description: Agent0 is oncoming, neither leading nor blocking object.
TEST(FindLeadingObjectsTest, AgentIsOncoming) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(1);
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_speed(Vec2d(-15.0, 0.0))
                                  .set_yaw(3.14)
                                  .set_length_width(4.5, 2.0)
                                  .set_pos(Vec2d(300.0, 5.5))
                                  .set_box_center(Vec2d(300.0, 5.5))
                                  .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_straight_line(Vec2d(300.0, 5.5), Vec2d(50.0, 5.5), /*init_v=*/15.0,
                          /*last_v=*/15.0)
      .set_probability(0.5);

  PlannerObject object = builder.Build();
  planner_objects.push_back(std::move(object));

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(120.0);
  pose.mutable_pos_smooth()->set_y(3.5);
  pose.mutable_vel_smooth()->set_x(11.1);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  const auto route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                            /*name_spot=*/"b9_w1_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  const auto& passage = *drive_passage;
  const auto path_sl_boundary = CreateFakePathSlBoundary(passage);

  ApolloTrajectoryPointProto plan_start_point;
  // Start position must in the range of drive passage.
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x() + 10.0);
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());

  // Calculate av frenet box.
  const Box2d av_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box, passage.QueryFrenetBoxAt(av_box));

  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  const auto leading_objects =
      FindLeadingObjects(vehicle_geometry_params, planner_semantic_map_manager,
                         /*stalled_objects=*/{},
                         /*scene reasoning=*/{}, passage, path_sl_boundary,
                         std::nullopt, st_traj_mgr, plan_start_point,
                         av_frenet_box, /*borrow_lane_boundary=*/false);
  EXPECT_EQ(leading_objects.size(), 0);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
