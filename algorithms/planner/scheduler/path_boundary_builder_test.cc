#include "onboard/planner/scheduler/path_boundary_builder.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {

namespace {

constexpr double kEpsilon = 0.1;  // m

PlannerObjectManager BuildPhantomVehicle(const PlannerSemanticMapManager &psmm,
                                         Vec2d obj_pos) {
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(0.0)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(obj_pos, 0.0);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

TEST(BuildPathBoundaryFromPose, BuildPathBoundaryFromPoseTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);
  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        /*keep_behind_len=*/10.0);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(), "multi/straight_drive_passage");

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(
        route_result.pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    const auto path_bound_or = BuildPathBoundaryFromPose(
        planner_semantic_map_manager, dp_or.value(), plan_start_point,
        veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

    EXPECT_OK(path_bound_or);

    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/straight_path_boundary");
  }

  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(118.286);
    plan_start_point.mutable_path_point()->set_y(3.23);
    plan_start_point.mutable_path_point()->set_theta(-0.17);
    plan_start_point.set_v(0.0);

    lc_state.set_lc_left(false);

    {
      lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/straight_path_boundary_lc");
    }

    {
      lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/straight_path_boundary_lcp");
    }
  }
}

TEST(BuildPathBoundaryFromPose, SolidLineTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result =
      CreateAStraightForwardRouteWithSolidInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);
  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        /*keep_behind_len=*/10.0);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  PlannerObjectManager object_mgr;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  {
    // solid_line_path_boundary
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(
        route_result.pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    {
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_NEAR(right_l_ext, -1.9, kEpsilon);
      EXPECT_NEAR(left_l_ext, 1.65, kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_NEAR(right_l, -1.9, kEpsilon);
      EXPECT_NEAR(left_l, 1.65, kEpsilon);
    }

    {
      // solid_line_path_boundary_borrow
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/true, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_borrow");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_LT(right_l_ext, -1.9 - kEpsilon);
      EXPECT_GT(left_l_ext, 1.65 + kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_LT(right_l, -1.9 - kEpsilon);
      EXPECT_GT(left_l, 1.65 + kEpsilon);
    }
  }

  {
    // solid_line_path_boundary_lc
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(
        route_result.pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(3.23);
    plan_start_point.mutable_path_point()->set_theta(-0.17);
    plan_start_point.set_v(0.0);

    lc_state.set_lc_left(false);

    {
      lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_lc");

      const auto [right_l_ext, left_l_ext] =
          path_bound_or->QueryBoundaryL(20.0);
      EXPECT_NEAR(right_l_ext, -1.9, kEpsilon);
      EXPECT_GT(left_l_ext, 1.65 + kEpsilon);

      const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(20.0);
      EXPECT_NEAR(right_l, -1.9, kEpsilon);
      EXPECT_GT(left_l, 1.65 + kEpsilon);
    }

    {
      lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      const auto path_bound_or = BuildPathBoundaryFromPose(
          planner_semantic_map_manager, dp_or.value(), plan_start_point,
          veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
          /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

      EXPECT_OK(path_bound_or);

      DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                                 "multi/solid_line_path_boundary_lcp");
    }
  }
}

TEST(BuildPathBoundaryFromPose, ObjectTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateAStraightForwardRouteInUrbanDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);
  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        /*keep_behind_len=*/10.0);

  EXPECT_OK(dp_or);

  VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(
      route_result.pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(
      route_result.pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
  plan_start_point.set_v(route_result.pose.vel_body().x());

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(planner_semantic_map_manager,
                                              /*obj_pos=*/Vec2d(140.0, -3.5));
  DrawPlannerObjectManagerToCanvas(object_mgr, "multi/static_object",
                                   vis::Color::kLightGreen);
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;

  const auto path_bound_or = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, dp_or.value(), plan_start_point,
      veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
      /*borrow_lane_boundary=*/false, /*should_smooth=*/false);

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/static_object_path_boundary");

  const auto [right_l_ext, left_l_ext] = path_bound_or->QueryBoundaryL(23.5);
  const auto [right_l, left_l] = path_bound_or->QueryTargetBoundaryL(23.5);
  EXPECT_NEAR(right_l_ext, right_l, kEpsilon);
  EXPECT_GT(left_l_ext, left_l + kEpsilon);
}

TEST(BuildPathBoundaryFromPose, SmoothTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  {
    const TestRouteResult route_result =
        CreateALeftTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    PlannerSemanticMapModification psmm_modifier;
    PlannerSemanticMapManager planner_semantic_map_manager(
        route_result.smm.get(), psmm_modifier);
    const auto dp_or =
        BuildDrivePassage(planner_semantic_map_manager,
                          route_result.route_lane_path.lane_paths().front(),
                          /*anchor_point=*/mapping::LanePoint(),
                          route_result.route_sections.planning_horizon(
                              planner_semantic_map_manager),
                          /*keep_behind_len=*/10.0);

    EXPECT_OK(dp_or);
    SendDrivePassageToCanvas(dp_or.value(),
                             "multi/smooth/left_turn/drive_passage");

    VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(848.186);
    plan_start_point.mutable_path_point()->set_y(-594.204);
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    PlannerObjectManager object_mgr;
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());

    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_NONE);
    SmoothedReferenceLineResultMap smooth_result_map;

    const auto path_bound_or = BuildPathBoundaryFromPose(
        planner_semantic_map_manager, dp_or.value(), plan_start_point,
        veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*should_smooth=*/true);

    EXPECT_OK(path_bound_or);

    for (int i = 0; i < path_bound_or->size(); ++i) {
      EXPECT_GT(path_bound_or->target_left_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
      EXPECT_LT(path_bound_or->target_right_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
    }
    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/smooth/left_turn/path_boundary");
  }

  {
    const TestRouteResult route_result =
        CreateARightTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    PlannerSemanticMapModification psmm_modifier;
    PlannerSemanticMapManager planner_semantic_map_manager(
        route_result.smm.get(), psmm_modifier);
    const auto dp_or =
        BuildDrivePassage(planner_semantic_map_manager,
                          route_result.route_lane_path.lane_paths().front(),
                          /*anchor_point=*/mapping::LanePoint(),
                          route_result.route_sections.planning_horizon(
                              planner_semantic_map_manager),
                          /*keep_behind_len=*/10.0);

    EXPECT_OK(dp_or);
    SendDrivePassageToCanvas(dp_or.value(),
                             "multi/smooth/right_turn/drive_passage");

    VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(848.186);
    plan_start_point.mutable_path_point()->set_y(-594.204);
    plan_start_point.mutable_path_point()->set_theta(route_result.pose.yaw());
    plan_start_point.set_v(route_result.pose.vel_body().x());

    PlannerObjectManager object_mgr;
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());

    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_NONE);
    SmoothedReferenceLineResultMap smooth_result_map;

    const auto path_bound_or = BuildPathBoundaryFromPose(
        planner_semantic_map_manager, dp_or.value(), plan_start_point,
        veh_geo_params, st_traj_mgr, lc_state, smooth_result_map,
        /*borrow_lane_boundary=*/false, /*should_smooth=*/true);

    EXPECT_OK(path_bound_or);

    for (int i = 0; i < path_bound_or->size(); ++i) {
      EXPECT_GT(path_bound_or->target_left_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
      EXPECT_LT(path_bound_or->target_right_l_vector()[i],
                path_bound_or->reference_center_l_vector()[i]);
    }
    DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                               "multi/smooth/right_turn/path_boundary");
  }
}

TEST(BuildPathBoundaryFromDrivePassage, BuildPathBoundaryFromDrivePassageTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);
  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        /*keep_behind_len=*/10.0);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(),
                           "multi/build_from_drive_passage_test");

  const auto path_bound_or = BuildPathBoundaryFromDrivePassage(
      planner_semantic_map_manager, dp_or.value());

  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(),
                             "multi/build_from_drive_passage");
}

}  // namespace
}  // namespace qcraft::planner
