#include "onboard/planner/initializer/reference_line_searcher.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/search_motion.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

namespace qcraft::planner {
namespace {
constexpr double kBuffer = 0.25;  // m.
PlannerObjectManager BuildPhantomVehicle(const PlannerSemanticMapManager& psmm,
                                         Vec2d obj_pos, double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(heading)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(obj_pos, heading);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

TEST(ReferenceLineSearcher, StationaryObjectFarAhead) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Create pose.
  PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(5.0, 0.0));
  // Plan start point.
  ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);
  // Routing.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b6_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  // Drive Passage.
  const auto dp_or = BuildDrivePassage(psmm, route_path.lane_paths().front(),
                                       /*anchor_point=*/mapping::LanePoint(),
                                       route_sections.planning_horizon(psmm),
                                       /*keep_behind_len=*/10.0);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "farahead/drive_passage");
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(27.0, -1.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "farahead/object",
                                   vis::Color::kLightGreen);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects(),
                                               nullptr);

  // Sampling params.
  const std::vector<double> kSampleStrategyRange = {
      1.0, 20.0, 60.0, 100.0, std::numeric_limits<double>::max()};
  const std::vector<int> kSampleStrategyStationStep = {5, 5, 10, 40, 60};
  const std::vector<double> kSampleStrategyLateralResolution = {0.25, 0.25, 0.5,
                                                                0.5, 0.5};
  const std::vector<int> kSampleStrategyCrossingLayer = {3, 3, 2, 1, 1};
  const std::vector<double> kSampleStrategyUnitLengthLateralSpan = {
      1.0, 0.25, 0.10, 0.05, 0.025};
  const GeometryGraphSamplingStrategy sampling_params = {
      .range_list = kSampleStrategyRange,
      .layer_stations_list = kSampleStrategyStationStep,
      .lateral_resolution_list = kSampleStrategyLateralResolution,
      .cross_layer_connection_list = kSampleStrategyCrossingLayer,
      .unit_length_lateral_span_list = kSampleStrategyUnitLengthLateralSpan};

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  bool borrow = false;
  bool should_smooth = false;
  // Path Sl boundary.
  ASSIGN_OR_DIE(
      const auto path_sl_boundary,
      BuildPathBoundaryFromPose(psmm, dp_or.value(), plan_start_point,
                                vehicle_geom, st_traj_mgr, lc_state,
                                smooth_result_map, borrow, should_smooth));
  DrawPathSlBoundaryToCanvas(path_sl_boundary, "farahead/path_boundary");
  // Constraint manager.
  const ConstraintManager constraint_manager;
  const GeometryFormBuilder geom_form_builder(&dp_or.value(), dp_or->end_s(),
                                              0.0);
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_traj_mgr, &vehicle_geom, MotionEdgeInfo::kSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .constraint_manager = &constraint_manager,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &plan_start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &geom_form_builder,
      .lc_multiple_traj = false};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph,
                            "farahead/curvy_geograph/before_searcher");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));

  // Reference line searcher.
  ReferenceLineSearcherInput ref_line_search_input{
      .geometry_graph = &geom_graph_or.value(),
      .drive_passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .st_traj_mgr = &st_traj_mgr,
      .planner_params = &planner_params,
      .vehicle_geom = &vehicle_geom,
      .vehicle_drive = &vehicle_drive,
  };

  const auto ref_line_output_or =
      SearchReferenceLine(ref_line_search_input, &initializer_debug, nullptr);
  EXPECT_TRUE(ref_line_output_or.ok());
  ASSERT_OK(ActivateGeometryGraph(*ref_line_output_or, path_sl_boundary,
                                  &geom_graph_or.value()));
  SendGeometryGraphToCanvas(&geom_graph,
                            "farahead/curvy_geograph/after_searcher");
  SendPathPointsToCanvas((*ref_line_output_or).ref_line_points,
                         "farahead/reference_line", 0, vis::Color::kLightGreen,
                         6);
}

TEST(ReferenceLineSearcher, StationaryObjectClose) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Create pose.
  PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(2.0, 0.0));
  // Plan start point.
  ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);
  // Routing.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b6_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  // Drive Passage.
  const auto dp_or = BuildDrivePassage(psmm, route_path.lane_paths().front(),
                                       /*anchor_point=*/mapping::LanePoint(),
                                       route_sections.planning_horizon(psmm),
                                       /*keep_behind_len=*/15.0);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "closeahead/drive_passage");
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(10.0, -1.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "closeahead/object",
                                   vis::Color::kLightGreen);

  const SpacetimeTrajectoryManager st_traj_mgr(object_mgr.planner_objects(),
                                               nullptr);

  // Sampling params.
  const std::vector<double> kSampleStrategyRange = {
      1.0, 20.0, 60.0, 100.0, std::numeric_limits<double>::max()};
  const std::vector<int> kSampleStrategyStationStep = {5, 5, 10, 40, 60};
  const std::vector<double> kSampleStrategyLateralResolution = {0.25, 0.25, 0.5,
                                                                0.5, 0.5};
  const std::vector<int> kSampleStrategyCrossingLayer = {3, 3, 2, 1, 1};
  const std::vector<double> kSampleStrategyUnitLengthLateralSpan = {
      1.0, 0.25, 0.10, 0.05, 0.025};
  const GeometryGraphSamplingStrategy sampling_params = {
      .range_list = kSampleStrategyRange,
      .layer_stations_list = kSampleStrategyStationStep,
      .lateral_resolution_list = kSampleStrategyLateralResolution,
      .cross_layer_connection_list = kSampleStrategyCrossingLayer,
      .unit_length_lateral_span_list = kSampleStrategyUnitLengthLateralSpan};

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  bool borrow = false;
  bool should_smooth = false;
  // Path Sl boundary.
  ASSIGN_OR_DIE(
      const auto path_sl_boundary,
      BuildPathBoundaryFromPose(psmm, dp_or.value(), plan_start_point,
                                vehicle_geom, st_traj_mgr, lc_state,
                                smooth_result_map, borrow, should_smooth));
  DrawPathSlBoundaryToCanvas(path_sl_boundary, "closeahead/path_boundary");
  // Constraint manager.
  const ConstraintManager constraint_manager;
  const GeometryFormBuilder geom_form_builder(&dp_or.value(), dp_or->end_s(),
                                              0.0);
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_traj_mgr, &vehicle_geom, MotionEdgeInfo::kSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .constraint_manager = &constraint_manager,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &plan_start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &geom_form_builder,
      .lc_multiple_traj = false};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph,
                            "closeahead/curvy_geograph/before_searcher");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));

  // Reference line searcher.
  ReferenceLineSearcherInput ref_line_search_input{
      .geometry_graph = &geom_graph_or.value(),
      .drive_passage = &dp_or.value(),
      .sl_boundary = &path_sl_boundary,
      .st_traj_mgr = &st_traj_mgr,
      .planner_params = &planner_params,
      .vehicle_geom = &vehicle_geom,
      .vehicle_drive = &vehicle_drive,
  };

  const auto ref_line_output_or =
      SearchReferenceLine(ref_line_search_input, &initializer_debug, nullptr);
  EXPECT_TRUE(ref_line_output_or.ok());
  ASSERT_OK(ActivateGeometryGraph(*ref_line_output_or, path_sl_boundary,
                                  &geom_graph_or.value()));
  SendGeometryGraphToCanvas(&geom_graph,
                            "closeahead/curvy_geograph/after_searcher");
  SendPathPointsToCanvas((*ref_line_output_or).ref_line_points,
                         "closeahead/reference_line", 0,
                         vis::Color::kLightGreen, 6);
}
}  // namespace

}  // namespace qcraft::planner
