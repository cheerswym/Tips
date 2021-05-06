#include "onboard/planner/initializer/multi_traj_selector.h"

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
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

namespace {
constexpr double kBuffer = 0.25;  // m.
PlannerObjectManager BuildMultiplePhantomVehicles(
    const PlannerSemanticMapManager& psmm,
    const std::vector<Vec2d>& object_positions) {
  ObjectVector<PlannerObject> objects;
  for (int i = 0; i < object_positions.size(); ++i) {
    const auto& pos = object_positions[i];
    const auto perc_obj = PerceptionObjectBuilder()
                              .set_id(absl::StrFormat("Phantom-%d", i))
                              .set_type(ObjectType::OT_VEHICLE)
                              .set_pos(pos)
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
        .set_stationary_traj(pos, /*heading=*/0.0);
    objects.push_back(builder.Build());
  }
  return PlannerObjectManager(objects);
}

TEST(MultiTrajectorySelector, LCIgnoreStalledObjects) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  PlannerParamsProto planner_params = DefaultPlannerParams();
  VehicleGeometryParamsProto vehicle_geom = DefaultVehicleGeometry();
  VehicleDriveParamsProto vehicle_drive = DefaultVehicleDriveParams();
  MotionConstraintParamsProto motion_constraint_params =
      DefaultPlannerParams().motion_constraint_params();

  // Construct sdc pose.
  absl::Time timestamp = absl::UnixEpoch();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(timestamp),
                 Vec2d(116.5, kDefaultLaneWidth), 0.0, Vec2d(11.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);

  // Routing.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Create multiple objects on target lane.
  const std::vector<Vec2d> object_positions = {
      Vec2d(117.57, 0.081), Vec2d(127.031, 0.081), Vec2d(134.936, -1.144),
      Vec2d(142.5, -1.478), Vec2d(153.3, -1.478)};

  const auto object_mgr = BuildMultiplePhantomVehicles(psmm, object_positions);

  DrawPlannerObjectManagerToCanvas(object_mgr, "LCIgnoreStalledObjects/objects",
                                   vis::Color::kLightGreen);
  // Create stalled objects.
  absl::flat_hash_set<std::string> stalled_objects;
  stalled_objects.insert("Phantom-1-idx0");
  stalled_objects.insert("Phantom-2-idx0");

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto& lp_info : lp_infos) {
    SendLanePathInfoToCanvas(lp_info, smm,
                             "LCIgnoreStalledObjects/lane_paths/" +
                                 std::to_string(lp_info.start_lane_id()));
  }

  // Drive Passage.
  const auto dp_or = BuildDrivePassage(psmm, lp_infos[1].lane_path(),
                                       /*anchor_point=*/mapping::LanePoint(),
                                       sections_info.planning_horizon(),
                                       /*keep_behind_len=*/10.0);
  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(),
                           "LCIgnoreStalledObjects/drive_passage");

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

  ASSIGN_OR_DIE(auto ego_frenet_box,
                dp_or->QueryFrenetBoxAt(GetAvBox(
                    Vec2dFromApolloTrajectoryPointProto(plan_start_point),
                    plan_start_point.path_point().theta(), vehicle_geom)));
  auto lc_state = MakeLaneChangeState(*dp_or, ego_frenet_box);
  SmoothedReferenceLineResultMap smooth_result_map;

  // Path Sl Boundary.

  ASSIGN_OR_DIE(const auto path_sl_boundary,
                BuildPathBoundaryFromPose(
                    psmm, dp_or.value(), plan_start_point, vehicle_geom,
                    st_traj_mgr, lc_state, smooth_result_map,
                    /*borrow=*/false, /*should_smooth=*/false));
  DrawPathSlBoundaryToCanvas(path_sl_boundary,
                             "LCIgnoreStalledObjects/path_boundary");

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
      .lc_multiple_traj = true};

  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph, "LCIgnoreStalledObjects/geom_graph");

  const auto search_configs = BuildSearchConfig(
      dp_or.value(), path_sl_boundary, st_traj_mgr, constraint_manager,
      /*lc_clearance=*/nullptr, stalled_objects, /*blocking_static=*/nullptr,
      geom_graph, vehicle_geom,
      /*is_lane_change=*/true, /*lc_left=*/false,
      /*lc_multiple_traj=*/true);

  const MotionSearchInput motion_search_input = {
      .start_point = &plan_start_point,
      .plan_time = timestamp,
      .sdc_pose = &sdc_pose,
      .drive_passage = &(*dp_or),
      .st_traj_mgr = &st_traj_mgr,
      .vehicle_geom = &vehicle_geom,
      .geom_graph = &geom_graph,
      .form_builder = &geom_form_builder,
      .collision_checker = collision_checker.get(),
      .constraint_manager = &constraint_manager,
      .sl_boundary = &path_sl_boundary,
      .planner_params = &planner_params,
      .blocking_static_obj = nullptr,
      .lc_clearance = nullptr,
      .search_configs = &search_configs,
      .is_lane_change = true,
      .lc_multiple_traj = true,
  };

  const auto output_or = SearchMotion(
      motion_search_input, ThreadPool::DefaultPool(), &initializer_debug);

  ASSERT_TRUE(output_or.ok());

  const auto& final_traj = output_or->traj_points;
  const auto& trajs = output_or->trajs_with_lead_obj;

  SendApolloTrajectoryPointsToCanvas(
      final_traj,
      absl::StrCat("LCIgnoreStalledObjects/trajectories/final_choice_",
                   absl::StrJoin(output_or->leading_objs, "_")),
      vis::Color::kLightGreen);

  for (const auto& traj : trajs) {
    SendApolloTrajectoryPointsToCanvas(
        traj.trajectory,
        absl::StrCat("LCIgnoreStalledObjects/trajectories/",
                     absl::StrJoin(traj.leading_obj_ids, "_")),
        vis::Color::kMagenta);
  }
}
}  // namespace
}  // namespace qcraft::planner
