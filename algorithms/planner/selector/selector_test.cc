#include "onboard/planner/selector/selector.h"

#include <utility>

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/planner_defs.h"
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
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

namespace {

PlannerObjectManager BuildPhantomVehicle(const PlannerSemanticMapManager &psmm,
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

SchedulerOutput BuildSchedulerOutput(
    const PlannerSemanticMapManager &psmm, double planning_horizon,
    const LanePathInfo &lp_info, bool borrow,
    const ApolloTrajectoryPointProto &start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const std::string &channel_prefix) {
  // Build drive passages.
  ASSIGN_OR_DIE(
      auto drive_passage,
      BuildDrivePassage(psmm, lp_info.lane_path(),
                        /*anchor_point=*/mapping::LanePoint(), planning_horizon,
                        kDrivePassageKeepBehindLength));
  SendDrivePassageToCanvas(drive_passage, channel_prefix + "/drive_passage");

  ASSIGN_OR_DIE(auto ego_frenet_box,
                drive_passage.QueryFrenetBoxAt(
                    GetAvBox(Vec2dFromApolloTrajectoryPointProto(start_point),
                             start_point.path_point().theta(), vehicle_geom)));
  auto lc_state = MakeLaneChangeState(drive_passage, ego_frenet_box);
  SmoothedReferenceLineResultMap smooth_result_map;
  // Build path boundary.
  ASSIGN_OR_DIE(
      auto path_boundary,
      BuildPathBoundaryFromPose(psmm, drive_passage, start_point, vehicle_geom,
                                st_traj_mgr, lc_state, smooth_result_map,
                                borrow, /*should_smooth=*/false));
  DrawPathSlBoundaryToCanvas(path_boundary, channel_prefix + "/path_boundary");

  // Scheduler output.
  return SchedulerOutput{
      .drive_passage = std::move(drive_passage),
      .sl_boundary = std::move(path_boundary),
      .lane_change_state = std::move(lc_state),
      .length_along_route = lp_info.length_along_route(),
      .borrow_lane = borrow,
      .av_frenet_box_on_drive_passage = std::move(ego_frenet_box),
      .reasons = std::vector<std::string>(),
      .clearance_output = std::nullopt};
}

TEST(Selector, OvertakeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(116.5, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(160.0, 0.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "overtake/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, smm,
        "overtake/lane_paths/" + std::to_string(lp_info.start_lane_id()));
  }

  PlannerState planner_state;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"nudge", "follow"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    // Scheduler output.
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[i], /*borrow=*/false,
        start_point, vehicle_geom, st_traj_mgr,
        "overtake/" + channel_prefix[i]);
    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points, "overtake/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {2471, 53}, 0.0, 1.0);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "overtake/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(
      multi_tasks[selected_idx].drive_passage.lane_path().front().lane_id(),
      2470);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.y(), kDefaultHalfLaneWidth);
}

TEST(Selector, RightTurnOvertakeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(200.0, 63.0), 0.0, Vec2d(6.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "8c_s3");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(240.0, 62.6));
  DrawPlannerObjectManagerToCanvas(object_mgr, "right_turn/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, smm,
        "right_turn/lane_paths/" + std::to_string(lp_info.start_lane_id()));
  }

  PlannerState planner_state;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"nudge", "follow"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[i + 1],
        /*borrow=*/false, start_point, vehicle_geom, st_traj_mgr,
        "right_turn/" + channel_prefix[i]);
    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "right_turn/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {498, 2483, 500}, 0.86, 1.0);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "right_turn/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(
      multi_tasks[selected_idx].drive_passage.lane_path().front().lane_id(),
      2491);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.x(), 229.0);
}

TEST(Selector, LaneChangeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time),
                 Vec2d(116.5, kDefaultLaneWidth), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(160.0, 0.0));
  DrawPlannerObjectManagerToCanvas(object_mgr, "lane_change/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, smm,
        "lane_change/lane_paths/" + std::to_string(lp_info.start_lane_id()));
  }

  PlannerState planner_state;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"nudge", "follow"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[i], /*borrow=*/false,
        start_point, vehicle_geom, st_traj_mgr,
        "lane_change/" + channel_prefix[i]);
    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "lane_change/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {2470, 51}, 0.0, 1.0);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "lane_change/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(
      multi_tasks[selected_idx].drive_passage.lane_path().front().lane_id(),
      2470);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.x(), 160.0);
}

TEST(Selector, EarlyLaneChangeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(180.0, -217.0), 0.0,
                 Vec2d(18.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path =
      *RoutingToLanePoint(smm, sdc_pose, mapping::LanePoint(1492, 0.2));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  PlannerObjectManager object_mgr;
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, smm,
        "early_lc/lane_paths/" + std::to_string(lp_info.start_lane_id()));
  }

  PlannerState planner_state;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[i], /*borrow=*/false,
        start_point, vehicle_geom, st_traj_mgr,
        "early_lc/" + channel_prefix[i]);

    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points, "early_lc/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {2341, 2348}, 0.2, 1.0);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "early_lc/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(
      multi_tasks[selected_idx].drive_passage.lane_path().front().lane_id(),
      2340);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_LT(last_traj_pt.y(), -218.9);
}

TEST(Selector, TrafficLightTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(210.0, 66.5), 0.0, Vec2d(6.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path =
      *RoutingToLanePoint(smm, sdc_pose, mapping::LanePoint(154, 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object.
  const auto object_mgr = BuildPhantomVehicle(psmm, Vec2d(240.0, 66.5));
  DrawPlannerObjectManagerToCanvas(object_mgr, "traffic_light/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects;

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(
        lp_info, smm,
        "traffic_light/lane_paths/" + std::to_string(lp_info.start_lane_id()));
  }

  // Create traffic light history manager.
  TrafficLightStatesProto tl_states_proto;
  tl_states_proto.mutable_header()->set_timestamp(
      absl::ToUnixMicros(absl::Now()));
  for (int i = 0; i < 3; ++i) {
    auto *new_state = tl_states_proto.add_states();
    new_state->set_traffic_light_id(917);
    new_state->set_color(TL_RED);
    new_state->set_flashing(false);
  }

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto tl_collector_output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &tl_states_proto,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  PlannerState planner_state;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_change"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[i], /*borrow=*/false,
        start_point, vehicle_geom, st_traj_mgr,
        "traffic_light/" + channel_prefix[i]);
    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_collector_output.tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "traffic_light/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {2491, 2484, 62}, 0.86, 1.0);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "traffic_light/selected",
                                     vis::Color::kLightGreen);
  EXPECT_EQ(
      multi_tasks[selected_idx].drive_passage.lane_path().front().lane_id(),
      2491);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_LT(last_traj_pt.x(), 245.0);
}

TEST(Selector, SingleLaneOvertakeTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_params = run_params.vehicle_params();
  const auto &vehicle_geom = vehicle_params.vehicle_geometry_params();

  vantage_client_man::CreateVantageClientMan(*param_manager);
  PlannerParamsProto planner_params = DefaultPlannerParams();

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(-133.0, -160.0), M_PI_2,
                 Vec2d(5.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "4d_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  const RouteSectionsInfo sections_info(psmm, &route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object.
  const auto object_mgr =
      BuildPhantomVehicle(psmm, Vec2d(-132.4, -140.0), /*heading=*/M_PI_2);
  DrawPlannerObjectManagerToCanvas(object_mgr, "single_lane_overtake/object",
                                   vis::Color::kLightGreen);
  // ObjectTracer analysis result.
  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());

  const auto lane_graph = BuildLaneGraph(psmm, sections_info, object_mgr,
                                         stalled_objects, /*avoid_lanes=*/{});
  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);
  for (const auto &lp_info : lp_infos) {
    SendLanePathInfoToCanvas(lp_info, smm,
                             "single_lane_overtake/lane_paths/" +
                                 std::to_string(lp_info.start_lane_id()));
  }

  PlannerState planner_state;
  TrafficLightInfoMap tl_info_map;
  SceneOutputProto scene_reasoning;
  std::vector<ApolloTrajectoryPointProto> time_aligned_prev_traj;
  TeleopState teleop_state;
  ThreadPool *thread_pool = ThreadPool::DefaultPool();

  std::vector<SchedulerOutput> multi_tasks(2);
  std::vector<PlannerStatus> status_list(2);
  std::vector<EstPlannerOutput> results(2);
  const std::vector<std::string> channel_prefix{"lane_keep", "lane_borrow"};
  const PlanStartPointInfo start_point_info{
      .reset = false,
      .start_point = start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = plan_time,
      .full_stop = false,
  };
  for (int i : {0, 1}) {
    multi_tasks[i] = BuildSchedulerOutput(
        psmm, sections_info.planning_horizon(), lp_infos[0], i, start_point,
        vehicle_geom, st_traj_mgr, "single_lane_overtake/" + channel_prefix[i]);
    status_list[i] = RunEstPlanner(
        EstPlannerInput{.semantic_map_manager = &smm,
                        .planner_semantic_map_manager = &psmm,
                        .plan_id = i,
                        .pose = &sdc_pose,
                        .planner_state = &planner_state,
                        .vehicle_params = &vehicle_params,
                        .planner_params = &planner_params,
                        .obj_mgr = &object_mgr,
                        .start_point_info = &start_point_info,
                        .tl_info_map = &tl_info_map,
                        .stalled_objects = &stalled_objects,
                        .scene_reasoning = &scene_reasoning,
                        .scheduler_output = &multi_tasks[i],
                        .route_sections_from_start = &route_sections,
                        .time_aligned_prev_traj = &time_aligned_prev_traj,
                        .teleop_state = &teleop_state,
                        .sensor_fovs = nullptr},
        &results[i], thread_pool);
    ASSERT_OK(status_list[i]);
    SendApolloTrajectoryPointsToCanvas(
        results[i].traj_points,
        "single_lane_overtake/" + channel_prefix[i] + "/trajectory",
        vis::Color::kLightOrange);
  }

  const mapping::LanePath prev_lp(&smm, {1224, 683}, 0.2, 0.5);
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &sections_info,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_lp,
      .prev_traj = nullptr,
      .motion_constraints = &planner_params.motion_constraint_params(),
      .vehicle_geom = &vehicle_geom,
      .plan_start_point = &start_point,
      .stalled_objects = &stalled_objects,
      .config = &planner_params.selector_params()};

  SelectorDebugProto selector_debug;
  ASSIGN_OR_DIE(const int selected_idx,
                SelectTrajectory(selector_input, multi_tasks, status_list,
                                 results, &selector_debug));
  const auto &selected_output = results[selected_idx];
  SendApolloTrajectoryPointsToCanvas(selected_output.traj_points,
                                     "single_lane_overtake/selected",
                                     vis::Color::kLightGreen);
  const auto last_traj_pt =
      Vec2dFromApolloTrajectoryPointProto(selected_output.traj_points.back());
  EXPECT_GT(last_traj_pt.y(), -140.0);
  EXPECT_TRUE(multi_tasks[selected_idx].borrow_lane);
}

}  // namespace
}  // namespace qcraft::planner
