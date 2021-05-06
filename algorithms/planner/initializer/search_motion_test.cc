#include "onboard/planner/initializer/search_motion.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/clock.h"
#include "onboard/planner/initializer/dp_motion_searcher.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/multi_traj_selector.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

namespace qcraft::planner {

namespace {
constexpr double kBuffer = 0.25;  // m.
TEST(SearchMotion, DISABLED_DummyTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  Vec2d p0(0.0, 0.0), p1(0.0, 1.0), p2(0.0, 2.0);
  std::vector<std::unique_ptr<GeometryForm>> ptr_geometry_forms;
  const auto geom_graph = BuildLineGraph({p0, p1, p2}, &ptr_geometry_forms);

  int n = 10;
  std::vector<double> s_vec, left_l, right_l, target_left_l, target_right_l;
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> left_xy, right_xy, target_left_xy, target_right_xy;
  left_xy.reserve(n);
  right_xy.reserve(n);
  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(std::move(s_vec), std::move(right_l),
                            std::move(left_l), std::move(target_right_l),
                            std::move(target_left_l), std::move(right_xy),
                            std::move(left_xy), std::move(target_right_xy),
                            std::move(target_left_xy));

  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(0);
  sdc_pose.mutable_pos_smooth()->set_y(-0.5);
  sdc_pose.set_yaw(d2r(90.0));
  sdc_pose.mutable_vel_body()->set_x(1.0);
  const SpacetimeTrajectoryManager st_traj_mgr({});
  ConstraintManager constraint_manager;
  const auto vehicle_geom = DefaultVehicleGeometry();
  MotionSearchInput input;
  input.sdc_pose = &sdc_pose;
  const auto start_point = ConvertToTrajPointProto(sdc_pose);
  input.start_point = &start_point;
  const auto fake_time = Clock::Now();
  input.plan_time = fake_time;

  const absl::flat_hash_set<std::string> stalled_objects;

  std::vector<InitializerSearchConfig> search_configs;
  InitializerSearchConfig search_config;
  search_configs.push_back(search_config);

  PlannerParamsProto planner_params = DefaultPlannerParams();
  input.planner_params = &planner_params;
  input.drive_passage = nullptr;
  input.geom_graph = &geom_graph;
  input.constraint_manager = &constraint_manager;
  input.st_traj_mgr = &st_traj_mgr;
  input.vehicle_geom = &vehicle_geom;
  input.sl_boundary = &path_bound;
  input.search_configs = &search_configs;
  input.is_lane_change = false;
  input.lc_multiple_traj = false;
  InitializerDebugProto debug_proto;

  const auto output =
      SearchMotion(input, ThreadPool::DefaultPool(), &debug_proto);
  ASSERT_OK(output);

  for (const auto& motion : output->traj_points) {
    LOG(INFO) << motion.DebugString();
  }

  // EXPECT_EQ(output->motion_states.size(), 4);
}

TEST(SearchMotionCurvyDp, StopLineTestWithTimeConsumptionTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  MotionSearchInput input;

  // Construct sdc pose.
  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(30.0);
  sdc_pose.mutable_pos_smooth()->set_y(0.0);
  sdc_pose.set_yaw(0.0);
  sdc_pose.mutable_vel_body()->set_x(5.0);
  input.sdc_pose = &sdc_pose;
  const auto start_point = ConvertToTrajPointProto(sdc_pose);
  input.start_point = &start_point;
  const auto fake_time = Clock::Now();
  input.plan_time = fake_time;

  // Load default planner params.
  auto planner_params = DefaultPlannerParams();
  planner_params.mutable_initializer_params()->set_search_algorithm(
      InitializerConfig::DP);

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  // Build drive passages.
  auto start_time = absl::Now();
  const auto drive_passages = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passages.ok() && !drive_passages.value().empty());
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage.";
  input.drive_passage = &(drive_passages.value());
  SendDrivePassageToCanvas(drive_passages.value(), "test/drive_passage");

  // Build geometry graph.
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

  const SpacetimeTrajectoryManager st_traj_mgr({});
  const auto vehicle_geom = DefaultVehicleGeometry();
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(&st_traj_mgr, &vehicle_geom,
                                                 MotionEdgeInfo::kSampleStep,
                                                 kBuffer, kBuffer);

  const auto traj_point = ConvertToTrajPointProto(sdc_pose);
  const auto path_sl_boundary =
      CreateFakePathSlBoundary(drive_passages.value());

  // Build stopline constraint.
  ConstraintManager constraint_manager;
  ConstraintProto::StopLineProto stop_line;
  HalfPlane halfplane(Vec2d(70.0, -5.25), Vec2d(70.0, 9.75));
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_time(0.0);
  stop_line.set_s(40.0);
  constraint_manager.AddStopLine(stop_line);
  const auto vehicle_drive = DefaultVehicleDriveParams();
  bool lc_multiple_traj = false;
  GeometryFormBuilder form_builder(&drive_passages.value(),
                                   drive_passages->end_s(), 0.0);
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &drive_passages.value(),
      .sl_boundary = &path_sl_boundary,
      .constraint_manager = &constraint_manager,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &traj_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &form_builder,
      .lc_multiple_traj = lc_multiple_traj};
  ThreadPool* thread_pool = ThreadPool::DefaultPool();
  InitializerDebugProto initializer_debug;
  start_time = absl::Now();
  GeometryGraphCache graph_cache;
  const auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              thread_pool, &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building GeometryGraph (without the time of "
                "building inputs)";

  const absl::flat_hash_set<std::string> stalled_objects;

  std::vector<InitializerSearchConfig> search_configs;
  InitializerSearchConfig search_config;
  search_configs.push_back(search_config);

  input.geom_graph = &geom_graph;
  input.form_builder = &form_builder;
  input.constraint_manager = &constraint_manager;
  input.st_traj_mgr = &st_traj_mgr;
  input.vehicle_geom = &vehicle_geom;
  input.sl_boundary = &path_sl_boundary;
  input.collision_checker = collision_checker.get();
  input.planner_params = &planner_params;
  input.lc_multiple_traj = lc_multiple_traj;
  input.search_configs = &search_configs;

  start_time = absl::Now();
  const auto output = SearchMotion(input, thread_pool, &initializer_debug);
  ASSERT_OK(output);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in searching motion";

  CanvasDrawTrajectory(output->traj_points,
                       "test/dp_stopline_initializer_traj");

  // Construct a rectangle to validate the output is straight. All geometry
  // state points are inside the rectangle to guarantee the straightness because
  // the width of rectangle is small enough.
  constexpr double kYTolerance = 0.1;
  const std::vector<Vec2d> tolerance_polygon_vertexes = {
      Vec2d(70.0, kYTolerance), Vec2d(24.0, kYTolerance),
      Vec2d(24.0, -kYTolerance), Vec2d(70.0, -kYTolerance)};
  const Polygon2d tolerance_polygon(tolerance_polygon_vertexes);

  for (const auto& motion : output->traj_points) {
    EXPECT_TRUE(tolerance_polygon.IsPointIn(
        Vec2d(motion.path_point().x(), motion.path_point().y())))
        << motion.DebugString();
  }

  // Execute trajecory speed constraints check.
  // TODO(zixuan) : introduce global v limit and lane speed limit for checking
  // and check v in decelerating and stopping.
  constexpr double kMaxSpeedLimit = Kph2Mps(40.0);
  for (const auto& motion : output->traj_points) {
    EXPECT_LE(motion.v(), kMaxSpeedLimit);
    EXPECT_GE(motion.v(), 0.0);
  }

  // Execute trajecory acceleration constraints check.
  constexpr double kMaxAcceleration = 2.0;
  constexpr double kMaxDeceleration = -4.0;
  for (const auto& motion : output->traj_points) {
    EXPECT_LE(motion.a(), kMaxAcceleration);
    EXPECT_GE(motion.a(), kMaxDeceleration);
  }

  // Execute trajecory length check.
  constexpr double kMinSearchResultTrajectoryLength = 0.5;
  const double search_result_trajectory_length =
      output->traj_points.back().path_point().s();
  QLOG(INFO) << "Trajectory length: " << search_result_trajectory_length;
  EXPECT_GE(search_result_trajectory_length, kMinSearchResultTrajectoryLength);

  // Execute trajecory end time check.
  const double search_result_trajectory_end_time =
      output->traj_points.back().relative_time();
  QLOG(INFO) << "Trajectory end time: " << search_result_trajectory_end_time;
  EXPECT_EQ(search_result_trajectory_end_time, 9.9);
}

TEST(SearchMotion,
     DISABLED_TimeConsumptionTest) {  // Time consumption recorded in stopline
                                      // test for SearchMoitonCurvyDp.
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  MotionSearchInput input;

  // Construct sdc pose.
  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(152.0);
  sdc_pose.mutable_pos_smooth()->set_y(66.5);
  sdc_pose.set_yaw(0.0);
  sdc_pose.mutable_vel_body()->set_x(11.1);
  input.sdc_pose = &sdc_pose;
  const auto start_point = ConvertToTrajPointProto(sdc_pose);
  input.start_point = &start_point;
  const auto fake_time = Clock::Now();
  input.plan_time = fake_time;

  // Load default planner params.
  PlannerParamsProto planner_params = DefaultPlannerParams();
  planner_params.mutable_initializer_params()->set_search_algorithm(
      InitializerConfig::DP);
  input.planner_params = &planner_params;

  // Load map and create optimized route path.
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, sdc_pose, "a8_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  // Build drive passages.
  auto start_time = absl::Now();
  const auto drive_passages = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building DrivePassage";
  ASSERT_TRUE(drive_passages.ok() && !drive_passages.value().empty());
  input.drive_passage = &(drive_passages.value());
  const auto path_sl_boundary =
      CreateFakePathSlBoundary(drive_passages.value());

  // Build stopline constraint.
  ConstraintManager constraint_manager;
  input.constraint_manager = &constraint_manager;
  const SpacetimeTrajectoryManager st_traj_mgr({});
  input.st_traj_mgr = &st_traj_mgr;
  const auto vehicle_geom = DefaultVehicleGeometry();
  input.vehicle_geom = &vehicle_geom;
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(&st_traj_mgr, &vehicle_geom,
                                                 MotionEdgeInfo::kSampleStep,
                                                 kBuffer, kBuffer);
  input.sl_boundary = &path_sl_boundary;
  input.collision_checker = collision_checker.get();
  ThreadPool* thread_pool = ThreadPool::DefaultPool();

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

  const auto vehicle_drive = DefaultVehicleDriveParams();
  GeometryFormBuilder form_builder(&drive_passages.value(),
                                   drive_passages->end_s(), 0.0);
  bool lc_multiple_traj = false;
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &drive_passages.value(),
      .sl_boundary = &path_sl_boundary,
      .constraint_manager = &constraint_manager,
      .st_traj_mgr = &st_traj_mgr,
      .plan_start_point = &start_point,
      .vehicle_geom = &vehicle_geom,
      .collision_checker = collision_checker.get(),
      .sampling_params = &sampling_params,
      .vehicle_drive = &vehicle_drive,
      .form_builder = &form_builder,
      .lc_multiple_traj = lc_multiple_traj};
  InitializerDebugProto initializer_debug;
  // Build geometry graph.
  start_time = absl::Now();
  GeometryGraphCache graph_cache;
  const auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              thread_pool, &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in building GeometryGraph";
  input.geom_graph = &geom_graph;
  input.form_builder = &form_builder;
  start_time = absl::Now();
  const auto output = SearchMotion(input, thread_pool, &initializer_debug);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in searching motion";
  ASSERT_OK(output);

  // Execute trajectory length check.
  constexpr double kMinSearchResultTrajectoryLength = 0.5;
  const double search_result_trajectory_length =
      output->traj_points.back().path_point().s();
  QLOG(INFO) << "Trajectory length: " << search_result_trajectory_length;
  EXPECT_GE(search_result_trajectory_length, kMinSearchResultTrajectoryLength);

  // Execute trajecory end time check.
  const double search_result_trajectory_end_time =
      output->traj_points.back().relative_time();
  QLOG(INFO) << "Trajectory end time: " << search_result_trajectory_end_time;
  EXPECT_EQ(search_result_trajectory_end_time, 9.9);
}

}  // namespace
}  // namespace qcraft::planner
