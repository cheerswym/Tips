#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/geometry/geometry_graph_debug.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/test_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {
namespace {
constexpr double kBuffer = 0.25;  // m.

TEST(GeometryGraph, BuildCurvyXYGeometryGraph) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const auto route_result = CreateAStraightForwardRouteInUrbanDojo();
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

  EXPECT_TRUE(dp_or.ok());
  SendDrivePassageToCanvas(dp_or.value(), "test/straight_drive_passage");
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
  const auto vehicle_drive = DefaultVehicleDriveParams();
  const std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_traj_mgr, &vehicle_geom, MotionEdgeInfo::kSampleStep,
          /*stationary_object_buffer=*/kBuffer,
          /*moving_object_buffer=*/2.0 * kBuffer);

  PoseProto sdc_pose;
  sdc_pose.mutable_pos_smooth()->set_x(route_result.pose.pos_smooth().x());
  sdc_pose.mutable_pos_smooth()->set_y(route_result.pose.pos_smooth().y());
  sdc_pose.set_yaw(route_result.pose.yaw());
  sdc_pose.mutable_vel_body()->set_x(0.1);

  const auto traj_point = ConvertToTrajPointProto(sdc_pose);
  const auto path_sl_boundary = CreateFakePathSlBoundary(dp_or.value());
  ConstraintManager constraint_manager;
  const GeometryFormBuilder form_builder(&dp_or.value(), dp_or->end_s(), 0.0);
  bool lc_multiple_traj = false;
  const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
      .passage = &dp_or.value(),
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
  GeometryGraphCache graph_cache;
  InitializerDebugProto initializer_debug;
  const auto geom_graph_or =
      BuildCurvyGeometryGraph(geom_graph_builder_input, false, &graph_cache,
                              ThreadPool::DefaultPool(), &initializer_debug);
  ASSERT_TRUE(geom_graph_or.ok());
  const auto& geom_graph = geom_graph_or.value();

  SendGeometryGraphToCanvas(&geom_graph, "test/curvy_geograph");
  EXPECT_OK(CheckGeometryGraphConnectivity(geom_graph));
  ASSERT_TRUE(geom_graph.nodes().size() > 0)
      << "Build lateral quintic polynomial geometry graph failed!";
}
}  // namespace
}  // namespace qcraft::planner
