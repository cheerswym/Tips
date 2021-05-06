#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"

#include <cmath>

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/clock.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/end_of_path_boundary.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

namespace qcraft {
namespace planner {
namespace {

struct EgoPose {
  Vec2d pos;     // {x, y}.
  double theta;  // rad.
  double v;      // m/s.
};

PlannerParamsProto planner_params = DefaultPlannerParams();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

TEST(TrajectoryOptimizerTest, LaneChangeLeft) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  EgoPose ego({.pos = {20.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "lane_change_left";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&map, psmm_modifier);

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(map, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(map, route_path);
  const auto target_lane_path =
      BackwardExtendTargetAlignedRouteLanePath(
          map, !route_path.transitions().front().lc_left,
          route_path.lane_paths()[1].front(), route_path.lane_paths().front())
          .Connect(route_path.lane_paths()[1]);
  auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, target_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr({});
  input.st_traj_mgr = &st_traj_mgr;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, input.trajectory.front(),
      veh_geo_params, st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false);
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  const auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(*end_of_path_boundary_constraint);
  input.constraint_mgr = &constraint_mgr;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &planner_semantic_map_manager;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output = OptimizeTrajectory(input, planner_params, veh_geo_params,
                                   veh_drive_params,
                                   debug_proto.mutable_trajectory_optimizer(),
                                   /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vantage_client_man::FlushAll();
}

TEST(TrajectoryOptimizerTest, NearRightCurb) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  EgoPose ego({.pos = {0.0, -3.8}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_right_curb";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }

  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&map, psmm_modifier);

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(map, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(map, route_path);
  auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr({});
  input.st_traj_mgr = &st_traj_mgr;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, input.trajectory.front(),
      veh_geo_params, st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false);
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  const auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(*end_of_path_boundary_constraint);
  input.constraint_mgr = &constraint_mgr;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &planner_semantic_map_manager;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output = OptimizeTrajectory(input, planner_params, veh_geo_params,
                                   veh_drive_params,
                                   debug_proto.mutable_trajectory_optimizer(),
                                   /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vantage_client_man::FlushAll();
}

TEST(TrajectoryOptimizerTest, NearStopLine) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_stop_line";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&map, psmm_modifier);

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(map, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(map, route_path);
  auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr({});
  input.st_traj_mgr = &st_traj_mgr;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, input.trajectory.front(),
      veh_geo_params, st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false);
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  const auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(*end_of_path_boundary_constraint);
  ConstraintProto::StopLineProto stop_line;
  HalfPlane halfplane(Vec2d(30.0, -1.8), Vec2d(30.0, -5.3));
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_s(30.0 - veh_geo_params.front_edge_to_center());
  stop_line.set_time(0.0);
  stop_line.mutable_source()->mutable_crosswalk()->set_id(0);
  constraint_mgr.AddStopLine(stop_line);
  input.constraint_mgr = &constraint_mgr;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &planner_semantic_map_manager;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output = OptimizeTrajectory(input, planner_params, veh_geo_params,
                                   veh_drive_params,
                                   debug_proto.mutable_trajectory_optimizer(),
                                   /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vantage_client_man::FlushAll();
}

TEST(TrajectoryOptimizerTest, NearSpeedRegionAndStopLine) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 10.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "near_speed_bump_stop_line";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }

  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&map, psmm_modifier);

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(map, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(map, route_path);
  auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  SpacetimeTrajectoryManager st_traj_mgr({});
  input.st_traj_mgr = &st_traj_mgr;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, input.trajectory.front(),
      veh_geo_params, st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false);
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  const auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(*end_of_path_boundary_constraint);
  ConstraintProto::StopLineProto stop_line;
  HalfPlane halfplane(Vec2d(30.0, -1.8), Vec2d(30.0, -5.3));
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_s(30.0);
  stop_line.set_time(0.0);
  stop_line.set_standoff(1.0);
  stop_line.mutable_source()->mutable_crosswalk()->set_id(0);
  constraint_mgr.AddStopLine(stop_line);
  SourceProto speed_bump_source;
  speed_bump_source.mutable_speed_bump()->set_id(1);

  ConstraintProto::SpeedRegionProto speed_bump;
  speed_bump.set_start_s(15.0);
  speed_bump.set_end_s(17.0);
  speed_bump.set_max_speed(2.0);
  speed_bump.set_min_speed(0.0);
  speed_bump.mutable_source()->mutable_speed_bump()->set_id(1);
  speed_bump.set_id(absl::StrFormat("speed_bump_%d", 1));
  constraint_mgr.AddSpeedRegion(speed_bump);
  input.constraint_mgr = &constraint_mgr;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &planner_semantic_map_manager;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output = OptimizeTrajectory(input, planner_params, veh_geo_params,
                                   veh_drive_params,
                                   debug_proto.mutable_trajectory_optimizer(),
                                   /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vantage_client_man::FlushAll();
}

TEST(TrajectoryOptimizerTest, RightNudgeObject) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  EgoPose ego({.pos = {0.0, 0.0}, .theta = 0.0, .v = 3.0});
  const double cos_theta = std::cos(ego.theta);
  const double sin_theta = std::sin(ego.theta);
  const std::string test = "right_nudge";

  TrajectoryOptimizerInput input;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int k = 0; k < kTrajectorySteps; ++k) {
    auto& pt = traj_points.emplace_back();
    auto* path_point = pt.mutable_path_point();
    path_point->set_x(ego.pos.x() +
                      k * kTrajectoryTimeStep * ego.v * cos_theta);
    path_point->set_y(ego.pos.y() +
                      k * kTrajectoryTimeStep * ego.v * sin_theta);
    path_point->set_z(0.0);
    path_point->set_theta(ego.theta);
    path_point->set_kappa(0.0);
    path_point->set_lambda(0.0);
    path_point->set_s(ego.v * k * kTrajectoryTimeStep);
    pt.set_v(ego.v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(k * kTrajectoryTimeStep);
  }
  input.trajectory = traj_points;

  // For space_time object and drive_passage.
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&map, psmm_modifier);

  // Build drive passage.
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(ego.pos.x());
  pose.mutable_pos_smooth()->set_y(ego.pos.y());
  pose.set_yaw(ego.theta);
  pose.mutable_vel_smooth()->set_x(ego.v * cos_theta);
  pose.mutable_vel_smooth()->set_y(ego.v * sin_theta);

  const auto route_path = RoutingToNameSpot(map, pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(map, route_path);
  auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      kDrivePassageKeepBehindLength);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";
  input.drive_passage = &*drive_passage;

  // Build spacetime object manager.
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d(10.0, 2.0))
                                  .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(Vec2d(10.0, 2.0), Vec2d(10.5, 2.0),
                         /*init_v=*/0.0, /*last_v=*/0.1);
  const PlannerObject object = builder.Build();
  SpacetimeTrajectoryManager st_traj_mgr(absl::MakeSpan(&object, 1));
  input.st_traj_mgr = &st_traj_mgr;

  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, input.trajectory.front(),
      veh_geo_params, st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false);
  QCHECK(path_sl_boundary.ok());
  input.path_sl_boundary = &*path_sl_boundary;

  SendDrivePassageToCanvas(
      *drive_passage,
      absl::StrCat("trajectory_optimizer_test/", test, "/drive_passage"));

  EstPlannerDebugProto debug_proto;

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  const auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(*end_of_path_boundary_constraint);
  input.constraint_mgr = &constraint_mgr;

  input.plan_start_point = input.trajectory.front();
  auto fake_time = Clock::Now();
  input.plan_start_time = fake_time;
  input.planner_semantic_map_mgr = &planner_semantic_map_manager;

  ThreadPool* tp = ThreadPool::DefaultPool();
  auto output = OptimizeTrajectory(input, planner_params, veh_geo_params,
                                   veh_drive_params,
                                   debug_proto.mutable_trajectory_optimizer(),
                                   /*charts_data=*/nullptr, tp);

  if (output.ok()) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&input](int index) {
              return Vec2d(input.trajectory[index].path_point().x(),
                           input.trajectory[index].path_point().y());
            },
            input.trajectory.size()),
        vis::Color::kLightRed,
        absl::StrCat("trajectory_optimizer_test/", test, "/input"));

    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&output](int index) {
              return Vec2d(output->trajectory_proto[index].path_point().x(),
                           output->trajectory_proto[index].path_point().y());
            },
            output->trajectory_proto.size()),
        vis::Color::kLightBlue,
        absl::StrCat("trajectory_optimizer_test/", test, "/output"));
  }

  vantage_client_man::FlushAll();
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
