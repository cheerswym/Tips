#include "onboard/planner/scheduler/target_lane_clearance.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {
namespace {

TEST(TargetLaneClearance, RampRefPath) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const mapping::LanePath target_lane_path(&semantic_map_manager, {2332, 2339},
                                           /*start_fraction=*/0.0,
                                           /*end_fraction=*/1.0);
  DrawLanePathToCanvas(semantic_map_manager, target_lane_path,
                       "ramp_target_lane_path", vis::Color::kMint);

  const Vec2d pos = target_lane_path.front().ComputePos(semantic_map_manager);
  const Vec2d tangent =
      target_lane_path.front().ComputeTangent(semantic_map_manager);
  ApolloTrajectoryPointProto start_point;
  start_point.mutable_path_point()->set_s(0.0);
  start_point.mutable_path_point()->set_x(pos[0]);
  start_point.mutable_path_point()->set_y(pos[1]);
  start_point.mutable_path_point()->set_theta(tangent.FastAngle());
  start_point.set_v(5.0);

  const SpacetimeTrajectoryManager st_traj_mgr({});

  // params
  const auto &vehicle_geom = DefaultVehicleGeometry();
  const auto &planner_params = DefaultPlannerParams();

  start_time = absl::Now();
  const auto res = CheckTargetLaneClearance(
      planner_semantic_map_manager, target_lane_path, start_point, st_traj_mgr,
      /*stalled_objects=*/{}, vehicle_geom, planner_params, 0.5,
      /*thread_pool=*/nullptr);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in safety check";

  ASSERT_TRUE(res.ok());
  SendPointsToCanvas(absl::MakeSpan(res.value().pseudo_traj),
                     "ramp_pseudo_traj", vis::Color::kOrange);
}

TEST(TargetLaneClearance, StationaryCollision) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const mapping::LanePath target_lane_path(&semantic_map_manager, {2472},
                                           /*start_fraction=*/0.0,
                                           /*end_fraction=*/1.0);
  DrawLanePathToCanvas(semantic_map_manager, target_lane_path,
                       "straight_target_lane_path", vis::Color::kMint);

  const Vec2d pos = target_lane_path.front().ComputePos(semantic_map_manager);
  const Vec2d tangent =
      target_lane_path.front().ComputeTangent(semantic_map_manager);
  ApolloTrajectoryPointProto start_point;
  start_point.mutable_path_point()->set_s(0.0);
  start_point.mutable_path_point()->set_x(pos[0]);
  start_point.mutable_path_point()->set_y(pos[1]);
  start_point.mutable_path_point()->set_theta(tangent.FastAngle());
  start_point.set_v(5.0);

  // objects
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("st_collision")
                            .set_type(OT_VEHICLE)
                            .set_pos(Vec2d(130.0, -3.5))
                            .set_yaw(0.0)
                            .set_length_width(4.0, 2.0)
                            .Build();
  objects.push_back(PlannerObjectBuilder()
                        .set_object(perception_obj)
                        .set_stationary(true)
                        .Build());

  const SpacetimeTrajectoryManager st_traj_mgr(objects);

  // params
  const auto &vehicle_geom = DefaultVehicleGeometry();
  const auto &planner_params = DefaultPlannerParams();

  start_time = absl::Now();
  const auto res = CheckTargetLaneClearance(
      planner_semantic_map_manager, target_lane_path, start_point, st_traj_mgr,
      /*stalled_objects=*/{}, vehicle_geom, planner_params, 0.5,
      /*thread_pool=*/nullptr);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in safety check";

  ASSERT_FALSE(res.ok());
}

TEST(TargetLaneClearance, StationaryNoCollision) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const mapping::LanePath target_lane_path(&semantic_map_manager, {2472},
                                           /*start_fraction=*/0.0,
                                           /*end_fraction=*/1.0);
  DrawLanePathToCanvas(semantic_map_manager, target_lane_path,
                       "straight_target_lane_path", vis::Color::kMint);

  const Vec2d pos = target_lane_path.front().ComputePos(semantic_map_manager);
  const Vec2d tangent =
      target_lane_path.front().ComputeTangent(semantic_map_manager);
  ApolloTrajectoryPointProto start_point;
  start_point.mutable_path_point()->set_s(0.0);
  start_point.mutable_path_point()->set_x(pos[0]);
  start_point.mutable_path_point()->set_y(pos[1]);
  start_point.mutable_path_point()->set_theta(tangent.FastAngle());
  start_point.set_v(5.0);

  // objects
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("st_no_collision")
                            .set_type(OT_VEHICLE)
                            .set_pos(Vec2d(110.0, -3.5))
                            .set_yaw(0.0)
                            .set_length_width(4.0, 2.0)
                            .Build();
  objects.push_back(PlannerObjectBuilder()
                        .set_object(perception_obj)
                        .set_stationary(true)
                        .Build());

  perception_obj = perception_builder.set_id("st_not_on_lane")
                       .set_type(OT_VEHICLE)
                       .set_pos(Vec2d(125.0, 3.5))
                       .set_yaw(0.0)
                       .set_length_width(4.0, 2.0)
                       .Build();
  objects.push_back(PlannerObjectBuilder()
                        .set_object(perception_obj)
                        .set_stationary(true)
                        .Build());

  const SpacetimeTrajectoryManager st_traj_mgr(objects);

  // params
  const auto &vehicle_geom = DefaultVehicleGeometry();
  const auto &planner_params = DefaultPlannerParams();

  start_time = absl::Now();
  const auto res = CheckTargetLaneClearance(
      planner_semantic_map_manager, target_lane_path, start_point, st_traj_mgr,
      /*stalled_objects=*/{}, vehicle_geom, planner_params, 0.5,
      /*thread_pool=*/nullptr);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in safety check";

  ASSERT_TRUE(res.ok());
  SendPointsToCanvas(absl::MakeSpan(res.value().pseudo_traj), "pseudo_traj",
                     vis::Color::kOrange);
}

TEST(TargetLaneClearance, MovingCollision) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const mapping::LanePath target_lane_path(&semantic_map_manager, {2472},
                                           /*start_fraction=*/0.0,
                                           /*end_fraction=*/1.0);
  DrawLanePathToCanvas(semantic_map_manager, target_lane_path,
                       "straight_target_lane_path", vis::Color::kMint);

  const Vec2d pos = target_lane_path.front().ComputePos(semantic_map_manager);
  const Vec2d tangent =
      target_lane_path.front().ComputeTangent(semantic_map_manager);
  ApolloTrajectoryPointProto start_point;
  start_point.mutable_path_point()->set_s(0.0);
  start_point.mutable_path_point()->set_x(pos[0]);
  start_point.mutable_path_point()->set_y(pos[1]);
  start_point.mutable_path_point()->set_theta(tangent.FastAngle());
  start_point.set_v(5.0);

  // objects
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("mv_collision")
                            .set_type(OT_VEHICLE)
                            .set_pos(Vec2d(125.0, -3.5))
                            .set_yaw(0.0)
                            .set_timestamp(0.0)
                            .set_velocity(6.0)
                            .set_length_width(4.0, 2.0)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d(125.0, -3.5), Vec2d(155.0, -3.5),
                         /*init_v=*/3.0, /*last_v=*/3.0);

  objects.push_back(builder.Build());

  const SpacetimeTrajectoryManager st_traj_mgr(objects);

  // params
  const auto &vehicle_geom = DefaultVehicleGeometry();
  const auto &planner_params = DefaultPlannerParams();

  start_time = absl::Now();
  const auto res = CheckTargetLaneClearance(
      planner_semantic_map_manager, target_lane_path, start_point, st_traj_mgr,
      /*stalled_objects=*/{}, vehicle_geom, planner_params, 0.5,
      /*thread_pool=*/nullptr);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in safety check";

  ASSERT_FALSE(res.ok());
}

TEST(TargetLaneClearance, MovingNoCollision) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();
  PlannerSemanticMapModification modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in loading map";

  const mapping::LanePath target_lane_path(&semantic_map_manager, {2472},
                                           /*start_fraction=*/0.0,
                                           /*end_fraction=*/1.0);
  DrawLanePathToCanvas(semantic_map_manager, target_lane_path,
                       "straight_target_lane_path", vis::Color::kMint);

  const Vec2d pos = target_lane_path.front().ComputePos(semantic_map_manager);
  const Vec2d tangent =
      target_lane_path.front().ComputeTangent(semantic_map_manager);
  ApolloTrajectoryPointProto start_point;
  start_point.mutable_path_point()->set_s(0.0);
  start_point.mutable_path_point()->set_x(pos[0]);
  start_point.mutable_path_point()->set_y(pos[1]);
  start_point.mutable_path_point()->set_theta(tangent.FastAngle());
  start_point.set_v(5.0);

  // objects
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("mv_collision")
                            .set_type(OT_VEHICLE)
                            .set_pos(Vec2d(130.0, -3.5))
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
      ->set_probability(0.5)
      .set_straight_line(Vec2d(130.0, -3.5), Vec2d(180.0, -3.5),
                         /*init_v=*/5.0, /*last_v=*/5.0);

  objects.push_back(builder.Build());

  const SpacetimeTrajectoryManager st_traj_mgr(objects);

  // params
  const auto &vehicle_geom = DefaultVehicleGeometry();
  const auto &planner_params = DefaultPlannerParams();

  start_time = absl::Now();
  const auto res = CheckTargetLaneClearance(
      planner_semantic_map_manager, target_lane_path, start_point, st_traj_mgr,
      /*stalled_objects=*/{}, vehicle_geom, planner_params, 0.5,
      /*thread_pool=*/nullptr);
  QLOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
             << " ms consumed in safety check";

  ASSERT_TRUE(res.ok());
  const auto decision =
      FindOrDie(res.value().objects_decision, "mv_collision-idx0");
  EXPECT_EQ(decision, ClearanceCheckOutput::LC_FOLLOW);
}

TEST(DecideClearanceForMovingObject, LeadSituation) {
  {
    // TRIAG-44281
    const double aggressiveness = 2.0;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 4.67,
        .max_brake = kEgoMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(28.0, 0.0), 6.0, 2.0),
        .id = "ego"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = ComputeObjectResponseTime(aggressiveness, -1.0),
        .current_v = 11.37,
        .max_v = 50.0,
        .min_brake = ComputeVehicleMinBrake(11.37, aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 6.5, 2.0),
        .id = "rear object"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_LEAD);
  }

  {
    // TRIAG-44025
    const double aggressiveness = 1.8;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 5.17,
        .max_brake = kEgoMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(42.0, 0.0), 6.0, 2.0),
        .id = "ego"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = ComputeObjectResponseTime(aggressiveness, 1.4),
        .current_v = 13.15,
        .max_v = 50.0,
        .min_brake = ComputeVehicleMinBrake(13.15, aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 6.5, 2.0),
        .id = "rear object"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_LEAD);
  }
}

TEST(DecideClearanceForMovingObject, DangerousSituation) {
  {
    // TRIAG-44908
    const double aggressiveness = 2.0;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 3.36,
        .max_brake = kEgoMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(22.0, 0.0), 6.0, 2.0),
        .id = "ego"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = ComputeObjectResponseTime(aggressiveness, 0.3),
        .current_v = 11.97,
        .max_v = 50.0,
        .min_brake = ComputeVehicleMinBrake(11.37, aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 3.7, 2.0),
        .id = "rear object"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_UNSAFE);
  }

  {
    // TRIAG-29041
    const double aggressiveness = 1.0;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 16.19,
        .max_brake = kEgoMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(22.0, 0.0), 6.0, 2.0),
        .id = "ego"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = ComputeObjectResponseTime(aggressiveness, 2.47),
        .current_v = 19.72,
        .max_v = 50.0,
        .min_brake = ComputeVehicleMinBrake(19.72, aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 10.0, 2.0),
        .id = "rear object"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_UNSAFE);
  }

  {
    // TRIAG-43970
    const double aggressiveness = 1.8;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 13.56,
        .max_brake = kEgoMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(8.13, 0.0), 6.0, 2.0),
        .id = "ego"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = ComputeObjectResponseTime(aggressiveness, 0.19),
        .current_v = 14.76,
        .max_v = 50.0,
        .min_brake = ComputeVehicleMinBrake(14.76, aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 5.62, 2.0),
        .id = "rear object"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_UNSAFE);
  }

  {
    //   TRIAG-66211
    const double aggressiveness = 1.0;
    const RssLongitudialFormulas::VehicleState front_state{
        .current_v = 18.0,
        .max_brake = kVehicleMaxBrakePlf(aggressiveness),
        .vehicle_box = AABox2d(Vec2d(14.0, 0.0), 6.0, 2.0),
        .id = "front object"};

    const RssLongitudialFormulas::VehicleState rear_state{
        .response_time = 0.3,
        .current_v = 21.0,
        .max_v = 50.0,
        .min_brake = kEgoMinBrakePlf(aggressiveness),
        .max_accel = 0.0,
        .vehicle_box = AABox2d(Vec2d(0.0, 0.0), 5.62, 2.0),
        .id = "ego"};

    const auto decision =
        DecideClearanceForMovingObject(front_state, rear_state, aggressiveness);

    EXPECT_EQ(decision, ClearanceCheckOutput::LC_UNSAFE);
  }
}

}  // namespace
}  // namespace qcraft::planner
