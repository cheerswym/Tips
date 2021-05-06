#include "onboard/planner/decision/ignore_object.h"

#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
namespace {
// Data generated from 'dojo.planner.ignore_object_1.pb.txt'. See
// (https://drive.google.com/file/d/1la7sKwvQo0gzU1SP_F7kKfIwJzofpTJR/view?usp=sharing)
TEST(GetIgnoreObjectsTest, GetIgnoreObjects) {
  PerceptionObjectBuilder perception_builder;
  const Vec2d obj_pos(140.6, 65.6);
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEGETATION)
                                  .set_timestamp(1.0)
                                  .set_yaw(0.0)
                                  .set_velocity(0.0)
                                  .set_length_width(1.0, 1.0)
                                  .set_pos(obj_pos)
                                  .set_box_center(Vec2d(140.6, 65.6))
                                  .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEGETATION)
      .set_object(perception_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(obj_pos, /*theta=*/0.0)
      .set_probability(1.0);
  const PlannerObject object = builder.Build();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(132.0);
  pose.mutable_pos_smooth()->set_y(66.6);
  pose.mutable_vel_smooth()->set_x(11.0);
  pose.mutable_vel_smooth()->set_y(0.0);
  pose.set_yaw(0.0);

  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  const auto& route_path = RoutingToNameSpot(semantic_map_manager, pose,
                                             /*name_spot=*/"a8_e3_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);
  const auto& drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);
  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";
  const auto& passage = drive_passage.value();

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const SpacetimeTrajectoryManager st_traj_mgr(absl::MakeSpan(&object, 1));
  const auto ignore_objects =
      FindObjectsToIgnore(vehicle_geometry_params, passage, st_traj_mgr, pose);
  const std::string traj_id =
      SpacetimeObjectTrajectory::MakeTrajectoryId("Agent0", 0);
  EXPECT_EQ(ignore_objects.size(), 1);
  EXPECT_EQ(ignore_objects[0].traj_id(), traj_id);
  EXPECT_EQ(ignore_objects[0].reason(),
            ConstraintProto::IgnoreObjectProto::CANNOT_STOP_BEFORE_VEGETATION);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
