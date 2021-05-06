#include "onboard/planner/decision/decision_util.h"

#include "gtest/gtest.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"

namespace qcraft {
namespace planner {
TEST(CreateSpeedProfileTest, CreateSpeedProfile) {
  // Load map
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  auto start_time = absl::Now();
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  start_time = absl::Now();
  const auto route_path =
      RoutingToNameSpot(semantic_map_manager, pose, "7a_n2");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(semantic_map_manager, route_path);

  start_time = absl::Now();
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager),
      /*keep_bebind_len=*/0.0);

  ASSERT_TRUE(drive_passage.ok() && !drive_passage.value().empty())
      << "Building drive passage failed!";

  const auto& passage = drive_passage.value();

  const double v_now = 5.0;  // m/s
  std::vector<ConstraintProto::SpeedRegionProto> speed_zone_vector;
  auto& speed_zone = speed_zone_vector.emplace_back();
  speed_zone.set_start_s(20.0);
  speed_zone.set_end_s(25.0);
  speed_zone.set_max_speed(10.0);

  std::vector<ConstraintProto::StopLineProto> stop_point_vector;
  auto& stop_point = stop_point_vector.emplace_back();
  stop_point.set_s(60.0);

  absl::Span<ConstraintProto::SpeedRegionProto> speed_zones =
      absl::MakeSpan(speed_zone_vector);
  absl::Span<ConstraintProto::StopLineProto> stop_points =
      absl::MakeSpan(stop_point_vector);

  const std::vector<double> v_s = CreateSpeedProfileWithConstraints(
      v_now, passage, planner_semantic_map_manager, speed_zones, stop_points);

  LOG(INFO) << "Speed profile:";
  for (int i = 0; i < drive_passage->size(); ++i) {
    LOG(INFO) << "i = " << i << " path_s = "
              << drive_passage->station(StationIndex(i)).accumulated_s()
              << " v = " << (v_s)[i];
  }

  const SpeedProfile speed_profile = IntegrateSpeedProfile(passage, v_s);
}

}  // namespace planner
}  // namespace qcraft
