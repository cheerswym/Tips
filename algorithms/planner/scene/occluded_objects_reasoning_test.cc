#include "onboard/planner/scene/occluded_objects_reasoning.h"

#include "gtest/gtest.h"
#include "onboard/perception/sensor_fov/sensor_fov_test_util.h"
#include "onboard/perception/test_util/obstacle_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
TEST(OccludedObjectsReasoningTest, BaseTest) {
  // Build planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Build route sections.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time),
                                        Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);

  // Build sensor fov.
  auto sensor_fov_builder = sensor_fov::BuildSensorFovBuilder();
  const auto& [clusters, obstacle_refs] = sensor_fov::BuildSegmentedClusters();
  const auto obstacle_ptrs =
      ConstructObstaclePtrsFromObstacleRefVector(obstacle_refs);
  const auto sensor_fovs = sensor_fov_builder->Compute(obstacle_ptrs, clusters,
                                                       {0.0, VehiclePose()});
  const auto sensor_fov_lidar = GetLidarViewSensorFov(sensor_fovs);

  ASSIGN_OR_DIE(const auto result,
                RunOccludedObjectsReasoning(OccludedObjectsReasoningInput{
                    .psmm = &psmm,
                    .sensor_fov = sensor_fov_lidar.get(),
                    .route_sections = &route_sections,
                    .prediction = nullptr /*Delete prediction later*/
                }));

  EXPECT_TRUE(true);
}
}  // namespace
}  // namespace qcraft::planner
