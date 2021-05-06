#include "onboard/planner/freespace/ipopt_global_path_smoother.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

const PlannerParamsProto planner_params = DefaultPlannerParams();
const VehicleGeometryParamsProto vehicle_geo_params = DefaultVehicleGeometry();
const VehicleDriveParamsProto vehicle_drive_params =
    DefaultVehicleDriveParams();

TEST(GlobalSmootherTest, ReverseParkingTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  std::vector<Vec2d> boundary1 = {Vec2d(20.0, 3.5),  Vec2d(13.0, 3.5),
                                  Vec2d(13.0, 10.0), Vec2d(10.0, 10.0),
                                  Vec2d(10.0, 3.5),  Vec2d(-2.0, 3.5)};
  std::vector<FreespaceBoundary> boundaries;
  boundaries.reserve(boundary1.size() - 1);
  for (int i = 0; i + 1 < boundary1.size(); ++i) {
    boundaries.push_back(
        {.id = "b" + std::to_string(i),
         .type = FreespaceMapProto::CURB,
         .segment = Segment2d(boundary1[i], boundary1[i + 1])});
  }
  FreespaceMap freespace_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                                .boundaries = boundaries};
  std::vector<PlannerObject> objects;
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;

  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_theta(0.0);

  PathPoint end;
  end.set_x(11.5);
  end.set_y(8.0);
  end.set_theta(-M_PI * 0.5);

  HybridAStartDebugProto ha_debug_info;
  const auto coarse_path_status = FindPath(
      planner_params.freespace_params_for_parking().hybrid_a_star_params(),
      vehicle_geo_params, vehicle_drive_params,
      FreespaceTaskProto::PERPENDICULAR_PARKING, freespace_map,
      stalled_object_trajs, start, end, &ha_debug_info);

  if (coarse_path_status.ok()) {
    const auto ret = SmoothGlobalPath(*coarse_path_status, vehicle_geo_params,
                                      vehicle_drive_params);
    ASSERT_TRUE(ret.ok());
  }
}

TEST(GlobalSmootherTest, PullOverTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  std::vector<Vec2d> boundary1 = {Vec2d(20.0, -3.0), Vec2d(13.0, -3.0),
                                  Vec2d(13.0, -6.0), Vec2d(5.0, -6.0),
                                  Vec2d(5.0, -3.0),  Vec2d(-2.0, -3.0)};
  std::vector<FreespaceBoundary> boundaries;
  boundaries.reserve(boundary1.size() - 1);
  for (int i = 0; i + 1 < boundary1.size(); ++i) {
    boundaries.push_back(
        {.id = "b" + std::to_string(i),
         .type = FreespaceMapProto::CURB,
         .segment = Segment2d(boundary1[i], boundary1[i + 1])});
  }
  FreespaceMap freespace_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                                .boundaries = boundaries};
  std::vector<std::pair<std::string, Polygon2d>> stationary_objects = {};
  std::vector<PlannerObject> objects;
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;

  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_theta(0.0);

  PathPoint end;
  end.set_x(6.5);
  end.set_y(-4.5);
  end.set_theta(0.0);

  HybridAStartDebugProto ha_debug_info;
  const auto coarse_path_status = FindPath(
      planner_params.freespace_params_for_parking().hybrid_a_star_params(),
      vehicle_geo_params, vehicle_drive_params,
      FreespaceTaskProto::PARALLEL_PARKING, freespace_map, stalled_object_trajs,
      start, end, &ha_debug_info);

  if (coarse_path_status.ok()) {
    const auto ret = SmoothGlobalPath(*coarse_path_status, vehicle_geo_params,
                                      vehicle_drive_params);
    ASSERT_TRUE(ret.ok());
  }
}

TEST(GlobalSmootherTest, UTurnTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  FreespaceMap freespace_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                                .boundaries = {}};
  std::vector<std::pair<std::string, Polygon2d>> stationary_objects = {};
  std::vector<PlannerObject> objects;
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;

  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_theta(0.0);

  PathPoint end;
  end.set_x(0.0);
  end.set_y(0.0);
  end.set_theta(M_PI);

  HybridAStartDebugProto ha_debug_info;
  const auto coarse_path_status = FindPath(
      planner_params.freespace_params_for_parking().hybrid_a_star_params(),
      vehicle_geo_params, vehicle_drive_params,
      FreespaceTaskProto::FREE_DRIVING, freespace_map, stalled_object_trajs,
      start, end, &ha_debug_info);

  if (coarse_path_status.ok()) {
    const auto ret = SmoothGlobalPath(*coarse_path_status, vehicle_geo_params,
                                      vehicle_drive_params);
    ASSERT_TRUE(ret.ok());
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
