#include "onboard/planner/freespace/tob_path_smoother.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

const PlannerParamsProto planner_params = DefaultPlannerParams();
const VehicleGeometryParamsProto vehicle_geo_params = DefaultVehicleGeometry();
const VehicleDriveParamsProto vehicle_drive_params =
    DefaultVehicleDriveParams();

TEST(LocalSmootherTest, ReverseParking) {
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
         .type = FreespaceMapProto::PARKING_SPOT,
         .segment = Segment2d(boundary1[i], boundary1[i + 1])});
  }
  FreespaceMap static_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                             .boundaries = boundaries};
  // Add objects.
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("unit_test")
                            .set_type(OT_VEHICLE)
                            .set_length_width(1.8, 1.8)
                            .set_box_center(Vec2d(13.75, 4.52))
                            .set_pos(Vec2d(13.75, 4.52))
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE).set_object(perception_obj).set_stationary(true);
  objects.push_back(builder.Build());
  SpacetimeTrajectoryManager st_traj_mgr(std::move(objects));
  std::vector<const SpacetimeObjectTrajectory *> stalled_object_trajs;
  stalled_object_trajs.reserve(st_traj_mgr.trajectories().size());
  for (const auto traj : st_traj_mgr.stationary_object_trajs()) {
    stalled_object_trajs.push_back(traj);
  }

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
      FreespaceTaskProto::PERPENDICULAR_PARKING, static_map,
      stalled_object_trajs, start, end, &ha_debug_info);
  LOG(INFO) << "Hybrid A star status: "
            << coarse_path_status.status().ToString();
  ASSERT_TRUE(coarse_path_status.ok());

  FreespaceLocalSmootherDebugProto smoother_debug_info;
  vis::vantage::ChartsDataProto smoother_chart_info;
  absl::flat_hash_set<std::string> stalled_objects;
  std::vector<PathPoint> prev_path = {};
  for (const auto &path : *coarse_path_status) {
    TrajectoryPoint plan_start_point;
    plan_start_point.set_pos(
        Vec2d(path.path.front().x(), path.path.front().y()));
    plan_start_point.set_theta(
        path.forward ? path.path.front().theta()
                     : NormalizeAngle(path.path.front().theta() + M_PI));
    plan_start_point.set_kappa(path.forward ? path.path.front().kappa()
                                            : -path.path.front().kappa());
    const auto res = SmoothLocalPath(
        vehicle_geo_params, vehicle_drive_params,
        planner_params.freespace_params_for_parking()
            .motion_constraint_params(),
        planner_params.freespace_params_for_parking().local_smoother_params(),
        planner_params.trajectory_optimizer_vehicle_model_params(),
        /*owner=*/"freesspace", static_map, st_traj_mgr, stalled_objects, path,
        plan_start_point,
        /*reset=*/true, prev_path, &smoother_debug_info, &smoother_chart_info);
    LOG(INFO) << "Path smoother status: " << res.status().ToString();
    ASSERT_TRUE(res.ok());
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
