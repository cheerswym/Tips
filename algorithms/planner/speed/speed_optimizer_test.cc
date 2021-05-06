#include "onboard/planner/speed/speed_optimizer.h"

#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {
namespace {
using ParamProto = SpeedFinderParamsProto;
TEST(SpeedOptimizerTest, SimpleTest) {
  auto planner_params = DefaultPlannerParams();
  ApolloTrajectoryPointProto init_point;
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(5.0, 0.0), StPoint(5.0, 5.0)};
  st_boundary_points.upper_points = {StPoint(50.0, 0.0), StPoint(50.0, 5.0)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0), VtPoint(0.0, 10.0)};
  StBoundaryRef st_boundary = StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, "001", /*probability=*/1.0,
      /*is_stationary=*/true);
  absl::flat_hash_map<ParamProto::SpeedLimitType, SpeedLimit> speed_limit_map;
  std::vector<SpeedLimit::SpeedLimitRange> range;
  range.push_back({.start_s = 0.0, .end_s = 100.0, .speed_limit = 5.0});
  SpeedLimit lane_limit(range, /*defalut_speed_limit=*/20.0);
  speed_limit_map.emplace(ParamProto::LANE, lane_limit);
  speed_limit_map.emplace(ParamProto::COMBINATION, lane_limit);

  std::vector<StBoundaryWithDecision> st_boundaries;
  st_boundaries.emplace_back(
      std::move(st_boundary), StBoundaryProto::FOLLOW,
      StBoundaryProto::UNKNOWN_REASON, /*decision_info=*/"",
      /*follow_standstill_distance=*/4.0, /*lead_standstill_distance=*/4.0,
      /*pass_time=*/0.0, /*yield_time=*/0.0);

  SpeedVector reference_speed;
  for (int i = 0; i < 100; ++i) {
    const double time = 0.1 * i;
    reference_speed.emplace_back(time, 5.0 * time, 5.0, 0.0, 0.0);
  }

  SpeedFinderDebugProto speed_finder_debug;
  SpeedVector speed_data;

  constexpr double max_path_length = 100.0;
  std::unique_ptr<SpeedOptimizer> optimizer = std::make_unique<SpeedOptimizer>(
      &planner_params.motion_constraint_params(),
      &planner_params.speed_finder_params(), max_path_length,
      planner_params.motion_constraint_params().default_speed_limit(),
      /*kTrajectorySteps=*/100);

  absl::Status ret =
      optimizer->Optimize(init_point, st_boundaries, speed_limit_map,
                          reference_speed, &speed_data, &speed_finder_debug);
  EXPECT_TRUE(ret.ok());

  for (const auto& res : speed_data) {
    QLOG(INFO) << "s[" << res.s() << "], t[" << res.t() << "], v[" << res.v()
               << "], a[" << res.a() << "].";
  }
}

}  // namespace
}  // namespace qcraft::planner
