#include "onboard/planner/freespace/path_manager_util.h"

#include <algorithm>

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/freespace/freespace_planner_plot_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
void DrawObject(std::string_view name,
                const SpacetimeTrajectoryManager &st_traj_mgr) {
  vis::Canvas &canvas = vantage_client_man::GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto traj : st_traj_mgr.stationary_object_trajs()) {
    canvas.DrawPolygon(traj->planner_object()->contour(), /*z*/ 0.0,
                       vis::Color::kLightGreen);
  }
  vantage_client_man::FlushAll();
}

struct State {
  Vec2d pos;
  double theta;  // Heading angle.
  double delta;  // Steering angle.
};

struct AvInputState {
  Vec2d av_pos;
  Vec2d av_v;
  double av_theta;  // Heading angle.
  double av_delta;  // Steering angle.
  double phi;       // Steering angle rate.
  double accel;
};

// TODO(yumeng): Try to reuse ThirdOrderBicycleDdp::EvaluateF to generate
// third-order trajectory, whose curvature is continuous.
std::vector<State> GenerateKinematicPath(
    const qcraft::VehicleGeometryParamsProto &vehicle_gem,
    const VehicleDriveParamsProto &vehicle_drive,
    const AvInputState &input_state, double delta_t, int generate_states_num,
    bool is_forward) {
  // This value calculated by robot-bus Q8001 (max_steer_angle / steer_ratio).
  const double max_delta =
      vehicle_drive.max_steer_angle() / vehicle_drive.steer_ratio();

  std::vector<State> states;
  states.reserve(generate_states_num);
  // Add init state.
  states.push_back({.pos = input_state.av_pos,
                    .theta = input_state.av_theta,
                    .delta = input_state.av_delta});

  double v = input_state.av_v.norm();
  for (int i = 0; i < generate_states_num; ++i) {
    // Kinematic bicycle model.
    const Vec2d xy_dot = v * Vec2d::FastUnitFromAngle(states[i].theta);
    const double theta_dot = v * std::tan(states[i].delta) /
                             vehicle_gem.wheel_base();  // rotation rate (𝜔)
    const double delta_dot = input_state.phi;
    v = v + input_state.accel * delta_t;

    // Derive next state.
    State next_state;
    next_state.pos = states[i].pos + xy_dot * delta_t;
    next_state.theta = states[i].theta + theta_dot * delta_t;
    next_state.delta = states[i].delta + delta_dot * delta_t;
    next_state.delta = std::clamp(next_state.delta, -max_delta, max_delta);
    states.push_back(std::move(next_state));
  }
  return states;
}

std::vector<DirectionalPath> GenerateDirectionalPath(
    const VehicleGeometryParamsProto &vehicle_geom,
    const VehicleDriveParamsProto &vehicle_drive) {
  constexpr double kDt = 0.1;  // s
  constexpr int kStatesNum = 50;
  Vec2d av_pos_smooth(40.0, 10.0);
  AvInputState forward_input_state{.av_pos = av_pos_smooth,
                                   .av_v = Vec2d(0.0, 0.0),
                                   .av_theta = 0.0,
                                   .av_delta = -0.45,
                                   .phi = 0.0,
                                   .accel = 1.0};
  // Forward
  const std::vector<State> forward_states =
      GenerateKinematicPath(vehicle_geom, vehicle_drive, forward_input_state,
                            kDt, kStatesNum, /*is_forward*/ true);

  std::vector<PathPoint> path_points_forward;
  path_points_forward.reserve(forward_states.size());
  for (int i = 0; i < forward_states.size(); ++i) {
    PathPoint point;
    point.set_x(forward_states[i].pos.x());
    point.set_y(forward_states[i].pos.y());
    point.set_theta(forward_states[i].theta);
    path_points_forward.push_back(std::move(point));
  }
  DiscretizedPath path_forward(std::move(path_points_forward));

  AvInputState backward_input_state{.av_pos = forward_states.back().pos,
                                    .av_v = Vec2d(0.0, 0.0),
                                    .av_theta = forward_states.back().theta,
                                    .av_delta = 0.45,
                                    .phi = 0.0,
                                    .accel = -1.0};
  // Backward
  const std::vector<State> backwark_states = GenerateKinematicPath(
      vehicle_geom, vehicle_drive, backward_input_state, kDt, kStatesNum,
      /*is_forward*/ false);
  std::vector<PathPoint> path_points_backward;
  path_points_backward.reserve(backwark_states.size());
  for (int i = 0; i < backwark_states.size(); ++i) {
    PathPoint point;
    point.set_x(backwark_states[i].pos.x());
    point.set_y(backwark_states[i].pos.y());
    point.set_theta(NormalizeAngle(backwark_states[i].theta + M_PI));
    path_points_backward.push_back(std::move(point));
  }
  DiscretizedPath path_backward(std::move(path_points_backward));

  std::vector<DirectionalPath> paths;
  paths.push_back({.path = std::move(path_forward), .forward = true});
  paths.push_back({.path = std::move(path_backward), .forward = false});
  DrawDirectionalPath("freespace_planner/path", paths);
  return paths;
}

TEST(FreespacePlannerUtilTest, PathSafetyCheck) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();
  const VehicleDriveParamsProto vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  std::vector<PlannerObject> planner_objects;
  {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("Agent1")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(0.0)
                                    .set_yaw(0)
                                    .set_length_width(1.3, 1.3)
                                    .set_pos(Vec2d(46.586, 9.248))
                                    .set_box_center(Vec2d(46.586, 9.248))
                                    .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  DrawObject("freespace_planner/st_traj", st_traj_mgr);

  std::vector<DirectionalPath> paths =
      GenerateDirectionalPath(vehicle_geom, vehicle_drive_params);

  constexpr int current_index = 0;
  {
    // Just visualize path swept volume.
    std::vector<const DirectionalPath *> global_paths_remain;
    global_paths_remain.reserve(paths.size() - current_index - 1);
    for (int i = current_index + 1; i < paths.size(); ++i) {
      global_paths_remain.push_back(&paths[i]);
    }
    constexpr double kLengthBuffer = 0.25;
    constexpr double kWidthBuffer = 0.25;
    const auto path_swept_volume = PathSweptVolume(
        vehicle_geom, global_paths_remain, kLengthBuffer, kWidthBuffer);
    DrawPathSweptVolume("freespace_planner/path_swept_volume",
                        path_swept_volume);
  }

  const auto res =
      PathSafetyCheck(vehicle_geom, st_traj_mgr.stationary_object_trajs(),
                      paths, current_index);
  EXPECT_NE(res, absl::OkStatus());
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
