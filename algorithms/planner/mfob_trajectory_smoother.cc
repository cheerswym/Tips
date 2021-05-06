#include "onboard/planner/mfob_trajectory_smoother.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/eval/qevent.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/trajectory_plot_util.h"

DEFINE_int32(mfob_trajectory_smoother_canvas_level, 0,
             "Mfob trajectory smoother canvas verbosity level");
DEFINE_int32(mfob_trajectory_smoother_verbosity_level, 0,
             "Mfob trajectory smoother verbosity level");

namespace qcraft {
namespace planner {
namespace {

using Mfob = MixedFourthOrderBicycle<kDdpTrajectorySteps>;
using StatesType = Mfob::StatesType;
using StateType = Mfob::StateType;
using ControlsType = Mfob::ControlsType;
using ControlType = Mfob::ControlType;

static constexpr int kHorizon = Mfob::kHorizon;

std::vector<TrajectoryPoint> GeneratePurePursuitInitTraj(
    const Mfob &problem, const std::vector<TrajectoryPoint> &init_traj,
    const VehicleGeometryParamsProto &veh_geo_params) {
  // Pure pursuit on the initial trajectory within control bounds.
  constexpr double kLongitudinalLookAhead = 0.5;  // s.
  const int kLongitudinalLookAheadSteps =
      RoundToInt(kLongitudinalLookAhead / kDdpTrajectoryTimeStep);

  StatesType init_xs = Mfob::FitState(init_traj);

  // Get longitudinal reference trajectory
  std::vector<StateType> longitudinal_extended_xs(kHorizon +
                                                  kLongitudinalLookAheadSteps);
  for (int k = 0; k < kHorizon; ++k) {
    longitudinal_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
  }
  StateType x = Mfob::GetStateAtStep(init_xs, kHorizon - 1);
  for (int k = 0; k < kLongitudinalLookAheadSteps; ++k) {
    x = problem.EvaluateF(k, x, ControlType::Zero());
    longitudinal_extended_xs[kHorizon + k] = x;
  }

  const auto get_longitudinal_target =
      [&longitudinal_extended_xs,
       kLongitudinalLookAheadSteps](int index) -> StateType {
    QCHECK_LT(index + kLongitudinalLookAheadSteps,
              longitudinal_extended_xs.size());
    return longitudinal_extended_xs[index + kLongitudinalLookAheadSteps];
  };

  // Get lateral reference trajectory
  constexpr double kLateralLookAhead = 0.5;  // s.
  std::vector<StateType> lateral_extended_xs(kHorizon);
  std::vector<double> lateral_extened_xs_s(kHorizon);
  for (int k = 0; k < kHorizon; ++k) {
    lateral_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
    lateral_extened_xs_s[k] = Mfob::s(init_xs, k);
  }

  const double init_traj_s = Mfob::StateGetS(lateral_extended_xs.back());
  const PiecewiseLinearFunction<StateType, double> lateral_reference_plf(
      lateral_extened_xs_s, lateral_extended_xs);
  const auto get_lateral_target =
      [&lateral_reference_plf, &lateral_extended_xs, &init_traj_s](
          const StateType &state, double lateral_look_ahead_dist) -> StateType {
    const double target_s = lateral_look_ahead_dist + Mfob::StateGetS(state);
    if (target_s >= init_traj_s) {
      StateType target = lateral_extended_xs.back();
      Vec2d target_theta_tangent =
          Vec2d::UnitFromAngle(Mfob::StateGetTheta(target));
      Mfob::StateSetPos(Mfob::StateGetPos(target) +
                            (target_s - init_traj_s) * target_theta_tangent,
                        &target);
      return target;
    } else {
      return lateral_reference_plf(target_s);
    }
  };

  ControlsType us = ControlsType::Zero();
  StatesType xs = StatesType::Zero();
  x = Mfob::GetStateAtStep(init_xs, 0);
  for (int k = 0; k < kHorizon; ++k) {
    Mfob::SetStateAtStep(x, k, &xs);
    StateType longitudinal_target = get_longitudinal_target(k);
    const double lateral_look_ahead_dist =
        kLateralLookAhead * Mfob::StateGetV(x) + veh_geo_params.wheel_base();
    StateType lateral_target = get_lateral_target(x, lateral_look_ahead_dist);
    ControlType u = problem.PurePursuitController(
        x, longitudinal_target, lateral_target, kLongitudinalLookAheadSteps,
        lateral_look_ahead_dist);
    x = problem.EvaluateF(k, x, u);
    Mfob::SetControlAtStep(u, k, &us);
  }

  std::vector<TrajectoryPoint> res(kHorizon);
  for (int k = 0; k < kHorizon; ++k) {
    TrajectoryPoint &point = res[k];
    problem.ExtractTrajectoryPoint(k, Mfob::GetStateAtStep(xs, k),
                                   Mfob::GetControlAtStep(us, k), &point);
  }
  return res;
}

}  // namespace

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    const std::vector<TrajectoryPoint> &ref_traj, const std::string &owner,
    const TrajectorySmootherCostWeightParamsProto &smoother_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params) {
  SCOPED_QTRACE("SmoothTrajectoryByMixedFourthOrderDdp");

  ControlsType ref_us = ControlsType::Zero();
  StatesType ref_xs = Mfob::FitState(ref_traj);

  Mfob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
               kDdpTrajectoryTimeStep, /*enable_post_process=*/false);

  std::vector<Vec2d> ref_path_points;
  ref_path_points.reserve(ref_traj.size() + 1);
  for (int i = 0; i < ref_traj.size(); ++i) {
    ref_path_points.push_back(ref_traj[i].pos());
  }
  // Add one more point at tail in case of short trajectory.
  constexpr double tail_length = 0.2;  // m.
  ref_path_points.push_back(
      ref_traj.back().pos() +
      tail_length * Vec2d::FastUnitFromAngle(ref_traj.back().theta()));

  std::vector<double> ref_path_deviation_gains(ref_path_points.size() - 1, 1.0);
  std::vector<double> state_deviation_weights(
      kHorizon * Mfob::kStateSize, smoother_params.state_deviation_gain());
  std::vector<double> control_penalty_weights(
      kHorizon * Mfob::kControlSize, smoother_params.jerk_penalty_gain());
  const double corrected_chi_penalty_gain =
      smoother_params.chi_penalty_gain() *
      std::max(
          smoother_params.chi_penalty_gain_min_factor(),
          smoother_params.chi_penalty_speed_gain() * Sqr(ref_traj.front().v()));
  for (int i = 0; i < kHorizon; ++i) {
    state_deviation_weights[i * Mfob::kStateSize + Mfob::kStateVIndex] =
        smoother_params.v_deviation_gain();
    control_penalty_weights[i * Mfob::kControlSize + Mfob::kControlChiIndex] =
        corrected_chi_penalty_gain;
  }

  // For x, y and theta end
  state_deviation_weights[(kHorizon - 1) * Mfob::kStateSize +
                          Mfob::kStateXIndex] =
      smoother_params.end_pose_deviation_gain();
  state_deviation_weights[(kHorizon - 1) * Mfob::kStateSize +
                          Mfob::kStateYIndex] =
      smoother_params.end_pose_deviation_gain();
  state_deviation_weights[(kHorizon - 1) * Mfob::kStateSize +
                          Mfob::kStateThetaIndex] =
      smoother_params.end_heading_deviation_gain();

  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      smoother_params.path_gain(), smoother_params.end_state_gain(),
      ref_path_points, /*center_line_helper=*/nullptr,
      std::move(ref_path_deviation_gains),
      "MfobTrajSmooth::ReferenceLineDeviationCost",
      smoother_params.ref_path_deviation_gain()));
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      std::move(ref_xs), std::move(state_deviation_weights),
      "MfobTrajSmooth::StateRegularization", smoother_params.scale()));
  problem.AddCost(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      std::move(ref_us), std::move(control_penalty_weights),
      "MfobTrajSmooth::ControlRegularization", smoother_params.scale()));
  constexpr double kCurvatureBufferRatio = 0.8;
  constexpr double kCurvatureRateBufferRatio = 0.8;
  problem.AddCost(std::make_unique<CurvatureCost<Mfob>>(
      GetCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      kHorizon, "MfobTrajSmooth::MfobCurvatureCost",
      smoother_params.curvature_penalty()));
  problem.AddCost(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
      "MfobTrajSmooth::MfobCurvatureRateCost",
      smoother_params.curvature_rate_penalty()));
  problem.AddCost(std::make_unique<ForwardSpeedCost<Mfob>>(
      "MfobTrajSmooth::MfobForwardSpeedCost",
      smoother_params.forward_speed_cost_weight()));
  constexpr double kAccelerationBufferRatio = 1.0;
  const std::vector<double> accel_cascade_buffers = {0.0};
  const std::vector<double> accel_cascade_gains = {smoother_params.scale()};
  const std::vector<double> decel_cascade_buffers = {0.0};
  const std::vector<double> decel_cascade_gains = {smoother_params.scale()};

  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      accel_cascade_buffers, accel_cascade_gains, decel_cascade_buffers,
      decel_cascade_gains, "MfobTrajSmooth::MfobLongitudinalAccelerationCost",
      smoother_params.longitudinal_acceleration_cost_weight()));

  DdpOptimizer<Mfob> smoother(
      &problem, /*owner=*/"mfob_smoother",
      /*verbosity=*/FLAGS_mfob_trajectory_smoother_verbosity_level);
  std::vector<TrajectoryPoint> init_vals =
      GeneratePurePursuitInitTraj(problem, ref_traj, veh_geo_params);

  smoother.SetInitialPoints(init_vals);

  const auto solve_start_time = absl::Now();
  const auto output = smoother.Solve(
      /*forward=*/true);
  const absl::Duration time_consuming = absl::Now() - solve_start_time;
  VLOG(2) << owner + " ddp smooth time consuming: "
          << absl::ToDoubleMilliseconds(time_consuming);

  if (!output.ok()) {
    QLOG(WARNING) << "trajectory smoother solve failed: "
                  << output.status().message();
    return ref_traj;
  }

  const std::vector<TrajectoryPoint> res = std::move(*output);

  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < ref_traj.size(); i++) {
      VLOG(4) << "ref_traj " << i << "  " << ref_traj[i].DebugString();
    }
    for (int i = 0; i < init_vals.size(); i++) {
      VLOG(4) << "pp_traj " << i << "  " << init_vals[i].DebugString();
    }
    for (int i = 0; i < res.size(); i++) {
      VLOG(4) << "res " << i << "  " << res[i].DebugString();
    }
  }

  if (FLAGS_mfob_trajectory_smoother_canvas_level >= 2) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&ref_path_points](int index) { return ref_path_points[index]; },
            ref_path_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.8, 0.4, 0.8),
        /*render_indices=*/true, owner + "/traj_smooth/input");
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&init_vals](int index) { return init_vals[index].pos(); },
            init_vals.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.7, 0.6, 0.2),
        /*render_indices=*/true, owner + "/traj_smooth/pp");
    CanvasDrawTrajectory(
        VisIndexTrajToVector([res](int index) { return res[index].pos(); },
                             res.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.5, 0.5, 0.7),
        /*render_indices=*/true, owner + "/traj_smooth/final");
  }
  return res;
}

}  // namespace planner
}  // namespace qcraft
