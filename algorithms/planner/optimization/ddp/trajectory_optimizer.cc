#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/auto_tuning_utils.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/mfob_trajectory_smoother.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_debug_hook.h"
#include "onboard/planner/optimization/ddp/object_cost_util.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/problem/aggregate_static_object_cost.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/curvature_deviation_cost.h"
#include "onboard/planner/optimization/problem/end_heading_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/lateral_acceleration_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_emeraude_object_cost.h"
#include "onboard/planner/optimization/problem/mfob_gradient_fixed_projective_object_cost.h"
#include "onboard/planner/optimization/problem/mfob_intrinsic_lateral_snap_cost.h"
#include "onboard/planner/optimization/problem/mfob_lateral_jerk_cost.h"
#include "onboard/planner/optimization/problem/mfob_path_boundary_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/msd_static_boundary_cost.h"
#include "onboard/planner/optimization/problem/partitioned_object_cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_longitudinal_jerk_deviation_cost.h"  // NOLINT
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"
#include "onboard/planner/optimization/problem/static_boundary_cost.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

DEFINE_bool(send_traj_optimizer_result_to_canvas, false,
            "Whether to send trajectory optimizer result to canvas.");
DEFINE_int32(traj_opt_canvas_level, 0, "Traj opt canvas level.");
DEFINE_int32(traj_opt_verbosity_level, 2, "Traj opt verbosity level.");
DEFINE_int32(static_boundary_canvas_level, 0,
             "Boundary points canvas verbosity level.");
DEFINE_double(auto_tuning_gamma, 0.99,
              "Only used in auto tuning mode, gamma is the discounted rate.");
DEFINE_bool(enable_ipopt_solver, false,
            "Whether to enable ipopt solver to be used for being compared with "
            "ddp optimizer.");
DEFINE_bool(traj_opt_draw_circle, false, "Whether to draw av model.");
DEFINE_bool(traj_opt_enable_inner_path_boundary, true,
            "Whether to enable inner (target) path boundary.");
DEFINE_double(traj_opt_inner_path_boundary_gain_factor, 1.0,
              "Multiplying factor to tune inner (target) path boundary gain.");

DEFINE_bool(mfob_path_boundary_use_qtfm, true,
            "mfob path boundary cost uses qtfm algorithm.");

DEFINE_bool(enable_msd_static_boundary_cost, true,
            "The static boundary cost that uses min segment distance problem "
            "to calculate collison cost.");

DEFINE_bool(msd_static_boundary_cost_use_qtfm_v2, true,
            "Use the quad tree field map accelerated msd problem in "
            "msd_static_boundary_cost.");

DEFINE_bool(trajectory_optimizer_frenet_frame_uses_qtfm_v2, true,
            "Use the quad tree field map accelerated frenet frame in "
            "trajectory optimizer.");

namespace qcraft {
namespace planner {
namespace {

using Mfob = optimizer::Mfob;
using MfobDAT = MixedFourthOrderBicycle<kDdpTrajectoryStepsDAT>;

constexpr int kCurvatureLimitIndex =
    static_cast<int>(kCurvatureLimitRange / kDdpTrajectoryTimeStep);

struct SpeedZoneInfo {
  double s_start;
  double s_end;
  Vec2d x_start;
  Vec2d x_end;
  double target_speed;
};

void ToDebugProto(const std::vector<TrajectoryPoint> &init_traj,
                  const std::vector<TrajectoryPoint> &smooth_init_traj,
                  const std::vector<TrajectoryPoint> &result_traj,
                  const OptimizerSolverDebugHook<Mfob> &solver_debug_hook,
                  TrajectoryOptimizerDebugProto *traj_opt_debug_proto,
                  ThreadPool *thread_pool) {
  SCOPED_QTRACE("TrajectoryOptimizer/ToDebugProto");
  // Write DdpDebugProto data.
  DdpOptimizerDebugProto *ddp_debug = traj_opt_debug_proto->mutable_ddp();
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ddp_debug->add_init_traj());
  }
  for (int k = 0; k < smooth_init_traj.size(); ++k) {
    smooth_init_traj[k].ToProto(ddp_debug->add_smooth_init_traj());
  }

  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ddp_debug->add_final_traj());
  }

  const auto &init_costs = solver_debug_hook.init_costs;
  ddp_debug->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        ddp_debug->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.ddp_costs[i].first);
    cost_proto->set_cost(init_costs.ddp_costs[i].second);
  }
  const auto &final_costs = solver_debug_hook.final_costs;
  ddp_debug->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        ddp_debug->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.ddp_costs[i].first);
    cost_proto->set_cost(final_costs.ddp_costs[i].second);
  }

  const int num_iters = solver_debug_hook.iterations.size();
  ddp_debug->mutable_iterations()->Reserve(num_iters);
  for (int i = 0; i < num_iters; ++i) {
    ddp_debug->add_iterations();
  }

  ParallelFor(0, num_iters, thread_pool, [&](int i) {
    const auto &iteration = solver_debug_hook.iterations[i];
    DdpOptimizerDebugProto::Iteration *iteration_proto =
        ddp_debug->mutable_iterations()->Mutable(i);
    for (int i = 0; i < iteration.init_xs.size(); ++i) {
      iteration_proto->add_init_xs(iteration.init_xs[i]);
    }
    for (int i = 0; i < iteration.init_us.size(); ++i) {
      iteration_proto->add_init_us(iteration.init_us[i]);
    }
    QCHECK_EQ(iteration.alphas.size(), iteration.line_search_costs.size());
    for (int i = 0; i < iteration.alphas.size(); ++i) {
      iteration_proto->add_line_search_alphas(iteration.alphas[i]);
      iteration_proto->add_line_search_costs(iteration.line_search_costs[i]);
    }
    QCHECK_EQ(iteration.k_s.size(), iteration.stepsize_adjustment_costs.size());
    for (int i = 0; i < iteration.k_s.size(); ++i) {
      iteration_proto->add_step_size_adjustment_ks(iteration.k_s[i]);
      iteration_proto->add_step_size_adjustment_costs(
          iteration.stepsize_adjustment_costs[i]);
    }
    iteration_proto->set_final_cost(iteration.final_cost);
    iteration_proto->set_js0(iteration.js0);
    for (int i = 0; i < iteration.ddp_costs.size(); ++i) {
      TrajectoryOptimizerCost *cost_proto = iteration_proto->add_costs();
      cost_proto->set_name(iteration.ddp_costs[i].first);
      cost_proto->set_cost(iteration.ddp_costs[i].second);
    }
  });

  ddp_debug->set_num_iters(num_iters);

  for (const auto &response : solver_debug_hook.object_responses) {
    *traj_opt_debug_proto->add_object_responses() = response;
  }
}

// Copied from empty_road_decider.
std::vector<SpeedZoneInfo> CreateExclusiveSpeedZones(
    const std::vector<SpeedZoneInfo> &speed_zones,
    std::vector<int> *exclusive_speed_zone_original_indices) {
  struct SpeedZoneEndpointInfo {
    int zone_index;
    bool start;
    double s;
    double speed;
  };

  std::vector<SpeedZoneEndpointInfo> endpoints;
  endpoints.reserve(speed_zones.size() * 2);
  for (int i = 0; i < speed_zones.size(); ++i) {
    const auto &speed_zone = speed_zones[i];
    endpoints.push_back({i, true, speed_zone.s_start, speed_zone.target_speed});
    endpoints.push_back({i, false, speed_zone.s_end, speed_zone.target_speed});
  }
  std::sort(endpoints.begin(), endpoints.end(),
            [](const SpeedZoneEndpointInfo &p0,
               const SpeedZoneEndpointInfo &p1) { return p0.s < p1.s; });

  // map: from speed to <zone index, start s>.
  std::multimap<double, std::pair<int, double>> active_zones;
  std::vector<SpeedZoneInfo> exclusive_speed_zones;
  if (exclusive_speed_zone_original_indices != nullptr) {
    exclusive_speed_zone_original_indices->clear();
  }
  for (int i = 0; i < endpoints.size(); ++i) {
    const SpeedZoneEndpointInfo &endpoint = endpoints[i];
    VLOG(3) << "endpoint " << i << ": " << endpoint.zone_index << " "
            << endpoint.start << " " << endpoint.s << " " << endpoint.speed;
    if (endpoint.start) {
      if (!active_zones.empty() &&
          endpoint.speed < active_zones.begin()->first) {
        exclusive_speed_zones.push_back(
            {.s_start = active_zones.begin()->second.second,
             .s_end = endpoint.s,
             .target_speed = active_zones.begin()->first});
        if (exclusive_speed_zone_original_indices != nullptr) {
          exclusive_speed_zone_original_indices->push_back(
              active_zones.begin()->second.first);
        }
      }
      active_zones.insert({endpoint.speed, {endpoint.zone_index, endpoint.s}});
    } else {
      QCHECK(!active_zones.empty());
      auto start_it = active_zones.end();
      const auto range = active_zones.equal_range(endpoint.speed);
      for (auto it = range.first; it != range.second; ++it) {
        if (endpoint.zone_index == it->second.first) {
          start_it = it;
          break;
        }
      }
      QCHECK(start_it != active_zones.end());
      QCHECK_EQ(endpoint.zone_index, start_it->second.first);

      if (start_it == active_zones.begin()) {  // Active.
        exclusive_speed_zones.push_back({.s_start = start_it->second.second,
                                         .s_end = endpoint.s,
                                         .target_speed = endpoint.speed});
        if (exclusive_speed_zone_original_indices != nullptr) {
          exclusive_speed_zone_original_indices->push_back(endpoint.zone_index);
        }
        const auto next = std::next(start_it);
        if (next != active_zones.end()) {
          next->second.second = endpoint.s;
        }
      }
      active_zones.erase(start_it);
    }
  }

  for (int i = 0; i + 1 < exclusive_speed_zones.size(); ++i) {
    QCHECK_LE(exclusive_speed_zones[i].s_end,
              exclusive_speed_zones[i + 1].s_start);
  }
  if (exclusive_speed_zone_original_indices != nullptr) {
    QCHECK_EQ(exclusive_speed_zones.size(),
              exclusive_speed_zone_original_indices->size());
  }
  return exclusive_speed_zones;
}

absl::Status CheckInputQuality(const TrajectoryOptimizerInput &input) {
  constexpr int kMinTrajectoryLength = 10;  // 1s.
  if (input.trajectory.size() < kMinTrajectoryLength) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Input trajectory is not long enough: %d time steps.",
                        input.trajectory.size()));
  }
  return absl::OkStatus();
}

void AddRegularizersCost(
    const std::vector<TrajectoryPoint> &init_traj,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  const Mfob::StateType x0 = Mfob::FitInitialState(init_traj);
  const Mfob::ControlsType init_us = Mfob::FitControl(init_traj, x0);
  const Mfob::StatesType init_xs = Mfob::FitState(init_traj);

  std::vector<double> state_regularization_weights(init_xs.size(), 0.0);
  std::vector<double> control_regularization_weights(init_us.size(), 0.0);
  for (int i = 0; i < init_traj.size(); ++i) {
    for (int j = 0; j < Mfob::kStateSize; ++j) {
      state_regularization_weights[i * Mfob::kStateSize + j] =
          cost_weight_params.state_regularization_coeffs().w(j);
    }
    for (int j = 0; j < Mfob::kControlSize; ++j) {
      control_regularization_weights[i * Mfob::kControlSize + j] =
          cost_weight_params.control_regularization_coeffs().w(j);
    }
  }
  costs->emplace_back(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      init_xs, std::move(state_regularization_weights),
      "MfobReferenceStateDeviationCost: StateRegularization",
      cost_weight_params.state_regularization_coeffs().multiplier()));
  costs->emplace_back(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      init_us, std::move(control_regularization_weights),
      "MfobReferenceControlDeviationCost: ControlRegularization",
      cost_weight_params.control_regularization_coeffs().multiplier()));
}

void AddAccelAndJerkCost(
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  std::vector<double> accel_cascade_buffers;
  std::vector<double> accel_cascade_gains;
  std::vector<double> decel_cascade_buffers;
  std::vector<double> decel_cascade_gains;
  for (const auto &cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .accel_cascade()) {
    accel_cascade_buffers.push_back(cascade.buffer());
    accel_cascade_gains.push_back(cascade.gain());
  }
  for (const auto &cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .decel_cascade()) {
    decel_cascade_buffers.push_back(cascade.buffer());
    decel_cascade_gains.push_back(cascade.gain());
  }
  constexpr double kAccelerationBufferRatio = 0.75;
  constexpr double kJerkBufferRatio = 0.75;
  costs->emplace_back(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      std::move(accel_cascade_buffers), std::move(accel_cascade_gains),
      std::move(decel_cascade_buffers), std::move(decel_cascade_gains),
      "MfobLongitudinalAccelerationCost",
      cost_weight_params.longitudinal_acceleration_cost_weight()));
  costs->emplace_back(std::make_unique<LateralAccelerationCost<Mfob>>(
      "MfobLateralAccelerationCost",
      cost_weight_params.lateral_acceleration_cost_weight()));
  // TODO(renjie, runbing): Try to use MfobLongitudinalJerkCost.
  costs->emplace_back(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "MfobIntrinsicJerkCost",
      cost_weight_params.intrinsic_jerk_cost_weight()));
  costs->emplace_back(std::make_unique<MfobLateralJerkCost<Mfob>>(
      "MfobLateralJerkCost", cost_weight_params.lateral_jerk_cost_weight()));
  costs->emplace_back(std::make_unique<MfobIntrinsicLateralSnapCost<Mfob>>(
      "MfobIntrinsicLateralSnapCost",
      cost_weight_params.intrinsic_lateral_snap_weight()));
}

void AddCurvatureCost(
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  constexpr double kCurvatureBufferRatio = 0.90;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  costs->emplace_back(std::make_unique<CurvatureCost<Mfob>>(
      GetCenterMaxCurvature(veh_geo_params, vehicle_drive_params) *
          kCurvatureBufferRatio,
      kCurvatureLimitIndex, "MfobCurvatureCost",
      cost_weight_params.curvature_cost_weight()));
  costs->emplace_back(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
      "MfobCurvatureRateCost",
      cost_weight_params.curvature_rate_cost_weight()));
  costs->emplace_back(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
      motion_constraint_params.max_chi(), "MfobCurvatureRateRateCost",
      cost_weight_params.curvature_rate_rate_cost_weight()));
}

void AddImmediateFutureCost(
    const std::vector<TrajectoryPoint> &prev_traj, double v_now,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  if (prev_traj.size() < 2) return;
  const Mfob::StateType x0 = Mfob::FitInitialState(prev_traj);
  const Mfob::ControlsType ref_us = Mfob::FitControl(prev_traj, x0);
  const Mfob::StatesType ref_xs = Mfob::FitState(prev_traj);

  std::array<double, Mfob::kHorizon> ref_jerk;
  std::vector<double> ref_s(Mfob::kHorizon);
  std::vector<double> ref_kappa(Mfob::kHorizon);
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    ref_jerk[i] = Mfob::j(ref_us, i);
    ref_s[i] = Mfob::s(ref_xs, i);
    ref_kappa[i] = Mfob::kappa(ref_xs, i);
  }
  constexpr double kImmediateJGain = 0.1;
  const PiecewiseLinearFunction<double> lon_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params().lon_weight_plf());
  std::array<double, Mfob::kHorizon> lon_weights;
  QCHECK_EQ(prev_traj.size(), Mfob::kHorizon);
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    const auto &prev_traj_point = prev_traj[i];
    const double lon_weight_plf_t = lon_weight_plf(prev_traj_point.t());
    lon_weights[i] = kImmediateJGain * lon_weight_plf_t;
  }
  costs->push_back(
      std::make_unique<ReferenceLongitudinalJerkDeviationCost<Mfob>>(
          std::move(ref_jerk), std::move(lon_weights),
          "MfobReferenceLongitudinalJerkDeviationCost: ImmediateFuture",
          cost_weight_params.immediate_future_cost_weight()));

  constexpr double kCurvatureDeviationCostWeight = 500.0;
  const double weight_decay_rate =
      -1.0 / std::clamp(cost_weight_params.immediate_future_cost_params()
                                .lat_weight_decay_time_distance() *
                            v_now,
                        10.0, 75.0);
  costs->push_back(std::make_unique<CurvatureDeviationCost<Mfob>>(
      ref_s, ref_kappa, kCurvatureDeviationCostWeight, weight_decay_rate,
      "MfobCurvatureDeviationCost: ImmediateFuture",
      cost_weight_params.curvature_deviation_immediate_future_cost_weight()));
}

void MergeSpeedLimitWithFirstStopLine(double first_stop_line_s,
                                      const DrivePassage &drive_passage,
                                      std::vector<double> *speed_limit_s,
                                      std::vector<Vec2d> *speed_limit_x,
                                      std::vector<double> *speed_limit_v) {
  const auto status = drive_passage.QueryPointXYAtS(first_stop_line_s);
  QCHECK_OK(status.status());
  const Vec2d first_stop_point = *status;
  constexpr double kEps = 1e-10;
  constexpr double kMergeStopPointS = 0.1;
  if (std::abs(speed_limit_s->back() - first_stop_line_s) < kEps) {
    speed_limit_v->back() = 0.0;
    const Vec2d end_tangent =
        (speed_limit_x->back() - *(speed_limit_x->rbegin() + 1)).normalized();
    constexpr double kEndExtension = 100.0;  // m.
    speed_limit_s->push_back(speed_limit_s->back() + kEndExtension);
    speed_limit_x->push_back(speed_limit_x->back() +
                             end_tangent * kEndExtension);
    speed_limit_v->push_back(0.0);
  } else if (first_stop_line_s - speed_limit_s->front() < kMergeStopPointS) {
    // First stop point very close or even ahead of first speed limit point
    // (could happen when AV rac point passed first stop point), merge them
    std::fill(speed_limit_v->begin(), speed_limit_v->end(), 0.0);
  } else {
    const int first_index_after_stop_point =
        std::upper_bound(speed_limit_s->begin(), speed_limit_s->end(),
                         first_stop_line_s) -
        speed_limit_s->begin();
    if ((first_stop_line_s -
         speed_limit_s->at(std::max(first_index_after_stop_point - 1, 0))) <
        kMergeStopPointS) {
      speed_limit_s->at(std::max(first_index_after_stop_point - 1, 0)) =
          first_stop_line_s;
      speed_limit_x->at(std::max(first_index_after_stop_point - 1, 0)) =
          first_stop_point;
      speed_limit_v->at(std::max(first_index_after_stop_point - 1, 0)) = 0.0;
    } else {
      speed_limit_s->insert(
          speed_limit_s->begin() + first_index_after_stop_point,
          first_stop_line_s);
      speed_limit_x->insert(
          speed_limit_x->begin() + first_index_after_stop_point,
          first_stop_point);
      speed_limit_v->insert(
          speed_limit_v->begin() + first_index_after_stop_point, 0.0);
    }
    for (int i = first_index_after_stop_point; i < speed_limit_s->size(); ++i) {
      (*speed_limit_v)[i] = 0.0;
    }
  }
}

void CreateFreeDrivingSpeedLimitInfo(
    const DrivePassage &drive_passage,
    const TurnSpeedLimitingParamsProto &turn_speed_limit_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    std::vector<double> *speed_limit_s, std::vector<Vec2d> *speed_limit_x,
    std::vector<double> *speed_limit_v) {
  QCHECK_NOTNULL(speed_limit_s);
  QCHECK_NOTNULL(speed_limit_x);
  QCHECK_NOTNULL(speed_limit_v);
  QCHECK(speed_limit_s->empty());
  QCHECK(speed_limit_x->empty());
  QCHECK(speed_limit_v->empty());

  // TODO(renjie): See if needs resampling other than use passage stations
  // directly.
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;      // From drive passage start.
  std::vector<double> station_speed_limits;  // m/s.
  std::vector<double> station_thetas;
  for (int i = 0, n = drive_passage.stations().size(); i < n; ++i) {
    const auto &station = drive_passage.station(StationIndex(i));
    station_points.push_back(station.xy());
    station_points_s.push_back(station.accumulated_s());
    station_thetas.push_back(station.tangent().Angle());
    station_speed_limits.push_back(station.speed_limit());
  }

  // Update lane speed limits considering previewed average passage curvature.
  constexpr double preview_distance = 10.0;
  const double gain = turn_speed_limit_params.gain();
  const double bias = turn_speed_limit_params.bias();
  constexpr double kEps = 1e-6;
  for (int i = 0; i < station_points.size(); ++i) {
    const double s_now = station_points_s[i];
    const double theta_now = station_thetas[i];
    const double s_preview = s_now + preview_distance;
    const auto &station_preview =
        drive_passage.FindNearestStationAtS(s_preview);
    const double theta_preview = station_preview.tangent().Angle();
    const double kappa_fd =
        NormalizeAngle(theta_preview - theta_now) / preview_distance;
    const double v_kappa_fd = bias + 1.0 / (std::abs(kappa_fd) + kEps) * gain;
    station_speed_limits[i] = std::min(station_speed_limits[i], v_kappa_fd);
  }

  // speed_limit_v.size() should be speed_limit_x.size() - 1.
  int station_i = 0;
  while (station_i < station_points_s.size()) {
    speed_limit_s->push_back(station_points_s[station_i]);
    speed_limit_x->push_back(station_points[station_i]);
    speed_limit_v->push_back(station_speed_limits[station_i]);
    ++station_i;
  }

  if (drive_passage.beyond_lane_path()) {
    // Place a virtual stop line at drive passage end.
    constexpr double kPassageEps = 0.1;  // m.
    const double max_passage_s = drive_passage.end_s() - kPassageEps;
    const double first_stop_line_s =
        std::max(max_passage_s - veh_geo_params.front_edge_to_center(),
                 drive_passage.front_s() + kPassageEps);
    MergeSpeedLimitWithFirstStopLine(first_stop_line_s, drive_passage,
                                     speed_limit_s, speed_limit_x,
                                     speed_limit_v);
  }
  speed_limit_v->pop_back();
}

void CreateSpeedLimitInfo(
    const DrivePassage &drive_passage,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines,
    absl::Span<const ConstraintProto::SpeedRegionProto> speed_regions,
    const TurnSpeedLimitingParamsProto &turn_speed_limit_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    std::vector<double> *speed_limit_s, std::vector<Vec2d> *speed_limit_x,
    std::vector<double> *speed_limit_v) {
  QCHECK_NOTNULL(speed_limit_s);
  QCHECK_NOTNULL(speed_limit_x);
  QCHECK_NOTNULL(speed_limit_v);
  QCHECK(speed_limit_s->empty());
  QCHECK(speed_limit_x->empty());
  QCHECK(speed_limit_v->empty());

  // TODO(renjie): See if needs resampling other than use passage stations
  // directly.
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;      // From drive passage start.
  std::vector<double> station_speed_limits;  // m/s.
  std::vector<double> station_thetas;
  for (const auto &station : drive_passage.stations()) {
    station_points.push_back(station.xy());
    station_points_s.push_back(station.accumulated_s());
    station_speed_limits.push_back(station.speed_limit());
    station_thetas.push_back(station.tangent().FastAngle());
  }
  // Update lane speed limits considering previewed average passage curvature.
  constexpr double preview_distance = 10.0;
  const double gain = turn_speed_limit_params.gain();
  const double bias = turn_speed_limit_params.bias();
  constexpr double kEps = 1e-6;
  for (int i = 0; i < station_points.size(); ++i) {
    const double s_now = station_points_s[i];
    const double theta_now = station_thetas[i];
    const double s_preview = s_now + preview_distance;
    const auto &station_preview =
        drive_passage.FindNearestStationAtS(s_preview);
    const double theta_preview = station_preview.tangent().FastAngle();
    const double kappa_fd =
        NormalizeAngle(theta_preview - theta_now) / preview_distance;
    const double v_kappa_fd = bias + 1.0 / (std::abs(kappa_fd) + kEps) * gain;
    station_speed_limits[i] = std::min(station_speed_limits[i], v_kappa_fd);
  }

  // Get passage s range.
  constexpr double kPassageEps = 0.1;  // m.
  const double min_passage_s = drive_passage.front_s() + kPassageEps;
  const double max_passage_s = drive_passage.end_s() - kPassageEps;

  // Speed zones from constraints. Note that speed zone should not exceed the
  // range of drive passage.
  std::vector<SpeedZoneInfo> speed_zones;
  for (const auto &speed_region : speed_regions) {
    // Only consider speed bumps now.
    if (speed_region.source().type_case() ==
        SourceProto::TypeCase::kSpeedBump) {
      auto &speed_zone = speed_zones.emplace_back();
      // Make sure that speed_zone.s_end is larger than speed_zone.s_start.
      speed_zone.s_start = std::clamp(
          speed_region.start_s() - veh_geo_params.front_edge_to_center(),
          min_passage_s, max_passage_s - kPassageEps);
      speed_zone.s_end = std::clamp(
          speed_region.end_s() + veh_geo_params.back_edge_to_center(),
          min_passage_s + kPassageEps, max_passage_s);
      speed_zone.target_speed = speed_region.max_speed();
    }
  }

  std::vector<int> speed_zone_indices;
  speed_zones = CreateExclusiveSpeedZones(speed_zones, &speed_zone_indices);
  for (int i = 0; i + 1 < speed_zones.size(); ++i) {
    QCHECK_LE(speed_zones[i].s_end, speed_zones[i + 1].s_start);
  }
  for (auto &speed_zone : speed_zones) {
    speed_zone.x_start =
        drive_passage.QueryPointXYAtS(speed_zone.s_start).value();
    speed_zone.x_end = drive_passage.QueryPointXYAtS(speed_zone.s_end).value();
  }

  // Merge speed zones with lane speed limits.
  int station_i = 0;
  int speed_zone_i = 0;
  while (station_i < station_points_s.size() ||
         speed_zone_i < speed_zones.size()) {
    if (station_i == station_points_s.size() ||
        (speed_zone_i < speed_zones.size() &&
         station_points_s[station_i] > speed_zones[speed_zone_i].s_start)) {
      if (speed_zones[speed_zone_i].s_start < 0.0 &&
          speed_zones[speed_zone_i].s_end < 0.0) {
        ++speed_zone_i;
        continue;
      }
      if (!speed_limit_s->empty() &&
          speed_limit_s->back() == speed_zones[speed_zone_i].s_start) {
        if (speed_limit_v->back() > speed_zones[speed_zone_i].target_speed) {
          speed_limit_v->back() = speed_zones[speed_zone_i].target_speed;
        }
      } else {
        speed_limit_s->push_back(speed_zones[speed_zone_i].s_start);
        speed_limit_x->push_back(speed_zones[speed_zone_i].x_start);
        speed_limit_v->push_back(speed_zones[speed_zone_i].target_speed);
      }
      while (station_i < station_points_s.size() &&
             station_points_s[station_i] <= speed_zones[speed_zone_i].s_end) {
        if (station_speed_limits[station_i] <
            speed_zones[speed_zone_i].target_speed) {
          speed_limit_s->push_back(station_points_s[station_i]);
          speed_limit_x->push_back(station_points[station_i]);
          speed_limit_v->push_back(station_speed_limits[station_i]);
        }
        ++station_i;
      }
      if (station_i < station_points_s.size()) {
        if (speed_limit_s->back() != speed_zones[speed_zone_i].s_end) {
          speed_limit_s->push_back(speed_zones[speed_zone_i].s_end);
          speed_limit_x->push_back(speed_zones[speed_zone_i].x_end);
          speed_limit_v->push_back(station_speed_limits[station_i]);
        }
      } else {
        speed_limit_s->push_back(station_points_s.back());
        speed_limit_x->push_back(station_points.back());
        speed_limit_v->push_back(
            std::min(station_speed_limits.back(),
                     speed_zones[speed_zone_i].target_speed));
      }
      ++speed_zone_i;
    } else {
      if (!speed_limit_s->empty() &&
          speed_limit_s->back() == station_points_s[station_i]) {
        speed_limit_v->back() =
            std::min(speed_limit_v->back(), station_speed_limits[station_i]);
      } else {
        speed_limit_s->push_back(station_points_s[station_i]);
        speed_limit_x->push_back(station_points[station_i]);
        speed_limit_v->push_back(station_speed_limits[station_i]);
      }
      ++station_i;
    }
  }

  // TODO(runbing): Create virtual stop lines for static objects.

  // Consider stand-offs and re-sort the stop line s. Note that stop line should
  // not exceed the range of drive passage.
  std::vector<double> stop_line_s;
  stop_line_s.reserve(stop_lines.size());
  for (const auto &stop_line : stop_lines) {
    stop_line_s.push_back(std::clamp(stop_line.s() - stop_line.standoff() -
                                         veh_geo_params.front_edge_to_center(),
                                     min_passage_s, max_passage_s));
  }
  std::sort(stop_line_s.begin(), stop_line_s.end());
  if (!stop_lines.empty()) {
    const double first_stop_line_s = stop_line_s.front();
    MergeSpeedLimitWithFirstStopLine(first_stop_line_s, drive_passage,
                                     speed_limit_s, speed_limit_x,
                                     speed_limit_v);
  }

  // speed_limit_v.size() should be speed_limit_x.size() - 1.
  speed_limit_v->pop_back();
}

void AddSpeedLimitCost(
    const DrivePassage &drive_passage,
    const ConstraintManager &constraint_manager,
    const TurnSpeedLimitingParamsProto &turn_speed_limit_params,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs,
    TrajectoryOptimizerDebugProto *traj_opt_debug_proto) {
  const auto &stop_lines = constraint_manager.StopLine();
  const auto &speed_regions = constraint_manager.SpeedRegion();
  std::vector<double> speed_limit_s;
  std::vector<Vec2d> speed_limit_x;
  std::vector<double> speed_limit_v;
  CreateSpeedLimitInfo(drive_passage, stop_lines, speed_regions,
                       turn_speed_limit_params, veh_geo_params, &speed_limit_s,
                       &speed_limit_x, &speed_limit_v);
  // For trajectory point beyond normal planning horizon, we use a
  // separate speed limit which ignores all stop lines and speed regions except
  // one stop line at drive passage end to encourage progress.
  std::vector<double> free_speed_limit_s;
  std::vector<Vec2d> free_speed_limit_x;
  std::vector<double> free_speed_limit_v;
  CreateFreeDrivingSpeedLimitInfo(drive_passage, turn_speed_limit_params,
                                  veh_geo_params, &free_speed_limit_s,
                                  &free_speed_limit_x, &free_speed_limit_v);
  const auto under_speed_gain_compensation_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.under_speed_gain_compensation_plf());
  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < speed_limit_x.size(); ++i) {
      if (i < speed_limit_x.size() - 1) {
        VLOG(4) << "Speed limit point " << i << " s = " << speed_limit_s[i]
                << ": x = " << speed_limit_x[i].transpose()
                << " v = " << speed_limit_v[i];
      } else {
        VLOG(4) << "Speed limit point " << i << " s = " << speed_limit_s[i]
                << ": x = " << speed_limit_x[i].transpose();
      }
    }
    for (int i = 0; i < free_speed_limit_x.size(); ++i) {
      if (i < free_speed_limit_x.size() - 1) {
        VLOG(4) << "Free speed limit point " << i
                << " s = " << free_speed_limit_s[i]
                << ": x = " << free_speed_limit_x[i].transpose()
                << " v = " << free_speed_limit_v[i];
      } else {
        VLOG(4) << "Free speed limit point " << i
                << " s = " << free_speed_limit_s[i]
                << ": x = " << free_speed_limit_x[i].transpose();
      }
    }
  }

  constexpr double kStopSpeedCostGain = 800.0;
  constexpr double kOverSpeedCostGain = 100.0;
  constexpr double kUnderSpeedCostGain = 1.0;

  costs->emplace_back(std::make_unique<SegmentedSpeedLimitCost<Mfob>>(
      speed_limit_x, speed_limit_v, free_speed_limit_x, free_speed_limit_v,
      optimizer::kFreeIndex, under_speed_gain_compensation_plf,
      "MfobSpeedLimitCost", cost_weight_params.speed_limit_cost_weight(),
      /*stop_speed_gain=*/kStopSpeedCostGain,
      /*over_speed_gain=*/kOverSpeedCostGain,
      /*under_speed_gain=*/kUnderSpeedCostGain,
      /*use_qtfm=*/FLAGS_trajectory_optimizer_frenet_frame_uses_qtfm_v2));
}

void CollectCurbSegmentsAroundDrivePassage(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    std::vector<std::pair<std::string, Segment2d>> *named_segments,
    std::vector<int> *start_segment_ids) {
  QCHECK_NOTNULL(named_segments);
  QCHECK_NOTNULL(start_segment_ids);
  const mapping::SemanticMapManager *smm = psmm.semantic_map_manager();
  QCHECK_NOTNULL(smm);

  // Collect curb boundaries around drive passage without repeated ones.
  absl::flat_hash_set<mapping::ElementId> curb_boundaries;
  for (int i = 1; i < drive_passage.size(); ++i) {
    const Vec2d p0 = drive_passage.station(StationIndex(i - 1)).xy();
    const Vec2d p1 = drive_passage.station(StationIndex(i)).xy();
    const double search_radius = kMaxLateralOffset + (p1 - p0).Length() * 0.5;
    const Vec2d search_center = 0.5 * (p0 + p1);

    const std::vector<const mapping::LaneBoundaryInfo *> candidate_boundaries =
        smm->GetLaneBoundariesInfoAtLevel(smm->GetLevel(), search_center,
                                          search_radius);
    for (const auto &candidate_boundary : candidate_boundaries) {
      if (candidate_boundary->type != mapping::LaneBoundaryProto::CURB) {
        continue;
      }
      curb_boundaries.insert(candidate_boundary->id);
    }
  }

  // Emplace to outputs.
  named_segments->clear();
  start_segment_ids->clear();
  for (const auto &id : curb_boundaries) {
    const std::vector<Vec2d> &points =
        smm->FindLaneBoundaryByIdOrDie(id).points_smooth;
    if (points.empty()) {
      continue;
    }

    bool is_first_segment = true;
    Vec2d prev = points.front();
    for (int i = 1; i < points.size(); ++i) {
      // Skip short segments.
      if (prev.DistanceTo(points[i]) <=
          QtfmSegmentMatcherV2::kMinSegmentLength) {
        continue;
      }

      // Append a segment.
      named_segments->emplace_back(absl::StrCat("id:", id, ",seg:", i),
                                   Segment2d{prev, points[i]});
      prev = points[i];

      if (is_first_segment) {
        start_segment_ids->push_back(named_segments->size() - 1);
        is_first_segment = false;
      }
    }
  }
}

void CanvasDrawCurbSegments(
    std::string_view base_name,
    const std::vector<std::pair<std::string, Segment2d>> &named_curb_segments) {
  vis::Canvas *canvas_curbs = nullptr;
  canvas_curbs = &vantage_client_man::GetCanvas(
      absl::StrFormat("%s/curb_segments_for_static_boundary_cost", base_name));
  canvas_curbs->SetGroundZero(1);
  if (canvas_curbs == nullptr) {
    return;
  }
  constexpr double kCurbLineZ = 0.2;
  constexpr int kCurbLineWidth = 3;
  const vis::Color curb_color(1.0, 0.7, 0.7);
  for (const auto &named_segment : named_curb_segments) {
    const Segment2d &seg = named_segment.second;
    canvas_curbs->DrawLine({seg.start().x(), seg.start().y(), kCurbLineZ},
                           {seg.end().x(), seg.end().y(), kCurbLineZ},
                           curb_color, kCurbLineWidth);
  }
}

void AddStaticBoundaryCosts(
    std::string_view base_name, const TrajectoryPoint &plan_start_point,
    const DrivePassage &drive_passage, const PlannerSemanticMapManager &psmm,
    const PathSlBoundary &path_sl_boundary,
    std::vector<double> inner_path_boundary_gains,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>
        &center_line_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  SCOPED_QTRACE("AddStaticBoundaryCosts");
  QCHECK_GT(drive_passage.stations().size(), 0);

  std::vector<double> mid_edges_to_center = {};
  for (const double mid_edge_to_center :
       cost_weight_params.vehicle_model_params().mid_edge_to_center()) {
    mid_edges_to_center.push_back(mid_edge_to_center);
  }

  // Get curb and path boundary info.
  const auto drive_passage_size = drive_passage.stations().size();
  std::vector<qcraft::Vec2d> station_points;
  std::vector<double> left_dists_to_curb;
  std::vector<double> right_dists_to_curb;
  left_dists_to_curb.reserve(drive_passage_size);
  station_points.reserve(drive_passage_size);
  right_dists_to_curb.reserve(drive_passage_size);
  for (const auto &station : drive_passage.stations()) {
    const auto curb_pair_or = station.QueryCurbOffsetAt(/*signed_lat=*/0.0);
    QCHECK_OK(curb_pair_or.status());
    station_points.push_back(station.xy());
    left_dists_to_curb.push_back(curb_pair_or->second);
    right_dists_to_curb.push_back(-curb_pair_or->first);
  }
  const auto path_boundary_size = path_sl_boundary.size();
  std::vector<double> left_dists_to_path_boundary;
  std::vector<double> right_dists_to_path_boundary;
  std::vector<double> left_dists_to_target_path_boundary;
  std::vector<double> right_dists_to_target_path_boundary;
  std::vector<double> reference_center_l_offsets;
  left_dists_to_path_boundary.reserve(path_boundary_size);
  right_dists_to_path_boundary.reserve(path_boundary_size);
  left_dists_to_target_path_boundary.reserve(path_boundary_size);
  right_dists_to_target_path_boundary.reserve(path_boundary_size);
  reference_center_l_offsets.reserve(path_boundary_size);
  for (int i = 0; i < path_boundary_size; ++i) {
    left_dists_to_path_boundary.push_back(path_sl_boundary.left_l_vector()[i]);
    right_dists_to_path_boundary.push_back(
        -path_sl_boundary.right_l_vector()[i]);
    left_dists_to_target_path_boundary.push_back(
        path_sl_boundary.target_left_l_vector()[i]);
    right_dists_to_target_path_boundary.push_back(
        -path_sl_boundary.target_right_l_vector()[i]);
    reference_center_l_offsets.push_back(
        path_sl_boundary.reference_center_l_vector()[i]);
  }

  // Curb boundary cost.
  // TODO(renjie): we may want a new type of static boundary cost to slow down
  // AV when it gets close to the boundary
  const auto av_sl_pos_or =
      drive_passage.QueryFrenetCoordinateAt(plan_start_point.pos());
  if (!av_sl_pos_or.ok()) return;
  const auto av_curb_pair_or =
      drive_passage.QueryCurbOffsetAtS(av_sl_pos_or->s);
  if (!av_curb_pair_or.ok()) return;

  const double vehicle_half_width = 0.5 * veh_geo_params.width();
  const auto get_soft_curb_clearance = [](double av_speed) {
    constexpr double kSoftCurbClearanceMax = 1.2;
    constexpr double kSoftCurbClearanceMin = 0.5;
    constexpr double kSoftCurbClearanceBase = 0.4;
    constexpr double kSoftCurbClearanceGain = 0.03;
    return std::clamp(
        kSoftCurbClearanceBase + av_speed * kSoftCurbClearanceGain,
        kSoftCurbClearanceMin, kSoftCurbClearanceMax);
  };

  const auto get_hard_curb_clearance =
      [&vehicle_half_width, &av_sl_pos_or](double dist_to_curb_at_av_s) {
        constexpr double kHardCurbClearanceShift = 0.05;
        constexpr double kHardCurbClearanceMax = 0.5;
        constexpr double kHardCurbClearanceMin = 0.15;
        return std::clamp(std::abs(dist_to_curb_at_av_s - av_sl_pos_or->l) -
                              vehicle_half_width + kHardCurbClearanceShift,
                          kHardCurbClearanceMin, kHardCurbClearanceMax);
      };

  const std::vector<double> cascade_gains = {
      cost_weight_params.static_boundary_soft_cost_weight(),
      cost_weight_params.static_boundary_hard_cost_weight()};

  if (FLAGS_enable_msd_static_boundary_cost) {
    const double soft_curb_clearance =
        get_soft_curb_clearance(plan_start_point.v());

    const double hard_curb_clearance = get_hard_curb_clearance(
        std::min(av_curb_pair_or->second, av_curb_pair_or->first));

    std::vector<int> start_segment_ids;
    std::vector<std::pair<std::string, Segment2d>> curb_segments;
    CollectCurbSegmentsAroundDrivePassage(psmm, drive_passage, &curb_segments,
                                          &start_segment_ids);

    if (!curb_segments.empty()) {
      if (FLAGS_static_boundary_canvas_level >= 1) {
        CanvasDrawCurbSegments(base_name, curb_segments);
      }
      constexpr double kCutoffDistExtraBuffer = 0.1;
      const double cutoff_distance =
          std::max(hard_curb_clearance, soft_curb_clearance) +
          0.5 * veh_geo_params.width() + kCutoffDistExtraBuffer;
      MinSegmentDistanceProblem curb_msd(
          std::move(curb_segments), FLAGS_msd_static_boundary_cost_use_qtfm_v2,
          cutoff_distance, start_segment_ids);

      if (FLAGS_static_boundary_canvas_level >= 1 &&
          FLAGS_msd_static_boundary_cost_use_qtfm_v2) {
        vis::Canvas *canvas_curbs = nullptr;
        canvas_curbs = &vantage_client_man::GetCanvas(
            absl::StrFormat("%s/qtfm_for_static_boundary_cost", base_name));
        curb_msd.DrawQtfmSegmentMatcher(canvas_curbs);
      }

      std::vector<Vec2d> circle_center_offsets;
      std::vector<double> circle_radiuses;
      circle_center_offsets.reserve(
          trajectory_optimizer_vehicle_model_params.circle_size());
      circle_radiuses.reserve(
          trajectory_optimizer_vehicle_model_params.circle_size());
      for (const auto &circle :
           trajectory_optimizer_vehicle_model_params.circle()) {
        circle_center_offsets.push_back(
            Vec2d(circle.dist_to_rac() * std::cos(circle.angle_to_axis()),
                  circle.dist_to_rac() * std::sin(circle.angle_to_axis())));
        circle_radiuses.push_back(circle.radius());
      }

      costs->emplace_back(std::make_unique<MsdStaticBoundaryCost<Mfob>>(
          veh_geo_params, std::move(curb_msd),
          /*sub_names=*/std::vector<std::string>({"Soft", "Hard"}),
          std::vector<double>{soft_curb_clearance, hard_curb_clearance},
          cascade_gains, circle_center_offsets, circle_radiuses,
          /*effect_index=*/optimizer::kFreeIndex, "MsdStaticBoundaryCost",
          cost_weight_params.msd_static_boundary_cost_weight(),
          /*enable_fast_math=*/true));
    }
  } else {
    const std::vector<std::string> sub_names = {"Soft", "Hard"};
    for (const bool left : {false, true}) {
      const std::vector<double> cascade_buffers = {
          get_soft_curb_clearance(plan_start_point.v()),
          get_hard_curb_clearance(left ? av_curb_pair_or->second
                                       : av_curb_pair_or->first)};

      costs->emplace_back(std::make_unique<StaticBoundaryCost<Mfob>>(
          veh_geo_params, station_points,
          left ? left_dists_to_curb : right_dists_to_curb, left,
          mid_edges_to_center, sub_names, cascade_buffers, cascade_gains,
          absl::StrCat("StaticBoundaryCost: ",
                       left ? "left curb" : "right curb"),
          /*ignore_head=*/false, /*ignore_tail=*/false,
          /*scale=*/
          left ? cost_weight_params.left_static_boundary_cost_weight()
               : cost_weight_params.right_static_boundary_cost_weight(),
          /*enable_fast_math=*/true));
    }
    // Canvas visualization.
    if (FLAGS_static_boundary_canvas_level >= 1) {
      vis::Color curb_color(1.0, 0.7, 0.7);
      vis::Canvas &canvas = vantage_client_man::GetCanvas(
          absl::StrFormat("%s/drive_passage_curb", base_name));
      canvas.SetGroundZero(1);
      std::vector<Vec2d> left_curbs;
      std::vector<Vec2d> right_curbs;
      for (const auto &station : drive_passage.stations()) {
        const Vec2d station_pose = station.xy();
        const Vec2d station_norm = station.tangent().Perp();
        const auto curb_pair = drive_passage.QueryCurbOffsetAt(station_pose);
        if (curb_pair.ok()) {
          const Vec2d left_curb =
              station_pose + curb_pair->second * station_norm;
          const Vec2d right_curb =
              station_pose + curb_pair->first * station_norm;
          canvas.DrawCircle(Vec3d(left_curb, 0.0), 0.1, curb_color);
          canvas.DrawCircle(Vec3d(right_curb, 0.0), 0.1, curb_color);
          if (FLAGS_static_boundary_canvas_level >= 3) {
            canvas.DrawLine(Vec3d(left_curb, 0.0), Vec3d(right_curb, 0.0),
                            curb_color);
          }
          if (FLAGS_static_boundary_canvas_level >= 4) {
            left_curbs.push_back(left_curb);
            right_curbs.push_back(right_curb);
          }
        }
        if (FLAGS_static_boundary_canvas_level >= 2) {
          canvas.DrawCircle(Vec3d(station_pose, 0.0), 0.1, curb_color,
                            curb_color);
        }
      }
      if (FLAGS_static_boundary_canvas_level >= 4) {
        for (int i = 0; i + 1 < left_curbs.size(); ++i) {
          canvas.DrawLine(Vec3d(left_curbs[i], 0.0),
                          Vec3d(left_curbs[i + 1], 0.0), vis::Color::kGray);
          canvas.DrawLine(Vec3d(right_curbs[i], 0.0),
                          Vec3d(right_curbs[i + 1], 0.0), vis::Color::kGray);
        }
      }
    }
  }

  // Path boundary costs.
  std::vector<double> rear_gain = {
      cost_weight_params.path_boundary_cost_params()
          .rear_path_boundary_cost_weight()};
  std::vector<double> front_gain = {
      cost_weight_params.path_boundary_cost_params()
          .front_path_boundary_cost_weight()};
  std::vector<double> buffers_min = {
      cost_weight_params.path_boundary_cost_params().buffer_min()};
  std::vector<double> rear_buffers_max = {
      cost_weight_params.path_boundary_cost_params().rear_buffer_max()};
  std::vector<double> front_buffers_max = {
      cost_weight_params.path_boundary_cost_params().front_buffer_max()};
  std::vector<double> mid_buffers_max = {
      cost_weight_params.path_boundary_cost_params().mid_buffer_max()};

  if (FLAGS_traj_opt_enable_inner_path_boundary) {
    rear_gain.push_back(FLAGS_traj_opt_inner_path_boundary_gain_factor *
                        cost_weight_params.target_path_boundary_cost_params()
                            .rear_path_boundary_cost_weight());
    front_gain.push_back(FLAGS_traj_opt_inner_path_boundary_gain_factor *
                         cost_weight_params.target_path_boundary_cost_params()
                             .front_path_boundary_cost_weight());
    buffers_min.push_back(
        cost_weight_params.target_path_boundary_cost_params().buffer_min());
    rear_buffers_max.push_back(
        cost_weight_params.target_path_boundary_cost_params()
            .rear_buffer_max());
    front_buffers_max.push_back(
        cost_weight_params.target_path_boundary_cost_params()
            .front_buffer_max());
    mid_buffers_max.push_back(
        cost_weight_params.target_path_boundary_cost_params().mid_buffer_max());
  }
  for (const bool left : {false, true}) {
    std::vector<std::vector<double>> dists_to_path_boundary;
    std::vector<double> cascade_gains;
    if (left) {
      dists_to_path_boundary.push_back(left_dists_to_path_boundary);
      cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                  .left_path_boundary_cost_weight());
      if (FLAGS_traj_opt_enable_inner_path_boundary) {
        dists_to_path_boundary.push_back(left_dists_to_target_path_boundary);
        cascade_gains.push_back(
            cost_weight_params.target_path_boundary_cost_params()
                .left_path_boundary_cost_weight());
      }
    } else {
      dists_to_path_boundary.push_back(right_dists_to_path_boundary);
      cascade_gains.push_back(cost_weight_params.path_boundary_cost_params()
                                  .right_path_boundary_cost_weight());
      if (FLAGS_traj_opt_enable_inner_path_boundary) {
        dists_to_path_boundary.push_back(right_dists_to_target_path_boundary);
        cascade_gains.push_back(
            cost_weight_params.target_path_boundary_cost_params()
                .right_path_boundary_cost_weight());
      }
    }
    std::vector<std::vector<double>> ref_gains;
    ref_gains.reserve(cascade_gains.size());
    std::vector<double> outer_path_boundary_gains(station_points.size(), 1.0);
    ref_gains.push_back(std::move(outer_path_boundary_gains));
    if (FLAGS_traj_opt_enable_inner_path_boundary) {
      ref_gains.push_back(inner_path_boundary_gains);
    }
    std::vector<std::string> sub_names = {"Outer"};
    if (FLAGS_traj_opt_enable_inner_path_boundary) {
      sub_names.emplace_back("Inner");
    }
    costs->emplace_back(std::make_unique<MfobPathBoundaryCost<Mfob>>(
        veh_geo_params, station_points, /*center_line_helper=*/nullptr,
        reference_center_l_offsets, dists_to_path_boundary, left,
        mid_edges_to_center, std::move(ref_gains), sub_names,
        FLAGS_mfob_path_boundary_use_qtfm, buffers_min, rear_buffers_max,
        front_buffers_max, mid_buffers_max, cascade_gains, rear_gain,
        front_gain,
        absl::StrCat("PathBoundaryCost: ",
                     left ? "left path boundary" : "right path boundary"),
        /*scale=*/1.0, /*enable_fast_math=*/true));
  }
}

void AddReferencePathCost(
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>
        &center_line_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  SCOPED_QTRACE("AddReferencePathCost");

  std::vector<double> ref_path_deviation_gains(
      center_line_query_helper->points()->size() - 1, 1.0);

  costs->emplace_back(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      cost_weight_params.reference_path_cost_weight().path_gain(),
      cost_weight_params.reference_path_cost_weight().end_state_gain(),
      *center_line_query_helper->points(), center_line_query_helper.get(),
      std::move(ref_path_deviation_gains), "MfobReferenceLineDeviationCost",
      cost_weight_params.reference_path_cost_weight()
          .reference_path_cost_weight()));
  costs->emplace_back(std::make_unique<EndHeadingCost<Mfob>>(
      *center_line_query_helper->points(), center_line_query_helper.get(),
      "MfobEndHeadingCost",
      cost_weight_params.reference_heading_cost_weight()));
}

absl::Status AddCosts(
    std::string_view base_name, const std::vector<TrajectoryPoint> &init_traj,
    const std::vector<TrajectoryPoint> &prev_traj,
    const std::vector<Vec2d> &ref_path_vector,
    const DrivePassage &drive_passage, const PathSlBoundary &path_sl_boundary,
    const ConstraintManager &constraint_manager,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const PlannerSemanticMapManager &psmm, double v_now,
    const TurnSpeedLimitingParamsProto &turn_speed_limit_params,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const TrajectoryOptimizerCostConfigProto &cost_config,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>
        &center_line_query_helper,
    const std::vector<std::unique_ptr<AvModelHelper<Mfob>>> &av_model_helpers,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs,
    TrajectoryOptimizerDebugProto *traj_opt_debug_proto,
    ThreadPool *thread_pool) {
  SCOPED_QTRACE("AddCosts");

  if (cost_config.enable_regularizers_cost()) {
    AddRegularizersCost(init_traj, cost_weight_params, costs);
  }
  if (cost_config.enable_acceleration_and_jerk_cost()) {
    AddAccelAndJerkCost(cost_weight_params, motion_constraint_params, costs);
  }
  if (cost_config.enable_curvature_cost()) {
    AddCurvatureCost(cost_weight_params, veh_geo_params, vehicle_drive_params,
                     motion_constraint_params, costs);
  }
  if (cost_config.enable_forward_speed_cost()) {
    // Forward speed cost.
    costs->emplace_back(std::make_unique<ForwardSpeedCost<Mfob>>(
        "MfobForwardSpeedCost",
        cost_weight_params.forward_speed_cost_weight()));
  }
  if (cost_config.enable_immediate_future_cost()) {
    AddImmediateFutureCost(prev_traj, v_now, cost_weight_params, costs);
  }
  if (cost_config.enable_speed_limit_cost()) {
    AddSpeedLimitCost(drive_passage, constraint_manager,
                      turn_speed_limit_params, cost_weight_params,
                      veh_geo_params, costs, traj_opt_debug_proto);
  }

  std::vector<double> inner_path_boudnary_gains(path_sl_boundary.size(), 1.0);

  if (cost_config.enable_object_cost()) {
    optimizer::AddObjectCosts(
        base_name, init_traj, drive_passage, path_sl_boundary,
        constraint_manager, st_traj_mgr, cost_weight_params, veh_geo_params,
        trajectory_optimizer_vehicle_model_params, av_model_helpers,
        &inner_path_boudnary_gains, costs, thread_pool);
  }
  if (cost_config.enable_reference_path_cost()) {
    AddReferencePathCost(cost_weight_params, center_line_query_helper, costs);
  }
  if (cost_config.enable_static_boundary_cost()) {
    // Curb and path boundary costs.
    AddStaticBoundaryCosts(
        base_name, init_traj.front(), drive_passage, psmm, path_sl_boundary,
        std::move(inner_path_boudnary_gains), cost_weight_params,
        veh_geo_params, trajectory_optimizer_vehicle_model_params,
        center_line_query_helper, costs);
  }
  return absl::OkStatus();
}

absl::Status AddCenterLineQueryHelper(
    std::string_view base_name, const std::vector<Vec2d> &ref_path_vector,
    std::unique_ptr<CenterLineQueryHelper<Mfob>> *center_line_query_helper) {
  *center_line_query_helper = std::make_unique<CenterLineQueryHelper<Mfob>>(
      ref_path_vector, "MfobCenterLineQueryHelper",
      /*use_qtfm=*/FLAGS_trajectory_optimizer_frenet_frame_uses_qtfm_v2);
  return absl::OkStatus();
}

absl::Status AddAvModelHelpers(
    std::string_view base_name,
    const VehicleGeometryParamsProto &veh_geo_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    std::vector<std::unique_ptr<AvModelHelper<Mfob>>> *av_model_helpers) {
  av_model_helpers->reserve(
      trajectory_optimizer_vehicle_model_params.circle_size());
  for (const auto &circle :
       trajectory_optimizer_vehicle_model_params.circle()) {
    av_model_helpers->push_back(std::make_unique<AvModelHelper<Mfob>>(
        circle.dist_to_rac(), circle.angle_to_axis(),
        /*enable_fast_math=*/true, "MfobAvModelHelper(" + circle.name() + ")"));
  }
  return absl::OkStatus();
}

// Extend by PurePursuit with a constant speed.
std::vector<TrajectoryPoint> GetExtendStateByPurePursuit(
    const TrajectoryPoint &extend_start_point, int k_extend_steps,
    double target_v, const DrivePassage &drive_passage,
    const PathSlBoundary &path_sl_boundary,
    const VehicleGeometryParamsProto &veh_geo_params,
    const MotionConstraintParamsProto &motion_constraint_params) {
  // Extend by PurePursuit with a constant speed. Copied from
  // GetExtendStateByPurePursuit.
  constexpr double kLateralLookAhead = 1.0;
  Mfob::StateType x = Mfob::MakeState(
      extend_start_point.pos().x(), extend_start_point.pos().y(),
      extend_start_point.theta(), extend_start_point.v(),
      extend_start_point.kappa(), extend_start_point.a(),
      extend_start_point.psi(), extend_start_point.s());
  std::vector<TrajectoryPoint> res;
  const double time_base = extend_start_point.t();
  while (res.size() < k_extend_steps) {
    const double a = (target_v - Mfob::StateGetV(x)) / kDdpTrajectoryTimeStep;
    const double j = (a - Mfob::StateGetA(x)) / kDdpTrajectoryTimeStep;
    Vec2d lateral_target_pos = drive_passage.stations().back().xy();
    const double lateral_look_ahead_dist =
        kLateralLookAhead * Mfob::StateGetV(x) + veh_geo_params.wheel_base();
    const auto lateral_nearest_point_status =
        drive_passage.QueryFrenetCoordinateAt(Mfob::StateGetPos(x));
    if (lateral_nearest_point_status.ok()) {
      lateral_target_pos = path_sl_boundary.QueryReferenceCenterXY(
          lateral_look_ahead_dist + lateral_nearest_point_status->s);
    }
    const double alpha =
        Vec2d(lateral_target_pos - Mfob::StateGetPos(x)).Angle() -
        Mfob::StateGetTheta(x);
    const double kappa = 2.0 * fast_math::Sin(alpha) / lateral_look_ahead_dist;
    const double psi =
        (kappa - Mfob::StateGetKappa(x)) / kDdpTrajectoryTimeStep;

    const double chi = (psi - Mfob::StateGetPsi(x)) / kDdpTrajectoryTimeStep;

    const auto u = Mfob::MakeControl(chi, j);
    x = Mfob::EvaluateF(res.size() - 1, x, u, kDdpTrajectoryTimeStep);

    TrajectoryPoint next_pt;
    next_pt.set_pos(Mfob::StateGetPos(x));
    next_pt.set_theta(Mfob::StateGetTheta(x));
    next_pt.set_kappa(Mfob::StateGetKappa(x));
    next_pt.set_psi(Mfob::StateGetPsi(x));
    next_pt.set_v(Mfob::StateGetV(x));
    next_pt.set_a(Mfob::StateGetA(x));
    next_pt.set_s(Mfob::StateGetS(x));
    next_pt.set_t((res.empty() ? time_base : res.back().t()) +
                  kDdpTrajectoryTimeStep);
    res.push_back(std::move(next_pt));
  }
  return res;
}

// Convert an input trajectory with time step kTrajectoryTimeStep to comply with
// trajectory optimizer input format: horizon kDdpTrajectorySteps and time step
// kDdpTrajectoryTimeStep.
std::vector<TrajectoryPoint> ToTrajectoryOptimizerInput(
    const std::vector<TrajectoryPoint> &input_traj,
    const DrivePassage &drive_passage, const PathSlBoundary &path_sl_boundary,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &veh_geo_params) {
  if (input_traj.empty()) return input_traj;
  std::vector<TrajectoryPoint> res;
  res.reserve(kDdpTrajectorySteps);
  for (int i = 0; i < kDdpTrajectorySteps; ++i) {
    if (i * optimizer::kSampleStep >= input_traj.size()) break;
    res.push_back(input_traj[i * optimizer::kSampleStep]);
  }
  if (res.size() == kDdpTrajectorySteps) return res;

  const double input_traj_end_v = input_traj.back().v();
  const auto traj_end_sl_status =
      drive_passage.QueryFrenetCoordinateAt(input_traj.back().pos());
  if (!traj_end_sl_status.ok()) {
    QLOG(ERROR) << "Input trajectory end is out of drive passage: "
                << input_traj.back().DebugString();
    // Extend by constant acceleration and curvature.
    constexpr double kMinSpeed = 1e-6;
    constexpr double kMinDist = 1e-6;
    while (res.size() < kDdpTrajectorySteps) {
      auto &pt = res.back();
      const double dist =
          std::max(kMinDist, pt.v() * kDdpTrajectoryTimeStep +
                                 0.5 * pt.a() * Sqr(kDdpTrajectoryTimeStep));
      PathPoint path_point;
      path_point.set_x(pt.pos().x());
      path_point.set_y(pt.pos().y());
      path_point.set_s(pt.s());
      path_point.set_theta(pt.theta());
      path_point.set_kappa(pt.kappa());
      path_point.set_lambda(pt.lambda());
      const auto next_path_point = GetPathPointAlongCircle(path_point, dist);
      auto next_pt = pt;
      next_pt.set_t(pt.t() + kDdpTrajectoryTimeStep);
      next_pt.set_pos(ToVec2d(next_path_point));
      next_pt.set_s(next_path_point.s());
      next_pt.set_theta(next_path_point.theta());
      next_pt.set_kappa(next_path_point.kappa());
      next_pt.set_v(
          std::max(kMinSpeed, pt.v() + pt.a() * kDdpTrajectoryTimeStep));
      next_pt.set_a(
          std::max(pt.a(), -1.0 * next_pt.v() / kDdpTrajectoryTimeStep));
      pt.set_j(std::clamp((next_pt.a() - next_pt.a()) / kDdpTrajectoryTimeStep,
                          motion_constraint_params.max_decel_jerk(),
                          motion_constraint_params.max_accel_jerk()));
      next_pt.set_psi(next_path_point.lambda() * next_pt.v());
      res.push_back(std::move(next_pt));
    }
    res.back().set_j((res.rbegin() + 1)->j());
    return res;
  }

  const double time_to_extend =
      kDdpTrajectoryTimeStep * (kDdpTrajectorySteps - res.size() + 1);
  constexpr double kMinEndSpeed = 2.0;  // m/s.
  const double target_v = std::min(
      std::max(kMinEndSpeed, input_traj_end_v),
      (drive_passage.end_s() - traj_end_sl_status->s) / time_to_extend);

  const auto extend_state = GetExtendStateByPurePursuit(
      res.back(), kDdpTrajectorySteps - res.size(), target_v, drive_passage,
      path_sl_boundary, veh_geo_params, motion_constraint_params);

  for (int k = 0; k < extend_state.size(); ++k) {
    res.push_back(extend_state[k]);
  }
  return res;
}

}  // namespace

absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput &input,
    const PlannerParamsProto &planner_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    TrajectoryOptimizerDebugProto *optimizer_debug,
    vis::vantage::ChartDataBundleProto *charts_data, ThreadPool *thread_pool) {
  SCOPED_QTRACE("EstPlanner/OptimizeTrajectory");

  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_NOTNULL(input.path_sl_boundary);
  QCHECK_NOTNULL(input.constraint_mgr);
  QCHECK_NOTNULL(input.planner_semantic_map_mgr);

  if (const auto input_status = CheckInputQuality(input); !input_status.ok()) {
    return input_status;
  }

  // Currently we only accept kDdpTrajectoryTimeStep is an integer multiple of
  // kTrajectoryTimeStep.
  static_assert(
      static_cast<int>(kDdpTrajectoryTimeStep / kTrajectoryTimeStep) ==
          kDdpTrajectoryTimeStep / kTrajectoryTimeStep,
      "kDdpTrajectoryTimeStep must be an integer multiple of "
      "kTrajectoryTimeStep");

  const auto &plan_start_point = input.plan_start_point;

  optimizer_debug->set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_start_time));

  // Convert from ApolloTrajectoryPoint to TrajectoryPoint.
  // TODO(renjie): Replace with ToTrajectoryPoint once initializer can produce
  // third-order Apollo trajectory.
  std::vector<TrajectoryPoint> init_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPointFromSecondOrderApollo(input.trajectory),
      *input.drive_passage, *input.path_sl_boundary,
      planner_params.motion_constraint_params(), veh_geo_params);

  // The size of prev_traj should be either 0 or equal to init_traj.
  // Note(renjie): make sure that prev_traj has been shifted in time to now.
  std::vector<TrajectoryPoint> prev_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPoint(input.previous_trajectory), *input.drive_passage,
      *input.path_sl_boundary, planner_params.motion_constraint_params(),
      veh_geo_params);
  QCHECK(prev_traj.empty() || prev_traj.size() == init_traj.size());

  ScopedMultiTimer timer("trajectory_optimizer");

  const std::string base_name_with_plan_id =
      absl::StrFormat("traj_opt_%d", input.plan_id);
  const std::string base_name = "traj_opt";

  constexpr double kStartSquareDistThreshold = 1.0;
  constexpr double kStartVDiffThreshold = 2.0;
  const auto &init_traj_start_point = init_traj.front();
  const double start_square_dist = (Vec2d(plan_start_point.path_point().x(),
                                          plan_start_point.path_point().y()) -
                                    init_traj_start_point.pos())
                                       .squaredNorm();
  const double start_v_diff = plan_start_point.v() - init_traj_start_point.v();
  if (start_square_dist < kStartSquareDistThreshold &&
      std::abs(start_v_diff) < kStartVDiffThreshold) {
    init_traj.begin()->FromProto(plan_start_point);
  } else {
    QLOG(FATAL) << "Initializer first state(" << init_traj_start_point.pos().x()
                << "," << init_traj_start_point.pos().y() << ","
                << init_traj_start_point.v()
                << ") too far from plan start point("
                << plan_start_point.path_point().x() << ","
                << plan_start_point.path_point().y() << ","
                << plan_start_point.v() << ")!";
  }

  const auto params = planner_params.trajectory_optimizer_params();
  const std::vector<TrajectoryPoint> smooth_init_traj =
      SmoothTrajectoryByMixedFourthOrderDdp(
          init_traj, base_name_with_plan_id, params.smoother_params(),
          planner_params.motion_constraint_params(), veh_geo_params,
          veh_drive_params);
  timer.Mark("smooth_init_trajectory");

  auto add_cost_start_time = absl::Now();

  const auto &drive_passage = *input.drive_passage;
  const auto &path_sl_boundary = *input.path_sl_boundary;
  const auto &constraint_manager = *input.constraint_mgr;
  const auto &st_traj_mgr = *input.st_traj_mgr;
  const double v_now = input.plan_start_point.v();
  const auto turn_speed_limit_params =
      planner_params.turn_speed_limiting_params();

  const std::vector<Vec2d> ref_path_vector(
      path_sl_boundary.reference_center_xy_vector().begin(),
      path_sl_boundary.reference_center_xy_vector().end());

  std::unique_ptr<CenterLineQueryHelper<Mfob>> center_line_query_helper;
  QCHECK_OK(AddCenterLineQueryHelper(base_name_with_plan_id, ref_path_vector,
                                     &center_line_query_helper));

  std::vector<std::unique_ptr<AvModelHelper<Mfob>>> av_model_helpers;
  QCHECK_OK(AddAvModelHelpers(
      base_name_with_plan_id, veh_geo_params,
      planner_params.trajectory_optimizer_vehicle_model_params(),
      &av_model_helpers));

  std::vector<std::unique_ptr<Cost<Mfob>>> costs;
  QCHECK_OK(AddCosts(
      base_name_with_plan_id, init_traj, prev_traj, ref_path_vector,
      drive_passage, path_sl_boundary, constraint_manager, st_traj_mgr,
      *input.planner_semantic_map_mgr, v_now, turn_speed_limit_params,
      params.cost_weight_params(), params.cost_config(), veh_geo_params,
      veh_drive_params, planner_params.motion_constraint_params(),
      planner_params.trajectory_optimizer_vehicle_model_params(),
      center_line_query_helper, av_model_helpers, &costs, optimizer_debug,
      thread_pool));

  const double add_cost_time =
      absl::ToDoubleMilliseconds(absl::Now() - add_cost_start_time);

  auto problem = std::make_unique<Mfob>(
      &planner_params.motion_constraint_params(), &veh_geo_params,
      &veh_drive_params, kDdpTrajectoryTimeStep, /*enable_post_process=*/false);

  problem->AddCostHelper(std::move(center_line_query_helper));
  for (auto &av_model_helper : av_model_helpers) {
    problem->AddCostHelper(std::move(av_model_helper));
  }
  for (auto &cost : costs) {
    problem->AddCost(std::move(cost));
  }

  DdpOptimizer<Mfob> solver(problem.get(),
                            /*owner=*/"trajectory_optimizer",
                            /*verbosity=*/FLAGS_traj_opt_verbosity_level,
                            params.optimizer_params());
  IterationVisualizerHook<Mfob> iteration_visualizer_hook(
      base_name_with_plan_id, true);
  if (FLAGS_traj_opt_canvas_level >= 2) {
    solver.AddHook(&iteration_visualizer_hook);
  }
  OptimizerSolverDebugHook<Mfob> solver_debug_hook(smooth_init_traj.front(),
                                                   st_traj_mgr);
  solver.AddHook(&solver_debug_hook);

  const auto &vehicle_model_params =
      planner_params.trajectory_optimizer_vehicle_model_params();
  std::vector<IterationAvModelVisualizerHook<Mfob>::circle> av_model_circle;
  av_model_circle.reserve(vehicle_model_params.circle_size());
  for (const auto &circle : vehicle_model_params.circle()) {
    av_model_circle.push_back(
        {circle.dist_to_rac(), circle.angle_to_axis(), circle.radius()});
  }
  IterationAvModelVisualizerHook<Mfob> iteration_av_model_visualizer_hook(
      base_name_with_plan_id, av_model_circle, &veh_geo_params);
  if (FLAGS_traj_opt_draw_circle && FLAGS_traj_opt_canvas_level >= 2) {
    solver.AddHook(&iteration_av_model_visualizer_hook);
  }

  std::vector<TrajectoryPoint> result_points;

  std::string error_code = "";
  bool optimizer_solve_success = true;

  solver.SetInitialPoints(smooth_init_traj);

  auto solve_start_time = absl::Now();
  const auto smooth_init_output = solver.Solve(
      /*forward=*/true);
  const double solve_time =
      absl::ToDoubleMilliseconds(absl::Now() - solve_start_time);

  if (!smooth_init_output.ok()) {
    error_code = smooth_init_output.status().message();
    optimizer_solve_success = false;
  } else {
    result_points = std::move(*smooth_init_output);
  }
  timer.Mark("solve");

  TrajectoryOptimizerOutput output;
  // BANDAID(jingqiao): Refactor to solve code divergence in the future.
  if (FLAGS_dump_expert_policy) {
    TrajectoryProto traj_proto;
    // TODO(jingqiao): Use input instead of directly reading file.
    QCHECK(file_util::TextFileToProto(
        absl::StrCat(FLAGS_specific_snapshot_folder, "pose_trajectory.pb.txt"),
        &traj_proto))
        << "Read expert policy failed!!!!";

    output.expert_costs =
        PoseTrajectoryToPolicy(solver, traj_proto, FLAGS_auto_tuning_gamma);
  }
  if (FLAGS_auto_tuning_mode) {
    VLOG(3) << "Final xs after solve end: "
            << solver_debug_hook.solve_end_xs.transpose();
    VLOG(3) << "Final us after solve end: "
            << solver_debug_hook.solve_end_us.transpose();
    solver.EvaluateEachDiscountedAccumulativeCost(
        Mfob::GetStatesBeforeStep(solver_debug_hook.solve_end_xs,
                                  kDdpTrajectoryStepsDAT),
        Mfob::GetControlsBeforeStep(solver_debug_hook.solve_end_us,
                                    kDdpTrajectoryStepsDAT),
        FLAGS_auto_tuning_gamma, &output.feature_costs);
  }

  ToDebugProto(init_traj, smooth_init_traj, result_points, solver_debug_hook,
               optimizer_debug, thread_pool);

  std::vector<TrajectoryPlotInfo> trajs = {
      {.traj = init_traj, .name = "init", .color = vis::Color::kRed},
      {.traj = smooth_init_traj,
       .name = "smooth_init",
       .color = vis::Color::kYellow},
      {.traj = result_points, .name = "res", .color = vis::Color::kBlue}};
  if (FLAGS_send_traj_optimizer_result_to_canvas) {
    CanvasDrawTrajectories(base_name_with_plan_id, trajs);
  }
  if (charts_data != nullptr) {
    optimizer::AddTrajCharts("traj_opt", trajs, charts_data->mutable_charts());
    optimizer::AddCostsChart(base_name, optimizer_debug->ddp(), charts_data);
  }

  if (FLAGS_enable_ipopt_solver) {
    const auto ipopt_output = optimizer::CompareWithIpopt(
        base_name, base_name_with_plan_id, init_traj, smooth_init_traj,
        result_points,
        /*enable_comparison_debug_info_output=*/optimizer_solve_success,
        optimizer_debug->ddp(), trajs.back(), problem.get(), charts_data);
    if (!ipopt_output.ok()) {
      LOG(INFO) << "Ipopt solve failed, " << ipopt_output.message();
    } else {
      LOG(INFO) << "Ipopt solve succeed. ";
    }
  }

  // And this at the end in case some one did optimizer_debug->reset()
  // somewhere.
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_add_cost_time(
      add_cost_time);
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_solve_time(
      solve_time);

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(problem);

  if (!optimizer_solve_success) {
    return absl::InternalError(error_code);
  }

  std::vector<ApolloTrajectoryPointProto> output_traj =
      ToApolloTrajectoryPointProto(result_points);
  output.trajectory_proto = ToApolloTrajectoryPointProto(result_points);
  output.trajectory = std::move(result_points);
  return output;
}

}  // namespace planner
}  // namespace qcraft
