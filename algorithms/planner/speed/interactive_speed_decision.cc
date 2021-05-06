#include "onboard/planner/speed/interactive_speed_decision.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/math/intelligent_driver_model.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/post_st_boundary_modifier.h"
#include "onboard/planner/speed/empty_road_speed.h"
#include "onboard/planner/speed/gridded_svt_graph.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_graph_data.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/terminal_color.h"
#include "onboard/vis/common/color.h"

DEFINE_bool(planner_add_optimal_preliminary_speed_to_debug, false,
            "Whether add optimal preliminary speed to debug.");

DECLARE_bool(enable_interactive_speed_decision_draw_st_traj);

namespace qcraft::planner {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-6;

// Agent decel discomfort with different priority.
const PiecewiseLinearFunction<double> low_prio_decel_discomfort_plf(
    {-3.0 - kEps, -3.0, -2.0, -1.0, -0.0}, {kInf, 40.0, 15.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> equal_prio_decel_discomfort_plf(
    {-2.2 - kEps, -2.2, -2.0, -1.0, -0.0}, {kInf, 35.0, 15.0, 5.0, 0.0});
const PiecewiseLinearFunction<double> high_prio_decel_discomfort_plf(
    {-1.5 - kEps, -1.5, -1.0, -0.5, -0.0}, {kInf, 30.0, 10.0, 5.0, 0.0});

const PiecewiseLinearFunction<double> agent_waiting_discomfort_plf(
    {0.0, 0.8, 0.8 + kEps}, {0.0, 25.0, kInf});

struct YieldingResult {
  bool already_yield = false;
  double start_yield_time = 0.0;             // s
  double arrive_yielding_point_time = 0.0;   // s
  double arrive_yielding_point_decel = 0.0;  // m/ss
  double v_at_yielding_point = 0.0;          // m/s
  double s_at_yielding_point = 0.0;          // m
  double waiting_duration = 0.0;             // s
  double finish_yield_time = 0.0;            // s
};

struct InteractiveResult {
  double cost = 0.0;
  std::optional<YieldingResult> yielding_result = std::nullopt;

  std::string traj_id;
  StBoundaryProto::DecisionType decision = StBoundaryProto::UNKNOWN;

  const SpacetimeObjectTrajectory* spacetime_obj = nullptr;
  const StBoundary* st_boundary = nullptr;
};

inline idm::Parameters GetIDMParams(double v_desire) {
  return idm::Parameters{.v_desire = v_desire,      // m/s
                         .s_min = 4.0,              // m
                         .t_desire = 2.5,           // s
                         .acc_max = 2.0,            // m/ss
                         .comfortable_brake = 2.0,  // m/ss
                         .brake_max = 4.0,          // m/ss
                         .delta = 4.0};
}

// Penalize 'distance' to previous optimal speed profile.
double CalculateConsistencyCost(
    absl::Span<const StBoundaryWithDecision> st_boundaries,
    const SpeedVector& speed_profile) {
  // TODO(yumeng): Pass in prev speed profile.
  double cost = 0.0;
  return cost;
}

// TODO(yumeng): In the future, try a more accurate free driving model which
// considering speed limit.
std::vector<AccelPoint> GenerateFreeDrivingAccelPointList(double start_time,
                                                          double total_time) {
  constexpr double kFreeDriveingAccelDuration = 1.5;  // s
  constexpr double kObjectFreeAccel = 0.8;            // m/ss
  std::vector<AccelPoint> accel_point_list;
  accel_point_list.emplace_back(start_time, kObjectFreeAccel);
  if (total_time - start_time > kFreeDriveingAccelDuration) {
    accel_point_list.emplace_back(start_time + kFreeDriveingAccelDuration, 0.0);
  }
  return accel_point_list;
}

absl::StatusOr<BruteForceFrenetFrame> BuildAgentFrenetFrame(
    const SpacetimeObjectTrajectory& spacetime_obj) {
  const auto& st_traj_points = spacetime_obj.trajectory()->points();
  std::vector<Vec2d> agent_traj_points_2d;
  agent_traj_points_2d.reserve(st_traj_points.size());
  for (const auto& point : st_traj_points) {
    agent_traj_points_2d.push_back(point.pos());
  }
  return BuildBruteForceFrenetFrame(agent_traj_points_2d);
}

idm::Parameters CalculateAgentIdmParams(
    const SpacetimeObjectTrajectory& spacetime_obj) {
  // Use prediction max speed as IDM desire speed.
  double agent_predicted_max_v = 0.0;
  for (const auto& state : spacetime_obj.states()) {
    agent_predicted_max_v =
        std::max(agent_predicted_max_v, state.traj_point->v());
  }
  return GetIDMParams(agent_predicted_max_v);
}

inline double GetAgentTimeStep(const SpacetimeObjectTrajectory& spacetime_obj) {
  return spacetime_obj.states().size() > 1
             ? spacetime_obj.states()[1].traj_point->t() -
                   spacetime_obj.states()[0].traj_point->t()
             : prediction::kPredictionTimeStep;
}

// Calculate agent following trajectory from that state.
struct StartFollowState {
  double t = 0.0;
  double v = 0.0;
  double a = 0.0;
  double s = 0.0;
};

StartFollowState CalculateAgentStartFollowState(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const YieldingResult& yielding_result) {
  if (yielding_result.already_yield) {
    return StartFollowState{.t = 0.0,
                            .v = spacetime_obj.pose().v(),
                            .a = spacetime_obj.pose().a(),
                            .s = 0.0};
  }
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();
  // Define agent will sliding a period of time to switching from yielding
  // to following.
  constexpr double kAgentSlidingDuration = 0.5;  // s
  return StartFollowState{
      .t = std::min(yielding_result.finish_yield_time + kAgentSlidingDuration,
                    obj_total_time),
      .v = yielding_result.v_at_yielding_point,
      .a = 0.0,
      .s = yielding_result.s_at_yielding_point +
           yielding_result.v_at_yielding_point * kAgentSlidingDuration};
}

std::vector<AccelPoint> CalculateAccelPointListBeforeFollow(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const YieldingResult& yielding_result) {
  if (yielding_result.already_yield) {
    return {AccelPoint(0.0, spacetime_obj.pose().a())};
  }

  std::vector<AccelPoint> accel_point_list;
  accel_point_list.reserve(3);
  // Add yielding decel point.
  accel_point_list.emplace_back(yielding_result.start_yield_time,
                                yielding_result.arrive_yielding_point_decel);
  // Add waiting accel point.
  if (yielding_result.waiting_duration > 0.0) {
    accel_point_list.emplace_back(yielding_result.arrive_yielding_point_time,
                                  /*waiting_accel=*/0.0);
  }
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();
  if (yielding_result.finish_yield_time > obj_total_time) {
    return accel_point_list;
  }
  // Define agent will sliding a period of time to switching from yielding
  // to following.
  accel_point_list.emplace_back(yielding_result.finish_yield_time,
                                /*sliding_accel=*/0.0);
  return accel_point_list;
}

inline bool IsAgentFreeDriving(const SpacetimeObjectTrajectory& spacetime_obj,
                               const DiscretizedPath& path,
                               const OverlapInfo& end_overlap, double agent_s,
                               double av_s) {
  const double agent_end_overlap_s =
      spacetime_obj.states()[end_overlap.obj_idx].traj_point->s();
  const double av_end_overlap_s = path[end_overlap.av_end_idx].s();
  const bool is_agent_traj_stay_on_av_path =
      spacetime_obj.states().size() - 1 == end_overlap.obj_idx;

  return agent_s > agent_end_overlap_s ||
         (av_s > av_end_overlap_s && !is_agent_traj_stay_on_av_path);
}

// Every time step av state when calculate agent following trajectory.
struct AvState {
  double s = 0.0;
  double v = 0.0;
  double theta = 0.0;
  Vec2d pos;
};

inline AvState CreateAvState(const DiscretizedPath& path,
                             const SpeedVector& speed_profile, double curr_t) {
  QCHECK_LE(curr_t, speed_profile.back().t());

  const auto av_speed_point = speed_profile.EvaluateByTime(curr_t);
  QCHECK(av_speed_point.has_value());
  const auto av_path_point = path.Evaluate(av_speed_point->s());
  return AvState{.s = av_speed_point->s(),
                 .v = av_speed_point->v(),
                 .theta = av_path_point.theta(),
                 .pos = Vec2d(av_path_point.x(), av_path_point.y())};
}

std::vector<AccelPoint> CalculateAgentAccelPointList(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeObjectTrajectory& spacetime_obj,
    const OverlapInfo& end_overlap, const DiscretizedPath& path,
    const SpeedVector& speed_profile, const YieldingResult& yielding_result) {
  QCHECK_GE(speed_profile.size(), 2);

  const StartFollowState start_follow_state =
      CalculateAgentStartFollowState(spacetime_obj, yielding_result);
  auto accel_point_list =
      CalculateAccelPointListBeforeFollow(spacetime_obj, yielding_result);
  const double t_step = GetAgentTimeStep(spacetime_obj);
  const double obj_total_time = spacetime_obj.states().back().traj_point->t();
  accel_point_list.reserve(
      3 + FloorToInt((obj_total_time - start_follow_state.t) / t_step));
  const auto agent_ff = BuildAgentFrenetFrame(spacetime_obj);
  if (!agent_ff.ok()) return accel_point_list;
  const auto agent_idm_params = CalculateAgentIdmParams(spacetime_obj);

  double prev_v = start_follow_state.v;
  double prev_s = start_follow_state.s;
  double prev_a = start_follow_state.a;
  double begin_follow_time = start_follow_state.t;
  // Calculate rest time following accels.
  for (double curr_t = begin_follow_time + t_step; curr_t <= obj_total_time;
       curr_t += t_step) {
    AvState av_state;
    bool is_curr_t_exceed_av_traj = false;
    if (curr_t <= speed_profile.back().t()) {
      av_state = CreateAvState(path, speed_profile, curr_t);
    } else {
      is_curr_t_exceed_av_traj = true;
    }
    const double curr_s = prev_s + prev_v * t_step + 0.5 * prev_a * Sqr(t_step);
    // Agent free driving from that time.
    if (IsAgentFreeDriving(spacetime_obj, path, end_overlap, curr_s,
                           av_state.s) ||
        is_curr_t_exceed_av_traj) {
      auto free_driving_a_list =
          GenerateFreeDrivingAccelPointList(curr_t, obj_total_time);
      std::move(free_driving_a_list.begin(), free_driving_a_list.end(),
                std::back_inserter(accel_point_list));
      break;
    }

    const auto av_back_edge_center_pos =
        av_state.pos - vehicle_geom.back_edge_to_center() *
                           Vec2d::FastUnitFromAngle(av_state.theta);
    const auto sl = agent_ff->XYToSL(av_back_edge_center_pos);
    const double ds =
        sl.s - curr_s - spacetime_obj.bounding_box().length() * 0.5;
    const double curr_v = prev_v + prev_a * t_step;

    double agent_theta = 0.0;
    const auto& st_traj_points = spacetime_obj.trajectory()->points();
    if (curr_s > st_traj_points.back().s()) {
      const auto& last_p = st_traj_points.back();
      const auto& second_to_last_p = st_traj_points[st_traj_points.size() - 2];
      agent_theta =
          LerpAngle(second_to_last_p.theta(), last_p.theta(),
                    LerpFactor(second_to_last_p.s(), last_p.s(), curr_s));
    } else {
      const auto traj_point = QueryTrajectoryPointByS(st_traj_points, curr_s);
      agent_theta = traj_point.theta();
    }

    const double agent_acc_v = std::max(
        0.0, av_state.v * fast_math::Cos(av_state.theta - agent_theta));
    const double dv = agent_acc_v - curr_v;
    double curr_a = ComputeIDMAcceleration(curr_v, ds, dv, agent_idm_params);
    if (curr_v + curr_a * t_step < 0.0) {
      curr_a = -curr_v / t_step;
    }
    accel_point_list.emplace_back(curr_t, curr_a);
    // Update.
    prev_v = curr_v;
    prev_a = curr_a;
  }

  return accel_point_list;
}

double CalculateAgentReactionTime(
    const StOverlapMetaProto& overlap_meta,
    const SpacetimeObjectTrajectory& spacetime_obj) {
  // T = f(a).
  PiecewiseLinearFunction<double> agent_reaction_time_plf;
  // Assume that agent will yield passive if it has right of way and yield
  // active if we have right of way. Define this motivation contribute to
  // reaction time.
  switch (overlap_meta.priority()) {
    case StOverlapMetaProto::HIGH: {
      const PiecewiseLinearFunction<double> low_prio_reaction_time_plf(
          {-1.0, -0.1, 0.1, 0.8}, {0.0, 0.1, 0.1, 0.5});
      agent_reaction_time_plf = low_prio_reaction_time_plf;
      break;
    }
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      const PiecewiseLinearFunction<double> equal_prio_reaction_time_plf(
          {-1.0, -0.1, 0.1, 0.8}, {0.0, 0.3, 0.3, 0.7});
      agent_reaction_time_plf = equal_prio_reaction_time_plf;
      break;
    }
    case StOverlapMetaProto::LOW: {
      const PiecewiseLinearFunction<double> high_prio_reaction_time_plf(
          {-1.0, -0.1, 0.1, 0.8}, {0.0, 0.5, 0.5, 1.2});
      agent_reaction_time_plf = high_prio_reaction_time_plf;
      break;
    }
  }

  // Only has value for source AV_CUTIN.
  if (overlap_meta.has_time_to_lc_complete()) {
    QCHECK_EQ(overlap_meta.source(), StOverlapMetaProto::AV_CUTIN);
    const PiecewiseLinearFunction<double> lc_time_discount_plf({0.0, 2.0},
                                                               {0.4, 1.0});
    return agent_reaction_time_plf(spacetime_obj.pose().a()) *
           lc_time_discount_plf(overlap_meta.time_to_lc_complete());
  }

  return agent_reaction_time_plf(spacetime_obj.pose().a());
}

// Calculate decel by uniform deceleration model.
// Ref: https://qcraft.feishu.cn/docs/doccnVVzu8CQXBkxsHzGpNVJOdb
std::optional<YieldingResult> CalculateYieldingResultIfAgentYieldAV(
    const SpacetimeObjectTrajectory& spacetime_obj,
    const OverlapInfo& overlap_info, const DiscretizedPath& path,
    const SpeedVector& speed_profile, StBoundaryProto::ObjectType object_type,
    double agent_reaction_time, bool enable_vlog = true) {
  YieldingResult yielding_result;
  // AV infos.
  // Use mid idx of av_start_idx and av_end_idx.
  const int overlap_av_idx =
      (overlap_info.av_start_idx + overlap_info.av_end_idx) / 2;
  QCHECK_GT(path.size(), overlap_av_idx);
  const auto& overlap_av_path_point = path[overlap_av_idx];
  if (overlap_av_path_point.s() > speed_profile.back().s()) {
    VLOG_IF(2, enable_vlog)
        << "AV cannot reach overlap point with this speed profile.";
    return std::nullopt;
  }
  const auto av_speed_point =
      speed_profile.EvaluateByS(overlap_av_path_point.s());
  QCHECK(av_speed_point.has_value());
  const double overlap_av_v = av_speed_point->v();

  // Agent infos.
  const int overlap_agent_idx = overlap_info.obj_idx;
  QCHECK_GT(spacetime_obj.states().size(), overlap_agent_idx);
  const auto* overlap_agent_traj_point =
      spacetime_obj.states()[overlap_agent_idx].traj_point;
  const double agent_overlap_s = overlap_agent_traj_point->s();
  const double overlap_agent_v = overlap_agent_traj_point->v();
  const double agent_v = spacetime_obj.pose().v();
  const double agent_a = spacetime_obj.pose().a();
  const double overlap_agent_signed_acc_v =
      overlap_av_v * fast_math::Cos(overlap_av_path_point.theta() -
                                    overlap_agent_traj_point->theta());
  const double overlap_agent_acc_v = std::max(0.0, overlap_agent_signed_acc_v);

  if (agent_v < kEps) {
    VLOG_IF(2, enable_vlog) << "Agent pose is stationary.";
    return std::nullopt;
  }

  // Calculate agent follow distance.
  double agent_standoff_distance = 0.0;  // m.
  // T = f(r).
  PiecewiseLinearFunction<double> agent_standoff_time_plf;
  if (object_type == StBoundaryProto::CYCLIST) {
    constexpr double kCyclistStandoffDistance = 2.0;  // m.
    const PiecewiseLinearFunction<double> cyclist_standoff_time_plf(
        {M_PI / 4.0, M_PI / 2.0, M_PI * 2.0 / 3.0, M_PI * 5.0 / 6.0, M_PI},
        {0.2, 0.4, 0.6, 0.8, 1.3});
    agent_standoff_distance = kCyclistStandoffDistance;
    agent_standoff_time_plf = cyclist_standoff_time_plf;
  } else {
    // For vehicle.
    constexpr double kVehicleStandoffDistance = 3.0;  // m.
    const PiecewiseLinearFunction<double> vehicle_standoff_time_plf(
        {M_PI / 4.0, M_PI / 2.0, M_PI * 2.0 / 3.0, M_PI * 5.0 / 6.0, M_PI},
        {0.2, 0.5, 0.7, 1.0, 1.5});
    agent_standoff_distance = kVehicleStandoffDistance;
    agent_standoff_time_plf = vehicle_standoff_time_plf;
  }
  const double heading_diff = NormalizeAngle(overlap_av_path_point.theta() -
                                             overlap_agent_traj_point->theta());
  const double kMaxTemporalFollowDistance = 5.0;  // m.
  const double temporal_follow_distance =
      std::min(std::abs(overlap_agent_signed_acc_v) *
                   agent_standoff_time_plf(std::abs(heading_diff)),
               kMaxTemporalFollowDistance);
  const double agent_follow_distance =
      agent_standoff_distance + temporal_follow_distance;

  // Check if agent already yield av.
  // TODO(yumeng): Add a QueryTrajectoryPointByT function to find more precise
  // point.
  const auto it = std::lower_bound(
      spacetime_obj.states().begin(), spacetime_obj.states().end(),
      av_speed_point->t(),
      [](const auto& state, double t) { return state.traj_point->t() < t; });
  if (it != spacetime_obj.states().end()) {
    const double agent_pred_s_at_yielding_point = it->traj_point->s();
    const double agent_pred_v_at_yielding_point = it->traj_point->v();
    const double agent_yield_s = agent_overlap_s - agent_follow_distance;
    if (agent_pred_s_at_yielding_point <= agent_yield_s &&
        agent_pred_v_at_yielding_point <= overlap_agent_acc_v) {
      yielding_result.already_yield = true;
      VLOG_IF(2, enable_vlog)
          << "Current st traj already yield this speed profile, no need to "
             "modify.";
      return yielding_result;
    }
  }

  double agent_start_yield_v = 0.0;
  double agent_reaction_s = 0.0;
  for (const auto& state : spacetime_obj.states()) {
    if (state.traj_point->t() >= agent_reaction_time) {
      agent_start_yield_v = state.traj_point->v();
      agent_reaction_s = state.traj_point->s();
      break;
    }
  }

  const double agent_decel_dist =
      agent_overlap_s - agent_reaction_s - agent_follow_distance;
  // No enough distance to brake.
  if (agent_decel_dist <= 0.0) {
    VLOG_IF(2, enable_vlog)
        << "agent_v: " << agent_v << ", agent_a: " << agent_a
        << ", overlap_av_v:" << overlap_av_v
        << ", overlap_agent_acc_v: " << overlap_agent_acc_v
        << ", overlap_av_s:" << overlap_av_path_point.s()
        << ", agent_overlap_s: " << agent_overlap_s
        << ", agent_reaction_s: " << agent_reaction_s
        << ", agent_follow_distance:" << agent_follow_distance << ansi::yellow
        << ", agent_decel_dist: " << agent_decel_dist << ansi::reset;
    VLOG_IF(2, enable_vlog) << "Agent has no enough distance to brake.";
    return std::nullopt;
  }
  const double agent_acc_decel =
      (Sqr(overlap_agent_acc_v) - Sqr(agent_start_yield_v)) / agent_decel_dist *
      0.5;

  const double agent_braking_to_stop_decel =
      -Sqr(agent_start_yield_v) / agent_decel_dist * 0.5;
  // Agent braking to stop time.
  const double t_stop = -agent_start_yield_v / agent_braking_to_stop_decel;
  // Agent decel to `overlap_agent_acc_v` time.
  const double t_decel_to_acc =
      (overlap_agent_acc_v - agent_start_yield_v) / agent_acc_decel;
  // AV arrive overlap point time, that is, agent should finish yielding within
  // this time.
  const double t_yield = av_speed_point->t() - agent_reaction_time;

  double agent_arrive_yielding_point_decel = 0.0;
  double agent_finish_yield_decel = 0.0;
  double agent_v_at_yielding_point = 0.0;
  double agent_arrive_yielding_point_time = 0.0;
  double agent_waiting_duration = 0.0;
  if (t_yield <= t_decel_to_acc) {
    // Agent drive to `overlap_agent_acc_v` at yielding point to yield av.
    // Agent will not arrive yielding point if `t_yield < t_decel_to_acc`, or
    // `overlap_agent_v < overlap_agent_acc_v` from another perspective. But we
    // still use this yielding process because agent already yield av and there
    // is no need to assume agent will accelerate to `overlap_agent_acc_v`.
    agent_arrive_yielding_point_decel = agent_acc_decel;
    agent_finish_yield_decel = agent_acc_decel;
    agent_arrive_yielding_point_time = agent_reaction_time + t_decel_to_acc;
    agent_v_at_yielding_point = overlap_agent_acc_v;
    VLOG_IF(2, enable_vlog) << ansi::green << "[acc]:" << ansi::reset;
  } else if (t_yield > t_decel_to_acc && t_yield < t_stop) {
    // To ensure agent not exceed yielding point, agent speed at yielding point
    // should slower than `overlap_agent_acc_v` but not braking to stop, now
    // the speed dictated by `t_yield`.
    agent_arrive_yielding_point_decel =
        2.0 * (agent_decel_dist - agent_start_yield_v * t_yield) / Sqr(t_yield);
    agent_finish_yield_decel = agent_arrive_yielding_point_decel;
    agent_arrive_yielding_point_time = agent_reaction_time + t_yield;
    agent_v_at_yielding_point =
        agent_start_yield_v + agent_arrive_yielding_point_decel * t_yield;
    VLOG_IF(2, enable_vlog) << ansi::green << "[slower acc]:" << ansi::reset;
  } else {
    // Braking to stop and waiting.
    agent_arrive_yielding_point_decel = agent_braking_to_stop_decel;
    agent_finish_yield_decel = 0.0;
    agent_v_at_yielding_point = 0.0;
    agent_arrive_yielding_point_time = agent_reaction_time + t_stop;
    agent_waiting_duration = t_yield - t_stop;
    VLOG_IF(2, enable_vlog)
        << ansi::green << "[stop to waiting]:" << ansi::reset;
  }
  const double agent_s_at_yielding_point =
      agent_overlap_s - agent_follow_distance;

  yielding_result.start_yield_time = agent_reaction_time;
  yielding_result.arrive_yielding_point_decel =
      agent_arrive_yielding_point_decel;
  yielding_result.arrive_yielding_point_time = agent_arrive_yielding_point_time;
  yielding_result.waiting_duration = agent_waiting_duration;
  yielding_result.finish_yield_time =
      agent_arrive_yielding_point_time + agent_waiting_duration;
  yielding_result.v_at_yielding_point = agent_v_at_yielding_point;
  yielding_result.s_at_yielding_point = agent_s_at_yielding_point;

  VLOG_IF(2, enable_vlog) << "agent_v: " << agent_v << ", agent_a: " << agent_a
                          << ", agent_start_yield_v: " << agent_start_yield_v
                          << ", overlap_av_v: " << overlap_av_v
                          << ", overlap_agent_v: " << overlap_agent_v
                          << ", overlap_agent_acc_v: " << overlap_agent_acc_v
                          << "\nagent_overlap_s:" << agent_overlap_s
                          << ", agent_reaction_s:" << agent_reaction_s
                          << ", agent_follow_distance: "
                          << agent_follow_distance << ", " << ansi::yellow
                          << "agent_decel_dist: " << agent_decel_dist
                          << ansi::reset
                          << "\nagent_acc_decel: " << agent_acc_decel
                          << ", agent_braking_to_stop_decel: "
                          << agent_braking_to_stop_decel;
  VLOG_IF(2, enable_vlog)
      << "t_yield: " << t_yield << ", t_decel_to_acc: " << t_decel_to_acc
      << ", t_stop: " << t_stop << ansi::yellow
      << "\nagent_arrive_yielding_point_decel: "
      << agent_arrive_yielding_point_decel << ansi::reset
      << ", agent_finish_yield_decel: " << agent_finish_yield_decel
      << ", agent_v_at_yielding_point: " << agent_v_at_yielding_point
      << ", agent_arrive_yielding_point_time: "
      << agent_arrive_yielding_point_time
      << ", agent_waiting_duration: " << agent_waiting_duration;

  return yielding_result;
}

double CalculateAgentDecelDiscomfort(
    StOverlapMetaProto::OverlapPriority av_priority, double decel) {
  switch (av_priority) {
    case StOverlapMetaProto::HIGH: {
      return low_prio_decel_discomfort_plf(decel);
    }
    case StOverlapMetaProto::UNKNOWN_PRIORITY:
    case StOverlapMetaProto::EQUAL: {
      return equal_prio_decel_discomfort_plf(decel);
    }
    case StOverlapMetaProto::LOW: {
      return high_prio_decel_discomfort_plf(decel);
    }
  }
}

double CalculateAgentWaitingDiscomfort(
    StOverlapMetaProto::OverlapPriority av_priority, double waiting_duration) {
  if (waiting_duration <= 0.0) return 0.0;
  // If we have no right of way, not assume agent will braking to stop to
  // waiting us.
  if (av_priority != StOverlapMetaProto::HIGH) return kInf;

  return agent_waiting_discomfort_plf(waiting_duration);
}

// Calculate speed profiles interactive cost, select the minimum cost speed
// profile and modify st boundaries based on interactive assumption. For more
// details, please refer to
// (https://qcraft.feishu.cn/docs/doccnzcGKxSY163fpCOEcgHPuvc)
std::vector<InteractiveResult> CalculateInteractiveResults(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const StBoundaryWithDecision> interactive_st_boundaries) {
  SCOPED_QTRACE("CalculateInteractiveResults");

  std::vector<InteractiveResult> interactive_results;
  interactive_results.reserve(interactive_st_boundaries.size());
  for (const auto& boundary_with_decision : interactive_st_boundaries) {
    const auto traj_id = boundary_with_decision.traj_id();
    QCHECK(traj_id.has_value());
    const auto* st_boundary = boundary_with_decision.st_boundary();
    QCHECK(st_boundary->overlap_meta().has_value());
    const auto av_priority = st_boundary->overlap_meta()->priority();

    VLOG(2) << ansi::magenta << "st-boundary: " << boundary_with_decision.id()
            << ", source: "
            << StOverlapMetaProto::OverlapSource_Name(
                   st_boundary->overlap_meta()->source())
            << ", av_priority: "
            << StOverlapMetaProto::OverlapPriority_Name(av_priority)
            << ansi::reset;

    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    QCHECK_NOTNULL(spacetime_obj);
    // TODO(yumeng): Get modification type from new st_boundaries_meta and apply
    // here.
    const auto& first_overlap_info = st_boundary->overlap_infos().front();
    const double agent_reaction_time = CalculateAgentReactionTime(
        *st_boundary->overlap_meta(), *spacetime_obj);
    auto yielding_result = CalculateYieldingResultIfAgentYieldAV(
        *spacetime_obj, first_overlap_info, path, speed_profile,
        st_boundary->object_type(), agent_reaction_time);

    if (!yielding_result.has_value()) {
      interactive_results.push_back(
          {.cost = kInf,
           .traj_id = *traj_id,
           .decision = boundary_with_decision.decision_type(),
           .spacetime_obj = spacetime_obj,
           .st_boundary = st_boundary});
      // Currently, if one of interactive obj yield failed, current
      // solution failed. So use break here. In the future, should try partial
      // success.
      break;
    }
    if (yielding_result->already_yield) {
      interactive_results.push_back(
          {.cost = 0.0,
           .yielding_result = std::move(*yielding_result),
           .traj_id = *traj_id,
           .decision = StBoundaryProto::LEAD,
           .spacetime_obj = spacetime_obj,
           .st_boundary = st_boundary});
      continue;
    }

    const double agent_decel_discomfort = CalculateAgentDecelDiscomfort(
        av_priority, yielding_result->arrive_yielding_point_decel);
    const double agent_waiting_discomfort = CalculateAgentWaitingDiscomfort(
        av_priority, yielding_result->waiting_duration);

    interactive_results.push_back(
        {.cost = agent_decel_discomfort + agent_waiting_discomfort,
         .yielding_result = std::move(*yielding_result),
         .traj_id = *traj_id,
         .decision = StBoundaryProto::LEAD,
         .spacetime_obj = spacetime_obj,
         .st_boundary = st_boundary});

    VLOG(2) << "agent_decel_discomfort: " << ansi::red << agent_decel_discomfort
            << ansi::reset << ", agent_waiting_discomfort: " << ansi::red
            << agent_waiting_discomfort << ansi::reset;
  }

  return interactive_results;
}

void GenerateInteractiveStBoundaryModificationInfo(
    const VehicleGeometryParamsProto& vehicle_geom, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const InteractiveResult> interactive_results,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info) {
  for (const auto& interactive_result : interactive_results) {
    if (!interactive_result.yielding_result.has_value()) {
      continue;
    }

    auto accel_point_list = CalculateAgentAccelPointList(
        vehicle_geom, *interactive_result.spacetime_obj,
        interactive_result.st_boundary->overlap_infos().back(), path,
        speed_profile, *interactive_result.yielding_result);
    // NOTE(yumeng): Remove this check if 'CalculateAgentAccelPointList' will
    // generate a legal empty accel point list and we want to only update
    // decision by post st-boundary modifier.
    QCHECK(!accel_point_list.empty());
    (*modification_info)[interactive_result.traj_id] = {
        .modifier_type = StBoundaryModifierProto::INTERACTIVE,
        .decision = interactive_result.decision,
        .is_decision_changed = true,
        .accel_point_list = std::move(accel_point_list)};
  }
}

absl::StatusOr<SpeedVector> SelectBestInteractivePreliminarySpeed(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    absl::Span<const PreliminarySpeedWithCost> candidate_speed_profiles,
    absl::Span<const StBoundaryWithDecision> interactive_st_boundaries,
    std::vector<InteractiveResult>* final_interactive_results) {
  SCOPED_QTRACE("SelectBestInteractivePreliminarySpeed");

  double min_cost = kInf;
  std::optional<int> min_cost_index = std::nullopt;
  for (int i = 0; i < candidate_speed_profiles.size(); ++i) {
    VLOG(2) << ansi::green << "------------------ " << i << "-th"
            << " ------------------" << ansi::reset;
    auto interactive_results = CalculateInteractiveResults(
        st_traj_mgr, path, candidate_speed_profiles[i].preliminary_speed,
        interactive_st_boundaries);
    // The cost of speed profile itself. That is, sampling dp cost.
    double cost = candidate_speed_profiles[i].cost;
    for (const auto& result : interactive_results) {
      cost += result.cost;
    }
    cost +=
        CalculateConsistencyCost(interactive_st_boundaries,
                                 candidate_speed_profiles[i].preliminary_speed);

    // TODO(yumeng): Use cost thershold as filter.
    if (std::isinf(cost)) {
      continue;
    }
    // Select min cost speed profile.
    if (cost < min_cost) {
      min_cost = cost;
      min_cost_index = i;
      *final_interactive_results = std::move(interactive_results);
    }
  }

  VLOG(2) << ansi::green << "-------------------------------------------"
          << ansi::reset;

  if (!min_cost_index.has_value()) {
    QEVENT_EVERY_N_SECONDS("yumeng", "interactive-speed-decision-failed", 3.0,
                           [&](QEvent* qevent) {});
    return absl::NotFoundError("No interactive speed profile found.");
  }

  QEVENT_EVERY_N_SECONDS("yumeng", "interactive-speed-decision-success", 3.0,
                         [&](QEvent* qevent) {});
  VLOG(2) << ansi::bg_green << "Select" << ansi::reset << " " << ansi::green
          << *min_cost_index << "-th speed profile." << ansi::reset;

  return candidate_speed_profiles[*min_cost_index].preliminary_speed;
}

absl::Status GetPreliminarySpeedBySamplingDp(
    const SpeedLimit& speed_limit, double path_length,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps, PreliminarySpeedWithCost* preliminary_speed_with_cost,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    SamplingDpDebugProto* sampling_dp_debug, ThreadPool* thread_pool) {
  SCOPED_QTRACE("GetPreliminarySpeedBySamplingDp");
  QCHECK_NOTNULL(preliminary_speed_with_cost);
  QCHECK_NOTNULL(st_boundaries_with_decision);

  StGraphData st_graph_data(speed_limit, speed_cap, path_length,
                            traj_steps * kTrajectoryTimeStep);
  TrajectoryPoint init_point;
  init_point.set_s(0.0);
  init_point.set_t(0.0);
  init_point.set_v(plan_start_point.v());
  init_point.set_a(plan_start_point.a());
  init_point.set_j(plan_start_point.j());
  GriddedSvtGraph gridded_svt_graph(&st_graph_data, init_point,
                                    &speed_finder_params, speed_cap,
                                    std::move(*st_boundaries_with_decision));
  RETURN_IF_ERROR(gridded_svt_graph.FindOptimalPreliminarySpeedWithCost(
      preliminary_speed_with_cost, sampling_dp_debug, thread_pool));
  gridded_svt_graph.SwapStBoundariesWithDecision(st_boundaries_with_decision);

  return absl::OkStatus();
}

absl::Status GenerateSpeedProfileCandidateSet(
    const SpeedLimit& speed_limit, double path_length,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps,
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    SamplingDpDebugProto* sampling_dp_debug,
    InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("GenerateSpeedProfileCandidateSet");
  QCHECK_NOTNULL(candidate_speed_profiles);
  QCHECK_NOTNULL(st_boundaries_with_decision);

  StGraphData st_graph_data(speed_limit, speed_cap, path_length,
                            traj_steps * kTrajectoryTimeStep);
  TrajectoryPoint init_point;
  init_point.set_s(0.0);
  init_point.set_t(0.0);
  init_point.set_v(plan_start_point.v());
  init_point.set_a(plan_start_point.a());
  init_point.set_j(plan_start_point.j());
  GriddedSvtGraph gridded_svt_graph(&st_graph_data, init_point,
                                    &speed_finder_params, speed_cap,
                                    std::move(*st_boundaries_with_decision));
  RETURN_IF_ERROR(gridded_svt_graph.GenerateSamplingDpSpeedProfileCandidateSet(
      candidate_speed_profiles, sampling_dp_debug, candidate_set_debug,
      thread_pool));
  // TODO(yumeng): Filter candidate_speed_profiles by decision.
  gridded_svt_graph.SwapStBoundariesWithDecision(st_boundaries_with_decision);

  return absl::OkStatus();
}

void FilterCandidateSpeedProfiles(
    const DiscretizedPath& path,
    absl::Span<const StBoundaryWithDecision> interactive_st_boundaries,
    absl::Span<const StBoundaryWithDecision> non_interactive_st_boundaries,
    const SpeedVector& lower_bound_speed,
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles) {
  QCHECK_GE(lower_bound_speed.size(), 2);

  std::vector<PreliminarySpeedWithCost> filtered_speed_profiles;
  filtered_speed_profiles.reserve(candidate_speed_profiles->size());

  double first_stopline_s = kInf;
  for (const auto& non_inter_st_boundary : non_interactive_st_boundaries) {
    const auto* st_boundary = non_inter_st_boundary.st_boundary();
    if (st_boundary->source_type() == StBoundary::SourceType::VIRTUAL) {
      first_stopline_s = std::min(first_stopline_s, st_boundary->min_s());
    }
  }

  double min_interactive_s = kInf;
  for (const auto& interactive_st_boundary : interactive_st_boundaries) {
    const int av_start_idx = interactive_st_boundary.st_boundary()
                                 ->overlap_infos()
                                 .front()
                                 .av_start_idx;
    QCHECK_GT(path.size(), av_start_idx);
    min_interactive_s = std::min(min_interactive_s, path[av_start_idx].s());
  }

  const double lower_bound_speed_t_step =
      lower_bound_speed[1].t() - lower_bound_speed[0].t();
  for (auto& speed_profile : *candidate_speed_profiles) {
    const auto& preliminary_speed = speed_profile.preliminary_speed;
    QCHECK_GE(preliminary_speed.size(), 2);

    // Filter by stopline.
    if (preliminary_speed.back().s() > first_stopline_s) {
      continue;
    }

    // Filter by min interactive s.
    if (preliminary_speed.back().s() < min_interactive_s) {
      continue;
    }

    // Filter by speed profile lower bound, that is, non-interactive speed
    // profile.
    const double t_step = preliminary_speed[1].t() - preliminary_speed[0].t();
    // If t step is not same, not filter, `EvaluateByTime` is too expensive to
    // do this.
    if (std::abs(t_step - lower_bound_speed_t_step) > kEps) {
      filtered_speed_profiles.push_back(std::move(speed_profile));
      continue;
    }
    const int size =
        std::min(preliminary_speed.size(), lower_bound_speed.size());
    for (int i = 0; i < size; ++i) {
      if (lower_bound_speed[i].s() < preliminary_speed[i].s()) {
        filtered_speed_profiles.push_back(std::move(speed_profile));
        break;
      }
    }
  }

  candidate_speed_profiles->swap(filtered_speed_profiles);
}

absl::flat_hash_set<std::string> PreFilterNonInteractiveStBoundries(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const PathPoint& plan_start_path_point) {
  absl::flat_hash_set<std::string> non_interactive_ids;

  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    VLOG(2) << "-- Pre-filter: " << boundary_with_decision.id() << " --";

    if (!boundary_with_decision.st_boundary()->overlap_meta().has_value()) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "No overlap meta data.";
      continue;
    }
    const auto& overlap_meta =
        *boundary_with_decision.st_boundary()->overlap_meta();

    VLOG(2) << "Object type: "
            << StBoundaryProto::ObjectType_Name(
                   boundary_with_decision.st_boundary()->object_type())
            << ", overlap pattern: "
            << StOverlapMetaProto::OverlapPattern_Name(overlap_meta.pattern())
            << ", overlap source: "
            << StOverlapMetaProto::OverlapSource_Name(overlap_meta.source())
            << ", modification type: "
            << StOverlapMetaProto::ModificationType_Name(
                   overlap_meta.modification_type())
            << ", priority: "
            << StOverlapMetaProto::OverlapPriority_Name(
                   overlap_meta.priority());

    // Filter by modification type.
    switch (overlap_meta.modification_type()) {
      case StOverlapMetaProto::NON_MODIFIABLE:
      case StOverlapMetaProto::LON_LAT_MODIFIABLE: {
        non_interactive_ids.insert(boundary_with_decision.id());
        VLOG(2) << "Not considered modification type.";
        continue;
      }
      case StOverlapMetaProto::LON_MODIFIABLE:
        break;
    }

    if (boundary_with_decision.decision_type() != StBoundaryProto::UNKNOWN) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "Pre-decision is: "
              << StBoundaryProto::DecisionType_Name(
                     boundary_with_decision.decision_type());
      continue;
    }

    // Currently, just modify once.
    if (boundary_with_decision.traj_id().has_value() &&
        processed_st_objects.contains(*boundary_with_decision.traj_id())) {
      non_interactive_ids.insert(boundary_with_decision.id());
      VLOG(2) << "Modified by others.";
      continue;
    }

    // If av out of agent view when av cutin, not interactive with agent.
    if (overlap_meta.source() == StOverlapMetaProto::AV_CUTIN &&
        boundary_with_decision.traj_id().has_value()) {
      const auto* spacetime_obj =
          st_traj_mgr.FindTrajectoryById(*boundary_with_decision.traj_id());
      const auto pos = spacetime_obj->pose().pos();
      const auto heading_perp =
          Vec2d::FastUnitFromAngle(spacetime_obj->pose().theta() - M_PI * 0.5);
      const HalfPlane agent_view(pos - heading_perp, pos + heading_perp);
      const auto av_box =
          GetAvBox(Vec2d(plan_start_path_point.x(), plan_start_path_point.y()),
                   plan_start_path_point.theta(), vehicle_geom);
      bool is_av_inside_agent_view = false;
      for (const auto& corner : av_box.GetCornersCounterClockwise()) {
        if (agent_view.IsPointInside(corner)) {
          is_av_inside_agent_view = true;
          break;
        }
      }
      if (!is_av_inside_agent_view) {
        non_interactive_ids.insert(boundary_with_decision.id());
        VLOG(2) << "Out of agent view when av cutin.";
        continue;
      }
    }
  }

  return non_interactive_ids;
}

void PostFilterNonInteractiveStBoundries(
    const SpeedProfile& empty_road_speed_profile,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    absl::flat_hash_set<std::string>* non_interactive_ids) {
  constexpr double kConsiderAsFrontAgentTimeBuffer = 1.0;      // s.
  constexpr double kAVTimeLaterThres = 1.5;                    // s.
  constexpr double kConsiderAsInteractiveTimeThreshold = 4.0;  // s.

  // Use to filter multi st-boundaries.
  absl::flat_hash_map<std::string, const StBoundaryWithDecision*> traj_id_map;
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    // Already added.
    if (non_interactive_ids->find(boundary_with_decision.id()) !=
        non_interactive_ids->end()) {
      continue;
    }

    VLOG(2) << "---- Post-filter: " << boundary_with_decision.id() << " ----";

    // Add LEAD st boundary to non-interactive set.
    if (boundary_with_decision.decision_type() == StBoundaryProto::LEAD) {
      non_interactive_ids->insert(boundary_with_decision.id());
      VLOG(2) << "Already LEAD.";
      continue;
    }

    // If AV need a long time to arrive first overlap point, add to
    // non-interactive set.
    const double min_time_to_overlap = empty_road_speed_profile.GetTimeAtS(
        boundary_with_decision.st_boundary()->bottom_left_point().s());
    if (min_time_to_overlap > kConsiderAsInteractiveTimeThreshold) {
      non_interactive_ids->insert(boundary_with_decision.id());
      VLOG(2) << "Long time to overlap: " << min_time_to_overlap;
      continue;
    }

    // If the agent has or is about to reach first overlap but av need longer
    // time to reach, add to non-interactive set.
    if (boundary_with_decision.decision_type() == StBoundaryProto::FOLLOW) {
      const double real_min_t =
          boundary_with_decision.raw_st_boundary()->min_t();
      const bool is_agent_near_overlap =
          real_min_t < kConsiderAsFrontAgentTimeBuffer;
      const bool is_av_need_longer_time_to_reach_overlap =
          min_time_to_overlap - real_min_t > kAVTimeLaterThres;
      if (is_agent_near_overlap && is_av_need_longer_time_to_reach_overlap) {
        non_interactive_ids->insert(boundary_with_decision.id());
        VLOG(2) << "Is front agent, real_min_t:" << real_min_t
                << ", min_time_to_overlap" << min_time_to_overlap;
        continue;
      }
    }

    // If st-traj has multi st-boundaries, keep min time st-boundary as
    // interactive st-boundary (other st-boundaries will be ignored when
    // modifying the st-traj).
    if (boundary_with_decision.traj_id().has_value()) {
      const auto traj_id = *boundary_with_decision.traj_id();
      const auto it = traj_id_map.find(traj_id);
      if (it != traj_id_map.end() &&
          non_interactive_ids->find(it->second->id()) ==
              non_interactive_ids->end()) {
        if (boundary_with_decision.st_boundary()->min_t() >
            it->second->st_boundary()->min_t()) {
          non_interactive_ids->insert(boundary_with_decision.id());
          VLOG(2) << "Less influential multi st-boundary: "
                  << boundary_with_decision.id();
          continue;
        } else {
          non_interactive_ids->insert(it->second->id());
          VLOG(2) << "Less influential multi st-boundary: " << it->second->id();
        }
      }
      traj_id_map[traj_id] = &boundary_with_decision;
    }
  }
}

void GenerateLeadStBoundaryModificationInfo(
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr, const DiscretizedPath& path,
    const SpeedVector& speed_profile,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info) {
  QCHECK_NOTNULL(modification_info);

  const bool enable_vlog = false;
  VLOG_IF(2, enable_vlog) << ansi::blue << "---------process lead---------"
                          << ansi::reset;

  // Use to filter multi st-boundaries.
  absl::flat_hash_map<std::string, double> min_t_map;
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (boundary_with_decision.decision_type() != StBoundaryProto::LEAD) {
      continue;
    }

    const auto traj_id = boundary_with_decision.traj_id();
    if (!traj_id.has_value()) continue;

    // Only modify once.
    if (processed_st_objects.contains(*traj_id)) {
      continue;
    }

    VLOG_IF(2, enable_vlog) << "traj_id: " << *traj_id;
    const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
    const auto* st_boundary = boundary_with_decision.st_boundary();
    const auto& overlap_infos = st_boundary->overlap_infos();
    if (!st_boundary->overlap_meta().has_value()) {
      continue;
    }
    const double agent_reaction_time = CalculateAgentReactionTime(
        *st_boundary->overlap_meta(), *spacetime_obj);

    const auto yielding_result = CalculateYieldingResultIfAgentYieldAV(
        *spacetime_obj, overlap_infos.front(), path, speed_profile,
        st_boundary->object_type(), agent_reaction_time, enable_vlog);
    if (!yielding_result.has_value()) {
      continue;
    }

    // If st-traj has multi st-boundaries, use min time st-boundary to calculate
    // modification info (other st-boundaries will be ignored when modifying the
    // st-traj).
    const auto it = min_t_map.find(*traj_id);
    if (it != min_t_map.end() &&
        boundary_with_decision.st_boundary()->min_t() > it->second) {
      continue;
    }
    min_t_map[*traj_id] = boundary_with_decision.st_boundary()->min_t();

    auto accel_point_list = CalculateAgentAccelPointList(
        vehicle_geom, *spacetime_obj, overlap_infos.back(), path, speed_profile,
        *yielding_result);
    // NOTE(yumeng): Remove this check if 'CalculateAgentAccelPointList' will
    // generate a legal empty accel point list and we want to only update
    // decision by post st-boundary modifier.
    QCHECK(!accel_point_list.empty());
    (*modification_info)[*traj_id] = {
        .modifier_type = StBoundaryModifierProto::LEADING,
        .decision = boundary_with_decision.decision_type(),
        .is_decision_changed = false,
        .accel_point_list = std::move(accel_point_list)};
  }
}

absl::Status GenerateInteractiveSpeedProfile(
    const VehicleGeometryParamsProto& vehicle_geom,
    const MotionConstraintParamsProto& motion_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpeedLimit& speed_limit, const DiscretizedPath& path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    const absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    int traj_steps, SpeedVector* preliminary_speed,
    absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
        modification_info,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("GenerateInteractiveSpeedProfile");
  QCHECK_NOTNULL(preliminary_speed);
  QCHECK_NOTNULL(modification_info);
  QCHECK_NOTNULL(st_boundaries_with_decision);
  QCHECK_NOTNULL(interactive_speed_debug);

  VLOG(2) << ansi::yellow << "-----------interactive-decision-----------"
          << ansi::reset;

  const auto start_time = absl::Now();
  if (FLAGS_enable_interactive_speed_decision_draw_st_traj) {
    ClearCanvasServerBuffers();
  }

  absl::flat_hash_set<std::string> non_interactive_ids;
  if (speed_finder_params.enable_interactive_speed_decision()) {
    non_interactive_ids = PreFilterNonInteractiveStBoundries(
        vehicle_geom, st_traj_mgr, *st_boundaries_with_decision,
        processed_st_objects, plan_start_point.path_point());
  }

  // Firstly search to make non-interactive decision.
  PreliminarySpeedWithCost non_interactive_preliminary_speed;
  RETURN_IF_ERROR(GetPreliminarySpeedBySamplingDp(
      speed_limit, path.length(), plan_start_point, speed_finder_params,
      speed_cap, traj_steps, &non_interactive_preliminary_speed,
      st_boundaries_with_decision,
      interactive_speed_debug->mutable_non_interactive_sampling_dp(),
      thread_pool));
  if (FLAGS_planner_add_optimal_preliminary_speed_to_debug) {
    non_interactive_preliminary_speed.preliminary_speed.ToProto(
        interactive_speed_debug->mutable_non_interactive_speed_profile());
  }

  // Currently, not apply interactive speed decision for fallback planner,
  if (!speed_finder_params.enable_interactive_speed_decision()) {
    *preliminary_speed =
        std::move(non_interactive_preliminary_speed.preliminary_speed);
    return absl::OkStatus();
  }

  const auto empty_road_speed_profile =
      CreateEmptyRoadSpeedProfile(motion_constraint_params, path, speed_limit,
                                  plan_start_point.v(), traj_steps);
  PostFilterNonInteractiveStBoundries(empty_road_speed_profile,
                                      *st_boundaries_with_decision,
                                      &non_interactive_ids);

  if (non_interactive_ids.size() == st_boundaries_with_decision->size()) {
    // No interactive st boundary found.
    *preliminary_speed =
        std::move(non_interactive_preliminary_speed.preliminary_speed);
    VLOG(2) << "No interactive st boundary found.";
    return absl::OkStatus();
  }

  // Separate interactive and non-interactive st boundaries.
  std::vector<StBoundaryWithDecision> non_interactive_st_boundaries;
  non_interactive_st_boundaries.reserve(non_interactive_ids.size());
  std::vector<StBoundaryWithDecision> interactive_st_boundaries;
  interactive_st_boundaries.reserve(st_boundaries_with_decision->size() -
                                    non_interactive_ids.size());
  for (auto& st_boundary : *st_boundaries_with_decision) {
    if (non_interactive_ids.find(st_boundary.id()) !=
        non_interactive_ids.end()) {
      non_interactive_st_boundaries.push_back(std::move(st_boundary));
    } else {
      interactive_st_boundaries.push_back(std::move(st_boundary));
    }
  }
  // `st_boundaries_with_decision` now contains unspecified values, clear it.
  st_boundaries_with_decision->clear();

  // Get trajectory candidate set based on non-interactive st boundaries.
  std::vector<PreliminarySpeedWithCost> candidate_speed_profiles;
  RETURN_IF_ERROR(GenerateSpeedProfileCandidateSet(
      speed_limit, path.length(), plan_start_point, speed_finder_params,
      speed_cap, traj_steps, &candidate_speed_profiles,
      &non_interactive_st_boundaries,
      interactive_speed_debug->mutable_interactive_sampling_dp(),
      interactive_speed_debug->mutable_candidate_set(), thread_pool));

  FilterCandidateSpeedProfiles(
      path, interactive_st_boundaries, non_interactive_st_boundaries,
      non_interactive_preliminary_speed.preliminary_speed,
      &candidate_speed_profiles);

  if (candidate_speed_profiles.empty()) {
    *preliminary_speed =
        std::move(non_interactive_preliminary_speed.preliminary_speed);
    std::move(interactive_st_boundaries.begin(),
              interactive_st_boundaries.end(),
              std::back_inserter(non_interactive_st_boundaries));
    st_boundaries_with_decision->swap(non_interactive_st_boundaries);
    VLOG(2) << "No candidate speed profile found.";
    return absl::OkStatus();
  }

  if (FLAGS_planner_add_optimal_preliminary_speed_to_debug) {
    const auto it = std::min_element(
        candidate_speed_profiles.begin(), candidate_speed_profiles.end(),
        [](const auto& sp1, const auto& sp2) { return sp1.cost < sp2.cost; });
    it->preliminary_speed.ToProto(
        interactive_speed_debug
            ->mutable_optimal_speed_profile_in_candidate_set());
  }
  candidate_speed_profiles.push_back(non_interactive_preliminary_speed);

  std::vector<InteractiveResult> final_interactive_results;
  // Output interactive speed profile and modified st boundaries.
  const auto result = SelectBestInteractivePreliminarySpeed(
      st_traj_mgr, path, candidate_speed_profiles, interactive_st_boundaries,
      &final_interactive_results);

  if (result.ok()) {
    // TODO(yumeng): Validate the feasibility of this speed profile. If not,
    // choose non interactive preliminary speed.
    *preliminary_speed = std::move(*result);
    GenerateInteractiveStBoundaryModificationInfo(
        vehicle_geom, path, *preliminary_speed, final_interactive_results,
        modification_info);
  } else {
    // Use non interactive preliminary speed profile.
    *preliminary_speed =
        std::move(non_interactive_preliminary_speed.preliminary_speed);
    VLOG(2) << ansi::yellow << "Interactive speed decision failed: "
            << result.status().message() << ansi::reset;

    // Draw origin st traj.
    if (FLAGS_enable_interactive_speed_decision_draw_st_traj) {
      for (const auto& boundary_with_decision : interactive_st_boundaries) {
        const auto traj_id = boundary_with_decision.traj_id();
        QCHECK(traj_id.has_value());
        const auto* spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
        DrawSpacetimeObjectTrajectory(
            *spacetime_obj, absl::StrCat("st_traj/before_modify/", *traj_id),
            vis::Color::kCyan);
      }
    }
  }

  // Combine st boundaries.
  std::move(interactive_st_boundaries.begin(), interactive_st_boundaries.end(),
            std::back_inserter(non_interactive_st_boundaries));
  st_boundaries_with_decision->swap(non_interactive_st_boundaries);

  // Add timeout qevent.
  const auto interactive_speed_decision_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  constexpr double kInteractiveSpeedDecisionTimeoutThres = 15.0;  // ms.
  if (interactive_speed_decision_time > kInteractiveSpeedDecisionTimeoutThres) {
    QEVENT("yumeng", "interactive_speed_decision_timeout", [&](QEvent* qevent) {
      qevent
          ->AddField("interactive_speed_decision_time[ms]",
                     interactive_speed_decision_time)
          .AddField("time_limit[ms]", kInteractiveSpeedDecisionTimeoutThres);
    });
  }

  VLOG(2) << "Interactive speed decision cost time(ms): "
          << interactive_speed_decision_time;

  return absl::OkStatus();
}

}  // namespace

absl::Status MakeInteractiveSpeedDecision(
    const VehicleGeometryParamsProto& vehicle_geom,
    const MotionConstraintParamsProto& motion_constraint_params,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpeedLimit& speed_limit, const DiscretizedPath& path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpeedFinderParamsProto& speed_finder_params, double speed_cap,
    int traj_steps, SpeedVector* preliminary_speed,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    InteractiveSpeedDebugProto* interactive_speed_debug,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("MakeInteractiveSpeedDecision");
  QCHECK_NOTNULL(preliminary_speed);
  QCHECK_NOTNULL(st_boundaries_with_decision);
  QCHECK_NOTNULL(processed_st_objects);
  QCHECK_NOTNULL(interactive_speed_debug);

  absl::flat_hash_map<std::string, StBoundaryModificationInfo>
      modification_info;
  RETURN_IF_ERROR(GenerateInteractiveSpeedProfile(
      vehicle_geom, motion_constraint_params, st_graph, st_traj_mgr,
      speed_limit, path, plan_start_point, speed_finder_params, speed_cap,
      *processed_st_objects, traj_steps, preliminary_speed, &modification_info,
      st_boundaries_with_decision, interactive_speed_debug, thread_pool));

  GenerateLeadStBoundaryModificationInfo(
      vehicle_geom, st_traj_mgr, path, *preliminary_speed,
      *st_boundaries_with_decision, *processed_st_objects, &modification_info);

  const PostStboundaryModifierInput modifier_input{
      .st_graph = &st_graph,
      .st_traj_mgr = &st_traj_mgr,
      .modification_info_map = &modification_info};
  PostModifyStBoundaries(modifier_input, st_boundaries_with_decision,
                         processed_st_objects);

  return absl::OkStatus();
}
}  // namespace qcraft::planner
