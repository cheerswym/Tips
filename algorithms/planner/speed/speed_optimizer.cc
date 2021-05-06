#include "onboard/planner/speed/speed_optimizer.h"

#include <algorithm>
#include <limits>
#include <map>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {
namespace {

using Sfp = SpeedFinderParamsProto;
using DecisionType = StBoundaryProto::DecisionType;
using ConstraintMgr = SpeedOptimizerConstraintManager;

struct IntegratedConstraintData {
  double bound = 0.0;
  double speed = 0.0;
  double prob = 0.0;
};

const double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kCheckThreshold = -1e-2;
constexpr double kLowSpeedThreshold = 1.0;
constexpr double kPathLengthThreshold = 0.1;  // m.

constexpr double kMaxProbOfAllowableCollision = 0.1;
constexpr double kMinProbOfNotShrunk = 0.8;
// 0.0 represents considering the average case prediction.
// 1.0 represents considering the worst case prediction.
constexpr double kPredictionImpactFactor = 0.5;

void PostProcessForValidity(SpeedVector *speed_data) {
  QCHECK_NOTNULL(speed_data);
  for (int i = 0; i < speed_data->size(); ++i) {
    const double s = (*speed_data)[i].s();
    if (i == 0) {
      (*speed_data)[i].set_s(std::max(s, 0.0));
    } else {
      // Monotomic requirement.
      const double prev_s = (*speed_data)[i - 1].s();
      const double s_diff = s - prev_s;
      if (s_diff < kCheckThreshold) {
        QEVENT_EVERY_N_SECONDS("ping", "speed_point_s_nonmonotomic", 1.0,
                               [&](QEvent *qevent) {
                                 qevent->AddField("s_diff", s_diff)
                                     .AddField("prev_s", prev_s)
                                     .AddField("now_s", s)
                                     .AddField("index", i);
                               });

        QLOG_EVERY_N_SEC(WARNING, 1.0) << absl::StrFormat(
            "The cumulative distance of speed point is nonmonotonic, the "
            "threshold: %.3f has been exceeded, prev_s: %.3f now_s: %.3f.",
            kCheckThreshold, prev_s, s);
      }
      (*speed_data)[i].set_s(std::max(s, (*speed_data)[i - 1].s()));
    }
    const double v = (*speed_data)[i].v();
    if (v < kCheckThreshold) {
      QEVENT_EVERY_N_SECONDS("ping", "osqp_neg_speed_too_large", 1.0,
                             [&](QEvent *qevent) {
                               qevent->AddField("v", v).AddField("index", i);
                             });

      QLOG_EVERY_N_SEC(WARNING, 1.0) << absl::StrFormat(
          "The negative speed %.3f calculated by osqp is too large,"
          "the threshold: %.3f has been exceeded.",
          v, kCheckThreshold);
    }
    (*speed_data)[i].set_v(std::max(v, 0.0));
    VLOG(3) << "speed_data:[" << i << "] = " << (*speed_data)[i].DebugString();
  }
}

void PostProcessForForceStop(const Sfp::SpeedOptimizerParamsProto &params,
                             SpeedVector *speed_data) {
  QCHECK_NOTNULL(speed_data);
  // TODO(renjie): Tune the force stop threshold down after control
  // performance improves.
  bool force_brake = true;
  int i = 0;
  while ((*speed_data)[i].t() < params.force_stop_traj_time_threshold()) {
    if ((*speed_data)[i].v() > params.force_stop_speed_threshold()) {
      force_brake = false;
      break;
    }
    ++i;
  }
  if (speed_data->TotalLength() >= params.force_stop_traj_length_threshold()) {
    force_brake = false;
  }
  if (force_brake) {
    for (int i = 0; i < speed_data->size(); ++i) {
      (*speed_data)[i].set_s(0.0);
      (*speed_data)[i].set_v(0.0);
      (*speed_data)[i].set_a(0.0);
      (*speed_data)[i].set_j(0.0);
    }
  }
}

double MakeUpperBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound,
                                   double max_path_length) {
  QCHECK_GE(max_path_length, collision_bound);
  QCHECK_GE(collision_bound, reaction_bound);
  const PiecewiseLinearFunction<double> upper_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {max_path_length, collision_bound, reaction_bound, reaction_bound}};
  return upper_bound_plf(prob);
}

double MakeLowerBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound) {
  QCHECK_GE(reaction_bound, collision_bound);
  QCHECK_GE(collision_bound, 0.0);
  const PiecewiseLinearFunction<double> lower_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {0.0, collision_bound, reaction_bound, reaction_bound}};
  return lower_bound_plf(prob);
}

absl::flat_hash_map<std::string, IntegratedConstraintData> IntegrateConstraint(
    const absl::flat_hash_map<
        std::string, std::vector<IntegratedConstraintData>> &raw_constraints,
    bool is_follow) {
  absl::flat_hash_map<std::string, IntegratedConstraintData>
      integrated_constraints;
  constexpr double kMaxProbWithMargin = 1.0 + 1e-6;
  for (const auto &[id, constraints] : raw_constraints) {
    CHECK(!constraints.empty());
    if (constraints.size() == 1) {
      integrated_constraints[id] = constraints[0];
      continue;
    }
    IntegratedConstraintData integrated_dat;
    double min_lower_bound = std::numeric_limits<double>::max();
    for (const auto &dat : constraints) {
      // TODO(ping): Delete this code when fixed sum of prob greater
      // than 1.0.
      const double new_prob = integrated_dat.prob + dat.prob;
      if (new_prob > kMaxProbWithMargin) {
        QLOG(WARNING) << "The sum of probabilities of an obstacle is "
                         "greater than 1.0. Object id: "
                      << id;
        break;
      }
      if (is_follow) {
        min_lower_bound = std::min(min_lower_bound, dat.bound);
      }
      integrated_dat.bound += dat.bound * dat.prob;
      integrated_dat.speed += dat.speed * dat.prob;
      integrated_dat.prob += dat.prob;
    }
    const double final_prob = integrated_dat.prob;
    QCHECK_GT(final_prob, 0.0);
    QCHECK_LE(final_prob, kMaxProbWithMargin);
    integrated_dat.bound /= final_prob;
    integrated_dat.speed /= final_prob;
    // "kPredictionImpactFactor" only work on the st_trajs with following
    // decision.
    if (is_follow) {
      integrated_dat.bound =
          Lerp(integrated_dat.bound, min_lower_bound, kPredictionImpactFactor);
    }
    integrated_constraints[id] = std::move(integrated_dat);
  }
  return integrated_constraints;
}

void MakeMovingIntegratedConstraintData(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double curr_time, double max_path_length,
    absl::flat_hash_map<std::string, IntegratedConstraintData>
        *integrated_following_data,
    absl::flat_hash_map<std::string, IntegratedConstraintData>
        *integrated_leading_data) {
  QCHECK_NOTNULL(integrated_following_data);
  QCHECK_NOTNULL(integrated_leading_data);

  absl::flat_hash_map<std::string, std::vector<IntegratedConstraintData>>
      raw_following_data;
  absl::flat_hash_map<std::string, std::vector<IntegratedConstraintData>>
      raw_leading_data;
  for (const StBoundaryWithDecision &stb_wd : st_boundaries_with_decision) {
    const StBoundary *st_boundary = stb_wd.st_boundary();
    const auto decision = stb_wd.decision_type();
    if (decision == StBoundaryProto::UNKNOWN ||
        decision == StBoundaryProto::IGNORE) {
      continue;
    }

    if (st_boundary->is_stationary()) continue;

    const auto unblocked_s_range =
        stb_wd.GetUnblockSRange(curr_time, max_path_length);
    if (!unblocked_s_range.has_value()) continue;

    const auto obj_speed = st_boundary->GetStBoundarySpeedAtT(curr_time);
    QCHECK(obj_speed.has_value());

    QCHECK(decision == StBoundaryProto::FOLLOW ||
           decision == StBoundaryProto::LEAD);
    IntegratedConstraintData dat = {.bound = decision == StBoundaryProto::FOLLOW
                                                 ? unblocked_s_range->second
                                                 : unblocked_s_range->first,
                                    .speed = *obj_speed,
                                    .prob = st_boundary->probability()};

    const auto integration_id = GetStBoundaryIntegrationId(*st_boundary);
    if (decision == StBoundaryProto::FOLLOW) {
      raw_following_data[integration_id].push_back(std::move(dat));
    } else if (decision == StBoundaryProto::LEAD) {
      raw_leading_data[integration_id].push_back(std::move(dat));
    }
  }

  *integrated_following_data =
      IntegrateConstraint(raw_following_data, /*is_follow=*/true);
  *integrated_leading_data =
      IntegrateConstraint(raw_leading_data, /*is_follow=*/false);
}

}  // namespace

SpeedOptimizer::SpeedOptimizer(
    const MotionConstraintParamsProto *motion_constraint_params,
    const SpeedFinderParamsProto *speed_finder_params, double path_length,
    double default_speed_limit, int traj_steps)
    : motion_constraint_params_(QCHECK_NOTNULL(motion_constraint_params)),
      speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      speed_optimizer_params_(&speed_finder_params->speed_optimizer_params()),
      total_time_(kTrajectoryTimeStep * traj_steps),
      knot_num_(speed_optimizer_params_->knot_num()),
      allowed_max_speed_(Mph2Mps(default_speed_limit)),
      max_path_length_(path_length - kPathLengthThreshold),
      follow_weight_min_s_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->follow_weight_min_s_plf())),
      follow_distance_rel_speed_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->follow_distance_rel_speed_plf())),
      probability_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->probability_gain_plf())),
      accel_weight_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_weight_gain_plf())),
      piecewise_time_range_(
          speed_optimizer_params_->piecewise_time_range().begin(),
          speed_optimizer_params_->piecewise_time_range().end()),
      object_speed_follow_time_headway_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->object_speed_follow_time_headway_plf())),
      accel_lower_bound_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_lower_bound_plf())) {
  QCHECK_GT(speed_optimizer_params_->knot_num() - 1, 0);
  QCHECK_GT(max_path_length_, 0.0);
  const auto &moving_time_gain =
      speed_optimizer_params_->time_gain_for_moving_object();
  const auto &static_time_gain =
      speed_optimizer_params_->time_gain_for_static_object();
  std::vector<double> time_gain_for_moving_object(moving_time_gain.begin(),
                                                  moving_time_gain.end());
  std::vector<double> time_gain_for_static_object(static_time_gain.begin(),
                                                  static_time_gain.end());
  moving_obj_time_gain_plf_ = {piecewise_time_range_,
                               std::move(time_gain_for_moving_object)};
  stationary_obj_time_gain_plf_ = {piecewise_time_range_,
                                   std::move(time_gain_for_static_object)};
  delta_t_ = total_time_ / (speed_optimizer_params_->knot_num() - 1);
}

absl::Status SpeedOptimizer::Optimize(
    const ApolloTrajectoryPointProto &init_point,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const absl::flat_hash_map<Sfp::SpeedLimitType, SpeedLimit> &speed_limit_map,
    const SpeedVector &reference_speed, SpeedVector *optimized_speed,
    SpeedFinderDebugProto *speed_finder_debug_proto) {
  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  init_point_ = init_point;

  const auto accel_bound =
      std::make_pair(motion_constraint_params_->max_deceleration(),
                     motion_constraint_params_->max_acceleration());

  const auto speed_bound_map =
      EstimateSpeedBound(speed_limit_map, reference_speed);

  // Only add upper bound constraints for speed.
  const auto &speed_upper_weights =
      speed_optimizer_params_->speed_upper_slack_weights();
  absl::flat_hash_map<Sfp::SpeedLimitType, double> speed_weight_map;
  speed_weight_map.reserve(speed_upper_weights.size());
  for (const auto &weight : speed_upper_weights) {
    InsertOrDie(&speed_weight_map, weight.type(), weight.weight());
  }

  ConstraintMgr constraint_mgr(piecewise_time_range_, delta_t_);
  for (int i = 1; i < knot_num_; ++i) {
    SCOPED_QTRACE("SpeedOptimizer::MakeConstraint");
    MakeSConstraint(i, init_point.v(), st_boundaries_with_decision,
                    &constraint_mgr, speed_finder_debug_proto);
    MakeSpeedConstraint(i, speed_bound_map, speed_weight_map, &constraint_mgr,
                        speed_finder_debug_proto);
    MakeAccelConstraint(i, init_point.v(), accel_bound, &constraint_mgr);
  }

  const int soft_s_lower_num = constraint_mgr.GetSlackNumOfLowerS();
  const int soft_s_upper_num = constraint_mgr.GetSlackNumOfUpperS();
  const int soft_v_lower_num = constraint_mgr.GetSlackNumOfLowerSpeed();
  const int soft_v_upper_num = constraint_mgr.GetSlackNumOfUpperSpeed();
  const int soft_a_lower_num = constraint_mgr.GetSlackNumOfLowerAccel();
  const int soft_a_upper_num = constraint_mgr.GetSlackNumOfUpperAccel();

  // Set debug proto.
  auto *speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  speed_optimizer_debug->set_init_v(init_point.v());
  speed_optimizer_debug->set_init_a(init_point.a());
  speed_optimizer_debug->set_delta_t(delta_t_);
  speed_optimizer_debug->set_lower_s_slack_term_num(soft_s_lower_num);
  speed_optimizer_debug->set_upper_s_slack_term_num(soft_s_upper_num);
  speed_optimizer_debug->set_lower_v_slack_term_num(soft_v_lower_num);
  speed_optimizer_debug->set_upper_v_slack_term_num(soft_v_upper_num);
  speed_optimizer_debug->set_lower_a_slack_term_num(soft_a_lower_num);
  speed_optimizer_debug->set_upper_a_slack_term_num(soft_a_upper_num);

  VLOG(3) << "lower_s_num = " << soft_s_lower_num;
  VLOG(3) << "upper_s_num = " << soft_s_upper_num;

  solver_ = std::make_unique<PiecewiseJerkQpSolver>(
      speed_optimizer_params_->knot_num(), delta_t_, soft_s_lower_num,
      soft_s_upper_num, soft_v_lower_num, soft_v_upper_num, soft_a_lower_num,
      soft_a_upper_num);

  if (!AddConstarints(init_point, constraint_mgr)) {
    const std::string err_msg = "Failed to add constraint.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_add_constraint",
                           2.0, [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });
    return absl::InternalError(err_msg);
  }

  if (!AddKernel(constraint_mgr,
                 FindOrDie(speed_bound_map, Sfp::COMBINATION))) {
    const std::string err_msg = "Failed to add kernel.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_add_kernel", 2.0,
                           [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });
    return absl::InternalError(err_msg);
  }

  if (const absl::Status status = Solve(); !status.ok()) {
    const auto err_msg = status.ToString();
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_qp_solver", 2.0,
                           [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });

    return absl::InternalError(err_msg);
  }

  // extract output
  optimized_speed->clear();
  for (double t = 0.0; t < total_time_;
       t += speed_optimizer_params_->output_time_resolution()) {
    std::array<double, 4> res = solver_->Evaluate(t);
    SpeedPoint speed_point(t, res[0], res[1], res[2], res[3]);
    speed_point.ToProto(speed_optimizer_debug->add_optimized_speed());
    optimized_speed->push_back(std::move(speed_point));
  }

  PostProcessForValidity(optimized_speed);
  if (speed_optimizer_params_->enable_force_stop()) {
    PostProcessForForceStop(*speed_optimizer_params_, optimized_speed);
  }

  const double speed_length = optimized_speed->TotalLength();
  const double path_length = max_path_length_ + kPathLengthThreshold;
  if (speed_length > path_length) {
    QEVENT_EVERY_N_SECONDS(
        "ping", "speed_length_is_larger_than_path_length", 1.0,
        [&](QEvent *qevent) {
          qevent->AddField("path_length", path_length)
              .AddField("optimized_speed_length", speed_length)
              .AddField("length_diff", path_length - speed_length);
        });
  }

  return absl::OkStatus();
}

absl::Status SpeedOptimizer::OptimizeWithMaxSpeed(
    const ApolloTrajectoryPointProto &init_point,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedVector *optimized_speed,
    SpeedFinderDebugProto *speed_finder_debug_proto) {
  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const auto accel_bound =
      std::make_pair(motion_constraint_params_->max_deceleration(),
                     motion_constraint_params_->max_acceleration());

  // Generate speed upper bound map with max speed.
  absl::flat_hash_map<Sfp::SpeedLimitType, std::vector<SpeedBoundWithInfo>>
      speed_bound_map;
  std::vector<SpeedBoundWithInfo> speed_bound(
      knot_num_, {.bound = allowed_max_speed_,
                  .info = Sfp::SpeedLimitType_Name(Sfp::LANE)});
  InsertIfNotPresent(&speed_bound_map, Sfp::LANE, speed_bound);
  InsertIfNotPresent(&speed_bound_map, Sfp::COMBINATION, speed_bound);

  // Only add upper bound constraints for speed.
  const auto &speed_upper_weights =
      speed_optimizer_params_->speed_upper_slack_weights();
  absl::flat_hash_map<Sfp::SpeedLimitType, double> speed_weight_map;
  speed_weight_map.reserve(speed_upper_weights.size());
  for (const auto &weight : speed_upper_weights) {
    InsertOrDie(&speed_weight_map, weight.type(), weight.weight());
  }

  ConstraintMgr constraint_mgr(piecewise_time_range_, delta_t_);
  for (int i = 1; i < knot_num_; ++i) {
    SCOPED_QTRACE("SpeedOptimizer::MakeConstraint");
    MakeSConstraint(i, init_point.v(), st_boundaries_with_decision,
                    &constraint_mgr, speed_finder_debug_proto);
    MakeSpeedConstraint(i, speed_bound_map, speed_weight_map, &constraint_mgr,
                        speed_finder_debug_proto);
    MakeAccelConstraint(i, init_point.v(), accel_bound, &constraint_mgr);
  }

  const int soft_s_lower_num = constraint_mgr.GetSlackNumOfLowerS();
  const int soft_s_upper_num = constraint_mgr.GetSlackNumOfUpperS();
  const int soft_v_lower_num = constraint_mgr.GetSlackNumOfLowerSpeed();
  const int soft_v_upper_num = constraint_mgr.GetSlackNumOfUpperSpeed();
  const int soft_a_lower_num = constraint_mgr.GetSlackNumOfLowerAccel();
  const int soft_a_upper_num = constraint_mgr.GetSlackNumOfUpperAccel();

  // Set debug proto.
  auto *speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  speed_optimizer_debug->set_init_v(init_point.v());
  speed_optimizer_debug->set_init_a(init_point.a());
  speed_optimizer_debug->set_lower_s_slack_term_num(soft_s_lower_num);
  speed_optimizer_debug->set_upper_s_slack_term_num(soft_s_upper_num);
  speed_optimizer_debug->set_upper_s_slack_term_num(soft_v_lower_num);
  speed_optimizer_debug->set_upper_v_slack_term_num(soft_v_upper_num);
  speed_optimizer_debug->set_lower_a_slack_term_num(soft_a_lower_num);
  speed_optimizer_debug->set_upper_a_slack_term_num(soft_a_upper_num);

  VLOG(3) << "lower_s_num = " << soft_s_lower_num;
  VLOG(3) << "upper_s_num = " << soft_s_upper_num;

  solver_ = std::make_unique<PiecewiseJerkQpSolver>(
      speed_optimizer_params_->knot_num(), delta_t_, soft_s_lower_num,
      soft_s_upper_num, soft_v_lower_num, soft_v_upper_num, soft_a_lower_num,
      soft_a_upper_num);

  if (!AddConstarints(init_point, constraint_mgr)) {
    const std::string err_msg = "Failed to add constraint.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_add_constraint",
                           2.0, [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });
    return absl::InternalError(err_msg);
  }

  if (!AddKernel(constraint_mgr,
                 FindOrDie(speed_bound_map, Sfp::COMBINATION))) {
    const std::string err_msg = "Failed to add kernel.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_add_kernel", 2.0,
                           [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });
    return absl::InternalError(err_msg);
  }

  if (const absl::Status status = Solve(); !status.ok()) {
    const auto err_msg = status.ToString();
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("ping", "speed_optimizer_failure_qp_solver", 2.0,
                           [err_msg](QEvent *qevent) {
                             qevent->AddField("error_message", err_msg);
                           });
    return absl::InternalError(err_msg);
  }

  // extract output
  optimized_speed->clear();
  for (double t = 0.0; t < total_time_;
       t += speed_optimizer_params_->output_time_resolution()) {
    std::array<double, 4> res = solver_->Evaluate(t);
    SpeedPoint speed_point(t, res[0], res[1], res[2], res[3]);
    speed_point.ToProto(speed_optimizer_debug->add_optimized_speed());
    optimized_speed->push_back(std::move(speed_point));
  }

  PostProcessForValidity(optimized_speed);
  if (speed_optimizer_params_->enable_force_stop()) {
    PostProcessForForceStop(*speed_optimizer_params_, optimized_speed);
  }

  return absl::OkStatus();
}

void SpeedOptimizer::MakeFollowConstraintForStationaryObject(
    int knot_idx, double s_upper, double min_s,
    double follow_standstill_distance, const std::string &st_boundary_id,
    ConstraintMgr *constraint_mgr,
    SpeedFinderDebugProto *speed_finder_debug_proto) const {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double time = knot_idx * delta_t_;

  double weight = 0.0;
  if (speed_optimizer_params_->enable_time_gain_for_stationary_obj()) {
    const double dist_of_disable_time_gain =
        speed_optimizer_params_->dist_of_disable_time_gain();
    const double time_gain = min_s < dist_of_disable_time_gain
                                 ? 1.0
                                 : stationary_obj_time_gain_plf_(time);
    weight = time_gain * speed_optimizer_params_->s_stop_weight();
  } else {
    const double min_s_dist_gain = follow_weight_min_s_plf_(min_s);
    weight = min_s_dist_gain * speed_optimizer_params_->s_stop_weight();
  }

  const double upper_bound =
      std::max(0.0, s_upper - follow_standstill_distance);

  constraint_mgr->AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = knot_idx, .weight = weight, .bound = upper_bound});

  const std::string chart_tips =
      absl::StrFormat("following_distance: %.2fm\nweight: %.2f(stop_weight)",
                      follow_standstill_distance, weight);

  StBoundaryProto &st_boundary_proto =
      (*speed_finder_debug_proto->mutable_st_boundaries())[st_boundary_id];
  if (!st_boundary_proto.has_safety_bound() ||
      st_boundary_proto.safety_bound() > upper_bound) {
    st_boundary_proto.set_safety_bound(upper_bound);
  }
  auto *speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto &plot_data =
      (*speed_optimizer_debug->mutable_soft_s_upper_bound())[st_boundary_id];
  plot_data.add_value(upper_bound);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(follow_standstill_distance);
  plot_data.add_info(chart_tips);
}

void SpeedOptimizer::MakeFollowConstraintForMovingObject(
    int knot_idx, double s_upper, double obj_speed, double prob,
    double av_speed, double follow_standstill_distance,
    const std::string &st_boundary_id, const std::string &integration_id,
    ConstraintMgr *constraint_mgr,
    SpeedFinderDebugProto *speed_finder_debug_proto) const {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double follow_safety_distance =
      std::min(speed_finder_params_->follow_safety_distance(),
               follow_standstill_distance);
  const double rel_speed_gain =
      follow_distance_rel_speed_plf_(obj_speed - av_speed);
  const double follow_standstill = follow_standstill_distance;

  // Step 1: Calculate following distance.
  const double follow_time_headway =
      object_speed_follow_time_headway_plf_(obj_speed);
  constexpr double kNegDistanceBuffer = 0.5;
  const double follow_distance =
      std::min(std::max((obj_speed * follow_time_headway + follow_standstill) *
                            rel_speed_gain,
                        follow_standstill),
               s_upper + kNegDistanceBuffer);

  // Step 2: Make raw upper bound.
  const double weak_upper_bound = s_upper - follow_distance;
  const double strong_upper_bound = std::max(
      s_upper - follow_safety_distance * rel_speed_gain, weak_upper_bound);

  // Step 3: Process the raw upper bound according to the probability.
  const double weak_upper_bound_by_prob = MakeUpperBoundByProbability(
      prob, weak_upper_bound, s_upper, max_path_length_);
  const double strong_upper_bound_by_prob = MakeUpperBoundByProbability(
      prob, strong_upper_bound, s_upper, max_path_length_);

  // Step 4: Make slack variable's weight.
  const double time = delta_t_ * knot_idx;
  const double time_gain = moving_obj_time_gain_plf_(time);

  const double strong_weight =
      time_gain * speed_optimizer_params_->s_follow_strong_weight();
  const double weak_weight =
      obj_speed < kLowSpeedThreshold
          ? time_gain * speed_optimizer_params_->s_stop_weight()
          : time_gain * speed_optimizer_params_->s_follow_weak_weight();

  // Step 5: Add constraint data to constraint manager.
  constraint_mgr->AddSoftSUpperConstraint(ConstraintMgr::BoundStrType::WEAK,
                                          {.knot_idx = knot_idx,
                                           .weight = weak_weight,
                                           .bound = weak_upper_bound_by_prob});
  constraint_mgr->AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = knot_idx,
       .weight = strong_weight,
       .bound = strong_upper_bound_by_prob});

  // Step 6: Debug and plot.
  const double actual_following_distance =
      s_upper - weak_upper_bound_by_prob;  // Could be negative.
  const std::string chart_tips = absl::StrFormat(
      "final prob: %.2f\nraw_following_distance: "
      "%.2fm\nactual_following_distance: "
      "%.2fm\nweight: %.2f\n"
      "obj_speed: %.2fm/s\ntime_gain: %.2f",
      prob, follow_distance, actual_following_distance, weak_weight, obj_speed,
      time_gain);

  StBoundaryProto &st_boundary_proto =
      (*speed_finder_debug_proto->mutable_st_boundaries())[st_boundary_id];
  if (!st_boundary_proto.has_safety_bound() ||
      st_boundary_proto.safety_bound() > strong_upper_bound_by_prob) {
    st_boundary_proto.set_safety_bound(strong_upper_bound_by_prob);
  }
  if (!st_boundary_proto.has_following_bound() ||
      st_boundary_proto.following_bound() > weak_upper_bound_by_prob) {
    st_boundary_proto.set_following_bound(weak_upper_bound_by_prob);
  }
  auto *speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto &plot_data =
      (*speed_optimizer_debug->mutable_soft_s_upper_bound())[integration_id];
  plot_data.add_value(weak_upper_bound_by_prob);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(actual_following_distance);
  plot_data.add_info(chart_tips);
}

// TODO(ping): Refactor this code.
void SpeedOptimizer::MakeLeadConstraint(
    int knot_idx, double s_lower, double obj_speed, double prob,
    double av_speed, double lead_standstill_distance,
    const std::string &integration_id, ConstraintMgr *constraint_mgr,
    SpeedFinderDebugProto *speed_finder_debug_proto) const {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  // Step 1: Calculate leading distance.
  // TODO(ping): consider relative speed.
  const double leading_distance =
      obj_speed * speed_finder_params_->lead_time_headway() +
      lead_standstill_distance;
  const double weak_lower_bound = s_lower + leading_distance;
  const double strong_lower_bound = s_lower;

  // Step 2: Make lower bound by the probability.
  const double weak_lower_bound_by_prob =
      MakeLowerBoundByProbability(prob, weak_lower_bound, s_lower);
  const double strong_lower_bound_by_prob =
      MakeLowerBoundByProbability(prob, strong_lower_bound, s_lower);

  // Step 3: Make slack variable's weight.
  const double time = knot_idx * delta_t_;
  const double time_gain = moving_obj_time_gain_plf_(time);
  const double strong_weight =
      time_gain * speed_optimizer_params_->s_lead_strong_weight();
  const double weak_weight =
      time_gain * speed_optimizer_params_->s_lead_weak_weight();

  // Step 4: Add constraint data to constraint manager.
  constraint_mgr->AddSoftSLowerConstraint(ConstraintMgr::BoundStrType::WEAK,
                                          {.knot_idx = knot_idx,
                                           .weight = weak_weight,
                                           .bound = weak_lower_bound_by_prob});
  constraint_mgr->AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = knot_idx,
       .weight = strong_weight,
       .bound = strong_lower_bound_by_prob});

  // Step 5: Debug and plot.
  const double actual_leading_distance =
      weak_lower_bound_by_prob - s_lower;  // Could be negative.
  auto *speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();

  auto &plot_data =
      (*speed_optimizer_debug->mutable_soft_s_lower_bound())[integration_id];
  plot_data.add_value(weak_lower_bound_by_prob);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(actual_leading_distance);
  plot_data.add_info(absl::StrFormat(
      "final prob: %.2f\nraw_leading_distance: "
      "%.2f\nactual_leading_distance: "
      "%.2f\nweight: %.2f\nobj_speed : %.2fm/s\ntime_gain: %.2f",
      prob, leading_distance, actual_leading_distance, weak_weight, obj_speed,
      time_gain));
}

void SpeedOptimizer::MakeSConstraint(
    int knot_idx, double av_speed,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    ConstraintMgr *constraint_mgr,
    SpeedFinderDebugProto *speed_finder_debug_proto) {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double time = knot_idx * delta_t_;
  absl::flat_hash_map<std::string, IntegratedConstraintData>
      integration_following_data;
  absl::flat_hash_map<std::string, IntegratedConstraintData>
      integration_leading_data;
  MakeMovingIntegratedConstraintData(
      st_boundaries_with_decision, time, max_path_length_,
      &integration_following_data, &integration_leading_data);

  absl::flat_hash_set<std::string> processed_moving_object_set;
  for (const auto &boundary : st_boundaries_with_decision) {
    const StBoundary *st_boundary = boundary.st_boundary();

    if (time < st_boundary->min_t() || time > st_boundary->max_t()) {
      continue;
    }

    const auto decision = boundary.decision_type();
    if (decision == StBoundaryProto::UNKNOWN ||
        decision == StBoundaryProto::IGNORE) {
      continue;
    }

    const auto integration_id = GetStBoundaryIntegrationId(*st_boundary);
    if (!st_boundary->is_stationary() &&
        processed_moving_object_set.contains(integration_id)) {
      continue;
    }

    if (decision == StBoundaryProto::FOLLOW) {
      if (st_boundary->is_stationary()) {
        const auto s_range = boundary.GetUnblockSRange(time, max_path_length_);
        if (!s_range.has_value()) continue;
        MakeFollowConstraintForStationaryObject(
            knot_idx, /*upper_s=*/s_range->second, st_boundary->min_s(),
            boundary.follow_standstill_distance(), st_boundary->id(),
            constraint_mgr, speed_finder_debug_proto);
      } else {
        const auto &data =
            FindOrDie(integration_following_data, integration_id);
        MakeFollowConstraintForMovingObject(
            knot_idx, /*upper_s=*/data.bound, std::max(data.speed, 0.0),
            data.prob, av_speed, boundary.follow_standstill_distance(),
            st_boundary->id(), integration_id, constraint_mgr,
            speed_finder_debug_proto);
        processed_moving_object_set.insert(integration_id);
      }
    } else if (decision == StBoundaryProto::LEAD) {
      const auto &data = FindOrDie(integration_leading_data, integration_id);
      MakeLeadConstraint(knot_idx, /*lower_s=*/data.bound,
                         std::max(data.speed, 0.0), data.prob, av_speed,
                         boundary.lead_standstill_distance(), integration_id,
                         constraint_mgr, speed_finder_debug_proto);
      processed_moving_object_set.insert(integration_id);
    } else if (decision == StBoundaryProto::UNKNOWN) {
      continue;
    } else {
      QLOG(WARNING) << "Unhandled boundary type: "
                    << StBoundaryProto::DecisionType_Name(
                           boundary.decision_type());
    }
  }

  // Add hard constraint.
  constraint_mgr->AddHardSConstraint(knot_idx, 0.0, max_path_length_);
}

void SpeedOptimizer::MakeSpeedConstraint(
    int knot_idx,
    const absl::flat_hash_map<Sfp::SpeedLimitType,
                              std::vector<SpeedBoundWithInfo>> &speed_limit_map,
    const absl::flat_hash_map<Sfp::SpeedLimitType, double> &speed_weight_map,
    ConstraintMgr *constraint_mgr,
    SpeedFinderDebugProto *speed_finder_debug) const {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug);

  auto *speed_optimizer_debug = speed_finder_debug->mutable_speed_optimizer();

  const double lane_speed_limit =
      FindOrDie(speed_limit_map, Sfp::LANE)[knot_idx].bound;

  constexpr double kDecel = -1.0;  // m/s^2
  constexpr double kAllowedIgnoreTime = 2.0;
  const double time = knot_idx * delta_t_;
  const double v_at_time = std::max(init_point_.v() + kDecel * time, 0.0);

  for (const auto &[type, speed_limit] : speed_limit_map) {
    if (type == Sfp::COMBINATION) continue;
    const double speed_upper_bound = speed_limit[knot_idx].bound;

    // Avoid the close range speed limits lower than the init speed, it may
    // increase the time cost and failure probability.
    // BANDAID(ping): Ignore curvature speed limit may cause steering
    // rate exceed the limitation.
    if (speed_upper_bound < v_at_time && time < kAllowedIgnoreTime &&
        type != Sfp::CURVATURE) {
      continue;
    }

    // Add debug info for plot speed limit in chart.
    auto &plot_data =
        (*speed_optimizer_debug
              ->mutable_speed_limit())[Sfp::SpeedLimitType_Name(type)];
    plot_data.add_value(speed_upper_bound);
    plot_data.add_time(knot_idx * delta_t_);
    plot_data.add_info(speed_limit[knot_idx].info);

    // If the speed limit is greater than lane speed, it will not be added to
    // the constraint, but it can be plotted in chart.
    if (type != Sfp::LANE && speed_upper_bound > lane_speed_limit) {
      continue;
    }
    const double speed_weight = FindOrDie(speed_weight_map, type);
    constraint_mgr->AddSoftSpeedUpperConstraints(type,
                                                 {.knot_idx = knot_idx,
                                                  .weight = speed_weight,
                                                  .bound = speed_upper_bound});
  }
}

bool SpeedOptimizer::AddConstarints(
    const ApolloTrajectoryPointProto &init_point,
    const ConstraintMgr &constraint_mgr) {
  SCOPED_QTRACE("SpeedOptimizer::AddConstarints");

  /******************* add init constraint **********************/
  solver_->AddNthOrderEqualityConstraint(/*order=*/0, std::vector<int>{0},
                                         std::vector<double>{1.0}, 0.0);
  solver_->AddNthOrderEqualityConstraint(/*order=*/1, std::vector<int>{0},
                                         std::vector<double>{1.0},
                                         std::max(0.0, init_point.v()));
  solver_->AddNthOrderEqualityConstraint(
      /*order=*/2, std::vector<int>{0}, std::vector<double>{1.0},
      init_point.a());

  // set last jerk to 0.0
  solver_->AddNthOrderEqualityConstraint(/*order=*/3,
                                         std::vector<int>{knot_num_ - 1},
                                         std::vector<double>{1.0}, 0.0);

  /******************** add s constraint ********************/
  for (const auto &data : constraint_mgr.hard_s_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/0, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto &data : constraint_mgr.GetLowerConstraintOfSoftS()) {
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound,
        kInfinity);
  }

  for (const auto &data : constraint_mgr.GetUpperConstraintOfSoftS()) {
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, -kInfinity,
        data.bound);
  }

  /******************** add v constraint ********************/
  for (const auto &data : constraint_mgr.GetLowerConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound,
        kInfinity);
  }

  for (const auto &data : constraint_mgr.GetUpperConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, 0.0,
        data.bound);
  }

  /******************** add accel constraint ******************/
  for (const auto &data : constraint_mgr.hard_a_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/2, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto &data : constraint_mgr.GetLowerConstraintOfSoftAccel()) {
    solver_->AddSecondOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound,
        kInfinity);
  }

  /**************** add monotone constraint ***************/
  solver_->AddMonotonicConstraints();

  return true;
}

bool SpeedOptimizer::AddKernel(
    const ConstraintMgr &constraint_mgr,
    const std::vector<SpeedBoundWithInfo> &floor_speed_limit) {
  SCOPED_QTRACE("SpeedOptimizer::AddKernel");
  QCHECK_EQ(floor_speed_limit.size(), knot_num_);

  // TODO(ping): Use a more accurate speed reference model.
  constexpr double kSpeedLookForward = 8.0;  // m/s.
  const double lf_v_ref = std::max(init_point_.v(), 0.0) + kSpeedLookForward;
  // Add s and v weight.
  for (int i = 0; i < knot_num_; ++i) {
    solver_->AddNthOrderReferencePointKernel(
        /*order=*/0, i, max_path_length_,
        speed_optimizer_params_->s_kernel_weight());

    const double ref_v = std::min(floor_speed_limit[i].bound, lf_v_ref);
    solver_->AddNthOrderReferencePointKernel(
        /*order=*/1, i, ref_v, speed_optimizer_params_->speed_kernel_weight());
  }

  const double accel_weight_gain = accel_weight_gain_plf_(init_point_.v());
  solver_->AddSecondOrderDerivativeKernel(
      accel_weight_gain * speed_optimizer_params_->accel_kernel_weight());

  solver_->AddThirdOrderDerivativeKernel(
      speed_optimizer_params_->jerk_kernel_weight());

  // Add s slack weight.
  for (const auto &[slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  for (const auto &[slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  // Add v slack weight.
  for (const auto &[slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                         {slack_weight});
  }
  for (const auto &[slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  // Add a slack weight.
  for (const auto [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfAccel()) {
    solver_->AddSecondOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                          {slack_weight});
  }

  // Add regularization.
  solver_->AddRegularization(speed_optimizer_params_->regularization());

  return true;
}

absl::Status SpeedOptimizer::Solve() {
  SCOPED_QTRACE("SpeedOptimizer::Solve");
  return solver_->Optimize();
}

void SpeedOptimizer::MakeAccelConstraint(
    int knot_idx, double reference_speed,
    const std::pair<double, double> &accel_bound,
    ConstraintMgr *constraint_mgr) const {
  QCHECK_NOTNULL(constraint_mgr);
  const double accel_lower_bound =
      std::max(accel_lower_bound_plf_(reference_speed),
               motion_constraint_params_->max_deceleration());
  constraint_mgr->AddAccelSoftLowerConstraint(
      {.knot_idx = knot_idx,
       .weight = speed_optimizer_params_->accel_lower_slack_weight(),
       .bound = accel_lower_bound});
  constraint_mgr->AddAccelConstraint(knot_idx, accel_bound.first,
                                     accel_bound.second);
}

absl::flat_hash_map<Sfp::SpeedLimitType,
                    std::vector<SpeedOptimizer::SpeedBoundWithInfo>>
SpeedOptimizer::EstimateSpeedBound(
    const absl::flat_hash_map<Sfp::SpeedLimitType, SpeedLimit> &speed_limit_map,
    const SpeedVector &reference_speed) const {
  absl::flat_hash_map<Sfp::SpeedLimitType, std::vector<SpeedBoundWithInfo>>
      speed_upper_bound_map;
  speed_upper_bound_map.reserve(speed_limit_map.size());
  for (const auto &[type, speed_limit] : speed_limit_map) {
    std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
    speed_bounds_with_info.reserve(knot_num_);
    for (int i = 0; i < knot_num_; ++i) {
      const double t = i * delta_t_;
      const auto speed_point = reference_speed.EvaluateByTime(t);
      const double estimated_s =
          speed_point.has_value() ? speed_point->s() : init_point_.v() * t;
      const auto speed_limit_range =
          speed_limit.GetSpeedLimitRangeByS(estimated_s);
      if (speed_limit_range.has_value()) {
        speed_bounds_with_info.push_back(
            {.bound = speed_limit_range->speed_limit,
             .info = speed_limit_range->info});
      } else {
        speed_bounds_with_info.push_back(
            {.bound = allowed_max_speed_, .info = "NO_TYPE"});
      }
    }
    speed_upper_bound_map.emplace(type, std::move(speed_bounds_with_info));
  }
  return speed_upper_bound_map;
}

}  // namespace qcraft::planner
