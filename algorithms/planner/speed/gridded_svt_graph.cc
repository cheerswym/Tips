#include "onboard/planner/speed/gridded_svt_graph.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"

DEFINE_bool(enable_sampling_dp_reference_speed, false,
            "True to penalize sampling dp result towards default cruise speed");
DEFINE_bool(add_sampling_dp_speed_profiles_to_debug, false,
            "Whether add sampling dp speed profiles to debug.");

namespace qcraft::planner {

namespace {
constexpr double kDoubleEpsilon = 1.0e-6;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct BestEndPoint {
  // If this struct used as value for absl::flat_hash_map, to support operator
  // `[]`, default constructor should be retained.
  BestEndPoint() = default;
  BestEndPoint(const SvtGraphPoint* p, double c) : point(p), cost(c) {}
  const SvtGraphPoint* point = nullptr;
  double cost = kInf;
};

// Find the minimum cost svt graph point.
absl::StatusOr<SvtGraphPoint> FindOptimalCandidate(
    absl::Span<const SvtGraphPoint> candidate_points) {
  if (candidate_points.empty()) {
    return absl::FailedPreconditionError("Candidate points empty.");
  }
  const auto optimal_point = std::min_element(
      candidate_points.begin(), candidate_points.end(),
      [](const SvtGraphPoint& point1, const SvtGraphPoint& point2) {
        return point1.total_cost() < point2.total_cost();
      });
  return *optimal_point;
}

std::vector<std::vector<double>> GenerateAccMatrix(
    double max_acceleration, double max_deceleration, double unit_acc,
    double total_length_s, int dimension_t, int unit_t, double cur_v) {
  constexpr int kKeepUnitAccLayerIndex = 2;
  constexpr int kNoNeedToAccelerateRapidlyLayerIndex = 3;
  constexpr int kNoNeedToDecelerateHardLayerIndex = 5;
  constexpr double kGentleAccelerateAcc = 1.0;     // m/ss
  constexpr double kGentleDecelerateAcc = -2.0;    // m/ss
  constexpr double kMaxAccStep = 1.0;              // m/ss
  constexpr double kAccStepGain = 0.1;             // m/ss
  constexpr double kMaxSampleGapToMaxValue = 0.3;  // m/ss

  std::vector<std::vector<double>> acc_matrix;
  acc_matrix.reserve(dimension_t);
  const int acc_sample_max_size =
      FloorToInt((max_acceleration - max_deceleration) / unit_acc) + 2;
  for (int i = 0; i < dimension_t - 1; ++i) {
    if (i > kNoNeedToAccelerateRapidlyLayerIndex) {
      max_acceleration = kGentleAccelerateAcc;
    }
    if (i > kNoNeedToDecelerateHardLayerIndex) {
      max_deceleration = kGentleDecelerateAcc;
    }

    std::vector<double> cur_layer_acc;
    cur_layer_acc.reserve(acc_sample_max_size);
    double acc_step = unit_acc;
    if (i > kKeepUnitAccLayerIndex) {
      acc_step = std::min(
          acc_step + (i - kKeepUnitAccLayerIndex) * kAccStepGain, kMaxAccStep);
    }
    // Make sure acc = 0.0 can be sampled.
    for (double acc = 0.0; acc <= max_acceleration; acc += acc_step) {
      cur_layer_acc.push_back(acc);
    }
    if (max_acceleration - cur_layer_acc.back() > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_acceleration);
    } else {
      cur_layer_acc.back() = max_acceleration;
    }
    for (double acc = -acc_step; acc >= max_deceleration; acc -= acc_step) {
      cur_layer_acc.push_back(acc);
    }
    if (cur_layer_acc.back() - max_deceleration > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_deceleration);
    } else {
      cur_layer_acc.back() = max_deceleration;
    }
    acc_matrix.push_back(std::move(cur_layer_acc));
  }
  // Make sure stop speed profile can be sampled.
  const double stoppable_decel = -cur_v / unit_t;
  if (stoppable_decel > max_deceleration) {
    acc_matrix.front().push_back(stoppable_decel);
  }

  return acc_matrix;
}
}  // namespace

GriddedSvtGraph::GriddedSvtGraph(
    const StGraphData* st_graph_data, const TrajectoryPoint& init_point,
    const SpeedFinderParamsProto* speed_finder_params, double speed_cap,
    std::vector<StBoundaryWithDecision> st_boundaries_wd)
    : st_graph_data_(st_graph_data),
      st_boundaries_with_decision_(std::move(st_boundaries_wd)),
      init_point_(init_point),
      speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      dp_params_(&speed_finder_params->sampling_dp_speed_params()),
      dp_svt_cost_(speed_finder_params_, st_graph_data_->total_time(),
                   st_graph_data_->path_length(),
                   &st_boundaries_with_decision()),
      total_duration_t_(st_graph_data_->total_time()),
      unit_t_(dp_params_->unit_t()),
      total_length_s_(st_graph_data_->path_length()),
      total_length_v_(speed_cap + dp_params_->speed_exceeding_cap()),
      unit_v_(dp_params_->unit_v()) {
  dp_svt_cost_.SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunctionFromProto(
      speed_finder_params->follow_distance_rel_speed_plf()));
}

absl::Status GriddedSvtGraph::InitLayers() {
  // Total t and v are always fixed value, but total s is dynamic, so sample t
  // and v with fixed step, but change s step length according to total s.
  unit_s_ = std::max(total_length_s_ / dp_params_->s_dimension_size(), 0.15);
  unit_inv_t_ = 1.0 / unit_t_;
  unit_inv_s_ = 1.0 / unit_s_;
  unit_inv_v_ = 1.0 / unit_v_;

  // Calculate dimension size.
  dimension_t_ = CeilToInt(total_duration_t_ * unit_inv_t_) + 1;
  dimension_grid_v_ = CeilToInt(total_length_v_ * unit_inv_v_);
  dimension_grid_s_ = CeilToInt(total_length_s_ * unit_inv_s_);

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    return absl::InternalError("unit_t is smaller than the kDoubleEpsilon.");
  }
  if (unit_v_ < kDoubleEpsilon) {
    return absl::InternalError("unit_v is smaller than the kDoubleEpsilon.");
  }
  if (unit_s_ < kDoubleEpsilon) {
    return absl::InternalError("unit_s is smaller than the kDoubleEpsilon.");
  }

  // Sanity check on s,v,t dimension size.
  if (dimension_grid_s_ < 1 || dimension_grid_v_ < 1 || dimension_t_ < 1) {
    return absl::InternalError(absl::StrFormat(
        "Sampling-dp discretized space dimension error, s:%d, v:%d, t:%d",
        dimension_grid_s_, dimension_grid_v_, dimension_t_));
  }

  // Set std::nullopt.
  layers_ = std::vector<std::vector<std::vector<std::optional<SvtGraphPoint>>>>(
      dimension_t_,
      std::vector<std::vector<std::optional<SvtGraphPoint>>>(
          dimension_grid_s_, std::vector<std::optional<SvtGraphPoint>>(
                                 dimension_grid_v_, std::nullopt)));

  s_knots_.clear();
  s_knots_.reserve(dimension_grid_s_ + 1);
  v_knots_.clear();
  v_knots_.reserve(dimension_grid_v_ + 1);
  t_knots_.clear();
  t_knots_.reserve(dimension_t_);

  // TODO(shaojie): Sampling by density.
  double cur_s = 0.0;
  for (int i = 0; i < dimension_grid_s_; ++i, cur_s += unit_s_) {
    s_knots_.push_back(cur_s);
  }
  s_knots_.push_back(total_length_s_);

  double cur_v = 0.0;
  for (int i = 0; i < dimension_grid_v_; ++i, cur_v += unit_v_) {
    v_knots_.push_back(cur_v);
  }
  v_knots_.push_back(total_length_v_);

  double cur_t = 0.0;
  for (int i = 0; i < dimension_t_ - 1; ++i, cur_t += unit_t_) {
    t_knots_.push_back(cur_t);
  }
  t_knots_.push_back(total_duration_t_);

  acc_matrix_ = GenerateAccMatrix(dp_params_->max_acceleration(),
                                  dp_params_->max_deceleration(),
                                  dp_params_->unit_acc(), total_length_s_,
                                  dimension_t_, unit_t_, init_point_.v());

  return absl::OkStatus();
}

// Return last layer sv grid indices that had points located on when search
// finished.
absl::StatusOr<absl::flat_hash_set<SvGridIndex>>
GriddedSvtGraph::SearchAndReturnFinalLayerPoints(ThreadPool* thread_pool) {
  SCOPED_QTRACE("SearchAndReturnFinalLayerPoints");
  const double cruise_speed = st_graph_data_->cruise_speed();
  const auto& speed_limit_table = st_graph_data_->speed_limit();

  // step 1: Init.
  RETURN_IF_ERROR(InitLayers());

  absl::flat_hash_set<SvGridIndex> cur_layer_indices;
  absl::flat_hash_set<SvGridIndex> next_layer_indices;
  next_layer_indices.reserve(dimension_grid_s_ * dimension_grid_v_);

  std::vector<std::vector<std::vector<SvtGraphPoint>>>
      next_layer_candidate_points(
          dimension_grid_s_,
          std::vector<std::vector<SvtGraphPoint>>(
              dimension_grid_v_, std::vector<SvtGraphPoint>()));
  // This is an experimental value.
  constexpr int kPointsNumPerGrid = 15;
  for (auto& row : next_layer_candidate_points) {
    for (auto& grid : row) {
      grid.reserve(kPointsNumPerGrid);
    }
  }

  double cur_t = 0.0;
  // t-level.
  for (int i = 0; i < layers_.size() - 1; ++i, cur_t += unit_t_) {
    auto& cur_layer = layers_[i];
    auto& next_layer = layers_[i + 1];
    const double next_t = cur_t + unit_t_;
    const int next_point_index_t = i + 1;
    next_layer_indices.clear();

    // Only one point need to expand in the first layer(t).
    if (i == 0) {
      if (init_point_.v() > total_length_v_) {
        return absl::InternalError(
            "Init point speed exceeds total length speed.");
      }
      if (init_point_.s() > total_length_s_) {
        return absl::InternalError("Init point s exceeds total length s.");
      }
      const int init_point_grid_index_v =
          FloorToInt(init_point_.v() * unit_inv_v_);
      layers_[0][0][init_point_grid_index_v] =
          std::make_optional<SvtGraphPoint>(
              /*grid_index_s=*/0, init_point_grid_index_v, /*index_t=*/0,
              SvtPoint(init_point_.s(), init_point_.v(), init_point_.t()));
      layers_[0][0][init_point_grid_index_v]->set_total_cost(0.0);

      cur_layer_indices.emplace(/*grid_index_s=*/0, init_point_grid_index_v);

      const auto st_boundary_decisions =
          dp_svt_cost_.GetStBoundaryDecisionsForInitPoint(
              *layers_[0][0][init_point_grid_index_v]);
      layers_[0][0][init_point_grid_index_v]->UpdateStBoundaryDecisions(
          st_boundary_decisions);
    }

    // step 2: Expand by sampling method.

    for (auto& row : next_layer_candidate_points) {
      for (auto& grid : row) {
        grid.clear();
      }
    }

    for (const auto& index : cur_layer_indices) {
      // SvGrid is a virtual GriddedSvtPoint, we can find all SvGrid by
      // a fixed step, and update params.
      auto& sv_grid = cur_layer[index.grid_index_s_][index.grid_index_v_];
      if (!sv_grid.has_value()) {
        return absl::InternalError("Current sv grid not existing.");
      }
      ExpandToNextLayer(i, next_t, next_point_index_t, speed_limit_table,
                        cruise_speed, init_point_.v(), *sv_grid,
                        &next_layer_candidate_points, &next_layer_indices);
    }

    // step 3: Update SvGrids.
    // TODO(shaojie): Parallel for.
    for (const auto& next_index : next_layer_indices) {
      const auto& grid_candidate_points =
          next_layer_candidate_points[next_index.grid_index_s_]
                                     [next_index.grid_index_v_];
      auto& optimal_candidate =
          next_layer[next_index.grid_index_s_][next_index.grid_index_v_];
      ASSIGN_OR_RETURN(auto find_res,
                       FindOptimalCandidate(grid_candidate_points));
      optimal_candidate = std::move(find_res);
      optimal_candidate->pre_point()->set_next_point(&*optimal_candidate);
    }

    // Update indices.
    cur_layer_indices.swap(next_layer_indices);
  }

  return cur_layer_indices;
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeed(
    SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("FindOptimalPreliminarySpeed");

  ASSIGN_OR_RETURN(const auto final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));

  const auto preliminary_speed_with_cost =
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices);
  if (!preliminary_speed_with_cost.ok()) {
    return preliminary_speed_with_cost.status();
  } else {
    *preliminary_speed =
        std::move(preliminary_speed_with_cost->preliminary_speed);
  }

  if (FLAGS_add_sampling_dp_speed_profiles_to_debug) {
    AddAllSpeedProfilesToDebug(final_layer_indices, sampling_dp_debug);
  }
  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeedWithCost(
    PreliminarySpeedWithCost* preliminary_speed_with_cost,
    SamplingDpDebugProto* sampling_dp_debug, ThreadPool* thread_pool) {
  SCOPED_QTRACE("FindOptimalPreliminarySpeedWithCost");

  ASSIGN_OR_RETURN(const absl::flat_hash_set<SvGridIndex> final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));
  ASSIGN_OR_RETURN(
      *preliminary_speed_with_cost,
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices));
  if (FLAGS_add_sampling_dp_speed_profiles_to_debug) {
    AddAllSpeedProfilesToDebug(final_layer_indices, sampling_dp_debug);
  }
  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::GenerateSamplingDpSpeedProfileCandidateSet(
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
    SamplingDpDebugProto* sampling_dp_debug,
    InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("GenerateSamplingDpSpeedProfileCandidateSet");

  ASSIGN_OR_RETURN(const absl::flat_hash_set<SvGridIndex> final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));
  *candidate_speed_profiles =
      SampleSpeedProfilesFromSamplingDp(final_layer_indices);
  if (FLAGS_add_sampling_dp_speed_profiles_to_debug) {
    for (const auto& speed_profile : *candidate_speed_profiles) {
      speed_profile.preliminary_speed.ToProto(
          candidate_set_debug->add_speed_profile());
    }
    AddAllSpeedProfilesToDebug(final_layer_indices, sampling_dp_debug);
  }
  return absl::OkStatus();
}

std::vector<SvtGraphPoint> GriddedSvtGraph::ExpandByConstAccModel(
    int cur_layer_index, double next_t, int next_point_index_t,
    const SpeedLimit& speed_limit, double cruise_speed, double init_speed,
    const SvtGraphPoint& cur_point) {
  using StBoundaryDecision = SvtGraphPoint::StBoundaryDecision;

  std::vector<SvtGraphPoint> candidate_points;
  candidate_points.reserve(acc_matrix_[cur_layer_index].size());
  const double cur_s = cur_point.point().s();
  const double cur_v = cur_point.point().v();
  double prev_speed_limit = speed_limit.GetSpeedLimitByS(cur_s);
  for (const double acc : acc_matrix_[cur_layer_index]) {
    const double next_s = cur_s + cur_v * unit_t_ + 0.5 * acc * Sqr(unit_t_);
    const double next_v = cur_v + acc * unit_t_;

    if (next_v > (total_length_v_ - kDoubleEpsilon) ||
        next_s > (total_length_s_ - kDoubleEpsilon) || next_v < 0.0 ||
        next_s < 0.0 || next_s < cur_s) {
      continue;
    }

    const int next_point_grid_index_s = FloorToInt(next_s * unit_inv_s_);
    const int next_point_grid_index_v = FloorToInt(next_v * unit_inv_v_);

    auto next_point =
        SvtGraphPoint(next_point_grid_index_s, next_point_grid_index_v,
                      next_point_index_t, SvtPoint(next_s, next_v, next_t));
    next_point.set_spatial_potential_cost(
        dp_svt_cost_.GetSpatialPotentialCost(next_s));
    next_point.set_vertex_cost(next_point.spatial_potential_cost());

    const double curr_speed_limit = speed_limit.GetSpeedLimitByS(next_s);
    prev_speed_limit = next_point_index_t == 1
                           ? curr_speed_limit
                           : std::min(prev_speed_limit, curr_speed_limit);

    const double average_speed = 0.5 * (cur_v + next_v);
    const double speed_limit_cost =
        dp_svt_cost_.GetSpeedLimitCost(average_speed, prev_speed_limit);
    const double reference_speed_cost =
        dp_svt_cost_.GetReferenceSpeedCost(average_speed, cruise_speed);
    const double accel_cost = dp_svt_cost_.GetAccelCost(acc);
    double object_cost = 0.0;
    std::vector<StBoundaryDecision> st_boundary_decisions;
    st_boundary_decisions.reserve(st_boundaries_with_decision_.size());
    dp_svt_cost_.GetStBoundaryCostAndDecisions(cur_point, next_point,
                                               init_speed, &object_cost,
                                               &st_boundary_decisions);
    const double edge_cost =
        speed_limit_cost + reference_speed_cost + accel_cost + object_cost;
    const double total_cost =
        next_point.vertex_cost() + edge_cost + cur_point.total_cost();

    next_point.set_speed_limit_cost(speed_limit_cost);
    next_point.set_reference_speed_cost(reference_speed_cost);
    next_point.set_accel_cost(accel_cost);
    next_point.set_object_cost(object_cost);
    next_point.set_edge_cost(edge_cost);
    next_point.set_total_cost(total_cost);
    next_point.set_pre_point(const_cast<SvtGraphPoint*>(&cur_point));
    next_point.set_acc_from_pre_point(acc);
    next_point.UpdateStBoundaryDecisions(st_boundary_decisions);
    candidate_points.push_back(std::move(next_point));
  }

  return candidate_points;
}

void GriddedSvtGraph::ExpandToNextLayer(
    int cur_layer_index, double next_t, int next_point_index_t,
    const SpeedLimit& speed_limit, double cruise_speed, double init_speed,
    const SvtGraphPoint& cur_point,
    std::vector<std::vector<std::vector<SvtGraphPoint>>>*
        next_layer_candidate_points,
    absl::flat_hash_set<SvGridIndex>* index_range) {
  // Note: next_layer_candidate_points and index_range must be guaranteed to be
  // empty by the call site.
  auto candidate_points =
      ExpandByConstAccModel(cur_layer_index, next_t, next_point_index_t,
                            speed_limit, cruise_speed, init_speed, cur_point);

  for (auto& candidate_point : candidate_points) {
    const int index_s = candidate_point.grid_index_s();
    const int index_v = candidate_point.grid_index_v();
    index_range->emplace(index_s, index_v);
    (*next_layer_candidate_points)[index_s][index_v].push_back(
        std::move(candidate_point));
  }
}

void GriddedSvtGraph::AddAllSpeedProfilesToDebug(
    const absl::flat_hash_set<SvGridIndex>& final_layer_index_range,
    SamplingDpDebugProto* sampling_dp_debug) {
  SCOPED_QTRACE("AddAllSpeedProfilesToDebug");

  const auto get_speed_profile = [](const SvtGraphPoint* cur_point) {
    SpeedProfileDebugProto speed_profile;
    // Backtracking to find the optimal speed profile.
    while (cur_point != nullptr) {
      auto* point = speed_profile.add_svt_graph_points();
      auto* speed_point = point->mutable_speed_point();
      speed_point->set_s(cur_point->point().s());
      speed_point->set_v(cur_point->point().v());
      speed_point->set_a(cur_point->acc_from_pre_point());
      speed_point->set_t(cur_point->point().t());
      point->set_speed_limit_cost(cur_point->speed_limit_cost());
      point->set_reference_speed_cost(cur_point->reference_speed_cost());
      point->set_accel_cost(cur_point->accel_cost());
      point->set_object_cost(cur_point->object_cost());
      point->set_spatial_potential_cost(cur_point->spatial_potential_cost());
      point->set_total_cost(cur_point->total_cost());
      cur_point = cur_point->pre_point();
    }
    // If debug only, no need to reverse.
    return speed_profile;
  };

  for (const auto& final_layer_index : final_layer_index_range) {
    const auto& end_point = layers_.back()[final_layer_index.grid_index_s_]
                                          [final_layer_index.grid_index_v_];
    *sampling_dp_debug->add_speed_profiles() = get_speed_profile(&*end_point);
  }

  for (const auto& layer : layers_) {
    const auto& end_s_row = layer.back();
    for (const auto& end_point : end_s_row) {
      if (!end_point.has_value() || !end_point->next_point().empty()) {
        continue;
      }
      *sampling_dp_debug->add_speed_profiles() = get_speed_profile(&*end_point);
    }
  }
}

absl::StatusOr<PreliminarySpeedWithCost>
GriddedSvtGraph::GetSpeedProfileAndCompleteStBoundariesWithDecision(
    const absl::flat_hash_set<SvGridIndex>& final_layer_index_range) {
  SCOPED_QTRACE("GetSpeedProfileAndCompleteStBoundariesWithDecision");

  double min_cost = kInf;
  const SvtGraphPoint* best_end_point = nullptr;

  for (const auto& final_layer_index : final_layer_index_range) {
    const auto& end_point = layers_.back()[final_layer_index.grid_index_s_]
                                          [final_layer_index.grid_index_v_];
    if (end_point->total_cost() < min_cost) {
      best_end_point = &*end_point;
      min_cost = end_point->total_cost();
    }
  }

  for (const auto& layer : layers_) {
    const auto& end_s_row = layer.back();
    for (const auto& end_point : end_s_row) {
      if (!end_point.has_value() || !end_point->next_point().empty()) {
        continue;
      }
      if (end_point->total_cost() < min_cost) {
        best_end_point = &*end_point;
        min_cost = end_point->total_cost();
      }
    }
  }

  // In some extreme cases, sampling dp cannot find a feasible trajectory, see a
  // typical
  // case:https://docs.google.com/document/d/14HuoJIIXCxsVdWxnz_ttt0oUL_YK8NmKK-GimPg5vOY/edit
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    QEVENT("yumeng", "sampling_dp_failed",
           [&](QEvent* qevent) { qevent->AddField("msg", msg); });
    return absl::InternalError(msg);
  }

  VLOG(2) << "Best end point cost: " << best_end_point->total_cost();
  std::vector<SpeedPoint> speed_profile;
  speed_profile.reserve(dimension_t_);
  const SvtGraphPoint* cur_point = best_end_point;

  // Backtracking to find the optimal speed profile.
  while (cur_point != nullptr) {
    VLOG(2) << "Time: " << cur_point->point().t();
    VLOG(2) << "S: " << cur_point->point().s();
    VLOG(2) << "V: " << cur_point->point().v();
    // Get speed point.
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_v(cur_point->point().v());
    speed_point.set_a(cur_point->acc_from_pre_point());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(std::move(speed_point));
    for (auto& st_boundary_with_decision : st_boundaries_with_decision_) {
      if (st_boundary_with_decision.decision_type() !=
          StBoundaryProto::UNKNOWN) {
        continue;
      }
      const auto decision =
          cur_point->GetStBoundaryDecision(st_boundary_with_decision.id());
      if (!decision.ok()) {
        continue;
      } else {
        if (*decision == StBoundaryProto::UNKNOWN) {
          return absl::InternalError(
              "Current point decision should not be UNKNOWN.");
        }
        // Modify decision in place.
        st_boundary_with_decision.set_decision_type(*decision);
        st_boundary_with_decision.set_decision_reason(
            StBoundaryProto::SAMPLING_DP);
        st_boundary_with_decision.set_decision_info("decided by sampling dp");
      }
    }
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());
  // Set next point `acc_from_pre_point` as current point acc. Keep last point
  // acc to avoid sharp jerk.
  for (int i = 1; i < speed_profile.size() - 1; ++i) {
    speed_profile[i].set_a(speed_profile[i + 1].a());
  }
  speed_profile.front().set_a(init_point_.a());

  return PreliminarySpeedWithCost(best_end_point->total_cost(),
                                  SpeedVector(std::move(speed_profile)));
}

std::vector<PreliminarySpeedWithCost>
GriddedSvtGraph::SampleSpeedProfilesFromSamplingDp(
    const absl::flat_hash_set<SvGridIndex>& final_layer_indices) {
  SCOPED_QTRACE("SampleSpeedProfilesFromSamplingDp");

  // Sample minimal cost speed profile in s dimension at final layer.
  // <s_dimension, BestEndPoint>
  absl::flat_hash_map<int, BestEndPoint> final_layer_points;
  constexpr int kSDimensionSampleInterval = 4;
  for (const auto& index : final_layer_indices) {
    const auto& end_point =
        layers_.back()[index.grid_index_s_][index.grid_index_v_];
    const int sample_index = index.grid_index_s_ / kSDimensionSampleInterval;
    if (final_layer_points.find(sample_index) != final_layer_points.end()) {
      if (end_point->total_cost() < final_layer_points[sample_index].cost) {
        final_layer_points[sample_index] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    } else {
      final_layer_points[sample_index] =
          BestEndPoint(&*end_point, end_point->total_cost());
    }
  }

  // Select minimal cost speed profile for last row(s) of every layer(t).
  // <t_dimension, BestEndPoint>
  absl::flat_hash_map<int, BestEndPoint> layers_final_point;
  for (int i = 0; i < layers_.size() - 1; ++i) {
    for (const auto& end_point : layers_[i].back()) {
      if (!end_point.has_value() || !end_point->next_point().empty()) {
        continue;
      }
      if (layers_final_point.find(i) != layers_final_point.end()) {
        if (end_point->total_cost() < layers_final_point[i].cost) {
          layers_final_point[i] =
              BestEndPoint(&*end_point, end_point->total_cost());
        }
      } else {
        layers_final_point[i] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    }
  }

  const auto get_speed_profile = [](double init_point_a,
                                    const SvtGraphPoint* cur_point) {
    std::vector<SpeedPoint> speed_profile;
    speed_profile.reserve(cur_point->index_t() + 1);
    // Backtracking to find the optimal speed profile.
    while (cur_point != nullptr) {
      SpeedPoint speed_point;
      speed_point.set_s(cur_point->point().s());
      speed_point.set_v(cur_point->point().v());
      speed_point.set_a(cur_point->acc_from_pre_point());
      speed_point.set_t(cur_point->point().t());
      speed_profile.push_back(std::move(speed_point));
      cur_point = cur_point->pre_point();
    }
    std::reverse(speed_profile.begin(), speed_profile.end());

    // Set next point `acc_from_pre_point` as current point acc. Keep last point
    // acc to avoid sharp jerk.
    for (int i = 1; i < speed_profile.size() - 1; ++i) {
      speed_profile[i].set_a(speed_profile[i + 1].a());
    }
    speed_profile.front().set_a(init_point_a);

    return speed_profile;
  };

  std::vector<PreliminarySpeedWithCost> speed_profiles;
  speed_profiles.reserve(layers_final_point.size() + final_layer_points.size());

  for (const auto& [_, best_end_point] : final_layer_points) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_point_.a(), best_end_point.point)));
  }
  for (const auto& [_, best_end_point] : layers_final_point) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_point_.a(), best_end_point.point)));
  }

  return speed_profiles;
}

}  // namespace qcraft::planner
