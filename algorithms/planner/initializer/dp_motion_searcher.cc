#include "onboard/planner/initializer/dp_motion_searcher.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto/pnc_scenario.pb.h"
#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto_converter.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/container/small_set.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/initializer/candidate_complete_motion_form.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/expert_complete_motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

namespace {
// DP search params
// TODO(xiangjun): move to planner config.
// If vehicle speed less than this threshold, we can set a stationary motion
// for it.
constexpr double kCanSetToZeroSpeed = 1.0;
// If vehicle speed less than this threshold and search failed, we can set a
// stationary motion for it.
constexpr double kSearchFailedCanSetToZeroSpeed = 1.5;
constexpr double kCanSetToZeroTrajLength = 4.0;
constexpr std::array<double, 9> kAccelerationSamplePoints = {
    -4.0, -3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
constexpr double kDpDiscreteSpeedSampleStep = 3.0;  // m/s
constexpr double kDpDiscreteTimeSampleStep = 2.5;   // second.
const int kDpDiscreteTimeHorizon = CeilToInt(
    kTrajectoryTimeHorizon / kDpDiscreteTimeSampleStep);  // number of steps.
constexpr double kMinSpeedForFinalCost = 3.0;
constexpr int kConstVelSampleLayerSizeThreshold = 5;

// Discrete motion samples on geometry node, used to get opt_motion_edge.
struct DpMotionSample {
  int v_discrete;  // discrete speed step
  int t_discrete;  // discrete time step
  DpMotionSample(double v, double t)
      : v_discrete(std::max(0, RoundToInt(v / kDpDiscreteSpeedSampleStep))),
        t_discrete(std::clamp(RoundToInt(t / kDpDiscreteTimeSampleStep), 0,
                              kDpDiscreteTimeHorizon)) {}

  friend bool operator==(const DpMotionSample &lhs, const DpMotionSample &rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const DpMotionSample &ms) {
    return H::combine(std::move(h), ms.v_discrete, ms.t_discrete);
  }
};

// Candidate trajectory info.
struct TrajInfo {
  MotionEdgeIndex idx;
  double total_cost;
  std::vector<double> feature_costs;
  std::vector<ApolloTrajectoryPointProto> traj_points;
};

struct BestEdgeInfo {
  MotionEdgeIndex idx;
  double total_cost = 0.0;
  bool is_created_stationary_motion = false;
};

std::vector<double> AddCost(absl::Span<const double> vec1,
                            absl::Span<const double> vec2) {
  const int vec1_size = vec1.size();
  std::vector<double> res(vec1_size);
  QCHECK_EQ(vec1_size, vec2.size());
  for (int i = 0; i < vec1_size; ++i) {
    res[i] = vec1[i] + vec2[i];
  }
  return res;
}

template <typename T>
inline std::string PrintVector(const std::vector<T> &vec) {
  return absl::StrJoin(vec, ", ");
}

void ComputeCost(const CostProvider &cost_provider,
                 DpMotionInfo *ptr_motion_info) {
  auto &dp_motion_info = *ptr_motion_info;
  dp_motion_info.costs.resize(cost_provider.weights().size());
  cost_provider.ComputeDpCost(dp_motion_info.start_t,
                              dp_motion_info.motion_form,
                              absl::MakeSpan(dp_motion_info.costs));
}

void ComputeCostByName(const CostProvider &cost_provider,
                       const std::string name, DpMotionInfo *ptr_motion_info) {
  auto &dp_motion_info = *ptr_motion_info;
  cost_provider.ComputeDpCostByName(name, dp_motion_info.start_t,
                                    dp_motion_info.motion_form,
                                    absl::MakeSpan(dp_motion_info.costs));
}

// Sample by acceleration.
void SampleDynamicMotions(
    MotionNodeIndex motion_node_idx, MotionEdgeIndex prev_motion_edge_idx,
    const MotionGraph &motion_graph, const GeometryEdge &geom_edge,
    const PlannerParamsProto &planner_params, const CostProvider &cost_provider,
    const MotionGraphCache *cost_cache, bool sample_const_v,
    std::vector<DpMotionInfo> *ptr_motions,
    std::vector<NewCacheInfo> *new_motion_forms_cache,
    ThreadPool *thread_pool) {
  const auto motion_state = motion_graph.GetMotionNode(motion_node_idx).state;
  const double v0 = motion_state.v;
  const double t0 = motion_state.t;
  auto &motions = *ptr_motions;
  // expand motion and get cost at the same time. if cannot get cost from cache,
  // record this idx and compute cost outside this function.

  // Create keys for the cost fetching.
  absl::flat_hash_set<MotionEdgeKey> unrepeated_keys;
  const auto expand_motion_by_a = [&unrepeated_keys, &geom_edge, t0](
                                      double v0, double a0) {
    unrepeated_keys.emplace(MotionEdgeKey(a0, v0, t0, geom_edge.index));
  };

  QCHECK_GE(v0, 0.0);
  const double a_max =
      planner_params.motion_constraint_params().max_acceleration();
  const double a_min =
      planner_params.motion_constraint_params().max_deceleration();
  QCHECK_LT(a_min, 0.0);
  QCHECK_GT(a_max, 0.0);
  const double v_limit = Mph2Mps(
      planner_params.motion_constraint_params().default_speed_limit());  // m/s
  const double reciprocal_s = 1.0 / geom_edge.geometry->length();        // 1/m
  const double pos_a_limit = (Sqr(v_limit) - Sqr(v0)) * reciprocal_s * 0.5;
  const double stop_a = -Sqr(v0) * reciprocal_s * 0.5;

  const auto a_lower = std::max(a_min, stop_a);
  const auto a_upper = std::min(pos_a_limit, a_max);
  SmallSet<double, kAccelerationSamplePoints.size() + 3> acc_samples;
  if (!sample_const_v) {
    const auto a_begin =
        std::lower_bound(kAccelerationSamplePoints.begin(),
                         kAccelerationSamplePoints.end(), a_lower);
    const auto a_end =
        std::lower_bound(a_begin, kAccelerationSamplePoints.end(), a_upper);
    // Sample the valid accelerations on the sampling grid.
    for (auto it = a_begin; it != a_end; ++it) {
      acc_samples.insert(*it);
    }
  }
  acc_samples.insert(a_upper);
  acc_samples.insert(a_lower);
  acc_samples.insert(a_min);
  acc_samples.insert(0.0);
  for (const double acc : acc_samples) {
    expand_motion_by_a(v0, acc);
  }

  // Collect costs from cache or calculate costs.

  // Prepare containers.
  std::vector<MotionEdgeKey> keys;
  keys.reserve(unrepeated_keys.size());
  for (auto &key : unrepeated_keys) {
    keys.emplace_back(std::move(key));
  }
  motions.reserve(keys.size());
  for (int i = 0; i < keys.size(); ++i) {
    motions.emplace_back(
        DpMotionInfo({.key = keys[i],
                      .sum_cost = 0.0,
                      .start_t = t0,
                      .prev_motion_edge_index = prev_motion_edge_idx,
                      .end_geometry_node_index = geom_edge.end,
                      .motion_form = nullptr,
                      .geometry_edge_index = geom_edge.index}));
  }
  std::vector<int> failed_idx;
  failed_idx.reserve(keys.size());
  // Fill in the calculated ones.
  cost_cache->BatchGetOrFail(keys, &motions, &failed_idx);

  // Calculated all costs and create new MotionForms for the failed_idx motion
  // edges given the cost_provider.
  std::vector<std::unique_ptr<MotionForm>> new_motion_forms;
  new_motion_forms.reserve(failed_idx.size());
  for (int i = 0; i < failed_idx.size(); ++i) {
    const int cur_failed_idx = failed_idx[i];
    const auto &key = keys[cur_failed_idx];
    new_motion_forms.emplace_back(std::make_unique<ConstAccelMotion>(
        key.v0(), key.a0(), geom_edge.geometry));
    motions[cur_failed_idx].motion_form = new_motion_forms[i].get();
  }

  for (int i = 0; i < failed_idx.size(); ++i) {
    const int cur_failed_idx = failed_idx[i];
    ComputeCost(cost_provider, &motions[cur_failed_idx]);
    MotionEdgeCache new_cache(
        {.ptr_motion_form = std::move(new_motion_forms[i]),
         .costs = motions[cur_failed_idx].costs});
    new_motion_forms_cache->push_back(NewCacheInfo{
        .key = keys[cur_failed_idx], .cache = std::move(new_cache)});
  }

  for (int i = 0; i < keys.size(); ++i) {
    if (std::find(failed_idx.begin(), failed_idx.end(), i) ==
        failed_idx.end()) {
      ComputeCostByName(cost_provider, "dp_leading_object", &motions[i]);
    }
    motions[i].sum_cost = absl::c_accumulate(motions[i].costs, 0.0);
  }
}

// Special handling for motion expansion from start node (planning start state).
std::vector<DpMotionInfo> ExpandStartMotionEdges(
    GeometryNodeIndex geom_node_idx, MotionNodeIndex motion_node_idx,
    const GeometryGraph &geometry, const MotionGraph &motion_graph,
    const PlannerParamsProto &planner_params, const CostProvider &cost_provider,
    std::vector<NewCacheInfo> *new_motion_forms,
    const MotionGraphCache *cost_cache, ThreadPool *thread_pool) {
  SCOPED_QTRACE("ExpandStartMotionEdges");
  const auto &outgoing_edge_idxs = geometry.GetOutgoingEdges(geom_node_idx);
  std::vector<DpMotionInfo> motions;  // Candidate motions (to consider).
  motions.reserve((kAccelerationSamplePoints.size() + 3) *
                  outgoing_edge_idxs.size());
  for (const auto &outgoing_edge_idx : outgoing_edge_idxs) {
    const auto &geom_edge = geometry.GetEdge(outgoing_edge_idx);
    if (!geometry.IsActive(outgoing_edge_idx)) {
      continue;
    }
    // No previous edge index, set to invalid.
    // Sample start from this motion node, based on this geom_edge, sampling
    // by different accelerations.
    std::vector<DpMotionInfo> motions_per_edge;
    SampleDynamicMotions(motion_node_idx,
                         MotionEdgeVector<MotionEdge>::kInvalidIndex,
                         motion_graph, geom_edge, planner_params, cost_provider,
                         cost_cache, /*sample_const_v=*/false,
                         &motions_per_edge, new_motion_forms, thread_pool);
    for (int i = 0; i < motions_per_edge.size(); i++) {
      motions.emplace_back(std::move(motions_per_edge[i]));
    }
  }
  return motions;
}

// Expand motions from an intermediate geometry node.
std::vector<DpMotionInfo> ExpandMotionEdges(
    GeometryNodeIndex geom_node_idx,
    const std::vector<MotionEdgeIndex> &motion_edge_idxes,
    const GeometryGraph &geometry, const MotionGraph &motion_graph,
    const PlannerParamsProto &planner_params, const CostProvider &cost_provider,
    bool sample_const_v, std::vector<NewCacheInfo> *new_motion_forms,
    const MotionGraphCache *cost_cache, ThreadPool *thread_pool) {
  SCOPED_QTRACE("ExpandMotionEdges");

  const auto &outgoing_edge_idxs = geometry.GetOutgoingEdges(geom_node_idx);
  std::vector<DpMotionInfo> motions;
  motions.reserve((kAccelerationSamplePoints.size() + 2) *
                  outgoing_edge_idxs.size() * motion_edge_idxes.size());
  for (const auto &outgoing_edge_idx : outgoing_edge_idxs) {
    const auto &geom_edge = geometry.GetEdge(outgoing_edge_idx);
    if (!geometry.IsActive(outgoing_edge_idx)) {
      continue;
    }
    for (const auto &motion_edge_idx : motion_edge_idxes) {
      // Motion_edge_idexes are the motions to expand (decide by certain
      // conditions) which are terminating at this particular geometrty node.
      std::vector<DpMotionInfo> motions_per_edge;
      const auto motion_node_idx =
          motion_graph.GetMotionEdge(motion_edge_idx).end;
      SampleDynamicMotions(motion_node_idx, motion_edge_idx, motion_graph,
                           geom_edge, planner_params, cost_provider, cost_cache,
                           sample_const_v, &motions_per_edge, new_motion_forms,
                           thread_pool);
      for (int i = 0; i < motions_per_edge.size(); i++) {
        motions.emplace_back(std::move(motions_per_edge[i]));
      }
    }
  }
  return motions;
}

double GetLeadingObjectsEndMinS(const SpacetimeTrajectoryManager &st_mgr,
                                const DrivePassage &drive_passage,
                                const std::vector<std::string> &leading_objs,
                                double sdc_length) {
  double min_s = std::numeric_limits<double>::max();
  for (const auto &lead_obj : leading_objs) {
    const auto states = st_mgr.FindTrajectoryById(lead_obj)->states();
    const auto fbox = drive_passage.QueryFrenetBoxAt(states.back().box);
    if (!fbox.ok()) {
      continue;
    }
    min_s = std::min(min_s, fbox->s_min);
  }
  return min_s - sdc_length;
}

BestEdgeInfo FindBestEdge(
    const CostProvider &cost_provider, const MotionState &sdc_motion,
    MotionNodeIndex sdc_node_index, const GeometryNodeIndex &sdc_geom_node,
    MotionGraph *mutable_motion_graph, MotionGraphCache *cost_cache,
    MotionEdgeVector<MotionSearchOutput::SearchCost> *ptr_mutable_search_costs,
    std::vector<MotionEdgeIndex> *ptr_mutable_terminated_idxes) {
  SCOPED_QTRACE("FindBestEdge");
  auto &mutable_search_costs = *ptr_mutable_search_costs;
  auto &mutable_terminated_idxes = *ptr_mutable_terminated_idxes;
  MotionEdgeIndex best_final_edge = MotionEdgeVector<MotionEdge>::kInvalidIndex;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0, n = mutable_terminated_idxes.size(); i < n; ++i) {
    const auto terminated_idx = mutable_terminated_idxes[i];
    const auto total_cost = mutable_search_costs[terminated_idx].cost_to_come;
    if (total_cost < min_cost) {
      best_final_edge = terminated_idx;
      min_cost = total_cost;
    }
  }

  // If sdc is at low speed and we cannot find a reasonable motion, create a
  // stationary motion.
  bool is_created_stationary_motion = false;
  if (best_final_edge == MotionEdgeVector<MotionEdge>::kInvalidIndex &&
      sdc_motion.v < kSearchFailedCanSetToZeroSpeed) {
    is_created_stationary_motion = true;
    auto set2zero_sdc_motion = sdc_motion;
    set2zero_sdc_motion.v = 0.0;
    MotionEdgeKey stationary_motion_key = MotionEdgeKey(
        set2zero_sdc_motion.v, set2zero_sdc_motion.a, set2zero_sdc_motion.t,
        GeometryEdgeVector<GeometryEdge>::kInvalidIndex);
    // Feature cost for this stationary motion not initialized.
    const auto end_node_index =
        mutable_motion_graph->AddMotionNode(set2zero_sdc_motion, sdc_geom_node);
    const auto ptr_motion_form_or =
        cost_cache->GetMotionForm(stationary_motion_key);
    MotionForm *ptr_motion_form = nullptr;
    if (!ptr_motion_form_or.ok()) {
      // Stationary motion form not constructed yet.
      std::unique_ptr<MotionForm> stationary_motion =
          std::make_unique<StationaryMotion>(
              kTrajectoryTimeHorizon,
              GeometryState{.xy = sdc_motion.xy,
                            .h = sdc_motion.h,
                            .k = sdc_motion.k,
                            .accumulated_s = sdc_motion.accumulated_s,
                            .l = sdc_motion.l});
      ptr_motion_form = stationary_motion.get();
      DpMotionInfo temp_motion_info{.sum_cost = 0.0,
                                    .start_t = 0.0,
                                    .motion_form = stationary_motion.get()};
      ComputeCost(cost_provider, &temp_motion_info);
      cost_cache->Insert(stationary_motion_key, temp_motion_info.costs,
                         std::move(stationary_motion));
      MotionSearchOutput::SearchCost new_search_cost{
          .feature_cost = temp_motion_info.costs,
          .cost_to_come = absl::c_accumulate(temp_motion_info.costs, 0.0)};
      mutable_search_costs.push_back(new_search_cost);
      min_cost = new_search_cost.cost_to_come;
    } else {
      ptr_motion_form = *ptr_motion_form_or;
      // Motion form and costs should exists in cache at the same time.
      const auto feature_costs =
          cost_cache->GetCosts(stationary_motion_key).value();
      MotionSearchOutput::SearchCost new_search_cost{
          .feature_cost = feature_costs,
          .cost_to_come = absl::c_accumulate(feature_costs, 0.0),
      };
      mutable_search_costs.push_back(new_search_cost);
      min_cost = new_search_cost.cost_to_come;
    }
    const auto motion_edge_index = mutable_motion_graph->AddMotionEdge(
        sdc_node_index, end_node_index, ptr_motion_form, sdc_geom_node,
        /*prev_motion_edge_idx=*/MotionEdgeVector<MotionEdge>::kInvalidIndex);
    mutable_terminated_idxes.push_back(motion_edge_index);
    best_final_edge = motion_edge_index;
  }
  return {.idx = best_final_edge,
          .total_cost = min_cost,
          .is_created_stationary_motion = is_created_stationary_motion};
}

bool CompareCost(const TrajInfo &traj1, const TrajInfo &traj2) {
  return traj1.total_cost < traj2.total_cost;
}

std::vector<TrajInfo> TopKTrajectories(
    absl::Span<const MotionEdgeIndex> terminated_edge_idxes,
    const MotionEdgeVector<MotionSearchOutput::SearchCost> &search_costs,
    int k_top_trajectories) {
  std::vector<TrajInfo> top_k_traj;
  top_k_traj.reserve(terminated_edge_idxes.size());
  for (int i = 0, n = terminated_edge_idxes.size(); i < n; ++i) {
    TrajInfo new_traj = TrajInfo(
        {.idx = terminated_edge_idxes[i],
         .total_cost = search_costs[terminated_edge_idxes[i]].cost_to_come});
    top_k_traj.emplace_back(new_traj);
  }
  std::sort(top_k_traj.begin(), top_k_traj.end(), CompareCost);
  QCHECK_EQ(top_k_traj.size(), terminated_edge_idxes.size());
  if (k_top_trajectories <= top_k_traj.size()) {
    return std::vector<TrajInfo>(top_k_traj.begin(),
                                 top_k_traj.begin() + k_top_trajectories);
  } else {
    return top_k_traj;
  }
}

MotionState PrepareStartMotionNode(
    const GeometryGraph *geometry,
    const std::vector<GeometryNodeIndex> &first_layer,
    const ApolloTrajectoryPointProto &start_point,
    int *start_node_idx_on_first_layer) {
  for (int i = first_layer.size() - 1; i >= 0; --i) {
    // Reverse look up because the first node is always reachable.
    const auto &node = geometry->GetNode(first_layer[i]);
    if (node.reachable) {
      *start_node_idx_on_first_layer = i;
      return MotionState{.xy = node.xy,
                         .h = start_point.path_point().theta(),
                         .k = node.k,
                         .t = 0.0,
                         .v = start_point.v(),
                         .a = start_point.a()};
    }
  }
  return MotionState();  // Should not reach here.
}

void SampledDpMotionEvaluation(
    const MotionSearchInput &input,
    const std::vector<std::string> &leading_objs,
    const MotionEdgeVector<MotionSearchOutput::SearchCost> &search_costs,
    const std::vector<MotionEdgeIndex> &terminated_edge_idxes,
    MotionSearchOutput *const output) {
  for (int i = 0, n = terminated_edge_idxes.size(); i < n; ++i) {
    const auto cur_idx = terminated_edge_idxes[i];

    std::vector<double> feature_costs = search_costs[cur_idx].feature_cost;
    auto cost_provider_weights = output->cost_provider->weights();
    const int cost_provider_weights_size = cost_provider_weights.size();
    for (int i = 0; i < cost_provider_weights_size; ++i) {
      feature_costs[i] = feature_costs[i] / cost_provider_weights[i];
    }

    double weighted_total_cost = search_costs[cur_idx].cost_to_come;

    std::vector<double> weights;
    weights.reserve(cost_provider_weights_size);
    for (int i = 0; i < cost_provider_weights_size; ++i) {
      weights.push_back(cost_provider_weights[i]);
    }

    auto candidate_traj =
        ConstructTrajFromLastEdge(*output->motion_graph, cur_idx);

    auto candidate_evaluation = MotionSearchOutput::TrajectoryEvaluationDumping{
        .weighted_total_cost = weighted_total_cost,
        .dumped_weights = std::move(weights),
        .feature_costs = std::move(feature_costs),
        .traj = std::move(candidate_traj)};

    output->candidates_evaluation.push_back(std::move(candidate_evaluation));
  }
}

absl::Status ExpertTrajectoryEvaluation(
    const MotionSearchInput &input,
    const std::vector<std::string> &leading_objs,
    MotionSearchOutput *const output) {
  // Read local file to load expert future trajectory
  planning_dataset::Scenario scenario;
  if (!file_util::BinaryFileToProto(FLAGS_expert_trajectory_file_path,
                                    &scenario)) {
    QCHECK(false) << absl::StrFormat(
        "Fail to read expert_trajectory_file at %s",
        FLAGS_expert_trajectory_file_path);
  }
  const auto &expert_future_trajectory =
      scenario.tracks(scenario.ego_track_index());
  HistoryBuffer<planning_dataset::ObjectState> buffer;
  for (int i = 0, size = scenario.timestamps_seconds_size(); i < size; ++i) {
    buffer.push_back(scenario.timestamps_seconds(i),
                     expert_future_trajectory.states(i));
  }

  // Interpolate the trajectory from plan_time with kDeltaT
  constexpr double kDeltaT = 0.2;  // s
  const double planner_start_timestamp_sec =
      ToUnixDoubleSeconds(input.plan_time);
  const int horizon = CeilToInt(kTrajectoryTimeHorizon / kDeltaT) + 1;
  for (int i = 0; i < horizon; ++i) {
    scenario.mutable_planner_data()->add_future_timestamps_for_planning(
        planner_start_timestamp_sec + i * kDeltaT);
  }
  auto *ego_future_track_for_planning_ptr =
      scenario.mutable_planner_data()->mutable_ego_future_track_for_planning();
  auto interpolated_buffer = HistoryBuffer<planning_dataset::ObjectState>();
  for (const auto &t :
       scenario.planner_data().future_timestamps_for_planning()) {
    auto *state_ptr = ego_future_track_for_planning_ptr->add_states();
    int least_idx = buffer.GetIndexWithTimeAtLeast(t);
    int most_idx = buffer.GetIndexWithTimeAtMost(t);
    if (least_idx == buffer.size() || most_idx == -1) {
      return absl::FailedPreconditionError(absl::StrFormat(
          "Interpolation t %f outside of collected ego future time range [%f, "
          "%f]",
          t, buffer.front_time(), buffer.back_time()));
    }
    ObjectStateLinearInterpolation(
        buffer[most_idx].second, buffer[least_idx].second,
        buffer[most_idx].first, buffer[least_idx].first, t, state_ptr);
    interpolated_buffer.push_back(t, *state_ptr);
  }

  // Load expert future trajectory into one MotionForm.
  const std::unique_ptr<MotionForm> expert_complete_motion =
      std::make_unique<ExpertCompleteMotion>(input.form_builder,
                                             interpolated_buffer);
  // Evaluate the MotionForm and save the results to initializerOutput.
  const int feature_size = output->cost_provider->weights().size();
  std::vector<double> weighted_feature_costs;
  weighted_feature_costs.resize(feature_size);

  output->cost_provider->ComputeDpCost(0.0, expert_complete_motion.get(),
                                       absl::MakeSpan(weighted_feature_costs));

  double weighted_total_cost = absl::c_accumulate(weighted_feature_costs, 0.0);

  std::vector<double> feature_costs = std::move(weighted_feature_costs);
  auto cost_provider_weights = output->cost_provider->weights();
  const int cost_provider_weights_size = cost_provider_weights.size();
  for (int i = 0; i < cost_provider_weights_size; ++i) {
    feature_costs[i] = feature_costs[i] / cost_provider_weights[i];
  }

  std::vector<double> weights;
  weights.reserve(cost_provider_weights_size);
  for (int i = 0; i < cost_provider_weights_size; ++i) {
    weights.push_back(cost_provider_weights[i]);
  }

  auto expert_traj = ResampleTrajectoryPoints({expert_complete_motion.get()});

  output->expert_evaluation = MotionSearchOutput::TrajectoryEvaluationDumping{
      .weighted_total_cost = weighted_total_cost,
      .dumped_weights = std::move(weights),
      .feature_costs = std::move(feature_costs),
      .traj = std::move(expert_traj),
  };

  return absl::OkStatus();
}

BestEdgeInfo FindBestEdgeFromTopK(
    const MotionSearchInput &input, const std::vector<std::string> leading_objs,
    const MotionEdgeVector<MotionSearchOutput::SearchCost> &search_costs,
    absl::Span<const MotionEdgeIndex> terminated_idxes,
    const MotionState &sdc_motion, MotionNodeIndex sdc_node_index,
    const GeometryNodeIndex &sdc_geom_node, const MotionGraph *motion_graph,
    const CostProvider *cost_provider, ThreadPool *thread_pool) {
  int kTopTrajectories =
      std::min(static_cast<int>(terminated_idxes.size()), 10);
  std::vector<TrajInfo> top_k_traj;
  top_k_traj.reserve(terminated_idxes.size());
  for (int i = 0, n = terminated_idxes.size(); i < n; ++i) {
    top_k_traj.push_back(TrajInfo(
        {.idx = terminated_idxes[i],
         .total_cost = search_costs[terminated_idxes[i]].cost_to_come}));
  }

  std::sort(top_k_traj.begin(), top_k_traj.end(), CompareCost);
  top_k_traj =
      kTopTrajectories <= top_k_traj.size()
          ? std::vector<TrajInfo>(top_k_traj.begin(),
                                  top_k_traj.begin() + kTopTrajectories)
          : top_k_traj;

  std::vector<double> costs(top_k_traj.size(), 0.0);
  ParallelFor(0, top_k_traj.size(), thread_pool, [&](int i) {
    top_k_traj[i].traj_points =
        ConstructTrajFromLastEdge(*motion_graph, top_k_traj[i].idx);
    // Load searched candidate trajectory into one MotionForm.
    const std::unique_ptr<MotionForm> candidate_complete_motion =
        std::make_unique<CandidateCompleteMotion>(input.form_builder,
                                                  top_k_traj[i].traj_points);
    // Evaluate the MotionForm and save the results to initializerOutput.
    const int feature_size = cost_provider->weights().size();
    std::vector<double> weighted_feature_costs;
    weighted_feature_costs.resize(feature_size);

    cost_provider->ComputeDpCost(0.0, candidate_complete_motion.get(),
                                 absl::MakeSpan(weighted_feature_costs));

    costs[i] = absl::c_accumulate(weighted_feature_costs, 0.0);
  });

  MotionEdgeIndex best_final_edge = MotionEdgeVector<MotionEdge>::kInvalidIndex;
  int min_idx = -1;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0, size = costs.size(); i < size; ++i) {
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      best_final_edge = top_k_traj[i].idx;
      min_idx = i;
    }
  }

  if (min_idx != 0) {
    QEVENT_EVERY_N_SECONDS(
        "Jinyun", "initializer_post_evaluation_chose_different_trajectory", 5.0,
        [&](QEvent *qevent) {
          qevent->AddField("post_evaluation_choice_number", min_idx);
        });
  } else {
    QEVENT_EVERY_N_SECONDS(
        "Jinyun", "initializer_post_evaluation_chose_same_trajectory", 5.0,
        [&](QEvent *qevent) {
          qevent->AddField("post_evaluation_choice_number", min_idx);
        });
  }

  return {.idx = best_final_edge,
          .total_cost = min_cost,
          .is_created_stationary_motion = false};
}

}  // namespace

absl::StatusOr<MotionSearchOutput> DpSearchForRawTrajectory(
    const MotionSearchInput &input, ThreadPool *thread_pool,
    InitializerDebugProto *debug_proto) {
  SCOPED_QTRACE("DpSearchForRawTrajectory");

  const DrivePassage *drive_passage = input.drive_passage;
  const SpacetimeTrajectoryManager *st_traj_mgr = input.st_traj_mgr;
  const VehicleGeometryParamsProto *vehicle_geom = input.vehicle_geom;
  const GeometryGraph *geometry = input.geom_graph;
  const ConstraintManager *constraint_manager = input.constraint_manager;
  const PlannerParamsProto *planner_params = input.planner_params;
  const CollisionChecker *collision_checker = input.collision_checker;
  const PathSlBoundary *path_sl = input.sl_boundary;
  const auto &search_configs = *input.search_configs;

  // Prepare motion graph cache.
  MotionGraphCache cost_cache(geometry);

  const double geom_graph_max_s = geometry->GetMaxAccumulatedS();
  const double speed_max_s = kMinSpeedForFinalCost * kTrajectoryTimeHorizon;
  MotionSearchOutput output;  // Final output.
  const auto stop_s_on_drive_passage = ConvertStoplineToStopS(
      constraint_manager->StopLine(), vehicle_geom->length());
  output.ref_speed_table =
      std::make_unique<RefSpeedTable>(*constraint_manager, *st_traj_mgr,
                                      *drive_passage, stop_s_on_drive_passage);
  std::vector<SingleTrajInfo> multi_trajs;
  multi_trajs.reserve(search_configs.size());

  // Multiple traj output start.
  for (const auto &search_config : search_configs) {
    SCOPED_QTRACE("DpSearchForSingleTrajectory");

    VLOG(1) << "--------- Start of One Search-----------";
    const auto start_time = absl::Now();
    SingleTrajInfo traj_output;
    std::vector<std::string> leading_objs;
    for (const auto &front_obj :
         search_config.leading_object_config().front()) {
      leading_objs.push_back(front_obj);
    }
    traj_output.lead_objs = leading_objs;
    traj_output.motion_graph = std::make_unique<XYTMotionGraph>(geometry);

    const double leading_obj_min_s = GetLeadingObjectsEndMinS(
        *st_traj_mgr, *drive_passage, leading_objs, vehicle_geom->length());
    const double max_accumulated_s =
        Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);

    traj_output.cost_provider = std::make_unique<CostProvider>(
        drive_passage, constraint_manager, &search_config,
        stop_s_on_drive_passage, st_traj_mgr, vehicle_geom, collision_checker,
        path_sl, *planner_params, output.ref_speed_table.get(),
        max_accumulated_s);
    // Keep track on finished motions (reached max trajectory time length).
    std::vector<MotionEdgeIndex> terminated_edge_idxes;
    // Keep track on best motions per GeometryNode.
    GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>
        opt_motion_samples;
    opt_motion_samples.resize(geometry->nodes().size());
    // Keep track on "expandable" edges (currently best edges) per GeometryNode.
    GeometryNodeVector<std::vector<MotionEdgeIndex>> motions_to_expand;
    motions_to_expand.resize(geometry->nodes().size());
    // Keep track on optimal costs.
    MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;
    // Prepare start node
    const auto &nodes_layers = geometry->nodes_layers();
    int start_node_idx_on_first_layer = 0;
    MotionState sdc_motion =
        PrepareStartMotionNode(geometry, nodes_layers[0], *input.start_point,
                               &start_node_idx_on_first_layer);
    const auto sdc_node_idx = traj_output.motion_graph->AddMotionNode(
        sdc_motion, nodes_layers[0][start_node_idx_on_first_layer]);
    // Main loop of DP search once a leading obj is specified.
    int motion_count = 0;
    for (int cur_layer_idx = 0, num_layer = nodes_layers.size();
         cur_layer_idx < num_layer; ++cur_layer_idx) {
      SCOPED_QTRACE("DpSearchOnLayer");

      const auto &nodes_layer = nodes_layers[cur_layer_idx];
      // 1. insert best so far motion nodes & edges of the current layer and
      // pick the motion edges to expand. The block is not executed for the
      // first layer since at the beginning we start to expand from the
      // planning_start_state.
      if (cur_layer_idx != 0) {
        SCOPED_QTRACE("InsertNode");
        for (int i = 0, n = nodes_layer.size(); i < n; ++i) {
          const auto geom_node_idx = nodes_layer[i];
          // best motion so far of a given geometry node.
          auto &best_motions = opt_motion_samples[geom_node_idx];
          for (auto &[_, dp_motion_info] : best_motions) {
            MotionState end_motion_state =
                dp_motion_info.motion_form->GetEndMotionState();
            MotionEdgeIndex motion_edge_index;
            const auto prev_motion_edge_index =
                dp_motion_info.prev_motion_edge_index;
            if (prev_motion_edge_index ==
                MotionEdgeVector<MotionEdge>::kInvalidIndex) {
              // This edge has no previous edge since it connects directly to
              // the start motion node.
              const auto end_node_index =
                  traj_output.motion_graph->AddMotionNode(
                      std::move(end_motion_state), geom_node_idx);
              motion_edge_index = traj_output.motion_graph->AddMotionEdge(
                  sdc_node_idx, end_node_index, dp_motion_info.motion_form,
                  geom_node_idx, MotionEdgeVector<MotionEdge>::kInvalidIndex);
              MotionSearchOutput::SearchCost cost;
              cost.feature_cost = dp_motion_info.costs;
              cost.cost_to_come = dp_motion_info.sum_cost;
              search_costs.emplace_back(cost);
            } else {
              // This edge has previous edge.
              const auto &prev_motion_edge =
                  traj_output.motion_graph->GetMotionEdge(
                      prev_motion_edge_index);
              end_motion_state.t +=
                  traj_output.motion_graph->GetMotionNode(prev_motion_edge.end)
                      .state.t;
              const auto end_node_index =
                  traj_output.motion_graph->AddMotionNode(
                      std::move(end_motion_state), geom_node_idx);
              motion_edge_index = traj_output.motion_graph->AddMotionEdge(
                  prev_motion_edge.end, end_node_index,
                  dp_motion_info.motion_form, geom_node_idx,
                  prev_motion_edge_index);
              MotionSearchOutput::SearchCost cost;
              cost.feature_cost =
                  AddCost(dp_motion_info.costs,
                          search_costs[prev_motion_edge_index].feature_cost);
              cost.cost_to_come =
                  dp_motion_info.sum_cost +
                  search_costs[prev_motion_edge_index].cost_to_come;
              search_costs.emplace_back(cost);
            }
            if (FLAGS_planner_initializer_debug_level >= 2 ||
                FLAGS_dumping_initializer_features) {
              auto &debug_info = traj_output.debug_info;
              debug_info.search_queue.emplace_back(motion_edge_index);
            }
            // Discard super slow state if it fails to reach the end of time
            // horizon.
            if (end_motion_state.v < 0.1 &&
                end_motion_state.t < kTrajectoryTimeHorizon) {
              continue;
            }
            // We let the motion expansion stop if it reachs the end of time
            // horizon.
            if (end_motion_state.t >= kTrajectoryTimeHorizon) {
              terminated_edge_idxes.push_back(motion_edge_index);
            } else {
              // Otherwise we add it to the to-be-expanded motions.
              motions_to_expand[geom_node_idx].push_back(motion_edge_index);
            }
          }
        }
      }

      // Reach the last layer, stop the iteration
      if (cur_layer_idx == num_layer - 1) {
        break;
      }

      // 2. Expand motions from nodes.
      // candidate_motions records the candidate motions for each node per
      // layer.
      std::vector<std::vector<DpMotionInfo>> candidate_motions;
      const absl::Cleanup cleanup_candidate_motions = [&candidate_motions] {
        MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(candidate_motions);
      };
      candidate_motions.resize(nodes_layer.size());

      std::vector<std::vector<NewCacheInfo>> new_motion_forms_container;
      const absl::Cleanup cleanup_new_motion_form_container =
          [&new_motion_forms_container] {
            MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(new_motion_forms_container);
          };
      new_motion_forms_container.resize(nodes_layer.size());

      if (cur_layer_idx == 0) {
        candidate_motions[start_node_idx_on_first_layer] =
            ExpandStartMotionEdges(
                nodes_layers[0][start_node_idx_on_first_layer], sdc_node_idx,
                *geometry, *(traj_output.motion_graph), *planner_params,
                *traj_output.cost_provider,
                &new_motion_forms_container[start_node_idx_on_first_layer],
                &cost_cache, thread_pool);
      } else {
        // This routine is the most computational heavy one, can be paralleled.
        // Create container per node to hold new motion forms temporarily.
        bool sample_const_v = false;
        if (cur_layer_idx >= kConstVelSampleLayerSizeThreshold - 1) {
          // If current layer >= kConstVelSampleLayerSizeThreshold (currently
          // set at 5), only sample acc = 0 and a_min, a_max, a_stop motions.
          sample_const_v = true;
        }
        ParallelFor(0, nodes_layer.size(), thread_pool, [&](int i) {
          // Parallel to calculate candidate motions and their cost for each
          // node on this layer.
          const auto geom_node_idx = nodes_layer[i];
          const auto &motion_edge_idxs = motions_to_expand[geom_node_idx];
          auto &new_motion_forms = new_motion_forms_container[i];
          candidate_motions[i] =
              ExpandMotionEdges(geom_node_idx, motion_edge_idxs, *geometry,
                                *(traj_output.motion_graph), *planner_params,
                                *traj_output.cost_provider, sample_const_v,
                                &new_motion_forms, &cost_cache, thread_pool);
        });
      }
      for (auto it =
               std::make_move_iterator(new_motion_forms_container.begin());
           it != std::make_move_iterator(new_motion_forms_container.end());
           ++it) {
        cost_cache.BatchInsert(*it);
      }
      // 3. Update optimal motions associated to each geometry node.
      for (auto &candidate_motions_per_node : candidate_motions) {
        motion_count += candidate_motions_per_node.size();
        int count = 0;

        for (auto &candidate_motion : candidate_motions_per_node) {
          const auto end_geometry_node_index =
              candidate_motion.end_geometry_node_index;
          // best_motion_so_far is motion terminating at this geometry node (at
          // the velocity and time determined by this motion).
          auto &best_motion_so_far =
              opt_motion_samples[end_geometry_node_index];
          const auto motion_form_or =
              cost_cache.GetMotionForm(candidate_motion.key);
          if (!motion_form_or.ok()) {
            continue;
          }
          candidate_motion.motion_form = *motion_form_or;
          const auto end_state = (*motion_form_or)->GetEndMotionState();
          const DpMotionSample dp_motion_sample(end_state.v, end_state.t);
          if (best_motion_so_far.find(dp_motion_sample) ==
              best_motion_so_far.end()) {
            // No motion is added yet to the hashmap, add it.
            // Only record some optimal edges costs to the cache.
            best_motion_so_far[dp_motion_sample] = std::move(candidate_motion);
          } else {
            // A motion is already in the hashmap, compare its cost with the
            // currently evaluating one and keep better.
            const auto &prev_best = best_motion_so_far[dp_motion_sample];
            auto prev_cost = prev_best.sum_cost;
            if (prev_best.prev_motion_edge_index !=
                MotionEdgeVector<MotionEdge>::kInvalidIndex) {
              prev_cost +=
                  search_costs[prev_best.prev_motion_edge_index].cost_to_come;
            }
            auto cur_cost = candidate_motion.sum_cost;
            if (candidate_motion.prev_motion_edge_index !=
                MotionEdgeVector<MotionEdge>::kInvalidIndex) {
              cur_cost += search_costs[candidate_motion.prev_motion_edge_index]
                              .cost_to_come;
            }
            if (cur_cost < prev_cost) {
              best_motion_so_far[dp_motion_sample] =
                  std::move(candidate_motion);
            }
          }
          count++;
        }
      }
    }

    VLOG(1) << "Total evaluated motions of DP: " << motion_count;
    VLOG(1) << "Collected motion edge cache size: " << cost_cache.size();

    // Find the best edge considering final cost.
    const auto best_edge_info =
        FindBestEdge(*traj_output.cost_provider, sdc_motion, sdc_node_idx,
                     nodes_layers[0][start_node_idx_on_first_layer],
                     traj_output.motion_graph.get(), &cost_cache, &search_costs,
                     &terminated_edge_idxes);
    const auto best_final_edge = best_edge_info.idx;
    if (best_final_edge == MotionEdgeVector<MotionEdge>::kInvalidIndex) {
      // No trajectories found.
      continue;
    }
    traj_output.last_edge_index = best_final_edge;
    traj_output.search_costs = search_costs;
    traj_output.total_cost = best_edge_info.total_cost;
    VLOG(1) << "best_final_edge: " << best_final_edge.value()
            << " Total cost: " << traj_output.total_cost;

    if (FLAGS_post_evaluation) {
      VLOG(1) << "--------- Start of One Post Evaluation-----------";
      const auto start_time = absl::Now();

      if (best_edge_info.is_created_stationary_motion) {
        QEVENT_EVERY_N_SECONDS("Jinyun",
                               "initializer_generating_a_stationary_motion",
                               5.0, [&](QEvent *qevent) {});
      } else {
        std::unique_ptr<CostProvider> post_cost_provider =
            std::make_unique<CostProvider>(
                drive_passage, constraint_manager, &search_config,
                stop_s_on_drive_passage, st_traj_mgr, vehicle_geom,
                collision_checker, path_sl, *planner_params,
                output.ref_speed_table.get(), max_accumulated_s, true);
        const auto best_post_eval_edge_info = FindBestEdgeFromTopK(
            input, leading_objs, search_costs, terminated_edge_idxes,
            sdc_motion, sdc_node_idx,
            nodes_layers[0][start_node_idx_on_first_layer],
            traj_output.motion_graph.get(), post_cost_provider.get(),
            thread_pool);
        traj_output.last_edge_index = best_post_eval_edge_info.idx;
        traj_output.total_cost = best_post_eval_edge_info.total_cost;
        VLOG(1) << "After Post Evaluation, best_final_edge: "
                << traj_output.last_edge_index.value()
                << " Total cost: " << traj_output.total_cost;
      }

      const auto duration =
          absl::ToDoubleMilliseconds(absl::Now() - start_time);
      VLOG(1) << "Time(ms) spent for One Post Evaluation: " << duration;
      VLOG(1) << "--------- End of One Post Evaluation-------------";
      QEVENT_EVERY_N_SECONDS(
          "jinyun", "initializer_post_evaluation_time_spent", 5.0,
          [&](QEvent *qevent) { qevent->AddField("time(ms)", duration); });
    }

    const auto traj_points = ConstructTrajFromLastEdge(
        *traj_output.motion_graph, traj_output.last_edge_index);
    const auto &front_pt = traj_points.front();
    const auto &back_pt = traj_points.back();
    if (front_pt.v() < kCanSetToZeroSpeed && back_pt.v() < kCanSetToZeroSpeed &&
        back_pt.path_point().s() - front_pt.path_point().s() <
            kCanSetToZeroTrajLength) {
      traj_output.traj_points = ConstructStationaryTraj(sdc_motion);
    } else {
      traj_output.traj_points = std::move(traj_points);
    }

    // For debug: show candidate trajectories
    if (FLAGS_planner_initializer_debug_level >= 2 ||
        FLAGS_dumping_initializer_features) {
      const int kTopTrajectories =
          std::min(static_cast<int>(terminated_edge_idxes.size()), 50);
      const auto top_k_traj_info = TopKTrajectories(
          terminated_edge_idxes, traj_output.search_costs, kTopTrajectories);
      std::vector<double> top_k_total_costs;
      top_k_total_costs.reserve(top_k_traj_info.size());
      std::vector<MotionEdgeIndex> top_k_edges;
      top_k_edges.reserve(top_k_traj_info.size());
      std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
      top_k_trajs.reserve(top_k_traj_info.size());
      for (const auto &traj_info : top_k_traj_info) {
        top_k_total_costs.push_back(traj_info.total_cost);
        top_k_edges.push_back(traj_info.idx);
        top_k_trajs.push_back(ConstructTrajFromLastEdge(
            *traj_output.motion_graph, traj_info.idx));
      }
      SingleTrajDebugInfo traj_debug_info({
          .terminated_edge_idxes = std::move(terminated_edge_idxes),
          .top_k_trajs = std::move(top_k_trajs),
          .top_k_total_costs = std::move(top_k_total_costs),
          .top_k_edges = std::move(top_k_edges),
      });
      traj_output.debug_info = std::move(traj_debug_info);
    }

    multi_trajs.push_back(std::move(traj_output));
    VLOG(1) << "Dp motion search for Time spent: " << absl::Now() - start_time;
    VLOG(1) << "--------- End of One Search-------------";
  }

  if (multi_trajs.empty()) {
    return absl::NotFoundError(
        "Weird! We can not find any terminated edge index for these leading "
        "objs.");
  }

  VLOG(2) << "---------- Leading Object Group ----------";
  int i = 1;
  for (const auto &traj_info : multi_trajs) {
    std::string out;
    if (traj_info.lead_objs.empty()) {
      out = "No leading object.";
    } else {
      out = PrintVector(traj_info.lead_objs);
    }
    VLOG(2) << i << ". " << out << " cost: " << traj_info.total_cost;
    i++;
  }
  VLOG(2) << "------------------------------------------";

  // Add these multi traj to motion search output result.
  output.trajs_with_lead_obj.reserve(multi_trajs.size());
  for (const auto &traj : multi_trajs) {
    MotionSearchOutput::TrajWithLeadingObject traj_with_lead_obj;
    traj_with_lead_obj.leading_obj_ids = traj.lead_objs;
    traj_with_lead_obj.trajectory = traj.traj_points;
    if (FLAGS_planner_initializer_debug_level >= 2 ||
        FLAGS_dumping_initializer_features) {
      if (traj.last_edge_index.value() < traj.search_costs.size()) {
        traj_with_lead_obj.feature_costs =
            traj.search_costs[traj.last_edge_index].feature_cost;
      }
      traj_with_lead_obj.last_edge_index = traj.last_edge_index;
      traj_with_lead_obj.total_cost = traj.total_cost;
    }
    output.trajs_with_lead_obj.push_back(std::move(traj_with_lead_obj));
  }

  // Need to determine the final output.
  const auto choice = EvaluateMultiTrajs(
      st_traj_mgr, drive_passage, *planner_params, multi_trajs,
      input.lc_clearance, vehicle_geom, thread_pool, debug_proto);
  VLOG(2) << "Evaluate choice: " << choice + 1;
  output.choice = choice;
  output.min_cost = multi_trajs[choice].total_cost;
  output.traj_points = multi_trajs[choice].traj_points;
  output.best_last_edge_index = multi_trajs[choice].last_edge_index;
  output.cost = multi_trajs[choice].search_costs;
  output.motion_graph = std::move(multi_trajs[choice].motion_graph);
  output.cost_provider = std::move(multi_trajs[choice].cost_provider);
  output.leading_objs = multi_trajs[choice].lead_objs;
  VLOG(1) << absl::StrJoin(
      output.cost[output.best_last_edge_index].feature_cost, ", ");

  if (FLAGS_planner_initializer_debug_level >= 2 ||
      FLAGS_dumping_initializer_features) {
    const auto &debug_info = multi_trajs[choice].debug_info;
    output.search_queue = std::move(debug_info.search_queue);
    output.terminated_edge_idxes = std::move(debug_info.terminated_edge_idxes);
    output.top_k_trajs = std::move(debug_info.top_k_trajs);
    output.top_k_total_costs = std::move(debug_info.top_k_total_costs);
    output.top_k_edges = std::move(debug_info.top_k_edges);
  }

  // For data dumping only.
  if (FLAGS_dumping_initializer_features) {
    auto res = ExpertTrajectoryEvaluation(input, output.leading_objs, &output);
    if (!res.ok()) {
      return res;
    }
    SampledDpMotionEvaluation(input, output.leading_objs,
                              output.cost /*search_cost*/,
                              output.terminated_edge_idxes, &output);
  }

  return output;
}

}  // namespace qcraft::planner
