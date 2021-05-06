#ifndef ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_
#define ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "onboard/async/parallel_for.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/dp_svt_cost.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_graph_data.h"
#include "onboard/planner/speed/svt_graph_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/utils/status_macros.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace qcraft::planner {

struct SvGridIndex {
  int grid_index_s_ = 0;
  int grid_index_v_ = 0;

  SvGridIndex(int grid_index_s, int grid_index_v)
      : grid_index_s_(grid_index_s), grid_index_v_(grid_index_v) {}

  bool operator==(const SvGridIndex& other) const {
    return grid_index_s_ == other.grid_index_s_ &&
           grid_index_v_ == other.grid_index_v_;
  }
  bool operator!=(const SvGridIndex& other) const { return !(*this == other); }
  bool operator<(const SvGridIndex& other) const {
    return grid_index_s_ < other.grid_index_s_ ||
           (grid_index_s_ == other.grid_index_s_ &&
            grid_index_v_ < other.grid_index_v_);
  }

  template <typename H>
  friend H AbslHashValue(H h, const SvGridIndex& index_range) {
    return H::combine(std::move(h), index_range.grid_index_s_,
                      index_range.grid_index_v_);
  }

  std::string DebugString() const {
    return absl::StrCat("(", grid_index_s_, ", ", grid_index_v_, ")");
  }
};

struct PreliminarySpeedWithCost {
  PreliminarySpeedWithCost() = default;
  PreliminarySpeedWithCost(double c, SpeedVector sv)
      : cost(c), preliminary_speed(std::move(sv)) {}
  double cost = 0.0;
  SpeedVector preliminary_speed;
};

class GriddedSvtGraph {
 public:
  GriddedSvtGraph(const StGraphData* st_graph_data,
                  const TrajectoryPoint& init_point,
                  const SpeedFinderParamsProto* speed_finder_params,
                  double speed_cap,
                  std::vector<StBoundaryWithDecision> st_boundaries_wd);

  void SwapStBoundariesWithDecision(
      std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
    st_boundaries_with_decision_.swap(*st_boundaries_with_decision);
  }

  absl::Status FindOptimalPreliminarySpeed(
      SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
      ThreadPool* thread_pool);

  absl::Status FindOptimalPreliminarySpeedWithCost(
      PreliminarySpeedWithCost* preliminary_speed_with_cost,
      SamplingDpDebugProto* sampling_dp_debug, ThreadPool* thread_pool);

  absl::Status GenerateSamplingDpSpeedProfileCandidateSet(
      std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
      SamplingDpDebugProto* sampling_dp_debug,
      InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
      ThreadPool* thread_pool);

 private:
  // Return last layer sv grid indices that had points located on when search
  // finished.
  absl::StatusOr<absl::flat_hash_set<SvGridIndex>>
  SearchAndReturnFinalLayerPoints(ThreadPool* thread_pool);

  // Discretize sampling space and set the grid size of S,V,T respectively.
  absl::Status InitLayers();

  std::vector<SvtGraphPoint> ExpandByConstAccModel(
      int cur_layer_index, double next_t, int next_point_index_t,
      const SpeedLimit& speed_limit, double cruise_speed, double init_speed,
      const SvtGraphPoint& cur_point);

  void ExpandToNextLayer(int cur_layer_index, double next_t,
                         int next_point_index_t, const SpeedLimit& speed_limit,
                         double cruise_speed, double init_speed,
                         const SvtGraphPoint& cur_point,
                         std::vector<std::vector<std::vector<SvtGraphPoint>>>*
                             next_layer_candidate_points,
                         absl::flat_hash_set<SvGridIndex>* index_range);

  absl::StatusOr<PreliminarySpeedWithCost>
  GetSpeedProfileAndCompleteStBoundariesWithDecision(
      const absl::flat_hash_set<SvGridIndex>& final_layer_index_range);

  std::vector<PreliminarySpeedWithCost> SampleSpeedProfilesFromSamplingDp(
      const absl::flat_hash_set<SvGridIndex>& final_layer_indices);

  void AddAllSpeedProfilesToDebug(
      const absl::flat_hash_set<SvGridIndex>& final_layer_index_range,
      SamplingDpDebugProto* sampling_dp_debug);

  const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision() {
    return st_boundaries_with_decision_;
  }

 private:
  using SamplingDpSpeedParamsProto =
      SpeedFinderParamsProto::SamplingDpSpeedParamsProto;

  const StGraphData* st_graph_data_;

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision_;

  TrajectoryPoint init_point_;
  const SpeedFinderParamsProto* speed_finder_params_;
  const SamplingDpSpeedParamsProto* dp_params_;

  DpSvtCost dp_svt_cost_;

  double total_duration_t_ = 0.0;  // Default total_duration_t = 10 s.
  double unit_t_ = 0.0;
  double unit_inv_t_ = 0.0;
  int dimension_t_ = 0;  // Dimension represents number of grid.

  double total_length_s_ = 0.0;  // Total_length_s depends on extended path.
  double unit_s_ = 0.0;
  double unit_inv_s_ = 0.0;
  int dimension_grid_s_ = 0;

  double total_length_v_ = 0.0;  // Default total_length_v = 45.0 mph.
  double unit_v_ = 0.0;
  double unit_inv_v_ = 0.0;
  int dimension_grid_v_ = 0;

  std::vector<std::vector<double>> acc_matrix_;

  // Knots represent all of the index of grid (s,v).
  std::vector<double> t_knots_;
  std::vector<double> v_knots_;
  std::vector<double> s_knots_;

  // Layers_ represents the grid of (s, v) for every layer(t), such as
  // layers_[t][s][v].
  std::vector<std::vector<std::vector<std::optional<SvtGraphPoint>>>> layers_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_
