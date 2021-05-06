#ifndef ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_REFERENCE_LINE_COST_FEATURE_H_

#include <algorithm>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

// Major objective of reference line searcher is to search for a reference line
// which avoids stationary objects so during motion expansion, more time
// can be used to expand useful geometry edges.

class RefLineStationaryObjectFeatureCost : public FeatureCost {
 public:
  explicit RefLineStationaryObjectFeatureCost(
      const SpacetimeTrajectoryManager* st_traj_mgr,
      const DrivePassage* passage)
      : FeatureCost("ref_line_stationary_object"),
        st_traj_mgr_(st_traj_mgr),
        passage_(passage) {
    const auto st_planner_trajs = st_traj_mgr->spacetime_planner_trajs();
    for (const auto& traj : st_planner_trajs) {
      if (traj.is_stationary()) {
        stationary_obj_states_.push_back(traj.states().front());
      }
    }
  }

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  static constexpr int sample_step_ = 1;
  std::vector<SpacetimeObjectState> stationary_obj_states_;
  [[maybe_unused]] const SpacetimeTrajectoryManager* st_traj_mgr_;
  [[maybe_unused]] const DrivePassage* passage_;
};

class RefLineProgressFeatureCost : public FeatureCost {
 public:
  explicit RefLineProgressFeatureCost(double geom_graph_max_accum_s,
                                      const PathSlBoundary* path_sl_boundary)
      : FeatureCost("ref_line_progress"),
        geom_graph_max_accum_s_(geom_graph_max_accum_s),
        path_sl_boundary_(path_sl_boundary) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double geom_graph_max_accum_s_;
  const PathSlBoundary* path_sl_boundary_;
};

class RefLinePathBoundaryFeatureCost : public FeatureCost {
 public:
  explicit RefLinePathBoundaryFeatureCost(
      const PathSlBoundary* path_sl_boundary)
      : FeatureCost("ref_line_path_boundary"),
        path_sl_boundary_(path_sl_boundary) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const PathSlBoundary* path_sl_boundary_;
};
class RefLineCurvatureFeatureCost : public FeatureCost {
 public:
  explicit RefLineCurvatureFeatureCost(const DrivePassage* passage,
                                       double relaxed_center_max_curvature)
      : FeatureCost("ref_line_curvature"),
        passage_(passage),
        inv_relaxed_center_max_curvature_(1.0 / relaxed_center_max_curvature) {}

  void ComputeCost(const GeometryEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  [[maybe_unused]] const DrivePassage* passage_;
  double inv_relaxed_center_max_curvature_;
};
}  // namespace qcraft::planner
#endif
