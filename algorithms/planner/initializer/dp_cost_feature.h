#ifndef ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft::planner {
class DpAccelerationFeatureCost : public FeatureCost {
 public:
  explicit DpAccelerationFeatureCost(const PlannerParamsProto* config)
      : FeatureCost("dp_acceleration"), config_(config) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  const PlannerParamsProto* config_;
};

class DpLaneBoundaryFeatureCost : public FeatureCost {
 public:
  explicit DpLaneBoundaryFeatureCost(const PathSlBoundary* path_sl,
                                     double sdc_half_width)
      : FeatureCost("dp_lane_boundary"),
        path_sl_(path_sl),
        sdc_half_width_(sdc_half_width) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  const PathSlBoundary* path_sl_;
  double sdc_half_width_;
};

class DpCurvatureFeatureCost : public FeatureCost {
 public:
  DpCurvatureFeatureCost() : FeatureCost("dp_curvature") {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;
};

class DpLateralAccelerationFeatureCost : public FeatureCost {
 public:
  explicit DpLateralAccelerationFeatureCost(bool is_lane_change)
      : FeatureCost("dp_lateral_acceleration"),
        is_lane_change_(is_lane_change) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  bool is_lane_change_;
};

class DpStopConstraintFeatureCost : public FeatureCost {
 public:
  explicit DpStopConstraintFeatureCost(const ConstraintManager* c_mgr,
                                       const std::vector<double>& stop_s)
      : FeatureCost("dp_stop_constraint"),
        constraint_mgr_(c_mgr),
        nearest_stop_s_(stop_s.empty()
                            ? std::numeric_limits<double>::max()
                            : *std::min_element(stop_s.begin(), stop_s.end())) {
  }

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  [[maybe_unused]] const ConstraintManager* constraint_mgr_;
  double nearest_stop_s_;
};

class DpRefSpeedFeatureCost : public FeatureCost {
 public:
  explicit DpRefSpeedFeatureCost(const RefSpeedTable* ref_speed_table)
      : FeatureCost("dp_ref_speed"), ref_speed_table_(ref_speed_table) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const RefSpeedTable* ref_speed_table_;
};

class DpDynamicCollisionFeatureCost : public FeatureCost {
 public:
  explicit DpDynamicCollisionFeatureCost(
      const CollisionChecker* collision_checker);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  const CollisionChecker* cc_;
};

class DpLeadingObjectFeatureCost : public FeatureCost {
 public:
  // Construct the DpLeadingObjectFeatureCost without specifying the leading
  // objects.
  explicit DpLeadingObjectFeatureCost(
      const ConstraintManager* c_mgr,
      const SpacetimeTrajectoryManager* st_traj_mgr,
      const DrivePassage* drive_passage, double sdc_length,
      const InitializerSearchConfig* search_config);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const DrivePassage* passage_;
  const ConstraintManager* c_mgr_;
  const SpacetimeTrajectoryManager* st_traj_mgr_;
  double sdc_length_;
  bool is_lane_change_;
  PiecewiseLinearFunction<double> max_s_t_;  // Front objects.
  PiecewiseLinearFunction<double> min_s_t_;  // Rear objects.
};

class DpFinalProgressFeatureCost : public FeatureCost {
 public:
  explicit DpFinalProgressFeatureCost(const PathSlBoundary* path_sl,
                                      double max_accumulated_s)
      : FeatureCost("dp_final_progress"),
        path_sl_(path_sl),
        max_accumulated_s_(max_accumulated_s) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const PathSlBoundary* path_sl_;
  double max_accumulated_s_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_
