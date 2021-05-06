#ifndef ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
#define ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft::planner {

class CostProviderBase {
 public:
  absl::Span<const std::string> cost_names() const { return cost_names_; }

  absl::Span<const double> weights() const { return weights_; }

  void ComputeDpCost(double start_t, const MotionForm* motion_form,
                     absl::Span<double> cost) const;
  void ComputeDpCostByName(const std::string name, double start_t,
                           const MotionForm* motion_form,
                           absl::Span<double> cost) const;
  void ComputeRefLineCost(const GeometryForm* geometry_form, bool terminating,
                          absl::Span<double> cost) const;

 protected:
  template <typename Config>
  void BuildWeightTable(const Config& cost_config);

  std::vector<std::unique_ptr<FeatureCost>> features_;

 private:
  // The name of each feature cost.
  std::vector<std::string> cost_names_;

  // The weight of each cost feature.
  std::vector<double> weights_;

  std::vector<int> feature_size_;
};

class CostProvider : public CostProviderBase {
 public:
  // DP.
  CostProvider(const DrivePassage* drive_passage,
               const ConstraintManager* constraint_mgr,
               const InitializerSearchConfig* search_config,
               const std::vector<double>& stop_s,
               const SpacetimeTrajectoryManager* st_traj_mgr,
               const VehicleGeometryParamsProto* vehicle_geom,
               const CollisionChecker* collision_checker,
               const PathSlBoundary* path_sl,
               const PlannerParamsProto& planner_params,
               const RefSpeedTable* ref_speed_table, double max_accumulated_s,
               const bool is_post_evaluation = false);
};

class RefLineCostProvider : public CostProviderBase {
 public:
  RefLineCostProvider(const SpacetimeTrajectoryManager* st_traj_mgr,
                      const DrivePassage* drive_passage,
                      const PathSlBoundary* path_sl,
                      double geom_graph_mac_accum_s,
                      double relaxed_center_max_curvature,
                      const PlannerParamsProto& planner_params);
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
