#include "onboard/planner/initializer/cost_provider.h"

#include <string>
#include <utility>

#include "onboard/planner/initializer/brute_force_collision_checker.h"
#include "onboard/planner/initializer/dp_cost_feature.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/initializer/reference_line_cost_feature.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft::planner {

namespace {
constexpr double kGeometryEdgeSampleStep = 0.5;  // m.
}

template <typename Config>
void CostProviderBase::BuildWeightTable(const Config& cost_config) {
  const auto* reflection = cost_config.GetReflection();
  const auto* descriptor = cost_config.GetDescriptor();
  for (const auto& feature : features_) {
    auto* feature_desc =
        descriptor->FindFieldByName(std::string(feature->name()));
    const auto& feature_conf =
        reflection->GetMessage(cost_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    int feature_size = 0;
    for (const auto* desc : feature_weights_desc) {
      if (desc->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
        continue;
      }
      ++feature_size;
      cost_names_.push_back(absl::StrCat(feature->name(), ".", desc->name()));
      weights_.push_back(feature_ref->GetDouble(feature_conf, desc));
    }
    feature_size_.push_back(feature_size);
  }
}

void CostProviderBase::ComputeDpCost(const double start_t,
                                     const MotionForm* motion_form,
                                     absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  MotionEdgeInfo edge_info;
  edge_info.start_t = start_t;
  edge_info.motion_form = motion_form;
  edge_info.states = motion_form->FastSample(prediction::kPredictionTimeStep *
                                             MotionEdgeInfo::kSampleStep);
  edge_info.edge = nullptr;  // Dp does not construct motion edge at this stage.
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    features_[i]->ComputeCost(edge_info, cost.subspan(index, feature_size_[i]));
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
}

void CostProviderBase::ComputeDpCostByName(const std::string name,
                                           double start_t,
                                           const MotionForm* motion_form,
                                           absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  MotionEdgeInfo edge_info;
  edge_info.motion_form = motion_form;
  edge_info.start_t = start_t;
  edge_info.states = motion_form->FastSample(prediction::kPredictionTimeStep *
                                             MotionEdgeInfo::kSampleStep);
  edge_info.edge = nullptr;
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    if (features_[i]->name() == name) {
      features_[i]->ComputeCost(edge_info,
                                cost.subspan(index, feature_size_[i]));
      for (int j = 0; j < feature_size_[i]; ++j) {
        cost[index + j] = cost[index + j] * weights_[index + j];
      }
      break;
    }
    index += feature_size_[i];
  }
}

void CostProviderBase::ComputeRefLineCost(const GeometryForm* geometry_form,
                                          bool terminating,
                                          absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  GeometryEdgeInfo edge_info;
  edge_info.geometry_form = geometry_form;
  edge_info.terminating = terminating;
  edge_info.states = geometry_form->Sample(kGeometryEdgeSampleStep);
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    features_[i]->ComputeCost(edge_info, cost.subspan(index, feature_size_[i]));
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
}

// -------------- CostProvider -------------
// Dp
CostProvider::CostProvider(const DrivePassage* drive_passage,
                           const ConstraintManager* constraint_mgr,
                           const InitializerSearchConfig* search_config,
                           const std::vector<double>& stop_s,
                           const SpacetimeTrajectoryManager* st_traj_mgr,
                           const VehicleGeometryParamsProto* vehicle_geom,
                           const CollisionChecker* collision_checker,
                           const PathSlBoundary* path_sl,
                           const PlannerParamsProto& planner_params,
                           const RefSpeedTable* ref_speed_table,
                           double max_accumulated_s,
                           const bool is_post_evaluation) {
  const auto& cost_config =
      is_post_evaluation
          ? planner_params.initializer_params().dp_post_cost_config()
          : planner_params.initializer_params().dp_cost_config();
  if (cost_config.has_dp_acceleration()) {
    features_.emplace_back(
        std::make_unique<DpAccelerationFeatureCost>(&planner_params));
  }

  if (cost_config.has_dp_lane_boundary()) {
    features_.emplace_back(std::make_unique<DpLaneBoundaryFeatureCost>(
        path_sl, vehicle_geom->width() * 0.5));
  }

  if (cost_config.has_dp_curvature()) {
    features_.emplace_back(std::make_unique<DpCurvatureFeatureCost>());
  }

  if (cost_config.has_dp_lateral_acceleration()) {
    features_.emplace_back(std::make_unique<DpLateralAccelerationFeatureCost>(
        search_config->is_lane_change()));
  }

  if (cost_config.has_dp_stop_constraint()) {
    features_.emplace_back(
        std::make_unique<DpStopConstraintFeatureCost>(constraint_mgr, stop_s));
  }

  if (cost_config.has_dp_ref_speed()) {
    features_.emplace_back(
        std::make_unique<DpRefSpeedFeatureCost>(ref_speed_table));
  }

  if (cost_config.has_dp_dynamic_collision()) {
    features_.emplace_back(
        std::make_unique<DpDynamicCollisionFeatureCost>(collision_checker));
  }

  if (cost_config.has_dp_leading_object()) {
    features_.emplace_back(std::make_unique<DpLeadingObjectFeatureCost>(
        constraint_mgr, st_traj_mgr, drive_passage, vehicle_geom->length(),
        search_config));
  }

  if (cost_config.has_dp_final_progress()) {
    features_.emplace_back(std::make_unique<DpFinalProgressFeatureCost>(
        path_sl, max_accumulated_s));
  }

  BuildWeightTable<InitializerConfig::DpFeatureCostConfig>(cost_config);
}

// RefLineCostProvider.

RefLineCostProvider::RefLineCostProvider(
    const SpacetimeTrajectoryManager* st_traj_mgr,
    const DrivePassage* drive_passage, const PathSlBoundary* path_sl,
    double geom_graph_max_accum_s, double relaxed_center_max_curvature,
    const PlannerParamsProto& planner_params) {
  const auto& cost_config =
      planner_params.initializer_params().ref_line_cost_config();
  if (cost_config.has_ref_line_stationary_object()) {
    features_.emplace_back(std::make_unique<RefLineStationaryObjectFeatureCost>(
        st_traj_mgr, drive_passage));
  }

  if (cost_config.has_ref_line_progress()) {
    features_.emplace_back(std::make_unique<RefLineProgressFeatureCost>(
        geom_graph_max_accum_s, path_sl));
  }

  if (cost_config.has_ref_line_path_boundary()) {
    features_.emplace_back(
        std::make_unique<RefLinePathBoundaryFeatureCost>(path_sl));
  }

  if (cost_config.has_ref_line_curvature()) {
    features_.emplace_back(std::make_unique<RefLineCurvatureFeatureCost>(
        drive_passage, relaxed_center_max_curvature));
  }

  BuildWeightTable<InitializerConfig::RefLineFeatureCostConfig>(cost_config);
}

}  // namespace qcraft::planner
