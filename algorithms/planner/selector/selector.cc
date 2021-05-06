#include "onboard/planner/selector/selector.h"

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/nets/trt/prophnet.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/selector/candidate_stats.h"
#include "onboard/planner/selector/traj_cost_features.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

using CostFeatures = std::vector<std::unique_ptr<CostFeatureBase>>;
using WeightTable = absl::flat_hash_map<std::string, CostFeatureBase::CostVec>;

struct CostType {
  double cost_common;
  double cost_same_start;

  inline double cost_sum() const { return cost_common + cost_same_start; }
  bool operator<(const CostType &other) const {
    return cost_sum() < other.cost_sum();
  }
  void add(double val, bool is_common) {
    (is_common ? cost_common : cost_same_start) += val;
  }
};

CostFeatures BuildCostFeatures(
    const SelectorInput &input,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &results) {
  QCHECK(input.config->has_cost_config());

  CostFeatures cost_features;
  const auto &cost_config = input.config->cost_config();
  if (cost_config.enable_progress_cost()) {
    cost_features.emplace_back(std::make_unique<TrajProgressCost>(
        input.smm, input.sections_info,
        ProgressStats(*input.smm, *input.plan_start_point, *input.vehicle_geom,
                      scheduler_outputs, est_status, results)));
  }
  if (cost_config.enable_max_jerk_cost()) {
    cost_features.emplace_back(
        std::make_unique<TrajMaxJerkCost>(*input.motion_constraints));
  }
  if (cost_config.enable_lane_change_cost()) {
    cost_features.emplace_back(std::make_unique<TrajLaneChangeCost>(
        input.smm, input.prev_lane_path_from_current, input.prev_traj,
        *input.plan_start_point, *input.vehicle_geom));
  }
  if (cost_config.enable_solid_boundary_cost()) {
    cost_features.emplace_back(
        std::make_unique<TrajCrossSolidBoundaryCost>(*input.vehicle_geom));
  }
  if (cost_config.enable_dist_to_objects_cost()) {
    cost_features.emplace_back(
        std::make_unique<TrajMinDistToObjectsCost>(*input.vehicle_geom));
  }
  if (cost_config.enable_route_look_ahead_cost()) {
    cost_features.emplace_back(
        std::make_unique<TrajRouteLookAheadCost>(RouteLookAheadStats(
            *input.smm, *input.sections_info, *input.lane_path_infos,
            *input.stalled_objects, scheduler_outputs, est_status, results)));
  }
  if (cost_config.enable_boundary_expansion_cost()) {
    cost_features.emplace_back(
        std::make_unique<TrajBoundaryExpansionCost>(*input.plan_start_point));
  }
  if (cost_config.enable_prohibited_region_cost()) {
    cost_features.emplace_back(std::make_unique<TrajProhibitedRegionCost>(
        input.sections_info, *input.plan_start_point, input.stalled_objects,
        *input.vehicle_geom));
  }
  if (cost_config.enable_is_fallback_cost()) {
    cost_features.emplace_back(std::make_unique<TrajIsFallbackCost>());
  }
  return cost_features;
}

WeightTable BuildWeightTable(
    const SelectorParamsProto::TrajCostWeights &weight_config,
    const CostFeatures &cost_features) {
  WeightTable weight_table;
  const auto *reflection = weight_config.GetReflection();
  const auto *descriptor = weight_config.GetDescriptor();
  for (const auto &feature : cost_features) {
    auto *feature_desc = descriptor->FindFieldByName(feature->name());
    const auto &feature_conf =
        reflection->GetMessage(weight_config, feature_desc);
    const auto *feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor *> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);

    auto &weight_vec = weight_table[feature->name()];
    weight_vec.resize(feature->size());
    int idx = 0;
    for (const auto *desc : feature_weights_desc) {
      weight_vec[idx++] = feature_ref->GetDouble(feature_conf, desc);
    }
  }
  return weight_table;
}

CostType ComputeTrajCost(const CostFeatures &cost_features,
                         const WeightTable &weights,
                         const SchedulerOutput &scheduler_output,
                         const EstPlannerOutput &planner_output,
                         TrajectoryCost *traj_cost) {
  CostType cost_res{0.0, 0.0};
  for (const auto &feature : cost_features) {
    std::vector<std::string> extra_info;
    const auto cost_vec =
        feature->ComputeCost(scheduler_output, planner_output, &extra_info);
    const double feat_cost =
        (FindOrDie(weights, feature->name()) * cost_vec).sum();

    auto &feat_debug = *traj_cost->add_features();
    feat_debug.set_name(feature->name());
    feat_debug.set_cost(feat_cost);
    for (auto &info : extra_info) {
      *feat_debug.add_extra_info() = std::move(info);
    }

    cost_res.add(feat_cost, feature->is_common());
  }
  traj_cost->set_start_lane_id(
      scheduler_output.drive_passage.lane_path().front().lane_id());
  traj_cost->set_sum_common(cost_res.cost_common);
  traj_cost->set_sum(cost_res.cost_sum());

  return cost_res;
}

absl::Status ExtractTrajectoryFeature(
    const PredictionDebugProto &prediction_debug,
    const std::vector<ApolloTrajectoryPointProto> &traj,
    ProphnetDumpedFeatureProto::ObjectsDumpedFeature *traj_model_feature) {
  if (traj.size() < 2) {
    return absl::FailedPreconditionError(
        " A trajectory is too short! Less than two points.");
  }

  if (!prediction_debug.has_features() ||
      !prediction_debug.features().has_prophnet_data() ||
      !prediction_debug.features().prophnet_data().has_objects_feature()) {
    return absl::FailedPreconditionError(
        "No valid est prophnet_data output for ExtractTrajectoryFeature");
  }

  const auto &pred_obj_features =
      prediction_debug.features().prophnet_data().objects_feature();
  const auto ref_pos =
      Vec2d(pred_obj_features.ego_pos(0), pred_obj_features.ego_pos(1));
  const float rot_rad_sin = pred_obj_features.rot_mat(2);
  const float rot_rad_cos = pred_obj_features.rot_mat(0);
  const float rot_rad = std::atan2(rot_rad_sin, rot_rad_cos);

  // Tranform pos to ego-based coordinate.
  const auto abs_cur_pos = Vec2dFromApolloTrajectoryPointProto(traj.front());
  const Vec2d cur_pos = (abs_cur_pos - ref_pos).Rotate(rot_rad);
  traj_model_feature->add_cur_poses(cur_pos.x());
  traj_model_feature->add_cur_poses(cur_pos.y());

  const float time_step = traj[1].relative_time() - traj[0].relative_time();
  const float init_v_x = traj[0].v() * std::cos(traj[0].path_point().theta());
  const float init_v_y = traj[0].v() * std::sin(traj[0].path_point().theta());
  Vec2d rot_vel = (Vec2d(init_v_x, init_v_y)).Rotate(rot_rad);
  traj_model_feature->add_trajs(rot_vel.x() * time_step);
  traj_model_feature->add_trajs(rot_vel.y() * time_step);
  traj_model_feature->add_speeds(traj[0].v());
  const double rot_heading =
      NormalizeAngle(traj[0].path_point().theta() + rot_rad);
  traj_model_feature->add_headings(std::sin(rot_heading));
  traj_model_feature->add_headings(std::cos(rot_heading));

  Vec2d prev_pos = cur_pos;
  for (int i = 1, size = traj.size(); i < size; ++i) {
    const Vec2d cur_pos =
        (Vec2dFromApolloTrajectoryPointProto(traj[i]) - ref_pos)
            .Rotate(rot_rad);
    traj_model_feature->add_trajs((cur_pos - prev_pos).x());
    traj_model_feature->add_trajs((cur_pos - prev_pos).y());
    prev_pos = cur_pos;
    traj_model_feature->add_speeds(traj[i].v());
    const double rot_heading =
        NormalizeAngle(traj[i].path_point().theta() + rot_rad);
    traj_model_feature->add_headings(std::sin(rot_heading));
    traj_model_feature->add_headings(std::cos(rot_heading));
  }

  return absl::OkStatus();
}

absl::Status ExtractLaneFeature(
    const PredictionDebugProto &prediction_debug,
    const mapping::LanePath &lane_path,
    ProphnetDumpedFeatureProto::LanesDumpedFeature *lane_feature) {
  const auto lanes_info = lane_path.GetLanesInfo();
  const int sample_points_num = prophnet::kLanePointsNum + 1;
  const double fraction_step = 1.0 / (sample_points_num - 1);

  if (!prediction_debug.has_features() ||
      !prediction_debug.features().has_prophnet_data() ||
      !prediction_debug.features().prophnet_data().has_objects_feature()) {
    return absl::FailedPreconditionError(
        "No valid est prophnet_data output for ExtractTrajectoryFeature");
  }

  const auto &pred_obj_features =
      prediction_debug.features().prophnet_data().objects_feature();
  const auto ref_pos =
      Vec2d(pred_obj_features.ego_pos(0), pred_obj_features.ego_pos(1));
  const float rot_rad_sin = pred_obj_features.rot_mat(2);
  const float rot_rad_cos = pred_obj_features.rot_mat(0);
  const float rot_rad = std::atan2(rot_rad_sin, rot_rad_cos);

  for (int i = 0; i < lanes_info.size(); ++i) {
    std::vector<Vec2d> lane_points;
    lane_points.reserve(sample_points_num);
    for (int j = 0; j < sample_points_num; ++j) {
      lane_points.push_back(
          lanes_info[i]->LerpPointFromFraction(j * fraction_step));
    }
    Vec2d prev_transformed_lane_point =
        (lane_points[0] - ref_pos).Rotate(rot_rad);
    for (int k = 1; k < sample_points_num; ++k) {
      const Vec2d transformed_lane_point =
          (lane_points[k] - ref_pos).Rotate(rot_rad);
      const Vec2d lane_center =
          (transformed_lane_point + prev_transformed_lane_point) * 0.5f;
      lane_feature->add_lane_centers(lane_center.x());
      lane_feature->add_lane_centers(lane_center.y());
      prev_transformed_lane_point = transformed_lane_point;
    }
  }

  return absl::OkStatus();
}

absl::Status DumpEvaluations(
    const SelectorInput &input, const CostFeatures &cost_features,
    const WeightTable &weights,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &planner_outputs,
    SelectorDebugProto *selector_debug) {
  for (int idx = 0, size = planner_outputs.size(); idx < size; ++idx) {
    bool is_expert = scheduler_outputs[idx].is_expert;
    if (!est_status[idx].ok() && is_expert) {
      return absl::FailedPreconditionError("Expert data not available!");
    }
    if (!est_status[idx].ok()) continue;
    auto *traj_feature = is_expert
                             ? selector_debug->mutable_expert_traj_features()
                             : selector_debug->add_candidate_trajs_features();
    const auto start_id =
        scheduler_outputs[idx].drive_passage.lane_path().front().lane_id();
    traj_feature->set_start_lane_id(start_id);

    for (const auto &feature : cost_features) {
      std::vector<std::string> extra_info;
      auto feature_vec = feature->ComputeCost(
          scheduler_outputs[idx], planner_outputs[idx], &extra_info);
      auto &feat_debug = *traj_feature->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.mutable_sub_names()->Add(feature->sub_names().begin(),
                                          feature->sub_names().end());
      feat_debug.mutable_values()->Add(std::begin(feature_vec),
                                       std::end(feature_vec));
      feat_debug.set_is_common(feature->is_common());
      for (auto &info : extra_info) {
        *feat_debug.add_extra_info() = std::move(info);
      }
    }
    auto *traj = is_expert ? selector_debug->mutable_expert_traj()
                           : selector_debug->add_candidate_trajs();
    *traj->mutable_trajectory_points() = {
        planner_outputs[idx].traj_points.begin(),
        planner_outputs[idx].traj_points.end()};

    auto *traj_model_feature =
        is_expert ? selector_debug->mutable_expert_traj_model_features()
                  : selector_debug->add_candidate_trajs_model_features();
    RETURN_IF_ERROR(ExtractTrajectoryFeature(*input.prediction_debug,
                                             planner_outputs[idx].traj_points,
                                             traj_model_feature));

    auto *lane_model_feature =
        is_expert ? selector_debug->mutable_expert_lane_model_features()
                  : selector_debug->add_candidate_lanes_model_features();
    RETURN_IF_ERROR(ExtractLaneFeature(
        *input.prediction_debug,
        scheduler_outputs[idx].drive_passage.extend_lane_path(),
        lane_model_feature));
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<int> SelectTrajectory(
    const SelectorInput &input,
    const std::vector<SchedulerOutput> &scheduler_outputs,
    const std::vector<PlannerStatus> &est_status,
    const std::vector<EstPlannerOutput> &results,
    SelectorDebugProto *selector_debug) {
  SCOPED_QTRACE("SelectTrajectory");

  QCHECK_EQ(scheduler_outputs.size(), results.size());
  if (results.empty()) return absl::NotFoundError("Input results empty!");

  const auto cost_features =
      BuildCostFeatures(input, scheduler_outputs, est_status, results);
  const auto weights =
      BuildWeightTable(input.config->cost_weights(), cost_features);

  absl::flat_hash_map<mapping::ElementId, CostType> traj_costs;
  absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
  traj_costs.reserve(results.size());
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    if (scheduler_outputs[idx].is_expert) continue;

    const auto start_id =
        scheduler_outputs[idx].drive_passage.lane_path().front().lane_id();
    auto cur_cost =
        ComputeTrajCost(cost_features, weights, scheduler_outputs[idx],
                        results[idx], selector_debug->add_traj_costs());
    if (!traj_costs.contains(start_id) || cur_cost < traj_costs.at(start_id)) {
      // Keep only one best trajectory for each start lane.
      traj_costs[start_id] = std::move(cur_cost);
      id_idx_map[start_id] = idx;
    }
  }

  int best_traj_idx = -1;
  double best_cost = std::numeric_limits<double>::max();
  // Only compare common cost features here.
  for (const auto &[lane_id, cost] : traj_costs) {
    if (cost.cost_common < best_cost) {
      best_traj_idx = id_idx_map.at(lane_id);
      best_cost = cost.cost_common;
    }
  }

  if (best_traj_idx == -1) {
    std::stringstream fail_reason;
    fail_reason << "No valid trajectory added for selection:\n";
    for (int idx = 0; idx < est_status.size(); ++idx) {
      fail_reason << "Task " << idx << ": " << est_status[idx].message();
    }
    return absl::NotFoundError(fail_reason.str());
  }

  if (scheduler_outputs[best_traj_idx].lane_change_state.has_stage()) {
    if (scheduler_outputs[best_traj_idx].lane_change_state.stage() ==
        LaneChangeStage::LCS_PAUSE) {
      QEVENT_EVERY_N_SECONDS("zixuan", "lane_change_pause_selected",
                             /*every_n_seconds=*/5.0, [](QEvent *qevent) {});
    }
  }

  if (FLAGS_dumping_selector_features) {
    RETURN_IF_ERROR(DumpEvaluations(input, cost_features, weights,
                                    scheduler_outputs, est_status, results,
                                    selector_debug));
  }

  return best_traj_idx;
}

}  // namespace qcraft::planner
