#include "onboard/planner/initializer/search_motion.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/async/async_macro.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/decision/leading_object.h"
#include "onboard/planner/initializer/brute_force_collision_checker.h"
#include "onboard/planner/initializer/dp_motion_searcher.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/reference_line_searcher.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_params.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
constexpr double kHighSpeedThreshold = 14.0;              // m/s ~ 50 km/h.
constexpr double kMediumSpeedThreshold = 8.0;             // m/s ~ 30 km/h.
constexpr double kSpeedHysteresis = 1.0;                  // m/s.
constexpr double kStationaryObjectCollisionBuffer = 0.3;  // m.
constexpr double kMovingObjectCollisionBuffer = 0.3;      // m.
constexpr double kReduceStaticBufferThresholdS = 15.0;    // m.
constexpr double kMaxSamplingDistance = 220.0;            // meters.
constexpr double kMaxSamplingLookForwardTime = 15.0;      // seconds.
constexpr double kMinSamplingDistance = 60.0;
constexpr double kInitializerLKTimeConsumptionReportThreshold = 30;  // ms.
constexpr double kInitializerLCTimeConsumptionReportThreshold = 50;  // ms.
constexpr double kQeventSeconds = 5.0;                               // s.

GeometryGraphSamplingStrategy ParseStrategy(
    const InitializerConfig::InitializerSamplePattern &sample_pattern,
    const LaneChangeStateProto &lane_change_state) {
  GeometryGraphSamplingStrategy strategy;
  bool is_lane_change = false;
  if (lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
      lane_change_state.stage() == LaneChangeStage::LCS_PAUSE) {
    is_lane_change = true;
  }

  strategy.is_lane_change = is_lane_change;
  const int config_len = sample_pattern.config().range().size();
  strategy.range_list.reserve(config_len);
  strategy.layer_stations_list.reserve(config_len);
  strategy.lateral_resolution_list.reserve(config_len);
  strategy.cross_layer_connection_list.reserve(config_len);
  strategy.unit_length_lateral_span_list.reserve(config_len);

  for (const auto &val : sample_pattern.config().range()) {
    strategy.range_list.push_back(val);
  }
  for (const auto &val : sample_pattern.config().layer_station()) {
    strategy.layer_stations_list.push_back(val);
  }
  for (const auto &val : sample_pattern.config().lateral_resolution()) {
    strategy.lateral_resolution_list.push_back(val);
  }
  for (const auto &val : sample_pattern.config().cross_layer_connection()) {
    strategy.cross_layer_connection_list.push_back(val);
  }
  for (const auto &val : sample_pattern.config().unit_length_lateral_span()) {
    strategy.unit_length_lateral_span_list.push_back(val);
  }
  return strategy;
}

GeometryGraphSamplingStrategy FindStrategy(
    InitializerSamplePatternConfig sample_pattern_config,
    const InitializerConfig &initializer_config,
    const LaneChangeStateProto &lane_change_state) {
  InitializerConfig::InitializerSamplePattern::Scenario scenario =
      InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_KEEPING;
  if (lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
      lane_change_state.stage() == LaneChangeStage::LCS_PAUSE) {
    scenario =
        InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_CHANGE;
  }
  for (const auto &sample_pattern : initializer_config.sample_patterns()) {
    if (sample_pattern.config_name() == sample_pattern_config &&
        sample_pattern.scenario() == scenario) {
      return ParseStrategy(sample_pattern, lane_change_state);
    }
  }
  // Cannot find a sample pattern, qcheck failed.
  QCHECK(false);
  return GeometryGraphSamplingStrategy();
}

InitializerSamplePatternConfig FindPattern(
    double cur_v, InitializerSamplePatternConfig prev_sample_config) {
  double high_speed_threshold = kHighSpeedThreshold;
  double medium_speed_threshold = kMediumSpeedThreshold;
  switch (prev_sample_config) {
    case InitializerSamplePatternConfig::ISC_NONE:
      break;
    case InitializerSamplePatternConfig::ISC_HIGH_SPEED:
      high_speed_threshold = high_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_MEDIUM_SPEED:
      medium_speed_threshold = medium_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_LOW_SPEED:
      break;
  }
  if (cur_v > high_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_HIGH_SPEED;
  } else if (cur_v > medium_speed_threshold && cur_v <= high_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_MEDIUM_SPEED;
  } else {
    return InitializerSamplePatternConfig::ISC_LOW_SPEED;
  }
}

// Get corresponding sample pattern based on current speed.
std::pair<InitializerSamplePatternConfig, GeometryGraphSamplingStrategy>
GetSamplingStrategy(const InitializerConfig &config,
                    const LaneChangeStateProto &lane_change_state, double cur_v,
                    InitializerSamplePatternConfig prev_sample_config) {
  InitializerSamplePatternConfig cur_pattern =
      FindPattern(cur_v, prev_sample_config);
  GeometryGraphSamplingStrategy strategy =
      FindStrategy(cur_pattern, config, lane_change_state);
  return std::make_pair(std::move(cur_pattern), std::move(strategy));
}

}  // namespace
absl::StatusOr<MotionSearchOutput> SearchMotion(
    const MotionSearchInput &input, ThreadPool *thread_pool,
    InitializerDebugProto *debug_proto) {
  absl::StatusOr<MotionSearchOutput> output_or;
  switch (input.planner_params->initializer_params().search_algorithm()) {
    case InitializerConfig::DP:
      output_or = DpSearchForRawTrajectory(input, thread_pool, debug_proto);
      break;
  }
  return output_or;
}

absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput &initializer_input,
    InitializerDebugProto *debug_proto,
    vis::vantage::ChartDataBundleProto *charts, ThreadPool *thread_pool) {
  const auto &plan_start_point =
      initializer_input.start_point_info->start_point;
  const auto plan_time = initializer_input.start_point_info->plan_time;
  const auto &sdc_pose = *initializer_input.sdc_pose;
  const auto &lane_change_state = *initializer_input.lane_change_state;
  const auto lc_multiple_traj = initializer_input.lc_multiple_traj;
  const auto &lc_clearance = *initializer_input.lc_clearance;
  const auto &drive_passage = *initializer_input.drive_passage;
  const auto &st_traj_mgr = *initializer_input.st_traj_mgr;
  const auto &vehicle_geom = *initializer_input.vehicle_geom;
  const auto &vehicle_drive = *initializer_input.vehicle_drive;
  const auto &path_sl_boundary = *initializer_input.sl_boundary;
  const auto &initializer_state = *initializer_input.prev_initializer_state;
  const auto &constraint_manager = *initializer_input.constraint_manager;
  const auto &planner_params = *initializer_input.planner_params;
  const auto plan_id = initializer_input.plan_id;
  const auto &stalled_objects = *initializer_input.stalled_objects;

  SCOPED_QTRACE("EstPlanner/Initializer");
  ScopedMultiTimer initializer_timer("initializer_debug");
  initializer_timer.Mark("initializer start");
  auto mutable_start_point = plan_start_point;  // Copy
  mutable_start_point.mutable_path_point()->set_theta(NormalizeAngle(
      mutable_start_point.path_point().theta()));  // Fix heading angle.
  // Create collision checker.
  std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_traj_mgr, &vehicle_geom, MotionEdgeInfo::kSampleStep,
          kStationaryObjectCollisionBuffer, kMovingObjectCollisionBuffer);

  InitializerSamplePatternConfig prev_config =
      InitializerSamplePatternConfig::ISC_NONE;
  if (initializer_state.has_sample_pattern_config()) {
    prev_config = initializer_state.sample_pattern_config();
  }
  const auto sample_strategy =
      GetSamplingStrategy(planner_params.initializer_params(),
                          lane_change_state, plan_start_point.v(), prev_config);
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos));

  debug_proto->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));

  const bool is_lane_change =
      (lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING) ||
      (lane_change_state.stage() == LaneChangeStage::LCS_PAUSE);

  // Get smooth reference line max length.
  const double kSamplingDistanceBySpeed = std::max(
      plan_start_point.v() * kMaxSamplingLookForwardTime, kMinSamplingDistance);
  double max_sampling_acc_s =
      std::min<double>(kMaxSamplingDistance, kSamplingDistanceBySpeed) +
      ego_sl.s;
  max_sampling_acc_s =
      std::min<double>(drive_passage.end_s(), max_sampling_acc_s);

  // Get s_from_start.
  double s_from_start = 0.0;
  if (!initializer_input.start_point_info->reset) {
    if (initializer_state.has_s_from_start() &&
        initializer_state.has_prev_start_point()) {
      const auto prev_pos = Vec2dFromApolloTrajectoryPointProto(
          initializer_state.prev_start_point());
      ASSIGN_OR_RETURN(const auto prev_sl,
                       drive_passage.QueryFrenetCoordinateAt(prev_pos));
      s_from_start = initializer_state.s_from_start() - prev_sl.s + ego_sl.s;
    }
  }

  absl::StatusOr<XYGeometryGraph> geom_graph_or;
  auto graph_cache = std::make_unique<GeometryGraphCache>();
  const double s_from_start_with_diff = s_from_start - ego_sl.s;
  auto form_builder = std::make_unique<GeometryFormBuilder>(
      &drive_passage, max_sampling_acc_s, s_from_start_with_diff);

  switch (planner_params.initializer_params().search_algorithm()) {
    case InitializerConfig::DP:
      const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
          .passage = &drive_passage,
          .sl_boundary = &path_sl_boundary,
          .constraint_manager = &constraint_manager,
          .st_traj_mgr = &st_traj_mgr,
          .plan_start_point = &mutable_start_point,
          .s_from_start = s_from_start,
          .vehicle_geom = &vehicle_geom,
          .collision_checker = collision_checker.get(),
          .sampling_params = &sample_strategy.second,
          .vehicle_drive = &vehicle_drive,
          .form_builder = form_builder.get(),
          .lc_multiple_traj = lc_multiple_traj};
      geom_graph_or = BuildCurvyGeometryGraph(
          geom_graph_builder_input, /*retry_collision_checker=*/false,
          graph_cache.get(), thread_pool, debug_proto);
      if (geom_graph_or.ok()) {
        // If the constructed graph is short & blocked by static obj, we try a
        // smaller buffer.
        const auto &geom_end_info = geom_graph_or->GetGeometryGraphEndInfo();
        if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ &&
            geom_end_info.end_accumulated_s() - ego_sl.s <
                kReduceStaticBufferThresholdS) {
          VLOG(2)
              << "Failed to construct graph, try a smaller stationary object "
                 "buffer.";
          collision_checker->UpdateStationaryObjectBuffer(
              0.5 * kStationaryObjectCollisionBuffer);
          geom_graph_or = BuildCurvyGeometryGraph(
              geom_graph_builder_input, /*retry_collision_checker=*/true,
              graph_cache.get(), thread_pool, debug_proto);
        }
      }
      initializer_timer.Mark("build curvy xy geometry graph");
      break;
  }
  if (!geom_graph_or.ok()) {
    return geom_graph_or.status();
  }
  VLOG(1) << "Done build geometry graph";

  // Add blocking object stop line to constraint manager if the geometry graph
  // is blocked by some object.
  const auto &geom_end_info = geom_graph_or->GetGeometryGraphEndInfo();
  std::unique_ptr<ConstraintProto::LeadingObjectProto> blocking_static_obj =
      nullptr;
  if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ) {
    blocking_static_obj =
        std::make_unique<ConstraintProto::LeadingObjectProto>();
    const auto trajs =
        st_traj_mgr.FindTrajectoriesByObjectId(geom_end_info.object_id());
    QCHECK_GT(trajs.size(), 0);
    blocking_static_obj->set_traj_id(std::string(trajs[0]->traj_id()));
    blocking_static_obj->set_reason(
        ConstraintProto::LeadingObjectProto::BLOCKING_STATIC);
  }

  absl::StatusOr<ReferenceLineSearcherOutput> ref_line_output_or;
  if (FLAGS_planner_initializer_use_refline_search && !is_lane_change) {
    ReferenceLineSearcherInput ref_line_search_input{
        .geometry_graph = &geom_graph_or.value(),
        .drive_passage = &drive_passage,
        .sl_boundary = &path_sl_boundary,
        .st_traj_mgr = &st_traj_mgr,
        .planner_params = &planner_params,
        .vehicle_geom = &vehicle_geom,
        .vehicle_drive = &vehicle_drive,
    };
    ref_line_output_or =
        SearchReferenceLine(ref_line_search_input, debug_proto, thread_pool);
  }
  auto *graph_proto = debug_proto->mutable_geom_graph();
  graph_proto->Clear();
  if (ref_line_output_or.ok()) {
    RETURN_IF_ERROR(ActivateGeometryGraph(*ref_line_output_or, path_sl_boundary,
                                          &(*geom_graph_or)));
    ParseReferenceLineResultToProto(*ref_line_output_or, graph_proto);
    if (FLAGS_planner_initializer_debug_level >= 2) {
      SendPathPointsToCanvas((*ref_line_output_or).ref_line_points,
                             "reference_line", plan_id, vis::Color::kLightGreen,
                             6);
    }
    initializer_timer.Mark("search reference line");
  }

  // Send geometry graph & refline search to proto.
  if (FLAGS_planner_initializer_debug_level >= 2) {
    geom_graph_or->ToProto(graph_proto);
    SendGeometryGraphToCanvas(
        &geom_graph_or.value(),
        absl::StrCat("geom_graph_", initializer_input.plan_id));
  }

  // Construct InitializerSearchConfig here using leading object proto,
  // construct multiple trajectory search config if necessary.

  const auto search_configs = BuildSearchConfig(
      drive_passage, path_sl_boundary, st_traj_mgr, constraint_manager,
      &lc_clearance, stalled_objects, blocking_static_obj.get(),
      geom_graph_or.value(), vehicle_geom, is_lane_change,
      lane_change_state.lc_left(), lc_multiple_traj);

  MotionSearchInput input{
      .start_point = &mutable_start_point,
      .plan_time = plan_time,
      .sdc_pose = &sdc_pose,
      .drive_passage = &drive_passage,
      .st_traj_mgr = &st_traj_mgr,
      .vehicle_geom = &vehicle_geom,
      .geom_graph = &geom_graph_or.value(),
      .form_builder = form_builder.get(),
      .collision_checker = collision_checker.get(),
      .constraint_manager = &constraint_manager,
      .sl_boundary = &path_sl_boundary,
      .planner_params = &planner_params,
      .blocking_static_obj = blocking_static_obj.get(),
      .lc_clearance = &lc_clearance,
      .search_configs = &search_configs,
      .is_lane_change = is_lane_change,
      .lc_multiple_traj = lc_multiple_traj,
  };
  initializer_timer.Mark("create initializer input");

  ASSIGN_OR_RETURN(auto motion_output,
                   SearchMotion(input, thread_pool, debug_proto),
                   _ << "Search Motion failure.");
  initializer_timer.Mark("run search motion");

  if (FLAGS_planner_initializer_debug_level > 0) {
    PrintMultiTimerReportStat(initializer_timer);
  }
  const auto perform_duration =
      absl::ToDoubleMilliseconds(absl::Now() - initializer_timer.start());

  if (is_lane_change &&
      perform_duration > kInitializerLCTimeConsumptionReportThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "changqing", "initializer_LC_time_greater_than_threhold",
        kQeventSeconds, [&](QEvent *qevent) {
          qevent
              ->AddField("num_trajs_considered",
                         motion_output.trajs_with_lead_obj.size())
              .AddField("time", perform_duration);
        });
  }

  if (!is_lane_change &&
      perform_duration > kInitializerLKTimeConsumptionReportThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "changqing", "initializer_LK_time_greater_than_threhold",
        kQeventSeconds, [&](QEvent *qevent) {
          qevent
              ->AddField("num_trajs_considered",
                         motion_output.trajs_with_lead_obj.size())
              .AddField("time", perform_duration);
        });
  }

  ParseMotionSearchOutputToInitializerResultTrajectoryProto(
      motion_output, debug_proto->mutable_resampled_initializer_trajectory());
  // Collect traj with different leading obj info to proto.
  ParseDpMotionSearchTrajectoryWithLeadingObjToProto(
      motion_output, debug_proto->mutable_dp_motion_search_result());

  if (FLAGS_planner_initializer_debug_level >= 2) {
    for (int i = 0; i < motion_output.trajs_with_lead_obj.size(); i++) {
      const auto &traj_info = motion_output.trajs_with_lead_obj.at(i);
      SendSingleTrajectoryToCanvas(traj_info, i, initializer_input.plan_id);
    }
    ParseMotionSearchOutputToSearchResultProto(
        motion_output, debug_proto->mutable_motion_search_result());
    // Collect candidate trajectories to proto.
    ParseDpMotionSearchOutputToDpSearchResultProto(
        motion_output, debug_proto->mutable_dp_motion_search_result());
    SendRefSpeedTableToCanvas(*motion_output.ref_speed_table, drive_passage);
    form_builder->ToProto(debug_proto->mutable_smoothed_drive_passage());
  }
  ExportMoionSpeedProfileToChart(motion_output, charts->add_charts());
  InitializerStateProto new_state;
  new_state.set_sample_pattern_config(sample_strategy.first);
  *new_state.mutable_end_info() = geom_graph_or->GetGeometryGraphEndInfo();
  new_state.set_s_from_start(s_from_start);
  auto *prev = new_state.mutable_prev_start_point();
  *prev = mutable_start_point;

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(form_builder);
  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(graph_cache);

  if (FLAGS_dumping_initializer_features) {
    ParseFeaturesDumpingProto(motion_output,
                              debug_proto->mutable_expert_evaluation(),
                              debug_proto->mutable_candidates_evaluation());
  }

  std::optional<std::vector<ConstraintProto::LeadingObjectProto>> lc_targets =
      std::nullopt;
  if (lc_multiple_traj) {
    lc_targets = ConvertLaneChangeTargets(drive_passage, st_traj_mgr,
                                          motion_output.leading_objs);
  }

  return InitializerOutput{
      .search_result = std::move(motion_output),
      .initializer_state = std::move(new_state),
      .blocking_static_obj = std::move(blocking_static_obj),
      .lc_targets = std::move(lc_targets)};
}

}  // namespace qcraft::planner
