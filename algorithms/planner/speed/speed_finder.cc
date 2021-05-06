#include "onboard/planner/speed/speed_finder.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/speed/constraint_generator.h"
#include "onboard/planner/speed/constraint_manager_decider.h"
#include "onboard/planner/speed/decider/post_st_boundary_modifier.h"
#include "onboard/planner/speed/decider/pre_st_boundary_modifier.h"
#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"
#include "onboard/planner/speed/ignore_decider.h"
#include "onboard/planner/speed/interactive_speed_decision.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/path_speed_combiner.h"
#include "onboard/planner/speed/plot_util.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_limit_generator.h"
#include "onboard/planner/speed/speed_optimizer.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/speed/st_overlap_analyzer.h"
#include "onboard/planner/speed/standstill_distance_decider.h"
#include "onboard/planner/speed/time_buffer_decider.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/utils/status_macros.h"

DEFINE_bool(
    planner_draw_st_boundary_canvas, false,
    "Whether to export upstream input speed profile to st-graph chart.");

DEFINE_bool(planner_send_speed_path_chart_data, false,
            "Whether to send speed path chart data.");

DEFINE_bool(planner_send_interactive_speed_to_chart, false,
            "Whether to send interactive speed profiles to chart.");

DEFINE_bool(planner_print_speed_finder_time_stats, false,
            "Whether to print speed finder time stats.");

namespace qcraft {
namespace planner {

namespace {
using SpeedLimitType = SpeedFinderParamsProto::SpeedLimitType;
using ChartSeriesDataProto = vis::vantage::ChartSeriesDataProto;
using PartialStTraj = PartialSpacetimeObjectTrajectory;
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
using Sfp = SpeedFinderParamsProto;

void KeepNearestStationarySpacetimeTrajectoryStBoundary(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  double nearest_stationary_s = std::numeric_limits<double>::max();
  const std::string* nearest_stationary_id_ptr = nullptr;
  for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_with_decision.raw_st_boundary();
    if (st_boundary.source_type() != StBoundary::SourceType::ST_OBJECT) {
      continue;
    }
    if (!st_boundary.is_stationary()) continue;
    if (const double follow_s =
            st_boundary.min_s() -
            st_boundary_with_decision.follow_standstill_distance();
        follow_s < nearest_stationary_s) {
      nearest_stationary_s = follow_s;
      nearest_stationary_id_ptr = &st_boundary.id();
    }
  }
  if (nearest_stationary_id_ptr != nullptr) {
    st_boundaries_with_decision->erase(
        std::remove_if(
            st_boundaries_with_decision->begin(),
            st_boundaries_with_decision->end(),
            [nearest_stationary_id_ptr](
                const StBoundaryWithDecision& st_boundary_with_decision) {
              const auto& st_boundary =
                  *st_boundary_with_decision.raw_st_boundary();
              if (st_boundary.source_type() !=
                  StBoundary::SourceType::ST_OBJECT) {
                return false;
              }
              if (!st_boundary.is_stationary()) return false;
              return st_boundary.id() != *nearest_stationary_id_ptr;
            }),
        st_boundaries_with_decision->end());
  }
}

absl::Status OptimizeSpeed(
    const ApolloTrajectoryPointProto& init_point, int traj_steps,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const absl::flat_hash_map<SpeedLimitType, SpeedLimit>& speed_limit,
    double path_length,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedVector& speed_profile, SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  SCOPED_QTRACE("OptimizeSpeed");

  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  SpeedOptimizer speed_optimizer(
      &motion_constraint_params, &speed_finder_params, path_length,
      motion_constraint_params.default_speed_limit(), traj_steps);
  const absl::Status status = speed_optimizer.Optimize(
      init_point, st_boundaries_with_decision, speed_limit, speed_profile,
      optimized_speed, speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

absl::Status OptimizeFreespaceSpeed(
    const ApolloTrajectoryPointProto& init_point, int traj_steps,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double path_length, bool forward,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  SCOPED_QTRACE("OptimizeSpeed");

  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  SpeedOptimizer speed_optimizer(
      &motion_constraint_params, &speed_finder_params, path_length,
      forward ? motion_constraint_params.default_speed_limit()
              : motion_constraint_params.default_reverse_speed_limit(),
      traj_steps);

  const absl::Status status = speed_optimizer.OptimizeWithMaxSpeed(
      init_point, st_boundaries_with_decision, optimized_speed,
      speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Freespace speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

void SetStBoundaryDebugInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedFinderDebugProto* speed_finder_proto) {
  QCHECK_NOTNULL(speed_finder_proto);
  speed_finder_proto->mutable_st_boundaries()->clear();
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    StBoundaryProto st_boundary_proto;
    const auto* st_boundary = boundary_with_decision.st_boundary();
    st_boundary_proto.set_decision_type(boundary_with_decision.decision_type());
    st_boundary_proto.set_decision_reason(
        boundary_with_decision.decision_reason());
    st_boundary_proto.set_decision_info(
        std::string(boundary_with_decision.decision_info()));
    st_boundary_proto.set_follow_standstill_distance(
        boundary_with_decision.follow_standstill_distance());
    st_boundary_proto.set_lead_standstill_distance(
        boundary_with_decision.lead_standstill_distance());
    st_boundary_proto.set_object_type(st_boundary->object_type());
    st_boundary_proto.set_probability(st_boundary->probability());
    st_boundary_proto.set_min_s(st_boundary->min_s());
    st_boundary_proto.set_max_s(st_boundary->max_s());
    st_boundary_proto.set_min_t(st_boundary->min_t());
    st_boundary_proto.set_max_t(st_boundary->max_t());
    st_boundary_proto.set_is_stationary(st_boundary->is_stationary());
    speed_finder_proto->mutable_st_boundaries()->insert(
        {st_boundary->id(), st_boundary_proto});
  }
}

std::optional<TrajectoryEndInfo> SetTrajectoryEndInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double speed_length) {
  std::optional<TrajectoryEndInfo> traj_end_info = std::nullopt;
  for (const StBoundaryWithDecision& st_boundary_with_decision :
       st_boundaries_with_decision) {
    const auto* st_boundary = st_boundary_with_decision.st_boundary();
    if (!st_boundary->is_stationary() ||
        st_boundary->source_type() != StBoundary::SourceType::ST_OBJECT) {
      continue;
    }

    const double min_s = st_boundary->min_s();
    const double object_upper_bound =
        min_s - st_boundary_with_decision.follow_standstill_distance();
    const double intrusion_value = speed_length - object_upper_bound;

    if (intrusion_value > 0.0) {
      if (!traj_end_info.has_value() || min_s < traj_end_info->end_s) {
        traj_end_info = {.st_traj_id = st_boundary->id(),
                         .end_s = min_s,
                         .intrusion_value = intrusion_value};
      }
    }
  }
  return traj_end_info;
}

std::vector<PartialStTraj> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects) {
  absl::flat_hash_map<std::string, PartialStTraj> considered_st_objects_map;
  considered_st_objects_map.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto decision_type = st_boundary_with_decision.decision_type();
    if (decision_type == StBoundaryProto::IGNORE ||
        decision_type == StBoundaryProto::UNKNOWN ||
        st_boundary_with_decision.st_boundary()->source_type() !=
            StBoundary::SourceType::ST_OBJECT) {
      continue;
    }
    const auto& traj_id = st_boundary_with_decision.traj_id();
    QCHECK(traj_id.has_value());
    const auto* raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (!considered_st_objects_map.contains(*traj_id)) {
      // Judge whether the spacetime_object corresponding to current st-boundary
      // has been processed.
      if (processed_st_objects.contains(*traj_id)) {
        // Use st_object from processed_st_objects.
        considered_st_objects_map.emplace(
            *traj_id, PartialStTraj(std::move(
                          FindOrDie(processed_st_objects, *traj_id))));
      } else {
        // Use st_object from obj_mgr.
        considered_st_objects_map.emplace(
            *traj_id,
            PartialStTraj(QCHECK_NOTNULL(obj_mgr.FindTrajectoryById(*traj_id))
                              ->CreateCopy()));
      }
    }
    QCHECK(decision_type == StBoundaryProto::LEAD ||
           decision_type == StBoundaryProto::FOLLOW);
    FindOrDie(considered_st_objects_map, *traj_id)
        .AppendTimeRangeAndDecisonType(
            raw_st_boundary->min_t(), raw_st_boundary->max_t(),
            decision_type == StBoundaryProto::LEAD
                ? PartialStTraj::DecisionType::LEAD
                : PartialStTraj::DecisionType::FOLLOW);
  }
  std::vector<PartialStTraj> considered_st_objects;
  considered_st_objects.reserve(considered_st_objects_map.size());
  for (auto& [_, st_obj] : considered_st_objects_map) {
    considered_st_objects.push_back(std::move(st_obj));
  }
  return considered_st_objects;
}

std::vector<StBoundaryWithDecision> InitializeStBoundaryWithDecision(
    std::vector<StBoundaryRef> raw_st_boundaries) {
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.reserve(raw_st_boundaries.size());
  for (auto& st_boundary : raw_st_boundaries) {
    // Initialize with unknown-decision.
    st_boundaries_with_decision.emplace_back(std::move(st_boundary));
  }
  return st_boundaries_with_decision;
}

ObjectsPredictionProto ConstructModifiedPrediction(
    const absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects) {
  // Use these elements to construct ObjectPrediction.
  struct PredElements {
    std::vector<prediction::PredictedTrajectory> trajs;
    const ObjectProto* object_proto = nullptr;
  };

  absl::flat_hash_map<std::string, PredElements> processed_preds;
  for (const auto& [traj_id, st_obj] : processed_st_objects) {
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj_id);
    if (processed_preds[object_id].object_proto == nullptr) {
      processed_preds[object_id].object_proto =
          &st_obj.planner_object()->object_proto();
    }
    processed_preds[object_id].trajs.push_back(*st_obj.trajectory());
  }

  ObjectsPredictionProto modified_prediction;
  for (auto& [_, pred_elements] : processed_preds) {
    const prediction::ObjectPrediction obj_pred(std::move(pred_elements.trajs),
                                                *pred_elements.object_proto);
    obj_pred.ToProto(modified_prediction.add_objects());
  }

  return modified_prediction;
}

std::vector<StPoint> GenerateAvEmergencyStopStPoints(double init_s,
                                                     double init_v,
                                                     int traj_steps) {
  QCHECK_GE(init_v, 0.0);
  // In line with max deceleration but can differ from it.
  constexpr double kEmergencyStopAccel = -3.0;  // m/s^2.
  // In line with path resoluation but can differ from it.
  constexpr double kCheckCollisionTime = 4.0;  // s.
  double curr_s = init_s;
  double curr_v = init_v;
  std::vector<StPoint> emergency_stop_points;
  emergency_stop_points.reserve(traj_steps);
  double t = 0.0;
  for (int i = 0; i < traj_steps && t < kCheckCollisionTime;
       ++i, t += kTrajectoryTimeStep) {
    emergency_stop_points.emplace_back(curr_s, t);
    curr_s +=
        std::max(0.0, curr_v * kTrajectoryTimeStep +
                          0.5 * kEmergencyStopAccel * Sqr(kTrajectoryTimeStep));
    curr_v = std::max(0.0, curr_v + kEmergencyStopAccel * kTrajectoryTimeStep);
  }
  return emergency_stop_points;
}

int GetSpeedFinderTrajectorySteps(double init_v, double default_speed_limit) {
  constexpr double kMaxSpeedOfFixedTrajectorySteps = 22.23;  // m/s.->80km/h
  if (default_speed_limit <= kMaxSpeedOfFixedTrajectorySteps) {
    return kTrajectorySteps;
  }

  const PiecewiseLinearFunction<double> plf = {
      {kMaxSpeedOfFixedTrajectorySteps, default_speed_limit},
      {kTrajectorySteps, kSpeedFinderMaxTrajectorySteps}};
  return static_cast<int>(plf(init_v));
}

}  // namespace

absl::StatusOr<SpeedFinderOutput> FindSpeed(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    ThreadPool* thread_pool) {
  if (input.base_name == "est") {
    SCOPED_QTRACE("EstPlanner/FindSpeed");
  } else if (input.base_name == "fallback") {
    SCOPED_QTRACE("FallbackPlanner/FindSpeed");
  }

  ScopedMultiTimer timer("speed_finder");
  const absl::Cleanup timeout_trigger = [start_time = absl::Now()]() {
    constexpr double kSpeedFinderTimeLimitMs = 30.0;
    const auto speed_total_time_ms =
        absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (speed_total_time_ms > kSpeedFinderTimeLimitMs) {
      QEVENT_EVERY_N_SECONDS("ping", "speed_finder_timeout", 10.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("time_ms", speed_total_time_ms);
                             });
    }
  };

  SpeedFinderOutput output;

  const auto& input_path = *input.path;
  QCHECK(!input_path.empty());
  if (input_path.size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("Input path has only 1 path point: ",
                     input_path.front().DebugString()));
  }

  for (const auto& p : input_path) {
    *output.speed_finder_proto.add_path() = p;
  }

  const int trajectory_steps = GetSpeedFinderTrajectorySteps(
      input.plan_start_point.v(),
      Mph2Mps(motion_constraint_params.default_speed_limit()));

  /*
   *  Step 1: Map st boundaries of objects onto st graph.
   *
   */
  auto start_time = absl::Now();
  auto st_graph =
      std::make_unique<StGraph>(input_path, /*forward=*/true, trajectory_steps,
                                &vehicle_geometry_params, &speed_finder_params);

  auto st_boundaries = st_graph->GetStBoundaries(
      input.traj_mgr->moving_object_trajs(),
      input.traj_mgr->stationary_object_trajs(), *input.constraint_mgr,
      *input.psmm->semantic_map_manager(), thread_pool);
  VLOG(2) << "Build st_graph cost time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(3) << "st_boundary size = " << st_boundaries.size();
  timer.Mark("map_st_boundaries");

  /*
   *  Step 2: Analyze overlap.
   *
   */
  std::vector<PathPointSemantic> path_semantics;
  // Analyze path semantics.
  int max_analyze_path_index = -1;
  for (const auto& st_boundary : st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    for (const auto& overlap_info : st_boundary->overlap_infos()) {
      max_analyze_path_index =
          std::max(max_analyze_path_index, overlap_info.av_end_idx);
    }
  }
  if (max_analyze_path_index >= 0) {
    const auto route_sections_within_dp = ClampRouteSectionsBeforeArcLength(
        *input.psmm, *input.route_sections_from_start,
        input.drive_passage->lane_path().length());
    QCHECK(route_sections_within_dp.ok());
    VLOG(3) << "route_sections_within_dp: "
            << route_sections_within_dp->DebugString();

    start_time = absl::Now();
    const auto path_semantics_or = AnalyzePathSemantics(
        input_path, max_analyze_path_index, *route_sections_within_dp,
        *input.psmm, thread_pool);
    VLOG(2) << "Analyze path semantics cost time(ms): "
            << absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (path_semantics_or.ok() && VLOG_IS_ON(4)) {
      for (int i = 0; i < path_semantics_or->size(); ++i) {
        VLOG(4) << "Path point[" << i << "]: (" << input_path[i].x() << ", "
                << input_path[i].y() << "), closest lane point "
                << (*path_semantics_or)[i].closest_lane_point.DebugString()
                << ", lane path id history size "
                << (*path_semantics_or)[i].lane_path_id_history.size();
        for (int j = 0; j < (*path_semantics_or)[i].lane_path_id_history.size();
             ++j) {
          VLOG(4) << "Path point[" << i << "] lane path id [" << j
                  << "]: " << (*path_semantics_or)[i].lane_path_id_history[j];
        }
      }
    }

    if (path_semantics_or.ok()) {
      AnalyzeStOverlaps(input_path, *path_semantics_or, *input.psmm,
                        *input.traj_mgr, vehicle_geometry_params,
                        input.plan_start_point.v(), &st_boundaries);
      path_semantics = std::move(*path_semantics_or);
      if (VLOG_IS_ON(4)) {
        for (const auto& st_boundary : st_boundaries) {
          if (st_boundary->overlap_meta().has_value()) {
            const auto& overlap_meta = *st_boundary->overlap_meta();
            VLOG(4) << "St-boundary " << st_boundary->id()
                    << " overlap pattern: "
                    << StOverlapMetaProto::OverlapPattern_Name(
                           overlap_meta.pattern())
                    << ", source: "
                    << StOverlapMetaProto::OverlapSource_Name(
                           overlap_meta.source())
                    << ", priority: "
                    << StOverlapMetaProto::OverlapPriority_Name(
                           overlap_meta.priority())
                    << ", priority reason: " << overlap_meta.priority_reason()
                    << ", modification type: "
                    << StOverlapMetaProto::ModificationType_Name(
                           overlap_meta.modification_type());
          }
        }
      }
    } else {
      QLOG(WARNING)
          << "Path semantic analyzer fails, skip analyzing overlap meta: "
          << path_semantics_or.status().message();
    }
  }

  /*
   *  Step 3: Pre-decision.
   *
   */
  // Initialze st-boudaries with decision.
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  // Set follow/lead standstill distance for st-boundaries.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = input.stalled_objects,
      .lane_path = &input.drive_passage->lane_path(),
      .st_traj_mgr = input.traj_mgr,
      .constraint_mgr = input.constraint_mgr};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }
  // Only keep the nearest stationary spacetime trajectory st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  const auto emergency_stop_points = GenerateAvEmergencyStopStPoints(
      input_path.front().s(), input.plan_start_point.v(), trajectory_steps);

  const auto ignore_decider_input =
      IgnoreDeciderInput({.emergency_stop_points = &emergency_stop_points,
                          .path = &input_path,
                          .path_semantics = &path_semantics,
                          .st_traj_mgr = input.traj_mgr,
                          .vehicle_geometry_params = &vehicle_geometry_params});
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    MakeIgnoreDecisionForStBoundary(ignore_decider_input,
                                    &st_boundary_with_decision);
  }

  // Make decisions for st-boundaries in respect of upstream constraints.
  MakeConstraintDecisionForStBoundaries(*input.constraint_mgr,
                                        &st_boundaries_with_decision);

  // Set pass/yield time buffers for st-boundaries.
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideTimeBuffersForStBoundary(&st_boundary_with_decision);
  }
  if (VLOG_IS_ON(4)) {
    for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
      VLOG(4) << "Set st-boundary " << st_boundary_with_decision.id()
              << " pass_time: " << st_boundary_with_decision.pass_time()
              << " yield_time: " << st_boundary_with_decision.yield_time();
    }
  }

  absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;

  const PreStboundaryModifierInput pre_st_boundary_modifier_input{
      .st_graph = st_graph.get(),
      .st_traj_mgr = input.traj_mgr,
      .current_v = input.plan_start_point.v(),
      .path = &input_path};
  PreModifyStBoundaries(pre_st_boundary_modifier_input,
                        &st_boundaries_with_decision, &processed_st_objects);

  // NOTE(renjie): We may need to append more path semantics after this point
  // because there may have been newly generated st-boundaries.

  // Add some additional constraints (speed region, stop line, etc) to
  // constraint manager.
  // TODO(ping): Differentiate the speed regions generated by speed finder which
  // are based on the path and those generated by upstream modules which are
  // based on drive passage.
  output.constraint_mgr = *input.constraint_mgr;
  const SpeedConstraintGeneratorInput constraint_generator_input{
      .st_boundaries_with_decision = &st_boundaries_with_decision,
      .st_graph = st_graph.get(),
      .st_traj_mgr = input.traj_mgr,
      .path = &input_path,
      .drive_passage = input.drive_passage,
      .path_sl_boundary = input.path_sl_boundary,
      .speed_decider_params = &speed_finder_params.speed_decider_params()};
  GenerateSpeedConstraints(constraint_generator_input, &output.constraint_mgr);

  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Speed regions in mutable constraint manager: ";
    for (const auto& speed_region : output.constraint_mgr.SpeedRegion()) {
      VLOG(3) << speed_region.DebugString();
    }
  }
  timer.Mark("constraint_decisions");

  /*
   *  Step 4: Calculate speed limit.
   *
   */
  const double planner_speed_cap =
      Mph2Mps(motion_constraint_params.default_speed_limit());  // m/s.
  const auto speed_limit_map = GetSpeedLimitMap(
      input_path, planner_speed_cap, vehicle_geometry_params,
      vehicle_drive_params, *input.drive_passage, output.constraint_mgr,
      speed_finder_params.speed_limit_params());
  timer.Mark("get_speed_limit");

  /*
   *  Step 5: Coarse speed planning given st-graph and speed limit, and make
   *          decisions on st-boundaries at the same time.
   */
  start_time = absl::Now();
  SpeedVector preliminary_speed;
  InteractiveSpeedDebugProto interactive_speed_debug;
  RETURN_IF_ERROR(MakeInteractiveSpeedDecision(
      vehicle_geometry_params, motion_constraint_params, *st_graph,
      *input.traj_mgr, FindOrDie(speed_limit_map, Sfp::COMBINATION), input_path,
      input.plan_start_point, speed_finder_params, planner_speed_cap,
      trajectory_steps, &preliminary_speed, &st_boundaries_with_decision,
      &processed_st_objects, &interactive_speed_debug, thread_pool));
  timer.Mark("speed_search");

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);

  /*
   *  Step 6: Speed optimization.
   *
   */
  start_time = absl::Now();

  SpeedVector optimized_speed;
  RETURN_IF_ERROR(OptimizeSpeed(input.plan_start_point, trajectory_steps,
                                st_boundaries_with_decision, speed_limit_map,
                                input_path.length(), motion_constraint_params,
                                speed_finder_params, preliminary_speed,
                                &optimized_speed, &output.speed_finder_proto));

  constexpr double kSpeedOptimizerTimeLimitMs = 15.0;
  const auto speed_optimizer_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(2) << "Speed optimizer cost time(ms): " << speed_optimizer_time;
  if (speed_optimizer_time > kSpeedOptimizerTimeLimitMs) {
    QEVENT("ping", "speed_optimizer_timeout", [&](QEvent* qevent) {
      qevent->AddField("speed_optimizer_running_time[ms]", speed_optimizer_time)
          .AddField("time_limit[ms]", kSpeedOptimizerTimeLimitMs);
    });
  }
  QCHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";
  timer.Mark("speed_optimization");

  // Remove the speed points after kTrajectoryTimeHorizon seconds.
  // Raw optimized_speed will be reserved for plotting.
  SpeedVector final_optimized_speed;
  final_optimized_speed.reserve(optimized_speed.size());
  for (const auto& speed_point : optimized_speed) {
    if (speed_point.t() >= kTrajectoryTimeStep * kTrajectorySteps) break;
    final_optimized_speed.push_back(speed_point);
  }

  // Set trajectory end info.
  output.trajectory_end_info = SetTrajectoryEndInfo(
      st_boundaries_with_decision, final_optimized_speed.TotalLength());

  /*
   *  Step 7: Integrate path and speed.
   *
   */
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(final_optimized_speed.size());
  RETURN_IF_ERROR(CombinePathAndSpeed(input_path, /*forward=*/true,
                                      final_optimized_speed,
                                      &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = p;
  }

  if (FLAGS_planner_print_speed_finder_time_stats) {
    PrintMultiTimerReportStat(timer);
  }

  /*
   *  Step 8: Plot speed data to chart.
   *
   */
  // st_graph chart.
  speed::ExportStBoundaryToChart(
      input.base_name, trajectory_steps, st_boundaries_with_decision,
      output.speed_finder_proto, input_path.length(), &output.st_graph_chart);
  speed::ExportSpeedStDataToChart(
      preliminary_speed, "preliminary_speed", vis::Color::kOrange,
      ChartSeriesDataProto::DASHLINE, &output.st_graph_chart);
  speed::ExportSpeedStDataToChart(
      optimized_speed, "optimized_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::SOLIDLINE, &output.st_graph_chart);
  if (FLAGS_planner_send_interactive_speed_to_chart) {
    speed::ExportInteractiveSpeedToChart(
        input.base_name, st_boundaries_with_decision, preliminary_speed,
        interactive_speed_debug, output.speed_finder_proto,
        &output.sampling_dp_chart, &output.interactive_speed_chart);
  }

  // vt_graph chart
  speed::ExportSpeedLimitToChart(input.base_name, output.speed_finder_proto,
                                 &output.vt_graph_chart);
  speed::ExportSpeedVtDataToChart(preliminary_speed, "preliminary_speed",
                                  vis::Color::kOrange, &output.vt_graph_chart);
  speed::ExportSpeedVtDataToChart(optimized_speed, "optimized_speed",
                                  vis::Color::kDarkGreen,
                                  &output.vt_graph_chart);

  // Traj chart
  speed::ExportTrajToChart(input.base_name, output_trajectory_points,
                           &output.traj_chart);

  if (FLAGS_planner_send_speed_path_chart_data) {
    speed::ExportPathToChart(input.base_name, input_path, &output.path_chart);
  }

  if (FLAGS_planner_draw_st_boundary_canvas) {
    speed::DrawStBoundaryOnCanvas(input.base_name, st_boundaries_with_decision,
                                  vehicle_geometry_params, *input.traj_mgr,
                                  input_path);
  }

  output.trajectory_points = std::move(output_trajectory_points);
  *output.speed_finder_proto.mutable_modified_prediction() =
      ConstructModifiedPrediction(processed_st_objects);
  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, *input.traj_mgr,
                             std::move(processed_st_objects));

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(st_graph);

  return output;
}

// TODO(ping): Delete this function after refactoring speed finder.
absl::StatusOr<FreespaceSpeedFinderOutput> FindFreespaceSpeed(
    const FreespaceSpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("FreespacePlanner/FindSpeed");

  ScopedMultiTimer timer("speed_finder");
  const absl::Cleanup timeout_trigger = [start_time = absl::Now()]() {
    constexpr double kSpeedFinderTimeLimitMs = 30.0;
    const auto speed_total_time_ms =
        absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (speed_total_time_ms > kSpeedFinderTimeLimitMs) {
      QEVENT_EVERY_N_SECONDS("ping", "freespace_speed_finder_timeout", 10.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("time_ms", speed_total_time_ms);
                             });
    }
  };

  FreespaceSpeedFinderOutput output;

  const auto& input_path = *input.path;
  QCHECK(!input_path.empty());
  if (input_path.size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("Input path has only 1 path point: ",
                     input_path.front().DebugString()));
  }

  auto plan_start_point = input.plan_start_point;
  if (!input.forward) {
    plan_start_point.set_v(-plan_start_point.v());
    plan_start_point.set_a(-plan_start_point.a());
    plan_start_point.set_j(-plan_start_point.j());
  }

  for (const auto& p : input_path) {
    *output.speed_finder_proto.add_path() = p;
  }

  /*
   *  Step 1: Map st boundaries of objects onto st graph.
   *
   */
  auto start_time = absl::Now();
  auto st_graph =
      std::make_unique<StGraph>(input_path, input.forward, kTrajectorySteps,
                                &vehicle_geometry_params, &speed_finder_params);

  auto st_boundaries = st_graph->GetStBoundaries(
      input.obj_mgr->moving_object_trajs(),
      input.obj_mgr->stationary_object_trajs(), *input.constraint_mgr,
      *input.semantic_map_manager, thread_pool);
  VLOG(2) << "Build st_graph cost time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(3) << "st_boundary size = " << st_boundaries.size();
  timer.Mark("map_st_boundaries");

  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));
  // Set follow/lead standstill distance for st-boundaries.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = input.stalled_objects,
      .lane_path = nullptr,
      .st_traj_mgr = input.obj_mgr,
      .constraint_mgr = input.constraint_mgr};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }
  // Only keep the nearest stationary spacetime trajectory st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  /*
   *  Step 2: Make some confident decisions in speed decider.
   *
   */
  // TODO(renjie): Implement freespace speed constraint decisions.
  output.constraint_mgr = *input.constraint_mgr;

  /*
   *  Step 3: Make pre-decision for st_boundary.
   *
   */
  const auto st_pre_decider_input = FreespaceStBoundaryPreDecisionInput{
      .plan_start_point = &plan_start_point,
      .speed_finder_params = &speed_finder_params};
  MakeFreespaceStBoundaryPreDecision(st_pre_decider_input,
                                     &st_boundaries_with_decision);
  timer.Mark("pre_decider");

  // Make follow decisions for all st-boundaries without prior decision.
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundary_with_decision.set_decision_type(StBoundaryProto::FOLLOW);
      st_boundary_with_decision.set_decision_reason(StBoundaryProto::FREESPACE);
    }
  }

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);

  /*
   *  Step 4: Speed optimization.
   *
   */
  start_time = absl::Now();

  SpeedVector optimized_speed;
  RETURN_IF_ERROR(OptimizeFreespaceSpeed(
      plan_start_point, kTrajectorySteps, st_boundaries_with_decision,
      input_path.length(), input.forward, motion_constraint_params,
      speed_finder_params, &optimized_speed, &output.speed_finder_proto));

  constexpr double kSpeedOptimizerTimeLimitMs = 15.0;
  const auto speed_optimizer_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(2) << "Speed optimizer cost time(ms): " << speed_optimizer_time;
  if (speed_optimizer_time > kSpeedOptimizerTimeLimitMs) {
    QEVENT("ping", "speed_optimizer_timeout", [&](QEvent* qevent) {
      qevent->AddField("speed_optimizer_running_time[ms]", speed_optimizer_time)
          .AddField("time_limit[ms]", kSpeedOptimizerTimeLimitMs);
    });
  }
  QCHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";
  timer.Mark("speed_optimization");

  /*
   *  Step 5: Integrate path and speed.
   *
   */
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(kTrajectorySteps);
  RETURN_IF_ERROR(CombinePathAndSpeed(
      input_path, input.forward, optimized_speed, &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = p;
  }

  if (FLAGS_planner_print_speed_finder_time_stats) {
    PrintMultiTimerReportStat(timer);
  }

  /*
   *  Step 6: Plot speed data to chart.
   *
   */
  // st_graph chart
  speed::ExportStBoundaryToChart(
      /*base_name=*/"freespace", kTrajectorySteps, st_boundaries_with_decision,
      output.speed_finder_proto, input_path.length(), &output.st_graph_chart);
  speed::ExportSpeedStDataToChart(
      optimized_speed, "optimized_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::SOLIDLINE, &output.st_graph_chart);

  // vt_graph chart
  speed::ExportSpeedLimitToChart(/*base_name=*/"freespace",
                                 output.speed_finder_proto,
                                 &output.vt_graph_chart);
  speed::ExportSpeedVtDataToChart(optimized_speed, "optimized_speed",
                                  vis::Color::kDarkGreen,
                                  &output.vt_graph_chart);

  // traj chart
  speed::ExportTrajToChart(/*base_name=*/"freespace", output_trajectory_points,
                           &output.traj_chart);

  if (FLAGS_planner_send_speed_path_chart_data) {
    speed::ExportPathToChart(/*base_name=*/"freespace", input_path,
                             &output.path_chart);
  }

  if (FLAGS_planner_draw_st_boundary_canvas) {
    speed::DrawStBoundaryOnCanvas(
        /*base_name=*/"freespace", st_boundaries_with_decision,
        vehicle_geometry_params, *input.obj_mgr, input_path);
  }

  output.trajectory_points = std::move(output_trajectory_points);
  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, *input.obj_mgr,
                             /*processed_st_objects=*/{});

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(st_graph);

  return output;
}

}  // namespace planner
}  // namespace qcraft
