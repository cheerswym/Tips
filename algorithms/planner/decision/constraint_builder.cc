#include "onboard/planner/decision/constraint_builder.h"

#include <limits>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/decision/beyond_length_along_route.h"
#include "onboard/planner/decision/cautious_brake_decider.h"
#include "onboard/planner/decision/crosswalk_decider.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/end_of_current_lane_path.h"
#include "onboard/planner/decision/end_of_path_boundary.h"
#include "onboard/planner/decision/ignore_object.h"
#include "onboard/planner/decision/lc_end_of_current_lane_constraint.h"
#include "onboard/planner/decision/leading_object.h"
#include "onboard/planner/decision/no_block.h"
#include "onboard/planner/decision/parking_brake_release.h"
#include "onboard/planner/decision/pedestrians_decider.h"
#include "onboard/planner/decision/speed_bump.h"
#include "onboard/planner/decision/standstill_decider.h"
#include "onboard/planner/decision/stop_sign_decider.h"
#include "onboard/planner/decision/toll_decider.h"
#include "onboard/planner/decision/traffic_light_decider.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_params.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
SpeedProfile CreateSpeedProfile(
    double v_now, const DrivePassage &passage,
    const PlannerSemanticMapManager &psmm,
    const absl::Span<const ConstraintProto::SpeedRegionProto> &speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto> &stop_points) {
  const std::vector<double> v_s = CreateSpeedProfileWithConstraints(
      v_now, passage, psmm, speed_zones, stop_points);
  return IntegrateSpeedProfile(passage, v_s);
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildPullOverStopConstraint(
    const DrivePassage &passage, double front_to_ra, double ego_v) {
  constexpr double kBrake = 1.0;           // m/s^2
  constexpr double kStandStillDist = 0.3;  // m.

  // NOTE: We dont want to add more states. Recalculate stop s each frame is
  // acceptable as we do not require stop at a certain point.
  const double stop_dist =
      ego_v < 1.0 ? kStandStillDist : Sqr(ego_v) * 0.5 / kBrake;
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra;

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("pull_over"));
  stop_line.mutable_source()->mutable_pull_over()->set_reason(
      "Teleop triggered pull over");
  return stop_line;
}

absl::StatusOr<ConstraintProto::StopLineProto> BuildBrakeToStopConstraint(
    const DrivePassage &passage, double front_to_ra, double ego_v,
    double brake) {
  constexpr double kStandStillDist = 0.3;  // m.

  // NOTE: We dont want to add more states. Recalculate stop s each frame is
  // acceptable as we do not require stop at a certain point.
  const double stop_dist =
      ego_v < 1.0 ? kStandStillDist : Sqr(ego_v) * 0.5 / brake;
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra;

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("brake_to_stop"));
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(
      "Teleop triggered brake");
  return stop_line;
}

}  // namespace

absl::StatusOr<DeciderOutput> BuildConstraints(
    const DeciderInput &decider_input) {
  QCHECK_NOTNULL(decider_input.vehicle_geometry_params);
  QCHECK_NOTNULL(decider_input.config);
  QCHECK_NOTNULL(decider_input.planner_semantic_map_manager);
  QCHECK_NOTNULL(decider_input.stalled_objects);
  QCHECK_NOTNULL(decider_input.scene_reasoning);
  QCHECK_NOTNULL(decider_input.plan_start_point);
  QCHECK_NOTNULL(decider_input.passage);
  QCHECK_NOTNULL(decider_input.obj_mgr);
  QCHECK_NOTNULL(decider_input.st_traj_mgr);
  QCHECK_NOTNULL(decider_input.pre_decider_state);
  QCHECK_NOTNULL(decider_input.pose);
  QCHECK_NOTNULL(decider_input.av_frenet_box);

  const auto &vehicle_geometry_params = *decider_input.vehicle_geometry_params;
  const auto &config = *decider_input.config;
  const auto &planner_semantic_map_manager =
      *decider_input.planner_semantic_map_manager;
  const auto &stalled_objects = *decider_input.stalled_objects;
  const auto &scene_reasoning = *decider_input.scene_reasoning;
  const auto lc_stage = decider_input.lc_stage;
  const auto &plan_start_point = *decider_input.plan_start_point;
  const auto &target_lane_path_from_current =
      decider_input.passage->lane_path();
  const auto &passage = *decider_input.passage;
  const auto &obj_mgr = *decider_input.obj_mgr;
  const auto &sl_boundary = *decider_input.sl_boundary;
  const auto &st_traj_mgr = *decider_input.st_traj_mgr;
  const auto &tl_info_map = *decider_input.tl_info_map;
  const auto &lc_clearance_check_output =
      *decider_input.lc_clearance_check_output;
  const auto &pre_decider_state = *decider_input.pre_decider_state;
  const auto &av_frenet_box = *decider_input.av_frenet_box;

  SCOPED_QTRACE("EstPlanner/ConstraintBuilder");
  ConstraintManager constraint_manager;
  DeciderStateProto new_decider_state;

  if (target_lane_path_from_current.IsEmpty()) {
    return absl::UnavailableError(
        "Target lane path from current not available.");
  }

  // TODO(PNC-501): This is a hack. Remove this hack after refactor planner
  // params.
  const bool requires_parking_brake_release =
      decider_input.vehicle_model != VehicleModel::VEHICLE_ZHONGXING;
  if (config.enable_parking_brake_release() && requires_parking_brake_release) {
    const auto parking_brake_release_constraint =
        BuildParkingBrakeReleaseConstraint(
            vehicle_geometry_params, passage,
            decider_input.parking_brake_release_time, decider_input.plan_time);
    if (parking_brake_release_constraint.ok()) {
      constraint_manager.AddStopLine(parking_brake_release_constraint.value());
    } else {
      VLOG(2) << "Build parking brake release constraint failed: "
              << parking_brake_release_constraint.status().ToString();
    }
  }

  if (config.enable_lc_end_of_current_lane() &&
      decider_input.lane_path_before_lc != nullptr) {
    const auto &lane_path_before_lc = *decider_input.lane_path_before_lc;
    if (lc_stage == LaneChangeStage::LCS_PAUSE &&
        !lane_path_before_lc.IsEmpty()) {
      auto lcp_speed = BuildLcEndOfCurrentLaneConstraints(
          lane_path_before_lc, passage, plan_start_point.v());
      if (lcp_speed.ok()) {
        constraint_manager.AddSpeedRegion(std::move(lcp_speed).value());
      }
    }
  }

  if (config.enable_beyond_length_along_route()) {
    auto beyond_len_along_route_speed = BuildBeyondLengthAlongRouteConstraint(
        passage, decider_input.length_along_route,
        decider_input.borrow_lane_boundary);
    if (beyond_len_along_route_speed.ok()) {
      constraint_manager.AddSpeedRegion(
          std::move(beyond_len_along_route_speed).value());
    }
  }

  if (config.enable_crosswalk()) {
    // Crosswalk.
    ASSIGN_OR_RETURN(const auto cw_decider_output,
                     BuildCrosswalkConstraints(
                         vehicle_geometry_params, planner_semantic_map_manager,
                         plan_start_point, passage, obj_mgr,
                         ToUnixDoubleSeconds(decider_input.plan_time),
                         pre_decider_state.crosswalk_state()));

    for (auto &cw_stop_line : cw_decider_output.stop_lines) {
      constraint_manager.AddStopLine(std::move(cw_stop_line));
    }
    for (auto &cw_speed_region : cw_decider_output.speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(cw_speed_region));
    }
    for (auto &crosswalk_state : cw_decider_output.crosswalk_states) {
      *new_decider_state.add_crosswalk_state() = std::move(crosswalk_state);
    }
  }

  // Pedestrians.
  if (config.enable_pedestrians()) {
    // lane keep
    if (lc_stage == LaneChangeStage::LCS_NONE) {
      ASSIGN_OR_RETURN(const auto ped_speed_regions,
                       BuildPedestriansConstraints(vehicle_geometry_params,
                                                   planner_semantic_map_manager,
                                                   plan_start_point, passage,
                                                   sl_boundary, st_traj_mgr));

      for (const auto &ped_speed_region : ped_speed_regions) {
        constraint_manager.AddSpeedRegion(std::move(ped_speed_region));
      }
    }
  }

  bool enable_lc_multi_traj = false;
  if (config.enable_leading_object()) {
    enable_lc_multi_traj = EnableInitializerLaneChangeTargetDecision(
                               lc_stage, sl_boundary, av_frenet_box) &&
                           FLAGS_planner_initializer_multiple_traj;
    // Leading object.
    if (enable_lc_multi_traj == false) {
      auto leading_objects = FindLeadingObjects(
          vehicle_geometry_params, planner_semantic_map_manager,
          stalled_objects, scene_reasoning, passage, sl_boundary,
          lc_clearance_check_output, st_traj_mgr, plan_start_point,
          av_frenet_box, decider_input.borrow_lane_boundary);
      for (auto &leading_object : leading_objects) {
        constraint_manager.AddLeadingObject(std::move(leading_object));
      }
    }
  }

  if (config.enable_ignore_object()) {
    // Ignore object.
    auto ignore_objects = FindObjectsToIgnore(vehicle_geometry_params, passage,
                                              st_traj_mgr, *decider_input.pose);
    for (auto &ignore_object : ignore_objects) {
      constraint_manager.AddIgnoreObject(std::move(ignore_object));
    }
  }

  // No block constraints.
  if (config.enable_no_block()) {
    auto no_block_regions =
        BuildNoBlockConstraints(planner_semantic_map_manager, passage);
    for (auto &no_block_region : no_block_regions) {
      QCHECK_LE(no_block_region.start_s(), no_block_region.end_s())
          << no_block_region.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(no_block_region));
    }
  }
  if (config.enable_stop_sign()) {
    ASSIGN_OR_RETURN(
        auto output,
        BuildStopSignConstraints(
            planner_semantic_map_manager, st_traj_mgr, vehicle_geometry_params,
            passage,
            /*now_in_seconds=*/ToUnixDoubleSeconds(decider_input.plan_time),
            plan_start_point, pre_decider_state.stop_sign_state()));
    for (auto &stop_line : output.stop_lines) {
      constraint_manager.AddStopLine(std::move(stop_line));
    }
    for (auto &stop_sign_state : output.stop_sign_states) {
      *new_decider_state.add_stop_sign_state() = std::move(stop_sign_state);
    }
  }

  // If we have end of current lane path constraint, add it.
  const auto end_of_cur_lp_constraint =
      BuildEndOfCurrentLanePathConstraint(passage);
  if (end_of_cur_lp_constraint.ok()) {
    constraint_manager.AddStopLine(end_of_cur_lp_constraint.value());
  } else {
    // If we don't have end of current route constraint, add the end of drive
    // passage constraint.
    const auto end_of_path_boundary_constraint =
        BuildEndOfPathBoundaryConstraint(passage, sl_boundary);
    if (end_of_path_boundary_constraint.ok()) {
      constraint_manager.AddStopLine(end_of_path_boundary_constraint.value());
    } else {
      QLOG(WARNING) << "Build end of path boundary constraint failed: "
                    << end_of_path_boundary_constraint.status().ToString();
    }
  }

  if (config.enable_speed_bump()) {
    // Speed bump.
    auto speed_bumps =
        BuildSpeedBumpConstraints(planner_semantic_map_manager, passage);
    for (auto &speed_bump : speed_bumps) {
      QCHECK_LE(speed_bump.start_s(), speed_bump.end_s())
          << speed_bump.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(speed_bump));
    }
  }

  if (config.enable_cautious_brake()) {
    // Speed bump.
    auto cautious_brake_regions = BuildCautiousBrakeConstraints(
        planner_semantic_map_manager, passage, st_traj_mgr);
    for (auto &cautious_brake : cautious_brake_regions) {
      QCHECK_LE(cautious_brake.start_s(), cautious_brake.end_s())
          << cautious_brake.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(cautious_brake));
    }
  }

  if (config.enable_toll()) {
    ASSIGN_OR_RETURN(
        const auto toll_speed_regions,
        BuildTollConstraints(planner_semantic_map_manager, passage));

    for (const auto &toll_speed_region : toll_speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(toll_speed_region));
    }
  }

  if (config.enable_traffic_light() &&
      decider_input.teleop_enable_traffic_light_stop) {
    // Traffic light.
    absl::StatusOr<TrafficLightDeciderOutput> tl_decider_output;

    // SpeedProfile need speed regions and stop line informations from other
    // constraints, build this constraint at last.
    SpeedProfile preliminary_speed_profile = CreateSpeedProfile(
        plan_start_point.v(), passage, planner_semantic_map_manager,
        constraint_manager.SpeedRegion(), constraint_manager.StopLine());

    tl_decider_output = BuildTrafficLightConstraints(
        planner_semantic_map_manager, vehicle_geometry_params, plan_start_point,
        target_lane_path_from_current, passage, tl_info_map,
        preliminary_speed_profile,
        pre_decider_state.traffic_light_decider_state());

    if (!tl_decider_output.ok()) {
      QLOG(WARNING) << "Build tl stop lines failed with message: "
                    << tl_decider_output.status().ToString();
    } else {
      for (const auto &tl_stop_line : tl_decider_output.value().stop_lines) {
        constraint_manager.AddStopLine(tl_stop_line);
      }
      *new_decider_state.mutable_traffic_light_decider_state() =
          tl_decider_output.value().traffic_light_decider_state;
    }
  }

  if (decider_input.enable_pull_over) {
    const auto stop_line_or = BuildPullOverStopConstraint(
        passage, vehicle_geometry_params.front_edge_to_center(),
        plan_start_point.v());

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }

  if (decider_input.brake_to_stop > 0.0) {
    const auto stop_line_or = BuildBrakeToStopConstraint(
        passage, vehicle_geometry_params.front_edge_to_center(),
        plan_start_point.v(), decider_input.brake_to_stop);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }

  if (config.enable_standstill()) {
    ASSIGN_OR_RETURN(
        auto ss_stop_lines,
        BuildStandstillConstraints(vehicle_geometry_params, plan_start_point,
                                   passage, constraint_manager.StopLine()));
    for (auto &ss_stop_line : ss_stop_lines) {
      constraint_manager.AddStopLine(std::move(ss_stop_line));
    }
  }

  return DeciderOutput{
      .constraint_manager = std::move(constraint_manager),
      .decider_state = std::move(new_decider_state),
      .initializer_lc_multiple_traj = enable_lc_multi_traj,
  };
}

}  // namespace planner
}  // namespace qcraft
