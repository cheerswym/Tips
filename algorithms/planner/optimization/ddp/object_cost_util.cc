#include "onboard/planner/optimization/ddp/object_cost_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/aggregate_static_object_cost.h"
#include "onboard/planner/optimization/problem/mfob_emeraude_object_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/partitioned_object_cost.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/trajectory_plot_util.h"

DEFINE_bool(traj_opt_draw_object_canvas, false,
            "If send object cost reults to canvas.");

namespace qcraft {
namespace planner {
namespace optimizer {
namespace {

// Increment in Z-axis to draw trajectory canvas.
constexpr double kTrajVisZInc =
    kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;

std::vector<SpacetimeObjectState> SampleObjectStates(
    absl::Span<const SpacetimeObjectState> states) {
  // TODO(zhuang): When modify DDP horizon to 15s, this check should be
  // removed.
  std::vector<SpacetimeObjectState> sampled_states;
  sampled_states.reserve(states.size() / kSampleStep);
  for (int i = 0; i < Mfob::kHorizon; ++i) {
    if (i * kSampleStep >= states.size()) break;
    sampled_states.push_back(states[i * kSampleStep]);
  }
  return sampled_states;
}

bool AddPartitionObjectCost(
    std::string_view base_name, double nudge_buffer,
    const std::vector<SpacetimeObjectState> &states,
    const std::vector<TrajectoryPoint> &init_traj,
    const DrivePassage &drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::vector<std::unique_ptr<AvModelHelper<Mfob>>> &av_model_helpers,
    absl::string_view traj_id, double gain,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  using ObjectCost = PartitionedObjectCost<Mfob>;

  constexpr double kSafeBuffer = 0.5;
  const int num_points = states.size();

  const std::vector<double> cascade_gains = {
      cost_weight_params.object_cost_params().object_b_cost_weight(),
      cost_weight_params.object_cost_params().object_a_cost_weight()};

  for (int idx = 0;
       idx < trajectory_optimizer_vehicle_model_params.circle_size(); ++idx) {
    const auto &circle = trajectory_optimizer_vehicle_model_params.circle(idx);
    const std::vector<double> buffers = {circle.radius() + kSafeBuffer,
                                         circle.radius() + nudge_buffer};
    std::vector<ObjectCost::Object> objects;
    // const double l = circle.dist_to_rac();
    objects.reserve(num_points);
    for (int k = 0; k < num_points; ++k) {
      const Vec2d x = init_traj[k].pos();
      const Vec2d tangent = Vec2d::FastUnitFromAngle(init_traj[k].theta() +
                                                     circle.angle_to_axis());
      const Vec2d x_center = x + tangent * circle.dist_to_rac();
      const auto &traj_point = *states[k].traj_point;
      const Vec2d obj_x = traj_point.pos();
      const Polygon2d &contour = states[k].contour;
      std::vector<Segment2d> lines;
      Vec2d ref_x;
      Vec2d ref_tangent;
      double offset = 0.0;
      CalcPartitionHalfContourInfo(
          x_center, obj_x, contour,
          *std::max_element(buffers.begin(), buffers.end()), &lines, &ref_x,
          &ref_tangent, &offset);
      QCHECK(!lines.empty());
      objects.push_back(ObjectCost::Object{.lines = lines,
                                           .buffers = buffers,
                                           .gains = {1.0, 1.0},
                                           .ref_x = ref_x,
                                           .offset = offset,
                                           .ref_tangent = ref_tangent,
                                           .enable = true});

      if (FLAGS_traj_opt_draw_object_canvas) {
        vis::Canvas *canvas_line = nullptr;
        vis::Canvas *canvas_contour = nullptr;

        canvas_line = &vantage_client_man::GetCanvas(
            absl::StrFormat("%s/object/%s/partition/line_%03f/%03d", base_name,
                            traj_id, circle.dist_to_rac(), k));
        canvas_contour = &vantage_client_man::GetCanvas(
            absl::StrFormat("%s/object/%s/partition/contour_%03f/%03d",
                            base_name, traj_id, circle.dist_to_rac(), k));
        const double z = k * kTrajVisZInc;
        // Draw rac
        for (int i = 0; i < lines.size(); i++) {
          const auto &line = lines[i];
          QCHECK_NOTNULL(canvas_line)
              ->DrawLine({line.start().x(), line.start().y(), z},
                         {line.end().x(), line.end().y(), z},
                         vis::Color(0.7, 0.2, 0.2));
        }
        canvas_line->SetGroundZero(1);
        QCHECK_NOTNULL(canvas_contour)
            ->DrawPolygon(contour, z, vis::Color(0.7, 0.2, 0.2));
        canvas_contour->SetGroundZero(1);
      }
    }
    costs->emplace_back(std::make_unique<ObjectCost>(
        std::move(objects), circle.dist_to_rac(), circle.angle_to_axis(),
        cascade_gains, av_model_helpers[idx].get(),
        /*sub_names=*/std::vector<std::string>({"Inner", "Outer"}),
        absl::StrFormat("Partition Object (%s): for %s", circle.name(),
                        traj_id),
        gain * cost_weight_params.object_cost_weight(),
        /*enable_fast_math=*/true));
  }
  return true;
}

absl::Status CalcEmeraudeHalfContourInfoForLeadingObject(
    const Polygon2d &contour, double follow_buffer, double penetration_offset,
    const DrivePassage &drive_passage, const PathSlBoundary &path_sl_boundary,
    const VehicleGeometryParamsProto &veh_geo_params,
    std::vector<Segment2d> *lines, std::vector<double> *buffers, Vec2d *ref_x,
    Vec2d *ref_tangent, double *back_offset, double *front_offset) {
  const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
  if (!frenet_box_or.ok()) {
    return absl::NotFoundError(frenet_box_or.status().message());
  }
  const auto &frenet_box = *frenet_box_or;
  const auto [right_boundary_point, left_boundary_point] =
      path_sl_boundary.QueryBoundaryXY(frenet_box.s_min);
  Segment2d mid_line(left_boundary_point, right_boundary_point);
  const Vec2d backward_vec = mid_line.unit_direction().Perp();
  Segment2d begin_line(left_boundary_point + backward_vec, left_boundary_point);
  Segment2d end_line(right_boundary_point, right_boundary_point + backward_vec);
  const double buffer_radius = 0.5 * veh_geo_params.width();
  lines->assign({begin_line, mid_line, end_line});
  buffers->assign(
      {0.0, follow_buffer + buffer_radius - penetration_offset, 0.0});
  *ref_x = mid_line.center();
  const auto lane_tangent_or =
      drive_passage.QueryLaterallyUnboundedTangentAt(*ref_x);
  if (!lane_tangent_or.ok()) {
    return absl::NotFoundError(lane_tangent_or.status().message());
  }
  const auto &lane_tangent = *lane_tangent_or;
  *ref_tangent = lane_tangent;
  *front_offset = std::numeric_limits<double>::infinity();
  *back_offset = -(follow_buffer + buffer_radius + mid_line.length() * 0.5);
  return absl::OkStatus();
}

absl::Status CalcEmeraudePenetrationOffsetForLeadingObject(
    const Vec2d &x_fac, const Polygon2d &contour, double follow_buffer,
    const DrivePassage &drive_passage, const PathSlBoundary &path_sl_boundary,
    const VehicleGeometryParamsProto &veh_geo_params,
    double *penetration_offset) {
  const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
  if (!frenet_box_or.ok()) {
    return absl::NotFoundError(frenet_box_or.status().message());
  }
  const auto &frenet_box = *frenet_box_or;
  const auto [right_boundary_point, left_boundary_point] =
      path_sl_boundary.QueryBoundaryXY(frenet_box.s_min);
  Segment2d mid_line(left_boundary_point, right_boundary_point);

  const double project_on_mid_line = mid_line.ProjectOntoUnit(x_fac);
  if (project_on_mid_line < 0.0 || project_on_mid_line > mid_line.length()) {
    *penetration_offset = 0.0;
  } else {
    const double buffer_radius = 0.5 * veh_geo_params.width();
    constexpr double kPenetrationOffsetOffset = 1.0;
    *penetration_offset =
        std::max(follow_buffer + mid_line.ProductOntoUnit(x_fac) +
                     buffer_radius - kPenetrationOffsetOffset,
                 0.0);
  }
  return absl::OkStatus();
}

bool AddEmeraudeObjectCostForLeadingObject(
    std::string_view base_name, const std::vector<SpacetimeObjectState> &states,
    const SecondOrderTrajectoryPoint &object_pose,
    const Polygon2d &object_contour,
    const std::vector<TrajectoryPoint> &init_traj,
    const DrivePassage &drive_passage, const PathSlBoundary &path_boundary,
    const ConstraintManager &constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params, absl::string_view traj_id,
    double gain, std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  using ObjectCost = MfobEmeraudeObjectCost<Mfob>;

  const double follow_buffer = cost_weight_params.acc_standstill_standoff();

  std::vector<ObjectCost::Object> objects_f, objects_r;
  const int num_points = states.size();
  objects_f.reserve(num_points);
  objects_r.reserve(num_points);

  const auto draw_lines = [](const std::vector<Segment2d> &lines, double z,
                             const vis::Color &color, vis::Canvas *canvas) {
    QCHECK_NOTNULL(canvas);
    for (int i = 0; i < lines.size(); ++i) {
      const auto &line = lines[i];
      canvas->DrawLine({line.start().x(), line.start().y(), z},
                       {line.end().x(), line.end().y(), z}, color);
    }
  };

  const auto draw_buffers = [](const std::vector<Segment2d> &lines,
                               const std::vector<double> &buffers, double z,
                               const vis::Color &color, vis::Canvas *canvas) {
    QCHECK_NOTNULL(canvas);
    std::vector<Vec2d> buffer_points;
    buffer_points.reserve(lines.size() * 2);
    for (int i = 0; i < lines.size(); ++i) {
      const Vec2d offset = -lines[i].unit_direction().Perp() * buffers[i];
      buffer_points.push_back(lines[i].start() + offset);
      buffer_points.push_back(lines[i].end() + offset);
    }
    for (int i = 0; i < buffer_points.size() - 1; ++i) {
      canvas->DrawLine({buffer_points[i].x(), buffer_points[i].y(), z},
                       {buffer_points[i + 1].x(), buffer_points[i + 1].y(), z},
                       color);
    }
  };

  double penetration_offset = 0.0;
  double t_range = kSpacetimePlannerTrajectoryHorizon;
  const Vec2d x = init_traj.front().pos();
  const Vec2d tangent = Vec2d::FastUnitFromAngle(init_traj.front().theta());
  const Vec2d x_fac = x + tangent * veh_geo_params.wheel_base();
  const Vec2d obj_x = object_pose.pos();

  const auto calc_info_status = CalcEmeraudePenetrationOffsetForLeadingObject(
      x_fac, object_contour, follow_buffer, drive_passage, path_boundary,
      veh_geo_params, &penetration_offset);
  if (!calc_info_status.ok()) return false;
  const auto &leading_object_proto =
      FindOrNull(constraint_manager.LeadingObjects(), traj_id);
  if (leading_object_proto != nullptr &&
      !leading_object_proto->st_constraints().empty()) {
    const int st_constraints_length =
        leading_object_proto->st_constraints_size();
    t_range =
        leading_object_proto->st_constraints(st_constraints_length - 1).t();
  }

  for (int k = 0; k < states.size(); ++k) {
    const auto &traj_point = *states[k].traj_point;
    if (traj_point.t() > t_range) {
      break;
    }
    const Polygon2d &contour = states[k].contour;
    std::vector<Segment2d> lines, fac_lines;
    std::vector<double> buffers, fac_buffers;
    Vec2d ref_x, fac_ref;
    double back_offset, left_fac_offset, front_offset, right_fac_offset;
    Vec2d ref_tangent, fac_ref_tangent;

    // If is leading object, only keep the rear box line.
    const auto calc_info_status = CalcEmeraudeHalfContourInfoForLeadingObject(
        contour, follow_buffer, penetration_offset, drive_passage,
        path_boundary, veh_geo_params, &lines, &buffers, &ref_x, &ref_tangent,
        &back_offset, &front_offset);
    if (!calc_info_status.ok()) break;
    fac_lines.assign(lines.begin(), lines.end());
    fac_buffers.assign(buffers.begin(), buffers.end());
    left_fac_offset = back_offset;
    right_fac_offset = front_offset;

    QCHECK(!lines.empty());
    QCHECK(!fac_lines.empty());

    std::vector<std::vector<double>> buffer_final, fac_buffer_final;
    for (int i = 0; i < lines.size(); ++i) {
      buffer_final.emplace_back().push_back(buffers[i]);
    }
    for (int i = 0; i < fac_buffers.size(); ++i) {
      fac_buffer_final.emplace_back().push_back(fac_buffers[i]);
    }

    objects_r.push_back(
        ObjectCost::Object{.lines = lines,
                           .buffers = buffer_final,
                           .gains = {1.0},
                           .ref_x = ref_x,
                           .ref_tangent = ref_tangent,
                           .back_offset = back_offset,
                           .front_offset = front_offset,
                           .type = ObjectCost::Object::Type::kRac,
                           .enable = true});
    objects_f.push_back(
        ObjectCost::Object{.lines = fac_lines,
                           .buffers = fac_buffer_final,
                           .gains = {1.0},
                           .ref_x = fac_ref,
                           .ref_tangent = fac_ref_tangent,
                           .back_offset = left_fac_offset,
                           .front_offset = right_fac_offset,
                           .type = ObjectCost::Object::Type::kFac,
                           .enable = true});

    if (FLAGS_traj_opt_draw_object_canvas) {
      vis::Canvas *canvas_line = nullptr;
      vis::Canvas *canvas_fac_line = nullptr;
      vis::Canvas *canvas_contour = nullptr;
      vis::Canvas *canvas_buffer = nullptr;
      vis::Canvas *canvas_fac_buffer = nullptr;

      canvas_line = &vantage_client_man::GetCanvas(absl::StrFormat(
          "%s/object/%s/emeraude/line/%03d", base_name, traj_id, k));
      canvas_fac_line = &vantage_client_man::GetCanvas(absl::StrFormat(
          "%s/object/%s/emeraude/fac_line/%03d", base_name, traj_id, k));
      canvas_buffer = &vantage_client_man::GetCanvas(absl::StrFormat(
          "%s/object/%s/emeraude/buffer/%03d", base_name, traj_id, k));
      canvas_fac_buffer = &vantage_client_man::GetCanvas(absl::StrFormat(
          "%s/object/%s/emeraude/fac_buffer/%03d", base_name, traj_id, k));
      canvas_contour = &vantage_client_man::GetCanvas(absl::StrFormat(
          "%s/object/%s/emeraude/contour/%03d", base_name, traj_id, k));

      const double z = k * kTrajVisZInc;
      // Draw rac
      draw_lines(lines, z, vis::Color(0.7, 0.7, 0.2), canvas_line);
      // Draw fac
      draw_lines(fac_lines, z, vis::Color(0.7, 0.7, 0.2), canvas_fac_line);
      // Draw rac buffers.
      draw_buffers(lines, buffers, z, vis::Color(0.2, 0.5, 0.8), canvas_buffer);
      // Draw fac buffers.
      draw_buffers(lines, buffers, z, vis::Color(0.2, 0.5, 0.8),
                   canvas_fac_buffer);
      QCHECK_NOTNULL(canvas_contour)
          ->DrawPolygon(contour, z, vis::Color(0.7, 0.2, 0.2));
    }
  }
  const std::vector<double> cascade_buffers = {
      cost_weight_params.object_cost_params().object_a_cost_weight()};

  costs->emplace_back(std::make_unique<ObjectCost>(
      &veh_geo_params, std::move(objects_r), cascade_buffers,
      /*sub_names=*/std::vector<std::string>({"Soft"}),
      absl::StrFormat("Emeraude Object (R): for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*enable_fast_math=*/true));
  costs->emplace_back(std::make_unique<ObjectCost>(
      &veh_geo_params, std::move(objects_f), cascade_buffers,
      /*sub_names=*/std::vector<std::string>({"Soft"}),
      absl::StrFormat("Emeraude Object (F): for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*enable_fast_math=*/true));

  return true;
}

std::optional<std::pair<std::string, double>> GetClosestLeadingObjectInfo(
    const DrivePassage &drive_passage,
    const ConstraintManager &constraint_manager,
    absl::Span<const SpacetimeObjectTrajectory *const> spacetime_trajs,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params) {
  // End s can't be larger than max leading object s.
  // first: traj_id, second: closest_s.
  std::optional<std::pair<std::string, double>> closest_leading_object_info =
      std::nullopt;
  for (const auto &traj_ptr : spacetime_trajs) {
    if (!traj_ptr->is_stationary()) continue;
    const bool is_leading =
        constraint_manager.IsLeadingObject(traj_ptr->traj_id());
    if (is_leading) {
      const Polygon2d &contour = traj_ptr->contour();
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        continue;
      }
      if (!closest_leading_object_info.has_value()) {
        closest_leading_object_info =
            std::make_pair(traj_ptr->traj_id(),
                           frenet_box_or->s_min -
                               cost_weight_params.acc_standstill_standoff());
      } else if (double leading_s =
                     frenet_box_or->s_min -
                     cost_weight_params.acc_standstill_standoff();
                 leading_s < closest_leading_object_info->second) {
        closest_leading_object_info =
            std::make_pair(traj_ptr->traj_id(), leading_s);
      }
    }
  }
  return closest_leading_object_info;
}

bool IgnoreObjectCost(
    std::string_view base_name, const SpacetimeObjectTrajectory &traj,
    const std::vector<SpacetimeObjectState> &sampled_states,
    const DrivePassage &drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const std::pair<std::string, double> &closest_leading_object_info) {
  // Don't ignore closest leading object itself.
  if (traj.traj_id() == closest_leading_object_info.first) {
    return false;
  }
  // Ignore spacetime trajectories whose entire trajectory is behind the closest
  // leading object.
  const double ignore_s = closest_leading_object_info.second;
  if (traj.is_stationary()) {
    const auto frenet_box_or =
        drive_passage.QueryFrenetBoxAtContour(traj.contour());
    if (!frenet_box_or.ok()) {
      VLOG(2) << base_name << " ignores stationary st-trajectory "
              << traj.traj_id()
              << " because we can't query its contour on drive passage.";
      return true;
    }
    if ((frenet_box_or->s_min - cost_weight_params.acc_standstill_standoff()) <
        ignore_s) {
      return false;
    }
  } else {
    for (int k = 0; k < sampled_states.size(); ++k) {
      const Polygon2d &contour = sampled_states[k].contour;
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        continue;
      }
      if ((frenet_box_or->s_min -
           cost_weight_params.acc_standstill_standoff()) < ignore_s) {
        return false;
      }
    }
  }
  VLOG(2) << base_name << " ignores st-trajectory " << traj.traj_id()
          << " because its entire trajectory is behind closest leading object "
          << closest_leading_object_info.first << " with closest s "
          << closest_leading_object_info.second;
  return true;
}

bool AddAggregateStaticObjectCost(
    std::string_view base_name, const TrajectoryPoint &plan_start_point,
    absl::Span<const SpacetimeObjectTrajectory *const> spacetime_trajs,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs) {
  std::vector<Segment2d> segments;
  constexpr int kEstimateLineCountsPerObject = 6;
  segments.reserve(kEstimateLineCountsPerObject * spacetime_trajs.size());
  for (int idx = 0; idx < spacetime_trajs.size(); ++idx) {
    const auto &traj = *spacetime_trajs[idx];
    const auto &object_segments = traj.contour().line_segments();
    segments.insert(segments.end(), object_segments.begin(),
                    object_segments.end());
  }

  if (segments.empty()) {
    return true;
  }

  constexpr double kSafeBuffer = 0.5;
  const double vehicle_half_width = veh_geo_params.width() * 0.5;
  //  Copy from close_object_slowdown_decider, please modify at the same time.
  const std::vector<double> station_inside_sl_boundary_static_max_speed = {
      3.0, 5.0, 10.0, 20.0, 30.0};  // m/s
  const std::vector<double> close_object_distance_distance = {0.0, 0.5, 1.0,
                                                              1.5, 2.0};  // m
  const PiecewiseLinearFunction<double> nudge_buffer_speed_plf(
      station_inside_sl_boundary_static_max_speed,
      close_object_distance_distance);
  const double nudge_buffer_a =
      std::max(kSafeBuffer, nudge_buffer_speed_plf(plan_start_point.v())) +
      vehicle_half_width;
  const double nudge_buffer_b = kSafeBuffer + vehicle_half_width;
  const std::vector<double> ls = {0.0, veh_geo_params.wheel_base() * 0.5,
                                  veh_geo_params.wheel_base()};
  const std::vector<std::string> names = {"R", "M", "F"};
  const std::vector<double> buffers = {nudge_buffer_a, nudge_buffer_b};
  const std::vector<double> gains = {
      cost_weight_params.object_cost_params().object_a_cost_weight(),
      cost_weight_params.object_cost_params().object_b_cost_weight()};
  const std::vector<std::string> sub_names = {"a", "b"};
  for (int idx = 0; idx < ls.size(); ++idx) {
    costs->push_back(std::make_unique<AggregateStaticObjectCost<Mfob>>(
        segments, ls[idx], buffers, gains, sub_names, kFreeIndex,
        absl::StrFormat("Aggregate Static Object (%s)", names[idx]),
        cost_weight_params.object_cost_weight(), /*enable_fast_math=*/true));
  }
  return true;
}

void DecayInnerPathBoundaryGains(const DrivePassage &drive_passage,
                                 bool is_stationary,
                                 const TrajectoryPoint &plan_start_point,
                                 const SecondOrderTrajectoryPoint &object_pose,
                                 const Polygon2d &object_contour,
                                 std::vector<double> *path_boundary_gains,
                                 char *is_gain_update) {
  if (is_stationary) {
    *is_gain_update = true;
    const auto obj_min_dist_dp_station_pt_index =
        drive_passage.FindNearestStationIndex(object_pose.pos());
    const auto &obj_min_dist_dp_station_pt =
        drive_passage.station(obj_min_dist_dp_station_pt_index);
    const Vec2d &min_dist_dp_station_pt_theta_tangent =
        obj_min_dist_dp_station_pt.tangent();
    const double min_dist_pt_s = obj_min_dist_dp_station_pt.accumulated_s();

    Vec2d front, back;
    object_contour.ExtremePoints(min_dist_dp_station_pt_theta_tangent, &front,
                                 &back);
    const double contour_length =
        (front - back).dot(min_dist_dp_station_pt_theta_tangent);

    constexpr double kGainSRangeBase = 5.0;
    constexpr double kGainSRangeCoeff = 0.5;
    const double gain_s = kGainSRangeBase +
                          kGainSRangeCoeff * Sqr(plan_start_point.v()) +
                          contour_length * 0.5;
    const double s_max = min_dist_pt_s + gain_s;
    const double s_min = min_dist_pt_s - gain_s;

    constexpr double kMinGain = 0.01;

    const auto &station_points = drive_passage.stations();
    const auto obj_min_dist_dp_station_pt_itr =
        station_points.begin() + obj_min_dist_dp_station_pt_index.value();
    auto station_itr =
        obj_min_dist_dp_station_pt_index.value() == (station_points.size() - 1)
            ? --station_points.end()
            : obj_min_dist_dp_station_pt_itr;
    auto gain_itr = path_boundary_gains->begin() +
                    std::distance(station_points.begin(), station_itr);

    while (gain_itr < path_boundary_gains->end() &&
           station_itr->accumulated_s() < s_max) {
      const double factor = (s_max - station_itr->accumulated_s()) / gain_s;
      *gain_itr = std::min(*gain_itr, std::pow(kMinGain, factor));
      ++station_itr;
      ++gain_itr;
    }

    station_itr =
        obj_min_dist_dp_station_pt_index.value() == (station_points.size() - 1)
            ? --station_points.end()
            : obj_min_dist_dp_station_pt_itr;
    gain_itr = path_boundary_gains->begin() +
               std::distance(station_points.begin(), station_itr);
    while (station_itr != station_points.begin() &&
           station_itr->accumulated_s() > s_min) {
      const double factor = (station_itr->accumulated_s() - s_min) / gain_s;
      *gain_itr = std::min(*gain_itr, std::pow(kMinGain, factor));
      --station_itr;
      --gain_itr;
    }
  }
}

double GenerateNudgeBuffer(
    bool is_static, const SecondOrderTrajectoryPoint &object_pose,
    const Polygon2d &object_contour, const TrajectoryPoint &plan_start_point,
    const DrivePassage &drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params) {
  double nudge_buffer = 1.0;
  const PiecewiseLinearFunction<double> nudge_buffer_av_speed_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.object_cost_params()
              .nudge_front_buffer_object_speed_plf());
  const PiecewiseLinearFunction<double>
      nudge_buffer_gain_object_speed_diff_plf =
          PiecewiseLinearFunctionFromProto(
              cost_weight_params.object_cost_params()
                  .nudge_buffer_gain_object_speed_diff_plf());

  if (is_static) {
    nudge_buffer = 0.5;
  } else {
    const auto object_frenet_box_output =
        drive_passage.QueryFrenetBoxAtContour(object_contour);
    if (!object_frenet_box_output.ok()) return false;
    const auto &object_frenet_box = object_frenet_box_output.value();
    const double object_s = object_frenet_box.center().s;
    const auto object_lane_boundary_info =
        drive_passage.QueryEnclosingLaneBoundariesAtS(object_s);
    constexpr double kMinNudgeBuffer = 0.5;
    constexpr double kNudgeBufferLaneWidthGain = 1.5;
    if (object_frenet_box.l_max < 0.0) {
      // If object on the right side of stations.
      if (object_lane_boundary_info.first.has_value()) {
        const bool lane_boundary_have_broken_line =
            object_lane_boundary_info.first->type ==
                StationBoundaryType::BROKEN_YELLOW ||
            object_lane_boundary_info.first->type ==
                StationBoundaryType::BROKEN_WHITE;
        if (!lane_boundary_have_broken_line) {
          nudge_buffer =
              std::clamp((-object_lane_boundary_info.first->lat_offset -
                          veh_geo_params.width() * 0.5) *
                             kNudgeBufferLaneWidthGain,
                         kMinNudgeBuffer, nudge_buffer);
        }
      }
    } else if (object_frenet_box.l_min > 0.0) {
      // If object on the left side of stations.
      if (object_lane_boundary_info.second.has_value()) {
        const bool lane_boundary_have_broken_line =
            object_lane_boundary_info.second->type ==
                StationBoundaryType::BROKEN_YELLOW ||
            object_lane_boundary_info.second->type ==
                StationBoundaryType::BROKEN_WHITE;
        if (!lane_boundary_have_broken_line) {
          nudge_buffer =
              std::clamp((object_lane_boundary_info.second->lat_offset -
                          veh_geo_params.width() * 0.5) *
                             kNudgeBufferLaneWidthGain,
                         kMinNudgeBuffer, nudge_buffer);
        }
      }
    }
    nudge_buffer =
        nudge_buffer * nudge_buffer_av_speed_plf(plan_start_point.v());
    const Vec2d av_local_dir =
        Vec2d::FastUnitFromAngle(plan_start_point.theta());
    const double obj_v_av_local =
        object_pose.v() *
        Vec2d::FastUnitFromAngle(object_pose.theta()).dot(av_local_dir);
    const double speed_diff = obj_v_av_local - plan_start_point.v();
    nudge_buffer *= nudge_buffer_gain_object_speed_diff_plf(speed_diff);
  }
  return nudge_buffer;
}

}  // namespace

void AddObjectCosts(
    std::string_view base_name, const std::vector<TrajectoryPoint> &init_traj,
    const DrivePassage &drive_passage, const PathSlBoundary &path_boundary,
    const ConstraintManager &constraint_manager,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::vector<std::unique_ptr<AvModelHelper<Mfob>>> &av_model_helpers,
    std::vector<double> *inner_path_boundary_gains,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs, ThreadPool *thread_pool) {
  QCHECK_GE(init_traj.size(), Mfob::kHorizon);
  // Use a map-reduce strategy to parallelize the obstacle cost collection.
  const auto spacetime_trajs = st_traj_mgr.spacetime_planner_trajs();
  const int num_trajs = spacetime_trajs.size();

  // Group st_trajs with different object cost types.
  // Trajs added to aggregate object cost.
  std::vector<const SpacetimeObjectTrajectory *> static_spacetime_trajs;
  // Trajs used to generate partition and emeraude object costs.
  std::vector<const SpacetimeObjectTrajectory *> generic_spacetime_trajs;

  static_spacetime_trajs.reserve(num_trajs);
  generic_spacetime_trajs.reserve(num_trajs);
  for (const auto &traj : spacetime_trajs) {
    if (IsStaticObjectType(traj.object_type()) &&
        !(constraint_manager.IsLeadingObject(traj.traj_id()))) {
      static_spacetime_trajs.push_back(&traj);
    } else {
      generic_spacetime_trajs.push_back(&traj);
    }
  }

  // Add aggregate static object cost first.
  AddAggregateStaticObjectCost(base_name, init_traj.front(),
                               static_spacetime_trajs, cost_weight_params,
                               veh_geo_params, costs);

  // Next, add partition and emeraude object costs.
  const int num_generic_spacetime_trajs = generic_spacetime_trajs.size();
  std::vector<std::vector<double>> inner_path_boundary_gains_all_trajs(
      num_generic_spacetime_trajs, *inner_path_boundary_gains);
  std::vector<std::vector<std::unique_ptr<Cost<Mfob>>>> costs_all_trajs(
      num_generic_spacetime_trajs);
  std::vector<char> is_gains_update(num_generic_spacetime_trajs, false);

  // Get Clostest leading object min_s, we will use the value to filter
  // objects whose prediction traj point s all larger than min_s.
  const auto closest_leading_object_info =
      GetClosestLeadingObjectInfo(drive_passage, constraint_manager,
                                  generic_spacetime_trajs, cost_weight_params);

  // Don't apply parallelism in debugging mode as canvas updating is not
  // thread safe.
  ThreadPool *used_tp =
      FLAGS_traj_opt_draw_object_canvas ? nullptr : thread_pool;
  ParallelFor(0, num_generic_spacetime_trajs, used_tp, [&](int i) {
    const auto &traj = *(generic_spacetime_trajs[i]);
    const auto states = SampleObjectStates(traj.states());
    if (!closest_leading_object_info.has_value() ||
        !IgnoreObjectCost(base_name, traj, states, drive_passage,
                          cost_weight_params, *closest_leading_object_info)) {
      const bool is_static = IsStaticObjectType(traj.object_type());
      const bool is_leading_object =
          constraint_manager.IsLeadingObject(traj.traj_id());
      if (is_leading_object) {
        AddEmeraudeObjectCostForLeadingObject(
            base_name, states, traj.pose(), traj.contour(), init_traj,
            drive_passage, path_boundary, constraint_manager,
            cost_weight_params, veh_geo_params, traj.traj_id(),
            traj.trajectory()->probability(), &costs_all_trajs[i]);
      }

      const double nudge_buffer = GenerateNudgeBuffer(
          is_static, traj.pose(), traj.contour(), init_traj.front(),
          drive_passage, cost_weight_params, veh_geo_params);
      AddPartitionObjectCost(
          base_name, nudge_buffer, states, init_traj, drive_passage,
          cost_weight_params, trajectory_optimizer_vehicle_model_params,
          av_model_helpers, traj.traj_id(), traj.trajectory()->probability(),
          &costs_all_trajs[i]);

      DecayInnerPathBoundaryGains(
          drive_passage, traj.is_stationary(), init_traj.front(), traj.pose(),
          traj.contour(), &inner_path_boundary_gains_all_trajs[i],
          &is_gains_update[i]);
    }
  });

  // Collect results from each trajectory.
  for (int idx = 0; idx < num_generic_spacetime_trajs; ++idx) {
    const auto &gains_per_traj = inner_path_boundary_gains_all_trajs[idx];
    if (is_gains_update[idx]) {
      for (int i = 0; i < gains_per_traj.size(); ++i) {
        (*inner_path_boundary_gains)[i] =
            std::min((*inner_path_boundary_gains)[i], gains_per_traj[i]);
      }
    }
  }
  for (auto &costs_per_traj : costs_all_trajs) {
    std::move(costs_per_traj.begin(), costs_per_traj.end(),
              std::back_inserter(*costs));
  }
}

void CalcPartitionHalfContourInfo(const Vec2d &x, const Vec2d &obj_x,
                                  const Polygon2d &contour, double buffer,
                                  std::vector<Segment2d> *lines, Vec2d *ref_x,
                                  Vec2d *ref_tangent, double *offset) {
  const Vec2d force_dir = (x - obj_x).normalized();
  const Vec2d force_right = -force_dir.Perp();

  Vec2d left, right, front, back;
  int left_index, right_index, front_index, back_index;
  contour.ExtremePoints(force_dir, &back_index, &front_index, &back, &front);
  contour.ExtremePoints(force_right, &left_index, &right_index, &left, &right);

  const auto &contour_lines = contour.line_segments();

  // Insert right border
  constexpr double kBorderExtent = 2.0;
  const Segment2d &right_line = contour_lines[right_index];
  lines->emplace_back(right_line.start() - force_dir * kBorderExtent,
                      right_line.start());
  if (front_index >= right_index && front_index <= left_index) {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.begin() + left_index);
  } else {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.end());
    if (left_index != 0) {
      lines->insert(lines->end(), contour_lines.begin(),
                    contour_lines.begin() + left_index);
    }
  }
  // Insert left border
  const Segment2d &left_line =
      contour_lines[left_index == 0 ? (contour_lines.size() - 1)
                                    : (left_index - 1)];
  lines->emplace_back(left_line.end(),
                      left_line.end() - force_dir * kBorderExtent);

  // Fill filter variables
  *ref_x = Vec2d((left.x() + right.x()) * 0.5, (front.y() + back.y()) * 0.5);
  *ref_tangent = force_dir;
  *offset = (front - (*ref_x)).dot(force_dir) + buffer;
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
