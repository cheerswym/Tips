#include "onboard/planner/freespace/tob_path_smoother.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/trace.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_debug_hook.h"
#include "onboard/planner/optimization/ddp/object_cost_util.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/problem/backward_speed_cost.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/partitioned_object_cost.h"
#include "onboard/planner/optimization/problem/partitioned_static_boundary_cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/planner/optimization/problem/tob_curvature_rate_cost.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/status_macros.h"

DEFINE_int32(freespace_local_smoother_ddp_max_iters, 50,
             "Max iterations of freespace local smoother ddp optimizer");
DEFINE_int32(freespace_local_smoother_verbosity_level, 0,
             "Freespace local smoother verbosity level");
DEFINE_bool(send_freespace_local_smoother_path_to_canvas, false,
            "whether send freespace local smoother path to canvas");
DEFINE_bool(
    draw_boundary_cost_buffer_circle, false,
    "whether send buffer circle of partitioned static boundary cost to canvas");

namespace qcraft {
namespace planner {
namespace {

constexpr int kTobSteps = 50;
constexpr double kTobDt = 0.2;
using Tob = ThirdOrderBicycle<kTobSteps>;
using ObjectCost = PartitionedObjectCost<Tob>;

std::vector<TrajectoryPoint> GetInitTrajectoryByPurePursuit(
    const TrajectoryPoint &plan_start_point,
    const BruteForceFrenetFrame &reference_line,
    const VehicleGeometryParamsProto &vehicle_geo_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params, bool forward,
    double max_speed) {
  const auto current_sl = reference_line.XYToSL(plan_start_point.pos());
  const double s_step = (reference_line.end_s() - current_sl.s) /
                        static_cast<double>(Tob::kHorizon);
  const double target_v = std::min(max_speed, s_step / kTobDt);

  constexpr double kLateralLookAhead = 0.4;  // s.
  Tob::ControlsType us = Tob::ControlsType::Zero();
  Tob::StatesType xs = Tob::StatesType::Zero();
  Tob::StateType x;
  // Init state.
  Tob::StateSetX(plan_start_point.pos().x(), &x);
  Tob::StateSetY(plan_start_point.pos().y(), &x);
  Tob::StateSetTheta(forward ? plan_start_point.theta()
                             : NormalizeAngle(plan_start_point.theta() + M_PI),
                     &x);
  Tob::StateSetKappa(
      forward ? plan_start_point.kappa() : -plan_start_point.kappa(), &x);
  Tob::StateSetV(target_v, &x);
  Tob::StateSetA(0.0, &x);
  Tob::StateSetS(0.0, &x);
  constexpr double kMaxKappaRelaxFactor = 2.0;
  const double vehicle_max_kappa =
      kMaxKappaRelaxFactor *
      GetCenterMaxCurvature(vehicle_geo_params, vehicle_drive_params);
  // Pure pursuit process.
  for (int k = 0; k < Tob::kHorizon; ++k) {
    Tob::SetStateAtStep(x, k, &xs);
    const double lateral_look_ahead_dist =
        kLateralLookAhead * target_v + vehicle_geo_params.wheel_base();
    const Vec2d lateral_target = reference_line.SLToXY(
        {Tob::StateGetS(x) + lateral_look_ahead_dist + current_sl.s, 0.0});
    const double theta = Tob::StateGetTheta(x);
    const double alpha =
        Vec2d(lateral_target - Tob::StateGetPos(x)).Angle() - theta;
    const double kappa =
        std::clamp(2.0 * fast_math::Sin(alpha) / lateral_look_ahead_dist,
                   -vehicle_max_kappa, vehicle_max_kappa);
    const double psi = (kappa - Tob::StateGetKappa(x)) / kTobDt;

    const auto u = Tob::MakeControl(psi, /*jerk = */ 0.0);
    x = Tob::EvaluateF(k, x, u, kTobDt);
    Tob::SetControlAtStep(u, k, &us);
  }
  // Extract result.
  std::vector<TrajectoryPoint> res(Tob::kHorizon);
  for (int k = 0; k < Tob::kHorizon; ++k) {
    Tob::ExtractTrajectoryPoint(k, Tob::GetStateAtStep(xs, k),
                                Tob::GetControlAtStep(us, k), kTobDt,
                                &(res[k]));
  }
  return res;
}

std::vector<TrajectoryPoint> GetInitTrajectoryByPrevPath(
    const TrajectoryPoint &plan_start_point,
    const std::vector<PathPoint> &prev_path, bool forward) {
  std::vector<TrajectoryPoint> res;
  res.reserve(Tob::kHorizon);

  DiscretizedPath path(prev_path);
  const auto current_sl = path.XYToSL(plan_start_point.pos());
  const double s_step = (prev_path.back().s() - current_sl.s) / Tob::kHorizon;
  const double target_v = s_step / kTobDt;

  for (int i = 0; i < Tob::kHorizon; ++i) {
    const auto pt = path.Evaluate(current_sl.s + s_step * i);
    TrajectoryPoint traj_pt;
    traj_pt.set_pos(Vec2d(pt.x(), pt.y()));
    traj_pt.set_theta(pt.theta());
    traj_pt.set_kappa(pt.kappa());
    traj_pt.set_s(s_step * i);
    traj_pt.set_v(target_v);
    traj_pt.set_a(0.0);
    traj_pt.set_j(0.0);
    traj_pt.set_psi(0.0);
    traj_pt.set_t(i * kTobDt);
    res.push_back(std::move(traj_pt));
  }
  // BANDAID(zhuang): This is a hack, replace the first point with plan start
  // point.
  res.front().set_pos(plan_start_point.pos());
  res.front().set_theta(forward
                            ? plan_start_point.theta()
                            : NormalizeAngle(plan_start_point.theta() + M_PI));
  res.front().set_kappa(forward ? plan_start_point.kappa()
                                : -plan_start_point.kappa());
  return res;
}

void AddObjectCosts(
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const FreespaceLocalSmootherParamsProto &local_smoother_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const FreespaceMap &static_map,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_object_ids,
    const DirectionalPath &input_path,
    const std::vector<TrajectoryPoint> &init_traj, Tob *problem) {
  QCHECK_EQ(init_traj.size(), Tob::kHorizon);
  constexpr double kGain = 1.0;
  constexpr double kNearbyBoundaryBuffer = 3.0;

  std::vector<Box2d> traj_boxes_with_buffer;
  traj_boxes_with_buffer.reserve(input_path.path.size());
  for (const auto &path_point : input_path.path) {
    const Vec2d av_pos(path_point.x(), path_point.y());
    const Box2d av_box_with_buffer =
        GetAvBoxWithBuffer(av_pos, path_point.theta(), veh_geo_params,
                           kNearbyBoundaryBuffer, kNearbyBoundaryBuffer);
    traj_boxes_with_buffer.push_back(std::move(av_box_with_buffer));
  }

  const auto is_nearby_objects =
      [&traj_boxes_with_buffer](const Polygon2d &object) -> bool {
    for (const auto &av_box : traj_boxes_with_buffer) {
      if (object.HasOverlap(av_box)) return true;
    }
    return false;
  };

  const double buffer = local_smoother_params.object_buffer();
  for (const auto traj_ptr : st_traj_mgr.trajectories()) {
    if (!stalled_object_ids.contains(traj_ptr->object_id())) {
      continue;
    }
    const Polygon2d contour = traj_ptr->contour();
    if (!is_nearby_objects(contour)) continue;
    const Vec2d obj_x = traj_ptr->pose().pos();
    const auto obj_id = std::string(traj_ptr->object_id());
    for (int i = 0; i < trajectory_optimizer_vehicle_model_params.circle_size();
         ++i) {
      const auto &param = trajectory_optimizer_vehicle_model_params.circle(i);

      std::vector<ObjectCost::Object> objects_model;
      objects_model.reserve(Tob::kHorizon);
      for (int k = 0; k < Tob::kHorizon; ++k) {
        const Vec2d x_model = init_traj[k].pos() +
                              Vec2d::FastUnitFromAngle(init_traj[k].theta() +
                                                       param.angle_to_axis()) *
                                  param.dist_to_rac();
        std::vector<Segment2d> model_lines;
        Vec2d model_ref_x, model_ref_tangent;
        double model_offset = 0.0;
        optimizer::CalcPartitionHalfContourInfo(
            x_model, obj_x, contour, param.radius() + buffer, &model_lines,
            &model_ref_x, &model_ref_tangent, &model_offset);
        QCHECK(!model_lines.empty());
        objects_model.push_back(
            ObjectCost::Object{.lines = model_lines,
                               .buffers = {param.radius() + buffer},
                               .gains = {kGain},
                               .ref_x = model_ref_x,
                               .offset = model_offset,
                               .ref_tangent = model_ref_tangent,
                               .enable = true});
      }
      problem->AddCost(std::make_unique<ObjectCost>(
          std::move(objects_model), param.dist_to_rac(), param.angle_to_axis(),
          /*cascade_buffers=*/std::vector<double>({1.0}),
          /*av_model_helper=*/nullptr,
          /*sub_names=*/std::vector<std::string>({""}),
          absl::StrFormat("Partition Object (%s) of %s", param.name(), obj_id),
          local_smoother_params.object_cost_weight(),
          /*enable_fast_math=*/true));
    }
  }
}

void AddStaticBoundaryCost(
    const VehicleGeometryParamsProto &veh_geo_params,
    const FreespaceLocalSmootherParamsProto &local_smoother_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const FreespaceMap &static_map, const DirectionalPath &input_path,
    Tob *problem, FreespaceLocalSmootherDebugProto *debug_info) {
  // Filter boundary that is far from path.
  absl::flat_hash_set<std::string> nearby_boundaries;
  absl::flat_hash_map<std::string, FreespaceBoundary> boundaries_map;
  for (const auto &boundary : static_map.boundaries) {
    boundaries_map.emplace(boundary.id, boundary);
  }
  // Construct k-D tree.
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  named_segments.reserve(static_map.boundaries.size());
  for (const auto &boundary : static_map.boundaries) {
    named_segments.push_back(std::make_pair(boundary.id, boundary.segment));
  }
  SegmentMatcherKdtree segments_kd_tree(std::move(named_segments));
  // Get nearby boundaries.
  constexpr double kNearbyBoundaryBuffer = 3.0;
  for (const auto &path_point : input_path.path) {
    const Vec2d av_pos(path_point.x(), path_point.y());
    const Vec2d av_dir(fast_math::Cos(path_point.theta()),
                       fast_math::Sin(path_point.theta()));
    const Vec2d av_geo_center = GetAvGeometryCenter(
        std::move(av_pos), std::move(av_dir), veh_geo_params);
    const Box2d av_box_with_buffer =
        GetAvBoxWithBuffer(av_pos, path_point.theta(), veh_geo_params,
                           kNearbyBoundaryBuffer, kNearbyBoundaryBuffer);
    const auto nearby_named_segments =
        segments_kd_tree.GetNamedSegmentsInRadius(
            av_geo_center.x(), av_geo_center.y(), veh_geo_params.length());
    for (const auto &named_segment : nearby_named_segments) {
      const auto iter = nearby_boundaries.find(named_segment.second);
      if (iter == nearby_boundaries.end() &&
          av_box_with_buffer.HasOverlap(*named_segment.first)) {
        nearby_boundaries.emplace(named_segment.second);
      }
    }
  }

  // Add costs.
  constexpr double kGain = 1.0;
  double buffer = 0.0;
  double weight = 1.0;
  const std::vector<double> cascade_gains = {kGain};
  for (auto iter = nearby_boundaries.begin(); iter != nearby_boundaries.end();
       iter++) {
    const auto boundary_iter = boundaries_map.find(*iter);
    QCHECK(boundary_iter != boundaries_map.end());
    const auto &boundary = boundary_iter->second;
    switch (boundary.type) {
      case FreespaceMapProto::CURB:
        buffer = local_smoother_params.curb_buffer();
        weight = local_smoother_params.curb_cost_weight();
        break;
      case FreespaceMapProto::BARRIER:
        buffer = local_smoother_params.barrier_buffer();
        weight = local_smoother_params.barrier_cost_weight();
        break;
      case FreespaceMapProto::YELLOW_SOLID_LANE:
      case FreespaceMapProto::WHITE_SOLID_LANE:
        buffer = local_smoother_params.solid_lane_buffer();
        weight = local_smoother_params.solid_lane_cost_weight();
        break;
      case FreespaceMapProto::PARKING_SPOT:
        buffer = local_smoother_params.spot_line_buffer();
        weight = local_smoother_params.spot_line_cost_weight();
        break;
      case FreespaceMapProto::VIRTUAL:
        buffer = local_smoother_params.virtual_boundary_buffer();
        weight = local_smoother_params.virtual_boundary_cost_weight();
        break;
      default:
        break;
    }

    for (int i = 0; i < trajectory_optimizer_vehicle_model_params.circle_size();
         ++i) {
      const auto &param = trajectory_optimizer_vehicle_model_params.circle(i);
      problem->AddCost(std::make_unique<PartitionedStaticBoundaryCost<Tob>>(
          &veh_geo_params, boundary.segment,
          PartitionedStaticBoundaryCost<Tob>::Type::kCor, param.dist_to_rac(),
          param.angle_to_axis(), /*sub_names=*/std::vector<std::string>({""}),
          std::vector<double>({param.radius() + buffer}), cascade_gains,
          absl::StrFormat("PartitionStaticBoundaryCost (%s) of %s",
                          param.name(), boundary.id),
          weight, /*enable_fast_math=*/true));
    }
  }

  // Add costs of dashed lane lines. We only consider lane line that has no
  // intersection with path.
  const auto is_nearby = [&input_path](const Segment2d &boundary) -> bool {
    constexpr double kNearbyBoundaryDistSqr = 5.0 * 5.0;
    for (const auto &pt : input_path.path) {
      if (boundary.DistanceSquareTo(Vec2d(pt.x(), pt.y())) <
          kNearbyBoundaryDistSqr) {
        return true;
      }
    }
    return false;
  };

  std::vector<Segment2d> seg_path;
  seg_path.reserve(input_path.path.size() - 1);
  for (int i = 0; i + 1 < input_path.path.size(); ++i) {
    seg_path.emplace_back(
        Vec2d(input_path.path[i].x(), input_path.path[i].y()),
        Vec2d(input_path.path[i + 1].x(), input_path.path[i + 1].y()));
  }
  const auto has_overlap = [&seg_path](const Segment2d &boundary) -> bool {
    const double min_x = boundary.min_x();
    const double min_y = boundary.min_y();
    const double max_x = boundary.max_x();
    const double max_y = boundary.max_y();
    for (const auto &seg : seg_path) {
      if (min_x > seg.max_x() || min_y > seg.max_y() || max_x < seg.min_x() ||
          max_y < seg.min_y()) {
        continue;
      }
      if (boundary.HasIntersect(seg)) return true;
    }
    return false;
  };

  for (const auto &boundary : static_map.special_boundaries) {
    if (boundary.type != SpecialBoundaryType::CROSSABLE_LANE_LINE) {
      continue;
    }
    if (is_nearby(boundary.segment) && !has_overlap(boundary.segment)) {
      for (int i = 0;
           i < trajectory_optimizer_vehicle_model_params.circle_size(); ++i) {
        const auto &param = trajectory_optimizer_vehicle_model_params.circle(i);
        problem->AddCost(std::make_unique<PartitionedStaticBoundaryCost<Tob>>(
            &veh_geo_params, boundary.segment,
            PartitionedStaticBoundaryCost<Tob>::Type::kCor, param.dist_to_rac(),
            param.angle_to_axis(), /*sub_names=*/std::vector<std::string>({""}),
            std::vector<double>(
                {param.radius() +
                 local_smoother_params.crossable_lane_buffer()}),
            cascade_gains,
            absl::StrFormat("PartitionStaticBoundaryCost (%s) of %s",
                            param.name(), boundary.id),
            local_smoother_params.crossable_lane_cost_weight(),
            /*enable_fast_math=*/true));
      }
      // Add to debug info here.
      auto seg_proto = debug_info->add_enabled_crossable_boundaries();
      seg_proto->set_id(boundary.id);
      Vec2dToProto(boundary.segment.start(), seg_proto->mutable_start());
      Vec2dToProto(boundary.segment.end(), seg_proto->mutable_end());
    }
  }
}

void ToDebugProto(double end_xy_error, double end_theta_error,
                  const std::vector<TrajectoryPoint> &init_traj,
                  const std::vector<TrajectoryPoint> &res_traj,
                  const OptimizerSolverDebugHook<Tob> &debug_hook,
                  FreespaceLocalSmootherDebugProto *debug_info) {
  QCHECK_NOTNULL(debug_info);
  debug_info->mutable_end_pose_error()->set_xy(end_xy_error);
  debug_info->mutable_end_pose_error()->set_theta(end_theta_error);
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(debug_info->add_init_traj());
  }
  for (int k = 0; k < res_traj.size(); ++k) {
    res_traj[k].ToProto(debug_info->add_res_traj());
  }
  const auto &init_costs = debug_hook.init_costs;
  debug_info->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        debug_info->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.ddp_costs[i].first);
    cost_proto->set_cost(init_costs.ddp_costs[i].second);
  }
  const auto &final_costs = debug_hook.final_costs;
  debug_info->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        debug_info->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.ddp_costs[i].first);
    cost_proto->set_cost(final_costs.ddp_costs[i].second);
  }
  for (const auto &iteration : debug_hook.iterations) {
    TrajectoryOptimizerCostInfo *iters_proto = debug_info->add_iter_costs();
    iters_proto->set_cost(iteration.final_cost);
    for (int i = 0; i < iteration.ddp_costs.size(); ++i) {
      const auto iters_cost_proto = iters_proto->add_costs();
      iters_cost_proto->set_name(iteration.ddp_costs[i].first);
      iters_cost_proto->set_cost(iteration.ddp_costs[i].second);
    }
  }
}

}  // namespace

absl::StatusOr<DirectionalPath> SmoothLocalPath(
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const FreespaceLocalSmootherParamsProto &local_smoother_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::string &owner, const FreespaceMap &static_map,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_object_ids,
    const DirectionalPath &input_path, const TrajectoryPoint &plan_start_point,
    bool reset, const std::vector<PathPoint> &prev_path,
    FreespaceLocalSmootherDebugProto *debug_info,
    vis::vantage::ChartsDataProto *charts_data) {
  SCOPED_QTRACE("SmoothLocalPath");

  constexpr double kScale = 1.0;
  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  constexpr double kCurvatureBufferGain = 0.75;
  constexpr double kCurvatureRateBufferGain = 0.9;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kSpeedDirectionGain = 10.0;
  constexpr double kMaxSpeed = 2.0;

  // Create reference line.
  std::vector<Vec2d> reference_line;
  constexpr double kPathStep = 0.1;  // 0.1m
  double s = input_path.path.front().s();
  while (s <= input_path.path.back().s()) {
    const auto pt = input_path.path.Evaluate(s);
    reference_line.emplace_back(Vec2d(pt.x(), pt.y()));
    s += kPathStep;
  }
  QCHECK_GT(reference_line.size(), 2);
  std::vector<double> ref_line_deviation_gains(
      reference_line.size() - 1,
      local_smoother_params.ref_path_deviation_cost_weight());

  ASSIGN_OR_RETURN(const auto frenet_ref_line,
                   BuildBruteForceFrenetFrame(reference_line));
  // Create init trajectory.
  std::vector<TrajectoryPoint> init_traj;
  if (reset || prev_path.empty()) {
    init_traj = GetInitTrajectoryByPurePursuit(
        plan_start_point, frenet_ref_line, veh_geo_params, veh_drive_params,
        motion_constraint_params, input_path.forward, kMaxSpeed);
    debug_info->set_init_traj_type(
        FreespaceLocalSmootherDebugProto::PURE_PURSUIT);
  } else {
    // TODO(zhuang): Maybe just repalce the fisrt point is enough.
    init_traj = GetInitTrajectoryByPrevPath(plan_start_point, prev_path,
                                            input_path.forward);
    debug_info->set_init_traj_type(FreespaceLocalSmootherDebugProto::PREV_PATH);
  }
  if (!input_path.forward) {
    for (auto &pt : init_traj) {
      pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
      pt.set_kappa(-pt.kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_s(-pt.s());
      pt.set_j(-pt.j());
      pt.set_psi(-pt.psi());
    }
  }
  if (FLAGS_send_freespace_local_smoother_path_to_canvas) {
    vis::Canvas &canvas =
        vantage_client_man::GetCanvas("freespace/local_smooth_path/init_traj");
    for (const auto &pt : init_traj) {
      canvas.DrawPoint(Vec3d(pt.pos(), 0.0), vis::Color::kLightGray, 2);
    }
  }

  // Create reference states and control for regularization.
  constexpr double kStateDeviationGain = 1.0e-5;
  constexpr double kControlDeviationGain = 1.0e-5;
  const auto ref_xs = Tob::FitState(init_traj);
  const auto ref_us =
      Tob::FitControl(init_traj, Tob::GetStateAtStep(ref_xs, 0));

  // Create end attraction.
  Tob::StatesType end_attraction_xs = Tob::StatesType::Zero();
  const auto current_sl = frenet_ref_line.XYToSL(plan_start_point.pos());
  const double s_step = (frenet_ref_line.end_s() - current_sl.s) /
                        static_cast<double>(Tob::kHorizon);
  const double target_v = s_step / kTobDt;
  // If target_v is very large, the end point should not be the end of
  // reference line.
  PathPoint end_point;
  if (target_v > kMaxSpeed) {
    const double end_s = current_sl.s + kMaxSpeed * kTobDt * Tob::kHorizon;
    end_point = input_path.path.Evaluate(end_s + input_path.path.front().s());
  } else {
    end_point = input_path.path.back();
  }
  Tob::set_x(end_point.x(), Tob::kHorizon - 1, &end_attraction_xs);
  Tob::set_y(end_point.y(), Tob::kHorizon - 1, &end_attraction_xs);
  if (input_path.forward) {
    Tob::set_theta(end_point.theta(), Tob::kHorizon - 1, &end_attraction_xs);
  } else {
    Tob::set_theta(NormalizeAngle(end_point.theta() + M_PI), Tob::kHorizon - 1,
                   &end_attraction_xs);
  }
  std::vector<double> end_attraction_weights(end_attraction_xs.size(), 0.0);
  end_attraction_weights[(Tob::kHorizon - 1) * Tob::kStateSize +
                         Tob::kStateXIndex] =
      local_smoother_params.end_pose_deviation_cost_weight();
  end_attraction_weights[(Tob::kHorizon - 1) * Tob::kStateSize +
                         Tob::kStateYIndex] =
      local_smoother_params.end_pose_deviation_cost_weight();
  end_attraction_weights[(Tob::kHorizon - 1) * Tob::kStateSize +
                         Tob::kStateThetaIndex] =
      local_smoother_params.end_theta_deviation_cost_weight();

  Tob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
              kTobDt,
              /*enable_post_process =*/false);

  // TODO(zhuang): Maybe need cascade buffer for ref_line.
  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Tob>>(
      kPathGain, kEndStateGain, reference_line, /*center_line_helper=*/nullptr,
      std::move(ref_line_deviation_gains), "ReferenceLineDeviationCost",
      kScale));
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Tob>>(
      ref_xs,
      std::vector<double>(Tob::kHorizon * Tob::kStateSize, kStateDeviationGain),
      "ReferenceStateDeviationCost", kScale));
  problem.AddCost(std::make_unique<ReferenceControlDeviationCost<Tob>>(
      ref_us,
      std::vector<double>(Tob::kHorizon * Tob::kControlSize,
                          kControlDeviationGain),
      "ReferenceControlDeviationCost", kScale));
  problem.AddCost(std::make_unique<CurvatureCost<Tob>>(
      kCurvatureBufferGain *
          GetCenterMaxCurvature(veh_geo_params, veh_drive_params),
      Tob::kHorizon, "CurvatureCost",
      local_smoother_params.curvature_cost_weight()));
  if (input_path.forward) {
    problem.AddCost(std::make_unique<ForwardSpeedCost<Tob>>(
        "ForwardSpeedCost", kSpeedDirectionGain));
  } else {
    problem.AddCost(std::make_unique<BackwardSpeedCost<Tob>>(
        "BackwardSpeedCost", kSpeedDirectionGain));
  }
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Tob>>(
      end_attraction_xs, std::move(end_attraction_weights), "EndAttractionCost",
      kScale));
  problem.AddCost(std::make_unique<TobCurvatureRateCost<Tob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferGain,
      "CurvatureRateCost", local_smoother_params.curvature_rate_cost_weight()));
  problem.AddCost(std::make_unique<IntrinsicJerkCost<Tob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "IntrinsicJerkCost", local_smoother_params.intrinsic_jerk_cost_weight()));
  AddObjectCosts(veh_geo_params, veh_drive_params, motion_constraint_params,
                 local_smoother_params,
                 trajectory_optimizer_vehicle_model_params, static_map,
                 st_traj_mgr, stalled_object_ids, input_path, init_traj,
                 &problem);
  AddStaticBoundaryCost(veh_geo_params, local_smoother_params,
                        trajectory_optimizer_vehicle_model_params, static_map,
                        input_path, &problem, debug_info);

  DdpOptimizer<Tob> smoother(
      &problem, owner,
      /*verbosity=*/FLAGS_freespace_local_smoother_verbosity_level,
      local_smoother_params.optimizer_params());
  OptimizerSolverDebugHook<Tob> debug_hook(init_traj.front(), st_traj_mgr);
  smoother.AddHook(&debug_hook);
  smoother.SetInitialPoints(init_traj);

  const auto solve_start_time = absl::Now();
  const auto output = smoother.Solve(
      input_path.forward, /*enable_first_iteration_postprocess=*/true);
  const absl::Duration time_consuming = absl::Now() - solve_start_time;

  VLOG(2) << "Freespace local smoother time(ms): "
          << absl::ToDoubleMilliseconds(time_consuming);
  if (!output.ok()) {
    debug_info->set_status(output.status().ToString());
    return absl::InternalError("Local path smoother: DDP failed! " +
                               output.status().ToString());
  }
  const double end_xy_error = Hypot(end_point.x() - output->back().pos().x(),
                                    end_point.y() - output->back().pos().y());
  const double end_theta_error = std::abs(NormalizeAngle(
      (input_path.forward ? end_point.theta() : (end_point.theta() + M_PI)) -
      output->back().theta()));
  VLOG(2) << "Endpoint pose error = " << end_xy_error
          << ", theta error = " << end_theta_error;
  ToDebugProto(end_xy_error, end_theta_error, init_traj, *output, debug_hook,
               debug_info);
  std::vector<TrajectoryPlotInfo> plot_trajs = {
      {.traj = *output, .name = "res", .color = vis::Color::kRed}};
  if (charts_data != nullptr) {
    optimizer::AddTrajCharts("freespace/local_smoother", plot_trajs,
                             charts_data->mutable_charts());
  }
  if (FLAGS_send_freespace_local_smoother_path_to_canvas) {
    vis::Canvas &canvas =
        vantage_client_man::GetCanvas("freespace/local_smooth_path/res_traj");
    for (const auto &pt : *output) {
      canvas.DrawPoint(Vec3d(pt.pos(), 0.0), vis::Color::kWhite, 2);
    }
  }
  if (FLAGS_draw_boundary_cost_buffer_circle) {
    const Vec2d rac = output->front().pos();
    const double heading = output->front().theta();
    const auto get_circle_center = [&rac, &heading](double dist_to_rac,
                                                    double angle_to_axis) {
      const auto tangent = Vec2d::FastUnitFromAngle(heading + angle_to_axis);
      return rac + dist_to_rac * tangent;
    };
    vis::Canvas &canvas = vantage_client_man::GetCanvas(
        "freespace/local_smooth_path/boundary_cost_buffer_circle");
    for (int i = 0; i < trajectory_optimizer_vehicle_model_params.circle_size();
         ++i) {
      const auto &param = trajectory_optimizer_vehicle_model_params.circle(i);
      const auto pos =
          get_circle_center(param.dist_to_rac(), param.angle_to_axis());
      canvas.DrawCircle(Vec3d(pos, 0.0), param.radius(), vis::Color::kWhite, 1);
    }
  }

  // Extract result.
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(output->size());
  for (const auto &traj_pt : *output) {
    PathPoint pt;
    pt.set_x(traj_pt.pos()[0]);
    pt.set_y(traj_pt.pos()[1]);
    pt.set_theta(input_path.forward ? traj_pt.theta()
                                    : NormalizeAngle(traj_pt.theta() + M_PI));
    pt.set_kappa(input_path.forward ? traj_pt.kappa() : -traj_pt.kappa());
    pt.set_s(input_path.forward ? traj_pt.s() : -traj_pt.s());
    raw_path_points.push_back(std::move(pt));
  }
  // Postprocess path s.
  raw_path_points.front().set_s(0.0);
  for (int i = 1; i < raw_path_points.size(); ++i) {
    const double d = DistanceTo(raw_path_points[i], raw_path_points[i - 1]);
    raw_path_points[i].set_s(raw_path_points[i - 1].s() + d);
  }

  DirectionalPath res = {DiscretizedPath(std::move(raw_path_points)),
                         input_path.forward};
  debug_info->set_status("success");
  return res;
}
}  // namespace planner
}  // namespace qcraft
