#include "onboard/planner/trajectory_validation.h"

#include <cmath>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {
namespace {

bool ValidateTrajectoryControlLimits(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    bool check_intrinsic_jerk, bool check_curvature_rate,
    TrajectoryValidationResultProto *result) {
  constexpr int kCurvatureCheckHorizon = 10;
  const double max_speed =
      Mph2Mps(motion_constraint_params.default_speed_limit());
  const double max_accel = motion_constraint_params.max_acceleration();
  const double max_decel = motion_constraint_params.max_deceleration();
  const double max_curvature = GetRelaxedCenterMaxCurvature(
      vehicle_geometry_params, vehicle_drive_params);
  const double max_accel_jerk = motion_constraint_params.max_accel_jerk();
  const double max_decel_jerk = motion_constraint_params.max_decel_jerk();
  const double max_psi = motion_constraint_params.max_psi();
  constexpr double kRelaxFactor = 1.05;
  for (int i = 0; i < traj_points.size(); ++i) {
    const auto &p = traj_points[i];
    if (p.v() > max_speed * kRelaxFactor) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::SPEED_OVER_LIMIT,
          absl::StrFormat(
              "speed over limit: v[%d] = %f, max_speed=%f, relax_factor=%f", i,
              p.v(), max_speed, kRelaxFactor));
      error.AddToProto(result);

      const auto callback = [i, &p](QEvent *qevent) {
        qevent->AddField("index", i).AddField("speed", p.v());
      };
      QEVENT("renjie", "trajectory_failure_speed_over_limit", callback);
      return false;
    }
    if (p.a() > max_accel * kRelaxFactor) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::ACCELERATION_OVER_LIMIT,
          absl::StrFormat("acceleration over limit: a[%d] = %f, max_accel=%f, "
                          "relax_factor=%f",
                          i, p.a(), max_accel, kRelaxFactor));
      error.AddToProto(result);

      const auto callback = [i, &p](QEvent *qevent) {
        qevent->AddField("index", i).AddField("acceleration", p.a());
      };
      QEVENT("renjie", "trajectory_failure_acceleration_over_limit", callback);
      return false;
    }
    if (p.a() < max_decel * kRelaxFactor) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::DECELERATION_OVER_LIMIT,
          absl::StrFormat("deceleration over limit: a[%d] = %f, max_decel=%f, "
                          "relax_factor=%f",
                          i, p.a(), max_decel, kRelaxFactor));
      error.AddToProto(result);

      const auto callback = [i, &p](QEvent *qevent) {
        qevent->AddField("index", i).AddField("deceleration", p.a());
      };
      QEVENT("renjie", "trajectory_failure_deceleration_over_limit", callback);
      return false;
    }
    if (i < kCurvatureCheckHorizon &&
        std::fabs(p.kappa()) > max_curvature * kRelaxFactor) {
      const auto callback = [i, &p](QEvent *qevent) {
        qevent->AddField("index", i).AddField("curvature", p.kappa());
      };
      QEVENT("renjie", "trajectory_failure_curvature_over_limit", callback);
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::CURVATURE_OVER_LIMIT,
          absl::StrFormat("curvature over limit: kappa[%d] = %f, "
                          "max_curvature=%f, relax_factor=%f",
                          i, p.kappa(), max_curvature, kRelaxFactor));
      error.AddToProto(result);
      return false;
    }

    if (i + 1 == traj_points.size()) continue;

    if (check_intrinsic_jerk) {
      const double j = p.j();
      if (j > max_accel_jerk * kRelaxFactor) {
        const auto callback = [i, j](QEvent *qevent) {
          qevent->AddField("index", i)
              .AddField("intrinsic_acceleration_jerk", j);
        };
        QEVENT("renjie", "trajectory_failure_intrinsic_jerk_over_limit",
               callback);
      }
      if (j < max_decel_jerk * kRelaxFactor) {
        const auto callback = [i, j](QEvent *qevent) {
          qevent->AddField("index", i)
              .AddField("intrinsic_deceleration_jerk", j);
        };
        QEVENT("renjie", "trajectory_failure_intrinsic_jerk_over_limit",
               callback);
      }
    }

    if (check_curvature_rate) {
      const double psi = p.psi();
      if (std::fabs(psi) > max_psi * kRelaxFactor) {
        const auto callback = [i, psi](QEvent *qevent) {
          qevent->AddField("index", i).AddField("psi", psi);
        };
        QEVENT("renjie", "trajectory_failure_psi_over_limit", callback);
      }
    }
  }

  return true;
}

bool ValidateTrajectoryInternalConsistency(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    TrajectoryValidationResultProto *result) {
  const double max_speed =
      Mph2Mps(motion_constraint_params.default_speed_limit());
  const double max_accel = motion_constraint_params.max_acceleration();
  const double max_curvature = GetRelaxedCenterMaxCurvature(
      vehicle_geometry_params, vehicle_drive_params);
  const double max_accel_jerk = motion_constraint_params.max_accel_jerk();
  const double max_psi = motion_constraint_params.max_psi();
  const double fd_speed_error_bound = max_accel * kTrajectoryTimeStep * 2.0;
  const double fd_heading_error_bound =
      max_curvature * max_speed * kTrajectoryTimeStep * 2.0;
  const double fd_accel_error_bound =
      max_accel_jerk * kTrajectoryTimeStep * 2.0 +
      fd_speed_error_bound / kTrajectoryTimeStep * 2.0;
  const double fd_curvature_error_bound =
      max_psi * kTrajectoryTimeStep * 2.0 +
      fd_heading_error_bound / kTrajectoryTimeStep * 2.0;
  for (int i = 0; i + 1 < traj_points.size() && i + 1 < kTrajectorySteps; ++i) {
    const TrajectoryPoint &p_this = traj_points[i];
    const TrajectoryPoint &p_next = traj_points[i + 1];
    const Vec2d vec = p_next.pos() - p_this.pos();
    const double len = vec.norm();
    const double fd_v = len / kTrajectoryTimeStep;
    const double traj_v = (p_this.v() + p_next.v()) * 0.5;
    if (std::fabs(fd_v - traj_v) > fd_speed_error_bound) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::SPEED_INCONSISTENCY,
          absl::StrFormat(
              "speed inconsistency: p_this.pos[%d] = (%f, %f) p_next.pos[%d] = "
              "(%f, %f) FD speed = %f v[%d] = %f v[%d] = %f traj speed = %f "
              "error bound = %f",
              i, p_this.pos().x(), p_this.pos().y(), i + 1, p_next.pos().x(),
              p_next.pos().y(), fd_v, i, p_this.v(), i + 1, p_next.v(), traj_v,
              fd_speed_error_bound));
      error.AddToProto(result);

      const auto callback = [i, fd_v, traj_v,
                             fd_speed_error_bound](QEvent *qevent) {
        qevent->AddField("index", i)
            .AddField("FD_speed", fd_v)
            .AddField("traj_speed", traj_v)
            .AddField("error_bound", fd_speed_error_bound);
      };
      QEVENT("renjie", "trajectory_failure_speed_inconsistency", callback);
      return false;
    }

    if (p_this.pos() == p_next.pos()) continue;
    const double fd_theta = vec.FastAngle();
    const double traj_theta = LerpAngle(p_this.theta(), p_next.theta(), 0.5);
    if (std::fabs(NormalizeAngle(fd_theta - traj_theta)) >
        fd_heading_error_bound) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::HEADING_INCONSISTENCY,
          absl::StrFormat("heading inconsistency: p_this.pos[%d] = (%f, %f) "
                          "p_next.pos[%d] = (%f, %f) FD theta = %f theta[%d] = "
                          "%f theta[%d] = %f traj theta = %f error bound = %f",
                          i, p_this.pos().x(), p_this.pos().y(), i + 1,
                          p_next.pos().x(), p_next.pos().y(), fd_theta, i,
                          p_this.theta(), i + 1, p_next.theta(), traj_theta,
                          fd_heading_error_bound));
      error.AddToProto(result);

      const auto callback = [i, fd_theta, traj_theta,
                             fd_heading_error_bound](QEvent *qevent) {
        qevent->AddField("index", i)
            .AddField("FD_theta", fd_theta)
            .AddField("traj_theta", traj_theta)
            .AddField("error_bound", fd_heading_error_bound);
      };
      QEVENT("renjie", "trajectory_failure_heading_inconsistency", callback);
      return false;
    }

    if (i == 0) continue;
    const TrajectoryPoint &p_prev = traj_points[i - 1];
    const Vec2d vec_prev = p_this.pos() - p_prev.pos();
    const double len_prev = vec_prev.norm();
    const double fd_a = (len - len_prev) / Sqr(kTrajectoryTimeStep);
    const double traj_a = p_this.a();
    if (std::fabs(fd_a - traj_a) > fd_accel_error_bound) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::ACCELERATION_INCONSISTENCY,
          absl::StrFormat(
              "acceleration inconsistency: p_prev.pos[%d] = (%f, %f) "
              "p_next.pos[%d] = (%f, %f) x[%d] = (%f, %f) FD accel = %f a[%d] "
              "= %f error bound = %f",
              i - 1, p_prev.pos().x(), p_prev.pos().y(), i, p_this.pos().x(),
              p_this.pos().y(), i + 1, p_next.pos().x(), p_next.pos().y(), fd_a,
              i, p_this.a(), fd_accel_error_bound));
      error.AddToProto(result);

      const auto callback = [i, &p_this, fd_a,
                             fd_accel_error_bound](QEvent *qevent) {
        qevent->AddField("index", i)
            .AddField("FD_accel", fd_a)
            .AddField("traj_accel", p_this.a())
            .AddField("error_bound", fd_accel_error_bound);
      };
      QEVENT("renjie", "trajectory_failure_acceleration_inconsistency",
             callback);
      return false;
    }

    if (p_prev.pos() == p_this.pos()) continue;
    const double fd_theta_prev = vec_prev.FastAngle();
    const double fd_kappa =
        NormalizeAngle(fd_theta - fd_theta_prev) / ((len + len_prev) * 0.5);
    const double traj_kappa = p_this.kappa();
    if (std::fabs(fd_kappa - traj_kappa) >
        fd_curvature_error_bound / (len + len_prev)) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::CURVATURE_INCONSISTENCY,
          absl::StrFormat(
              "curvature inconsistency: p_prev.pos[%d] = (%f, %f) "
              "p_next.pos[%d] = (%f, %f) x[%d] = (%f, %f) len = %f len_prev = "
              "%f FD kappa = %f kappa[%d] = %f error bound = %f",
              i - 1, p_prev.pos().x(), p_prev.pos().y(), i, p_this.pos().x(),
              p_this.pos().y(), i + 1, p_next.pos().x(), p_next.pos().y(), len,
              len_prev, fd_kappa, i, p_this.kappa(), fd_curvature_error_bound));
      error.AddToProto(result);

      const auto callback = [i, &p_this, fd_kappa,
                             fd_curvature_error_bound](QEvent *qevent) {
        qevent->AddField("index", i)
            .AddField("FD_kappa", fd_kappa)
            .AddField("traj_kappa", p_this.kappa())
            .AddField("error_bound", fd_curvature_error_bound);
      };
      QEVENT("renjie", "trajectory_failure_curvature_inconsistency", callback);
      return false;
    }
  }

  return true;
}

bool ValidateEstTrajectoryCurbCollision(
    const SemanticMapManager &semantic_map_manager,
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    TrajectoryValidationResultProto *result) {
  constexpr double kCurbColCheckLengthHorizon = 50.0;
  constexpr int kCurbColCheckHorizon = 50;
  const double av_diagonal_length =
      Vec2d(vehicle_geometry_params.length(), vehicle_geometry_params.width())
          .norm();
  constexpr double kRadiusBuffer = 0.2;
  const double search_radius = 0.5 * av_diagonal_length + kRadiusBuffer;
  constexpr double kLatExtendBuffer = 0.1;
  for (int i = 0; i < traj_points.size() && i < kCurbColCheckHorizon; ++i) {
    if (traj_points[i].s() > kCurbColCheckLengthHorizon) break;

    // Extend the box laterally to account for lateral control error.
    const Box2d av_box = GetAvBoxWithBuffer(
        traj_points[i].pos(), traj_points[i].theta(), vehicle_geometry_params,
        /*length_buffer=*/0.0, /*width_buffer=*/kLatExtendBuffer);
    const Vec2d av_geo_center = GetAvGeometryCenter(
        traj_points[i].pos(), traj_points[i].theta(), vehicle_geometry_params);
    const auto level_id = semantic_map_manager.GetLevel();
    const auto curb_segments =
        semantic_map_manager.GetImpassableBoundariesAtLevel(
            level_id, av_geo_center, search_radius);
    for (const auto &curb_segment : curb_segments) {
      if (av_box.HasOverlap(curb_segment)) {
        TrajectoryValidationError error(
            TrajectoryValidationResultProto::CURB_COLLISION,
            absl::StrFormat("curb collision: traj point index = %d pose: x = "
                            "(%f, %f) theta "
                            "= %f segment (%f, %f) - (%f, %f)",
                            i, traj_points[i].pos().x(),
                            traj_points[i].pos().y(), traj_points[i].theta(),
                            curb_segment.start().x(), curb_segment.start().y(),
                            curb_segment.end().x(), curb_segment.end().y()));
        error.AddToProto(result);
        return false;
      }
    }
  }

  return true;
}

bool IsCollisionEnd(const Polygon2d &av_box, const Polygon2d &obj_contour,
                    const TrajectoryPoint &pose) {
  Polygon2d overlap_polygon;
  if (!av_box.ComputeOverlap(obj_contour, &overlap_polygon)) {
    return true;
  } else {
    const Vec2d av_tangent = Vec2d::FastUnitFromAngle(pose.theta());
    Vec2d front, back;
    overlap_polygon.ExtremePoints(av_tangent, &back, &front);
    if ((front - pose.pos()).dot(av_tangent) < 0.0) {
      return true;
    }
  }
  return false;
}

struct StObjectCollisionInfo {
  std::string object_id = "";
  double probability = 0.0;
};

void GetStObjectCollisionInfo(
    const PartialSpacetimeObjectTrajectory &st_object_with_time_range,
    const std::vector<TrajectoryPoint> &traj_points,
    const std::vector<Polygon2d> &av_box, double av_length,
    StObjectCollisionInfo *info) {
  const int check_horizon = av_box.size();
  const auto &st_traj = st_object_with_time_range.st_traj();
  info->object_id = st_traj.planner_object()->id();

  double collision_prob = 0.0;
  const int agent_point_num = st_traj.states().size();
  for (int k = 0; k < check_horizon && k < agent_point_num; ++k) {
    const double time = k * kTrajectoryTimeStep;
    if (!st_object_with_time_range.GetDecisionTypeAtTime(time).has_value()) {
      continue;
    }
    const auto &object_contour = st_traj.states()[k].contour;
    if (av_box[k].HasOverlap(object_contour)) {
      // We ignore the collision if the first collision point doesn't satisfy
      // any of the following conditions:
      // 1) it is rear-end;
      // 2) the object contour hits current AV box;
      // 3) AV is fully stopped but object is moving;
      // TODO(huaiyuan): Use a more comprehensive model to check legal
      // collisions considering liabilities.
      constexpr double kAvFullStopSpeedThreshold = 0.05;  // m/s.
      constexpr double kObjMovingSpeedThreshold = 0.5;    // m/s.
      const bool is_av_stop_and_object_moving =
          traj_points[k].v() < kAvFullStopSpeedThreshold &&
          st_traj.states()[k].traj_point->v() > kObjMovingSpeedThreshold;
      if (!is_av_stop_and_object_moving &&
          !IsCollisionEnd(av_box[k], object_contour, traj_points[k]) &&
          !av_box[0].HasOverlap(object_contour)) {
        collision_prob = st_traj.trajectory()->probability();
        VLOG(3) << "Collide with st object " << st_traj.traj_id() << " at step "
                << k << " of probability "
                << st_traj.trajectory()->probability();
      }
      break;
    }
  }
  info->probability = collision_prob;
}

bool ValidateEstTrajectoryObjectCollision(
    const std::vector<PartialSpacetimeObjectTrajectory> &considered_st_objects,
    const std::vector<TrajectoryPoint> &traj_points,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    TrajectoryValidationResultProto *result, ThreadPool *thread_pool) {
  constexpr int kObsColCheckHorizon = 40;
  QCHECK_LE(kObsColCheckHorizon, traj_points.size());

  // Get AV boxes at each time step. Extend the box laterally to account for
  // lateral control error.
  std::vector<Polygon2d> av_box;
  av_box.reserve(kObsColCheckHorizon);
  constexpr double kLatExtendBuffer = 0.1;
  for (int i = 0; i < kObsColCheckHorizon; ++i) {
    const Box2d box =
        GetAvBoxWithBuffer(traj_points[i].pos(), traj_points[i].theta(),
                           vehicle_geometry_params, /*length_buffer=*/0.0,
                           /*width_buffer=*/kLatExtendBuffer);
    av_box.emplace_back(box);
  }

  // Collision info: {string: planner_object_id, double: collision probability}.
  std::vector<StObjectCollisionInfo> collision_info(
      considered_st_objects.size());
  ParallelFor(0, considered_st_objects.size(), thread_pool, [&](int i) {
    GetStObjectCollisionInfo(considered_st_objects[i], traj_points, av_box,
                             vehicle_geometry_params.length(),
                             &collision_info[i]);
  });

  // Collect planner_object_id -> collision probability map. Check the result
  // and record collision result.
  constexpr double kCollisionProbThres = 0.8;
  absl::flat_hash_map<std::string, double> object_collision_prob_map;
  object_collision_prob_map.reserve(collision_info.size());
  for (const auto &info : collision_info) {
    if (object_collision_prob_map.find(info.object_id) ==
        object_collision_prob_map.end()) {
      object_collision_prob_map.emplace(info.object_id, info.probability);
    } else {
      object_collision_prob_map[info.object_id] += info.probability;
    }
    // Early exit if find an object whose cumulative collision probability is
    // beyond the threshold.
    const double cumulative_collision_prob =
        object_collision_prob_map[info.object_id];
    if (cumulative_collision_prob > kCollisionProbThres) {
      result->add_collision_object_id(info.object_id);
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::OBJECT_COLLISION,
          absl::StrFormat(
              "collision with object %s of cumulative probability %f",
              info.object_id, cumulative_collision_prob));
      error.AddToProto(result);
      const auto callback = [&info, cumulative_collision_prob](QEvent *qevent) {
        qevent->AddField("id", info.object_id)
            .AddField("probability", cumulative_collision_prob);
      };
      QEVENT("renjie", "trajectory_failure_object_collision", callback);

      return false;
    }
  }

  return true;
}

void ValidateTrajectoryLateralComfort(
    const std::vector<TrajectoryPoint> &traj_points,
    const MotionConstraintParamsProto &motion_constraint_params) {
  // TODO(renjie): maybe these bounds should be scenario-dependent.
  constexpr int kLateralComfortCheckHorizon = 40;
  QCHECK_LE(kLateralComfortCheckHorizon, traj_points.size());
  for (int i = 0; i < kLateralComfortCheckHorizon; ++i) {
    const auto &traj_point = traj_points[i];

    const double lat_accel = ComputeLateralAcceleration(traj_point);
    const double lat_jerk = ComputeLateralJerk(traj_point);
    if (std::fabs(lat_accel) > motion_constraint_params.max_lateral_accel()) {
      const auto callback = [i, lat_accel](QEvent *qevent) {
        qevent->AddField("index", i).AddField("lat_accel", lat_accel);
      };
      QEVENT("renjie", "trajectory_failure_lat_accel_over_limit", callback);
      return;
    }
    if (std::fabs(lat_jerk) > motion_constraint_params.max_lateral_jerk()) {
      const auto callback = [i, lat_jerk](QEvent *qevent) {
        qevent->AddField("index", i).AddField("lat_jerk", lat_jerk);
      };
      QEVENT("renjie", "trajectory_failure_lat_jerk_over_limit", callback);
      return;
    }
  }
}

bool ValidateDrivePassageAndPathBoundaryViolation(
    const std::vector<TrajectoryPoint> &traj_points,
    const DrivePassage &drive_passage, const PathSlBoundary &path_sl_boundary,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    TrajectoryValidationResultProto *result) {
  constexpr double kPathBoundaryViolationLimit = 1.2;  // meter
  for (int i = 0; i < traj_points.size(); ++i) {
    const auto &pt = traj_points[i];
    const auto frenet_pt_or = drive_passage.QueryFrenetCoordinateAt(pt.pos());
    if (!frenet_pt_or.ok()) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::OUT_OF_DRIVE_PASSAGE_QUERY_AREA,
          absl::StrFormat("QueryFrenetCoordinateAt fail at traj point %d (%f, "
                          "%f), return info: ",
                          i, pt.pos().x(), pt.pos().y()) +
              frenet_pt_or.status().ToString());
      error.AddToProto(result);
      QEVENT("renjie", "trajectory_exceed_drive_passage_query_area",
             ([i](QEvent *qevent) { qevent->AddField("index", i); }));
      return false;
    }
    const auto l_pair = path_sl_boundary.QueryBoundaryL(frenet_pt_or->s);
    const double excced_left_boundary_dist =
        l_pair.second - frenet_pt_or->l - 0.5 * vehicle_geometry_params.width();
    if (excced_left_boundary_dist < -kPathBoundaryViolationLimit) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::OUT_OF_PATH_BOUNDARY,
          absl::StrFormat(
              "Trajectory exceed left path boundary by %f m at traj "
              "point %d (%f, %f)",
              -excced_left_boundary_dist, i, pt.pos().x(), pt.pos().y()));
      error.AddToProto(result);
      QEVENT("renjie", "trajectory_exceed_left_path_boundary",
             ([i, &excced_left_boundary_dist](QEvent *qevent) {
               qevent->AddField("index", i)
                   .AddField("left_exceed_dist", -excced_left_boundary_dist);
             }));
      return false;
    }
    const double excced_right_boundary_dist =
        l_pair.first - frenet_pt_or->l + 0.5 * vehicle_geometry_params.width();
    if (excced_right_boundary_dist > kPathBoundaryViolationLimit) {
      TrajectoryValidationError error(
          TrajectoryValidationResultProto::OUT_OF_PATH_BOUNDARY,
          absl::StrFormat("Trajectory exceed right path boundary by %f m at "
                          "traj point %d (%f, %f)",
                          excced_right_boundary_dist, i, pt.pos().x(),
                          pt.pos().y()));
      error.AddToProto(result);
      QEVENT("renjie", "trajectory_exceed_right_path_boundary",
             ([i, &excced_right_boundary_dist](QEvent *qevent) {
               qevent->AddField("index", i)
                   .AddField("right_exceed_dist", excced_right_boundary_dist);
             }));
      return false;
    }
  }
  return true;
}

}  // namespace
bool ValidatePlanStartPointCollisionFree(
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerObjectManager &obj_mgr, std::string *ptr_obj_id) {
  const auto av_box = Polygon2d(
      GetAvBoxWithBuffer(Vec2d(plan_start_point.path_point().x(),
                               plan_start_point.path_point().y()),
                         plan_start_point.path_point().theta(),
                         vehicle_geometry_params, /*length_buffer=*/0.0,
                         /*width_buffer=*/0.0));
  for (const auto &obj : obj_mgr.planner_objects()) {
    if (av_box.HasOverlap(obj.contour())) {
      *ptr_obj_id = obj.id();
      return false;
    }
  }
  return true;
}

bool ValidateEstTrajectory(
    const SemanticMapManager &semantic_map_manager,
    const std::vector<PartialSpacetimeObjectTrajectory> &considered_st_objects,
    bool full_stop, const PoseProto &pose,
    const SchedulerOutput &scheduler_output,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto *result, ThreadPool *thread_pool) {
  SCOPED_QTRACE("ValidateEstTrajectory");

  std::vector<TrajectoryPoint> traj_points = ToTrajectoryPoint(traj);
  TrajectoryValidationOptionsProto trajectory_validation_options;
  // Don't check curvature rate for Est trajectory until speed finder can handle
  // curvature rate constraints.
  trajectory_validation_options.set_check_curvature_rate(false);
  bool valid = ValidateTrajectory(
      pose, traj_points, trajectory_validation_options, vehicle_geometry_params,
      vehicle_drive_params, motion_constraint_params, result);
  if (trajectory_validation_options.check_curb_collision()) {
    valid &= ValidateEstTrajectoryCurbCollision(
        semantic_map_manager, traj_points, vehicle_geometry_params, result);
  }
  if (!full_stop) {
    valid &= ValidateEstTrajectoryObjectCollision(
        considered_st_objects, traj_points, vehicle_geometry_params, result,
        thread_pool);
  }
  ValidateTrajectoryLateralComfort(traj_points, motion_constraint_params);
  valid &= ValidateDrivePassageAndPathBoundaryViolation(
      traj_points, scheduler_output.drive_passage, scheduler_output.sl_boundary,
      vehicle_geometry_params, result);
  if (!valid) {
    return false;
  }

  return true;
}

bool ValidateEstPrevTrajectory(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto *result) {
  // TODO(renjie): Add more safety checks for previous trajectory validation.
  std::vector<TrajectoryPoint> traj_points;
  traj_points.reserve(traj.size());
  for (const auto &point : traj) {
    traj_points.emplace_back();
    traj_points.back().FromProto(point);
  }

  TrajectoryValidationOptionsProto trajectory_validation_options;
  // Don't check curvature rate for Est trajectory until speed finder can handle
  // curvature rate constraints.
  trajectory_validation_options.set_check_curvature_rate(false);
  bool valid = ValidateTrajectory(
      pose, traj_points, trajectory_validation_options, vehicle_geometry_params,
      vehicle_drive_params, motion_constraint_params, result);
  if (trajectory_validation_options.check_curb_collision()) {
    valid &= ValidateEstTrajectoryCurbCollision(
        semantic_map_manager, traj_points, vehicle_geometry_params, result);
  }
  ValidateTrajectoryLateralComfort(traj_points, motion_constraint_params);
  if (!valid) {
    return false;
  }

  return true;
}

bool ValidateTrajectory(
    const PoseProto &current_pose,
    absl::Span<const TrajectoryPoint> traj_points,
    const TrajectoryValidationOptionsProto &trajectory_validation_options,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    TrajectoryValidationResultProto *result) {
  SCOPED_QTRACE("trajectory_validation");

  // Size check.
  constexpr int kMinTrajectoryLength = 10;  // 1s.
  if (traj_points.size() < kMinTrajectoryLength) {
    TrajectoryValidationError error(
        TrajectoryValidationResultProto::TRAJECTORY_TOO_SHORT,
        absl::StrFormat("trajectory is not long enough: %d time steps.",
                        traj_points.size()));
    error.AddToProto(result);
    QEVENT("renjie", "trajectory_failure_trajectory_too_short",
           [&traj_points](QEvent *qevent) {
             qevent->AddField("trajectory_size",
                              static_cast<int64>(traj_points.size()));
           });
    return false;
  }

  if (trajectory_validation_options.check_control_limits() &&
      !ValidateTrajectoryControlLimits(
          traj_points, vehicle_geometry_params, vehicle_drive_params,
          motion_constraint_params,
          trajectory_validation_options.check_intrinsic_jerk(),
          trajectory_validation_options.check_curvature_rate(), result)) {
    return false;
  }

  if (trajectory_validation_options.check_internal_consistency() &&
      !ValidateTrajectoryInternalConsistency(
          traj_points, vehicle_geometry_params, vehicle_drive_params,
          motion_constraint_params, result)) {
    return false;
  }

  return true;
}

}  // namespace planner
}  // namespace qcraft
