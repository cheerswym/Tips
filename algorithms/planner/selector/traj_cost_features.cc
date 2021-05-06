#include "onboard/planner/selector/traj_cost_features.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/utils/status_macros.h"

#define btoa(x) ((x) ? "true" : "false")
#define btof(x) ((x) ? 1.0 : 0.0)

namespace qcraft::planner {

namespace {

constexpr double kMinStandardProgressDiff = 50.0;   // m.
constexpr double kMinLaneSpeedDiffStandard = 8.0;   // m.
constexpr double kLaneSpeedDiffToLimitRatio = 0.5;  // s.
constexpr double kMinLcSpeedInIntersection = 5.0;   // m/s.
constexpr double kContinuousBoundaryMaxLatOffset = kDefaultHalfLaneWidth;
constexpr double kStandardAverageTrajDiff = 1.6 * kDefaultHalfLaneWidth;
constexpr double kObjDistCutoffDistance = 0.5;         // m.
constexpr double kReachDestinationCutOffDist = 100.0;  // m.
constexpr double kIgnoreObjectsAtRouteEndLen = 10.0;   // m.
constexpr double kTimeToStalledRear = 2.0;             // s.
constexpr double kStalledRearRegionMinLength = kMinLcLaneLength;
constexpr double kSingleLaneSectionStalledRearRegionFactor = 5.0;

using CostVec = CostFeatureBase::CostVec;

using IndexedStationBoundary = std::pair<StationIndex, StationBoundary>;
struct BoundaryInterval {
  std::vector<Vec2d> points;
  StationBoundaryType type;
};

inline double CalcPsi(const ApolloTrajectoryPointProto &prev_pt,
                      const ApolloTrajectoryPointProto &succ_pt) {
  return (succ_pt.path_point().kappa() - prev_pt.path_point().kappa()) /
         (succ_pt.relative_time() - prev_pt.relative_time());
}

inline double CalcLatJerk(const ApolloTrajectoryPointProto &pt, double psi) {
  // j_lat = 3 * v * a * kappa + v^2 * psi
  return std::abs(3.0 * pt.v() * pt.a() * pt.path_point().kappa() +
                  Sqr(pt.v()) * psi);
}

absl::StatusOr<PointOnRouteSections> FindLastTrajPointOnRouteSections(
    const SemanticMapManager &smm, const RouteSectionsInfo &sections_info,
    const DrivePassage &dp, const PlannerTrajectory &traj_pts) {
  const double lp_end_s = dp.lane_path().length() + dp.lane_path_start_s();
  for (int idx = traj_pts.size() - 1; idx >= 0; --idx) {
    const Vec2d pt_pos = Vec2dFromApolloTrajectoryPointProto(traj_pts[idx]);
    const double heading = traj_pts[idx].path_point().theta();
    const auto proj_or = dp.QueryFrenetLonOffsetAt(pt_pos);
    if (!proj_or.ok() || proj_or->accum_s > lp_end_s) continue;

    ASSIGN_OR_CONTINUE(
        const auto lane_pt,
        FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
            smm.GetLevel(), smm, pt_pos, dp.lane_path(), heading));

    const auto &sections = sections_info.section_segments();
    double accum_s = 0.0;
    int sec_idx = 0;
    for (; sec_idx < sections_info.size(); ++sec_idx) {
      if (sections[sec_idx].id_idx_map.contains(lane_pt.lane_id())) {
        break;
      }
      accum_s += sections[sec_idx].length();
    }
    accum_s += sections[sec_idx].average_length *
               (lane_pt.fraction() - sections[sec_idx].start_fraction);
    return PointOnRouteSections{.accum_s = accum_s,
                                .section_idx = sec_idx,
                                .fraction = lane_pt.fraction(),
                                .lane_id = lane_pt.lane_id()};
  }

  return absl::NotFoundError("The whole trajectory is not on route sections.");
}

bool IsInTlControlledIntersection(const SemanticMapManager &smm,
                                  const DrivePassage &drive_passage, double s) {
  const auto &station = drive_passage.FindNearestStationAtS(s);
  if (!station.is_in_intersection()) return false;

  const auto lane_pt = station.GetLanePoint();
  const auto &lane_info = smm.FindLaneInfoOrDie(lane_pt.lane_id());
  for (const auto &[idx, frac] : lane_info.intersections) {
    if (smm.IntersectionAt(idx).proto->traffic_light_controlled() &&
        frac[0] <= lane_pt.fraction() && lane_pt.fraction() <= frac[1]) {
      return true;
    }
  }
  return false;
}

void AddBoundariesToIntervals(const DrivePassage &drive_passage,
                              std::vector<IndexedStationBoundary> boundaries,
                              std::vector<BoundaryInterval> *intervals) {
  if (boundaries.size() < 2) return;

  BoundaryInterval interval;
  interval.type = boundaries.front().second.type;
  interval.points.reserve(boundaries.size());
  for (const auto &boundary : boundaries) {
    interval.points.emplace_back(drive_passage.station(boundary.first)
                                     .lat_point(boundary.second.lat_offset));
  }
  intervals->emplace_back(std::move(interval));
}

std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const DrivePassage &drive_passage, double cutoff_s) {
  std::vector<BoundaryInterval> intervals;
  std::vector<std::vector<IndexedStationBoundary>> active_intervals;
  for (const auto index : drive_passage.stations().index_range()) {
    const auto &station = drive_passage.station(index);
    if (station.accumulated_s() < -kRouteStationUnitStep) continue;
    if (station.accumulated_s() > cutoff_s) break;

    std::vector<StationBoundary> new_boundaries;
    for (const auto &boundary : station.boundaries()) {
      if (!boundary.IsSolid() ||
          boundary.type == StationBoundaryType::VIRTUAL_CURB) {
        // Only consider real boundaries here, not the virtual curbs.
        continue;
      }

      double match_dist = kContinuousBoundaryMaxLatOffset;
      int match_idx = -1;
      for (int j = 0; j < active_intervals.size(); ++j) {
        const auto &interval_back = active_intervals[j].back();
        if (interval_back.first.value() + 1 == index.value() &&
            interval_back.second.type == boundary.type) {
          const double lat_dist =
              std::abs(interval_back.second.lat_offset - boundary.lat_offset);
          if (lat_dist < match_dist) {
            match_dist = lat_dist;
            match_idx = j;
          }
        }
      }
      if (match_idx == -1) {
        new_boundaries.push_back(boundary);
      } else {
        active_intervals[match_idx].emplace_back(index, boundary);
      }
    }
    for (auto it = active_intervals.begin(); it != active_intervals.end();) {
      if (it->back().first != index) {
        AddBoundariesToIntervals(drive_passage, std::move(*it), &intervals);
        it = active_intervals.erase(it);
      } else {
        ++it;
      }
    }
    for (auto &new_boundary : new_boundaries) {
      active_intervals.emplace_back().emplace_back(index,
                                                   std::move(new_boundary));
    }
  }
  for (auto &interval : active_intervals) {
    AddBoundariesToIntervals(drive_passage, std::move(interval), &intervals);
  }

  return intervals;
}

int CountCrossingBoundaryType(
    const std::vector<BoundaryInterval> &solid_boundaries,
    const std::vector<Box2d> &ego_boxes, StationBoundaryType type) {
  int count = 0;
  for (const auto &boundary : solid_boundaries) {
    if (boundary.type != type) continue;

    const auto &boundary_pts = boundary.points;
    for (const auto &ego_box : ego_boxes) {
      bool has_overlap = false;
      for (int j = 1; j < boundary_pts.size(); ++j) {
        const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
        if (ego_box.HasOverlap(boundary_seg)) {
          has_overlap = true;
          break;
        }
      }
      if (has_overlap) {
        count += 1;
        break;
      }
    }
  }
  return count;
}

bool IsLanePointAtRouteDestination(const RouteSectionsInfo &sections_info,
                                   const mapping::LanePoint &lane_pt) {
  if (lane_pt.lane_id() != sections_info.destination().lane_id()) return false;

  constexpr double kEpsilon = 0.1;  // m.
  const auto &last_sec = sections_info.back();
  return last_sec.average_length *
             std::abs(last_sec.end_fraction - lane_pt.fraction()) <
         kEpsilon;
}

}  // namespace

CostVec TrajProgressCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(2);
  QCHECK_NOTNULL(extra_info);

  ASSIGN_OR_DIE(const auto point_proj,
                FindLastTrajPointOnRouteSections(*smm_, *sections_info_,
                                                 scheduler_output.drive_passage,
                                                 planner_output.traj_points));
  const double standard_progress =
      std::max(kMinStandardProgressDiff, kTrajectoryTimeHorizon * ego_v_);
  cost_vec[0] =
      std::max(0.0, standard_progress - point_proj.accum_s) / standard_progress;

  const auto start_lane_id =
      scheduler_output.drive_passage.lane_path().front().lane_id();
  // Regularization is related to lane speed limit.
  const auto ref_speed_diff =
      std::max(kMinLaneSpeedDiffStandard,
               kLaneSpeedDiffToLimitRatio *
                   smm_->QueryLaneSpeedLimitById(start_lane_id));
  const auto [lane_speed, block_id, continuous_num] =
      FindOrDie(lane_speed_map_, start_lane_id);
  cost_vec[1] = std::min(1.0, (max_lane_speed_ - lane_speed) / ref_speed_diff);
  if (continuous_num > 0) cost_vec[1] /= continuous_num;

  extra_info->emplace_back(absl::StrFormat(
      "Progress: %.2f / %.2f", point_proj.accum_s, standard_progress));
  extra_info->emplace_back(absl::StrFormat("Lane speed: %.2f m/s", lane_speed));
  if (block_id.has_value()) {
    extra_info->back() += absl::StrFormat(" from %s", *block_id);
  }

  return cost_vec;
}

CostVec TrajMaxJerkCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(2);
  QCHECK_NOTNULL(extra_info);

  const auto &traj_pts = planner_output.traj_points;
  const int n_pts = traj_pts.size();
  int max_accel_idx = -1, max_decel_idx = -1, max_lat_idx = -1;

  double accel_jerk_cost = 0.0, decel_jerk_cost = 0.0;
  for (int i = 0; i < n_pts; ++i) {
    const auto lon_jerk_cost = traj_pts[i].j() * coeffs_[i];
    if (lon_jerk_cost > accel_jerk_cost) {
      accel_jerk_cost = lon_jerk_cost;
      max_accel_idx = i;
    }
    if (lon_jerk_cost < decel_jerk_cost) {
      decel_jerk_cost = lon_jerk_cost;
      max_decel_idx = i;
    }
  }
  cost_vec[0] = std::max(accel_jerk_cost / accel_jerk_constraint_,
                         decel_jerk_cost / decel_jerk_constraint_);

  std::vector<double> psi(n_pts);
  psi[0] = CalcPsi(traj_pts[0], traj_pts[1]);
  for (int i = 1; i < n_pts - 1; ++i) {
    psi[i] = CalcPsi(traj_pts[i - 1], traj_pts[i + 1]);
  }
  psi[n_pts - 1] = CalcPsi(traj_pts[n_pts - 1], traj_pts[n_pts - 2]);

  double max_lat_jerk_cost = 0.0;
  for (int i = 0; i < n_pts; ++i) {
    const double lat_jerk_cost = CalcLatJerk(traj_pts[i], psi[i]) * coeffs_[i];
    if (lat_jerk_cost > max_lat_jerk_cost) {
      max_lat_jerk_cost = lat_jerk_cost;
      max_lat_idx = i;
    }
  }
  cost_vec[1] = max_lat_jerk_cost / lat_jerk_constraint_;

  if (max_accel_idx != -1) {
    extra_info->emplace_back(absl::StrFormat("Max accel jerk cost %.2f at %d",
                                             accel_jerk_cost, max_accel_idx));
  }
  if (max_decel_idx != -1) {
    extra_info->emplace_back(absl::StrFormat("Max decel jerk cost %.2f at %d",
                                             -decel_jerk_cost, max_decel_idx));
  }
  if (max_lat_idx != -1) {
    extra_info->emplace_back(absl::StrFormat("Max lat jerk cost %.2f at %d",
                                             max_lat_jerk_cost, max_lat_idx));
  }

  return cost_vec;
}

CostVec TrajLaneChangeCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(4);
  QCHECK_NOTNULL(extra_info);

  const auto &drive_passage = scheduler_output.drive_passage;
  const bool target_switched = prev_lp_from_current_->front().lane_id() !=
                               drive_passage.lane_path().front().lane_id();
  cost_vec[0] = btof(target_switched);

  const auto lc_stage = scheduler_output.lane_change_state.stage();
  const bool lc_ongoing = lc_stage == LaneChangeStage::LCS_EXECUTING ||
                          lc_stage == LaneChangeStage::LCS_PAUSE;
  const auto ego_sl = *drive_passage.QueryFrenetCoordinateAt(ego_pos_);
  const double lat_offset = std::abs(ego_sl.l);
  cost_vec[1] = lc_ongoing ? lat_offset / kDefaultLaneWidth : 0.0;

  double avg_dist = 0.0;
  if (prev_traj_ff_or_.ok()) {
    const auto &traj_pts = planner_output.traj_points;
    for (const auto &pt : traj_pts) {
      avg_dist += std::abs(
          prev_traj_ff_or_->XYToSL(Vec2dFromApolloTrajectoryPointProto(pt)).l);
    }
    avg_dist /= traj_pts.size();
  }
  cost_vec[2] = avg_dist / kStandardAverageTrajDiff;

  // Discourage lane change in intersection at low speed.
  constexpr double kLcPreviewTime = 5.0;  // s.
  const double preview_t = (lat_offset / kDefaultLaneWidth) * kLcPreviewTime;
  const double preview_s =
      ego_sl.s + ego_ra_to_front_ + std::max(ego_v_, kMinLCSpeed) * preview_t;
  const bool in_intersection =
      IsInTlControlledIntersection(*smm_, drive_passage, ego_sl.s) ||
      IsInTlControlledIntersection(*smm_, drive_passage, preview_s);
  cost_vec[3] =
      btof(lc_ongoing && in_intersection && ego_v_ < kMinLcSpeedInIntersection);

  extra_info->emplace_back(absl::StrFormat("Has switched target lane path: %s",
                                           btoa(target_switched)));
  extra_info->emplace_back(
      absl::StrFormat("Is performing lane change: %s", btoa(lc_ongoing)));
  extra_info->emplace_back(absl::StrFormat("Ego lat offset: %.2f", lat_offset));
  extra_info->emplace_back(
      absl::StrFormat("Average dist to prev traj: %.2f", avg_dist));
  extra_info->emplace_back(
      absl::StrFormat("Preview in intersection: %s", btoa(in_intersection)));

  return cost_vec;
}

CostVec TrajCrossSolidBoundaryCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(4);
  QCHECK_NOTNULL(extra_info);

  const auto &traj_pts = planner_output.traj_points;
  const auto &drive_passage = scheduler_output.drive_passage;

  // Only check a first small part of trajectory on lane change pause.
  const int check_first_n =
      scheduler_output.lane_change_state.stage() == LaneChangeStage::LCS_PAUSE
          ? std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1, traj_pts.size())
          : traj_pts.size();
  const auto last_pt =
      Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
  const auto last_pt_s = drive_passage.QueryFrenetLonOffsetAt(last_pt)->accum_s;

  constexpr double kTrajectorySExtension = 10.0;  // m.
  const auto solid_boundaries = FindSolidBoundaryIntervals(
      drive_passage, last_pt_s + kTrajectorySExtension);

  constexpr int kCheckEveryNPt = 5;
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(
      CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
  for (int i = kCheckEveryNPt - 1; i < check_first_n; i += kCheckEveryNPt) {
    const auto &traj_pt = traj_pts[i];
    Box2d ego_box(Vec2dFromApolloTrajectoryPointProto(traj_pt),
                  traj_pt.path_point().theta(), ego_length_, ego_width_);
    ego_boxes.emplace_back(std::move(ego_box));
  }
  // For low speed condition before a stop line.
  const Segment2d last_pt_to_ref_center_seg(
      last_pt, scheduler_output.sl_boundary.QueryReferenceCenterXY(last_pt_s));
  ego_boxes.emplace_back(last_pt_to_ref_center_seg, ego_width_);

  cost_vec[0] = CountCrossingBoundaryType(solid_boundaries, ego_boxes,
                                          StationBoundaryType::SOLID_WHITE);
  cost_vec[1] = CountCrossingBoundaryType(solid_boundaries, ego_boxes,
                                          StationBoundaryType::SOLID_YELLOW);
  cost_vec[2] = CountCrossingBoundaryType(
      solid_boundaries, ego_boxes, StationBoundaryType::SOLID_DOUBLE_YELLOW);
  cost_vec[3] = CountCrossingBoundaryType(solid_boundaries, ego_boxes,
                                          StationBoundaryType::CURB);

  extra_info->emplace_back(
      absl::StrFormat("Solid white: %d", static_cast<int>(cost_vec[0])));
  extra_info->emplace_back(
      absl::StrFormat("Solid yellow: %d", static_cast<int>(cost_vec[1])));
  extra_info->emplace_back(absl::StrFormat("Solid double yellow: %d",
                                           static_cast<int>(cost_vec[2])));
  extra_info->emplace_back(
      absl::StrFormat("Curb: %d", static_cast<int>(cost_vec[3])));

  return cost_vec;
}

CostVec TrajMinDistToObjectsCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(1);
  QCHECK_NOTNULL(extra_info);

  const auto &traj_pts = planner_output.traj_points;
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(traj_pts.size());
  for (const auto &traj_pt : traj_pts) {
    const Vec2d center = Vec2dFromApolloTrajectoryPointProto(traj_pt);
    const Vec2d tangent =
        Vec2d::FastUnitFromAngle(traj_pt.path_point().theta());
    ego_boxes.push_back(Box2d(center + ego_center_to_ra_ * tangent, tangent,
                              ego_length_, ego_width_));
  }

  const auto calc_cost = [this](double dist, int i) {
    return dist >= kObjDistCutoffDistance
               ? 0.0
               : coeffs_[i] * Sqr(1.0 - dist / kObjDistCutoffDistance);
  };

  double max_cost = 0.0;
  std::string closest_id;
  int max_cost_idx = -1;
  for (const auto &traj_with_time_range :
       planner_output.considered_st_objects) {
    const auto &traj = traj_with_time_range.st_traj();
    if (traj.object_type() == ObjectType::OT_VEGETATION) continue;

    if (traj.is_stationary()) {
      const auto &obj_cont = traj.states().front().contour;
      for (int i = 0; i < ego_boxes.size(); ++i) {
        const double dist = obj_cont.DistanceTo(ego_boxes[i]);
        const double cost = calc_cost(dist, i);
        if (cost > max_cost) {
          max_cost = cost;
          closest_id = traj.traj_id();
          max_cost_idx = i;
        }
      }
    } else {
      const auto &obj_states = traj.states();
      for (int i = 0; i < obj_states.size(); ++i) {
        const auto time = obj_states[i].traj_point->t();
        if (!traj_with_time_range.GetDecisionTypeAtTime(time).has_value()) {
          continue;
        }
        const double dist = obj_states[i].contour.DistanceTo(ego_boxes[i]);
        const double cost = calc_cost(dist, i);
        if (cost > max_cost) {
          max_cost = cost;
          closest_id = traj.traj_id();
          max_cost_idx = i;
        }
      }
    }
  }
  cost_vec[0] = max_cost;

  extra_info->emplace_back(
      max_cost_idx == -1 ? "No close trajectory."
                         : absl::StrFormat("Max cost from traj %s at idx %d.",
                                           closest_id, max_cost_idx));

  return cost_vec;
}

CostVec TrajRouteLookAheadCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(3);
  QCHECK_NOTNULL(extra_info);

  const auto start_lane_id =
      scheduler_output.drive_passage.lane_path().front().lane_id();

  constexpr double kEpsilon = 1.0;  // m.
  const double length_along_route =
      FindOrDie(len_along_route_map_, start_lane_id);
  const bool is_max_len_along_route =
      length_along_route >= max_len_along_route_ - kEpsilon;
  cost_vec[0] = is_max_len_along_route
                    ? 0.0
                    : Sqr(1.0 - length_along_route / local_horizon_);

  // Lanes connected to destination will have 10m longer driving distance.
  // See onboard/planner/router/route_sections_info.cc.
  const double driving_dist = FindOrDie(driving_dist_map_, start_lane_id);
  cost_vec[1] = driving_dist > std::min(route_len_ + kEpsilon,
                                        kReachDestinationCutOffDist)
                    ? 0.0
                    : Sqr(1.0 - driving_dist / kReachDestinationCutOffDist);

  // Preview beyond the local map horizon for lane changes that are far away.
  const int lc_num_to_targets =
      FindOrDie(lc_num_to_targets_map_, start_lane_id);
  cost_vec[2] = btof((max_len_along_route_ >= local_horizon_ - kEpsilon ||
                      !is_max_len_along_route) &&
                     lc_num_to_targets > 1 && lc_num_to_targets > min_lc_num_);

  extra_info->emplace_back(
      absl::StrFormat("Length along route: %.2f", length_along_route));
  extra_info->emplace_back(absl::StrFormat(
      "Reach destination: %s.", btoa(driving_dist > route_len_ + kEpsilon)));
  extra_info->emplace_back(
      absl::StrFormat("Driving distance: %.2f.", driving_dist));

  return cost_vec;
}

CostVec TrajBoundaryExpansionCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(1);
  QCHECK_NOTNULL(extra_info);

  ASSIGN_OR_DIE(const double ego_l,
                scheduler_output.drive_passage.QueryFrenetLatOffsetAt(ego_pos_),
                "Plan start point is not on drive passage!");

  const auto &path_boundary = scheduler_output.sl_boundary;
  const auto &left_l_vec = path_boundary.target_left_l_vector();
  const auto &right_l_vec = path_boundary.target_right_l_vector();
  const double left_offset =
      std::accumulate(left_l_vec.begin(), left_l_vec.end(), 0.0) /
      path_boundary.size();
  const double right_offset =
      std::accumulate(right_l_vec.begin(), right_l_vec.end(), 0.0) /
      path_boundary.size();

  const double dist = left_offset > -right_offset
                          ? std::abs(left_offset - ego_l)
                          : std::abs(ego_l - right_offset);

  constexpr double kDefaultOneAndHalfLaneWidth = 1.5 * kDefaultLaneWidth;
  cost_vec[0] = dist / kDefaultOneAndHalfLaneWidth;

  extra_info->emplace_back(
      absl::StrFormat("Average width left: %.2f", left_offset));
  extra_info->emplace_back(
      absl::StrFormat("Average width right: %.2f", -right_offset));

  return cost_vec;
}

CostVec TrajProhibitedRegionCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(1);
  QCHECK_NOTNULL(extra_info);

  const int traj_size = planner_output.traj_points.size();
  const auto &passage = scheduler_output.drive_passage;
  std::vector<FrenetCoordinate> traj_pts_sl;
  traj_pts_sl.reserve(traj_size);
  for (const auto &traj_pt : planner_output.traj_points) {
    traj_pts_sl.emplace_back(*passage.QueryFrenetCoordinateAt(
        Vec2dFromApolloTrajectoryPointProto(traj_pt)));
  }

  // Regions behind stalled objects.
  std::vector<double> behind_stalled(traj_size, 0.0);
  std::vector<std::string> related_objs;
  const bool passage_reached_dest = IsLanePointAtRouteDestination(
      *sections_info_, passage.lane_path().back());
  const double last_pt_s = traj_pts_sl.back().s;
  for (const auto &traj_with_time_range :
       planner_output.considered_st_objects) {
    const auto &traj = traj_with_time_range.st_traj();
    if (!stalled_objects_->contains(traj.object_id())) continue;

    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (passage_reached_dest && passage.lane_path().length() +
                                        passage.lane_path_start_s() -
                                        aabbox.s_max <
                                    kIgnoreObjectsAtRouteEndLen) {
      // Ignore stalled objects at route end.
      continue;
    }
    if (last_pt_s > aabbox.s_min) continue;  // Just passing by.

    const auto obj_lane_pt =
        passage.FindNearestStationAtS(aabbox.center().s).GetLanePoint();
    const auto *obj_sec =
        sections_info_->FindSegmentContainingLanePointOrNull(obj_lane_pt);
    if (obj_sec == nullptr) continue;

    const double punish_factor =
        obj_sec->lane_ids.size() > 1
            ? 1.0
            : kSingleLaneSectionStalledRearRegionFactor;

    const double region_len =
        std::max(ego_v_ * kTimeToStalledRear, kStalledRearRegionMinLength);
    const double region_rear_s =
        std::max(passage.front_s(), aabbox.s_min - region_len);

    bool related = false;
    for (int i = 0; i < traj_size; ++i) {
      const auto &pt_sl = traj_pts_sl[i];
      if (region_rear_s <= pt_sl.s && pt_sl.s <= aabbox.s_min &&
          aabbox.l_min <= pt_sl.l + ego_half_width_ &&
          pt_sl.l - ego_half_width_ <= aabbox.l_max) {
        related = true;
        behind_stalled[i] = std::max(behind_stalled[i], punish_factor);
      }
    }
    if (related) related_objs.emplace_back() = traj.object_id();
  }
  double cost_sum = 0.0;
  for (int i = 0; i < traj_size; ++i) {
    cost_sum += behind_stalled[i] * (kTrajectorySteps - i - 1);
  }
  constexpr double kOneOverWeightSum =
      2.0 / (kTrajectorySteps * (kTrajectorySteps - 1));
  cost_vec[0] = cost_sum * kOneOverWeightSum;

  if (related_objs.empty()) {
    extra_info->emplace_back("No related regions found.");
  } else {
    auto &rear_region_info = extra_info->emplace_back("Stalled objects:");
    for (const auto &obj_id : related_objs) rear_region_info += " " + obj_id;
  }

  return cost_vec;
}

CostVec TrajIsFallbackCost::ComputeCost(
    const SchedulerOutput &scheduler_output,
    const EstPlannerOutput &planner_output,
    std::vector<std::string> *extra_info) const {
  CostVec cost_vec(1);
  QCHECK_NOTNULL(extra_info);

  cost_vec[0] = btof(scheduler_output.is_fallback);

  extra_info->emplace_back(
      absl::StrFormat("Is fallback: %s.", btoa(scheduler_output.is_fallback)));

  return cost_vec;
}

}  // namespace qcraft::planner
