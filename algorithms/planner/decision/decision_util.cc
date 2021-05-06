#include "onboard/planner/decision/decision_util.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/global/trace.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements) {
  QCHECK_GT(elements.size(), 0);
  ConstraintProto::SpeedRegionProto merged_ele = elements[0];
  int begin_idx = 0;
  double min_s = std::numeric_limits<double>::infinity();
  int end_idx = 0;
  double max_s = -std::numeric_limits<double>::infinity();
  for (int i = 0, n = elements.size(); i < n; ++i) {
    const auto &ele = elements[i];
    if (ele.start_s() < min_s) {
      min_s = ele.start_s();
      begin_idx = i;
    }
    if (ele.end_s() > max_s) {
      max_s = ele.end_s();
      end_idx = i;
    }
  }
  merged_ele.set_start_s(min_s);
  merged_ele.set_end_s(max_s);
  *merged_ele.mutable_start_point() = elements[begin_idx].start_point();
  *merged_ele.mutable_end_point() = elements[end_idx].end_point();
  return merged_ele;
}

void ApplySpeedConstraintForReferenceSpeedProfile(
    double s_start, double s_end, double v_max,
    const DrivePassage &drive_passage, std::vector<double> *v_s) {
  QCHECK_LE(s_start, s_end);
  QCHECK_EQ(drive_passage.size(), v_s->size());
  // Max speed dictated by this speed constraint as a function of s.
  // Between s_start and s_end, max speed is v_max. Before that the max
  // speed follows a constant deceleration profile, and after that a
  // constant acceleration profile.
  const double v_max_sqr = Sqr(v_max);
  const auto compute_v_constraint_sqr = [s_start, s_end, v_max_sqr](double s) {
    constexpr double kAccelLimit = 2.0;  // m/s^2.
    if (s < s_start) {
      // Deceleration segment.
      return v_max_sqr + (s_start - s) * (kAccelLimit * 2.0);
    }
    if (s > s_end) {
      // Acceleration segment.
      return v_max_sqr + (s - s_end) * (kAccelLimit * 2.0);
    }
    return v_max_sqr;
  };

  for (int i = 0; i < drive_passage.size(); ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    const double v_constraint_sqr = compute_v_constraint_sqr(s);
    if (v_constraint_sqr < Sqr((*v_s)[i])) {
      (*v_s)[i] = std::sqrt(v_constraint_sqr);
    }
  }
}

// This function is used to create speed profile in traffic light decision
// in planner 3.0. Future design may allow to remove this function if tl
// decision won't use speed profile anymore.
std::vector<double> CreateSpeedProfileWithConstraints(
    double v_now, const DrivePassage &drive_passage,
    const PlannerSemanticMapManager &psmm,
    absl::Span<const ConstraintProto::SpeedRegionProto> speed_zones,
    absl::Span<const ConstraintProto::StopLineProto> stop_points) {
  SCOPED_QTRACE("CreateSpeedProfileWithConstraints");

  // Do a crude speed planning along the ref path based on empty_road_decision
  // speed zones and stop points. The speed planning takes places in the v-s
  // space, and assumes a fixed acceleration/deceleration. This is similar to
  // AccSpeedCalculator.
  VLOG(2) << "Building v-s profile.";

  std::vector<double> v_s(drive_passage.size());
  for (int i = 0; i < drive_passage.size(); ++i) {
    const double speed_limit_mps =
        drive_passage.station(StationIndex(i)).speed_limit();  // m/s
    v_s[i] = speed_limit_mps;
  }

  for (const auto &speed_zone : speed_zones) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        speed_zone.start_s(), speed_zone.end_s(), speed_zone.max_speed(),
        drive_passage, &v_s);
  }
  for (const auto &stop_point : stop_points) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        stop_point.s(), /*s_end=*/std::numeric_limits<double>::infinity(),
        /*v_max=*/0.0, drive_passage, &v_s);
  }

  for (int i = 0; i < drive_passage.size(); ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    const double v_constraint =
        std::pow(std::abs(s) + pow(std::abs(v_now) / 2.3, 2.5), 0.4) * 2.3;
    (v_s)[i] = std::min((v_s)[i], v_constraint);
  }

  VLOG(4) << "Speed profile:";
  for (int i = 0; i < drive_passage.size(); ++i) {
    VLOG(4) << "i = " << i << " path_s = "
            << drive_passage.station(StationIndex(i)).accumulated_s()
            << " v = " << (v_s)[i];
  }

  return v_s;
}

SpeedProfile IntegrateSpeedProfile(const DrivePassage &drive_passage,
                                   const std::vector<double> &v_s) {
  // Integrate the v-s profile in time.
  QCHECK_EQ(drive_passage.size(), v_s.size());
  VLOG(2) << "Integrating v-s profile to create the s-t profile.";
  std::vector<double> path_s(drive_passage.size());
  for (int i = 0; i < drive_passage.size(); ++i) {
    path_s[i] = drive_passage.station(StationIndex(i)).accumulated_s();
  }
  const PiecewiseSqrtFunction<double, double> v_s_plf(path_s, v_s);
  std::vector<double> t(kTrajectorySteps);
  for (int i = 0; i < t.size(); ++i) t[i] = i * kTrajectoryTimeStep;
  std::vector<double> s(t.size());
  double current_s = 0.0;
  for (int i = 0; i < t.size(); ++i) {
    s[i] = current_s;
    const double current_v = v_s_plf(current_s);
    VLOG(4) << "i = " << i << " t = " << t[i] << " s = " << current_s
            << " v = " << current_v;
    current_s +=
        std::max(current_v, kJoptPlannerMinSpeed) * kTrajectoryTimeStep;
    current_s = std::min(path_s.back(), current_s);
  }

  // Create the speed profile from the s-t function.
  return SpeedProfile(PiecewiseLinearFunction(std::move(t), std::move(s)));
}

void UpdateDecisionConstraintDebugInfo(const ConstraintManager &constraint_mgr,
                                       ConstraintProto *constraint) {
  QCHECK_NOTNULL(constraint);
  for (const auto &stop_line : constraint_mgr.StopLine()) {
    constraint->add_stop_line()->CopyFrom(stop_line);
  }
  for (const auto &speed_region : constraint_mgr.SpeedRegion()) {
    constraint->add_speed_region()->CopyFrom(speed_region);
  }
  for (const auto &[traj_id, leading_object] :
       constraint_mgr.LeadingObjects()) {
    constraint->add_leading_object()->CopyFrom(leading_object);
  }
  for (const auto &ignore_object : constraint_mgr.IgnoreObject()) {
    constraint->add_ignore_object()->CopyFrom(ignore_object);
  }
}

bool EnableInitializerLaneChangeTargetDecision(
    const LaneChangeStage &lane_change_stage, const PathSlBoundary &sl_boundary,
    const FrenetBox &av_frenet_box) {
  constexpr double lateral_buffer = 0.1;  // m.
  if (lane_change_stage == LaneChangeStage::LCS_NONE) {
    // Lane keeping.
    return false;
  }
  const auto box_center_sl = av_frenet_box.center();
  const auto center_l = sl_boundary.QueryReferenceCenterL(box_center_sl.s);
  const auto min_l =
      std::min<double>(std::fabs(center_l - av_frenet_box.l_min),
                       std::fabs(center_l - av_frenet_box.l_max));
  const auto center_to_target_lane = std::fabs(box_center_sl.l - center_l);
  if (min_l < kDefaultHalfLaneWidth + lateral_buffer ||
      center_to_target_lane < kDefaultHalfLaneWidth) {
    // Assuming vehicle has entered target lane. Do not enable multiple
    // trajectory.
    // 1. frenet box l_min, l_max (corners of the vehicle) enters target lane.
    // 2. center_l in target lane (corners of the vehicle might be out because
    // of vehicle heading/geometry?).
    return false;
  }
  return true;
}

}  // namespace planner
}  // namespace qcraft
