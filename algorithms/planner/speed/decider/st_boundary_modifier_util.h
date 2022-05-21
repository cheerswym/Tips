#ifndef ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_

#include <algorithm>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

struct StBoundaryModificationResult {
  // Will be empty if the new trajectory has no overlap with path.
  std::vector<StBoundaryWithDecision> newly_generated_st_boundaries_wd;
  SpacetimeObjectTrajectory processed_st_traj;
  StBoundaryModifierProto::ModifierType modifier_type;
};

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryByDecelAfterDelay(
    const SpacetimeObjectTrajectory& st_traj, double delay, double decel);

/**
 * @brief Calculate the minimum deceleration an object has to take if it wishes
 * to stop within a given distance and time delay. The object is assumed to keep
 * its initial velocity and acceleration within the time delay.
 * @param s stop distance.
 * @param v0 initial velocity.
 * @param a0 initial acceleration.
 * @param t_delay time delay.
 * @return return std::nullopt if a legal deceleration can't be calculated.
 */
inline std::optional<double> ComputeSafeStopDecelWithDelayedAction(
    double s, double v0, double a0, double t_delay) {
  if (v0 < 0.0) {
    return std::nullopt;
  }
  if (a0 < 0.0) {
    t_delay = std::min(std::fabs(v0 / a0), t_delay);
  }
  const double s_delay = (v0 + 0.5 * a0 * t_delay) * t_delay;
  if (s_delay >= s) {
    return std::nullopt;
  }
  const double v_delay = std::max(0.0, v0 + a0 * t_delay);
  const double ds = s - s_delay;
  return -0.5 * v_delay * v_delay / ds;
}

template <typename T>
void ModifyAndUpdateStBoundaries(
    const T& modifier_input,
    const std::function<std::optional<StBoundaryModificationResult>(
        const T&, const StBoundaryWithDecision&)>& modifier_func,
    absl::flat_hash_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  std::vector<StBoundaryWithDecision> all_newly_generated_st_boundaries_wd;

  const auto ignore_modified_st_boundary =
      [](const std::string& decision_info,
         StBoundaryWithDecision* st_boundary_wd) {
        st_boundary_wd->set_id(absl::StrCat(st_boundary_wd->id(), "|raw"));
        st_boundary_wd->set_decision_type(StBoundaryProto::IGNORE);
        st_boundary_wd->set_decision_reason(
            StBoundaryProto::ST_BOUNDARY_MODIFIER);
        st_boundary_wd->set_decision_info(decision_info);
      };

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (const auto& traj_id = st_boundary_wd.raw_st_boundary()->traj_id();
        traj_id.has_value() && processed_st_objects->contains(*traj_id)) {
      // Ignore the st-boundary if its associated prediction has been
      // modified.
      ignore_modified_st_boundary("ignore associated st-boundary modified",
                                  &st_boundary_wd);
    } else {
      auto res = modifier_func(modifier_input, st_boundary_wd);
      if (res.has_value()) {
        all_newly_generated_st_boundaries_wd.reserve(
            all_newly_generated_st_boundaries_wd.size() +
            res->newly_generated_st_boundaries_wd.size());
        std::move(res->newly_generated_st_boundaries_wd.begin(),
                  res->newly_generated_st_boundaries_wd.end(),
                  std::back_inserter(all_newly_generated_st_boundaries_wd));
        QCHECK(st_boundary_wd.traj_id().has_value());
        processed_st_objects->insert_or_assign(
            *st_boundary_wd.traj_id(), std::move(res->processed_st_traj));
        ignore_modified_st_boundary(
            absl::StrCat(
                "ignore modified by ",
                StBoundaryModifierProto::ModifierType_Name(res->modifier_type)),
            &st_boundary_wd);
      }
    }
  }

  // Append newly generated st-boundaries with decision to original ones.
  std::move(all_newly_generated_st_boundaries_wd.begin(),
            all_newly_generated_st_boundaries_wd.end(),
            std::back_inserter(*st_boundaries_wd));
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_MODIFIER_UTIL_H_
