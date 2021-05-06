#ifndef ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_
#define ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

class ConstraintManager {
 public:
  ConstraintManager() = default;

  void AddSpeedRegion(const ConstraintProto::SpeedRegionProto& speed_region) {
    const auto it = std::lower_bound(
        speed_region_.begin(), speed_region_.end(), speed_region,
        [](const ConstraintProto::SpeedRegionProto& elem,
           const ConstraintProto::SpeedRegionProto& val) {
          return elem.start_s() < val.start_s();
        });
    speed_region_.insert(it, speed_region);
  }

  void AddTimeRegion(const ConstraintProto::TimeRegionProto& time_region) {
    time_region_.push_back(time_region);
  }

  void AddAvoidRegion(const ConstraintProto::AvoidRegionProto& avoid_region) {
    avoid_region_.push_back(avoid_region);
  }

  void AddStopLine(const ConstraintProto::StopLineProto& stop_line) {
    const auto it =
        std::lower_bound(stop_line_.begin(), stop_line_.end(), stop_line,
                         [](const ConstraintProto::StopLineProto& elem,
                            const ConstraintProto::StopLineProto& val) {
                           return elem.s() < val.s();
                         });
    stop_line_.insert(it, stop_line);
  }

  void AddLeadingObject(ConstraintProto::LeadingObjectProto leading_object) {
    leading_objects_map_.emplace(leading_object.traj_id(),
                                 std::move(leading_object));
  }

  void AddIgnoreObject(ConstraintProto::IgnoreObjectProto ignore_object) {
    ignore_object_ids_.emplace(ignore_object.traj_id());
    ignore_objects_.push_back(std::move(ignore_object));
  }

  absl::Span<const ConstraintProto::AvoidRegionProto> AvoidRegion() const {
    return avoid_region_;
  }

  absl::Span<const ConstraintProto::SpeedRegionProto> SpeedRegion() const {
    return speed_region_;
  }

  absl::Span<const ConstraintProto::TimeRegionProto> TimeRegion() const {
    return time_region_;
  }

  absl::Span<const ConstraintProto::StopLineProto> StopLine() const {
    return stop_line_;
  }

  const absl::flat_hash_map<std::string_view,
                            ConstraintProto::LeadingObjectProto>&
  LeadingObjects() const {
    return leading_objects_map_;
  }

  bool IsLeadingObject(std::string_view traj_id) const {
    return leading_objects_map_.find(traj_id) != leading_objects_map_.end();
  }

  absl::Span<const ConstraintProto::IgnoreObjectProto> IgnoreObject() const {
    return ignore_objects_;
  }

  bool IsIgnoreObject(std::string_view traj_id) const {
    return ignore_object_ids_.find(traj_id) != ignore_object_ids_.end();
  }

 private:
  std::vector<ConstraintProto::AvoidRegionProto> avoid_region_;
  std::vector<ConstraintProto::SpeedRegionProto> speed_region_;
  std::vector<ConstraintProto::TimeRegionProto> time_region_;
  std::vector<ConstraintProto::StopLineProto> stop_line_;
  std::vector<ConstraintProto::IgnoreObjectProto> ignore_objects_;

  absl::flat_hash_map<std::string_view, ConstraintProto::LeadingObjectProto>
      leading_objects_map_;

  absl::flat_hash_set<std::string> ignore_object_ids_;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_CONSTRAINT_MANAGER_H_
