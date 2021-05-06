#ifndef ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

class DrivePassageFilter : public TrajectoryFilter {
 public:
  explicit DrivePassageFilter(const DrivePassage* drive_passage,
                              const PathSlBoundary* sl_boundary)
      : drive_passage_(QCHECK_NOTNULL(drive_passage)),
        sl_boundary_(QCHECK_NOTNULL(sl_boundary)) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;
  const PathSlBoundary* sl_boundary_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_
