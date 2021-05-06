#ifndef ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// This class filters reflected objects that are in AV proximity. Any objects
// that are considered reflected objects with in SDC's proximity (less than the
// provided `padding` distance) will be filtered.
class ReflectedObjectInProximityFilter : public TrajectoryFilter {
 public:
  ReflectedObjectInProximityFilter(
      const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
      double padding);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  Box2d padded_box_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_
