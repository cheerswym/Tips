#ifndef ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

absl::StatusOr<CrosswalkDeciderOutput> BuildCrosswalkConstraints(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerSemanticMapManager &psmm,
    const ApolloTrajectoryPointProto &plan_start_point,
    const DrivePassage &passage, const PlannerObjectManager &obj_mgr,
    double now_in_seconds,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
        &last_crosswalk_states);
}  // namespace planner
}  // namespace qcraft
#endif
