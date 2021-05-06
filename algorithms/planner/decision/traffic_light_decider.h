#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed_profile.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

struct TlDirectiveGenerator {
  mapping::ElementId lane_id = -1;
  mapping::ElementId tl_id = -1;
  // TODO(zixuan): may can be deleted in the future
  double tl_route_s = std::numeric_limits<double>::infinity();
  double tl_path_s = std::numeric_limits<double>::infinity();
  bool proceed = true;
  bool protected_proceed = true;
  double stop_point = std::numeric_limits<double>::infinity();
  double turn_red_time_remaining = std::numeric_limits<double>::infinity();
  std::string reason;
};

absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
    const PlannerSemanticMapManager &psmm,
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &lane_path, const DrivePassage &passage,
    const TrafficLightInfoMap &tl_info_map,
    const SpeedProfile &preliminary_speed_profile,
    const TrafficLightDeciderStateProto &decider_state);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
