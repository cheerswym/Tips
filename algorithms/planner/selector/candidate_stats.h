#ifndef ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_
#define ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/scheduler_output.h"

namespace qcraft::planner {

struct LaneSpeedInfo {
  double lane_speed;
  std::optional<std::string> block_obj;
  int continuous_object_num;
};

struct ProgressStats {
  ProgressStats(const SemanticMapManager &smm,
                const ApolloTrajectoryPointProto &plan_start_point,
                const VehicleGeometryParamsProto &vehicle_geom,
                const std::vector<SchedulerOutput> &scheduler_outputs,
                const std::vector<PlannerStatus> &est_status,
                const std::vector<EstPlannerOutput> &results);

  double ego_v;
  absl::flat_hash_map<mapping::ElementId, LaneSpeedInfo> lane_speed_map;
  double max_lane_speed = 0.0;
};

struct RouteLookAheadStats {
  RouteLookAheadStats(const SemanticMapManager &smm,
                      const RouteSectionsInfo &sections_info,
                      const std::vector<LanePathInfo> &lp_infos,
                      const absl::flat_hash_set<std::string> &stalled_objects,
                      const std::vector<SchedulerOutput> &scheduler_outputs,
                      const std::vector<PlannerStatus> &est_status,
                      const std::vector<EstPlannerOutput> &results);

  double route_len, local_horizon;
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map,
      len_along_route_map;
  absl::flat_hash_map<mapping::ElementId, int> lc_num_to_targets_map;
  double max_length_along_route = 0.0;
  int min_lc_num = INT_MAX;
};
// ----------------------------------------------------------------

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_
