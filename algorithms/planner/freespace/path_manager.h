#ifndef ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_
#define ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_

#include <limits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/planner_params.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

struct PathManagerOutput {
  // Path.
  DirectionalPath path;
  // Whether this is a new path.
  bool is_new_path;
};

class PathManager {
 public:
  PathManager() = delete;
  explicit PathManager(const PathManagerStateProto &state);

  absl::StatusOr<PathManagerOutput> GeneratePath(
      bool new_task, const PoseProto &ego_pose,
      const CoordinateConverter &coordinate_converter, const Chassis &chassis,
      const HybridAStarParamsProto &hybrid_a_star_params,
      const VehicleGeometryParamsProto &veh_geo_params,
      const VehicleDriveParamsProto &veh_drive_params,
      absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
      const FreespaceMap &freespace_map, absl::Time unused_plan_time,
      PathManagerStateProto *path_mgr_state,
      HybridAStartDebugProto *ha_star_debug,
      PathManagerDebugProto *path_mgr_debug);

  PathManagerStateProto MakeState() {
    PathManagerStateProto state;
    *state.mutable_goal() = goal_;
    *state.mutable_global_goal() = global_goal_;
    state.set_task_type(task_type_);
    for (int i = 0; i < paths_.size(); ++i) {
      paths_[i].ToProto(state.add_paths());
      auto *path_pos = state.add_paths_global_pos();
      auto *path_theta = state.add_paths_global_theta();
      path_pos->mutable_pos()->Reserve(paths_[i].path.size());
      path_theta->mutable_theta()->Reserve(paths_[i].path.size());
      const auto &global_pos = paths_global_pos_[i];
      const auto &global_theta = paths_global_theta_[i];
      for (int j = 0; j < paths_[i].path.size(); ++j) {
        global_pos[j].ToProto(path_pos->add_pos());
        path_theta->add_theta(global_theta[j]);
      }
    }
    state.set_curr_path_idx(curr_path_idx_);
    state.set_drive_state(drive_state_);

    return state;
  }

  static PathManagerStateProto MakeInitState(
      const PathManagerStateProto::GlobalGoal &global_goal,
      const PathPoint &goal, FreespaceTaskProto::TaskType task_type) {
    PathManagerStateProto init_state;
    *init_state.mutable_goal() = goal;
    *init_state.mutable_global_goal() = global_goal;
    init_state.set_task_type(task_type);
    init_state.set_curr_path_idx(-1);
    init_state.set_drive_state(PathManagerStateProto::SWITCHING_TO_NEXT);

    return init_state;
  }

 private:
  // ********************** States **********************
  // Goal point. We should always have a goal, no matter it is valid or not.
  PathPoint goal_;
  PathManagerStateProto::GlobalGoal global_goal_;
  FreespaceTaskProto::TaskType task_type_ = FreespaceTaskProto::UNKNOWN_TASK;
  // Global smooth path towards goal which may contain more than one segments.
  // If there is no valid plan, it would be empty.
  std::vector<DirectionalPath> paths_;
  // Global coordinates of the global paths. Size should be equal to paths_;
  std::vector<std::vector<Vec2d>> paths_global_pos_;
  std::vector<std::vector<double>> paths_global_theta_;

  // Indicate which path segment is being followed and should be no larger than
  // paths_.size() - 1. If there is no valid plan, it would be -1.
  int curr_path_idx_ = -1;

  DriveState drive_state_ = PathManagerStateProto::UNKNOWN;

  // ********************** Data **********************
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_
