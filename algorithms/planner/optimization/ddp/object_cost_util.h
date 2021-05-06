#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

namespace optimizer {

void AddObjectCosts(
    std::string_view base_name, const std::vector<TrajectoryPoint> &init_traj,
    const DrivePassage &drive_passage, const PathSlBoundary &path_boundary,
    const ConstraintManager &constraint_manager,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const TrajectoryOptimizerCostWeightParamsProto &cost_weight_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const TrajectoryOptimizerVehicleModelParamsProto
        &trajectory_optimizer_vehicle_model_params,
    const std::vector<std::unique_ptr<AvModelHelper<Mfob>>> &av_model_helpers,
    std::vector<double> *inner_path_boundary_gains,
    std::vector<std::unique_ptr<Cost<Mfob>>> *costs, ThreadPool *thread_pool);

void CalcPartitionHalfContourInfo(const Vec2d &x, const Vec2d &obj_x,
                                  const Polygon2d &contour, double buffer,
                                  std::vector<Segment2d> *lines, Vec2d *ref_x,
                                  Vec2d *ref_tangent, double *offset);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_OBJECT_COST_UTIL_H_
