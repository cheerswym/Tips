#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {
namespace optimizer {

void AddTrajCharts(
    const std::string &base_name, const std::vector<TrajectoryPlotInfo> &trajs,
    google::protobuf::RepeatedPtrField<vis::vantage::ChartDataProto>
        *charts_data);

void AddCostsChart(const std::string &base_name,
                   const DdpOptimizerDebugProto &ddp_opt_proto,
                   vis::vantage::ChartDataBundleProto *charts_data);

void AddCompareTrajCanvas(const std::string &base_name,
                          const std::vector<TrajectoryPoint> &first_traj,
                          const std::string &first_name,
                          const std::vector<TrajectoryPoint> &second_traj,
                          const std::string &second_name);

absl::Status CompareWithIpopt(
    const std::string &base_name, const std::string &canvas_base_name,
    const std::vector<TrajectoryPoint> &init_traj,
    const std::vector<TrajectoryPoint> &smooth_init_traj,
    const std::vector<TrajectoryPoint> &result_traj,
    bool enable_comparison_debug_info_output,
    const DdpOptimizerDebugProto &ddp_debug_proto,
    const TrajectoryPlotInfo &ddp_result_traj, const optimizer::Mfob *problem,
    vis::vantage::ChartDataBundleProto *charts_data);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_UTIL_H_
