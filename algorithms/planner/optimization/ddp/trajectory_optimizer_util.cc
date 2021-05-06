#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/ipopt/ipopt_adapter.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/trajectory_util.h"

DEFINE_string(
    traj_opt_compare_log_file_folder,
    "offboard/planner/optimizer/compare/data/",
    "Only used in comparison mode, it is the folder name of a log file."
    "Make sure the folder exists, see the default value as example.");
DEFINE_int32(traj_opt_ipopt_canvas_level, 0,
             "Traj opt ipopt solver canvas level.");

namespace qcraft {
namespace planner {
namespace optimizer {

namespace {

void ToIpoptDebugProto(const std::vector<TrajectoryPoint> &init_traj,
                       const std::vector<TrajectoryPoint> &smooth_init_traj,
                       const std::vector<TrajectoryPoint> &result_traj,
                       const IpoptOptimizerDebugHook<Mfob> &solver_debug_hook,
                       IpoptSolverDebugProto *ipopt_debug_proto) {
  // Write DdpDebugProto data.
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ipopt_debug_proto->add_init_traj());
  }
  for (int k = 0; k < smooth_init_traj.size(); ++k) {
    smooth_init_traj[k].ToProto(ipopt_debug_proto->add_smooth_init_traj());
  }
  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ipopt_debug_proto->add_final_traj());
  }

  const auto &init_costs = solver_debug_hook.init_costs();
  ipopt_debug_proto->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        ipopt_debug_proto->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.costs[i].first);
    cost_proto->set_cost(init_costs.costs[i].second);
  }
  const auto &final_costs = solver_debug_hook.final_costs();
  ipopt_debug_proto->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost *cost_proto =
        ipopt_debug_proto->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.costs[i].first);
    cost_proto->set_cost(final_costs.costs[i].second);
  }

  for (const auto &iteration : solver_debug_hook.iterations()) {
    IpoptSolverDebugProto::Iteration *iteration_proto =
        ipopt_debug_proto->add_iterations();
    for (int i = 0; i < iteration.final_xs.size(); ++i) {
      iteration_proto->add_final_xs(iteration.final_xs[i]);
    }
    for (int i = 0; i < iteration.final_us.size(); ++i) {
      iteration_proto->add_final_us(iteration.final_us[i]);
    }
    iteration_proto->set_final_cost(iteration.cost_info.cost);
    for (int i = 0; i < iteration.cost_info.costs.size(); ++i) {
      TrajectoryOptimizerCost *cost_proto = iteration_proto->add_costs();
      cost_proto->set_name(iteration.cost_info.costs[i].first);
      cost_proto->set_cost(iteration.cost_info.costs[i].second);
    }
  }
  ipopt_debug_proto->set_num_iters(solver_debug_hook.iterations().size());
}

void AddCompareTrajCharts(const std::string &base_name,
                          const TrajectoryPlotInfo &ddp_traj,
                          const TrajectoryPlotInfo &ipopt_traj,
                          vis::vantage::ChartDataBundleProto *charts_data) {
  QCHECK_EQ(ddp_traj.traj.size(), ipopt_traj.traj.size());

  const auto add_compare_charts =
      [&base_name, charts_data](
          const std::function<const TrajectoryPoint(int)> &traj_func1,
          const std::function<const TrajectoryPoint(int)> &traj_func2,
          const std::string &traj_name1, const std::string &traj_name2,
          int size) {
        if (charts_data == nullptr) return;
        if (size == 0) return;
        std::vector<double> s1, s2;
        std::vector<double> t;
        std::vector<double> x1, x2, y1, y2;
        std::vector<std::string> names = {
            "v_" + traj_name1,         "v_" + traj_name2,
            "a_" + traj_name1,         "a_" + traj_name2,
            "j_" + traj_name1,         "j_" + traj_name2,
            "lon_jerk_" + traj_name1,  "lon_jerk_" + traj_name2,
            "theta_" + traj_name1,     "theta_" + traj_name2,
            "kappa_" + traj_name1,     "kappa_" + traj_name2,
            "psi_" + traj_name1,       "psi_" + traj_name2,
            "chi_" + traj_name1,       "chi_" + traj_name2,
            "lat_accel_" + traj_name1, "lat_accel_" + traj_name2,
            "lat_jerk_" + traj_name1,  "lat_jerk_" + traj_name2};
        std::vector<std::vector<double>> values(names.size());
        s1.reserve(size);
        s2.reserve(size);
        t.reserve(size);
        x1.reserve(size);
        x2.reserve(size);
        y1.reserve(size);
        y2.reserve(size);
        for (auto &v : values) v.reserve(size);
        const TrajectoryPoint traj_point_base1 = traj_func1(0);
        const TrajectoryPoint traj_point_base2 = traj_func2(0);
        for (int i = 0; i < size; ++i) {
          const TrajectoryPoint traj_point1 = traj_func1(i);
          const TrajectoryPoint traj_point2 = traj_func2(i);
          s1.push_back(traj_point1.s() - traj_point_base1.s());
          s2.push_back(traj_point2.s() - traj_point_base2.s());
          t.push_back(traj_point1.t() - traj_point_base2.t());
          int y_index = 0;
          values[y_index++].push_back(traj_point1.v());
          values[y_index++].push_back(traj_point2.v());
          values[y_index++].push_back(traj_point1.a());
          values[y_index++].push_back(traj_point2.a());
          values[y_index++].push_back(traj_point1.j());
          values[y_index++].push_back(traj_point2.j());
          values[y_index++].push_back(ComputeLongitudinalJerk(traj_point1));
          values[y_index++].push_back(ComputeLongitudinalJerk(traj_point2));
          values[y_index++].push_back(traj_point1.theta());
          values[y_index++].push_back(traj_point2.theta());
          values[y_index++].push_back(traj_point1.kappa());
          values[y_index++].push_back(traj_point2.kappa());
          values[y_index++].push_back(traj_point1.psi());
          values[y_index++].push_back(traj_point2.psi());
          values[y_index++].push_back(traj_point1.chi());
          values[y_index++].push_back(traj_point2.chi());
          values[y_index++].push_back(ComputeLateralAcceleration(traj_point1));
          values[y_index++].push_back(ComputeLateralAcceleration(traj_point2));
          values[y_index++].push_back(ComputeLateralJerk(traj_point1));
          values[y_index++].push_back(ComputeLateralJerk(traj_point2));

          x1.push_back(traj_point1.pos().x());
          y1.push_back(traj_point1.pos().y());
          x2.push_back(traj_point2.pos().x());
          y2.push_back(traj_point2.pos().y());
        }

        vis::vantage::ChartDataProto *time_chart = charts_data->add_charts();
        time_chart->set_title(base_name + "/compare/time");
        vis::vantage::GenerateSubchartFromData(t, {s1}, "t",
                                               {"s_" + traj_name1}, {}, {{}},
                                               time_chart->add_subcharts());
        vis::vantage::GenerateSubchartFromData(t, {s2}, "t",
                                               {"s_" + traj_name2}, {}, {{}},
                                               time_chart->add_subcharts());
        vis::vantage::GenerateSubchartFromData(
            t, values, "t", names, {},
            std::vector<std::vector<std::string>>(names.size()),
            time_chart->add_subcharts());
      };

  add_compare_charts(
      [&ddp_traj](int index) { return ddp_traj.traj[index]; },
      [&ipopt_traj](int index) { return ipopt_traj.traj[index]; },
      ddp_traj.name, ipopt_traj.name, ddp_traj.traj.size());
}

absl::Status ExportToFile(const DdpOptimizerDebugProto &ddp_debug_proto,
                          const IpoptSolverDebugProto &ipopt_debug_proto) {
  TrajectoryOptimizerCompareProto debug_proto;
  *debug_proto.mutable_ddp() = ddp_debug_proto;
  *debug_proto.mutable_ipopt() = ipopt_debug_proto;
  const auto time_current = absl::Now();
  file_util::ProtoToTextFile(
      debug_proto,
      absl::StrFormat("%s%s_%d.pb.txt", FLAGS_traj_opt_compare_log_file_folder,
                      "data", absl::ToUnixMillis(time_current)));
  return absl::OkStatus();
}

}  // namespace

void AddCompareTrajCanvas(const std::string &base_name,
                          const std::vector<TrajectoryPoint> &first_traj,
                          const std::string &first_name,
                          const std::vector<TrajectoryPoint> &second_traj,
                          const std::string &second_name) {
  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&first_traj](int index) { return first_traj[index].pos(); },
          first_traj.size(), 0.1, 0.0),
      vis::Color(0.8, 0.4, 0.4),
      /*render_indices=*/true, base_name + "/" + first_name);
  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&second_traj](int index) { return second_traj[index].pos(); },
          second_traj.size(), 0.1, 0.0),
      vis::Color(0.4, 0.8, 0.8),
      /*render_indices=*/true, base_name + "/" + second_name);
}

void AddCostsChart(const std::string &base_name,
                   const DdpOptimizerDebugProto &ddp_opt_proto,
                   vis::vantage::ChartDataBundleProto *charts_data) {
  if (charts_data == nullptr) return;

  int costs_size = ddp_opt_proto.init_costs().costs_size();
  for (int i = 0; i < ddp_opt_proto.iterations_size(); ++i) {
    costs_size += ddp_opt_proto.iterations(i).costs_size();
  }
  if (costs_size == 0) {
    return;
  }
  vis::vantage::ChartDataProto *costs_chart = charts_data->add_charts();
  costs_chart->set_title(base_name + "/" + "costs");
  vis::vantage::ChartSubchartDataProto *costs_subchart =
      costs_chart->add_subcharts();
  costs_subchart->set_x_name("iterations");

  using DdpCost = std::tuple<std::string, double>;
  using DdpCosts = std::vector<DdpCost>;

  absl::btree_map<std::string, vis::vantage::ChartSeriesDataProto *>
      y_subcharts;
  DdpCosts init_costs;
  std::vector<DdpCosts> iteration_costs(ddp_opt_proto.iterations_size());

  init_costs.reserve(ddp_opt_proto.init_costs().costs_size());
  for (int i = 0; i < ddp_opt_proto.init_costs().costs_size(); ++i) {
    const auto [it, success] = y_subcharts.insert(
        {ddp_opt_proto.init_costs().costs(i).name(), nullptr});
    if (success) {
      it->second = costs_subchart->add_y();
      it->second->set_name(it->first);
    }
    init_costs.push_back({ddp_opt_proto.init_costs().costs(i).name(),
                          ddp_opt_proto.init_costs().costs(i).cost()});
  }
  for (int i = 0; i < ddp_opt_proto.iterations_size(); ++i) {
    iteration_costs[i].reserve(ddp_opt_proto.iterations(i).costs_size());
    for (int j = 0; j < ddp_opt_proto.iterations(i).costs_size(); ++j) {
      const auto [it, success] = y_subcharts.insert(
          {ddp_opt_proto.iterations(i).costs(j).name(), nullptr});
      if (success) {
        it->second = costs_subchart->add_y();
        it->second->set_name(it->first);
      }
      iteration_costs[i].push_back(
          {ddp_opt_proto.iterations(i).costs(j).name(),
           ddp_opt_proto.iterations(i).costs(j).cost()});
    }
  }

  auto add_costs_to_chart = [&y_subcharts](const DdpCosts &costs) {
    absl::btree_map<std::string, vis::vantage::ChartSeriesDataProto *> alt;
    for (int i = 0; i < costs.size(); ++i) {
      auto it = y_subcharts.find(std::get<std::string>(costs[i]));
      if (it != y_subcharts.end()) {
        it->second->add_values(std::get<double>(costs[i]));
        std::string tip =
            it->second->values_size() == 1
                ? absl::StrFormat("init\nname : %s\ncost : %f", it->first,
                                  std::get<double>(costs[i]))
                : absl::StrFormat("iteration : %d\nname : %s\ncost : %f",
                                  it->second->values_size() - 1, it->first,
                                  std::get<double>(costs[i]));
        it->second->add_tips(tip);
      }
      alt.insert(y_subcharts.extract(it));
    }
    for (auto &[name, y] : y_subcharts) {
      y->add_values(0);
      std::string tip =
          y->values_size() == 1
              ? absl::StrFormat("init\nname : %s\ncost : %f", name, 0.0)
              : absl::StrFormat("iteration : %d\nname : %s\ncost : %f",
                                y->values_size() - 1, name, 0.0);
      y->add_tips(tip);
    }
    y_subcharts.merge(alt);
  };

  const int interval = 10;

  add_costs_to_chart(init_costs);
  int x_value = 0;
  costs_subchart->add_x_values(x_value);

  for (const auto &costs : iteration_costs) {
    add_costs_to_chart(costs);
    x_value += interval;
    costs_subchart->add_x_values(x_value);
  }
}

void AddTrajCharts(
    const std::string &base_name, const std::vector<TrajectoryPlotInfo> &trajs,
    google::protobuf::RepeatedPtrField<vis::vantage::ChartDataProto>
        *charts_data) {
  const auto add_charts_for_traj =
      [&base_name, charts_data](
          const std::function<const TrajectoryPoint(int)> &traj_func, int size,
          const std::string &traj_name) {
        if (charts_data == nullptr) return;
        if (size == 0) return;
        std::vector<double> s;
        std::vector<double> t;
        std::vector<std::string> names = {
            "v",     "a",   "j",   "lon_jerk",  "theta",
            "kappa", "psi", "chi", "lat_accel", "lat_jerk"};
        std::vector<std::vector<double>> values(names.size());
        s.reserve(size);
        t.reserve(size);
        for (auto &v : values) v.reserve(size);

        const TrajectoryPoint traj_point0 = traj_func(0);
        for (int i = 0; i < size; ++i) {
          const TrajectoryPoint traj_point = traj_func(i);
          s.push_back(traj_point.s() - traj_point0.s());
          t.push_back(traj_point.t() - traj_point0.t());
          int y_index = 0;
          values[y_index++].push_back(traj_point.v());
          values[y_index++].push_back(traj_point.a());
          values[y_index++].push_back(traj_point.j());
          values[y_index++].push_back(ComputeLongitudinalJerk(traj_point));
          values[y_index++].push_back(traj_point.theta());
          values[y_index++].push_back(traj_point.kappa());
          values[y_index++].push_back(traj_point.psi());
          values[y_index++].push_back(traj_point.chi());
          values[y_index++].push_back(ComputeLateralAcceleration(traj_point));
          values[y_index++].push_back(ComputeLateralJerk(traj_point));
        }

        vis::vantage::ChartDataProto *path_chart = charts_data->Add();
        path_chart->set_title(base_name + "/" + traj_name + "/path");
        vis::vantage::GenerateSubchartFromData(s, {t}, "s", {"t"}, {}, {{}},
                                               path_chart->add_subcharts());
        vis::vantage::GenerateSubchartFromData(
            s, values, "s", names, {},
            std::vector<std::vector<std::string>>(names.size()),
            path_chart->add_subcharts());

        vis::vantage::ChartDataProto *time_chart = charts_data->Add();
        time_chart->set_title(base_name + "/" + traj_name + "/time");
        vis::vantage::GenerateSubchartFromData(t, {s}, "t", {"s"}, {}, {{}},
                                               time_chart->add_subcharts());
        vis::vantage::GenerateSubchartFromData(
            t, values, "t", names, {},
            std::vector<std::vector<std::string>>(names.size()),
            time_chart->add_subcharts());
      };
  for (const auto &traj : trajs) {
    add_charts_for_traj([&traj](int index) { return traj.traj[index]; },
                        traj.traj.size(), traj.name);
  }
}

absl::Status CompareWithIpopt(
    const std::string &base_name, const std::string &canvas_base_name,
    const std::vector<TrajectoryPoint> &init_traj,
    const std::vector<TrajectoryPoint> &smooth_init_traj,
    const std::vector<TrajectoryPoint> &result_traj,
    bool enable_comparison_debug_info_output,
    const DdpOptimizerDebugProto &ddp_debug_proto,
    const TrajectoryPlotInfo &ddp_result_traj, const Mfob *problem,
    vis::vantage::ChartDataBundleProto *charts_data) {
  IpoptAdapter<Mfob> solver(problem, "ipopt_traj_opt");
  solver.SetInitialPoints(smooth_init_traj);
  IpoptOptimizerDebugHook<Mfob> debug_hook;
  solver.AddHook(&debug_hook);
  std::string result_info;
  const auto output = solver.Solve(&result_info);
  VLOG(2) << "Ipopt solver result info: " << result_info;
  if (!output.ok()) {
    return absl::InternalError(output.status().message());
  }
  std::vector<TrajectoryPoint> result_points = std::move(*output);
  IpoptSolverDebugProto ipopt_debug_proto;
  ToIpoptDebugProto(init_traj, smooth_init_traj, result_traj, debug_hook,
                    &ipopt_debug_proto);
  std::vector<TrajectoryPlotInfo> ipopt_trajs = {
      {.traj = result_points, .name = "ipopt_res", .color = vis::Color::kBlue}};
  if (charts_data != nullptr) {
    AddTrajCharts(base_name, ipopt_trajs, charts_data->mutable_charts());
  }
  if (enable_comparison_debug_info_output) {
    if (FLAGS_traj_opt_ipopt_canvas_level > 0) {
      AddCompareTrajCanvas(canvas_base_name, result_traj, "ddp_res",
                           result_points, "ipopt_res");
    }
    AddCompareTrajCharts(base_name, ddp_result_traj, ipopt_trajs.front(),
                         charts_data);
    return ExportToFile(ddp_debug_proto, ipopt_debug_proto);
  } else {
    return absl::OkStatus();
  }
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
