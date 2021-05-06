#include "onboard/planner/initializer/initializer_util.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/colormap.h"

namespace qcraft::planner {

void SendSingleTrajectoryToCanvas(
    const MotionSearchOutput::TrajWithLeadingObject& traj_info, int idx,
    int planner_id) {
  auto& canvas = vantage_client_man::GetCanvas(
      absl::StrCat("multi_traj_idx_", idx, "_planner_", planner_id));
  canvas.SetGroundZero(1);
  const auto& trajectory = traj_info.trajectory;
  std::vector<Vec3d> points;
  points.reserve(trajectory.size());
  for (const auto& pt : trajectory) {
    Vec3d point(pt.path_point().x(), pt.path_point().y(), pt.relative_time());
    points.emplace_back(std::move(point));
  }
  // Draw trajectory.
  canvas.DrawLineStrip(points, vis::Color::kMagenta, 3);
  const auto& last_point = points.back();
  // Write leading object info.
  canvas.DrawText(absl::StrJoin(traj_info.leading_obj_ids, ","), last_point,
                  trajectory.front().path_point().theta() - M_PI / 2.0, 0.8,
                  vis::Color::kWhite);
  vantage_client_man::FlushAll();
}

void SendMultiTrajectoriesToCanvas(
    const std::vector<MotionSearchOutput::TrajWithLeadingObject>& multi_trajs,
    const std::string& canvas_channel) {
  auto& canvas = vantage_client_man::GetCanvas(canvas_channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i < multi_trajs.size(); i++) {
    const auto& traj_info = multi_trajs[i];
    const auto& trajectory = traj_info.trajectory;
    std::vector<Vec3d> points;
    points.reserve(trajectory.size());
    if (trajectory.size() == 0) {
      VLOG(2) << "traj idx: " << i << " has zero traj point.";
    }
    for (const auto& pt : trajectory) {
      Vec3d point(pt.path_point().x(), pt.path_point().y(), pt.relative_time());
      points.emplace_back(std::move(point));
    }
    // Draw trajectory.
    canvas.DrawLineStrip(points, vis::Color::kMagenta, 3);
    const auto& last_point = points.back();
    // Write leading object info.
    canvas.DrawText(absl::StrJoin(traj_info.leading_obj_ids, ","), last_point,
                    trajectory.front().path_point().theta() - M_PI / 2.0, 0.8,
                    vis::Color::kWhite);
  }
  vantage_client_man::FlushAll();
}

void SendGeometryGraphToCanvas(const GeometryGraph* graph,
                               const std::string& channel) {
  auto& canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto& node : graph->nodes()) {
    if (node.reachable) {
      canvas.DrawPoint(Vec3d(node.xy.x(), node.xy.y(), 0), vis::Color::kCyan,
                       7);
    } else {
      canvas.DrawPoint(Vec3d(node.xy.x(), node.xy.y(), 0), vis::Color::kOrange,
                       7);
    }

    if (!node.resampled) {
      canvas.DrawText(absl::StrCat(node.station_index, node.index.value()),
                      Vec3d(node.xy.x(), node.xy.y(), 0), 0, 0.1,
                      vis::Color::kLightRed);
    } else {
      canvas.DrawText(absl::StrCat(node.station_index, node.index.value()),
                      Vec3d(node.xy.x(), node.xy.y(), 0), 0, 0.1,
                      vis::Color::kLightYellow);
    }

    for (const auto& out_edge_index : graph->GetOutgoingEdges(node.index)) {
      const auto& edge = graph->edges()[out_edge_index];

      const auto samples = edge.geometry->Sample(0.1);
      std::vector<Vec3d> vertices;
      vertices.reserve(samples.size());
      for (const auto& sample : samples) {
        vertices.emplace_back(sample.xy.x(), sample.xy.y(), 0);
      }
      if (edge.active) {
        canvas.DrawLineStrip(vertices, vis::Color::kLightYellow, 1);
      } else {
        canvas.DrawLineStrip(vertices, vis::Color::kLightGray, 1);
      }
    }
  }
  vantage_client_man::FlushAll();
}

void SendRefSpeedTableToCanvas(const RefSpeedTable& ref_speed_table,
                               const DrivePassage& drive_passage) {
  constexpr double kSpanSampleStep = 0.5;
  const std::vector<std::string> graph_times = {"0.0s", "0.9s", "1.9s", "2.9s",
                                                "3.9s", "4.9s", "5.9s", "6.9s",
                                                "7.9s", "8.9s", "9.9s"};

  const auto& ref_speed_vecs = ref_speed_table.ref_speed_table();
  QCHECK_EQ(ref_speed_vecs.size(), graph_times.size());

  double max_speed_limit = 0.0;
  for (const auto& station : drive_passage.stations()) {
    max_speed_limit = std::max(max_speed_limit, station.speed_limit());
  }
  const vis::Colormap color_map("jet", 0.0, max_speed_limit);

  const double end_s = drive_passage.end_s();
  const int sample_num = FloorToInt(end_s / kSpanSampleStep) + 1;
  std::vector<Vec3d> points;
  points.reserve(sample_num);
  for (double sample_s = 0; sample_s < end_s; sample_s += kSpanSampleStep) {
    const Vec2d xy_pos = drive_passage.QueryPointXYAtS(sample_s).value();
    points.emplace_back(xy_pos, 0.0);
  }

  for (int i = 0; i < ref_speed_vecs.size(); ++i) {
    int sample_index_end = sample_num;
    for (int j = 0; j < sample_num; ++j) {
      const double sample_s = j * kSpanSampleStep;
      points[j].z() = ref_speed_vecs[i].FastComputeRefSpeed(sample_s);

      if (points[j].z() < 1e-6) {  // Cut off at some early stop constraint.
        sample_index_end = j;
        break;
      }
    }

    auto& canvas =
        vantage_client_man::GetCanvas("ref_speed_vecs/" + graph_times.at(i));
    canvas.SetGroundZero(1);
    for (int j = 0; j < sample_index_end - 1; ++j) {
      canvas.DrawLine(points[j], points[j + 1], color_map.Map(points[j].z()),
                      /*line_width=*/3);
    }
  }

  vantage_client_man::FlushAll();
}

void ParseMotionSearchOutputToSearchResultProto(
    const MotionSearchOutput& search_output, MotionSearchResultProto* proto) {
  proto->Clear();

  proto->set_best_last_edge_index(search_output.best_last_edge_index.value());

  search_output.motion_graph->ToProto(proto->mutable_motion_graph());

  proto->mutable_search_queue()->Reserve(search_output.search_queue.size());
  for (const auto index : search_output.search_queue) {
    proto->mutable_search_queue()->Add(index.value());
  }

  // Set cost_names.
  for (const auto& name : search_output.cost_provider->cost_names()) {
    auto* cost_name = proto->add_cost_names();
    *cost_name = name;
  }

  // Set edge costs.
  proto->mutable_edge_costs()->Reserve(search_output.motion_graph->edge_size());
  for (const auto& edge_cost : search_output.cost) {
    auto* edge_cost_proto = proto->add_edge_costs();

    edge_cost_proto->mutable_costs()->Reserve(edge_cost.feature_cost.size());
    for (const double c : edge_cost.feature_cost) {
      edge_cost_proto->add_costs(c);
    }
    QCHECK_EQ(edge_cost_proto->costs_size(), proto->cost_names_size());

    edge_cost_proto->set_cum_cost(edge_cost.cost_to_come);
  }
}

void ParseDpMotionSearchOutputToDpSearchResultProto(
    const MotionSearchOutput& search_output, DpMotionSearchResultProto* proto) {
  // proto->Clear();

  proto->set_min_cost(search_output.min_cost);

  // Get feature cost names.
  for (const auto& name : search_output.cost_provider->cost_names()) {
    auto* cost_name = proto->add_cost_names();
    *cost_name = name;
  }

  // Get considered trajectory costs (compare with Set edge costs)
  // feature costs, final costs, final progress costs
  proto->mutable_traj_costs()->Reserve(search_output.top_k_trajs.size());
  for (int i = 0, n = search_output.top_k_trajs.size(); i < n; ++i) {
    const auto& last_edge = search_output.top_k_edges[i];
    auto* traj_costs_proto = proto->add_traj_costs();
    // Feature costs
    traj_costs_proto->mutable_costs()->Reserve(
        search_output.cost_provider->cost_names().size());
    for (const auto& c : search_output.cost[last_edge].feature_cost) {
      traj_costs_proto->add_costs(c);
    }
    traj_costs_proto->set_total_cost(search_output.top_k_total_costs[i]);
    traj_costs_proto->set_last_edge_idx(last_edge.value());
  }

  // Get trajectories
  for (const auto& traj : search_output.top_k_trajs) {
    auto* traj_proto = proto->add_trajectory();
    for (const auto& point : traj) {
      auto* new_trajectory_point = traj_proto->add_trajectory_points();
      *new_trajectory_point = point;
    }
  }
}

void ParseDpMotionSearchTrajectoryWithLeadingObjToProto(
    const MotionSearchOutput& search_output, DpMotionSearchResultProto* proto) {
  const auto& multi_trajs = search_output.trajs_with_lead_obj;
  proto->mutable_traj_with_lead_obj()->Reserve(multi_trajs.size());
  for (const auto& traj : multi_trajs) {
    auto* traj_proto = proto->add_traj_with_lead_obj();
    for (const auto& point : traj.trajectory) {
      auto* new_traj_point =
          traj_proto->mutable_trajectory()->add_trajectory_points();
      *new_traj_point = point;
    }
    traj_proto->mutable_leading_obj_id()->Reserve(traj.leading_obj_ids.size());
    for (const auto& lead_obj_traj_id : traj.leading_obj_ids) {
      traj_proto->add_leading_obj_id(lead_obj_traj_id);
    }
    // Set cost.
    auto* traj_cost_proto = traj_proto->mutable_cost();
    traj_cost_proto->set_last_edge_idx(traj.last_edge_index.value());
    traj_cost_proto->set_total_cost(traj.total_cost);
    for (const auto& feature_cost : traj.feature_costs) {
      traj_cost_proto->add_costs(feature_cost);
    }
  }
}

void SendPathPointsToCanvas(const std::vector<PathPoint>& points,
                            const std::string& channel, int planner_id,
                            vis::Color color, int point_size) {
  auto& canvas =
      vantage_client_man::GetCanvas(absl::StrCat(channel, "_", planner_id));
  canvas.SetGroundZero(1);
  std::vector<Vec3d> pt_to_draw;
  pt_to_draw.reserve(points.size());
  for (const auto& pt : points) {
    Vec3d point(pt.x(), pt.y(), 0.0);
    pt_to_draw.emplace_back(std::move(point));
  }
  canvas.DrawPoints(pt_to_draw, color, point_size);
}

void ParseMotionSearchOutputToInitializerResultTrajectoryProto(
    const MotionSearchOutput& search_output,
    InitializerResultTrajectoryProto* proto) {
  proto->Clear();
  for (const auto& point : search_output.traj_points) {
    auto* new_trajectory_point = proto->add_trajectory_points();
    *new_trajectory_point = point;
  }
}

void ExportMoionSpeedProfileToChart(const MotionSearchOutput& search_output,
                                    vis::vantage::ChartDataProto* chart) {
  QCHECK_NOTNULL(chart);

  chart->set_title("initializer/speed_profile");
  auto* subchart = chart->add_subcharts();
  subchart->set_x_name("time(s)");
  auto* speed = subchart->add_y();
  speed->set_name("speed(m/s)");
  auto* a = subchart->add_y();
  a->set_name("acceleration");

  for (const auto& point : search_output.traj_points) {
    subchart->add_x_values(point.relative_time());
    speed->add_values(point.v());
    a->add_values(point.a());
  }
}

void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto) {
  // Set expert_proto.
  expert_proto->Clear();

  for (const auto& name : search_output.cost_provider->cost_names()) {
    *expert_proto->add_cost_names() = name;
  }

  expert_proto->mutable_costs()->Reserve(expert_proto->cost_names_size());
  for (const auto& c : search_output.expert_evaluation.feature_costs) {
    expert_proto->add_costs(c);
  }

  expert_proto->set_total_cost(
      search_output.expert_evaluation.weighted_total_cost);

  expert_proto->mutable_trajectory()->mutable_trajectory_points()->Reserve(
      search_output.expert_evaluation.traj.size());
  for (const auto& point : search_output.expert_evaluation.traj) {
    auto* new_trajectory_point =
        expert_proto->mutable_trajectory()->add_trajectory_points();
    *new_trajectory_point = point;
  }

  // Set candidates_proto.
  candidates_proto->Clear();

  auto candidates_cost_name = candidates_proto->mutable_cost_names();
  candidates_cost_name->CopyFrom(expert_proto->cost_names());

  // Get trajectories.
  candidates_proto->mutable_traj_costs()->Reserve(
      search_output.candidates_evaluation.size());
  candidates_proto->mutable_trajectory()->Reserve(
      search_output.candidates_evaluation.size());
  for (const auto& traj_eval : search_output.candidates_evaluation) {
    auto* traj_cost = candidates_proto->add_traj_costs();
    for (const auto& c : traj_eval.feature_costs) {
      traj_cost->add_costs(c);
    }
    traj_cost->set_total_cost(traj_eval.weighted_total_cost);

    auto* traj_proto = candidates_proto->add_trajectory();
    traj_proto->mutable_trajectory_points()->Reserve(traj_eval.traj.size());
    for (const auto& point : traj_eval.traj) {
      auto* new_trajectory_point = traj_proto->add_trajectory_points();
      *new_trajectory_point = point;
    }
  }

  candidates_proto->set_min_cost(search_output.min_cost);
}
}  // namespace qcraft::planner
