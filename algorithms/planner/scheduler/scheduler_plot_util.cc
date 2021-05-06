#include "onboard/planner/scheduler/scheduler_plot_util.h"

#include <algorithm>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/maps/semantic_map_util.h"

namespace qcraft::planner {

void SendLaneGraphToCanvas(const LaneGraph &lane_graph,
                           const SemanticMapManager &semantic_map_manager,
                           const RouteSectionsInfo &sections_info,
                           const std::string &topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(topic);
  canvas.SetGroundZero(1);

  const auto &layers = lane_graph.layers;
  const auto &sections = sections_info.section_segments();
  absl::flat_hash_map<BasicGraph::VertexId, Vec2d> vertex_pos;

  for (int i = 0; i < layers.size(); ++i) {
    const auto [sec_idx, frac] = layers[i];
    for (const auto lane_id : sections[sec_idx].lane_ids) {
      const Vec2d pos =
          mapping::LanePoint(lane_id, frac).ComputePos(semantic_map_manager);
      canvas.DrawPoint(Vec3d(pos, 0.1), vis::Color::kLightBlue, 8);
      vertex_pos[absl::StrFormat("%d-%d", i, lane_id)] = pos;
    }
  }

  const auto graph = lane_graph.graph;
  for (const auto &vertex : graph.vertices()) {
    if (!vertex_pos.contains(vertex)) continue;
    for (const auto &[to_id, cost] : graph.edges_from(vertex)) {
      if (!vertex_pos.contains(to_id)) continue;

      const auto &from_pos = vertex_pos[vertex], to_pos = vertex_pos[to_id];
      canvas.DrawLine(Vec3d(from_pos, 0.1), Vec3d(to_pos, 0.1),
                      vis::Color::kLightCyan, 3);
      canvas.DrawText(absl::StrFormat("%.3f", cost),
                      Vec3d(Lerp(from_pos, to_pos, 0.15), 0.1),
                      (to_pos - from_pos).normalized().FastAngle(), 0.2,
                      vis::Color::kLightRed);
    }
  }

  vantage_client_man::FlushAll();
}

void SendLanePathInfoToCanvas(const LanePathInfo &lp_info,
                              const SemanticMapManager &semantic_map_manager,
                              const std::string &topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(topic);
  canvas.SetGroundZero(1);

  const auto &lp = lp_info.lane_path();
  const auto points = mapping::SampleLanePathPoints(semantic_map_manager, lp);
  std::vector<Vec3d> points_3d(points.size());
  std::transform(points.begin(), points.end(), points_3d.begin(),
                 [](Vec2d p) { return Vec3d(p, 1.0); });
  canvas.DrawLineStrip(points_3d, vis::Color::kMint, 3);

  const Vec2d max_len_pt = lp.ArclengthToPos(lp_info.length_along_route());
  canvas.DrawCircle(Vec3d(max_len_pt, 1.0), 0.3, vis::Color::kLightMagenta,
                    vis::Color::kLightMagenta);

  vantage_client_man::FlushAll();
}

void SendLocalMapToCanvas(const std::vector<mapping::LanePath> &lane_paths,
                          const SemanticMapManager &semantic_map_manager,
                          const std::string &topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(topic);
  canvas.SetGroundZero(1);

  for (const auto &lane_path : lane_paths) {
    const auto points =
        mapping::SampleLanePathPoints(semantic_map_manager, lane_path);
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](Vec2d p) { return Vec3d(p, 1.0); });
    canvas.DrawLineStrip(points_3d, vis::Color::kSkyBlue, 3);

    double accumulate_length = 0.0;
    for (const auto &seg : lane_path) {
      accumulate_length += seg.length();
      const Vec2d seg_end_pt = lane_path.ArclengthToPos(accumulate_length);
      canvas.DrawCircle(Vec3d(seg_end_pt, 1.0), 0.3, vis::Color::kMint,
                        vis::Color::kMint);
    }
  }
  vantage_client_man::FlushAll();
}

void SendAvSlTrajectoryToCanvas(
    const DrivePassage &drive_passage,
    const PiecewiseLinearFunction<double, double> &traj_l_s,
    const std::string &topic) {
  QCHECK(!topic.empty()) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(topic);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < traj_l_s.x().size(); ++i) {
    const auto xy =
        drive_passage.QueryPointXYAtSL(traj_l_s.x()[i], traj_l_s.y()[i])
            .value();
    const auto xy_next =
        drive_passage.QueryPointXYAtSL(traj_l_s.x()[i + 1], traj_l_s.y()[i + 1])
            .value();
    canvas.DrawLine(Vec3d(xy, 0.1), Vec3d(xy_next, 0.1), vis::Color::kRed, 3);
    canvas.DrawPoint(Vec3d(xy, 0.1), vis::Color::kRed, 5);
  }

  vantage_client_man::FlushAll();
}

}  // namespace qcraft::planner
