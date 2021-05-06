#include "onboard/planner/common/plot_util.h"

#include <algorithm>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/maps/semantic_map_util.h"

namespace qcraft::planner {

void SendPointsToCanvas(absl::Span<const Vec2d> points,
                        const std::string &channel, vis::Color color) {
  VLOG(3) << "draw vector of points...";
  QCHECK(channel.length() > 0) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto &pt : points) {
    canvas.DrawPoint(Vec3d(pt, 0.1), color, 6);
  }

  vantage_client_man::FlushAll();
}

void SendApolloTrajectoryPointsToCanvas(
    absl::Span<const ApolloTrajectoryPointProto> traj_pts,
    const std::string &channel, vis::Color color) {
  std::vector<Vec2d> points;
  points.reserve(traj_pts.size());

  for (const auto &traj_pt : traj_pts) {
    points.push_back(Vec2dFromApolloTrajectoryPointProto(traj_pt));
  }

  SendPointsToCanvas(points, channel, color);
}

void SendTrajectoryPointsToCanvas(
    const ::google::protobuf::RepeatedPtrField< ::qcraft::TrajectoryPointProto>
        &traj_pts,
    const std::string &channel, vis::Color color) {
  std::vector<Vec2d> points;
  points.reserve(traj_pts.size());

  for (const auto &traj_pt : traj_pts) {
    points.push_back(Vec2d(traj_pt.pos().x(), traj_pt.pos().y()));
  }

  SendPointsToCanvas(points, channel, color);
}

void DrawPathSlBoundaryToCanvas(const PathSlBoundary &path_bound,
                                const std::string &channel) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < path_bound.size(); ++i) {
    canvas.DrawLine(Vec3d(path_bound.reference_center_xy_vector()[i], 0.1),
                    Vec3d(path_bound.reference_center_xy_vector()[i + 1], 0.1),
                    vis::Color::kOrange, 5);

    canvas.DrawLine(Vec3d(path_bound.right_xy_vector()[i], 0.1),
                    Vec3d(path_bound.right_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 4);

    canvas.DrawLine(Vec3d(path_bound.target_right_xy_vector()[i], 0.1),
                    Vec3d(path_bound.target_right_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 3, vis::BorderStyleProto::DASHED);

    canvas.DrawLine(Vec3d(path_bound.left_xy_vector()[i], 0.1),
                    Vec3d(path_bound.left_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 4);

    canvas.DrawLine(Vec3d(path_bound.target_left_xy_vector()[i], 0.1),
                    Vec3d(path_bound.target_left_xy_vector()[i + 1], 0.1),
                    vis::Color::kLightCyan, 3, vis::BorderStyleProto::DASHED);
  }

  vantage_client_man::FlushAll();
}

void DrawLanePathToCanvas(const SemanticMapManager &semantic_map_manager,
                          const mapping::LanePath &lp,
                          const std::string &channel, vis::Color color) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  const auto points = SampleLanePathPoints(semantic_map_manager, lp);
  std::vector<Vec3d> points_3d(points.size());
  std::transform(points.begin(), points.end(), points_3d.begin(),
                 [](Vec2d p) { return Vec3d(p, 0.1); });
  canvas.DrawLineStrip(points_3d, color, 3);
  vantage_client_man::FlushAll();
}

}  // namespace qcraft::planner
