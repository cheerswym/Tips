#include "onboard/planner/util/trajectory_plot_util.h"

#include <string>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {

namespace {
constexpr bool kCanvasDrawTrajectoryDefaultRenderIndices = false;
constexpr double kCanvasDrawTrajectoryDefaultLineWidth = 1.0;
constexpr double kCanvasDrawTrajectoryDefaultPointSize = 4.0;
}  // namespace

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color line_color,
                          vis::Color point_color, double line_width,
                          double point_size, bool render_indices,
                          vis::Canvas *canvas) {
  canvas->SetGroundZero(1);
  canvas->DrawPoints(traj, point_color, point_size);
  canvas->DrawLineStrip(traj, line_color, line_width);
  if (render_indices) {
    for (int i = 0; i < traj.size(); ++i) {
      canvas->DrawText(absl::StrFormat("%03d", i), traj[i], 0.0, 0.1,
                       point_color);
    }
  }
  vantage_client_man::FlushAll();
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color line_color,
                          vis::Color point_color, double line_width,
                          double point_size, vis::Canvas *canvas) {
  CanvasDrawTrajectory(
      traj, line_color, point_color, line_width, point_size,
      /*render_indices=*/kCanvasDrawTrajectoryDefaultRenderIndices, canvas);
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color line_color,
                          vis::Color point_color, vis::Canvas *canvas) {
  CanvasDrawTrajectory(traj, line_color, point_color,
                       /*line_width=*/kCanvasDrawTrajectoryDefaultLineWidth,
                       /*point_size=*/kCanvasDrawTrajectoryDefaultPointSize,
                       canvas);
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color color,
                          vis::Canvas *canvas) {
  CanvasDrawTrajectory(traj, /*line_color=*/color, /*point_color=*/color,
                       canvas);
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color color,
                          const std::string &canvas_channel) {
  CanvasDrawTrajectory(traj, color,
                       &vantage_client_man::GetCanvas(canvas_channel));
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color color,
                          double line_width, double point_size,
                          bool render_indices, vis::Canvas *canvas) {
  CanvasDrawTrajectory(traj, /*line_color=*/color, /*point_color=*/color,
                       line_width, point_size, render_indices, canvas);
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color color,
                          bool render_indices, vis::Canvas *canvas) {
  CanvasDrawTrajectory(traj, color,
                       /*line_width=*/kCanvasDrawTrajectoryDefaultLineWidth,
                       /*point_size=*/kCanvasDrawTrajectoryDefaultPointSize,
                       render_indices, canvas);
}

void CanvasDrawTrajectory(const std::vector<Vec3d> &traj, vis::Color color,
                          bool render_indices,
                          const std::string &canvas_channel) {
  CanvasDrawTrajectory(traj, color, render_indices,
                       &vantage_client_man::GetCanvas(canvas_channel));
}

void CanvasDrawTrajectory(const std::vector<ApolloTrajectoryPointProto> &traj,
                          const std::string &channel) {
  std::vector<Vec3d> points;
  points.reserve(traj.size());
  for (const auto &p : traj) {
    points.emplace_back(p.path_point().x(), p.path_point().y(), 0.1);
  }
  CanvasDrawTrajectory(points, vis::Color::kGreen, /*render_indices*/ false,
                       &vantage_client_man::GetCanvas(channel));
}

std::vector<Vec3d> VisVectorTrajToVector(const std::vector<Vec2d> &traj,
                                         double z_inc, double z0) {
  std::vector<Vec3d> traj3d(traj.size());
  for (int i = 0; i < traj.size(); ++i) {
    traj3d[i] = Vec3d(traj[i], z0 + i * z_inc);
  }
  return traj3d;
}

std::vector<Vec3d> VisVectorTrajToVector(const std::vector<Vec2d> &traj) {
  return VisVectorTrajToVector(
      traj,
      /*z_inc=*/kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep,
      /*z0=*/0.0);
}

std::vector<Vec3d> VisTimeTrajToVector(const std::function<Vec2d(double)> &traj,
                                       double horizon, double z_inc,
                                       double z0) {
  std::vector<Vec3d> traj3d(RoundToInt(horizon / kTrajectoryTimeStep));
  for (int i = 0; i < traj3d.size(); ++i) {
    traj3d[i] = Vec3d(traj(i * kTrajectoryTimeStep), z0 + i * z_inc);
  }
  return traj3d;
}

std::vector<Vec3d> VisTimeTrajToVector(const std::function<Vec2d(double)> &traj,
                                       double horizon) {
  return VisTimeTrajToVector(
      traj, horizon,
      /*z_inc=*/kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep,
      /*z0=*/0.0);
}

std::vector<Vec3d> VisIndexTrajToVector(const std::function<Vec2d(int)> &traj,
                                        int horizon, double z_inc, double z0) {
  std::vector<Vec3d> traj3d(horizon);
  for (int i = 0; i < traj3d.size(); ++i) {
    traj3d[i] = Vec3d(traj(i), z0 + i * z_inc);
  }
  return traj3d;
}

std::vector<Vec3d> VisIndexTrajToVector(const std::function<Vec2d(int)> &traj,
                                        int horizon) {
  return VisIndexTrajToVector(
      traj, horizon,
      /*z_inc=*/kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep,
      /*z0=*/0.0);
}

void CanvasDrawTrajectories(const std::string &base_name,
                            const std::vector<TrajectoryPlotInfo> &trajs) {
  for (const auto &traj : trajs) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&traj](int index) { return traj.traj[index].pos(); },
            traj.traj.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        traj.color, /*render_indices=*/true, base_name + "/" + traj.name);
  }
}

}  // namespace planner
}  // namespace qcraft
