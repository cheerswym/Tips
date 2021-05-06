#include "onboard/planner/speed/decider/st_boundary_modifier_util.h"

#include <memory>
#include <utility>

#include "onboard/planner/trajectory_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1e-6;

std::vector<prediction::PredictedTrajectoryPoint>
GenerateUniformDecelPredTrajPointsAfterDelay(
    const std::vector<prediction::PredictedTrajectoryPoint>& pred_traj_points,
    double delay, double decel) {
  QCHECK(!pred_traj_points.empty());
  QCHECK_LE(decel, 0.0);
  std::vector<prediction::PredictedTrajectoryPoint> decel_traj_points;
  decel_traj_points.reserve(pred_traj_points.size());
  const double t_step = pred_traj_points.size() > 1
                            ? pred_traj_points[1].t() - pred_traj_points[0].t()
                            : prediction::kPredictionTimeStep;
  decel_traj_points.push_back(pred_traj_points.front());
  for (int i = 1; i < pred_traj_points.size(); ++i) {
    const double curr_t = i * t_step;
    const auto& traj_pt = pred_traj_points[i];
    if (curr_t < delay) {
      decel_traj_points.push_back(traj_pt);
      continue;
    }
    prediction::PredictedTrajectoryPoint next_traj_pt;
    auto& prev_traj_pt = decel_traj_points.back();
    if (prev_traj_pt.v() < kEps) {
      next_traj_pt = prev_traj_pt;
      next_traj_pt.set_t(curr_t);
      next_traj_pt.set_v(0.0);
      next_traj_pt.set_a(0.0);
      decel_traj_points.push_back(std::move(next_traj_pt));
      continue;
    }
    const double init_v = prev_traj_pt.v();
    const double init_s = prev_traj_pt.s();
    const double curr_v = std::max(0.0, init_v + decel * t_step);
    const double curr_s =
        init_s + std::max(0.0, init_v * t_step + 0.5 * decel * Sqr(t_step));
    next_traj_pt = QueryTrajectoryPointByS(pred_traj_points, curr_s);
    next_traj_pt.set_t(curr_t);
    next_traj_pt.set_v(curr_v);
    prev_traj_pt.set_a((curr_v - prev_traj_pt.v()) / t_step);
    decel_traj_points.push_back(std::move(next_traj_pt));
  }
  return decel_traj_points;
}

}  // namespace

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryByDecelAfterDelay(
    const SpacetimeObjectTrajectory& st_traj, double delay, double decel) {
  auto decel_pred_traj = GenerateUniformDecelPredTrajPointsAfterDelay(
      st_traj.trajectory()->points(), delay, decel);
  auto new_pred_traj = *st_traj.trajectory();
  *new_pred_traj.mutable_points() = std::move(decel_pred_traj);
  auto new_pred_traj_ptr =
      std::make_unique<const prediction::PredictedTrajectory>(
          std::move(new_pred_traj));
  return st_traj.CreateTrajectoryMutatedInstance(std::move(new_pred_traj_ptr));
}

}  // namespace planner
}  // namespace qcraft
