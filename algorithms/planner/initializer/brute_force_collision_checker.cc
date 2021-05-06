#include "onboard/planner/initializer/brute_force_collision_checker.h"

#include <vector>

namespace qcraft {
namespace planner {

namespace {
template <typename StateType>
std::vector<Box2d> StatesToBoxes(absl::Span<const StateType> states,
                                 const VehicleGeometryParamsProto* params) {
  std::vector<Box2d> boxes;
  boxes.reserve(states.size());
  const double ra_to_center =
      params->front_edge_to_center() - 0.5 * params->length();
  for (const auto& state : states) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(state.h);
    const Vec2d center = state.xy + tangent * ra_to_center;
    boxes.push_back(Box2d(center, tangent, params->length(), params->width()));
  }
  return boxes;
}

std::vector<GeometryState> MotionStatesToGeomStates(
    absl::Span<const MotionState> states) {
  std::vector<GeometryState> geom_states;
  double accum_s = 0.0;
  for (const auto& s : states) {
    if (!geom_states.empty()) {
      accum_s += geom_states.back().xy.DistanceTo(s.xy);
    }
    geom_states.push_back(GeometryState{
        .xy = s.xy, .h = s.h, .accumulated_s = accum_s, .l = 0.0});
  }
  return geom_states;
}

void CheckStationaryObjectCollision(
    int sample_step, absl::Span<const GeometryState> states,
    absl::Span<const SpacetimeObjectTrajectory* const> stationary_trajs,
    const VehicleGeometryParamsProto* vehicle_geom, double buffer,
    CollisionInfo* info) {
  const auto av_boxes = StatesToBoxes(states, vehicle_geom);
  const double extend_len = buffer * 2.0;
  bool has_collision = false;
  for (const auto* traj : stationary_trajs) {
    auto obj_contour = traj->planner_object()->contour();
    obj_contour = obj_contour.ExpandByDistance(extend_len);
    for (int i = 0; i < av_boxes.size(); ++i) {
      if (obj_contour.HasOverlap(av_boxes[i])) {
        info->collision_objects.push_back(
            {.time = i * sample_step, .traj = traj});
        has_collision = true;
      }
      if (has_collision) break;
    }
  }
}
}  // namespace

BruteForceCollisionChecker::BruteForceCollisionChecker(
    const SpacetimeTrajectoryManager* st_traj_mgr,
    const VehicleGeometryParamsProto* vehicle_geom, int sample_step,
    double stationary_buffer, double moving_buffer)
    : vehicle_geom_(vehicle_geom),
      sample_step_(sample_step),
      stationary_buffer_(stationary_buffer),
      moving_buffer_(moving_buffer) {
  QCHECK_GE(sample_step, 1);
  for (const auto& traj : st_traj_mgr->spacetime_planner_trajs()) {
    if (traj.is_stationary()) {
      considered_stationary_trajs_.push_back(&traj);
    } else {
      considered_moving_trajs_.push_back(&traj);
    }
  }
  // Sample the prediction trajectories.
  sampled_trajs_ = SampleTrajectory(considered_moving_trajs_, sample_step_);
}
void BruteForceCollisionChecker::UpdateStationaryObjectBuffer(
    double stationary_buffer) {
  stationary_buffer_ = stationary_buffer;
}
void BruteForceCollisionChecker::UpdateMovingObjectBuffer(
    double moving_buffer) {
  moving_buffer_ = moving_buffer;
}
void BruteForceCollisionChecker::CheckCollision(
    double init_t, absl::Span<const MotionState> states,
    CollisionInfo* info) const {
  CheckCollisionWithTrajectories(init_t, states, info);
  const std::vector<GeometryState> geom_states =
      MotionStatesToGeomStates(states);
  CheckStationaryObjectCollision(sample_step_, geom_states,
                                 considered_stationary_trajs_, vehicle_geom_,
                                 stationary_buffer_, info);
}

void BruteForceCollisionChecker::CheckCollisionWithTrajectories(
    double init_t, absl::Span<const MotionState> states,
    CollisionInfo* info) const {
  const double extend_len = moving_buffer_ * 2.0;
  const auto av_boxes = StatesToBoxes(states, vehicle_geom_);
  const int start_ix =
      RoundToInt(init_t / (sample_step_ * prediction::kPredictionTimeStep));
  bool has_collision = false;
  for (const auto& sampled_traj : sampled_trajs_) {
    const auto& obj_states = sampled_traj.states;
    for (int i = 0, t = start_ix; i < states.size() && t < obj_states.size();
         ++i, ++t) {
      auto obj_box = obj_states[t].box;
      obj_box.LongitudinalExtend(extend_len);
      obj_box.LateralExtend(extend_len);
      if (av_boxes[i].HasOverlap(obj_box)) {
        info->collision_objects.push_back(
            {.time = t * sample_step_, .traj = sampled_traj.traj});
        has_collision = true;
      }
      if (has_collision) break;
    }
  }
}

void BruteForceCollisionChecker::CheckCollisionWithStationaryObjects(
    absl::Span<const GeometryState> states, CollisionInfo* info) const {
  CheckStationaryObjectCollision(/*sample_step=*/1, states,
                                 considered_stationary_trajs_, vehicle_geom_,
                                 stationary_buffer_, info);
}

}  // namespace planner
}  // namespace qcraft
