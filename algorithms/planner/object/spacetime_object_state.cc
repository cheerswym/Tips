#include "onboard/planner/object/spacetime_object_state.h"

#include "onboard/planner/planner_defs.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {

std::vector<SpacetimeObjectState> SampleTrajectoryStates(
    const prediction::PredictedTrajectory& pred_traj, Vec2d init_pos,
    const Polygon2d& init_contour, const Box2d& init_box,
    const Box2d& init_perception_box) {
  const auto& traj_points = pred_traj.points();
  if (traj_points.empty()) return {};

  const int num_traj_points = traj_points.size();

  if (prediction::IsStationaryTrajectory(pred_traj)) {
    QCHECK_GE(num_traj_points, 1);
    // The information in `traj_point` may be inconsistent with box and
    // perception box.
    const SpacetimeObjectState obj_state{.traj_point = &traj_points[0],
                                         .box = init_box,
                                         .perception_box = init_perception_box,
                                         .contour = init_contour};
    std::vector<SpacetimeObjectState> states;
    states.reserve(kTrajectorySteps);
    while (states.size() < kTrajectorySteps) {
      states.push_back(obj_state);
    }
    return states;
  }

  QCHECK_GT(traj_points.size(), 0);

  // Note that besides transforming the object contour polygon, we also need to
  // transform the object's bounding box center as box's center may not be at
  // the object's pos.
  std::vector<SpacetimeObjectState> states;
  states.reserve(num_traj_points);
  const Vec2d box_pos_shift = init_box.center() - init_pos;
  const Vec2d perception_box_pos_shift =
      init_perception_box.center() - init_pos;
  for (int i = 0; i < num_traj_points; ++i) {
    SpacetimeObjectState& obj_state = states.emplace_back();
    const auto& pt = traj_points[i];
    obj_state.traj_point = &pt;
    const Vec2d rotation =
        Vec2d::FastUnitFromAngle(pt.theta() - init_box.heading());
    obj_state.contour = init_contour.Transform(
        init_pos, rotation.x(), rotation.y(), pt.pos() - init_pos);

    obj_state.box = Box2d(pt.pos() + box_pos_shift, pt.theta(),
                          init_box.length(), init_box.width());
    obj_state.perception_box =
        Box2d(pt.pos() + perception_box_pos_shift, pt.theta(),
              init_perception_box.length(), init_perception_box.width());
  }
  return states;
}

}  // namespace planner
}  // namespace qcraft
