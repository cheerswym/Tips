#include "onboard/planner/object/plot_util.h"

#include <algorithm>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

namespace qcraft::planner {
constexpr double kEps = 1e-6;

void DrawBox2dToCanvas(vis::Canvas *canvas, const Box2d &box,
                       vis::Color color) {
  canvas->DrawBox(Vec3d(box.center(), 0.0), box.heading(),
                  Vec2d(box.length(), box.width()), color, 3);
  const Vec2d front_center = box.center() + box.half_length() * box.tangent();
  canvas->DrawLine(
      Vec3d(front_center, 0.0),
      Vec3d(front_center + 0.15 * box.length() * box.tangent(), 0.0), color, 3);
}

void DrawPlannerObjectManagerToCanvas(const PlannerObjectManager &object_mgr,
                                      const std::string &channel,
                                      vis::Color color) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);

  for (const auto &object : object_mgr.planner_objects()) {
    const auto &bbox = object.bounding_box();
    DrawBox2dToCanvas(&canvas, bbox, color);
  }

  vantage_client_man::FlushAll();
}

void DrawPredictionToCanvas(const ObjectsPredictionProto &prediction,
                            const std::string &channel, vis::Color color) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);

  for (const auto &object_pred : prediction.objects()) {
    const auto &bbox_proto = object_pred.perception_object().bounding_box();
    DrawBox2dToCanvas(&canvas, Box2d(bbox_proto), color);
  }

  vantage_client_man::FlushAll();
}

void DrawSpacetimeObjectTrajectory(const SpacetimeObjectTrajectory &st_obj_traj,
                                   const std::string &channel,
                                   vis::Color color) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto &state : st_obj_traj.states()) {
    canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0, color);
  }
  vantage_client_man::FlushAll();
}

void DrawStTrajWithColorableAccel(const SpacetimeObjectTrajectory &st_obj_traj,
                                  const std::string &channel) {
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (const auto &state : st_obj_traj.states()) {
    if (std::abs(state.traj_point->a()) < kEps) {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kAzure);
    } else if (state.traj_point->a() > kEps) {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kDarkGreen);
    } else {
      canvas.DrawPolygon(state.contour, state.traj_point->t() * 10.0,
                         vis::Color::kCrimson);
    }
  }
  vantage_client_man::FlushAll();
}

void ClearCanvasServerBuffers() { vantage_client_man::ClearServerBuffers(); }

}  // namespace qcraft::planner
