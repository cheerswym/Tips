#include "onboard/planner/freespace/freespace_planner_plot_util.h"

#include <algorithm>
#include <utility>

#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {
void DrawDirectionalPath(std::string_view name,
                         absl::Span<const DirectionalPath> paths) {
  vis::Canvas &canvas = vantage_client_man::GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto &path : paths) {
    for (const auto &point : path.path) {
      if (path.forward) {
        canvas.DrawCircle(Vec3d(point.x(), point.y(), 0.0), /*size*/ 0.2,
                          vis::Color::kLightGreen);
      } else {
        canvas.DrawCircle(Vec3d(point.x(), point.y(), 0.0), /*size*/ 0.2,
                          vis::Color::kLightBlue);
      }
    }
  }
  vantage_client_man::FlushAll();
}

void DrawPathSweptVolume(std::string_view name,
                         absl::Span<const Box2d> path_swept_volume) {
  vis::Canvas &canvas = vantage_client_man::GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto &box : path_swept_volume) {
    canvas.DrawBox(Vec3d(box.center(), 0.0), box.heading(),
                   Vec2d(box.length(), box.width()), vis::Color::kLightYellow);
  }
  vantage_client_man::FlushAll();
}

void DrawParkingInfos(const AABox2d &map_aabox, const PathPoint &goal) {
  vis::Canvas &canvas = vantage_client_man::GetCanvas("freespace");
  Box2d box(map_aabox);
  canvas.DrawBox(Vec3d(box.center(), 0.0), box.heading(),
                 Vec2d(box.width(), box.length()), vis::Color::kYellow);
  Vec2d goal_pos(goal.x(), goal.y());
  canvas.DrawPoint(Vec3d(goal_pos, 0.0), vis::Color::kYellow, 10);
  canvas.DrawLine(Vec3d(goal_pos, 0.0),
                  Vec3d(goal_pos + Vec2d::FastUnitFromAngle(goal.theta()), 0.0),
                  vis::Color::kYellow);
}

}  // namespace planner
}  // namespace qcraft
