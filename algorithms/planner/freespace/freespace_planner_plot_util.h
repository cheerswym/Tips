#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_

#include <string>
#include <string_view>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/planner/freespace/path_manager_util.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {
void DrawDirectionalPath(std::string_view name,
                         absl::Span<const DirectionalPath> paths);

void DrawPathSweptVolume(std::string_view name,
                         absl::Span<const Box2d> path_swept_volume);

void DrawParkingInfos(const AABox2d &map_aabox, const PathPoint &goal);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_
