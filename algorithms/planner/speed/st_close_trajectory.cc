#include "onboard/planner/speed/st_close_trajectory.h"

#include <algorithm>
#include <utility>

#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft::planner {

namespace {
using StNearestPoint = StCloseTrajectory::StNearestPoint;
std::string RecoverTrajId(const std::string& id) {
  const auto it = id.find("|");
  return it == std::string::npos ? id : id.substr(0, it);
}
}  // namespace

StCloseTrajectory::StCloseTrajectory(
    std::vector<StNearestPoint> st_nearest_points, std::string id,
    StBoundaryProto::ObjectType object_type, double probability,
    bool is_stationary)
    : st_nearest_points_(std::move(st_nearest_points)),
      id_(std::move(id)),
      traj_id_(RecoverTrajId(id_)),
      object_type_(object_type),
      probability_(probability),
      is_stationary_(is_stationary) {
  QCHECK(!st_nearest_points_.empty());
  Init();
}

std::optional<StNearestPoint> StCloseTrajectory::GetNearestPointByTime(
    double time) const {
  if (!InRange(time, min_t_, max_t_)) return std::nullopt;
  const auto it =
      std::lower_bound(st_nearest_points_.begin(), st_nearest_points_.end(),
                       time, [](const auto& pt, double t) { return pt.t < t; });
  return it == st_nearest_points_.end() ? st_nearest_points_.back() : *it;
}

void StCloseTrajectory::Init() {
  min_t_ = st_nearest_points_.front().t;
  max_t_ = st_nearest_points_.back().t;
  for (const auto& pt : st_nearest_points_) {
    min_s_ = std::min(min_s_, pt.s);
    max_s_ = std::max(max_s_, pt.s);
  }
}

}  // namespace qcraft::planner
