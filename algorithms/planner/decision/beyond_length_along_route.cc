#include "onboard/planner/decision/beyond_length_along_route.h"

#include <algorithm>
#include <limits>
#include <string>

#include "onboard/planner/planner_defs.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
constexpr double kMaxLCSpeedOnBorrow = 15.0 / 3.6;  // m/s.
}

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildBeyondLengthAlongRouteConstraint(const DrivePassage &dp,
                                      double length_along_route,
                                      bool borrow_boundary) {
  if (length_along_route == std::numeric_limits<double>::max()) {
    return absl::NotFoundError("Length along route not available.");
  }
  const double length_along_route_on_dp =
      length_along_route + dp.lane_path_start_s();

  constexpr double kEpsilon = 1.0;  // m.
  if (length_along_route_on_dp > dp.end_s() - kEpsilon) {
    return absl::NotFoundError("Target lane path reaches destination.");
  }

  ConstraintProto::SpeedRegionProto proto;
  proto.set_start_s(std::max(0.0, length_along_route_on_dp));
  proto.set_end_s(dp.end_s());

  ASSIGN_OR_RETURN(const auto start_pt, dp.QueryPointXYAtS(proto.start_s()));
  start_pt.ToProto(proto.mutable_start_point());

  ASSIGN_OR_RETURN(const auto end_pt, dp.QueryPointXYAtS(proto.end_s()));
  end_pt.ToProto(proto.mutable_end_point());

  const double speed_limit =
      borrow_boundary ? kMaxLCSpeedOnBorrow : kMinLCSpeed;
  proto.set_min_speed(kMinLCSpeed);
  proto.set_max_speed(speed_limit);

  const std::string id = "beyond_length_along_route";
  proto.set_id(id);
  proto.mutable_source()->mutable_beyond_length_along_route()->set_id(id);

  return proto;
}

}  // namespace qcraft::planner
