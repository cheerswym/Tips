#include "onboard/planner/router/route_error.h"

#include "absl/strings/str_cat.h"

namespace qcraft::planner::route {

std::string RouteErrorCode::ToString() const {
  return absl::StrCat("code:", code_, ", action:", action_,
                      ", message:", message_);
}

RouteErrorCode OkRouteStatus() {
  return RouteErrorCode(RouteErrorCode::StatusCode::kOk, "", "");
}

RouteErrorCode RoutePathNotFoundError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kRoutePathNotFound,
                        "RoutePathNotFound", message);
}

RouteErrorCode RouteCreateLanePathFailedError(std::string_view message) {
  return RouteErrorCode(
      RouteErrorCode::StatusCode::kRouteCreateLanePathFailedError,
      "CreateLanePathFailed", message);
}

RouteErrorCode RerouteError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kReroute, "reroute",
                        message);
}

RouteErrorCode MapMatchError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kMapMatch, "map_match",
                        message);
}

}  // namespace qcraft::planner::route
