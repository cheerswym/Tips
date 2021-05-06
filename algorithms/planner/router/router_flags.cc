#include "onboard/planner/router/router_flags.h"

DEFINE_bool(random_route, false,
            "Generate a random route connecting all nearby named spots.");

// Example: destinations { name: DEST_M4_WORK } destinations { name:
// DEST_M4_HOTEL } destinations { name: DEST_M4_GARAGE_W }
DEFINE_string(route, "",
              "Proto file or text that describes the RoutingRequestProto.");

DEFINE_string(route_str, "",
              "A string with named spots separated by space as a route");

DEFINE_string(multi_stops_route, "", "Multiple stops for robobus operation.");

DEFINE_bool(route_recover_multi_stops_from_log, false,
            "Recover multiple stops route request from log.");

DEFINE_bool(route_recover_destinations_from_log, false,
            "Recover route destination request from log.");

DEFINE_bool(enable_bus_station, false, "enable/disable bus station stop area.");

DEFINE_bool(allow_routed_lane_change, true,
            "Allow lane change when searching for route to destinations");

DEFINE_string(route_default_params_file,
              "onboard/planner/router/params/route_default_params.pb.txt",
              "Default params, route map match and search cost");

DEFINE_bool(
    route_auto_next_stop, false,
    "Auto goto next stop when near stop(<kReachRouteDestinationRadius)");

DEFINE_bool(route_turn_cost_enable, false, "Add turn cost when search route.");
