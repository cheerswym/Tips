#pragma once

#include "gflags/gflags.h"

DECLARE_bool(random_route);

// Example: destinations { name: DEST_M4_WORK } destinations { name:
// DEST_M4_HOTEL } destinations { name: DEST_M4_GARAGE_W }
DECLARE_string(route);

DECLARE_string(route_str);

DECLARE_string(multi_stops_route);

DECLARE_bool(route_recover_multi_stops_from_log);

DECLARE_bool(route_recover_destinations_from_log);

DECLARE_bool(enable_bus_station);

DECLARE_bool(allow_routed_lane_change);

// The default route params file path
DECLARE_string(route_default_params_file);

DECLARE_bool(route_auto_next_stop);

DECLARE_bool(route_turn_cost_enable);
