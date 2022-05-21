#ifndef ONBOARD_CONTROL_CONTROL_FLAGS_H_
#define ONBOARD_CONTROL_CONTROL_FLAGS_H_

#include "gflags/gflags.h"

DECLARE_bool(enable_openloop_control);
DECLARE_string(openloop_control_path);
DECLARE_double(stationary_steering_precision);
DECLARE_double(control_estop_jerk);
DECLARE_double(control_estop_acceleration);

DECLARE_double(control_max_error_warning_factor);

// Kickout thresholds based on control error.
DECLARE_double(control_max_station_forward_error);
DECLARE_double(control_max_station_backward_error);
DECLARE_double(control_max_lateral_error);
DECLARE_double(control_max_heading_error);
DECLARE_double(control_max_speed_error);

DECLARE_double(max_steering_angle_diff_threshold);
DECLARE_double(engage_protection_min_speed);

#endif  // ONBOARD_CONTROL_CONTROL_FLAGS_H_
