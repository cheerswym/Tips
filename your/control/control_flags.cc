#include "onboard/control/control_flags.h"

#include <cmath>

DEFINE_bool(enable_openloop_control, false, "enable test control command.");
DEFINE_string(openloop_control_path, "",
              "Test control command protobuf text file path.");
// TODO(shijun): replace it with constexpr double in function after stationary
// steering satisfies.
DEFINE_double(stationary_steering_precision, 0.02,
              "stationary_steering_precision(rad)");
DEFINE_double(control_estop_jerk, -4.0,
              "m/s^3, the longitudinal jerk at control emergency stop state.");
DEFINE_double(control_estop_acceleration, -2.0,
              "m/s^2, the minimal longitudinal acceleration at control "
              "emergency stop state.");

DEFINE_double(control_max_error_warning_factor, 0.8,
              "The ratio to start send out kickout warning.");

// Values which will cause kick-out if reached.
DEFINE_double(
    control_max_station_forward_error, 3.2,
    "Control's maximum forward error in longitudinal direction in meters.");
DEFINE_double(
    control_max_station_backward_error, -5.0,
    "Control minimum backward error in longitudinal direction in meters.");
DEFINE_double(control_max_lateral_error, 0.5,
              "The maximum lateral error in meters");
DEFINE_double(control_max_heading_error, M_PI / 6.0,
              "The maximum heading error in radians.");
DEFINE_double(control_max_speed_error, 3.0,
              "The maximum speed error in meters per second.");

DEFINE_double(max_steering_angle_diff_threshold, 0.5 * M_PI_2,
              "The max steering angle diff between steering anlge cmd and "
              "steering angle feedback");
DEFINE_double(engage_protection_min_speed, 1.0,
              " m/s, only apply engage protection when speed is over it");
