#ifndef ONBOARD_CONTROL_CONTROL_DEFS_H_
#define ONBOARD_CONTROL_CONTROL_DEFS_H_

#include <math.h>

#include "onboard/math/util.h"

namespace qcraft::control {

constexpr double kControlInterval = 0.01;               // s.
constexpr double kGravitationalAcceleration = 9.80665;  // m/s^2.
constexpr double kTrajPointInterval = 0.1;              // s.
constexpr int kMaxPastPointNum = 50;                    // 5s.
constexpr int kTrajectorySteps = 100;                   // 10s.
constexpr double kPoseInterval = 0.01;                  // s.
constexpr double kSinSlopeLimit = 0.2588;     // slope angle limit, 15 degrees.
constexpr double kMaxAccelerationCmd = 2.0;   // m/s^2.
constexpr double kMaxDecelerationCmd = -6.0;  // m/s^2.
constexpr int kTControlHorizon = 10;
constexpr int kSControlHorizon = 10;

/*
 * Steering protection params.
 */
constexpr double kLateralkJerkLimitLow = 3.5;   // m/s^3
constexpr double kLateralkJerkLimitHigh = 4.0;  // m/s^3
// TODO(shijun): change it to 300deg/s. Now it is same as before.
constexpr double kSteeringSpeedLimit = d2r(360.0);  // rad/s.
constexpr double kRelaxFactorThreshold = 0.9;

constexpr double kStopSpeed = 0.01;  // m/s.
constexpr double kFullStopBrakeCmdIntegral = kControlInterval * 1.0;
}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROL_DEFS_H_
