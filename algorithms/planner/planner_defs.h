#ifndef ONBOARD_PLANNER_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_PLANNER_DEFS_H_

#include "absl/time/time.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

// Run planner main loop at 10Hz.
constexpr absl::Duration kPlannerMainLoopInterval = absl::Milliseconds(100);

constexpr double kTrajectoryTimeStep = 0.1;  // 100ms.
constexpr int kTrajectorySteps = 100;        // 10s.
constexpr double kTrajectoryTimeHorizon =
    (kTrajectorySteps - 1) * kTrajectoryTimeStep;  // from 0.0s to 9.9s.
constexpr double kSpeedSampleStep = 0.5;           // m/s
constexpr int kTrajResampleNum = 100;

constexpr double kSpacetimePlannerTrajectoryHorizon =
    kTrajectoryTimeStep * kTrajectorySteps;

constexpr double kDdpTrajectoryTimeStep = 0.2;  // 200ms.
constexpr int kDdpTrajectorySteps = 75;         // 15s.
constexpr double kPlanningTimeHorizon =
    kDdpTrajectoryTimeStep * kDdpTrajectorySteps + 1.0;

constexpr int kSpeedFinderMaxTrajectorySteps = 150;

constexpr int kMaxPastPointNum = 50;

constexpr double kSpaceTimeVisualizationDefaultTimeScale = 10.0;  // m/s.

constexpr double kJoptPlannerMinSpeed = 1e-3;  // m/s.

constexpr double kMinLCSpeed = 5.0 / 3.6;  // m/s.
constexpr double kMinLcLaneLength = 20.0;  // m.  Planner 3.0

constexpr double kDefaultLaneWidth = 3.5;  // m.
constexpr double kDefaultHalfLaneWidth = 0.5 * kDefaultLaneWidth;
constexpr double kMaxHalfLaneWidth = 2.7;  // m.
constexpr double kMinLaneWidth = 2.6;      // m.

// Desired lateral distance for nudge.
constexpr double kMaxLateralOffset = 10.0;         // m.
constexpr double kMaxLaneKeepLateralOffset = 0.4;  // m.

constexpr double kRouteStationUnitStep = 1.0;  // m.

constexpr double kDrivePassageKeepBehindLength = 10.0;  // m.

constexpr double kMaxTravelDistanceBetweenFrames = 20.0;  // m.

constexpr double kInitializerMinFollowDistance = 3.0;

// Local map extension length over the planning horizon.
constexpr double kLocalMapExtension = 100.0;  // m.

// Trajectory time length with curvature constraint
constexpr double kCurvatureLimitRange = 4.0;

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_DEFS_H_
