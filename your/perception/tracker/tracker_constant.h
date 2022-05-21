#ifndef ONBOARD_PERCEPTION_TRACKER_TRACKER_CONSTANT_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACKER_CONSTANT_H_

#include "onboard/math/vec.h"

namespace qcraft::tracker {

// The maximum duration of a track without any update. That is, a track can last
// this time before receiving any update.

// NOTE(zheng): 0.3s duration is ok for onroad/offroad track, but when the
// perception(vision, radar) module is running at a very low frequency(
// the lowest frequency is 3 hz), 0.3s would cause some wrong delete of tracks.
// so we set the duration to 0.35s
inline constexpr double kMaxOnroadTrackLifeWithoutUpdate = 0.35;        // sec
inline constexpr double kMaxCertainStaticTrackLifeWithoutUpdate = 0.1;  // sec
inline constexpr double kMaxOffroadTrackLifeWithoutUpdate = 0.35;       // sec

inline constexpr double kTrackLifeTimeBufferForDifferentSensorType =
    0.5;                                                              // sec
inline constexpr double kTrackLifeTimeBufferForSameSensorType = 0.0;  // sec
// The minimum time in which the track is updated by at least
// kNumUpdatesToPromote times before this track is promoted.
inline constexpr double kTrackPromoteMinTime = 0.3;
// The maximum distance between matched track and measurement.
inline constexpr double kOnroadMaxMatchDistPerSec = 30.0;   // m/s
inline constexpr double kOffroadMaxMatchDistPerSec = 20.0;  // m/s
// A vehicle is classified as parked if at least this ratio of its
// contour/bounding box area is in a parking area.
inline constexpr double kParkingAreaOverlapMinRatio = 0.3;
inline constexpr double kIgnoranceZoneOverlapMinRatio = 0.8;
inline constexpr double kStaticObjectZoneOverlapMinRatio = 0.8;
// A track with unknown type  is classified as static if at least this ratio of
// its contour/bounding box area is in a vegetation zone.
inline constexpr double kVegetationZoneOverlapMinRatio = 0.7;
// The maximum speed under which a car may be classified as parked if it is in a
// parking area.
inline constexpr double kParkedCarMaxSpeed = 1.0;
inline constexpr double kStaticObjectMaxSpeed = 1.0;
// Ped: Ped with objects could be as large as 2.5m x 1m -> 65 obstacles.
// Xcyclist: Triccylist could be as large as 2m x 4m -> 200 obstacles.
// This threshold should be a conservative one with high precision,
// So we should loosen the threshold to a higher value [80, 300].
inline constexpr int kPedMaxObstacles = 80;
inline constexpr int kCyclistMaxObstacles = 300;

const Vec2d kUpsTruckSize(8.0, 2.5);
const Vec2d kBusSize(12.5, 3.0);

// Minimum heading jump to be considered as detection fault
inline constexpr double kCarHeadingFlipMinRad = d2r(150.0);

// Discounted smoothing factor to average tracked object size
inline constexpr double kBoundingBoxSmoothFactor = 0.9;

inline constexpr double kUnknownMovableMinHeight = 0.5;  // m
inline constexpr double kLaneWidth = 3.7;                // m

// TODO(zheng): Put them into a proto file.
inline constexpr double kNearRegionMaxDistance = 70.0;     // <= 70m
inline constexpr double kMiddleRegionMaxDistance = 100.0;  // 70~100m
// The minimum number of updates of each track before it is promoted.
inline constexpr int kNumUpdatesToPromoteForNearRegion = 3;    // 0~70m
inline constexpr int kNumUpdatesToPromoteForMiddleRegion = 5;  // 70~100m
inline constexpr int kNumUpdatesToPromoteForFarRegion = 7;     // >100m
// one-quarters
inline constexpr double kOneQuartersPi = 0.25 * M_PI;

// three-quarters
inline constexpr double kThreeQuartersPi = 0.75 * M_PI;

inline constexpr double kMaxMeasurementHistoryBufferLength = 2.3;  // 2.3s
inline constexpr double kMaxCheckPointHistoryBufferLength = 2.3;   // 2.3s

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACKER_CONSTANT_H_
