#ifndef ONBOARD_PERCEPTION_OBSTACLE_CONSTANTS_H
#define ONBOARD_PERCEPTION_OBSTACLE_CONSTANTS_H

namespace qcraft {

// Obstacle detection range.
inline constexpr float kObstacleDetectionLateralDist = 60.0f;  // m
inline constexpr float kObstacleDetectionFrontDist = 150.0f;   // m
inline constexpr float kObstacleDetectionRearDist = 80.0f;     // m

// Obstacle offroad detection range.
inline constexpr float kObstacleOffroadDetectionLateralDist = 20.0f;  // m
inline constexpr float kObstacleOffroadDetectionFrontDist = 50.0f;    // m
inline constexpr float kObstacleOffroadDetectionRearDist = 20.0f;     // m

inline constexpr int kMaxNumPointsPerObstacle = 64;

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_CONSTANTS_H
