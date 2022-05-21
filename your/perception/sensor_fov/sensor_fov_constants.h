#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_CONSTANTS_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_CONSTANTS_H_

namespace qcraft::sensor_fov {

// Detection range in vehicle coordinate.
constexpr double kSFDetectionRangeFront = 80.0f;    // m
constexpr double kSFDetectionRangeRear = 20.0f;     // m
constexpr double kSFDetectionRangeLateral = 40.0f;  // m
// Max height
constexpr float kSFDetectionRangeHeight = 3.0;  // m
// Grid pillar diameter size.
constexpr double kSFGridDiameter = 0.4f;  // m
// Grid serialize info
constexpr float kSerializedElevationResolution = 0.015;  // m
// Occluded ratio threshold.
constexpr float kLidarViewMinBoxOccludedRatio = 0.85f;
constexpr float kLidarViewMinPolygonOccludedRatio = 0.90f;
constexpr float kCameraViewMinBoxOccludedRatio = 0.50f;
constexpr float kCameraViewMinPolygonOccludedRatio = 0.50f;

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_CONSTANTS_H_
