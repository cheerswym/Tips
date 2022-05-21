#ifndef ONBOARD_PREDICTION_PREDICTION_DEFS_H_
#define ONBOARD_PREDICTION_PREDICTION_DEFS_H_

#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "onboard/proto/perception.pb.h"
namespace qcraft {
namespace prediction {
using ResampledObjectsHistory = std::vector<std::vector<qcraft::ObjectProto>>;

constexpr double kEpsilon = 1e-8;
using ObjectIDType = std::string;

constexpr char kAvObjectId[] = "AV";
constexpr char kInvalidObjectId[] = "NA";

constexpr double kPredictionTimeStep = 0.1;  // Seconds.
constexpr double kPredictionDuration =
    10.0;  // Prediction seconds for non-stationary trajectory.
constexpr int kYawRateEffectiveStepThreshold =
    30;  // Avoid using const yaw rate in long time.
constexpr double kMinPublishProb = 1e-3;

constexpr double kEmergencyGuardHorizon = 3.0;  // s.
constexpr double kSafeHorizon = 5.0;            // s.
constexpr double kComfortableHorizon = 8.0;     // s.
constexpr double kVehCurvatureLimit = 0.25;     // m^-1.
constexpr double kVehLateralAccelLimit = 2.5;   // m^-1.

constexpr double kHistoryLen = 3.0;
/*
  Note:
    Depending on the vehicle model, not all these parameters are independent.
    E.g. for the vehicle bicycle model, wheel base, max steering angle and
    minimum turning radius are related.
 */
struct VehicleParameter {
  double length;         // m.
  double width;          // m.
  double height;         // m.
  double wheel_base;     // m. Distance between the centers of the
                         // front and rear axis.
  double gross_weitght;  // kg. gross weight.
  double drag_coefficient;
  double max_power;           // W.
  double max_steering_angle;  // radius.
  double max_steering_rate;   // radius / s.
  double min_turning_radius;  // m.
  double max_accel;           // m / s^2.
  double max_braking_decel;   // m / s^2.
  double wheelbase_to_length_ratio;
  double rac_to_front_length;
};

/*
  Deafault parameters based on actual profile of a 2021 BMW M760Li.

  Do we need a more practical vehicle here?
 */
constexpr VehicleParameter kDefaultVehicleParameter{
    /* length = */ 5.268,
    /* width = */ 1.902,
    /* height = */ 1.478,
    /* wheel_base = */ 3.211,
    /* gross_weitght = */ 2839.942,
    /* drag_coefficient = */ 0.24,
    /* max_power = */ 447.42e3,
    /* max_steering_angle = */
    3 * M_PI / 16.3,  // one and a half full turn (180 * 3) with steering
                      // ratio of 16.3.
    /* max_steering_rate = */ M_PI / 4.0,  // I made this up, but a human being
                                           // probably cannot beat this.
    /* min_turning_radius = */ 6.45,
    /* max_accel = */ 10.0,
    /* max_braking_decel = */ 10.0,
    /* wheelbase_to_length_ratio = */ 0.61,
    /* rac_to_front_length = */ 3.86  //  m
};

// Valid types of objects to apply vehicle prediction.
const std::unordered_set<ObjectType> kVehicleLikeObjectTypesToPredict{
    OT_VEHICLE, OT_MOTORCYCLIST, OT_UNKNOWN_MOVABLE};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_DEFS_H_
