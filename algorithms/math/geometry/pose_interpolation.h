#ifndef ONBOARD_MATH_GEOMETRY_POSE_INTERPOLATION_H_
#define ONBOARD_MATH_GEOMETRY_POSE_INTERPOLATION_H_

#include <utility>
#include <vector>

#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {

// Interpolate from pose history
// two types of interfaces
VehiclePose ComputeInterpolatedPose(const std::vector<double> &times,
                                    const std::vector<VehiclePose> &poses,
                                    double timestamp);

VehiclePose ComputeInterpolatedPose(
    const std::vector<std::pair<double, VehiclePose>> &pose_history,
    double timestamp);

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_POSE_INTERPOLATION_H_
