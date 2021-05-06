#include "onboard/math/geometry/pose_interpolation.h"

namespace qcraft {

VehiclePose ComputeInterpolatedPose(const std::vector<double>& times,
                                    const std::vector<VehiclePose>& poses,
                                    double timestamp) {
  QCHECK_EQ(times.size(), poses.size());
  PiecewiseLinearFunction<VehiclePose, double, VehiclePoseLerper> plf(times,
                                                                      poses);
  return plf.Evaluate(timestamp);
}

VehiclePose ComputeInterpolatedPose(
    const std::vector<std::pair<double, VehiclePose>>& pose_history,
    double timestamp) {
  QCHECK_GT(pose_history.size(), 0);
  std::vector<double> ts;
  std::vector<VehiclePose> poses;
  ts.reserve(pose_history.size());
  poses.reserve(pose_history.size());
  for (const auto& [t, pose] : pose_history) {
    ts.push_back(t);
    poses.push_back(pose);
  }
  return ComputeInterpolatedPose(ts, poses, timestamp);
}

}  // namespace qcraft
