#include "onboard/math/geometry/pose_interpolation.h"

#include <random>
#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kOmega = 0.01;
constexpr double kEps = 1e-6;

AffineTransformation GenerateRandomTransform() {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  VehiclePose pose(dis(gen), dis(gen), dis(gen), dis(gen), dis(gen), dis(gen));
  return pose.ToTransform();
}

std::vector<std::pair<double, VehiclePose>> GenerateCircleTrajectory(
    const double time_inc, const AffineTransformation& transform) {
  std::vector<std::pair<double, VehiclePose>> pose_history;
  constexpr double t_end = 5;
  for (double t = 0; t <= t_end; t += time_inc) {
    const double theta = kOmega * t;
    const double x = std::cos(theta);
    const double y = std::sin(theta);
    const VehiclePose pose(x, y, 0, theta, 0, 0);
    const auto pose_transform = transform * pose.ToTransform();
    pose_history.emplace_back(t, VehiclePose::FromTransform(pose_transform));
  }
  return pose_history;
}

TEST(PoseInterpolationTest, TestCircleTrajectory) {
  for (int i = 0; i < 10; ++i) {
    const AffineTransformation transform = GenerateRandomTransform();
    const auto& pose_history = GenerateCircleTrajectory(0.1, transform);
    const auto& pose_history_dense = GenerateCircleTrajectory(0.05, transform);
    for (const auto& [t, pose] : pose_history_dense) {
      const auto pose_inter = ComputeInterpolatedPose(pose_history, t);
      const auto diff_transform =
          pose_inter.ToTransform().Inverse() * pose.ToTransform();
      EXPECT_LT(diff_transform.GetTranslation().norm(), kEps);
      EXPECT_LT(diff_transform.GetRotation().vec().norm(), kEps);
    }
  }
}

}  // namespace
}  // namespace qcraft
