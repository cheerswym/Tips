#include "onboard/perception/registration/icp.h"

#include <random>

#include "gtest/gtest.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace {

constexpr int kNumPoints = 1000;

std::vector<Vec3d> GenerateRandomPoints() {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  std::vector<Vec3d> points;
  for (int i = 0; i < kNumPoints; ++i) {
    while (true) {
      const Vec3d point(dis(gen), dis(gen), dis(gen));
      if (std::abs(point.x()) > 0.9 || std::abs(point.y()) > 0.9 ||
          std::abs(point.z()) > 0.9) {
        points.emplace_back(point);
        break;
      }
    }
  }
  return points;
}

void TestICP(const AffineTransformation& transform) {
  std::vector<Vec3d> ref_points = GenerateRandomPoints();
  std::vector<Vec3d> obj_points;
  for (const auto& ref_point : ref_points) {
    obj_points.push_back(transform.TransformPoint(ref_point));
  }

  const Icp point_matcher;
  const PointMatcherOptions options{.max_mse = Sqr(0.001),
                                    .max_num_iters = 100};
  const auto icp_result =
      point_matcher.MatchPoints(ref_points, obj_points, options);
  const auto max_diff = (icp_result.transform.mat().inverse() - transform.mat())
                            .cwiseAbs()
                            .maxCoeff();
  EXPECT_LT(max_diff, 1e-3);
}

TEST(PointMatcherTest, Simple) {
  TestICP(AffineTransformation::FromYawPitchRoll(0.0, 0.0, 0.0)
              .ApplyTranslation(0.3, 0.3, 0.3));
  TestICP(AffineTransformation::FromYawPitchRoll(0.1, 0.1, 0.1)
              .ApplyTranslation(0.0, 0.0, 0.0));
  TestICP(AffineTransformation::FromYawPitchRoll(0.1, 0.1, 0.1)
              .ApplyTranslation(0.3, 0.3, 0.3));
}

TEST(PointMatcherTest, RandomTransform) {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(-0.5, 0.5);
  for (int i = 0; i < 20; ++i) {
    TestICP(AffineTransformation::FromYawPitchRoll(dis(gen), dis(gen), dis(gen))
                .ApplyTranslation(dis(gen), dis(gen), dis(gen)));
  }
}

}  // namespace
}  // namespace qcraft
