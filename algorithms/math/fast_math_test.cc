#include "onboard/math/fast_math.h"

#include "gtest/gtest.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace {
constexpr double kEpsilon = 1e-4;

TEST(FastMathTest, TestAtan2Float) {
  for (float angle = -M_PI; angle < M_PI; angle += d2r(kEpsilon)) {
    const float y = std::sin(angle);
    const float x = std::cos(angle);
    const float fast_angle = fast_math::Atan2(y, x);
    EXPECT_LT(std::abs(angle - NormalizeAngle(fast_angle)), d2r(0.012))
        << r2d(angle) << ' ' << r2d(fast_angle);
  }
}

TEST(FastMathTest, TestAtan2Double) {
  for (double angle = -M_PI; angle < M_PI; angle += d2r(kEpsilon)) {
    const double y = std::sin(angle);
    const double x = std::cos(angle);
    const double fast_angle = fast_math::Atan2(y, x);
    EXPECT_LT(std::abs(angle - NormalizeAngle(fast_angle)), d2r(0.012))
        << r2d(angle) << ' ' << r2d(fast_angle);
  }
}

TEST(FastMathTest, TestSinCosFloat) {
  for (float angle = -M_PI; angle < M_PI; angle += d2r(kEpsilon)) {
    EXPECT_LT(std::abs(fast_math::Sin(angle) - std::sin(angle)), 5e-5) << angle;
    EXPECT_LT(std::abs(fast_math::Cos(angle) - std::cos(angle)), 5e-5) << angle;
  }
}

TEST(FastMathTest, TestSinCosDouble) {
  for (double angle = -M_PI; angle < M_PI; angle += d2r(kEpsilon)) {
    EXPECT_LT(std::abs(fast_math::Sin(angle) - std::sin(angle)), 5e-5) << angle;
    EXPECT_LT(std::abs(fast_math::Cos(angle) - std::cos(angle)), 5e-5) << angle;
  }
}

}  // namespace
}  // namespace qcraft
