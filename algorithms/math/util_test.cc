#include "onboard/math/util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace {

using ::testing::ElementsAre;

template <typename T>
constexpr T kValues[] = {1.0,  0.999999,  1.5,  1.000001,
                         -1.0, -0.999999, -1.5, -1.000001};
constexpr bool kIsInt[] = {true, false, false, false,
                           true, false, false, false};

TEST(MathUtilTest, TestFloorToInt) {
  const auto floor = [](auto val) { return static_cast<int>(std::floor(val)); };
  for (const float value : kValues<float>) {
    EXPECT_EQ(floor(value), FloorToInt(value));
  }
  for (const double value : kValues<double>) {
    EXPECT_EQ(floor(value), FloorToInt(value));
  }
}

TEST(MathUtilTest, TestCeilToInt) {
  const auto ceil = [](auto val) { return static_cast<int>(std::ceil(val)); };
  for (const float value : kValues<float>) {
    EXPECT_EQ(ceil(value), CeilToInt(value));
  }
  for (const double value : kValues<double>) {
    EXPECT_EQ(ceil(value), CeilToInt(value));
  }
}

TEST(MathUtilTest, TestRoundToInt) {
  const auto round = [](auto val) { return static_cast<int>(std::round(val)); };
  for (const float value : kValues<float>) {
    EXPECT_EQ(round(value), RoundToInt(value));
  }
  for (const double value : kValues<double>) {
    EXPECT_EQ(round(value), RoundToInt(value));
  }

#ifdef __AVX512F__
  // RoundToInt rounds half to even. E.g., it rounds 1.5 to 2, and 2.5 to 2.
  EXPECT_NE(round(2.5), RoundToInt(2.5));
#endif  // __AVX512F__
}

TEST(MathUtilTest, TestPower) {
  double base = 2.5;

  EXPECT_NEAR(Power(base, 0), 1.0, 1e-6);
  EXPECT_NEAR(Power(base, 2), 6.25, 1e-6);
  EXPECT_NEAR(Power(base, 5), 97.65625, 1e-6);
  // EXPECT_NEAR(Power(base, 1.5), 3.95284, 1e-6);  // Should not compile.
}

TEST(MathUtilTest, TestTruncateToInt) {
  const auto truncate = [](auto val) { return static_cast<int>(val); };
  for (const float value : kValues<float>) {
    EXPECT_EQ(truncate(value), TruncateToInt(value));
  }
  for (const double value : kValues<double>) {
    EXPECT_EQ(truncate(value), TruncateToInt(value));
  }
}

TEST(MathUtilTest, TestIsInt) {
  for (int i = 0; i < sizeof(kValues<float>) / sizeof(float); ++i) {
    EXPECT_EQ(kIsInt[i], IsInt(kValues<float>[i]));
  }
  for (int i = 0; i < sizeof(kValues<double>) / sizeof(double); ++i) {
    EXPECT_EQ(kIsInt[i], IsInt(kValues<double>[i]));
  }
}

TEST(MathUtilTest, WrapToRange) {
  constexpr double kEpsilon = 1e-14;
  EXPECT_NEAR(WrapToRange(0.4, 0.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(1.4, 0.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(10.4, 0.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(-0.6, 0.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(-10.6, 0.0, 1.0), 0.4, kEpsilon);

  EXPECT_NEAR(WrapToRange(0.4, -1.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(-0.4, -1.0, 1.0), -0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(1.6, -1.0, 1.0), -0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(-1.6, -1.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(10.4, -1.0, 1.0), 0.4, kEpsilon);
  EXPECT_NEAR(WrapToRange(-10.4, -1.0, 1.0), -0.4, kEpsilon);
}

TEST(MathUtilTest, NormalizeAngle2D) {
  EXPECT_EQ(NormalizeAngle2D(Vec2d(0.0, 1.0), Vec2d(0.0, 1.0)), 0.0);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(1.0, 0.0), Vec2d(0.0, 1.0)), M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(0.1, 0.0), Vec2d(0.0, 9.0)), M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(0.0, 0.1), Vec2d(7.1, 0.0)), -M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(1.0, 0.0), Vec2d(-1.0, 0.0)), -M_PI);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(1.0, 0.0), Vec2d(-2.1, -2.1)), -0.75 * M_PI);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(-2.0, 2.0), Vec2d(-9.9, -9.9)), M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(-9.9, -9.9), Vec2d(-2.0, 2.0)), -M_PI / 2);

  EXPECT_EQ(NormalizeAngle2D(Vec2d(1.0, 0.0)), 0);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(-1.0, 0.0)), -M_PI);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(0, 1.0)), M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(0, -1.0)), -M_PI / 2);
  EXPECT_EQ(NormalizeAngle2D(Vec2d(-1.0, -1.0)), -0.75 * M_PI);
}

TEST(MathUtilTest, UpdateMax) {
  double x = 0.0;
  UpdateMax(-1.0, &x);
  EXPECT_EQ(x, 0.0);
  UpdateMax(3.0, &x);
  EXPECT_EQ(x, 3.0);

  UpdateMax(1.0, &x);
  EXPECT_EQ(x, 3.0);
}
TEST(MathUtilTest, UpdateMin) {
  double x = 0.0;
  UpdateMin(3.0, &x);
  EXPECT_EQ(x, 0.0);
  UpdateMin(-1.0, &x);
  EXPECT_EQ(x, -1.0);
  UpdateMin(3.0, &x);
  EXPECT_EQ(x, -1.0);
}

TEST(MathUtilsTest, QuadraticRoot) {
  EXPECT_THAT(QuadraticRoot(0.0, 0.0, 1.0), ElementsAre());
  EXPECT_THAT(QuadraticRoot(0.0, 1.0, 1.0), ElementsAre(-1.0));
  EXPECT_THAT(QuadraticRoot(1.0, 0.0, -4.0), ElementsAre(-2.0, 2.0));
  EXPECT_THAT(QuadraticRoot(1.0, 0.0, 4.0), ElementsAre());
  EXPECT_THAT(QuadraticRoot(1.0, -1.0, -2.0), ElementsAre(-1.0, 2.0));
}

TEST(MathUtilTest, OppositeAngle) {
  EXPECT_EQ(OppositeAngle(M_PI), 0.0);
  EXPECT_EQ(OppositeAngle(-M_PI), 0.0);
  EXPECT_EQ(OppositeAngle(0.0), -M_PI);
  EXPECT_EQ(OppositeAngle(M_PI_2), -M_PI_2);
  EXPECT_EQ(OppositeAngle(-M_PI_2), M_PI_2);
  EXPECT_NEAR(OppositeAngle(d2r(30.0)), d2r(-150.0), 1e-10);
  EXPECT_NEAR(OppositeAngle(d2r(150.0)), d2r(-30.0), 1e-10);
}

}  // namespace
}  // namespace qcraft
