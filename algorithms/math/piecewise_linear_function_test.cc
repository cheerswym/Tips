#include "onboard/math/piecewise_linear_function.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEps = 1e-14;

TEST(PiecewiseLinearFunctionTest, SingleEvaluationTest) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> xs;

  x = {1.0, 2.0};
  y = {3.0, 4.0};
  PiecewiseLinearFunction plf(x, y);
  EXPECT_NEAR(plf.Evaluate(0.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.5), 3.5, kEps);
  EXPECT_NEAR(plf.Evaluate(2.0), 4.0, kEps);
  EXPECT_NEAR(plf.Evaluate(3.0), 4.0, kEps);

  x = {1.0, 2.0, 4.0};
  y = {3.0, 4.0, 5.0};
  plf = PiecewiseLinearFunction(x, y);
  EXPECT_NEAR(plf.Evaluate(0.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.5), 3.5, kEps);
  EXPECT_NEAR(plf.Evaluate(2.0), 4.0, kEps);
  EXPECT_NEAR(plf.Evaluate(3.0), 4.5, kEps);
  EXPECT_NEAR(plf.Evaluate(4.0), 5.0, kEps);
  EXPECT_NEAR(plf.Evaluate(5.0), 5.0, kEps);
}

TEST(PiecewiseLinearFunctionTest, SlopeEvaluationTest) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> xs;

  x = {1.0, 2.0};
  y = {3.0, 4.0};
  PiecewiseLinearFunction plf(x, y);
  EXPECT_NEAR(plf.EvaluateSlope(0.0), 0.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(1.0), 1.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(1.5), 1.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(2.0), 1.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(3.0), 0.0, kEps);

  x = {1.0, 2.0, 4.0};
  y = {3.0, 4.0, 5.0};
  plf = PiecewiseLinearFunction(x, y);
  EXPECT_NEAR(plf.EvaluateSlope(0.0), 0.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(1.0), 1.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(1.5), 1.0, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(2.0), 0.5, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(3.0), 0.5, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(4.0), 0.5, kEps);
  EXPECT_NEAR(plf.EvaluateSlope(5.0), 0.0, kEps);
}

void Expect(const std::vector<double> &y, const std::vector<double> &y_gt) {
  ASSERT_EQ(y.size(), y_gt.size());
  for (int i = 0; i < y.size(); ++i) {
    EXPECT_EQ(y[i], y_gt[i]);
  }
}

TEST(PiecewiseLinearFunctionTest, MultipleEvaluationTest) {
  std::vector<double> x{1.0, 2.0};
  std::vector<double> y{3.0, 4.0};
  std::vector<double> xs;
  PiecewiseLinearFunction plf(x, y);
  xs = {0.0};
  Expect(plf.Evaluate(xs), {3.0});
  xs = {0.0, 0.5};
  Expect(plf.Evaluate(xs), {3.0, 3.0});
  xs = {0.0, 0.5, 1.0};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.0});
  xs = {0.0, 0.5, 1.0, 1.5};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.0, 3.5});
  xs = {0.0, 0.5, 1.0, 1.5, 2.0};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.0, 3.5, 4.0});
  xs = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.0, 3.5, 4.0, 4.0});
  xs = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.0, 3.5, 4.0, 4.0, 4.0});
  xs = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
  Expect(plf.Evaluate(xs), {3.0, 3.0, 3.5, 4.0, 4.0, 4.0});
  xs = {1.0, 1.5, 2.0, 2.5, 3.0};
  Expect(plf.Evaluate(xs), {3.0, 3.5, 4.0, 4.0, 4.0});
  xs = {1.5, 2.0, 2.5, 3.0};
  Expect(plf.Evaluate(xs), {3.5, 4.0, 4.0, 4.0});
  xs = {2.0, 2.5, 3.0};
  Expect(plf.Evaluate(xs), {4.0, 4.0, 4.0});
  xs = {2.5, 3.0};
  Expect(plf.Evaluate(xs), {4.0, 4.0});
  xs = {3.0};
  Expect(plf.Evaluate(xs), {4.0});
  xs = {1.2};
  Expect(plf.Evaluate(xs), {3.2});
  xs = {1.2, 1.8};
  Expect(plf.Evaluate(xs), {3.2, 3.8});
}

TEST(PiecewiseLinearFunctionTest, ProtoTest) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> xs;

  x = {1.0, 2.0};
  y = {3.0, 4.0};
  PiecewiseLinearFunction plf(x, y);
  PiecewiseLinearFunctionDoubleProto plf_proto;
  PiecewiseLinearFunctionToProto(plf, &plf_proto);
  plf = PiecewiseLinearFunctionFromProto(plf_proto);
  EXPECT_NEAR(plf.Evaluate(0.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.5), 3.5, kEps);
  EXPECT_NEAR(plf.Evaluate(2.0), 4.0, kEps);
  EXPECT_NEAR(plf.Evaluate(3.0), 4.0, kEps);

  x = {1.0, 2.0, 4.0};
  y = {3.0, 4.0, 5.0};
  plf = PiecewiseLinearFunction(x, y);
  PiecewiseLinearFunctionToProto(plf, &plf_proto);
  plf = PiecewiseLinearFunctionFromProto(plf_proto);
  EXPECT_NEAR(plf.Evaluate(0.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.0), 3.0, kEps);
  EXPECT_NEAR(plf.Evaluate(1.5), 3.5, kEps);
  EXPECT_NEAR(plf.Evaluate(2.0), 4.0, kEps);
  EXPECT_NEAR(plf.Evaluate(3.0), 4.5, kEps);
  EXPECT_NEAR(plf.Evaluate(4.0), 5.0, kEps);
  EXPECT_NEAR(plf.Evaluate(5.0), 5.0, kEps);
}

}  // namespace
}  // namespace qcraft
