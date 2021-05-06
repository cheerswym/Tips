#include "onboard/math/vec.h"

#include <typeinfo>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/affine_transformation.pb.h"
#include "onboard/utils/test_util.h"

namespace qcraft::planner {
namespace {

TEST(Vec2d, MemberOperators) {
  const Vec2d v1(1.5, 2.5);
  const Vec2d v2(1, -2);

  const auto vec_add_vec = v1 + v2;
  EXPECT_EQ(typeid(vec_add_vec), typeid(Vec2d));
  EXPECT_NE(typeid(vec_add_vec), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_add_vec.x(), 2.5, 1e-6);
  EXPECT_NEAR(vec_add_vec.y(), 0.5, 1e-6);

  const auto vec_add_builtin = v1 + 0.7;
  EXPECT_EQ(typeid(vec_add_builtin), typeid(Vec2d));
  EXPECT_NE(typeid(vec_add_builtin), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_add_builtin.x(), 2.2, 1e-6);
  EXPECT_NEAR(vec_add_builtin.y(), 3.2, 1e-6);

  const auto vec_add_builtin2 = v1 + 2;
  EXPECT_EQ(typeid(vec_add_builtin2), typeid(Vec2d));
  EXPECT_NE(typeid(vec_add_builtin2), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_add_builtin2.x(), 3.5, 1e-6);
  EXPECT_NEAR(vec_add_builtin2.y(), 4.5, 1e-6);

  const auto vec_sub_vec = v1 - v2;
  EXPECT_EQ(typeid(vec_sub_vec), typeid(Vec2d));
  EXPECT_NE(typeid(vec_sub_vec), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_sub_vec.x(), 0.5, 1e-6);
  EXPECT_NEAR(vec_sub_vec.y(), 4.5, 1e-6);

  const auto vec_sub_builtin = v1 - 0.7;
  EXPECT_EQ(typeid(vec_sub_builtin), typeid(Vec2d));
  EXPECT_NE(typeid(vec_sub_builtin), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_sub_builtin.x(), 0.8, 1e-6);
  EXPECT_NEAR(vec_sub_builtin.y(), 1.8, 1e-6);

  const auto vec_sub_builtin2 = v1 - 1;
  EXPECT_EQ(typeid(vec_sub_builtin2), typeid(Vec2d));
  EXPECT_NE(typeid(vec_sub_builtin2), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_sub_builtin2.x(), 0.5, 1e-6);
  EXPECT_NEAR(vec_sub_builtin2.y(), 1.5, 1e-6);

  const auto vec_mul_vec_t = v1 * v2.transpose();
  EXPECT_NEAR(vec_mul_vec_t(0, 0), 1.5, 1e-6);
  EXPECT_NEAR(vec_mul_vec_t(0, 1), -3.0, 1e-6);
  EXPECT_NEAR(vec_mul_vec_t(1, 0), 2.5, 1e-6);
  EXPECT_NEAR(vec_mul_vec_t(1, 1), -5.0, 1e-6);

  const auto vec_mul_builtin = v1 * 0.8;
  EXPECT_EQ(typeid(vec_mul_builtin), typeid(Vec2d));
  EXPECT_NE(typeid(vec_mul_builtin), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_mul_builtin.x(), 1.2, 1e-6);
  EXPECT_NEAR(vec_mul_builtin.y(), 2.0, 1e-6);

  const auto vec_mul_builtin2 = v1 * 3;
  EXPECT_EQ(typeid(vec_mul_builtin2), typeid(Vec2d));
  EXPECT_NE(typeid(vec_mul_builtin2), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_mul_builtin2.x(), 4.5, 1e-6);
  EXPECT_NEAR(vec_mul_builtin2.y(), 7.5, 1e-6);

  const auto vec_div_builtin = v1 / 0.5;
  EXPECT_EQ(typeid(vec_div_builtin), typeid(Vec2d));
  EXPECT_NE(typeid(vec_div_builtin), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_div_builtin.x(), 3.0, 1e-6);
  EXPECT_NEAR(vec_div_builtin.y(), 5.0, 1e-6);

  const auto vec_div_builtin2 = v1 / 5;
  EXPECT_EQ(typeid(vec_div_builtin2), typeid(Vec2d));
  EXPECT_NE(typeid(vec_div_builtin2), typeid(Vec2d::Base));
  EXPECT_NEAR(vec_div_builtin2.x(), 0.3, 1e-6);
  EXPECT_NEAR(vec_div_builtin2.y(), 0.5, 1e-6);
}

TEST(Vec2d, FriendOperators) {
  const Vec2d vec(1.5, 2.5);

  const auto builtin_add_vec = 0.7 + vec;
  EXPECT_EQ(typeid(builtin_add_vec), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_add_vec), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_add_vec.x(), 2.2, 1e-6);
  EXPECT_NEAR(builtin_add_vec.y(), 3.2, 1e-6);

  const auto builtin_add_vec2 = 1 + vec;
  EXPECT_EQ(typeid(builtin_add_vec2), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_add_vec2), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_add_vec2.x(), 2.5, 1e-6);
  EXPECT_NEAR(builtin_add_vec2.y(), 3.5, 1e-6);

  const auto builtin_sub_vec = 3.0 - vec;
  EXPECT_EQ(typeid(builtin_sub_vec), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_sub_vec), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_sub_vec.x(), 1.5, 1e-6);
  EXPECT_NEAR(builtin_sub_vec.y(), 0.5, 1e-6);

  const auto builtin_sub_vec2 = 3 - vec;
  EXPECT_EQ(typeid(builtin_sub_vec2), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_sub_vec2), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_sub_vec2.x(), 1.5, 1e-6);
  EXPECT_NEAR(builtin_sub_vec2.y(), 0.5, 1e-6);

  Eigen::Matrix<double, 2, 3> mat;
  mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const auto mat_mul_vec = mat.transpose() * vec;
  EXPECT_NEAR(mat_mul_vec[0], 11.5, 1e-6);
  EXPECT_NEAR(mat_mul_vec[1], 15.5, 1e-6);
  EXPECT_NEAR(mat_mul_vec[2], 19.5, 1e-6);

  const auto builtin_mul_vec = 0.8 * vec;
  EXPECT_EQ(typeid(builtin_mul_vec), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_mul_vec), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_mul_vec.x(), 1.2, 1e-6);
  EXPECT_NEAR(builtin_mul_vec.y(), 2.0, 1e-6);

  const auto builtin_mul_vec2 = 2 * vec;
  EXPECT_EQ(typeid(builtin_mul_vec2), typeid(Vec2d));
  EXPECT_NE(typeid(builtin_mul_vec2), typeid(Vec2d::Base));
  EXPECT_NEAR(builtin_mul_vec2.x(), 3.0, 1e-6);
  EXPECT_NEAR(builtin_mul_vec2.y(), 5.0, 1e-6);
}

TEST(Vec2d, FullPrecision) {
  const Vec2d v1(1.0 / 7.0, 1.0 / 3.0);
  const Vec2d v2(2.0 / 7.0, -2.0 / 3.0);
  const auto out = qcraft::DebugStringFullPrecision({v1, v2});
  EXPECT_EQ(
      out,
      "{x: 1.428571428571428492127e-01, y: 3.333333333333333148296e-01}, {x: "
      "2.857142857142856984254e-01, y: -6.666666666666666296592e-01}");
}

}  // namespace
}  // namespace qcraft::planner
