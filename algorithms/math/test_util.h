#ifndef ONBOARD_MATH_TEST_UTIL_H_
#define ONBOARD_MATH_TEST_UTIL_H_

#include "gmock/gmock.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

MATCHER_P(Vec2dEq, v, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  return arg.x() == v.x() && arg.y() == v.y();
}

MATCHER_P2(Vec2dEqXY, x, y, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  return arg.x() == x && arg.y() == y;
}

MATCHER_P2(Vec2dNear, v, epsilon, "") {
  *result_listener << "Actual value: " << arg.DebugString()
                   << "\nExpected value: " << v.DebugString();
  const qcraft::Vec2d diff = arg - v;
  return diff.Sqr() < qcraft::Sqr(epsilon);
}

MATCHER_P3(Vec2dNearXY, x, y, epsilon, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  const qcraft::Vec2d diff = arg - qcraft::Vec2d(x, y);
  return diff.Sqr() < qcraft::Sqr(epsilon);
}

MATCHER_P3(AABox2dEq, center, length, width, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  const qcraft::AABox2d expected_box(center, length, width);
  return arg == expected_box;
}

MATCHER_P4(AABox2dNear, center, length, width, epsilon, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  const ::qcraft::AABox2d expected_box(center, length, width);
  const ::qcraft::Vec2d center_diff = expected_box.center() - arg.center();
  return center_diff.Sqr() < ::qcraft::Sqr(epsilon) &&
         std::abs(expected_box.length() - arg.length()) < epsilon &&
         std::abs(expected_box.width() - arg.width()) < epsilon;
}

MATCHER_P5(Box2dNear, center, heading, length, width, epsilon, "") {
  *result_listener << "Actual value: " << arg.DebugString();
  const ::qcraft::Box2d expected_box(center, heading, length, width);
  const ::qcraft::Vec2d center_diff = expected_box.center() - arg.center();
  return center_diff.Sqr() < ::qcraft::Sqr(epsilon) &&
         std::abs(expected_box.length() - arg.length()) < epsilon &&
         std::abs(expected_box.width() - arg.width()) < epsilon &&
         std::abs(::qcraft::AngleDifference(expected_box.heading(),
                                            arg.heading())) < epsilon;
}

#endif  // ONBOARD_MATH_TEST_UTIL_H_
