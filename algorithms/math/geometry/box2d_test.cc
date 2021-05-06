/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "onboard/math/geometry/box2d.h"

#include <random>

#include "gtest/gtest.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/test_util.h"

namespace qcraft {
namespace {

using testing::ElementsAre;

double RandomDouble(double start, double end) {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(start, end);
  return dis(gen);
}

bool CheckBoxOverlapSlow(const Box2d &box1, const Box2d &box2,
                         bool *const ambiguous) {
  double headings[4] = {box1.heading(), box1.heading() + M_PI_2, box2.heading(),
                        box2.heading() + M_PI_2};
  *ambiguous = false;
  for (int k = 0; k < 4; ++k) {
    const double heading = headings[k];
    const double cos_heading = cos(heading);
    const double sin_heading = sin(heading);
    const auto c1 = box1.GetCornersCounterClockwise();
    const auto c2 = box2.GetCornersCounterClockwise();
    double s1 = std::numeric_limits<double>::infinity();
    double t1 = -std::numeric_limits<double>::infinity();
    double s2 = std::numeric_limits<double>::infinity();
    double t2 = -std::numeric_limits<double>::infinity();
    for (const auto &p : c1) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s1) s1 = proj;
      if (proj > t1) t1 = proj;
    }
    for (const auto &p : c2) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s2) s2 = proj;
      if (proj > t2) t2 = proj;
    }
    if (std::abs(s1 - t2) <= 1e-5 || std::abs(s2 - t1) <= 1e-5) {
      *ambiguous = true;
    }
    if (s1 > t2 || s2 > t1) {
      return false;
    }
  }
  return true;
}

Box2d box1({0, 0}, 0, 4, 2);
Box2d box2({5, 2}, 0, 4, 2);
Box2d box3(Segment2d({2, 3}, {6, 3}), 2);
Box2d box4({7, 8}, M_PI_4, 5.0, 3.0);
Box2d box5({-2, -3}, -M_PI, 0.0, 0.0);
Box2d box6(Segment2d({2, 3}, {6, 3}), 0.0);
Box2d box7(AABox2d({4, 5}, 0, 0));

TEST(Box2dTest, DebugString) {
  Box2d box1({0, 0}, 0, 4, 2);
  EXPECT_THAT(box1.center(), Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(box1.heading(), 0.0);
  EXPECT_EQ(box1.length(), 4.0);
  EXPECT_EQ(box1.width(), 2.0);

  Box2d box2({5, 2}, 0, 4, 2);
  EXPECT_THAT(box2.center(), Vec2dEqXY(5.0, 2.0));
  EXPECT_EQ(box2.heading(), 0.0);
  EXPECT_EQ(box2.length(), 4.0);
  EXPECT_EQ(box2.width(), 2.0);
}

TEST(Box2dTest, ConstructFromTangent) {
  Box2d box1({0.0, 0.0}, {0.0, 1.0}, 4.0, 2.0);
  EXPECT_NEAR(box1.heading(), M_PI_2, 1e-8);
  EXPECT_EQ(box1.cos_heading(), 0.0);
  EXPECT_EQ(box1.sin_heading(), 1.0);

  Box2d box2({0.0, 0.0}, {std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0}, 4.0,
             2.0);
  EXPECT_NEAR(box2.heading(), M_PI_2 / 2.0, 1e-8);
  EXPECT_EQ(box2.cos_heading(), std::sqrt(2.0) / 2.0);
  EXPECT_EQ(box2.sin_heading(), std::sqrt(2.0) / 2.0);
}

TEST(Box2dTest, GetCornersCounterClockwise) {
  const auto corners1 = box1.GetCornersCounterClockwise();

  EXPECT_THAT(
      corners1,
      ElementsAre(Vec2dNearXY(2.0, 1.0, 1e-5), Vec2dNearXY(-2.0, 1.0, 1e-5),
                  Vec2dNearXY(-2.0, -1.0, 1e-5), Vec2dNearXY(2.0, -1.0, 1e-5)));

  const auto corners2 = box3.GetCornersCounterClockwise();
  EXPECT_THAT(
      corners2,
      ElementsAre(Vec2dNearXY(6.0, 4.0, 1e-5), Vec2dNearXY(2.0, 4.0, 1e-5),
                  Vec2dNearXY(2.0, 2.0, 1e-5), Vec2dNearXY(6.0, 2.0, 1e-5)));
}

TEST(Box2dTest, FromAABox2d) {
  AABox2d aabox(Vec2d(1.0, 2.0), 3.0, 4.0);
  Box2d box(aabox);
  EXPECT_EQ(box.GetCornersCounterClockwise().size(), 4);
}

TEST(Box2dTest, CenterAndLength) {
  EXPECT_NEAR(4, box3.center().x(), 1e-5);
  EXPECT_NEAR(3, box3.center().y(), 1e-5);
  EXPECT_NEAR(4, box3.length(), 1e-5);
  EXPECT_NEAR(2, box3.width(), 1e-5);
  EXPECT_NEAR(2, box3.half_length(), 1e-5);
  EXPECT_NEAR(1, box3.half_width(), 1e-5);
  EXPECT_NEAR(Hypot(4.0, 2.0), box3.diagonal(), 1e-5);
}

TEST(Box2dTest, FrontRearPoint) {
  constexpr double kEpsilon = 1e-5;
  EXPECT_THAT(box1.FrontCenterPoint(), Vec2dNearXY(2.0, 0.0, kEpsilon));
  EXPECT_THAT(box1.RearCenterPoint(), Vec2dNearXY(-2.0, 0.0, kEpsilon));
  EXPECT_THAT(box2.FrontCenterPoint(), Vec2dNearXY(7.0, 2.0, kEpsilon));
  EXPECT_THAT(box2.RearCenterPoint(), Vec2dNearXY(3.0, 2.0, kEpsilon));
}

TEST(Box2dTest, HasOverlap) {
  EXPECT_FALSE(box1.HasOverlap(box2));
  EXPECT_FALSE(box1.HasOverlap(box3));
  EXPECT_FALSE(box1.HasOverlap(box4));
  EXPECT_FALSE(box2.HasOverlap(box4));
  EXPECT_FALSE(box3.HasOverlap(box4));

  EXPECT_TRUE(box1.HasOverlap(Segment2d({0, 0}, {1, 1})));
  EXPECT_TRUE(box1.HasOverlap(Segment2d({0, 0}, {3, 3})));
  EXPECT_TRUE(box1.HasOverlap(Segment2d({0, -3}, {0, 3})));
  EXPECT_TRUE(box1.HasOverlap(Segment2d({4, 0}, {-4, 0})));
  EXPECT_TRUE(box1.HasOverlap(Segment2d({-4, -4}, {4, 4})));
  EXPECT_TRUE(box1.HasOverlap(Segment2d({4, -4}, {-4, 4})));
  EXPECT_FALSE(box1.HasOverlap(Segment2d({-4, -4}, {4, -4})));
  EXPECT_FALSE(box1.HasOverlap(Segment2d({4, -4}, {4, 4})));
}

TEST(Box2dTest, HasOverlapWithAABox2d) {
  const std::vector<Box2d> boxes = {box1, box2, box3, box4, box5, box6, box7};
  std::vector<AABox2d> aaboxes;
  aaboxes.reserve(boxes.size());
  for (const auto &box : boxes) {
    aaboxes.push_back(box.GetAABox());
  }

  for (const auto &box : boxes) {
    const Polygon2d polygon_a(box);
    for (const auto &aabox : aaboxes) {
      const Box2d wrap_aabox(aabox);
      VLOG(0) << wrap_aabox.DebugString();
      const Polygon2d polygon_b(wrap_aabox);
      EXPECT_EQ(polygon_a.HasOverlap(polygon_b), box.HasOverlap(aabox));
    }
  }
}

TEST(Box2dTest, GetAABox) {
  AABox2d aabox1 = box1.GetAABox();
  AABox2d aabox2 = box2.GetAABox();
  AABox2d aabox3 = box3.GetAABox();
  AABox2d aabox4 = box4.GetAABox();
  EXPECT_NEAR(aabox1.center_x(), 0.0, 1e-5);
  EXPECT_NEAR(aabox1.center_y(), 0.0, 1e-5);
  EXPECT_NEAR(aabox1.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox1.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox2.center_x(), 5.0, 1e-5);
  EXPECT_NEAR(aabox2.center_y(), 2.0, 1e-5);
  EXPECT_NEAR(aabox2.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox2.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox3.center_x(), 4.0, 1e-5);
  EXPECT_NEAR(aabox3.center_y(), 3.0, 1e-5);
  EXPECT_NEAR(aabox3.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox3.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox4.center_x(), 7.0, 1e-5);
  EXPECT_NEAR(aabox4.center_y(), 8.0, 1e-5);
  EXPECT_NEAR(aabox4.length(), 4.0 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(aabox4.width(), 4.0 * sqrt(2.0), 1e-5);
}

TEST(Box2dTest, DistanceTo) {
  EXPECT_NEAR(box1.DistanceTo({3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({-3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, -2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, -1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({-1, 0}), 0.0, 1e-5);

  EXPECT_NEAR(box1.DistanceTo(Segment2d({-4, -4}, {4, 4})), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(Segment2d({4, -4}, {-4, 4})), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(Segment2d({0, 2}, {4, 4})), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(Segment2d({2, 2}, {3, 1})), std::sqrt(2.0) / 2.0,
              1e-5);
}

TEST(Box2dTest, IsPointIn) {
  EXPECT_TRUE(box1.IsPointIn({0, 0}));
  EXPECT_TRUE(box1.IsPointIn({1, 0.5}));
  EXPECT_TRUE(box1.IsPointIn({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointIn({2, 1}));
  EXPECT_FALSE(box1.IsPointIn({-3, 0}));
  EXPECT_FALSE(box1.IsPointIn({0, 2}));
  EXPECT_FALSE(box1.IsPointIn({-4, -2}));
}

TEST(Box2dTest, IsPointOnBoundary) {
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({1, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointOnBoundary({2, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-2, 1}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-3, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 2}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-4, -2}));
}

TEST(Box2dTest, FromToProto) {
  const Box2d box({0.0, 1.0}, 0.5, 2, 3);

  // Convert from box to proto.
  Box2dProto proto;
  box.ToProto(&proto);
  EXPECT_EQ(proto.x(), 0.0);
  EXPECT_EQ(proto.y(), 1.0);
  EXPECT_EQ(proto.heading(), 0.5);
  EXPECT_EQ(proto.length(), 2);
  EXPECT_EQ(proto.width(), 3);

  // Convert from proto to box.
  const Box2d box2(proto);
  EXPECT_THAT(box2, Box2dNear(
                        /*center=*/Vec2d{0.0, 1.0}, 0.5, 2.0, 3.0, 1e-8));
}

TEST(Box2dTest, RotateFromCenterAndShift) {
  Box2d box({0, 0}, 0, 4, 2);
  EXPECT_NEAR(box.heading(), 0.0, 1e-5);
  box.RotateFromCenter(M_PI_2);
  EXPECT_NEAR(box.heading(), M_PI_2, 1e-5);
  auto corners = box.GetCornersCounterClockwise();
  EXPECT_THAT(
      corners,
      ElementsAre(Vec2dNearXY(-1.0, 2.0, 1e-5), Vec2dNearXY(-1.0, -2.0, 1e-5),
                  Vec2dNearXY(1.0, -2.0, 1e-5), Vec2dNearXY(1.0, 2.0, 1e-5)));

  box.Shift({30, 40});
  corners = box.GetCornersCounterClockwise();
  EXPECT_THAT(corners, ElementsAre(Vec2dNearXY(29.0, 42.0, 1e-5),
                                   Vec2dNearXY(29.0, 38.0, 1e-5),
                                   Vec2dNearXY(31.0, 38.0, 1e-5),
                                   Vec2dNearXY(31.0, 42.0, 1e-5)));
}

TEST(Box2dTest, MirrorByX) {
  Box2d box({2.0, 3.0}, M_PI / 3.0, 4, 2);
  box.MirrorByX();
  EXPECT_NEAR(box.center_x(), 2.0, 1e-5);
  EXPECT_NEAR(box.center_y(), -3.0, 1e-5);
  EXPECT_NEAR(box.heading(), -M_PI / 3.0, 1e-5);
  EXPECT_NEAR(box.sin_heading(), -std::sqrt(3.0) / 2, 1e-5);
  EXPECT_NEAR(box.cos_heading(), 0.5, 1e-5);
}

TEST(Box2dTest, TestByRandom) {
  bool ambiguous = false;
  for (int iter = 0; iter < 10000; ++iter) {
    const double x1 = RandomDouble(-10, 10);
    const double y1 = RandomDouble(-10, 10);
    const double x2 = RandomDouble(-10, 10);
    const double y2 = RandomDouble(-10, 10);
    const double heading1 = RandomDouble(0, M_PI * 2.0);
    const double heading2 = RandomDouble(0, M_PI * 2.0);
    const double l1 = RandomDouble(1, 5);
    const double l2 = RandomDouble(1, 5);
    const double w1 = RandomDouble(1, 5);
    const double w2 = RandomDouble(1, 5);
    const Box2d box1({x1, y1}, heading1, l1, w1);
    const Box2d box2({x2, y2}, heading2, l2, w2);
    bool overlap = CheckBoxOverlapSlow(box1, box2, &ambiguous);
    if (!ambiguous) {
      EXPECT_EQ(overlap, box1.HasOverlap(box2));
    }
  }
}

}  // namespace
}  // namespace qcraft
