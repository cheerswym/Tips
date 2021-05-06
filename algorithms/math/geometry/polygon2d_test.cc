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

// NOTE: This file is copied from Apollo project and modified by qcraft.ai for
// its own use.

#include "onboard/math/geometry/polygon2d.h"

#include <random>

#include "gtest/gtest.h"
#include "onboard/math/test_util.h"

namespace qcraft {
namespace {

using testing::ElementsAre;
using testing::UnorderedElementsAre;

double RandomDouble(double start, double end) {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(start, end);
  return dis(gen);
}

double RandomInt(int start, int end) {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(start, end);
  return dis(gen);
}

bool ProjectByXSlow(const std::vector<Vec2d> &points, double x,
                    double *const min_y, double *const max_y) {
  *min_y = std::numeric_limits<double>::infinity();
  *max_y = -std::numeric_limits<double>::infinity();
  for (const Vec2d &p1 : points) {
    if (p1.x() < x) {
      for (const Vec2d &p2 : points) {
        if (p2.x() > x) {
          const double y = ((p2.x() - x) * p1.y() + (x - p1.x()) * p2.y()) /
                           (p2.x() - p1.x());
          *min_y = std::min(*min_y, y);
          *max_y = std::max(*max_y, y);
        }
      }
    }
  }
  return *min_y <= *max_y;
}

TEST(Polygon2dTest, AreConvexHullPoints) {
  {
    // This is not a convex polygon.
    // https://docs.google.com/drawings/d/1v5zrcw6QkUgQvzGO8oLn0jYFkvwfqowOHGmtvuhbmhs
    const std::vector<Vec2d> points = {{1.0, 1.0},  {-1.0, 1.0}, {-1.0, -1.0},
                                       {1.0, -1.0}, {0.0, 1.0},  {0.0, -1.0}};
    EXPECT_FALSE(Polygon2d::AreConvexHullPoints(points));
  }
  {
    // A square.
    const std::vector<Vec2d> points = {
        {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}};
    Polygon2d polygon(points);
    EXPECT_TRUE(Polygon2d::AreConvexHullPoints(points));
  }

  {
    // A Star shape.
    // https:docs.google.com/drawings/d/1z65akPNsbEdBPD-YNNFVL462k2LhcD4xCW-s9dj5Be0
    const std::vector<Vec2d> points = {
        {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {0.0, 2.0}, {-1.0, -1.0}};
    EXPECT_FALSE(Polygon2d::AreConvexHullPoints(points));
  }
}

TEST(Polygon2dTest, polygon_IsPointIn) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  EXPECT_THAT(poly1.points(),
              ElementsAre(Vec2dEqXY(1.0, 1.0), Vec2dEqXY(0.0, 1.0),
                          Vec2dEqXY(0.0, 0.0), Vec2dEqXY(1.0, 0.0)));

  EXPECT_TRUE(poly1.IsPointIn({0.5, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({0.2, 0.2}));
  EXPECT_TRUE(poly1.IsPointIn({0.2, 0.8}));
  EXPECT_TRUE(poly1.IsPointIn({0.8, 0.2}));
  EXPECT_TRUE(poly1.IsPointIn({0.8, 0.8}));

  EXPECT_TRUE(poly1.IsPointOnBoundary({0.0, 0.0}));
  EXPECT_TRUE(poly1.IsPointIn({0.0, 0.0}));
  EXPECT_TRUE(poly1.IsPointOnBoundary({0.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({0.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointOnBoundary({1.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({1.0, 0.5}));

  EXPECT_FALSE(poly1.IsPointIn({-0.2, 0.5}));
  EXPECT_FALSE(poly1.IsPointIn({1.2, 0.5}));
  EXPECT_FALSE(poly1.IsPointIn({0.5, -0.2}));
  EXPECT_FALSE(poly1.IsPointIn({0.5, 1.2}));

  EXPECT_FALSE(poly1.IsPointIn({0, -0.1}));
  EXPECT_FALSE(poly1.IsPointIn({-0.1, 0}));
  EXPECT_FALSE(poly1.IsPointIn({1.0, -0.1}));
  EXPECT_FALSE(poly1.IsPointIn({-0.1, 1.0}));
  EXPECT_FALSE(poly1.IsPointIn({0, 1.1}));
  EXPECT_FALSE(poly1.IsPointIn({1.1, 0}));
  EXPECT_FALSE(poly1.IsPointIn({1.0, 1.1}));
  EXPECT_FALSE(poly1.IsPointIn({1.1, 1.0}));

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_TRUE(poly2.IsPointIn({0, 0}));
  EXPECT_TRUE(poly2.IsPointIn({0, 0.9}));
  EXPECT_TRUE(poly2.IsPointIn({0.9, 0}));
  EXPECT_TRUE(poly2.IsPointIn({0, -0.9}));
  EXPECT_TRUE(poly2.IsPointIn({-0.9, 0}));

  EXPECT_FALSE(poly2.IsPointIn({0, 1.1}));
  EXPECT_FALSE(poly2.IsPointIn({1.1, 0}));
  EXPECT_FALSE(poly2.IsPointIn({0, -1.1}));
  EXPECT_FALSE(poly2.IsPointIn({-1.1, 0}));

  const Polygon2d poly3({{4, 4}, {5, 6}, {6, 6}});
  EXPECT_FALSE(poly3.IsPointIn({5, 4.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 4.5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 5}));
  EXPECT_TRUE(poly3.IsPointOnBoundary({5, 5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 5.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 5.5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 6}));
  EXPECT_TRUE(poly3.IsPointOnBoundary({5, 6}));
  EXPECT_FALSE(poly3.IsPointIn({5, 6.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 6.5}));

  // Concave polygons.
  const Polygon2d poly4({{0, 0}, {2, 0}, {2, 2}, {1, 1}, {0, 2}});
  EXPECT_TRUE(poly4.IsPointIn({0.5, 1.5}));
  EXPECT_TRUE(poly4.IsPointOnBoundary({0.5, 1.5}));
  EXPECT_FALSE(poly4.IsPointIn({1.0, 1.5}));
  EXPECT_TRUE(poly4.IsPointIn({1.5, 1.5}));
  EXPECT_TRUE(poly4.IsPointOnBoundary({1.5, 1.5}));

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  EXPECT_FALSE(poly5.IsPointIn({-0.5, 2.0}));
  EXPECT_TRUE(poly5.IsPointIn({0.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({1.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({2.0, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({2.5, 2.0}));
  EXPECT_TRUE(poly5.IsPointIn({3.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({4.5, 2.0}));
}

TEST(Polygon2dTest, DistanceToPoint) {
  const Box2d box1(Box2d::CreateAABox({0, 0}, {1, 1}));
  const Polygon2d poly1(box1);
  EXPECT_NEAR(poly1.DistanceTo({0.5, 0.5}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({-0.2, 0.5}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.2, 0.5}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0.5, -0.2}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0.5, 1.2}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({0, -0.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.0, -0.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 1.0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 1.0}), 0.1, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({-0.1, -0.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 1.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, -0.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 1.1}), 0.1 * sqrt(2.0), 1e-5);

  for (int iter = 0; iter < 10000; ++iter) {
    const double x = RandomDouble(-10, 10);
    const double y = RandomDouble(-10, 10);
    EXPECT_NEAR(poly1.DistanceTo({x, y}), box1.DistanceTo({x, y}), 1e-5);
  }
  for (int iter = 0; iter < 100; ++iter) {
    const double center_x = RandomDouble(-10, 10);
    const double center_y = RandomDouble(-10, 10);
    const double heading = RandomDouble(0, M_PI * 2.0);
    const double length = RandomDouble(1, 5);
    const double width = RandomDouble(1, 5);
    const Box2d box({center_x, center_y}, heading, length, width);
    const Polygon2d polygon(box);
    for (int iter2 = 0; iter2 < 100; ++iter2) {
      const double x = RandomDouble(-20, 20);
      const double y = RandomDouble(-20, 20);
      EXPECT_NEAR(polygon.DistanceTo({x, y}), box.DistanceTo({x, y}), 1e-5);
    }
  }

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_NEAR(poly2.DistanceTo({0, 0}), 0.0, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({1.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({0, -1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({-1.1, 0}), 0.1, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({0.5, 0.5}), 0.0, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({0.6, 0.6}), 0.1 * sqrt(2.0), 1e-5);

  const Polygon2d poly3(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  EXPECT_NEAR(poly3.DistanceTo({-0.5, 2.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({0.5, 2.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({1.5, 2.0}), 0.5 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.0, 2.0}), 1.0 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.5, 2.0}), 0.5 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({3.5, 2.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({4.5, 2.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({-0.5, 1.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.0, 1.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({4.5, 1.0}), 0.5, 1e-5);
}

TEST(Polygon2dTest, DistanceToLineSegment) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 0.5}, {1.0, 1.0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-0.2, 0.5}, {1.2, 0.5}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-2.0, -2.0}, {2.0, 2.0}}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-0.2, 0.5}, {-0.2, 0.8}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.2, 0.5}, {1.2, 0.3}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, -0.2}, {0.8, -0.2}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 1.2}, {0.3, 1.2}}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-0.3, 0.5}, {-0.2, 0.8}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.2, 0.5}, {1.3, 0.3}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, -0.3}, {0.8, -0.2}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 1.2}, {0.3, 1.3}}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{0, -0.1}, {-0.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.0, 1.1}, {1.1, 1.0}}), 0.1 / sqrt(2.0),
              1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-10.0, 0.5}, {2.0, 0.5}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-1.0, 0.5}, {10.0, 0.5}}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-1.0, 2.0}, {-1.0, 2.0}}), sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 0.5}, {0.5, 0.5}}), 0.0, 1e-5);

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_NEAR(poly2.DistanceTo({{-2, 0}, {2, 0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -2}, {0, 2}}), 0.0, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({{0, 1.1}, {1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, 1.1}, {-1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -1.1}, {1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -1.1}, {-1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0.6, 0.6}, {0.7, 0.7}}), 0.1 * sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{-0.6, -0.6}, {-0.7, -0.7}}), 0.1 * sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{-0.6, -0.6}, {0.7, 0.7}}), 0.0, 1e-5);

  const Polygon2d poly3({{0, 0}, {2, 0}, {2, 2}, {1, 1}, {0, 2}});
  EXPECT_NEAR(poly3.DistanceTo({{-2, 0}, {2, 0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 2.0}, {1.2, 2.0}}), 0.7 / sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 2.0}, {1.4, 2.0}}), 0.6 / sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 1.5}, {1.6, 1.5}}), 0.0, 1e-5);
}

TEST(Polygon2dTest, DistanceToPolygon) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  const Polygon2d poly3(Box2d::CreateAABox({2, 2}, {3, 3}));
  const Polygon2d poly4(Box2d::CreateAABox({-10, -10}, {10, 10}));

  EXPECT_NEAR(poly1.DistanceTo(poly2), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo(poly3), sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo(poly3), 1.5 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly1), 0.0, 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly2), 0.0, 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly3), 0.0, 1e-5);
}

TEST(Polygon2dTest, ContainPolygon) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {3, 3}));
  const Polygon2d poly2(Box2d::CreateAABox({1, 1}, {2, 2}));
  const Polygon2d poly3(Box2d::CreateAABox({1.5, 1.5}, {4, 4}));
  const Polygon2d poly4(Box2d::CreateAABox({-10, -10}, {10, 10}));
  EXPECT_TRUE(poly1.Contains(poly2));
  EXPECT_FALSE(poly2.Contains(poly1));

  EXPECT_FALSE(poly1.Contains(poly3));
  EXPECT_FALSE(poly2.Contains(poly3));
  EXPECT_FALSE(poly3.Contains(poly1));
  EXPECT_FALSE(poly3.Contains(poly2));

  EXPECT_TRUE(poly4.Contains(poly1));
  EXPECT_TRUE(poly4.Contains(poly2));
  EXPECT_TRUE(poly4.Contains(poly3));

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  const Polygon2d poly6({{0, 1}, {4, 1}, {4, 2}, {0, 2}});
  const Polygon2d poly7({{0, 1}, {1, 1}, {1, 2}, {0, 2}});
  const Polygon2d poly8({{3, 1}, {4, 1}, {4, 2}, {3, 2}});
  const Polygon2d poly9({{0, 0}, {4, 0}, {4, 1}, {0, 1}});
  EXPECT_FALSE(poly5.Contains(poly6));
  EXPECT_TRUE(poly5.Contains(poly7));
  EXPECT_TRUE(poly5.Contains(poly8));
  EXPECT_TRUE(poly5.Contains(poly9));
}

TEST(Polygon2dTest, ConvexHull) {
  Polygon2d polygon;
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({}, &polygon));
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({{1, 2}}, &polygon));
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({{3, 4}, {5, 6}}, &polygon));
  EXPECT_FALSE(
      Polygon2d::ComputeConvexHull({{3, 4}, {3, 4}, {5, 6}, {5, 6}}, &polygon));

  EXPECT_TRUE(Polygon2d::ComputeConvexHull({{0, 0}, {0, 4}, {3, 0}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(6.0, polygon.area(), 1e-5);

  EXPECT_TRUE(
      Polygon2d::ComputeConvexHull({{0, 0}, {0, 4}, {3, 0}, {3, 4}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(12.0, polygon.area(), 1e-5);

  EXPECT_TRUE(Polygon2d::ComputeConvexHull(
      {{0, 0}, {2, 2}, {1, 1}, {0, 4}, {3, 0}, {3, 4}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(12.0, polygon.area(), 1e-5);

  EXPECT_TRUE(Polygon2d::ComputeConvexHull(
      {{0, 0}, {0, 4}, {0, 1}, {0, 3}, {0, 2}, {1, 0}, {3, 0}, {2, 0}},
      &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(6.0, polygon.area(), 1e-5);

  for (int iter = 0; iter < 10000; ++iter) {
    const int kRange = 10;
    const int n = RandomInt(3, 10);
    std::vector<Vec2d> points;
    for (int i = 0; i < n; ++i) {
      points.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    double area = 0;
    for (int x0 = 0; x0 < kRange; ++x0) {
      double min_y = 0.0;
      double max_y = 0.0;
      if (ProjectByXSlow(points, x0 + 0.5, &min_y, &max_y)) {
        area += max_y - min_y;
      }
    }
    Polygon2d polygon;
    if (area < 1e-3) {
      EXPECT_FALSE(Polygon2d::ComputeConvexHull(points, &polygon));
    } else {
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points, &polygon));
      EXPECT_NEAR(area, polygon.area(), 1e-5);
    }
  }
}

TEST(Polygon2dTest, MergePolygon) {
  Polygon2d polygon1;
  EXPECT_TRUE(
      Polygon2d::ComputeConvexHull({{0, 0}, {0, 4}, {3, 0}}, &polygon1));
  EXPECT_TRUE(polygon1.is_convex());
  EXPECT_NEAR(6.0, polygon1.area(), 1e-5);

  Polygon2d polygon2;
  EXPECT_TRUE(
      Polygon2d::ComputeConvexHull({{0, 0}, {3, 4}, {3, 0}}, &polygon2));
  EXPECT_TRUE(polygon2.is_convex());
  EXPECT_NEAR(6.0, polygon2.area(), 1e-5);

  Polygon2d merged_polygon = Polygon2d::MergeTwoPolygons(polygon1, polygon2);
  EXPECT_TRUE(merged_polygon.is_convex());
  EXPECT_NEAR(12.0, merged_polygon.area(), 1e-5);

  Box2d box1({0, 0}, 0, 4, 2);
  Box2d box2({1, 1}, 0, 4, 2);
  merged_polygon = Polygon2d::MergeTwoBoxes(box1, box2);
  EXPECT_TRUE(merged_polygon.is_convex());
  EXPECT_NEAR(14.0, merged_polygon.area(), 1e-5);

  merged_polygon = Polygon2d::MergeBoxes({box1, box2});
  EXPECT_TRUE(merged_polygon.is_convex());
  EXPECT_NEAR(14.0, merged_polygon.area(), 1e-5);
}

TEST(Polygon2dTest, PolygonOverlap1) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {2, 2}));
  const Polygon2d poly2(Box2d::CreateAABox({1, 1}, {3, 3}));
  const Polygon2d poly3(Box2d::CreateAABox({2, 0}, {4, 2}));
  const Polygon2d poly4(Box2d({2, 2}, M_PI_4, sqrt(2.0), sqrt(2.0)));
  Polygon2d overlap_polygon;

  EXPECT_TRUE(poly1.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);
  EXPECT_TRUE(poly2.ComputeOverlap(poly1, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);

  EXPECT_TRUE(poly2.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);
  EXPECT_TRUE(poly3.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);

  EXPECT_FALSE(poly1.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_FALSE(poly3.ComputeOverlap(poly1, &overlap_polygon));

  EXPECT_TRUE(poly1.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly1, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);

  EXPECT_TRUE(poly2.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);

  EXPECT_TRUE(poly3.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);

  Vec2d first_intersect;
  Vec2d last_intersect;
  EXPECT_FALSE(poly1.GetOverlap(Segment2d({-1, 0}, {-1, 2}), &first_intersect,
                                &last_intersect));
  EXPECT_FALSE(poly1.GetOverlap(Segment2d({-1, 1}, {-3, 1}), &first_intersect,
                                &last_intersect));
  EXPECT_FALSE(poly1.GetOverlap(Segment2d({1, 3}, {1, 5}), &first_intersect,
                                &last_intersect));

  EXPECT_TRUE(poly1.GetOverlap(Segment2d({1, -1}, {1, 3}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(0.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(Segment2d({1, 1}, {1, 3}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(Segment2d({1, -1}, {1, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(0.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(Segment2d({1, 3}, {3, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(2.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_FALSE(poly1.GetOverlap(Segment2d({4, 3}, {4, 3}), &first_intersect,
                                &last_intersect));
  EXPECT_TRUE(poly1.GetOverlap(Segment2d({1, 1}, {1, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.y(), 1e-5);

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  std::vector<Segment2d> overlap_line_segments =
      poly5.GetAllOverlaps(Segment2d({-10, 1.5}, {10, 1.5}));
  EXPECT_EQ(2, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].end().y(), 1e-5);
  EXPECT_NEAR(2.5, overlap_line_segments[1].start().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[1].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[1].end().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[1].end().y(), 1e-5);

  overlap_line_segments = poly5.GetAllOverlaps(Segment2d({-10, 1}, {10, 1}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(1.0, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(1.0, overlap_line_segments[0].end().y(), 1e-5);

  overlap_line_segments =
      poly5.GetAllOverlaps(Segment2d({-10, 0.5}, {10, 0.5}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].end().y(), 1e-5);

  overlap_line_segments =
      poly5.GetAllOverlaps(Segment2d({-10, -0.5}, {10, -0.5}));
  EXPECT_EQ(0, overlap_line_segments.size());
  overlap_line_segments =
      poly5.GetAllOverlaps(Segment2d({-10, 2.5}, {10, 2.5}));
  EXPECT_EQ(0, overlap_line_segments.size());

  overlap_line_segments = poly5.GetAllOverlaps(Segment2d({2, 0.5}, {2, 0.5}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(2.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(2.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].end().y(), 1e-5);
  overlap_line_segments = poly5.GetAllOverlaps(Segment2d({5, 0.5}, {5, 0.5}));
  EXPECT_EQ(0, overlap_line_segments.size());
}

TEST(Polygon2dTest, PolygonOverlap2) {
  Polygon2d overlap_polygon;

  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{1, 1}, {1, 3}, {3, 1}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);

  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{1, 1}, {-1, 1}, {1, 3}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.5, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{2, 1}, {-1, 1}, {2, 4}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.0, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{3, 1}, {-1, 1}, {3, 5}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.5, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{4, 1}, {-1, 1}, {4, 6}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.5, 1e-5);

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
    Box2d box1({x1, y1}, heading1, l1, w1);
    Box2d box2({x2, y2}, heading2, l2, w2);
    Box2d shrinked_box2({x2, y2}, heading2, l2 - 0.2, w2 - 0.2);
    Box2d extended_box2({x2, y2}, heading2, l2 + 0.2, w2 + 0.2);
    if (!box1.HasOverlap(extended_box2)) {
      EXPECT_FALSE(
          Polygon2d(box1).ComputeOverlap(Polygon2d(box2), &overlap_polygon));
    } else if (box1.HasOverlap(shrinked_box2)) {
      EXPECT_TRUE(
          Polygon2d(box1).ComputeOverlap(Polygon2d(box2), &overlap_polygon));
    }
  }

  for (int iter = 0; iter < 10000; ++iter) {
    const int kRange = 10;
    const int n1 = RandomInt(3, 10);
    const int n2 = RandomInt(3, 10);
    std::vector<Vec2d> points1, points2;
    for (int i = 0; i < n1; ++i) {
      points1.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    for (int i = 0; i < n2; ++i) {
      points2.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    Polygon2d polygon1;
    Polygon2d polygon2;
    if (!Polygon2d::ComputeConvexHull(points1, &polygon1) ||
        !Polygon2d::ComputeConvexHull(points2, &polygon2)) {
      continue;
    }
    std::vector<double> key_points;
    for (int x0 = 0; x0 <= kRange; ++x0) {
      key_points.push_back(x0);
    }
    for (const auto &line_segment1 : polygon1.line_segments()) {
      for (const auto &line_segment2 : polygon2.line_segments()) {
        Vec2d pt;
        if (line_segment1.GetIntersect(line_segment2, &pt)) {
          key_points.push_back(pt.x());
        }
      }
    }
    double area = 0;
    std::sort(key_points.begin(), key_points.end());
    for (size_t i = 0; i + 1 < key_points.size(); ++i) {
      const double width = key_points[i + 1] - key_points[i];
      if (width < 1e-6) {
        continue;
      }
      const double x = (key_points[i] + key_points[i + 1]) / 2.0;
      double min_y1 = 0.0;
      double max_y1 = 0.0;
      double min_y2 = 0.0;
      double max_y2 = 0.0;
      if (ProjectByXSlow(points1, x, &min_y1, &max_y1) &&
          ProjectByXSlow(points2, x, &min_y2, &max_y2)) {
        area +=
            std::max(0.0, std::min(max_y1, max_y2) - std::max(min_y1, min_y2)) *
            width;
      }
    }
    Polygon2d overlap_polygon;
    if (area < 1e-3) {
      EXPECT_FALSE(polygon1.ComputeOverlap(polygon2, &overlap_polygon));
    } else {
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points1, &polygon1));
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points2, &polygon2));
      EXPECT_TRUE(polygon1.ComputeOverlap(polygon2, &overlap_polygon));
      EXPECT_NEAR(area, overlap_polygon.area(), 1e-5);
    }
  }
}

TEST(Polygon2dTest, BoundingBox) {
  Polygon2d poly1(Box2d::CreateAABox({0, 0}, {2, 2}));
  Box2d box = poly1.BoundingBoxWithHeading(0.0);
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));
  AABox2d aabox = poly1.AABoundingBox();
  EXPECT_NEAR(1.0, aabox.center().x(), 1e-5);
  EXPECT_NEAR(1.0, aabox.center().y(), 1e-5);
  EXPECT_NEAR(4.0, aabox.area(), 1e-5);
  EXPECT_NEAR(2.0, aabox.length(), 1e-5);
  EXPECT_NEAR(2.0, aabox.width(), 1e-5);

  box = poly1.BoundingBoxWithHeading(M_PI_4);
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(8.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));

  box = poly1.MinAreaBoundingBox();
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));

  Polygon2d poly2({{1, 0}, {0, 1}, {-1, 0}, {0, -1}});
  box = poly2.BoundingBoxWithHeading(0.0);
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));
  aabox = poly2.AABoundingBox();
  EXPECT_NEAR(0.0, aabox.center().x(), 1e-5);
  EXPECT_NEAR(0.0, aabox.center().y(), 1e-5);
  EXPECT_NEAR(4.0, aabox.area(), 1e-5);
  EXPECT_NEAR(2.0, aabox.length(), 1e-5);
  EXPECT_NEAR(2.0, aabox.width(), 1e-5);

  box = poly2.BoundingBoxWithHeading(M_PI_4);
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(2.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));

  box = poly2.MinAreaBoundingBox();
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(2.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));

  for (int iter = 0; iter < 1000; ++iter) {
    const int num_sample_points = RandomInt(3, 10);
    std::vector<Vec2d> points;
    for (int i = 0; i < num_sample_points; ++i) {
      const double x = RandomInt(-10, 10);
      const double y = RandomInt(-10, 10);
      points.emplace_back(x, y);
    }
    Polygon2d polygon;
    if (!Polygon2d::ComputeConvexHull(points, &polygon)) {
      continue;
    }
    double min_area = std::numeric_limits<double>::infinity();
    for (int iter2 = 0; iter2 < 10; ++iter2) {
      const double heading = RandomDouble(0, M_PI * 2.0);
      box = polygon.BoundingBoxWithHeading(heading);
      EXPECT_TRUE(Polygon2d(box).Contains(polygon));
      min_area = std::min(min_area, box.area());
    }
    box = polygon.MinAreaBoundingBox();
    EXPECT_TRUE(Polygon2d(box).Contains(polygon));
    EXPECT_LE(box.area(), min_area + 1e-5);
  }
}

TEST(Polygon2dTest, Expand) {
  {
    const Polygon2d poly(Box2d::CreateAABox({0, 0}, {2, 2}));
    const Polygon2d exp_poly = poly.ExpandByDistance(1.0);
    EXPECT_TRUE(exp_poly.is_convex());
    const Box2d box = exp_poly.BoundingBoxWithHeading(0.0);
    EXPECT_NEAR(box.center().x(), 1.0, 1e-6);
    EXPECT_NEAR(box.center().y(), 1.0, 1e-6);
    EXPECT_NEAR(box.width(), 4.0, 1e-6);
    EXPECT_NEAR(box.length(), 4.0, 1e-6);
    EXPECT_NEAR(exp_poly.area(), 12 + M_PI, 0.1);
  }
  {
    const std::vector<Vec2d> points{{0, 0}, {2, 0}, {2, 2}, {0, 2}, {1, 1}};
    const Polygon2d poly(points);
    const Polygon2d exp_poly = poly.ExpandByDistance(1.0);
    EXPECT_TRUE(exp_poly.is_convex());
    const Box2d box = exp_poly.BoundingBoxWithHeading(0.0);
    EXPECT_NEAR(box.center().x(), 1.0, 1e-6);
    EXPECT_NEAR(box.center().y(), 1.0, 1e-6);
    EXPECT_NEAR(box.width(), 4.0, 1e-6);
    EXPECT_NEAR(box.length(), 4.0, 1e-6);
    EXPECT_NEAR(exp_poly.area(), 12 + M_PI, 0.1);
  }
}

TEST(Polygon2dTest, TestComputeOverlapDegenerate) {
  const std::vector<Vec2d> points1 = {
      {-88.4000030681, -52.2000022903}, {-88.0000030622, -52.6000022963},
      {-87.2000030503, -53.2000023052}, {-86.6000030413, -53.6000023112},
      {-86.0000030324, -53.8000023142}, {-85.8000030294, -53.8000023142},
      {-85.8000030294, -53.6000023112}, {-87.0000030473, -52.2000022903},
      {-87.2000030503, -52.0000022873}, {-88.0000030622, -51.8000022843},
      {-88.4000030681, -51.8000022843},
  };
  EXPECT_TRUE(Polygon2d::AreConvexHullPoints(points1));
  const std::vector<Vec2d> points2 = {
      {-88.4000015631, -52.7999999747}, {-88.2000015602, -53.3999999836},
      {-87.4000015482, -54.1999999955}, {-87.0000015423, -54.3999999985},
      {-86.0000015274, -54.6000000015}, {-85.8000015244, -54.6000000015},
      {-85.8000015244, -54.3999999985}, {-88.0000015572, -52.5999999717},
      {-88.4000015631, -52.5999999717},
  };
  EXPECT_TRUE(Polygon2d::AreConvexHullPoints(points2));
  const Polygon2d polygon1(points1);
  const Polygon2d polygon2(points2);
  Polygon2d overlap;
  EXPECT_FALSE(polygon1.ComputeOverlap(polygon2, &overlap));
}

TEST(Polygon2dTest, Transform) {
  constexpr double kEpsilon = 1e-8;
  Polygon2d box(Box2d(Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), 4.0, 2.0));
  EXPECT_THAT(box.points(), UnorderedElementsAre(
                                Vec2dEqXY(2.0, 1.0), Vec2dEqXY(-2.0, 1.0),
                                Vec2dEqXY(-2.0, -1.0), Vec2dEqXY(2.0, -1.0)));

  // Rotate 90 degrees counterclockwise.
  const auto r1 = box.Transform(/*center*/ Vec2d(0.0, 0.0),
                                /*cos_angle=*/0.0, /*sin_angle=*/1.0,
                                /*translation=*/Vec2d::Zero());
  EXPECT_THAT(r1.points(),
              UnorderedElementsAre(Vec2dNearXY(1.0, 2.0, kEpsilon),
                                   Vec2dNearXY(-1.0, 2.0, kEpsilon),
                                   Vec2dNearXY(-1.0, -2.0, kEpsilon),
                                   Vec2dNearXY(1.0, -2.0, kEpsilon)));

  // Rotate around the box's left side center point 90 degrees counterclockwise.
  const auto r2 = box.Transform(/*center*/ Vec2d(0.0, 1.0),
                                /*cos_angle=*/0.0, /*sin_angle=*/1.0,
                                /*translation=*/Vec2d::Zero());
  EXPECT_THAT(r2.points(),
              UnorderedElementsAre(Vec2dNearXY(0.0, 3.0, kEpsilon),
                                   Vec2dNearXY(0.0, -1.0, kEpsilon),
                                   Vec2dNearXY(2.0, -1.0, kEpsilon),
                                   Vec2dNearXY(2.0, 3.0, kEpsilon)));

  // Rotate around the box's left side center point 90 degrees counterclockwise,
  // and shift (1.0, 1.0).
  const auto r3 = box.Transform(/*center*/ Vec2d(0.0, 1.0),
                                /*cos_angle=*/0.0, /*sin_angle=*/1.0,
                                /*translation=*/Vec2d(1.0, 1.0));
  EXPECT_THAT(r3.points(),
              UnorderedElementsAre(Vec2dNearXY(1.0, 4.0, kEpsilon),
                                   Vec2dNearXY(1.0, 0.0, kEpsilon),
                                   Vec2dNearXY(3.0, 0.0, kEpsilon),
                                   Vec2dNearXY(3.0, 4.0, kEpsilon)));
}

TEST(ToPolygon2d, FromContainer) {
  std::vector<Vec2dProto> proto_points;
  auto polygon = Polygon2d::FromPoints(proto_points, /*is_convex=*/true);
  ASSERT_FALSE(polygon.has_value());

  for (const Vec2d &point : {Vec2d{1.0, 1.0}, Vec2d{-1.0, 1.0},
                             Vec2d{-1.0, -1.0}, Vec2d{1.0, -1.0}}) {
    Vec2dProto proto;
    proto.set_x(point.x());
    proto.set_y(point.y());
    proto_points.push_back(proto);
  }

  polygon = Polygon2d::FromPoints(proto_points, /*is_convex=*/true);
  ASSERT_TRUE(polygon.has_value());

  EXPECT_THAT(polygon->points(),
              ElementsAre(Vec2dEqXY(1.0, 1.0), Vec2dEqXY(-1.0, 1.0),
                          Vec2dEqXY(-1.0, -1.0), Vec2dEqXY(1.0, -1.0)));
}
TEST(Polygon2dTest, TestDuplicatePointRemovialLogic) {
  const std::vector<Vec2d> points = {
      {-88.4000030681, -52.2000022903}, {-88.0000030622, -52.6000022963},
      {-87.2000030503, -53.2000023052}, {-86.6000030413, -53.6000023112},
      {-86.0000030324, -53.8000023142}, {-85.8000030294, -53.8000023142},
      {-85.8000030294, -53.6000023112}, {-87.0000030473, -52.2000022903},
      {-87.2000030503, -52.0000022873}, {-88.0000030622, -51.8000022843},
      {-88.4000030681, -51.8000022843}, {-88.4000030681, -51.8000022843},
      {-88.4000030681, -51.8000022843}};
  EXPECT_FALSE(Polygon2d::AreConvexHullPoints(points));
  std::vector<Vec2d> convex_hull_points;
  Polygon2d::ComputeConvexHullPoints(points, &convex_hull_points);
  EXPECT_TRUE(Polygon2d::AreConvexHullPoints(convex_hull_points));
}

TEST(Polygon2dTest, IsSelfIntersecting) {
  const std::vector<Vec2d> intersecting_points = {
      {1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}, {-1.0, 1.0}};
  const Polygon2d self_intersecting_polygon(intersecting_points,
                                            /*is_convex=*/false);
  EXPECT_TRUE(self_intersecting_polygon.IsSelfIntersecting());

  std::vector<Vec2d> non_intersecting_points = {
      {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}};
  const Polygon2d polygon(non_intersecting_points, /*is_convex=*/true);
  EXPECT_FALSE(polygon.IsSelfIntersecting());
}

}  // namespace
}  // namespace qcraft
