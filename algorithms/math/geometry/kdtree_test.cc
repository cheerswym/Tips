#include "onboard/math/geometry/kdtree.h"

#include <random>

#include "gtest/gtest.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace {

constexpr int kNumPoints = 10000;

std::vector<Vec3d> GenerateRandomPoints() {
  std::mt19937 gen({});
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  std::vector<Vec3d> points;
  for (int i = 0; i < kNumPoints; ++i) {
    points.emplace_back(dis(gen), dis(gen), dis(gen));
  }
  return points;
}

TEST(KDTreeTest, TestRandomPoints) {
  const std::vector<Vec3d> points = GenerateRandomPoints();
  const KDTree<Vec3d> kd_tree(points);

  double best_dist_sqr;
  kd_tree.FindNearest({0.0, 0.0, 0.0}, &best_dist_sqr);
  for (const auto& point : points) {
    EXPECT_LE(best_dist_sqr, point.squaredNorm());
  }
}

TEST(KDTreeTest, TestDuplicatePoints) {
  std::vector<Vec3d> points = GenerateRandomPoints();
  points.reserve(points.size() * 2);
  std::copy(points.begin(), points.end(), std::back_inserter(points));
  const KDTree<Vec3d> kd_tree(points);

  EXPECT_EQ(points.size(), kd_tree.size());

  double best_dist_sqr;
  kd_tree.FindNearest({0.0, 0.0, 0.0}, &best_dist_sqr);
  for (const auto& point : points) {
    EXPECT_LE(best_dist_sqr, point.squaredNorm());
  }
}

TEST(KDTreeTest, TestExistingPoint) {
  const std::vector<Vec3d> points = GenerateRandomPoints();
  const KDTree<Vec3d> kd_tree(points);

  for (const auto& point : points) {
    double best_dist_sqr;
    kd_tree.FindNearest(point, &best_dist_sqr);
    EXPECT_EQ(0.0, best_dist_sqr);
  }
}

TEST(KDTreeTest, TestKNN) {
  std::vector<Vec3d> points = GenerateRandomPoints();
  points.reserve(points.size() * 2);
  std::copy(points.begin(), points.end(), std::back_inserter(points));
  const KDTree<Vec3d> kd_tree(points);

  for (int k = 1; k < 20; ++k) {
    const std::set<int> nearest_indices =
        kd_tree.FindKNearest({0.0, 0.0, 0.0}, k);
    EXPECT_EQ(nearest_indices.size(), k);
    for (int i = 0; i < points.size(); ++i) {
      const auto& point = kd_tree.PointAt(i);
      if (nearest_indices.count(i) > 0) continue;
      for (const int index : nearest_indices) {
        EXPECT_LE(kd_tree.PointAt(index).squaredNorm(), point.squaredNorm());
      }
    }
  }
}

TEST(KDTreeTest, TestRadiusQuery) {
  std::vector<Vec3d> points = {
      {1.0, 0.0, 0.0}, {0.1, 0.1, 0.1}, {0.5, -0.5, -0.5}, {0.1, 0.1, 0.1}};
  const KDTree<Vec3d> kd_tree(points);
  const Vec3d query_center(0.0, 0.0, 0.0);

  const auto all_indices = kd_tree.FindNearestInRadius(query_center, 1.0);
  EXPECT_EQ(all_indices.size(), 4);

  const auto near_indices = kd_tree.FindNearestInRadius(query_center, 0.9);
  EXPECT_EQ(near_indices.size(), 3);

  const auto original_indices = kd_tree.GetOriginalIndices(near_indices);
  EXPECT_EQ(original_indices.size(), 3);
  EXPECT_FALSE(original_indices.count(0));
  EXPECT_TRUE(original_indices.count(1));
  EXPECT_TRUE(original_indices.count(2));
  EXPECT_TRUE(original_indices.count(3));
}

}  // namespace
}  // namespace qcraft
