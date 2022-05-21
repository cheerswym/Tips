#include "onboard/perception/obstacle_util.h"

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <set>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft::obstacle_util {

TEST(NeighborRangeTest, TestGetNeighborOffsets) {
  NeighborRange nr1(8);
  NeighborRange nr2([](const Obstacle& obstacle) { return 1; });

  const auto offsets1 = nr1.GetNeighborOffsets();
  Obstacle obstacle;
  const auto offsets2 = nr2.GetNeighborOffsets(&obstacle);

  EXPECT_EQ(offsets1.size(), offsets2.size());

  for (int i = 0; i < offsets1.size(); ++i) {
    EXPECT_EQ(offsets1[i].first, offsets2[i].first);
    EXPECT_EQ(offsets1[i].second, offsets2[i].second);
  }

  NeighborRange nr3([](const Obstacle& obstacle) { return 0; });
  EXPECT_EQ(nr3.GetNeighborOffsets(&obstacle).size(), 0);
}

TEST(LocalObstacleGridTest, TestFindNearestInRadius) {
  constexpr uint16_t kGridHeight = 8;
  constexpr uint16_t kGridWidth = 8;
  /*
  | | | | | | | | |
  | | | |*| | |*| |
  | | | | | | | | |
  | | | |*| | |*| |
  | | | | | | | | |
  | | | |*| | |*| |
  | | | | | | | | |
  | | | | | | | | |
  */
  std::vector<Obstacle> obstacles;
  for (uint16_t i = 2; i < kGridHeight; ++i) {
    for (uint16_t j = 3; j < kGridWidth; ++j) {
      if (i % 2 == 0 && j % 3 == 0) {
        obstacles.push_back(ObstacleBuilder().set_col(j).set_row(i).Build());
      }
    }
  }
  obstacle_util::LocalObstacleGrid local_obstacle_grid(
      ConstructObstaclePtrsFromObstacles(obstacles));

  std::vector<int> indices;
  for (uint16_t i = 2; i < kGridHeight; ++i) {
    for (uint16_t j = 3; j < kGridWidth; ++j) {
      if (i % 2 == 0 && j % 3 == 0) {
        indices = local_obstacle_grid.FindNearestInRadius(i, j, 0);
        EXPECT_EQ(indices.size(), 1);
      }
    }
  }
  for (uint16_t i = 2; i < kGridHeight; ++i) {
    for (uint16_t j = 3; j < kGridWidth; ++j) {
      if (i % 2 != 0 && j % 3 != 0) {
        indices = local_obstacle_grid.FindNearestInRadius(i, j, 0);
        EXPECT_EQ(indices.size(), 0);
      }
    }
  }
  indices = local_obstacle_grid.FindNearestInRadius(0, 0, 1);
  EXPECT_EQ(indices.size(), 0);
  indices = local_obstacle_grid.FindNearestInRadius(0, 0, 2);
  EXPECT_EQ(indices.size(), 0);
  indices = local_obstacle_grid.FindNearestInRadius(0, 0, 3);
  EXPECT_EQ(indices.size(), 1);
  EXPECT_EQ(indices[0], 0);
  indices = local_obstacle_grid.FindNearestInRadius(4, 4, 0);
  EXPECT_EQ(indices.size(), 0);
  indices = local_obstacle_grid.FindNearestInRadius(4, 4, 1);
  EXPECT_EQ(indices.size(), 1);
  indices = local_obstacle_grid.FindNearestInRadius(4, 4, 2);
  EXPECT_EQ(indices.size(), 6);
  indices = local_obstacle_grid.FindNearestInRadius(4, 4, 1000);
  EXPECT_EQ(indices.size(), 6);
  indices = local_obstacle_grid.FindNearestInRadius(5, 0, 1);
  EXPECT_EQ(indices.size(), 0);
  indices = local_obstacle_grid.FindNearestInRadius(7, 7, 7);
  EXPECT_EQ(indices.size(), 6);
  for (int i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(indices[i], i);
  }
  EXPECT_EQ(local_obstacle_grid.DebugString(),
            "min_vertex: (2, 3) max_vertex: (6, 6) width: 4 height: 5");
}

TEST(LocalObstacleGridTest, TestFindConnectedNeighbors) {
  /*
  | | | | | | | | |
  | | | | |o|o|o|o|
  | | | | |o|o|o|o|
  | | | | |o|o|o|o|
  | | | |x|o|o|o|o|
  |*|*|*| | | | | |
  |*|*|*| | | | | |
  |*|*|*| | | | | |
  */
  std::vector<Obstacle> obstacles;
  for (uint16_t i = 0; i < 3; ++i) {
    for (uint16_t j = 0; j < 3; ++j) {
      obstacles.push_back(ObstacleBuilder()
                              .set_col(j)
                              .set_row(i)
                              .set_type(ObstacleProto::STATIC)
                              .Build());
    }
  }
  for (uint16_t i = 3; i < 7; ++i) {
    for (uint16_t j = 4; j < 8; ++j) {
      obstacles.push_back(ObstacleBuilder()
                              .set_col(j)
                              .set_row(i)
                              .set_type(ObstacleProto::DYNAMIC)
                              .Build());
    }
  }
  obstacles.push_back(ObstacleBuilder()
                          .set_col(3)
                          .set_row(3)
                          .set_type(ObstacleProto::CONE)
                          .Build());

  const auto obstacle_ptrs = ConstructObstaclePtrsFromObstacles(obstacles);

  obstacle_util::LocalObstacleGrid local_obstacle_grid(obstacle_ptrs);

  std::vector<int> indices;
  for (uint16_t i = 0; i < 3; ++i) {
    for (uint16_t j = 0; j < 3; ++j) {
      indices = local_obstacle_grid.FindConnectedNeighbors(
          i, j, 4,
          [](const Obstacle& init, const Obstacle& seed,
             const Obstacle& neighbor) { return init.type == neighbor.type; });
      EXPECT_EQ(9, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(
          i, j, 8,
          [](const Obstacle& init, const Obstacle& seed,
             const Obstacle& neighbor) { return init.type == neighbor.type; });
      EXPECT_EQ(9, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 4);
      EXPECT_EQ(9, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 8);
      EXPECT_EQ(26, indices.size());
    }
  }
  for (uint16_t i = 3; i < 7; ++i) {
    for (uint16_t j = 4; j < 8; ++j) {
      indices = local_obstacle_grid.FindConnectedNeighbors(
          i, j, 4,
          [](const Obstacle& init, const Obstacle& seed,
             const Obstacle& neighbor) { return init.type == neighbor.type; });
      EXPECT_EQ(16, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(
          i, j, 8,
          [](const Obstacle& init, const Obstacle& seed,
             const Obstacle& neighbor) { return init.type == neighbor.type; });
      EXPECT_EQ(16, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 4);
      EXPECT_EQ(17, indices.size());
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 8);
      EXPECT_EQ(26, indices.size());
    }
  }
  indices = local_obstacle_grid.FindConnectedNeighbors(3, 3, 4);
  EXPECT_EQ(17, indices.size());
  indices = local_obstacle_grid.FindConnectedNeighbors(3, 3, 8);
  EXPECT_EQ(26, indices.size());
  indices = local_obstacle_grid.FindConnectedNeighbors(
      3, 3, 4,
      [](const Obstacle& init, const Obstacle& seed, const Obstacle& neighbor) {
        return init.type == neighbor.type;
      });
  EXPECT_EQ(1, indices.size());
  for (uint16_t i = 3; i < 8; ++i) {
    for (uint16_t j = 0; j < 3; ++j) {
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 4);
      EXPECT_EQ(0, indices.size());
    }
  }
  for (uint16_t i = 0; i < 3; ++i) {
    for (uint16_t j = 3; j < 8; ++j) {
      indices = local_obstacle_grid.FindConnectedNeighbors(i, j, 4);
      EXPECT_EQ(0, indices.size());
    }
  }

  std::vector<bool> processed(obstacle_ptrs.size(), false);
  std::vector<std::vector<int>> connected_components;
  for (int i = 0; i < obstacle_ptrs.size(); ++i) {
    if (processed[i]) continue;
    auto raw_indices = local_obstacle_grid.FindConnectedNeighbors(
        obstacle_ptrs[i]->row, obstacle_ptrs[i]->col, 4);
    for (const auto index : raw_indices) {
      processed[index] = true;
    }
    connected_components.emplace_back(std::move(raw_indices));
  }
  EXPECT_EQ(2, connected_components.size());
  connected_components.clear();
  std::fill(processed.begin(), processed.end(), false);
  for (int i = 0; i < obstacle_ptrs.size(); ++i) {
    if (processed[i]) continue;
    auto raw_indices = local_obstacle_grid.FindConnectedNeighbors(
        obstacle_ptrs[i]->row, obstacle_ptrs[i]->col, 8,
        [](const Obstacle& init, const Obstacle& seed,
           const Obstacle& neighbor) { return init.type == neighbor.type; });
    for (const auto index : raw_indices) {
      processed[index] = true;
    }
    connected_components.emplace_back(std::move(raw_indices));
  }
  EXPECT_EQ(3, connected_components.size());

  const int total_num_elems =
      std::accumulate(connected_components.begin(), connected_components.end(),
                      0, [](const int init, const std::vector<int>& component) {
                        return init + component.size();
                      });
  EXPECT_EQ(total_num_elems, obstacle_ptrs.size());

  std::vector<int> elements;
  for (const auto& component : connected_components) {
    elements.insert(elements.end(), component.begin(), component.end());
  }
  std::sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); ++i) {
    EXPECT_EQ(i, elements[i]);
  }
}

TEST(GetObstacleNearGroundMaxZTest, ShouldPass) {
  const float near_ground_max_z = GetObstacleNearGroundMaxZ(
      ObstacleBuilder()
          .set_ground_z(1.)
          .set_points(
              {{.range = 0.5 * (kMinNearGroundDist + kMaxNearGroundDist)}})
          .Build());
  EXPECT_NEAR(near_ground_max_z,
              1.0 + 0.5 * (kMinNearGroundHeight + kMaxNearGroundHeight), 1e-6);
}

TEST(GetObstacleNearGroundMaxZTest, TestOnEdge) {
  const float near_ground_max_z = GetObstacleNearGroundMaxZ(
      ObstacleBuilder().set_ground_z(1.).set_points({{.range = 0.}}).Build());
  EXPECT_NEAR(near_ground_max_z, 1.0 + kMinNearGroundHeight, 1e-6);
}

TEST(GetObstacleOverhangMinZTest, ShouldPass) {
  const float overhang_min_z = GetObstacleOverhangMinZ(
      ObstacleBuilder()
          .set_ground_z(1.)
          .set_points({{.range = 0.5 * (kOverhangNearDist + kOverhangFarDist)}})
          .Build(),
      2.28);
  EXPECT_NEAR(overhang_min_z,
              2.28 + 1.0 +
                  0.5 * (kOverhangMinAboveAvBuffer + kOverhangMaxAboveAvBuffer),
              1e-6);
}

TEST(GetObstacleOverhangMinZTest, TestOnEdge) {
  const float overhang_min_z = GetObstacleOverhangMinZ(
      ObstacleBuilder().set_ground_z(1.).set_points({{.range = 0.}}).Build(),
      2.28);
  EXPECT_NEAR(overhang_min_z, 2.28 + 1.0 + kOverhangMinAboveAvBuffer, 1e-6);
}

TEST(IsRealObstaclePointTest, ShouldPass) {
  EXPECT_TRUE(IsRealObstaclePoint(ObstacleBuilder()
                                      .set_x(1.0)
                                      .set_y(1.0)
                                      .set_ground_z(1.0)
                                      .set_points({{.range = 0.}})
                                      .Build(),
                                  {.x = 1.0, .y = 1.1, .z = 2.0}, 2.28));
}

TEST(IsRealObstaclePointTest, TestOnGroundPoints) {
  EXPECT_FALSE(IsRealObstaclePoint(ObstacleBuilder()
                                       .set_x(1.0)
                                       .set_y(1.0)
                                       .set_ground_z(1.0)
                                       .set_points({{.range = 0.}})
                                       .Build(),
                                   {.x = 1.0, .y = 1.1, .z = 0.0}, 2.28));
}

TEST(IsRealObstaclePointTest, TestOnOutOfRangePoints) {
  EXPECT_FALSE(IsRealObstaclePoint(ObstacleBuilder()
                                       .set_x(1.0)
                                       .set_y(1.0)
                                       .set_ground_z(1.0)
                                       .set_points({{.range = 0.}})
                                       .Build(),
                                   {.x = 1.0, .y = 1.12, .z = 2.0}, 2.28));
}

TEST(IsAboveGroundObstaclePointTest, ShouldPass) {
  EXPECT_TRUE(IsAboveGroundObstaclePoint(ObstacleBuilder()
                                             .set_x(1.0)
                                             .set_y(1.0)
                                             .set_ground_z(1.0)
                                             .set_points({{.range = 0.}})
                                             .Build(),
                                         {.x = 1.0, .y = 1.1, .z = 2.0}));
}

TEST(IsAboveGroundObstaclePointTest, TestOnGroundPoints) {
  EXPECT_FALSE(IsAboveGroundObstaclePoint(ObstacleBuilder()
                                              .set_x(1.0)
                                              .set_y(1.0)
                                              .set_ground_z(1.0)
                                              .set_points({{.range = 0.}})
                                              .Build(),
                                          {.x = 1.0, .y = 1.1, .z = 0.0}));
}

TEST(ComputeAvHeightTest, ShouldPassIfVehicleHeightExceedsLidarHeight) {
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  EXPECT_NEAR(ComputeAvHeight(run_params), 2.695, 1e-6);
}

TEST(ComputeAvHeightTest, ShouldPassIfLidarHeightExceedsVehicleHeight) {
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  EXPECT_NEAR(ComputeAvHeight(run_params), 2.0887777805328369, 1e-6);
}

TEST(ComputeGaussianKernel, ShouldPass) {
  const auto kernel_5 = ComputeGaussianKernel(5);
  const float gt[5][5] = {
      {0.00296902, 0.0133062, 0.0219382, 0.0133062, 0.00296902},
      {0.0133062, 0.0596343, 0.0983204, 0.0596343, 0.0133062},
      {0.0219382, 0.0983204, 0.162103, 0.0983204, 0.0219382},
      {0.0133062, 0.0596343, 0.0983204, 0.0596343, 0.0133062},
      {0.00296902, 0.0133062, 0.0219382, 0.0133062, 0.00296902}};
  EXPECT_EQ(kernel_5.size(), 5 * 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_NEAR(kernel_5[i * 5 + j], gt[i][j], 1e-6);
    }
  }
}

}  // namespace qcraft::obstacle_util
