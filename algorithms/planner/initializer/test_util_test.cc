#include "onboard/planner/initializer/test_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/planner/initializer/motion_graph.h"

namespace qcraft {
namespace planner {

namespace {

using testing::ElementsAre;

constexpr double kEpsilon = 1e-8;

TEST(BuildConstAccelLineGraph, ConstSpeed) {
  const auto [geom_graph, motion_graph] = BuildConstAccelLineGraph(
      {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
      /*init_speed=*/1.0, /*init_time=*/2.0, /*accel=*/0.0);
  ASSERT_EQ(motion_graph->node_size(), 3);
  ASSERT_EQ(motion_graph->edge_size(), 2);

  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(0)).state.v, 1.0,
              kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(0)).state.t, 2.0,
              kEpsilon);

  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(1)).state.v, 1.0,
              kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(1)).state.t, 3.0,
              kEpsilon);

  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(2)).state.v, 1.0,
              kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(2)).state.t, 4.0,
              kEpsilon);
}

TEST(BuildConstAccelLineGraph, ConstAccel) {
  const auto [geom_graph, motion_graph] =
      BuildConstAccelLineGraph({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
                               /*init_speed=*/1.0, /*init_time=*/2.0,
                               /*accel=*/1.0);
  ASSERT_EQ(motion_graph->node_size(), 3);
  ASSERT_EQ(motion_graph->edge_size(), 2);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(0)).state.t, 2.0,
              kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(0)).state.v, 1.0,
              kEpsilon);

  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(1)).state.t,
              1.0 + std::sqrt(3.0), kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(1)).state.v,
              std::sqrt(3.0), kEpsilon);

  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(2)).state.t,
              1.0 + std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(motion_graph->GetMotionNode(MotionNodeIndex(2)).state.v,
              std::sqrt(5.0), kEpsilon);

  ASSERT_EQ(motion_graph->GetOutgoingEdges(MotionNodeIndex(0)).size(), 1);
  EXPECT_EQ(motion_graph->GetOutgoingEdges(MotionNodeIndex(0))[0],
            MotionEdgeIndex(0));

  ASSERT_EQ(motion_graph->GetOutgoingEdges(MotionNodeIndex(1)).size(), 1);
  EXPECT_EQ(motion_graph->GetOutgoingEdges(MotionNodeIndex(1))[0],
            MotionEdgeIndex(1));

  EXPECT_EQ(motion_graph->GetMotionEdge(MotionEdgeIndex(0)).start,
            MotionNodeIndex(0));
  EXPECT_EQ(motion_graph->GetMotionEdge(MotionEdgeIndex(1)).start,
            MotionNodeIndex(1));
}

}  // namespace

}  // namespace planner
}  // namespace qcraft
