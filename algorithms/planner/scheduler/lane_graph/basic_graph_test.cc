#include "onboard/planner/scheduler/lane_graph/basic_graph.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(BasicGraph, VertexEdgeTest) {
  BasicGraph graph;
  for (char ch = 'a'; ch <= 'f'; ++ch) graph.AddVertex(std::string(1, ch));

  graph.AddEdge("a", "b", 3.0);
  graph.AddEdge("a", "c", 2.0);
  graph.AddEdge("b", "d", 4.0);
  graph.AddEdge("c", "b", 1.0);
  graph.AddEdge("c", "d", 2.0);
  graph.AddEdge("c", "e", 3.0);
  graph.AddEdge("d", "e", 2.0);
  graph.AddEdge("d", "f", 1.0);
  graph.AddEdge("e", "f", 2.0);

  EXPECT_EQ(graph.edges_from("a").size(), 2);
  EXPECT_EQ(graph.edges_from("b").size(), 1);
  EXPECT_EQ(graph.edges_from("c").size(), 3);
  EXPECT_EQ(graph.edges_from("d").size(), 2);
  EXPECT_EQ(graph.edges_from("e").size(), 1);
  EXPECT_EQ(graph.edges_from("f").size(), 0);

  graph.ModifyEdge("a", "b", 1.0);
  EXPECT_EQ(graph.edge_cost("a", "b"), 1.0);

  graph.RemoveEdge("a", "b");
  EXPECT_EQ(graph.edges_from("a").size(), 1);
}

}  // namespace
}  // namespace qcraft::planner
