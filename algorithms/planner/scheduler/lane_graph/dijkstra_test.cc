#include "onboard/planner/scheduler/lane_graph/dijkstra.h"

#include "gtest/gtest.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TEST(Dijkstra, ShortestPathTest) {
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

  ASSIGN_OR_DIE(const auto path, Dijkstra(graph, "a", "f"));
  EXPECT_EQ(path.total_cost, 5.0);

  const auto& vertices = path.vertices;
  EXPECT_EQ(vertices.size(), 4);
  EXPECT_EQ(vertices[0], "a");
  EXPECT_EQ(vertices[1], "c");
  EXPECT_EQ(vertices[2], "d");
  EXPECT_EQ(vertices[3], "f");

  graph.RemoveEdge("d", "f");
  graph.RemoveEdge("e", "f");
  ASSERT_FALSE(Dijkstra(graph, "a", "f").ok());
}

}  // namespace
}  // namespace qcraft::planner
