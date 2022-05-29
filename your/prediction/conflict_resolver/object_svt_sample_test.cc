#include "onboard/prediction/conflict_resolver/object_svt_sample.h"

#include "gtest/gtest.h"

namespace qcraft {

namespace prediction {

namespace {

TEST(ObjectSvtSample, Acceleration) {
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);

  const double length = 2.0;
  const double ds = 0.1;
  const double a = 1.0;

  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });

  const auto states = SampleDp(nodes, edge_info, ds);
}

TEST(ObjectSvtSample, DecelerationStopEarly) {
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);

  const double length = 2.0;
  const double ds = 0.1;
  const double a = -1.0;

  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });

  const auto states = SampleDp(nodes, edge_info, ds);
  EXPECT_NEAR(states.back().t, 10.0, 1e-1);
  EXPECT_NEAR(states.back().s, 0.5, 1e-6);
  EXPECT_NEAR(states.back().v, 0.0, 1e-6);
}

TEST(ObjectSvtSample, Stationary) {
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 0.0,
      .t = 3.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);

  const double length = 2.0;
  const double ds = 0.1;
  const double a = 0.0;

  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });

  const auto states = SampleDp(nodes, edge_info, ds);
  EXPECT_NEAR(states.back().t, 10.0, 1e-6);
  EXPECT_NEAR(states.back().v, 0.0, 1e-6);
  EXPECT_NEAR(states.back().s, 0.0, 1e-6);
}

TEST(ObjectSvtSample, ConstAcceleration) {
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);

  const double length = 2.0;
  const double ds = 0.1;
  const double a = 0.0;

  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  EXPECT_NEAR(states.back().t, 2.0, 1e-6);
  EXPECT_NEAR(states.back().s, 2.0, 1e-6);
  EXPECT_NEAR(states.back().v, 1.0, 1e-6);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
