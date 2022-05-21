#include "onboard/prediction/conflict_resolver/svt_feature_cost.h"

#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "gtest/gtest.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/conflict_resolver/object_svt_sample.h"

namespace qcraft {

namespace prediction {

namespace {

using StBoundaryRef = planner::StBoundaryRef;
using StBoundaryPoints = planner::StBoundaryPoints;

TEST(SvtFeatureCost, ConstV) {
  // Load params.
  ConflictResolverParams params;
  params.LoadParams();

  // Const V sample. 6m/s 15m.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 6.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 15.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);

  // Object Collision.
  std::vector<double> stationary_objects;
  stationary_objects.push_back(10.0);
  std::vector<planner::StBoundaryRef> moving_objects;
  std::vector<const planner::StBoundary*> moving_objects_ptrs;

  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);

  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_collision_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, moving_objects_ptrs,
          object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Reference speed 5m/s and progress feature.
  std::vector<double> v;
  std::vector<double> t;
  std::vector<double> s;
  const int point_size = 16;
  for (int i = 0; i < point_size; ++i) {
    v.push_back(5.0);
    t.push_back(i);
    s.push_back(i);
  }
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  std::vector<double> collision_costs;
  collision_costs.resize(2);
  std::vector<double> reference_speed_costs;
  reference_speed_costs.resize(2);

  svt_collision_cost->ComputeFeatureCost(states,
                                         absl::MakeSpan(collision_costs));
  svt_ref_speed_cost->ComputeFeatureCost(states,
                                         absl::MakeSpan(reference_speed_costs));
  QCHECK_GT(collision_costs[0], 0.0);
  QCHECK_GT(reference_speed_costs[0], 0.0);
  QCHECK_GT(reference_speed_costs[1], 0.0);
}

TEST(SvtFeatureCost, SvtObjectCollisionFeatureCost_ConstV) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  std::vector<planner::StBoundaryRef> moving_objects;
  stationary_objects.push_back(2.0);
  std::vector<const planner::StBoundary*> ptrs;
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, ptrs, object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.1;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost,
     SvtObjectCollisionFeatureCost_DecelerateBeforeStationaryObject) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  std::vector<planner::StBoundaryRef> moving_objects;
  stationary_objects.push_back(2.0);
  std::vector<const planner::StBoundary*> ptrs;
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, ptrs, object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost,
     SvtObjectCollisionFeatureCost_ConstVelocityAcrossStopline) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  std::vector<planner::StBoundaryRef> moving_objects;
  stationary_objects.push_back(2.0);
  std::vector<const planner::StBoundary*> ptrs;
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, ptrs, object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.2;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  // TODO(changqing): Increase accuracy after improving sampling method.
}

TEST(SvtFeatureCost, SvtObjectCollisionFeatureCost_MovingFarAhead) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  std::vector<planner::StBoundaryRef> moving_objects;

  // Create a const moving st boundary.
  StBoundaryPoints points;
  std::vector<double> v = {1.0, 1.0, 1.0};
  std::vector<double> t = {2.0, 4.0, 6.0};
  std::vector<double> s_lower = {7.0, 9.0, 11.0};
  std::vector<double> s_upper = {10.0, 12.0, 14.0};
  for (int i = 0; i < v.size(); ++i) {
    points.speed_points.emplace_back(v[i], t[i]);
    points.lower_points.emplace_back(s_lower[i], t[i]);
    points.upper_points.emplace_back(s_upper[i], t[i]);
  }
  moving_objects.emplace_back(planner::StBoundary::CreateInstance(
      points, StBoundaryProto::VEHICLE, "FarAhead-idx0",
      /*probability=*/1.0, /*is_stationary=*/false));
  std::vector<const planner::StBoundary*> ptrs;
  for (const auto& bound : moving_objects) {
    ptrs.push_back(bound.get());
  }
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, ptrs, object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 25.0;
  const double ds = 0.5;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  // The vehicle is far ahead, cost should be zero.
  EXPECT_NEAR(costs[1], 0.0, 1e-4);
}

TEST(SvtFeatureCost, SvtObjectCollisionFeatureCost_MovingCloseAhead) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  std::vector<planner::StBoundaryRef> moving_objects;

  // Create a const moving st boundary.
  StBoundaryPoints points;
  std::vector<double> v = {1.0, 1.0, 1.0};
  std::vector<double> t = {2.0, 4.0, 6.0};
  std::vector<double> s_lower = {3.0, 5.0, 7.0};
  std::vector<double> s_upper = {6.0, 8.0, 10.0};
  for (int i = 0; i < v.size(); ++i) {
    points.speed_points.emplace_back(v[i], t[i]);
    points.lower_points.emplace_back(s_lower[i], t[i]);
    points.upper_points.emplace_back(s_upper[i], t[i]);
  }
  moving_objects.push_back(planner::StBoundary::CreateInstance(
      points, StBoundaryProto::VEHICLE, "CloseAhead-idx0",
      /*probability=*/1.0, /*is_stationary=*/false));

  std::vector<const planner::StBoundary*> ptrs;
  for (const auto& bound : moving_objects) {
    ptrs.push_back(bound.get());
  }
  for (const auto* ptr : ptrs) {
    LOG(INFO) << ptr->DebugString();
  }
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, ptrs, object_config.stationary_follow_distance(),
          object_config.dynamic_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 10.0;
  const double ds = 0.5;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));

  // Dynamic follow distance set at 3.0m, each state's squared pass ratio 4/9,
  // duration is 4s.
  EXPECT_NEAR(costs[1], 16.0 / 9.0, 1e-6);

  // If it slightly above reference speed of the st boundary, the cost should be
  // greater than the sampled states move at the same speed of the mapped
  // object.

  std::vector<SvtNode> nodes_2;
  const SvtNode start_node_2({
      .s = 0.0,
      .v = 1.1,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes_2.push_back(start_node_2);
  const double length_2 = 10.0;
  const double ds_2 = 0.5;
  const double a_2 = 0.0;
  DpEdgeInfo edge_info_2({
      .start_index = SvtNodeIndex(0),
      .a = a_2,
      .length = length_2,
  });
  const auto states_2 = SampleDp(nodes_2, edge_info_2, ds_2);

  std::vector<double> costs_2;
  costs_2.resize(2);
  svt_feature_cost->ComputeFeatureCost(states_2, absl::MakeSpan(costs_2));

  EXPECT_GT(costs_2[1], costs[1]);
}

TEST(SvtFeatureCost, SvtReferenceSpeedFeatureCost_ConstVelocity) {
  std::vector<double> v = {1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  // Get svt states samples.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 2.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 3.0;
  const double ds = 0.1;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_ref_speed_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost, SvtReferenceSpeedFeatureCost_Deceleration) {
  std::vector<double> v = {1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  // Get svt states samples.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 2.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 3.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_ref_speed_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  // It completely stops, which is very different from original speed vector.
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
