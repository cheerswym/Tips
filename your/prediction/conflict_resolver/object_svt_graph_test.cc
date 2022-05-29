#include "onboard/prediction/conflict_resolver/object_svt_graph.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_util.h"
#include "onboard/prediction/conflict_resolver/object_svt_sample.h"
#include "onboard/prediction/conflict_resolver/svt_cost_provider.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {

void PrintSpeedVectorCompare(const planner::SpeedVector& vec1,
                             const planner::SpeedVector& vec2) {
  auto it1 = vec1.begin();
  auto it2 = vec2.begin();
  while (it1 != vec1.end() || it2 != vec2.end()) {
    std::string s1;
    std::string s2;
    if (it1 != vec1.end()) {
      s1 = it1->DebugString();
      it1++;
    }
    if (it2 != vec2.end()) {
      s2 = it2->DebugString();
      it2++;
    }
    QLOG(INFO) << s1 << "\t\t" << s2;
  }
}

TEST(ObjectSvtGraphTest, StationaryObjectSearch) {
  ConflictResolverParams params;
  params.LoadParams();
  const auto& cost_config = params.GetCostConfig();
  const auto& general_config = params.GetGeneralConfig();
  // Planning for vehicle.
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);

  // Reference speed: const v 2m/s. -> to 20.0 m in 10 s.
  SvtNode start_node({
      .s = 0.0,
      .v = 4.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  std::vector<SvtNode> nodes;
  nodes.push_back(start_node);

  const double a = 0.0;
  const double length = 20.0;
  const double ds = 0.1;

  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<planner::SpeedPoint> points;
  points.reserve(states.size());
  for (const auto& state : states) {
    points.push_back(SvtStateToSpeedPoint(state, a));
  }
  const planner::SpeedVector ref_speed(points);

  // Build cost provider.
  const double stationary_object_s = 15.0;
  std::vector<double> stationary_objects = {stationary_object_s};
  std::vector<double> stoplines;
  std::vector<planner::StBoundaryRef> moving_objects;
  std::vector<const planner::StBoundary*> moving_objects_ptrs;

  SvtCostProviderInput cost_provider_input({
      .stationary_objects = &stationary_objects,
      .stoplines = &stoplines,
      .moving_objects = &moving_objects_ptrs,
      .ref_speed = &ref_speed,
      .cost_config = &cost_config,
      .stationary_follow_distance = object_config.stationary_follow_distance(),
      .dynamic_follow_distance = object_config.dynamic_follow_distance(),
  });

  std::unique_ptr<SvtCostProvider> cost_provider =
      std::make_unique<SvtCostProvider>(cost_provider_input);

  // Build svt graph.
  ObjectProto object_proto;
  object_proto.set_id("vehicle");

  ObjectSvtGraphInput svt_graph_input({
      .traj_id = "vehicle-idx0",
      .object_proto = &object_proto,
      .stoplines = &stoplines,
      .stationary_objects = &stationary_objects,
      .ref_speed = &ref_speed,
      .cost_provider = cost_provider.get(),
      .general_config = &general_config,
      .object_config = &object_config,
  });

  std::unique_ptr<ObjectSvtGraph> svt_graph = std::make_unique<ObjectSvtGraph>(
      svt_graph_input, /*thread_pool=*/nullptr);
  const auto& speed_vec_or = svt_graph->Search();
  PrintSpeedVectorCompare(ref_speed, *speed_vec_or);
  QCHECK_OK(speed_vec_or.status());
  EXPECT_NEAR(speed_vec_or->back().s(),
              stationary_object_s - object_config.stationary_follow_distance(),
              object_config.stationary_follow_distance());
}
}  // namespace

}  // namespace prediction
}  // namespace qcraft
