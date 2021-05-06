#include "onboard/planner/scheduler/target_lane_path_filter.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

TEST(TargetLanePathFilter, DivergingLanePathTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, PlannerObjectManager(),
                     /*stalled_objects=*/{}, /*avoid_lanes=*/{});
  SendLaneGraphToCanvas(lane_graph, *route_result.smm, sections_info,
                        "test/fork_lane/lane_graph");

  auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(route_result.pose.pos_smooth().x());
  pose.mutable_path_point()->set_y(route_result.pose.pos_smooth().y());
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      sections_info, route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), 1699);
  EXPECT_EQ(target_lp_infos[1].start_lane_id(), 1701);

  for (int i = 0; i < target_lp_infos.size(); ++i) {
    SendLanePathInfoToCanvas(target_lp_infos[i], *route_result.smm,
                             absl::StrCat("test/fork_lane/target_lane_",
                                          target_lp_infos[i].start_lane_id()));
  }
}

TEST(TargetLanePathFilter, SingleLanePathTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateASingleLaneRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, PlannerObjectManager(),
                     /*stalled_objects=*/{}, /*avoid_lanes=*/{});

  SendLaneGraphToCanvas(lane_graph, *route_result.smm, sections_info,
                        "test/single_lane/lane_graph");

  auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(route_result.pose.pos_smooth().x());
  pose.mutable_path_point()->set_y(route_result.pose.pos_smooth().y());
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      sections_info, route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 1);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), 692);

  SendLanePathInfoToCanvas(target_lp_infos[0], *route_result.smm,
                           absl::StrCat("test/single_lane/target_lane_",
                                        target_lp_infos[0].start_lane_id()));
}

TEST(TargetLanePathFilter, LeftTurnLanePathTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, PlannerObjectManager(),
                     /*stalled_objects=*/{}, /*avoid_lanes=*/{});
  SendLaneGraphToCanvas(lane_graph, *route_result.smm, sections_info,
                        "test/left_turn/lane_graph");

  auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(route_result.pose.pos_smooth().x());
  pose.mutable_path_point()->set_y(route_result.pose.pos_smooth().y());
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      sections_info, route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 1);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), 63);

  SendLanePathInfoToCanvas(target_lp_infos[0], *route_result.smm,
                           absl::StrCat("test/left_turn/target_lane_",
                                        target_lp_infos[0].start_lane_id()));
}

}  // namespace

}  // namespace qcraft::planner
