#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"

namespace qcraft::planner {

namespace {

TEST(LaneGraph, ForkLaneTest) {
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
                        "lane_graph_fork");

  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto &lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto &[lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(lp_info, *route_result.smm,
                             "lane_graph_fork_lp/" + std::to_string(lane_id));
  }

  EXPECT_NEAR(id_info_map.at(1701).lane_path().length(), 158.893, 1e-3);
  EXPECT_NEAR(id_info_map.at(1701).length_along_route(), 130.0, 1e-3);
}

TEST(LaneGraph, ContinuousLaneChangeTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  const TestRouteResult route_result = CreateAContinuousLaneChangeRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  const auto lane_graph =
      BuildLaneGraph(psmm, sections_info, PlannerObjectManager(),
                     /*stalled_objects=*/{}, /*avoid_lanes=*/{});
  SendLaneGraphToCanvas(lane_graph, *route_result.smm, sections_info,
                        "lane_graph_cont_lc");

  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto &lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto &[lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(
        lp_info, *route_result.smm,
        "lane_graph_cont_lc_lp/" + std::to_string(lane_id));
  }

  EXPECT_NEAR(id_info_map.at(2334).lane_path().length(), 156.514, 1e-3);
  EXPECT_NEAR(id_info_map.at(2334).length_along_route(), 90.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(2333).lane_path().length(), 156.467, 1e-3);
  EXPECT_NEAR(id_info_map.at(2333).length_along_route(), 110.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(2332).lane_path().length(), 156.371, 1e-3);
  EXPECT_NEAR(id_info_map.at(2332).length_along_route(), 130.0, 1e-3);

  EXPECT_FALSE(id_info_map.contains(1697));
}

TEST(LaneGraph, ContinuousLaneChangeWithSolidTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  const TestRouteResult route_result =
      CreateAContinuousLaneChangeRouteWithSolidInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  // Build one object
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(Vec2d(155.0, 0.0))
                            .set_length_width(4.5, 2.2)
                            .set_yaw(0.0)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(Vec2d(155.0, 0.0), 0.0);

  objects.push_back(builder.Build());
  PlannerObjectManager object_mgr(objects);
  DrawPlannerObjectManagerToCanvas(object_mgr, "object",
                                   vis::Color::kLightGreen);

  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, object_mgr, stalled_objects, /*avoid_lanes=*/{2472});
  SendLaneGraphToCanvas(lane_graph, *route_result.smm, sections_info,
                        "lane_graph_solid");

  const auto lp_infos =
      FindBestLanePathsFromStart(psmm, sections_info, lane_graph, nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto &lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto &[lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(lp_info, *route_result.smm,
                             "lane_graph_solid_lp/" + std::to_string(lane_id));
  }

  EXPECT_NEAR(id_info_map.at(2470).lane_path().length(), 134.864, 1e-3);
  EXPECT_NEAR(id_info_map.at(2470).length_along_route(), 90.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(2471).lane_path().length(), 134.768, 1e-3);
  EXPECT_NEAR(id_info_map.at(2471).length_along_route(), 20.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(2472).lane_path().length(), 134.769, 1e-3);
  EXPECT_NEAR(id_info_map.at(2472).length_along_route(), 0.0, 1e-3);
}

}  // namespace

}  // namespace qcraft::planner
