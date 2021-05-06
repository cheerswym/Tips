#include "onboard/planner/router/route_sections_info.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_test_util.h"

namespace qcraft::planner {

namespace {

TEST(RouteSectionsInfoTest, BuildTest) {
  FLAGS_planner_increase_lane_speed_limit_fraction = 0.0;

  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  SendRouteSectionsAreaToCanvas(route_result.smm.get(),
                                route_result.route_sections, "test/fork_route");

  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{1761});

  EXPECT_NEAR(sections_info.section_segment(0).driving_distance[0],
              sections_info.section_segment(0).driving_distance[1], 1e-3);

  EXPECT_NEAR(sections_info.section_segment(1).length(),
              sections_info.section_segment(1).driving_distance.back(), 1e-3);

  EXPECT_NEAR(0.0, sections_info.section_segment(2).driving_distance.back(),
              1e-3);

  EXPECT_NEAR(sections_info.planning_horizon(), 230.748, 1e-3);
}

TEST(RouteSectionsInfoTest, PlanningHorizonTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);

  const TestRouteResult route_result = CreateAContinuousLaneChangeRouteInDojo();
  const PlannerSemanticMapManager psmm(route_result.smm.get(),
                                       PlannerSemanticMapModification());
  SendRouteSectionsAreaToCanvas(route_result.smm.get(),
                                route_result.route_sections,
                                "test/continuous_lc_route");

  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections,
                                        /*avoid_lanes=*/{});

  EXPECT_GT(sections_info.planning_horizon(), sections_info.length());
}

}  // namespace

}  // namespace qcraft::planner
