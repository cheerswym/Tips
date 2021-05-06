#include "onboard/planner/scheduler/smooth_reference_line_builder.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"

namespace qcraft::planner {
namespace {

void SendSmoothedResultToCanvas(
    const DrivePassage &drive_passage,
    const SmoothedReferenceCenterResult &smoothed_result,
    const std::string &channel) {
  QCHECK(!channel.empty()) << "empty canvas channel name!";
  auto &canvas = vantage_client_man::GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < drive_passage.size(); ++i) {
    const double relative_s =
        drive_passage.station(StationIndex(i)).accumulated_s() -
        drive_passage.front_s();
    const double next_relative_s =
        drive_passage.station(StationIndex(i + 1)).accumulated_s() -
        drive_passage.front_s();
    const auto lane_point =
        drive_passage.extend_lane_path().ArclengthToLanePoint(relative_s);
    const auto next_lane_point =
        drive_passage.extend_lane_path().ArclengthToLanePoint(next_relative_s);
    ASSIGN_OR_CONTINUE(const auto smoothed_l,
                       smoothed_result.GetSmoothedLateralOffset(lane_point));
    ASSIGN_OR_CONTINUE(
        const auto next_smoothed_l,
        smoothed_result.GetSmoothedLateralOffset(next_lane_point));
    const auto xy =
        drive_passage.station(StationIndex(i)).lat_point(smoothed_l);
    const auto next_xy =
        drive_passage.station(StationIndex(i + 1)).lat_point(next_smoothed_l);
    canvas.DrawLine(Vec3d(xy, 0.2), Vec3d(next_xy, 0.2),
                    vis::Color::kLightGreen, 3, vis::BorderStyleProto::DASHED);
  }
}

TEST(SmoothLanePathBoundedByPathBoundary, SmootherTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(route_result.smm.get(),
                                                         psmm_modifier);
  const auto dp_or =
      BuildDrivePassage(planner_semantic_map_manager,
                        route_result.route_lane_path.lane_paths().front(),
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(
                            planner_semantic_map_manager),
                        /*keep_behind_len=*/10.0);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(), "left_turn_drive_passage");

  const auto path_bound_or = BuildPathBoundaryFromDrivePassage(
      planner_semantic_map_manager, dp_or.value());
  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(), "left_turn_path_bound");

  const auto smoothed_line_or = SmoothLanePathBoundedByPathBoundary(
      planner_semantic_map_manager, dp_or.value(), path_bound_or.value(), {193},
      /*half_av_width=*/0.0);
  EXPECT_OK(smoothed_line_or);

  SendSmoothedResultToCanvas(dp_or.value(), smoothed_line_or.value(),
                             "smoothed_left_turn");
}

TEST(FindLanesToSmoothFromRoute, FindLaneIdsTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  {
    const TestRouteResult route_result =
        CreateALeftTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    PlannerSemanticMapModification psmm_modifier;
    PlannerSemanticMapManager planner_semantic_map_manager(
        route_result.smm.get(), psmm_modifier);

    const auto lane_ids_vec = FindLanesToSmoothFromRoute(
        planner_semantic_map_manager, route_result.route_sections);
    EXPECT_TRUE(lane_ids_vec.ok());
    EXPECT_EQ(lane_ids_vec->size(), 1);
    for (const auto &lane_ids : *lane_ids_vec) {
      EXPECT_EQ(lane_ids.size(), 1);
      for (const auto &id : lane_ids) {
        EXPECT_EQ(id, 7814);
      }
    }
  }

  {
    const TestRouteResult route_result =
        CreateARightTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    PlannerSemanticMapModification psmm_modifier;
    PlannerSemanticMapManager planner_semantic_map_manager(
        route_result.smm.get(), psmm_modifier);

    const auto lane_ids_vec = FindLanesToSmoothFromRoute(
        planner_semantic_map_manager, route_result.route_sections);

    EXPECT_TRUE(lane_ids_vec.ok());
    EXPECT_EQ(lane_ids_vec->size(), 2);
    for (const auto &lane_ids : *lane_ids_vec) {
      EXPECT_EQ(lane_ids.size(), 1);
      for (const auto &id : lane_ids) {
        EXPECT_TRUE(id == 7807 || id == 7806);
      }
    }
  }
}

TEST(BuildSmoothedResultMapFromRouteSections, UpdateSmoothResultTest) {
  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         psmm_modifier);

  SmoothedReferenceLineResultMap results;

  const mapping::ElementId id = 7814;
  const std::vector<mapping::ElementId> lane_ids = {id};
  const auto smoothed_line_or = SmoothLanePathByLaneIds(
      planner_semantic_map_manager, lane_ids, /*half_av_width=*/0.0);

  EXPECT_OK(smoothed_line_or);
  EXPECT_TRUE(!results.Contains(lane_ids));
  results.AddResult(lane_ids, *smoothed_line_or);
  EXPECT_TRUE(results.Contains(lane_ids));

  const TestRouteResult route_result =
      CreateARightTurnWithDirectionInfoRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto new_results = BuildSmoothedResultMapFromRouteSections(
      planner_semantic_map_manager, route_result.route_sections,
      /*half_av_width=*/0.0, std::move(results));
  EXPECT_OK(new_results);
  EXPECT_TRUE(!new_results->Contains(lane_ids));
  EXPECT_TRUE(new_results->Contains({7806}));
  EXPECT_TRUE(new_results->Contains({7807}));
}

}  // namespace
}  // namespace qcraft::planner
