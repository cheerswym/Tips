#include "onboard/planner/router/off_road_route_searcher.h"

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/router/plot_util.h"

namespace qcraft::planner {

namespace {

TEST(FindCrossingLanePointsFromPose, FromOffRoad) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    const auto lane_points = FindCrossingLanePointsFromPose(
        semantic_map_manager, pose, /*radius=*/20.0);

    std::vector<Vec2d> points;
    points.reserve(lane_points.size());
    for (const auto lp : lane_points) {
      points.emplace_back(lp.ComputePos(semantic_map_manager));
    }

    SendPointsToCanvas(points, "test/crossing_points", vis::Color::kRed);

    EXPECT_EQ(lane_points.size(), 2);
    EXPECT_EQ(lane_points[0].lane_id(), 3093);
    EXPECT_EQ(lane_points[1].lane_id(), 3104);
  }

  {
    // From side road
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(673.1);
    pose.mutable_pos_smooth()->set_y(-1346);
    pose.set_yaw(0.648);

    const auto lane_points = FindCrossingLanePointsFromPose(
        semantic_map_manager, pose, /*radius=*/20.0);

    std::vector<Vec2d> points;
    points.reserve(lane_points.size());
    for (const auto lp : lane_points) {
      points.emplace_back(lp.ComputePos(semantic_map_manager));
    }

    SendPointsToCanvas(points, "test/crossing_points_1", vis::Color::kBlue);

    EXPECT_EQ(lane_points.size(), 1);
    EXPECT_EQ(lane_points[0].lane_id(), 8294);
  }
}

TEST(SearchForRouteFromOffRoad, FromOffRoad) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  qcraft::vantage_client_man::CreateVantageClientMan(*param_manager);
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    RoutingRequestProto routing_request_proto;

    auto *stop = routing_request_proto.mutable_multi_stops()->add_stops();
    mapping::LanePoint(3841, 0.7).ToProto(
        stop->mutable_stop_point()->mutable_lane_point());

    ASSIGN_OR_DIE(
        auto request,
        BuildMultipleStopsRequest(semantic_map_manager, routing_request_proto));

    ASSIGN_OR_DIE(auto route_result,
                  SearchForRouteFromOffRoad(semantic_map_manager, pose, request,
                                            /*search_radius=*/20.0));

    std::string channel = "test1/whole_route";
    SendRouteLanePathToCanvas(semantic_map_manager,
                              route_result.on_road_route_lane_path,
                              absl::StrCat(channel, "_on_road"));

    auto &canvas =
        vantage_client_man::GetCanvas(absl::StrCat(channel, "_off_road"));
    canvas.SetGroundZero(1);
    const Vec2d link_point =
        route_result.link_lane_point.ComputePos(semantic_map_manager);
    canvas.DrawLine(Vec3d(pose.pos_smooth().x(), pose.pos_smooth().y(), 0.0),
                    Vec3d(link_point.x(), link_point.y(), 0.0),
                    vis::Color::kBlue);
    vantage_client_man::FlushAll();
  }

  {
    // From parking spot
    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(536.98);
    pose.mutable_pos_smooth()->set_y(29.33);
    pose.set_yaw(-1.63);

    RoutingRequestProto routing_request_proto;

    auto *stop = routing_request_proto.mutable_multi_stops()->add_stops();
    mapping::LanePoint(3092, 0.7).ToProto(
        stop->mutable_stop_point()->mutable_lane_point());

    ASSIGN_OR_DIE(
        auto request,
        BuildMultipleStopsRequest(semantic_map_manager, routing_request_proto));

    ASSIGN_OR_DIE(auto route_result,
                  SearchForRouteFromOffRoad(semantic_map_manager, pose, request,
                                            /*search_radius=*/20.0));

    std::string channel = "test2/whole_route";
    SendRouteLanePathToCanvas(semantic_map_manager,
                              route_result.on_road_route_lane_path,
                              absl::StrCat(channel, "_on_road"));

    auto &canvas =
        vantage_client_man::GetCanvas(absl::StrCat(channel, "_off_road"));
    canvas.SetGroundZero(1);
    const Vec2d link_point =
        route_result.link_lane_point.ComputePos(semantic_map_manager);
    canvas.DrawLine(Vec3d(pose.pos_smooth().x(), pose.pos_smooth().y(), 0.0),
                    Vec3d(link_point.x(), link_point.y(), 0.0),
                    vis::Color::kBlue);
    vantage_client_man::FlushAll();
  }
}

}  // namespace

}  // namespace qcraft::planner
