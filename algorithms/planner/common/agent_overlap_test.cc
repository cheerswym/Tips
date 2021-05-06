#include "onboard/planner/common/agent_overlap.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {
namespace {

MATCHER_P4(AgentOverlapNear, first_ra_s, last_ra_s, lat_dist, epsilon, "") {
  *result_listener << "Actual value: first_ra_s=" << arg.first_ra_s
                   << ", last_ra_s=" << arg.last_ra_s
                   << ", lat_dist=" << arg.lat_dist;
  return std::abs(arg.first_ra_s - first_ra_s) < epsilon &&
         std::abs(arg.last_ra_s - last_ra_s) < epsilon &&
         std::abs(arg.lat_dist - lat_dist) < epsilon;
}

using ::testing::ElementsAre;

TEST(StraightTest, Works) {
  PathPoint p0, p1, p2, p3;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.0, s: 2.0)", &p2);
  TextToProto(R"(x: 3.0 y: 0.0 theta: 0.0, s: 3.0)", &p3);

  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  const auto path_approx = BuildPathApprox({p0, p1, p2, p3}, vehicle_rect,
                                           /*tolerance=*/0.05);

  const Polygon2d box1(Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/1.0,
                             /*width=*/1.0));

  std::vector<AgentOverlap> overlaps;
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box1,
                                  /*max_lat_dist=*/10.0, vehicle_geom);
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 1.5, 0.0, 1e-6)));

  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 1, box1,
                                  /*max_lat_dist=*/10.0, vehicle_geom);
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 1.0, 0.0, 1e-6)));

  const Polygon2d box2(Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/1.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box2,
                                  /*max_lat_dist=*/10.0, vehicle_geom);
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 2.0, 0.0, 1e-6)));

  const Polygon2d box3(Box2d(Vec2d(0.0, 10.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/2.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box3,
                                  /*max_lat_dist=*/10.0, vehicle_geom);
  EXPECT_THAT(overlaps, ElementsAre(AgentOverlapNear(0.0, 0.0, 8.0, 1e-6)));

  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box3,
                                  /*max_lat_dist=*/1.0, vehicle_geom);
  EXPECT_TRUE(overlaps.empty());

  const Polygon2d box4(Box2d(Vec2d(-10.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                             /*width=*/2.0));
  overlaps = ComputeAgentOverlaps(path_approx, /*step_length=*/1.0, 0, 3, box4,
                                  /*max_lat_dist=*/10.0, vehicle_geom);
  EXPECT_TRUE(overlaps.empty());
}

TEST(AgentNearestPointTest, test) {
  PathPoint p0, p1, p2, p3;
  TextToProto(R"(x: 0.0 y: 0.0 theta: 0.0, s: 0.0)", &p0);
  TextToProto(R"(x: 1.0 y: 0.0 theta: 0.0, s: 1.0)", &p1);
  TextToProto(R"(x: 2.0 y: 0.0 theta: 0.0, s: 2.0)", &p2);
  TextToProto(R"(x: 3.0 y: 0.0 theta: 0.0, s: 3.0)", &p3);

  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto vehicle_rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  const auto path_approx = BuildPathApprox({p0, p1, p2, p3}, vehicle_rect,
                                           /*tolerance=*/0.05);
  constexpr double kMaxLatDist = 5.0;

  const Box2d box1 = Box2d(Vec2d(0.0, 5.0), /*heading=*/0.0, /*length=*/1.0,
                           /*width=*/1.0);
  const Polygon2d poly1(box1);
  std::optional<AgentNearestPoint> agent_close = ComputeAgentNearestPoint(
      path_approx, /*step_length=*/1.0, /*first_index=*/0,
      /*last_index=*/3, poly1, kMaxLatDist);

  EXPECT_EQ(agent_close->nearest_ra_s, 0.0);
  EXPECT_NEAR(agent_close->lat_dist,
              kMaxLatDist - box1.width() * 0.5 - vehicle_geom.width() * 0.5,
              1e-5);

  const Polygon2d poly2(Box2d(Vec2d(0.0, 10.0), /*heading=*/0.0,
                              /*length=*/1.0,
                              /*width=*/1.0));
  agent_close = ComputeAgentNearestPoint(path_approx, /*step_length=*/1.0,
                                         /*first_index=*/0,
                                         /*last_index=*/3, poly2, kMaxLatDist);
  EXPECT_FALSE(agent_close.has_value());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
