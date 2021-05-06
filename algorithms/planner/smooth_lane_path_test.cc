#include "onboard/planner/smooth_lane_path.h"

#include "gtest/gtest.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/smooth_lane.h"

namespace qcraft {
namespace planner {

TEST(SmoothLanePath, DISABLED_LaneArclengthTest) {
  RouteProto route_proto;
  route_proto.add_lane_ids(69);
  route_proto.set_start_fraction(0.3);
  route_proto.set_end_fraction(0.7);
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Route route(&semantic_map_manager, route_proto);
  const mapping::LanePath lane_path(&semantic_map_manager, route_proto);
  const SmoothLanePath smooth_lane_path(&semantic_map_manager, lane_path);
  const mapping::LaneProto& lane =
      semantic_map_manager.FindLaneByIdOrDie(route_proto.lane_ids(0));
  const SmoothLane smooth_lane(semantic_map_manager, lane);
  const double len = smooth_lane.s_max();
  EXPECT_NEAR(smooth_lane_path.s_max(), len * 0.4, 1e-10);

  const Vec3d p0 = smooth_lane_path.Sample(0.1 * len);
  const Vec3d p0_gt = smooth_lane.Sample(0.4 * len);
  EXPECT_LE((p0 - p0_gt).norm(), 1e-10)
      << p0.transpose() << "; " << p0_gt.transpose();

  const Vec3d p1 = smooth_lane_path.Sample(0.3 * len);
  const Vec3d p1_gt = smooth_lane.Sample(0.6 * len);
  EXPECT_LE((p1 - p1_gt).norm(), 1e-10)
      << p1.transpose() << "; " << p1_gt.transpose();
}

TEST(SmoothLanePath, DISABLED_LaneSequenceArclengthTest) {
  RouteProto route_proto;
  route_proto.add_lane_ids(69);
  route_proto.add_lane_ids(69);
  route_proto.add_lane_ids(69);
  route_proto.set_start_fraction(0.3);
  route_proto.set_end_fraction(0.7);
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Route route(&semantic_map_manager, route_proto);
  const mapping::LanePath lane_path(&semantic_map_manager, route_proto);
  const SmoothLanePath smooth_lane_path(&semantic_map_manager, lane_path);
  const mapping::LaneProto& lane =
      semantic_map_manager.FindLaneByIdOrDie(route_proto.lane_ids(0));
  const SmoothLane smooth_lane(semantic_map_manager, lane, &lane, &lane, 0.0);
  const double len = smooth_lane.s_max();
  EXPECT_NEAR(smooth_lane_path.s_max(), len * 2.4, 0.1);

  const Vec3d p0 = smooth_lane_path.Sample(0.6999999 * len);
  const Vec3d p0_gt = smooth_lane.Sample(0.9999999 * len);
  EXPECT_LE((p0 - p0_gt).norm(), 0.1)
      << p0.transpose() << "; " << p0_gt.transpose();

  const Vec3d p1 = smooth_lane_path.Sample(0.7000001 * len);
  const Vec3d p1_gt = smooth_lane.Sample(0.0000001 * len);
  EXPECT_LE((p1 - p1_gt).norm(), 0.1)
      << p1.transpose() << "; " << p1_gt.transpose();

  const Vec3d p2 = smooth_lane_path.Sample(1.6999999 * len);
  const Vec3d p2_gt = smooth_lane.Sample(0.9999999 * len);
  EXPECT_LE((p2 - p2_gt).norm(), 0.1)
      << p2.transpose() << "; " << p2_gt.transpose();

  const Vec3d p3 = smooth_lane_path.Sample(1.7000001 * len);
  const Vec3d p3_gt = smooth_lane.Sample(0.0000001 * len);
  EXPECT_LE((p3 - p3_gt).norm(), 0.1)
      << p3.transpose() << "; " << p3_gt.transpose();
}

}  // namespace planner
}  // namespace qcraft
