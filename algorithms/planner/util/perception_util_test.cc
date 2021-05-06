#include "onboard/planner/util/perception_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {

using testing::ElementsAre;

TEST(PerceptionUtil, ComputeObjectContour) {
  ObjectProto proto;
  const std::vector<Vec2d> points{{1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
  for (const auto& pt : points) {
    Vec2dToProto(pt, proto.add_contour());
  }
  const Polygon2d contour = ComputeObjectContour(proto);
  EXPECT_EQ(contour.num_points(), 3);
  EXPECT_THAT(contour.points(),
              ElementsAre(Vec2dEqXY(1.0, 0.0), Vec2dEqXY(0.0, 1.0),
                          Vec2dEqXY(0.0, -1.0)));
}

TEST(PerceptionUtil, AvPoseProtoToObjectProto) {
  const auto vehicle_geom = DefaultVehicleGeometry();
  const std::string id = "abc";
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(1.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(M_PI_2);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(2.0);
  pose.mutable_accel_smooth()->set_x(0.0);
  pose.mutable_accel_smooth()->set_y(1.0);
  pose.set_timestamp(10.0);
  pose.set_speed(2.0);
  pose.set_curvature(0.1);
  const auto object_proto =
      AvPoseProtoToObjectProto(id, vehicle_geom, pose, /*offroad=*/false);
  EXPECT_THAT(object_proto, ProtoPartialApproximatelyMatchesText(R"(
        type: OT_VEHICLE
        id:  "abc"
        pos { x: 1 y: 1.5 }
        vel { x: 0 y: 2 }
        accel { x: 0 y: 1 }
        contour { x: 0.0 y: 4.0 }
        contour { x: 0.0 y: -1.0 }
        contour { x: 2 y: -1 }
        contour { x: 2 y: 4 }
        timestamp: 10
        yaw: 1.571
        yaw_rate: 0.2
        parked: false
        offroad: false
        moving_state: MS_MOVING
        min_z: 0
        max_z: 2.2
        bounding_box { x: 1 y: 1.5 heading: 1.571 width: 2 length: 5 }
        )",
                                                                 1e-3));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
