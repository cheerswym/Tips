#include "onboard/perception/tracker/shooter_killer.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/test_util/track_builder.h"
#include "onboard/perception/tracker/shooter_killer_friend.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {
namespace shooter_killer {

TEST(ShooterKiller, TestIsShooters) {
  ShooterKiller shooter_killer;
  ShooterKillerFriend shooter_killer_friend(&shooter_killer);

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{params, params};
  tracker::Estimator estimator(tracker::ToImmProto(v, {0.5, 0.5}));
  tracker::CarState car_state;
  car_state.vel() = 100;
  tracker::StateData state(car_state);
  estimator.SetStateData(state);
  EXPECT_TRUE(estimator.IsCarModel());
  EXPECT_FALSE(estimator.IsPointModel());
  TrackBuilder track_builder;
  track_builder.set_estimator(estimator);

  EXPECT_TRUE(
      shooter_killer_friend.IsShooterByHugeSpeed(track_builder.Build()));
  EXPECT_FALSE(
      shooter_killer_friend.IsShooterByMaxAccCheck(track_builder.Build()));

  track_builder.set_type(MT_UNKNOWN);
  const Polygon2d polygon(Box2d::CreateAABox({0, 0}, {10, 10}));
  track_builder.set_contour(polygon);
  EXPECT_TRUE(
      shooter_killer_friend.IsShooterByHugeUnknownCheck(track_builder.Build()));
}

TEST(ShooterKiller, TestDebugToString) {
  EXPECT_EQ(ShooterKiller::DebugToString(ShooterKiller::kIsShooterByHugeSpeed),
            "kIsShooterByHugeSpeed");
}

}  // namespace shooter_killer
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
