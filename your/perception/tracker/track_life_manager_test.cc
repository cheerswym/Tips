#include "onboard/perception/tracker/track_life_manager.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/test_util.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/perception/test_util/track_builder.h"

namespace qcraft::tracker {
TEST(TrackLifeManagerTest, ConfirmedStateTest) {
  VehiclePose pose;
  pose.x = 0.0;
  pose.y = 0.0;
  TrackBuilder track_builder;
  // Near distance promotion test.
  const Polygon2d track_poly0(Box2d::CreateAABox({0, 10}, {3, 13}));
  track_builder.set_contour(track_poly0);
  // Add 10 measurements to track.
  std::vector<MeasurementProto> laser_measurements;
  constexpr int laser_m_num = 10;
  for (int i = 0; i < laser_m_num; ++i) {
    const double timestamp = i * 0.1;
    const auto measurement = BuildMeasurementProto(
        timestamp, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
    laser_measurements.emplace_back(measurement);
  }
  // Near distance test.
  for (int i = 0; i < 2; ++i) {
    track_builder.AddMeasurementToMeasurementHistory(laser_measurements[i]);
  }
  EXPECT_EQ(track_builder.Build().measurement_history.size(), 2);
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  for (int i = 2; i < 3; ++i) {
    track_builder.AddMeasurementToMeasurementHistory(laser_measurements[i]);
  }
  EXPECT_EQ(track_builder.Build().measurement_history.size(), 3);
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));
  track_builder.set_life_state(TrackLifeState::kInit);
  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  track_builder.set_life_state(TrackLifeState::kConfirmed);
  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  track_builder.set_life_state(TrackLifeState::kLost);
  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  // Middle distance promotion test.
  track_builder.set_life_state(TrackLifeState::kInit);
  const Polygon2d track_poly1(Box2d::CreateAABox({0, 80}, {3, 83}));
  track_builder.set_contour(track_poly1);
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  for (int i = 3; i < 5; ++i) {
    track_builder.AddMeasurementToMeasurementHistory(laser_measurements[i]);
  }
  EXPECT_EQ(track_builder.Build().measurement_history.size(), 5);

  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  // Far distance promotion test.
  const Polygon2d track_poly2(Box2d::CreateAABox({0, 110}, {3, 113}));
  track_builder.set_contour(track_poly2);
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));

  for (int i = 5; i < 7; ++i) {
    track_builder.AddMeasurementToMeasurementHistory(laser_measurements[i]);
  }
  EXPECT_EQ(track_builder.Build().measurement_history.size(), 7);

  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));
}

TEST(TrackLifeManagerTest, DISABLED_IdleStateTest) {
  VehiclePose pose;
  pose.x = 0.0;
  pose.y = 0.0;
  TrackBuilder track_builder;
  const Polygon2d track_poly0(Box2d::CreateAABox({0, 10}, {3, 13}));
  track_builder.set_contour(track_poly0);
  // Add 10 measurements to track.
  std::vector<MeasurementProto> laser_measurements;
  constexpr int laser_m_num = 10;
  for (int i = 0; i < laser_m_num; ++i) {
    const double timestamp = i * 0.1;
    const auto measurement = BuildMeasurementProto(
        timestamp, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
    laser_measurements.emplace_back(measurement);
  }
  for (int i = 0; i < 2; ++i) {
    track_builder.AddMeasurementToMeasurementHistory(laser_measurements[i]);
  }
  EXPECT_EQ(track_builder.Build().measurement_history.size(), 2);
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
      track_builder.Build(), pose,
      track_builder.Build().measurement_history.back_time()));
  EXPECT_TRUE(track_life_manager::TrackLifeManager::ShouldBeAtIdleState(
      track_builder.Build(), {0., 0.},
      track_builder.Build().measurement_history.back_time() + 0.5, 0.3));
  EXPECT_FALSE(track_life_manager::TrackLifeManager::ShouldBeAtIdleState(
      track_builder.Build(), {0., 0.},
      track_builder.Build().measurement_history.back_time() + 0.5, 0.6));
}
}  // namespace qcraft::tracker
