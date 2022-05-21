#include "onboard/perception/test_util/track_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/perception/test_util/measurement_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace perception {
namespace {

TEST(TrackBuilderTest, Build) {
  const auto track = TrackBuilder().Build();
  EXPECT_EQ(track.track_state.id, 0);
}

TEST(TrackBuilderTest, SetId) {
  const auto track = TrackBuilder().set_id(5).Build();
  EXPECT_EQ(track.track_state.id, 5);
}

TEST(TrackBuilderTest, SetRefPoint) {
  const auto track =
      TrackBuilder()
          .set_ref_point({tracker::TrackState::RefPoint::kBBCenter, {3.0, 3.0}})
          .Build();
  EXPECT_THAT(track.track_state.ref_point.pos, Vec2dEqXY(3.0, 3.0));
}

TEST(TrackBuilderTest, SetContour) {
  TrackBuilder track_builder;
  const Polygon2d poly(Box2d::CreateAABox({0, 0}, {1, 1}));
  track_builder.set_contour(poly);
  EXPECT_THAT(track_builder.Build().track_state.contour.points(),
              testing::ElementsAre(Vec2dEqXY(1.0, 1.0), Vec2dEqXY(0.0, 1.0),
                                   Vec2dEqXY(0.0, 0.0), Vec2dEqXY(1.0, 0.0)));
}

TEST(TrackBuilderTest, SetLifeState) {
  TrackBuilder track_builder;
  TrackLifeState life_state = TrackLifeState::kIdle;
  track_builder.set_life_state(life_state);
  EXPECT_EQ(track_builder.Build().track_state.life_state,
            TrackLifeState::kIdle);

  life_state = TrackLifeState::kInit;
  track_builder.set_life_state(life_state);
  EXPECT_EQ(track_builder.Build().track_state.life_state,
            TrackLifeState::kInit);

  life_state = TrackLifeState::kConfirmed;
  track_builder.set_life_state(life_state);
  EXPECT_EQ(track_builder.Build().track_state.life_state,
            TrackLifeState::kConfirmed);

  life_state = TrackLifeState::kLost;
  track_builder.set_life_state(life_state);
  EXPECT_EQ(track_builder.Build().track_state.life_state,
            TrackLifeState::kLost);
}

TEST(TrackBuilderTest, AddMeasurementToMeasurementHistory) {
  const auto measurement = BuildMeasurementProto(
      1.0, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
  const auto track =
      TrackBuilder().AddMeasurementToMeasurementHistory(measurement).Build();
  EXPECT_EQ(track.measurement_history.size(), 1);
  EXPECT_EQ(track.measurement_history.front_value(), &measurement);
}

TEST(TrackBuilderTest, AddMeasurementToMeasurementHistoryOutOfOrder) {
  const auto measurement = BuildMeasurementProto(
      1.0, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
  const auto measurement_2 = BuildMeasurementProto(
      0.0, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
  EXPECT_DEATH(TrackBuilder()
                   .AddMeasurementToMeasurementHistory(measurement)
                   .AddMeasurementToMeasurementHistory(measurement_2)
                   .Build(),
               "");
}

TEST(TrackBuilderTest, AdjustFirstLastTimestampsGivenMeasurementHistory) {
  const auto measurement = BuildMeasurementProto(
      0.0, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
  const auto measurement_2 = BuildMeasurementProto(
      1.5, MT_VEHICLE, MTS_LL_NET, LaserMeasurementBuilder().Build());
  const auto measurement_3 = BuildMeasurementProto(
      2.0, MT_VEHICLE, MTS_LL_NET, RadarMeasurementBuilder().Build());
  const auto track = TrackBuilder()
                         .AddMeasurementToMeasurementHistory(measurement)
                         .AddMeasurementToMeasurementHistory(measurement_2)
                         .AddMeasurementToMeasurementHistory(measurement_3)
                         .AdjustFirstLastTimestampsGivenMeasurementHistory()
                         .Build();
  EXPECT_EQ(track.track_state.first_timestamp, 0.0);
  EXPECT_EQ(track.track_state.last_laser_timestamp, 1.5);
  EXPECT_EQ(track.track_state.last_timestamp, 2.0);
}

}  // namespace
}  // namespace perception
}  // namespace qcraft
