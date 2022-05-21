#include "onboard/control/controllers/predict_vehicle_pose.h"

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {
constexpr double kEpsilon = 1e-5;

TEST(PredictVehiclePoseTest, PredictedPoseByConstAccKinematicModel) {
  VehState state = {.steer_delay_time = 0.3,
                    .throttle_delay_time = 0.3,
                    .init_pose = {.x = 0.0, .y = 0.0, .v = 2.0, .heading = 0.0},
                    .control_history_state_mgr = {}};

  VehPose pose = PredictedPoseByConstAccKinematicModel(state, 3.6);
  EXPECT_NEAR(pose.y, state.steer_delay_time * state.init_pose.v, kEpsilon);
}

}  // namespace
}  // namespace qcraft::control
