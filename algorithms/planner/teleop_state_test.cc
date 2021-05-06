#include "onboard/planner/teleop_state.h"

#include "gtest/gtest.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {

TEST(TeleopState, FromProto) {
  TeleopState src;
  TeleopState target;

  RemoteAssistToCarProto proto;

  src.request_case = RemoteAssistToCarProto::kLeftBlinkerOverride;
  src.override_left_blinker = true;
  src.override_left_blinker_on = true;
  src.FillProto(&proto);
  target.FromProto(proto);
  EXPECT_EQ(src, target);
  src.Clear();
  target.Clear();

  src.request_case = RemoteAssistToCarProto::kRightBlinkerOverride;
  src.override_right_blinker = true;
  src.override_right_blinker_on = true;
  src.FillProto(&proto);
  target.FromProto(proto);

  EXPECT_EQ(src, target);
  src.Clear();
  target.Clear();

  src.request_case = RemoteAssistToCarProto::kDoorOverride;
  src.override_door = true;
  src.override_door_open = true;
  src.FillProto(&proto);
  target.FromProto(proto);
  EXPECT_EQ(src, target);
  src.Clear();
  target.Clear();

  src.request_case = RemoteAssistToCarProto::kEnableFeatureOverride;
  src.enable_traffic_light_stopping = false;
  src.enable_lc_objects = true;
  src.enable_pull_over = false;
  src.FillProto(&proto);
  target.FromProto(proto);
  EXPECT_EQ(src, target);
  EXPECT_EQ(false, target.enable_traffic_light_stopping);
  src.Clear();
  target.Clear();

  src.request_case = RemoteAssistToCarProto::kStopVehicle;
  src.brake_to_stop = 1.0;
  src.FillProto(&proto);
  target.FromProto(proto);
  EXPECT_EQ(src, target);
  src.Clear();
  target.Clear();
}
}  // namespace planner
}  // namespace qcraft
