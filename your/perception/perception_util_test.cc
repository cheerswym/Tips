#include "onboard/perception/perception_util.h"

#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/lidar.pb.h"

namespace qcraft::perception_util {

TEST(SelectLidarParamsTest, LidarPriority) {
  LidarParametersProto lidar_center;
  lidar_center.mutable_installation()->set_lidar_id(LDR_CENTER);
  LidarParametersProto lidar_front;
  lidar_front.mutable_installation()->set_lidar_id(LDR_FRONT);
  LidarParametersProto lidar_front_left;
  lidar_front_left.mutable_installation()->set_lidar_id(LDR_FRONT_LEFT);
  LidarParametersProto lidar_front_right;
  lidar_front_right.mutable_installation()->set_lidar_id(LDR_FRONT_RIGHT);
  LidarParametersProto lidar_rear_blind;
  lidar_rear_blind.mutable_installation()->set_lidar_id(LDR_REAR_BLIND);

  RunParamsProtoV2 run_params_v2;
  VehicleParamApi* vehicle_param_api = run_params_v2.mutable_vehicle_params();
  using Message = google::protobuf::util::MessageDifferencer;

  {
    // LDR_CENTER has highest priority
    HardwareParametersProto hardware_parameters;
    *hardware_parameters.add_lidars() = lidar_front;
    *hardware_parameters.add_lidars() = lidar_center;
    vehicle_param_api->Set(hardware_parameters);

    LidarParametersProto selected = SelectLidarParams(run_params_v2, false);
    EXPECT_TRUE(Message::Equals(selected, lidar_center));
    selected = SelectLidarParams(run_params_v2, true);
    EXPECT_TRUE(Message::Equals(selected, lidar_center));
  }

  {
    // LDR_FRONT has 2nd highest priority
    HardwareParametersProto hardware_parameters;
    *hardware_parameters.add_lidars() = lidar_front;
    *hardware_parameters.add_lidars() = lidar_front_left;
    vehicle_param_api->Set(hardware_parameters);

    LidarParametersProto selected = SelectLidarParams(run_params_v2, false);
    EXPECT_TRUE(Message::Equals(selected, lidar_front));
    selected = SelectLidarParams(run_params_v2, true);
    EXPECT_TRUE(Message::Equals(selected, lidar_front_left));
  }

  {
    // LDR_FRONT_LEFT has 3rd highest priority
    HardwareParametersProto hardware_parameters;
    *hardware_parameters.add_lidars() = lidar_front_left;
    *hardware_parameters.add_lidars() = lidar_front_right;
    vehicle_param_api->Set(hardware_parameters);

    LidarParametersProto selected = SelectLidarParams(run_params_v2, false);
    EXPECT_TRUE(Message::Equals(selected, lidar_front_left));
    selected = SelectLidarParams(run_params_v2, true);
    EXPECT_TRUE(Message::Equals(selected, lidar_front_left));
  }

  {
    // for others, the first lidar in the list should be returned
    HardwareParametersProto hardware_parameters;
    *hardware_parameters.add_lidars() = lidar_rear_blind;
    *hardware_parameters.add_lidars() = lidar_front_right;
    vehicle_param_api->Set(hardware_parameters);

    LidarParametersProto selected = SelectLidarParams(run_params_v2, false);
    EXPECT_TRUE(Message::Equals(selected, lidar_rear_blind));
    selected = SelectLidarParams(run_params_v2, true);
    EXPECT_TRUE(Message::Equals(selected, lidar_rear_blind));
  }
}

}  // namespace qcraft::perception_util
