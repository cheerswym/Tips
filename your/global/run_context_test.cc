#include "onboard/global/run_context.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/sensor_scenario_config_helper.h"
#include "onboard/params/utils/vehicle_params_flags.h"

namespace qcraft {
TEST(QCraftRunContext, QCraftRunContext1) {
  // FLAGS_vehicle_param_dir = "/qcraft/onboard/params/run_params/vehicles";
  // FLAGS_car_name = "Q8021";
  // FLAGS_run_conf = "robobus_production";
  FLAGS_enable_context_test = 1;
  const auto run_context = QCraftRunContext();
  EXPECT_EQ(run_context, VehicleInstallationProto_VehiclePlan_VP_DBQ_V3);
}

TEST(QCraftRunContext, SensorScenarioConfig) {
  FLAGS_sensor_scenario_config_file = "sensor_scenario_config_test.pb.txt";
  const auto& sensor_scenario_config = QCraftSensorScenarioConfig();
  EXPECT_EQ(sensor_scenario_config.sensor_scenarios().size(), 3);
}

}  // namespace qcraft
