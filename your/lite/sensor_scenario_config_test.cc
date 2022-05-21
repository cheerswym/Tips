#include <functional>
#include <memory>
#include <set>
#include <thread>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/global/registry.h"
#include "onboard/lite/sensor_scenario_config_helper.h"
#include "onboard/proto/sensor_scenario_config.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
TEST(SensorScenarioConfigTest, LoadSensorScenarioConfigFromCurrentConfigFile) {
  auto sensor_scenario_config = LoadSensorScenarioConfigFromCurrentConfigFile();
  EXPECT_OK(sensor_scenario_config);
}

TEST(SensorScenarioConfigTest, LoadSensorScenarioConfigFromFileFaild) {
  auto sensor_scenario_config =
      LoadSensorScenarioConfigFromFile("not_exist_file");
  if (!sensor_scenario_config.ok()) {
    LOG(INFO) << sensor_scenario_config.status().message();
  }
  EXPECT_FALSE(sensor_scenario_config.ok());
}

TEST(SensorScenarioConfigTest, LoadSensorScenarioConfigFromFileSuccessed) {
  auto sensor_scenario_config =
      LoadSensorScenarioConfigFromFile("sensor_scenario_config_test.pb.txt");

  EXPECT_EQ(sensor_scenario_config->sensor_scenarios().size(), 3);
  auto sensor_scenario = sensor_scenario_config->sensor_scenarios(0);
  EXPECT_EQ(sensor_scenario.cameras().size(), 2);
  EXPECT_EQ(sensor_scenario.lidars().size(), 1);
  EXPECT_EQ(sensor_scenario.radars().size(), 1);
  EXPECT_EQ(sensor_scenario.scenario(), "dbqv2");

  sensor_scenario = sensor_scenario_config->sensor_scenarios(1);
  EXPECT_EQ(sensor_scenario.cameras().size(), 2);
  EXPECT_EQ(sensor_scenario.lidars().size(), 2);
  EXPECT_EQ(sensor_scenario.radars().size(), 0);
  EXPECT_EQ(sensor_scenario.scenario(), "dbqv4");

  sensor_scenario = sensor_scenario_config->sensor_scenarios(2);
  EXPECT_EQ(sensor_scenario.cameras().size(), 2);
  EXPECT_EQ(sensor_scenario.lidars().size(), 1);
  EXPECT_EQ(sensor_scenario.radars().size(), 2);
  EXPECT_EQ(sensor_scenario.scenario(), "pbqv1");

  EXPECT_EQ(sensor_scenario_config->ignore_checkers().size(), 1);
  const auto ignore_checkers = sensor_scenario_config->ignore_checkers(0);
  for (const auto& ignore_checker : sensor_scenario_config->ignore_checkers()) {
    EXPECT_EQ(ignore_checker.channel_checks(0).channel(),
              "gnss_raw_reading_proto");
    EXPECT_EQ(ignore_checker.scenario(), "dbqv3");
  }
}
}  // namespace qcraft
