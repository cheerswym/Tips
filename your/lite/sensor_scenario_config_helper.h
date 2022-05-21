#ifndef ONBOARD_LITE_SENSOR_SCENARIO_CONFIG_HELPER_H_
#define ONBOARD_LITE_SENSOR_SCENARIO_CONFIG_HELPER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/base/base.h"
#include "onboard/proto/sensor_scenario_config.pb.h"
#include "onboard/utils/map_util.h"

DECLARE_string(sensor_scenario_config_file);

namespace qcraft {

// Load sensor scenario configs from sensor scenario config file.
absl::StatusOr<SensorScenarioConfig> LoadSensorScenarioConfigFromFile(
    const std::string& sensor_scenario_config_file);

// Load sensor scenario configs from sensor scenario config file.
absl::StatusOr<SensorScenarioConfig> LoadSensorScenarioConfigFromFile(
    const std::string& parent_dir,
    const std::string& sensor_scenario_config_file);

// Load sensor scenario configs from FLAGS_sensor_scenario_config_file.
absl::StatusOr<SensorScenarioConfig>
LoadSensorScenarioConfigFromCurrentConfigFile();
}  // namespace qcraft

#endif  // ONBOARD_LITE_SENSOR_SCENARIO_CONFIG_HELPER_H_
