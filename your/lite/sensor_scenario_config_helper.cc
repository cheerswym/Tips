#include "onboard/lite/sensor_scenario_config_helper.h"

#include <set>
#include <string>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "boost/range/iterator_range.hpp"
#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/utils/file_util.h"

DEFINE_string(sensor_scenario_config_file, "sensor_scenario_config.pb.txt",
              "The filename of sensor scenario, which is under "
              "onboard/lite/scenario_config");

namespace qcraft {
absl::StatusOr<SensorScenarioConfig> LoadSensorScenarioConfigFromFile(
    const std::string& parent_dir,
    const std::string& sensor_scenario_config_file) {
  SensorScenarioConfig sensor_scenario_config;
  const std::string path =
      absl::StrCat(parent_dir, "/", sensor_scenario_config_file);
  if (!file_util::FileToProto(path, &sensor_scenario_config)) {
    return absl::NotFoundError(absl::StrCat("Loading file ", path, " error."));
  }
  return sensor_scenario_config;
}

absl::StatusOr<SensorScenarioConfig> LoadSensorScenarioConfigFromFile(
    const std::string& sensor_scenario_config_file) {
  return LoadSensorScenarioConfigFromFile("onboard/lite/scenario_config",
                                          sensor_scenario_config_file);
}

absl::StatusOr<SensorScenarioConfig>
LoadSensorScenarioConfigFromCurrentConfigFile() {
  return LoadSensorScenarioConfigFromFile("onboard/lite/scenario_config",
                                          FLAGS_sensor_scenario_config_file);
}
}  // namespace qcraft
