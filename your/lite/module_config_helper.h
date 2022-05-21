#ifndef ONBOARD_LITE_MODULE_CONFIG_HELPER_H_
#define ONBOARD_LITE_MODULE_CONFIG_HELPER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/base/base.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/utils/map_util.h"

DECLARE_string(launch_file);
DECLARE_string(module_config_dir);

namespace qcraft {
// Load launch config from a launch config file.
LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile(
    const std::string& parent_dir, const std::string& launch_config_file);

// Load launch config from a launch config file.
LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile(
    const std::string& launch_config_file);

// Load launch config from specified launch_file by flag.
LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile();

// Load all module configs from a launch config file.
std::vector<LiteModuleConfig> LoadModuleConfigsFromLaunchFile(
    const std::string& parent_dir, const std::string& launch_config_file);

// Load all module configs from a launch config file.
std::vector<LiteModuleConfig> LoadModuleConfigsFromLaunchFile(
    const std::string& launch_config_file);

// Load all module config from specified launch_file by flag.
std::vector<LiteModuleConfig> LoadModuleConfigsFromCurrentLaunchFile();

// Load all module configs from specified module config dir by flag.
std::vector<LiteModuleConfig> LoadModuleConfigsFromAllDeclared();

// Load output messages from a module config. Useful for modules that
// unconditionally listens for everything
std::map<std::pair<std::string, std::string>, std::string>
GetOutputsFromModuleConfig(const LiteModuleConfig& module_config);

// Load all output messages from a module config. Useful for modules that
// unconditionally listens for everything, such as logger or vantage module.
std::map<std::pair<std::string, std::string>, std::string>
GetOutputsFromModuleConfigs(
    const std::vector<LiteModuleConfig>& module_configs);

// Load all outputs messages from a module configs, including messages would
// output by remote nodes.  Useful for modules that needs to subscribe
// everything by all nodes, such as logger or vantage module running onboard.
std::map<std::pair<std::string, std::string>, std::string>
GetOutputsByAllNodesFromModuleConfigs(
    const std::vector<LiteModuleConfig>& module_configs);

// Load all input messages from a module config. Useful for modules that
// unconditionally listens for everything, such as node module.
std::map<std::pair<std::string, std::string>, std::string>
GetInputsFromModuleConfigs(const std::vector<LiteModuleConfig>& module_configs);

// Load all input messages from a group of module configs, return those
// remarked as accepting cross node inputs.
std::map<std::pair<std::string, std::string>, std::string>
GetNodeInputsFromModuleConfigs(
    const std::vector<LiteModuleConfig>& module_configs);

// Blacklist certain lite messages that are not logged.
bool IsInLiteMsgBlackList(const std::string& field_name);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODULE_CONFIG_HELPER_H_
