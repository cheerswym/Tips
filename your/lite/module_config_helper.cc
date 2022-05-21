#include "onboard/lite/module_config_helper.h"

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

DEFINE_string(
    launch_file, "launch_run_with_ra_2.pb.txt",
    "The filename of launch run, which is under onboard/lite/launch_config");

DEFINE_string(module_config_dir, "onboard/lite/launch_config/module_configs/",
              "the folder that hold all the module configs");

namespace qcraft {
namespace {
std::vector<LiteModuleConfig> LoadModuleConfigsFromDir(
    const std::string &parent_dir) {
  std::vector<LiteModuleConfig> module_configs;
  boost::filesystem::path p(parent_dir);
  QCHECK(boost::filesystem::is_directory(p));
  for (auto &entry : boost::make_iterator_range(
           boost::filesystem::directory_iterator(p), {})) {
    const std::string file_name = entry.path().string();
    if (!absl::EndsWith(file_name, "pb.txt")) continue;
    module_configs.emplace_back();
    QCHECK(file_util::FileToProto(file_name, &module_configs.back()))
        << entry.path().string();
  }
  return module_configs;
}

std::vector<LiteModuleConfig> CheckSensorLiveModule(
    const std::vector<LiteModuleConfig> &input) {
  std::vector<LiteModuleConfig> result;
  if (OnTestBench()) {
    result = input;
    bool has_sensor_live_module = false;
    for (const auto &ele : input) {
      if (ele.module_class_name() == "SensorLiveModule") {
        has_sensor_live_module = true;
        break;
      }
    }
    if (!has_sensor_live_module) {
      std::string path(
          "onboard/lite/launch_config/module_configs/"
          "sensor_live_module.pb.txt");
      LiteModuleConfig sensor_live_config;
      QCHECK(file_util::FileToProto(path, &sensor_live_config)) << path;
      result.push_back(sensor_live_config);
    }
  } else {
    for (const auto &ele : input) {
      if (ele.module_class_name() != "SensorLiveModule") {
        result.push_back(ele);
      }
    }
  }
  return result;
}

}  // namespace

LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile(
    const std::string &parent_dir, const std::string &launch_config_file) {
  LaunchRunConfig launch_run_config;
  const std::string path = absl::StrCat(parent_dir, "/", launch_config_file);
  QCHECK(file_util::FileToProto(path, &launch_run_config))
      << "Loading file " << path << " error";
  return launch_run_config;
}

LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile(
    const std::string &launch_config_file) {
  return LoadLaunchConfigFromCurrentLaunchFile("onboard/lite/launch_config",
                                               launch_config_file);
}

LaunchRunConfig LoadLaunchConfigFromCurrentLaunchFile() {
  return LoadLaunchConfigFromCurrentLaunchFile("onboard/lite/launch_config",
                                               FLAGS_launch_file);
}

std::vector<LiteModuleConfig> LoadModuleConfigsFromLaunchFile(
    const std::string &parent_dir, const std::string &launch_config_file) {
  std::vector<LiteModuleConfig> module_configs;
  LaunchRunConfig launch_run_config;
  const std::string path = absl::StrCat(parent_dir, "/", launch_config_file);
  QCHECK(file_util::FileToProto(path, &launch_run_config))
      << "Loading file " << path << " error";
  for (const std::string &filename :
       launch_run_config.lite_module_config_filenames()) {
    module_configs.emplace_back();
    QCHECK(file_util::FileToProto(
        absl::StrCat(parent_dir, "/module_configs/", filename),
        &module_configs.back()))
        << absl::StrCat(parent_dir, "/module_configs/", filename);
  }
  return module_configs;
}

std::vector<LiteModuleConfig> LoadModuleConfigsFromLaunchFile(
    const std::string &launch_config_file) {
  return LoadModuleConfigsFromLaunchFile("onboard/lite/launch_config",
                                         launch_config_file);
}

std::vector<LiteModuleConfig> LoadModuleConfigsFromCurrentLaunchFile() {
  return CheckSensorLiveModule(LoadModuleConfigsFromLaunchFile(
      "onboard/lite/launch_config", FLAGS_launch_file));
}

std::vector<LiteModuleConfig> LoadModuleConfigsFromAllDeclared() {
  return LoadModuleConfigsFromDir(FLAGS_module_config_dir);
}

std::map<std::pair<std::string, std::string>, std::string>
GetOutputsFromModuleConfig(const LiteModuleConfig &module_config) {
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (const auto &output : module_config.outputs()) {
    std::string channel = output.channel();
    if (channel.empty()) {
      channel = output.field_name();
    }
    channel_domain_to_field_name[std::make_pair(channel, output.domain())] =
        output.field_name();
  }
  return channel_domain_to_field_name;
}

std::map<std::pair<std::string, std::string>, std::string>
GetOutputsFromModuleConfigs(
    const std::vector<LiteModuleConfig> &module_configs) {
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (const auto &module_config : module_configs) {
    for (const auto &output : module_config.outputs()) {
      std::string channel = output.channel();
      if (channel.empty()) {
        channel = output.field_name();
      }
      channel_domain_to_field_name[std::make_pair(channel, output.domain())] =
          output.field_name();
    }
  }
  return channel_domain_to_field_name;
}

std::map<std::pair<std::string, std::string>, std::string>
GetOutputsByAllNodesFromModuleConfigs(
    const std::vector<LiteModuleConfig> &module_configs) {
  auto channel_domain_to_field_name =
      GetOutputsFromModuleConfigs(module_configs);
  for (const auto &[channel_domain, field_name] :
       GetNodeInputsFromModuleConfigs(module_configs)) {
    if (!ContainsKey(channel_domain_to_field_name, channel_domain)) {
      channel_domain_to_field_name[channel_domain] = field_name;
    }
  }
  return channel_domain_to_field_name;
}

std::map<std::pair<std::string, std::string>, std::string>
GetInputsFromModuleConfigs(
    const std::vector<LiteModuleConfig> &module_configs) {
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (const auto &module_config : module_configs) {
    for (const auto &input : module_config.inputs()) {
      std::string channel = input.channel();
      if (channel.empty()) {
        channel = input.field_name();
      }
      channel_domain_to_field_name[std::make_pair(channel, input.domain())] =
          input.field_name();
    }
  }
  return channel_domain_to_field_name;
}

std::map<std::pair<std::string, std::string>, std::string>
GetNodeInputsFromModuleConfigs(
    const std::vector<LiteModuleConfig> &module_configs) {
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (const auto &module_config : module_configs) {
    for (const auto &input : module_config.inputs()) {
      if (input.has_allow_xnode_input() && input.allow_xnode_input()) {
        std::string channel = input.channel();
        if (channel.empty()) {
          channel = input.field_name();
        }
        channel_domain_to_field_name[std::make_pair(channel, input.domain())] =
            input.field_name();
      }
    }
  }
  return channel_domain_to_field_name;
}

bool IsInLiteMsgBlackList(const std::string &field_name) {
  const std::set<std::string> blacklist = {"pandar_packet_proto"};
  return blacklist.find(field_name) != blacklist.end();
}

}  // namespace qcraft
