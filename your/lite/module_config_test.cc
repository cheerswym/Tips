#include <functional>
#include <memory>
#include <set>
#include <thread>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "offboard/hil_tests/pacmod/joystick_control_module.h"
#include "offboard/virtual_world/vehicle_engine_module.h"
#include "offboard/virtual_world/virtual_world_module.h"
#include "onboard/global/registry.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/offboard_modules.h"
#include "onboard/lite/onboard_modules.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/transport.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace {
constexpr int kMinWindowSize = 2;
constexpr int kMinFrequencyLimit = 0;
}  // namespace

TEST(ModlueConfigTest, ModuleResourceConfigs) {
  FLAGS_launch_file = "launch_module_config_help_test.pb.txt";
  auto launch_configs = LoadLaunchConfigFromCurrentLaunchFile();
  EXPECT_EQ(launch_configs.module_resource_configs().size(), 1);
  const auto& module_resource_config =
      launch_configs.module_resource_configs(0);
  EXPECT_EQ(module_resource_config.cpu_core_to_bind().size(), 2);
  for (int i = 0; i < module_resource_config.cpu_core_to_bind().size(); i++) {
    EXPECT_EQ(i * 5, module_resource_config.cpu_core_to_bind(i));
  }

  EXPECT_EQ(module_resource_config.cpu_config().kickout_deadline(), 50);
  EXPECT_EQ(module_resource_config.cpu_config().terminate_deadline(), 150);

  EXPECT_EQ(module_resource_config.memory_config().kickout_deadline(), 300);
  EXPECT_EQ(module_resource_config.memory_config().terminate_deadline(), 2048);

  EXPECT_EQ(module_resource_config.shared_memory_config().kickout_deadline(),
            50);
  EXPECT_EQ(module_resource_config.shared_memory_config().terminate_deadline(),
            500);
}

TEST(ModlueConfigTest, LoadFromFile) {
  std::vector<std::string> launching_files = {
      "launch_run.pb.txt", "launch_run_offboard_playback.pb.txt"};
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (int i = 0; i < launching_files.size(); ++i) {
    LaunchRunConfig launch_run_config;
    std::set<LiteModuleName> existed_module_names;
    std::vector<LiteModuleConfig> module_configs =
        LoadModuleConfigsFromLaunchFile("onboard/lite/launch_config",
                                        launching_files[i]);
    for (const LiteModuleConfig& module_config : module_configs) {
      ASSERT_NE(module_config.module_name(), UNSPECIFIED_MODULE);
      EXPECT_FALSE(
          ContainsKey(existed_module_names, module_config.module_name()));
      existed_module_names.insert(module_config.module_name());
      const bool registered =
          Registry<LiteModule, LiteClientBase*>::IsRegistered(
              module_config.module_class_name());
      LOG(ERROR) << "Checking " << module_config.module_class_name();
      EXPECT_TRUE(registered)
          << "Possible reason: missing header/dependency for module "
          << module_config.module_class_name();

      for (const auto& input : module_config.inputs()) {
        LiteMsgWrapper lite_msg;
        EXPECT_OK(LiteMsgConverter::Get().SetLiteMsgByName(
            "", input.field_name(), &lite_msg));
        std::string channel = input.channel();
        if (channel.empty()) {
          channel = input.field_name();
        }
        std::string& field = channel_domain_to_field_name[std::make_pair(
            channel, input.domain())];
        if (!field.empty()) {
          ASSERT_EQ(field, input.field_name());
        }
      }
      for (const auto& output : module_config.outputs()) {
        LiteMsgWrapper lite_msg;
        EXPECT_OK(LiteMsgConverter::Get().SetLiteMsgByName(
            "", output.field_name(), &lite_msg));
        std::string channel = output.channel();
        if (channel.empty()) {
          channel = output.field_name();
        }
        std::string& field = channel_domain_to_field_name[std::make_pair(
            channel, output.domain())];
        if (!field.empty()) {
          ASSERT_EQ(field, output.field_name());
        }
      }
    }
  }
}

TEST(ModlueConfigTest, LoadAllModuleConfigs) {
  auto configs = LoadModuleConfigsFromAllDeclared();
  EXPECT_GT(configs.size(), 0);
}

TEST(ModlueConfigTest, ChannelCheckerConfig) {
  auto configs = LoadModuleConfigsFromAllDeclared();
  std::map<std::pair<std::string, std::string>, std::string>
      channel_domain_to_field_name;
  for (const LiteModuleConfig& module_config : configs) {
    // Do not check test or mock modules.
    if (module_config.module_name() > 100000) {
      continue;
    }
    ASSERT_NE(module_config.module_name(), UNSPECIFIED_MODULE);
    LOG(ERROR) << "Checking " << module_config.module_class_name();

    auto max_cpu_usage = module_config.max_cpu_usage();
    if (max_cpu_usage != 0) {
      // Cpu at least 5%.
      EXPECT_TRUE(max_cpu_usage >= 5) << "max_cpu_usage:" << max_cpu_usage;
    }

    auto max_memory_usage_kb = module_config.max_memory_usage_kb();
    if (max_memory_usage_kb != 0) {
      // Memory at least 100M.
      EXPECT_TRUE(max_memory_usage_kb >= 10000)
          << "max_memory_usage_kb:" << max_memory_usage_kb;
    }

    for (const auto& channel_check_config : module_config.checkers()) {
      LiteMsgWrapper lite_msg;
      EXPECT_OK(LiteMsgConverter::Get().SetLiteMsgByName(
          "", channel_check_config.field_name(), &lite_msg));
      std::string channel = channel_check_config.channel();
      if (channel.empty()) {
        channel = channel_check_config.field_name();
      }
      std::string& field = channel_domain_to_field_name[std::make_pair(
          channel, channel_check_config.domain())];
      if (!field.empty()) {
        ASSERT_EQ(field, channel_check_config.field_name());
      }

      // min_frequency and max_frequency.
      int64_t min_frequency = channel_check_config.min_frequency();
      int64_t max_frequency = channel_check_config.max_frequency();
      EXPECT_TRUE(min_frequency > kMinFrequencyLimit);
      EXPECT_TRUE(max_frequency >= min_frequency);

      // frame_window_size.
      int64_t window_size = max_frequency * 2 + 1;
      if (channel_check_config.has_frame_window_size()) {
        window_size = channel_check_config.frame_window_size();
      }
      EXPECT_TRUE(window_size >= kMinWindowSize);
    }

    for (const auto& input : module_config.inputs()) {
      LiteMsgWrapper lite_msg;
      EXPECT_OK(LiteMsgConverter::Get().SetLiteMsgByName("", input.field_name(),
                                                         &lite_msg));
      std::string channel = input.channel();
      if (channel.empty()) {
        channel = input.field_name();
      }
      std::string& field =
          channel_domain_to_field_name[std::make_pair(channel, input.domain())];
      if (!field.empty()) {
        ASSERT_EQ(field, input.field_name());
      }
    }

    for (const auto& output : module_config.outputs()) {
      LiteMsgWrapper lite_msg;
      EXPECT_OK(LiteMsgConverter::Get().SetLiteMsgByName(
          "", output.field_name(), &lite_msg));
      std::string channel = output.channel();
      if (channel.empty()) {
        channel = output.field_name();
      }
      std::string& field = channel_domain_to_field_name[std::make_pair(
          channel, output.domain())];
      if (!field.empty()) {
        ASSERT_EQ(field, output.field_name());
      }
    }
  }
}

TEST(AllLaunchFileCheckTest, AllLaunchFileCheckTest) {
  boost::filesystem::path my_path("onboard/lite/launch_config");
  boost::filesystem::directory_iterator end;
  for (boost::filesystem::directory_iterator iter(my_path); iter != end;
       ++iter) {
    if (boost::filesystem::is_directory(*iter)) {
      continue;
    }
    std::string file_path = iter->path().string();
    if (!absl::EndsWith(file_path, ".pb.txt")) {
      continue;
    }

    LaunchRunConfig launch_run_config;
    EXPECT_TRUE(file_util::FileToProto(file_path, &launch_run_config));
    for (const std::string& filename :
         launch_run_config.lite_module_config_filenames()) {
      std::string full_name =
          absl::StrCat("onboard/lite/launch_config/module_configs/", filename);
      if (!boost::filesystem::exists(full_name)) {
        LOG(ERROR) << "In " << file_path << " " << filename << " is not exist.";
      }
      EXPECT_TRUE(boost::filesystem::exists(full_name));
    }
  }
}

}  // namespace qcraft

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}
