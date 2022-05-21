#include "onboard/global/system_misc.h"

#include <regex>
#include <string>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "glog/logging.h"
#include "onboard/utils/file_util.h"

namespace qcraft {

namespace {
constexpr char kHwmon[] = "/sys/class/hwmon/";
}

[[maybe_unused]] void SetMiscInfo(MiscInfoProto* misc) {
  try {
    for (const auto& entry : boost::filesystem::directory_iterator(kHwmon)) {
      const std::string path = entry.path().string();
      std::string name;
      file_util::GetFileContent(absl::StrCat(path, "/name"), &name);
      name.erase(name.find_last_not_of(" \n\r\t") + 1);
      if (name.compare("k10temp") == 0 || name.compare("coretemp") == 0) {
        std::string input;
        file_util::GetFileContent(absl::StrCat(path, "/temp1_input"), &input);
        auto* temp_info = misc->add_temperature_info();
        temp_info->set_name(name);
        temp_info->set_temperature(std::stoi(input) / 1000);
      }

      for (const auto& file : boost::filesystem::directory_iterator(path)) {
        const std::regex e("(fan\\d+)_input");
        std::smatch sm;
        std::regex_match(file.path().filename().string(), sm, e);
        if (sm.size() > 1) {
          auto* fan_info = misc->add_fan_info();
          fan_info->set_name(sm[1]);
          std::string fan_rpm;
          file_util::GetFileContent(file.path().string(), &fan_rpm);
          fan_info->set_rpm(std::stoi(fan_rpm));
        }
      }
    }
  } catch (boost::filesystem::filesystem_error& e) {
    LOG(ERROR) << e.what();
  }
}

}  // namespace qcraft
