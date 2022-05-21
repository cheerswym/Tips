#include "onboard/utils/release_util.h"

#include <boost/filesystem.hpp>

#include "nlohmann/json.hpp"
#include "onboard/global/constants.h"
#include "onboard/logging/proto/lite_run.pb.h"
#include "onboard/maps/map_selector.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace release {

namespace {

std::string GetVehicleParamCommit() {
  std::string dir = "/qcraft/onboard/params/run_params/vehicles/VERSION";
  if (boost::filesystem::exists(dir)) {
    std::ifstream fin(dir);
    if (fin) {
      std::string commit;
      fin >> commit;
      fin.close();
      return commit;
    }
    return "OPEN_VERSION_FAILED";
  }
  return "NO_VERSION_FILE";
}

}  // namespace

bool FillReleaseInfoWithReleaseJson(ReleaseInfo* release_info,
                                    const std::string& release_json_path) {
  std::string content;
  if (!qcraft::file_util::GetFileContent(release_json_path, &content)) {
    LOG(WARNING) << "No release.json, not a release build: "
                 << release_json_path;
    return false;
  }
  nlohmann::json json;
  const auto obj =
      json.parse(content, nullptr /*ignore */, false /*no exception*/);

  if (obj.is_discarded()) {
    LOG(ERROR) << "ERROR! Failed to parse " << release_json_path;
    return false;
  }
  const auto country_short_name =
      qcraft::locale_util::GetCurrentCountryShortName();
  release_info->set_git_tag(obj["tag"]);
  release_info->set_release_tag(obj["tag"]);
  release_info->set_release_xavier_tag(obj["xavier_tag"]);
  release_info->set_release_country(country_short_name);
  auto country_specific_info = obj.find(country_short_name);
  if (country_specific_info == obj.end()) {
    LOG(ERROR) << "Country not supported";
    return false;
  }
  release_info->set_release_map_sha((*country_specific_info)["map_sha"]);
  release_info->set_release_vehicle_commit(GetVehicleParamCommit());
  LOG(INFO) << "release_vehicle_commit: "
            << release_info->release_vehicle_commit();
  return true;
}

bool FillReleaseInfoWithReleaseJsonV2(ReleaseInfo* release_info,
                                      const std::string& release_json_path) {
  std::string content;
  if (!qcraft::file_util::GetFileContent(release_json_path, &content)) {
    LOG(WARNING) << "No release_v2.json, not an release build";
    return false;
  }

  nlohmann::json json;
  const auto obj =
      json.parse(content, nullptr /*ignore */, false /*no exception*/);
  if (obj.is_discarded()) {
    LOG(ERROR) << "ERROR! Failed to parse " << release_json_path;
    return false;
  }
  const auto country_short_name =
      qcraft::locale_util::GetCurrentCountryShortName();
  release_info->set_git_tag(obj["tag"]);
  release_info->set_release_tag(obj["tag"]);
  release_info->set_release_xavier_tag(obj["tag"]);
  release_info->set_release_country(country_short_name);
  auto country_specific_info = obj.find(country_short_name);
  if (country_specific_info == obj.end()) {
    LOG(ERROR) << "Country not supported";
    return false;
  }
  release_info->set_release_map_sha((*country_specific_info)["map_sha"]);
  {
    auto qlfs_sha_iter = country_specific_info->find("qlfs_map_sha");
    if (qlfs_sha_iter != country_specific_info->end()) {
      release_info->set_release_map_qlfs_sha(
          (*country_specific_info)["qlfs_map_sha"]);
    }
  }
  release_info->set_release_vehicle_commit(GetVehicleParamCommit());
  LOG(INFO) << "release_vehicle_commit: "
            << release_info->release_vehicle_commit();
  return true;
}

bool FillMapVersionWithSourceCode(ReleaseInfo* release_info) {
  const auto country_short_name =
      qcraft::locale_util::GetCurrentCountryShortName();
  if (country_short_name.empty()) {
    LOG(ERROR) << "Fail to find map version, empty country";
    return false;
  }

  std::string version_file_path = "";
  if (country_short_name == "us") {
    version_file_path = kUsMapsVersionFileWithSourceCode;
  } else if (country_short_name == "cn") {
    version_file_path = kChinaMapsVersionFileWithSourceCode;
  } else {
    LOG(ERROR) << "Fail to find map version, country:[" << country_short_name
               << "] is not supported";
    return false;
  }

  std::string map_version_sha;
  if (!qcraft::file_util::GetFileContent(version_file_path, &map_version_sha)) {
    LOG(ERROR) << "Fail to read map version file:" << version_file_path;
    return false;
  }
  // git_tag will be set outside.
  release_info->set_release_country(country_short_name);
  release_info->set_release_map_sha(map_version_sha);
  return true;
}
}  // namespace release
}  // namespace qcraft
