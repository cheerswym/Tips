#ifndef ONBOARD_UTILS_RELEASE_UTIL_H_
#define ONBOARD_UTILS_RELEASE_UTIL_H_

#include <string>

#include "onboard/logging/proto/lite_run.pb.h"

namespace qcraft::release {
// Fill the release info from release json
bool FillReleaseInfoWithReleaseJson(ReleaseInfo* release_info,
                                    const std::string& release_json_path);

bool FillReleaseInfoWithReleaseJsonV2(ReleaseInfo* release_info,
                                      const std::string& release_json_path);

bool FillMapVersionWithSourceCode(ReleaseInfo* release_info);

}  // namespace qcraft::release

#endif  // ONBOARD_UTILS_RELEASE_UTIL_H_
