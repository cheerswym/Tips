#include "onboard/perception/tracker/track_classifier/data_storage.h"

#include <string>

#include "absl/strings/str_cat.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"

namespace qcraft::tracker {
void DataStorage::InitLevelDB(const std::string& data_db_prefix,
                              const std::string& db_name) {
  const auto result = OpenLevelDB(data_db_prefix, db_name);
  if (!result.ok()) {
    QLOG(FATAL) << "Failed to open the leveldb at "
                << absl::StrCat(data_db_prefix_, db_name)
                << ", err=" << result.status();
  }
  data_db_.reset(*result);
}
}  // namespace qcraft::tracker
