#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_DATA_STORAGE_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_DATA_STORAGE_H_
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "leveldb/db.h"

namespace qcraft::tracker {
class DataStorage {
 protected:
  void InitLevelDB(const std::string& data_db_prefix,
                   const std::string& db_name);
  std::unique_ptr<leveldb::DB> data_db_;
  std::string data_db_prefix_;
  std::string key_prefix_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_DATA_STORAGE_H_
