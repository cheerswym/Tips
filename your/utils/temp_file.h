#ifndef ONBOARD_UTILS_TEMP_FILE_H_
#define ONBOARD_UTILS_TEMP_FILE_H_

#include <iostream>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "glog/logging.h"
#include "onboard/global/singleton.h"

namespace qcraft {

class TempFile {
 public:
  TempFile();
  TempFile(TempFile&& other);
  explicit TempFile(const std::string& path) : fname_(path) {}

  ~TempFile() {
    if (!fname_.empty()) boost::filesystem::remove_all(fname_);
  }

  const std::string& path() const { return fname_; }

  absl::Status CopyTo(const std::string& to);

  absl::Status DeepCopyTo(const std::string& dir);
  TempFile& operator=(TempFile&& other);

 private:
  std::string fname_;

  DISALLOW_COPY_AND_ASSIGN(TempFile);
};

// A util class used to create a temporary directory. Note that caller is
// responsible for cleaning up the temporary directory, if needed.
class TempDirectory {
 public:
  explicit TempDirectory(bool auto_delete = true);
  explicit TempDirectory(const std::string& path, bool auto_delete = true);
  ~TempDirectory() {
    if (auto_delete_) {
      LOG(INFO) << "removing " << pname_;
      boost::filesystem::remove_all(pname_);
    }
  }
  std::string path() const { return pname_; }

  TempDirectory& operator=(TempDirectory&& other);

 private:
  std::string pname_;
  bool auto_delete_;

  DISALLOW_COPY_AND_ASSIGN(TempDirectory);
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_TEMP_FILE_H_
