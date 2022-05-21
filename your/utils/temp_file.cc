#include "onboard/utils/temp_file.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "onboard/lite/check.h"
#include "onboard/utils/errors.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace {
absl::Status RecursiveCopy(const boost::filesystem::path& src,
                           const boost::filesystem::path& dst) {
  if (boost::filesystem::is_directory(src)) {
    boost::filesystem::create_directories(dst);
    for (boost::filesystem::directory_entry& item :
         boost::filesystem::directory_iterator(src)) {
      RETURN_IF_ERROR(RecursiveCopy(item.path(), dst / item.path().filename()));
    }
    return absl::OkStatus();
  } else if (boost::filesystem::is_regular_file(src)) {
    boost::system::error_code ec;
    boost::filesystem::copy_file(
        src, dst, boost::filesystem::copy_option::overwrite_if_exists,
        ec);  // NOLINT
    if (ec.value() != boost::system::errc::success) {
      return absl::InternalError(
          absl::StrCat("Failed to copy file from ", src.string(),
                       " to: ", dst.string(), " error code: ", ec.value()));
    }

    return absl::OkStatus();
  } else {
    return absl::InternalError(
        absl::StrCat(dst.generic_string(), "is not a dir or file"));
  }
}
}  // namespace

TempFile::TempFile() {
  // TODO(mike): figure out why creating files in hosthome fails on EC2.
  // // Create the temp file on the host's disks instead of docker's to improve
  // // I/O performance.
  // const char kTempDir[] = "/hosthome/.docker_tmp";
  // if (!boost::filesystem::exists(kTempDir)) {
  //   boost::system::error_code ec;
  //   QCHECK(boost::filesystem::create_directories(kTempDir, ec))
  //       << "Failed to create a temp dir at " << kTempDir << " error code "
  //       << ec.value();
  // }
  // fname_ =
  //     absl::StrCat(kTempDir, "/", boost::filesystem::unique_path().c_str());
  fname_ = (boost::filesystem::temp_directory_path() /
            boost::filesystem::unique_path())
               .c_str();
}

TempFile::TempFile(TempFile&& other) {
  fname_ = other.fname_;
  other.fname_ = "";
}

// Only copy when current file is a directory
absl::Status TempFile::DeepCopyTo(const std::string& dir) {
  if (!boost::filesystem::is_directory(fname_)) {
    return absl::InvalidArgumentError(absl::StrCat(fname_, " is not a dir."));
  }
  return RecursiveCopy(fname_, dir);
}

absl::Status TempFile::CopyTo(const std::string& to) {
  if (path() == to) {
    return absl::InvalidArgumentError(
        absl::StrCat("From and to file paths must be different. Src: ", path(),
                     " to: ", to));
  }

  boost::system::error_code ec;
  boost::filesystem::copy_file(
      path(), to, boost::filesystem::copy_option::overwrite_if_exists,
      ec);  // NOLINT
  if (ec.value() != boost::system::errc::success) {
    return absl::InternalError(absl::StrCat("Failed to copy file from ", path(),
                                            " to: ", to,
                                            " error code: ", ec.value()));
  }

  return absl::OkStatus();
}

TempFile& TempFile::operator=(TempFile&& other) {
  fname_ = other.fname_;
  other.fname_ = "";
  return *this;
}

TempDirectory::TempDirectory(bool auto_delete)
    : TempDirectory((boost::filesystem::temp_directory_path() /
                     boost::filesystem::unique_path())
                        .c_str(),
                    auto_delete) {}

TempDirectory::TempDirectory(const std::string& path, bool auto_delete)
    : pname_(path), auto_delete_(auto_delete) {
  if (!boost::filesystem::exists(pname_)) {
    boost::system::error_code ec;
    QCHECK(boost::filesystem::create_directories(pname_, ec))
        << "Failed to create tmpdir=" << pname_ << ", errcode=" << ec.value()
        << ", errmsg=" << ec.message();
  }
}

TempDirectory& TempDirectory::operator=(TempDirectory&& other) {
  if (path() == other.path()) {
    // noop
    return *this;
  }
  if (auto_delete_) {
    boost::filesystem::remove_all(pname_);
  }
  pname_ = std::move(other.pname_);
  auto_delete_ = other.auto_delete_;
  other.auto_delete_ = false;
  return *this;
}
}  // namespace qcraft
