#include "onboard/utils/file_util.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <system_error>

#include "glog/logging.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/utils/filesystem.h"

namespace qcraft {
namespace file_util {

bool GetFileContent(const std::string &filename, std::string *content) {
  QCHECK(content != nullptr);
  std::ifstream fin(filename.c_str(), std::ios::binary | std::ios::in);
  if (!fin.good()) {
    return false;
  }
  *content = std::string((std::istreambuf_iterator<char>(fin)),
                         std::istreambuf_iterator<char>());
  return true;
}

std::string GetFileContentOrDie(const std::string &filename) {
  std::string content;
  bool success = GetFileContent(filename, &content);
  QCHECK(success);
  return content;
}

bool SetFileContent(const std::string &content, const std::string &filename) {
  std::ofstream fout(filename.c_str(), std::ios::binary | std::ios::out);
  if (!fout.good()) {
    return false;
  }
  fout.write(content.data(), content.size());
  return true;
}

void SetFileContentOrDie(const std::string &content,
                         const std::string &filename) {
  bool success = SetFileContent(content, filename);
  QCHECK(success);
}

std::string GetFileExtension(const std::string &filename) {
  const auto pos = filename.rfind(".");
  if (pos == std::string::npos) {
    return std::string();
  } else {
    // E.g., 1) a.txt => txt 2) a.pb.txt => txt
    return filename.substr(pos + 1);
  }
}

absl::Status RemoveFile(std::string_view filepath) {
  std::error_code ec;
  const bool success = filesystem::remove(filepath, ec);
  // According to https://en.cppreference.com/w/cpp/filesystem/remove
  // returns true if the file was deleted, false if it did not exist.
  if (success || ec.value() == 0) {
    return absl::OkStatus();
  } else {
    return absl::UnknownError(ec.message());
  }
}

absl::Status CreateDirectory(std::string_view dir) {
  std::error_code ec;
  const bool success = filesystem::create_directories(dir, ec);
  if (success || ec.value() == 0) {
    return absl::OkStatus();
  } else {
    return absl::UnknownError(ec.message());
  }
}

absl::Status RemoveDirectory(std::string_view dir) {
  std::error_code ec;
  const auto count = filesystem::remove_all(dir, ec);
  if (count == static_cast<std::uintmax_t>(-1)) {  // on error
    return absl::UnknownError(ec.message());
  } else {
    return absl::OkStatus();
  }
}

absl::StatusOr<std::vector<std::string>> ListDirectory(std::string_view dir) {
  if (!filesystem::is_directory(dir)) {
    return absl::NotFoundError(absl::StrCat(dir, " is not a directory"));
  }
  std::vector<std::string> result;
  for (const auto &entry :
       filesystem::directory_iterator(filesystem::path(dir))) {
    result.push_back(entry.path().filename().string());
  }
  return result;
}

bool StringToProto(const std::string &proto_string,
                   google::protobuf::Message *proto) {
  if (!google::protobuf::TextFormat::ParseFromString(proto_string, proto)) {
    return false;
  }
  return true;
}

bool TextFileToProto(const std::string &file_name,
                     google::protobuf::Message *message) {
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int fd = open(file_name.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(ERROR) << "Failed to open " << file_name
               << " in text mode: " << std::strerror(errno);
    return false;
  }

  std::unique_ptr<ZeroCopyInputStream> input =
      std::make_unique<FileInputStream>(fd);
  bool success = google::protobuf::TextFormat::Parse(input.get(), message);
  if (!success) {
    LOG(ERROR) << "Failed to parse " << file_name << " as text proto.";
    ::close(fd);
    return false;
  }

  ::close(fd);
  return success;
}

bool BinaryFileToProto(const std::string &filename,
                       google::protobuf::Message *proto) {
  std::ifstream fin(filename.c_str(), std::ios::binary | std::ios::in);
  if (!fin.is_open()) {
    return false;
  }
  if (!proto->ParseFromIstream(&fin)) {
    return false;
  }
  return true;
}

bool FileToProto(const std::string &filename,
                 google::protobuf::Message *proto) {
  return BinaryFileToProto(filename, proto) || TextFileToProto(filename, proto);
}

bool ProtoToTextFile(const google::protobuf::Message &proto,
                     const std::string &filename) {
  std::string content;
  google::protobuf::TextFormat::PrintToString(proto, &content);
  if (!SetFileContent(content, filename)) {
    return false;
  }
  return true;
}

bool ProtoToBinaryFile(const google::protobuf::Message &proto,
                       const std::string &filename) {
  std::ofstream fout(filename.c_str(), std::ios::binary | std::ios::out);
  if (!fout.is_open()) {
    return false;
  }
  if (!proto.SerializeToOstream(&fout)) {
    return false;
  }
  return true;
}

bool ProtoToJsonFile(const google::protobuf::Message &proto,
                     const std::string &filename) {
  std::string content;

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  options.always_print_primitive_fields = true;
  options.preserve_proto_field_names = true;
  google::protobuf::util::MessageToJsonString(proto, &content, options);

  if (!SetFileContent(content, filename)) {
    return false;
  }
  return true;
}

bool IsSameFile(const std::string &filename, const std::string &filename2) {
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) return false;
  auto auto_close = [](int *fd_ptr) { ::close(*fd_ptr); };
  std::unique_ptr<int, void (*)(int *)> auto_close_fd(&fd, auto_close);

  struct stat statbuf;
  int err = fstat(fd, &statbuf);
  if (err < 0) return false;

  int fd2 = open(filename2.c_str(), O_RDONLY);
  if (fd2 < 0) return false;
  std::unique_ptr<int, void (*)(int *)> auto_close_fd2(&fd2, auto_close);
  struct stat statbuf2;
  int err2 = fstat(fd2, &statbuf2);
  if (err2 < 0 || statbuf.st_size != statbuf2.st_size) return false;

  auto auto_munmap = std::bind(munmap, std::placeholders::_1, statbuf.st_size);

  std::unique_ptr<void, decltype(auto_munmap)> ptr1(
      mmap(nullptr, statbuf.st_size, PROT_READ, MAP_SHARED, fd, 0),
      auto_munmap);
  if (ptr1.get() == MAP_FAILED) return false;
  std::unique_ptr<void, decltype(auto_munmap)> ptr2(
      mmap(nullptr, statbuf.st_size, PROT_READ, MAP_SHARED, fd2, 0),
      auto_munmap);
  if (ptr2.get() == MAP_FAILED) return false;
  return memcmp(ptr1.get(), ptr2.get(), statbuf.st_size) == 0;
}

}  // namespace file_util
}  // namespace qcraft
