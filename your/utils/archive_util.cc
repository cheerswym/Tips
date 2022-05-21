#include "onboard/utils/archive_util.h"

#include <fcntl.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "glog/logging.h"
#include "libarchive/archive.h"
#include "libarchive/archive_entry.h"
#include "onboard/utils/filesystem.h"

namespace qcraft {
namespace archive {

namespace {
enum Format : uint32_t {
  TAR = 1,
  TAR_GZ = 2,
  TAR_BZ2 = 3,
  TAR_XZ = 4,
  ZIP = 5,
};

absl::Status CreateArchiveImpl(std::string_view dest, Format format,
                               absl::Span<const std::string> srcs) {
  struct archive *a = archive_write_new();
  switch (format) {
    case Format::TAR_GZ:
      archive_write_add_filter_gzip(a);
      break;
    case Format::TAR_BZ2:
      archive_write_add_filter_bzip2(a);
      break;
    case Format::TAR_XZ:
      archive_write_add_filter_xz(a);
      break;
    case Format::ZIP:
      archive_write_set_format_zip(a);
      break;
    default:
      archive_write_add_filter_none(a);
      break;
  }
  archive_write_set_format_ustar(a);
  archive_write_open_filename(a, dest.data());

  VLOG(1) << "Now processing: " << filesystem::current_path();
  for (const auto &src : srcs) {
    struct archive *disk = archive_read_disk_new();
    archive_read_disk_set_standard_lookup(disk);
    int r = archive_read_disk_open(disk, src.c_str());
    if (r != ARCHIVE_OK) {
      return absl::InternalError(archive_error_string(disk));
    }

    for (;;) {
      struct archive_entry *entry = archive_entry_new();
      r = archive_read_next_header2(disk, entry);
      if (r == ARCHIVE_EOF) {
        break;
      }
      if (r != ARCHIVE_OK) {
        return absl::InternalError(archive_error_string(disk));
      }

      archive_read_disk_descend(disk);
      const char *curr_file = archive_entry_pathname(entry);
      VLOG(1) << "a " << curr_file;
      r = archive_write_header(a, entry);
      if (r < ARCHIVE_OK) {
        if (r == ARCHIVE_FATAL) {
          return absl::InternalError(absl::StrCat(
              "Fatal error for ", curr_file, ": ", archive_error_string(a)));
        }
        LOG(WARNING) << curr_file << ": " << archive_error_string(a);
      }
      if (r > ARCHIVE_FAILED) {
        /* For now, we use a simpler loop to copy data
         * into the target archive. */
        char buff[4096];
        int fd = open(archive_entry_sourcepath(entry), O_RDONLY);
        ssize_t len = read(fd, buff, sizeof(buff));
        while (len > 0) {
          archive_write_data(a, buff, len);
          len = read(fd, buff, sizeof(buff));
        }
        close(fd);
      }
      archive_entry_free(entry);
    }
    archive_read_close(disk);
    archive_read_free(disk);
  }

  archive_write_close(a);
  archive_write_free(a);
  return absl::OkStatus();
}

int CopyData(struct archive *ar, struct archive *aw) {
  const void *buff = nullptr;
  for (;;) {
    size_t size = 0;
    int64_t offset = 0;
    int r = archive_read_data_block(ar, &buff, &size, &offset);
    if (r == ARCHIVE_EOF) {
      return ARCHIVE_OK;
    }

    if (r != ARCHIVE_OK) {
      LOG(WARNING) << archive_error_string(ar);
      return r;
    }

    r = archive_write_data_block(aw, buff, size, offset);
    if (r != ARCHIVE_OK) {
      LOG(WARNING) << "archive_write_data_block(): "
                   << archive_error_string(ar);
      return r;
    }
  }
}

absl::Status ExtractArchiveImpl(std::string_view src, Format format,
                                std::string_view dest_dir) {
  struct archive_entry *entry;

  struct archive *a = archive_read_new();
  struct archive *ext = archive_write_disk_new();
  const int flags = ARCHIVE_EXTRACT_TIME | ARCHIVE_EXTRACT_PERM |
                    ARCHIVE_EXTRACT_ACL | ARCHIVE_EXTRACT_FFLAGS;
  archive_write_disk_set_options(ext, flags);

  switch (format) {
    case Format::TAR_GZ:
      archive_read_support_filter_gzip(a);
      break;
    case Format::TAR_BZ2:
      archive_read_support_filter_bzip2(a);
      break;
    case Format::TAR_XZ:
      archive_read_support_filter_xz(a);
      break;
    case Format::ZIP:
      archive_read_support_format_zip(a);
      break;
    default:
      break;
  }
  archive_read_support_format_tar(a);
  archive_write_disk_set_standard_lookup(ext);

  constexpr size_t kBufferSize = 10240;
  int r = 0;
  if ((r = archive_read_open_filename(a, src.data(), kBufferSize))) {
    return absl::InternalError(archive_error_string(a));
  }
  for (;;) {
    r = archive_read_next_header(a, &entry);
    if (r == ARCHIVE_EOF) {
      break;
    }
    if (r != ARCHIVE_OK) {
      return absl::InternalError(archive_error_string(a));
    }
    const char *curr_file = archive_entry_pathname(entry);
    VLOG(1) << "x " << curr_file;
    const std::string output_path = absl::StrCat(dest_dir, "/", curr_file);
    archive_entry_set_pathname(entry, output_path.c_str());
    r = archive_write_header(ext, entry);
    if (r != ARCHIVE_OK) {
      LOG(WARNING) << curr_file << ": " << archive_error_string(a);
    } else {
      (void)CopyData(a, ext);
      r = archive_write_finish_entry(ext);
      if (r != ARCHIVE_OK) {
        return absl::InternalError(absl::StrCat(
            "archive_write_finish_entry(): ", archive_error_string(ext)));
      }
    }
  }
  archive_read_close(a);
  archive_read_free(a);

  archive_write_close(ext);
  archive_write_free(ext);
  return absl::OkStatus();
}

}  // namespace

absl::Status CreateTar(std::string_view dest,
                       absl::Span<const std::string> srcs) {
  return CreateArchiveImpl(dest, Format::TAR, srcs);
}

absl::Status CreateTarGz(std::string_view dest,
                         absl::Span<const std::string> srcs) {
  return CreateArchiveImpl(dest, Format::TAR_GZ, srcs);
}

absl::Status CreateTarBz2(std::string_view dest,
                          absl::Span<const std::string> srcs) {
  return CreateArchiveImpl(dest, Format::TAR_BZ2, srcs);
}

absl::Status CreateTarXz(std::string_view dest,
                         absl::Span<const std::string> srcs) {
  return CreateArchiveImpl(dest, Format::TAR_XZ, srcs);
}

absl::Status CreateZip(std::string_view dest,
                       absl::Span<const std::string> srcs) {
  return CreateArchiveImpl(dest, Format::ZIP, srcs);
}

absl::Status ExtractTar(std::string_view archive, std::string_view dest_dir) {
  return ExtractArchiveImpl(archive, Format::TAR, dest_dir);
}

absl::Status ExtractTarGz(std::string_view archive, std::string_view dest_dir) {
  return ExtractArchiveImpl(archive, Format::TAR_GZ, dest_dir);
}

absl::Status ExtractTarBz2(std::string_view archive,
                           std::string_view dest_dir) {
  return ExtractArchiveImpl(archive, Format::TAR_BZ2, dest_dir);
}

absl::Status ExtractTarXz(std::string_view archive, std::string_view dest_dir) {
  return ExtractArchiveImpl(archive, Format::TAR_XZ, dest_dir);
}

absl::Status ExtractZip(std::string_view archive, std::string_view dest_dir) {
  return ExtractArchiveImpl(archive, Format::ZIP, dest_dir);
}

}  // namespace archive
}  // namespace qcraft
