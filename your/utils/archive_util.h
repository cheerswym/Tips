#pragma once

#include <initializer_list>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/types/span.h"

namespace qcraft {
namespace archive {

// @desc  create .tar archive
// @param   dest  Path to the generated archive, e.g. output/my.tar
// @param   srcs  Files/dirs to be compressed
absl::Status CreateTar(std::string_view dest,
                       absl::Span<const std::string> srcs);

// @desc  create .tar.gz archive
// @param   dest  Path to the generated archive, e.g. output/my.tar.gz
// @param   srcs  Files/dirs to be compressed
absl::Status CreateTarGz(std::string_view dest,
                         absl::Span<const std::string> srcs);

// Create .tar.bz2 archive
absl::Status CreateTarBz2(std::string_view dest,
                          absl::Span<const std::string> srcs);

// Create .tar.xz archive
absl::Status CreateTarXz(std::string_view dest,
                         absl::Span<const std::string> srcs);

// Create .zip archive
absl::Status CreateZip(std::string_view dest,
                       absl::Span<const std::string> srcs);

// @desc  Extract .tar archive
// @param   archive   Path of the archive to be extracted
// @param   dest_dir  Directory to extract the archive into. similar to that of
//                    "tar -x -C dest_dir". Use "." to extract to current dir.
absl::Status ExtractTar(std::string_view archive, std::string_view dest_dir);

// @desc  Extract .tar.gz/.tgz archive
// @param   archive   Path of the archive to be extracted
// @param   dest_dir  Directory to extract the archive into. similar to that of
//                    "tar -xz -C dest_dir". Use "." to extract to current dir.
absl::Status ExtractTarGz(std::string_view archive, std::string_view dest_dir);

// @desc  Extract .tar.bz2 archive
// @param   archive   Path of the archive to be extracted
// @param   dest_dir  Directory to extract the archive into. similar to that of
//                    "tar -x -C dest_dir". Use "." to extract to current dir.
absl::Status ExtractTarBz2(std::string_view archive, std::string_view dest_dir);

// @desc  Extract .tar.xz/.txz archive
// @param   archive   Path of the archive to be extracted
// @param   dest_dir  Directory to extract the archive into. similar to that of
//                    "tar -x -C dest_dir". Use "." to extract to current dir.
absl::Status ExtractTarXz(std::string_view archive, std::string_view dest_dir);

// @desc  Extract .zip archive
// @param   archive   Path of the archive to be extracted
// @param   dest_dir  Directory to extract the archive into. similar to that of
//                    "unzip archive -d dest_dir".
absl::Status ExtractZip(std::string_view archive, std::string_view dest_dir);

}  // namespace archive
}  // namespace qcraft
