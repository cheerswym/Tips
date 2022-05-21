#include "onboard/utils/archive_util.h"

#include <cstdio>
#include <cstdlib>

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "gtest/gtest.h"

namespace qcraft {
namespace archive {

// NOTE(jiaming):
// As a result of Bazel's symlink settings, this UT can't be run by
// bazel test -c opt //onboard/utils:archive_util_test
// Instead, you should bazel build it and run
// bazel-bin/onboard/utils/archive_util_test manually
namespace {

std::string UniqTmpPathWithSuffix(std::string_view suffix) {
  auto path = boost::filesystem::temp_directory_path() /
              boost::filesystem::unique_path();
  return absl::StrCat(path.native(), suffix);
}

}  // namespace

TEST(ArchiveUtilTest, TestCreateTar) {
  std::string my_tar = UniqTmpPathWithSuffix(".tar");
  auto ret = CreateTar(my_tar, {"onboard/utils/testdata/archive_util"});
  EXPECT_TRUE(ret.ok());
  int retval = std::remove(my_tar.c_str());
  EXPECT_EQ(retval, 0);
}

TEST(ArchiveUtilTest, TestCreateTarGz) {
  std::string my_tar_gz = UniqTmpPathWithSuffix(".tar.gz");
  auto ret = CreateTarGz(my_tar_gz, {"onboard/utils/testdata/archive_util"});
  EXPECT_TRUE(ret.ok());
  int retval = std::remove(my_tar_gz.c_str());
  EXPECT_EQ(retval, 0);
}

TEST(ArchiveUtilTest, TestCreateTarBz2) {
  std::string my_tar_bz2 = UniqTmpPathWithSuffix(".tar.bz2");
  auto ret = CreateTarBz2(my_tar_bz2, {"onboard/utils/testdata/archive_util"});
  EXPECT_TRUE(ret.ok());
  int retval = std::remove(my_tar_bz2.c_str());
  EXPECT_EQ(retval, 0);
}

TEST(ArchiveUtilTest, TestCreateTarXz) {
  std::string my_tar_xz = UniqTmpPathWithSuffix(".tar.xz");
  auto ret = CreateTarXz(my_tar_xz, {"onboard/utils/testdata/archive_util"});
  EXPECT_TRUE(ret.ok());
  int retval = std::remove(my_tar_xz.c_str());
  EXPECT_EQ(retval, 0);
}

TEST(ArchiveUtilTest, TestExtractTarGz) {
  char output_dir_template[1024] = "/tmp/sample-XXXXXX";
  const char *dest_dir = mkdtemp(output_dir_template);
  auto ret = ExtractTarGz("onboard/utils/testdata/archive_util/sample.tar.gz",
                          dest_dir);
  EXPECT_TRUE(ret.ok());
  boost::system::error_code ec;
  boost::filesystem::remove_all(dest_dir, ec);
  EXPECT_EQ(ec, boost::system::error_code());
}

TEST(ArchiveUtilTest, TestCreateAndExtractZip) {
  std::string my_zip = UniqTmpPathWithSuffix(".zip");
  auto ret = CreateZip(my_zip, {"onboard/utils/testdata/archive_util"});
  EXPECT_TRUE(ret.ok());

  char output_dir_template[1024] = "/tmp/sample-XXXXXX";
  const char *dest_dir = mkdtemp(output_dir_template);
  ret = ExtractTarGz(my_zip, dest_dir);
  EXPECT_TRUE(ret.ok());
  boost::system::error_code ec;
  boost::filesystem::remove_all(dest_dir, ec);
  EXPECT_EQ(ec, boost::system::error_code());

  int retval = std::remove(my_zip.c_str());
  EXPECT_EQ(retval, 0);
}

}  // namespace archive
}  // namespace qcraft
