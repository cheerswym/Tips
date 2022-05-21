#include "onboard/utils/file_util.h"

#include "common/proto/lane_point.pb.h"
#include "gtest/gtest.h"

#define EXPECT_OK(status) EXPECT_TRUE((status).ok())

namespace qcraft {
namespace file_util {

TEST(FileUtilTest, TestProtoText) {
  ::qcraft::mapping::LanePointProto point_proto, point_proto_in;
  point_proto.set_lane_id(123);
  EXPECT_TRUE(ProtoToTextFile(point_proto, "lane_point.pb.txt"));
  EXPECT_TRUE(FileToProto("lane_point.pb.txt", &point_proto_in));
  RemoveFile("lane_point.pb.txt").IgnoreError();
}

TEST(FileUtilTest, TestGetFileExtension) {
  std::string filename = "BUILD";
  EXPECT_TRUE(GetFileExtension(filename).empty());
  filename = "vantage.pb.txt";
  EXPECT_EQ(GetFileExtension(filename), "txt");
}

TEST(FileUtilTest, TestRemoveFile) {
  std::string fpath = "/path/to/non-exist-file";
  EXPECT_OK(RemoveFile(fpath));

  fpath = "abc.txt";
  EXPECT_TRUE(SetFileContent("a", fpath));
  EXPECT_OK(RemoveFile(fpath));
}

TEST(FileUtilTest, TestDirOps) {
  std::string dir = "path/to/dir";
  EXPECT_OK(CreateDirectory(dir));
  EXPECT_OK(RemoveDirectory(dir));
  dir = "path/to/non-exist-dir";
  EXPECT_OK(RemoveDirectory(dir));
}

}  // namespace file_util
}  // namespace qcraft
