
#include "onboard/utils/file_util_mmap.h"

#include <thread>

#include "gtest/gtest.h"
#include "onboard/lite/check.h"
#include "onboard/utils/file_util.h"

namespace qcraft::file_util {
class MMapFileTest : public ::testing::Test {
 protected:
  static constexpr char kTestFile[] = "test.txt";
  static constexpr char kMagicSpell[] = "Open Sesame";
  void SetUp() override { QCHECK(SetFileContent(kMagicSpell, kTestFile)); }
};
TEST_F(MMapFileTest, TestRead) {
  auto file = MMapFile(kTestFile);
  EXPECT_OK(file.status());
  std::string_view sv;
  std::string s;
  EXPECT_TRUE(file.GetFileContentView(&sv));
  EXPECT_TRUE(sv == kMagicSpell);
  EXPECT_TRUE(file.GetFileContent(&s));
  EXPECT_TRUE(s == kMagicSpell);
  sv = file.GetFileContentViewOrDie();
  EXPECT_TRUE(sv == kMagicSpell);
  s = file.GetFileContentOrDie();
  EXPECT_TRUE(s == kMagicSpell);
}

TEST_F(MMapFileTest, TestMultipleReaderWriter) {
  auto ro_file = MMapFile(kTestFile);
  auto rw_file = MMapFile(kTestFile, /* writable= */ true);
  EXPECT_OK(ro_file.status());
  EXPECT_OK(rw_file.status());
  EXPECT_TRUE(ro_file.GetFileContentViewOrDie() ==
              rw_file.GetFileContentViewOrDie());
  std::string s = ro_file.GetFileContentOrDie();
  EXPECT_TRUE(s == kMagicSpell);
  s[0] = 'o';
  // ro_file sees the change to the rw_file instantly. If multi threads were
  // involved, then a pair of read/writer locks would be necessary.
  static_cast<char*>(rw_file.data())[0] = 'o';
  EXPECT_TRUE(rw_file.GetFileContentViewOrDie() == s);
  EXPECT_TRUE(ro_file.GetFileContentViewOrDie() ==
              rw_file.GetFileContentViewOrDie());
}

TEST_F(MMapFileTest, TestOps) {
  const std::string file = "/a/b/c/d/e";
  EXPECT_FALSE(MMapFile(file).status().ok());
}
}  // namespace qcraft::file_util
