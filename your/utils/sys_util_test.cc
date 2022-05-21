#include "onboard/utils/sys_util.h"

#include "gtest/gtest.h"

namespace qcraft {

TEST(SysUtilTest, TestGetHostNameByFile) {
  auto hostname = GetHostNameByFile("/etc/hostname");
  EXPECT_TRUE(hostname.ok());
  EXPECT_TRUE(!hostname->empty());
  hostname = GetHostNameByFile("/etc/non_exist_hostname");
  EXPECT_FALSE(hostname.ok());
}

TEST(SysUtilTest, TestGetHostNameByAPI) {
  const auto hostname = GetHostNameByAPI();
  EXPECT_TRUE(hostname.ok());
  EXPECT_TRUE(!hostname->empty());
}

TEST(SysUtilTest, TestGetHostNameEquivalent) {
  const auto hostname_api = GetHostNameByAPI();
  const auto hostname_file = GetHostNameByFile();
  EXPECT_TRUE(hostname_api.ok());
  EXPECT_TRUE(hostname_file.ok());
  EXPECT_EQ(*hostname_api, *hostname_file);
}

}  // namespace qcraft
