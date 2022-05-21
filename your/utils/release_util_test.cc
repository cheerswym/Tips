#include "onboard/utils/release_util.h"

#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"

namespace qcraft::release {
namespace {

TEST(ReleaseInfo, FillChinaGood) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();

  qcraft::locale_util::SetLocale(qcraft::LOC_SHENZHEN);
  EXPECT_TRUE(qcraft::release::FillReleaseInfoWithReleaseJson(
      release_info, "onboard/utils/testdata/release_util/release.json"));

  EXPECT_EQ(release_info->release_country(), "cn");
  EXPECT_EQ(release_info->git_tag(), "release_v1");
  EXPECT_EQ(release_info->release_tag(), "release_v1");
  EXPECT_EQ(release_info->release_xavier_tag(), "xavier_v0");
  EXPECT_EQ(release_info->release_map_sha(),
            "5a633aa8e60d3f1c07014350d4308240366428ad");
}
TEST(ReleaseInfo, FillUSGood) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();

  qcraft::locale_util::SetLocale(qcraft::LOC_BAY_AREA);
  EXPECT_TRUE(qcraft::release::FillReleaseInfoWithReleaseJson(
      release_info, "onboard/utils/testdata/release_util/release.json"));

  EXPECT_EQ(release_info->release_country(), "us");
  EXPECT_EQ(release_info->git_tag(), "release_v1");
  EXPECT_EQ(release_info->release_tag(), "release_v1");
  EXPECT_EQ(release_info->release_xavier_tag(), "xavier_v0");
  EXPECT_EQ(release_info->release_map_sha(),
            "db37db6d0f3a3d400ac49d6dbfdc85ed60fc9e8e");
}
TEST(ReleaseInfo, FillDOJOBad) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();
  qcraft::locale_util::SetLocale(qcraft::LOC_DOJO);
  EXPECT_TRUE(qcraft::release::FillReleaseInfoWithReleaseJson(
      release_info, "onboard/utils/testdata/release_util/release.json"));
}
TEST(ReleaseInfo, FillBadJson) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();
  qcraft::locale_util::SetLocale(qcraft::LOC_DOJO);
  EXPECT_FALSE(qcraft::release::FillReleaseInfoWithReleaseJson(
      release_info, "onboard/utils/testdata/release_util/bad_release.json"));

  EXPECT_EQ(release_info->release_country(), "");
  EXPECT_EQ(release_info->git_tag(), "");
  EXPECT_EQ(release_info->release_tag(), "");
  EXPECT_EQ(release_info->release_xavier_tag(), "");
}

TEST(ReleaseInfo, FillDevChinaGood) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();

  qcraft::locale_util::SetLocale(qcraft::LOC_SHENZHEN);
  EXPECT_TRUE(qcraft::release::FillMapVersionWithSourceCode(release_info));
  EXPECT_EQ(release_info->release_country(), "cn");
  EXPECT_FALSE(release_info->release_map_sha().empty());
}

TEST(ReleaseInfo, FillDevUSGood) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();

  qcraft::locale_util::SetLocale(qcraft::LOC_BAY_AREA);
  EXPECT_TRUE(qcraft::release::FillMapVersionWithSourceCode(release_info));
  EXPECT_EQ(release_info->release_country(), "us");
  EXPECT_FALSE(release_info->release_map_sha().empty());
}

TEST(ReleaseInfo, FillDevBad) {
  qcraft::LiteRun lite_run;
  auto* release_info = lite_run.mutable_release_info();

  qcraft::locale_util::SetLocale(qcraft::LOC_UNKNOWN);
  EXPECT_FALSE(qcraft::release::FillMapVersionWithSourceCode(release_info));
}

}  // namespace
}  // namespace qcraft::release
