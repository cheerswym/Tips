#include "onboard/utils/progress_bar.h"

#include <chrono>
#include <thread>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gtest/gtest.h"
#include "onboard/async/parallel_for.h"

namespace qcraft {
namespace {

template <typename T>
void TestProgressBar(const T& max_cnt, const std::string& prefix) {
  ProgressBar progress_bar(max_cnt, prefix);
  int i = 0;
  while (i <= max_cnt) {
    progress_bar.Progressed(i);
    i++;
  }
}

template <typename T>
void TestProgressBarCopy(const T& max_cnt, const std::string& prefix) {
  ProgressBar progress_bar_ori(max_cnt, prefix);
  ProgressBar progress_bar = progress_bar_ori;
  int i = 0;
  while (i <= max_cnt) {
    progress_bar.Progressed(i);
    i++;
  }
}

TEST(ProgressBarTest, TestInt) {
  const int max_cnt = 10;
  TestProgressBar(max_cnt, "test");
}

TEST(ProgressBarTest, DISABLED_TestMultiThread) {
  std::vector<int> nums;
  const int total = 10;
  const int start = 1;
  const int scale = 10;
  for (int i = 0; i < total; ++i) {
    nums.push_back((i + start) * scale);
  }
  std::vector<int64_t> outputs(total);
  ParallelFor(0, total, [&](int i) {
    TestProgressBar(nums[i],
                    absl::StrCat("number : ", absl::StrFormat("%d", i)));
  });
}

TEST(ProgressBarTestCopy, TestInt) {
  const int max_cnt = 10;
  TestProgressBarCopy(max_cnt, "test");
}

TEST(ProgressBarTestCopy, DISABLED_TestMultiThread) {
  std::vector<int> nums;
  const int total = 10;
  const int start = 1;
  const int scale = 10;
  for (int i = 0; i < total; ++i) {
    nums.push_back((i + start) * scale);
  }
  std::vector<int64_t> outputs(total);
  ParallelFor(0, total, [&](int i) {
    TestProgressBarCopy(nums[i],
                        absl::StrCat("number : ", absl::StrFormat("%d", i)));
  });
}

}  // namespace
}  // namespace qcraft
