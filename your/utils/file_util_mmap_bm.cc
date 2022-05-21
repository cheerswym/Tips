#include <algorithm>
#include <sstream>
#include <string>

#include "benchmark/benchmark.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/file_util_mmap.h"

namespace qcraft::file_util {
constexpr char kFile[] = "test.txt";

// prepare data
class MyFixture : public benchmark::Fixture {
 public:
  MyFixture() {
    std::stringstream ss;
    constexpr char kFoo[] = "FooBar\n";
    // 7 bytes * 100'000 ~= 700kB
    for (int i = 0; i < 100'000; ++i) ss << kFoo;
    CHECK(SetFileContent(ss.str(), kFile));
  }
};

BENCHMARK_F(MyFixture, BM_GetFileContent)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    std::string content;
    CHECK(GetFileContent(kFile, &content));
  }
}

BENCHMARK_F(MyFixture, BM_MMapFileContent)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    QCHECK_OK(MMapFile(kFile).status());
  }
}

BENCHMARK_F(MyFixture, BM_GetFileContentMMap)
(benchmark::State& state) {  // NOLINT
  // PrepareFileContent();
  for (auto _ : state) {
    std::string content;
    CHECK(MMapFile(kFile).GetFileContent(&content));
  }
}
}  // namespace qcraft::file_util

BENCHMARK_MAIN();
