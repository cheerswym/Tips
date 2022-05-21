#include <unistd.h>

#include <cstdio>   // for remove() and printf
#include <cstdlib>  // for system()

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "benchmark/benchmark.h"
#include "boost/filesystem.hpp"
#include "onboard/utils/archive_util.h"

namespace {

std::string UniqTmpPath() {
  auto path = boost::filesystem::temp_directory_path() /
              boost::filesystem::unique_path();
  return path.native();
}

absl::Status CreateTarGzCmd(std::string_view dest, std::string_view src_dir) {
  std::string cmd = absl::StrCat("tar -czf ", dest, " ", src_dir);
  int ret = system(cmd.c_str());
  if (ret != 0) {
    // NOTE(jiaming):
    // There must be something wrong with the command. We choose to terminate
    // the calling process "immediately"
    _exit(1);
  }
  return absl::OkStatus();
}

}  // namespace

void BM_CreateTarGzCmd(benchmark::State& state) {  // NOLINT
  const int num = state.range(0);
  std::string output_path_prefix = UniqTmpPath();
  for (auto _ : state) {
    for (int i = 0; i < num; ++i) {
      state.PauseTiming();  // Stop timers. They will not count until they are
                            // resumed.
      std::string output_path =
          absl::StrCat(output_path_prefix, "_", i, ".tar.gz");
      state.ResumeTiming();  // And resume timers. They are now counting again.
      benchmark::DoNotOptimize(
          CreateTarGzCmd(output_path, "onboard/utils/testdata/archive_util"));
      state.PauseTiming();  // Stop timers. They will not count until they are
                            // resumed.
      int ret = remove(output_path.c_str());
      if (ret != 0) {
        fprintf(stderr, "Warning: failed to remove %s\n", output_path.c_str());
      }
      state.ResumeTiming();  // And resume timers. They are now counting again.
    }
  }
}

BENCHMARK(BM_CreateTarGzCmd)->Arg(10)->Arg(100)->Arg(200);

void BM_CreateTarGzLib(benchmark::State& state) {  // NOLINT
  const int num = state.range(0);
  std::string output_path_prefix = UniqTmpPath();
  for (auto _ : state) {
    for (int i = 0; i < num; ++i) {
      state.PauseTiming();  // Stop timers. They will not count until they are
                            // resumed.
      std::string output_path =
          absl::StrCat(output_path_prefix, "_", i, ".tar.gz");
      state.ResumeTiming();  // And resume timers. They are now counting again.
      benchmark::DoNotOptimize(qcraft::archive::CreateTarGz(
          output_path, {"onboard/utils/testdata/archive_util"}));
      state.PauseTiming();  // Stop timers. They will not count until they are
                            // resumed.
      int ret = remove(output_path.c_str());
      if (ret != 0) {
        fprintf(stderr, "Warning: failed to remove %s\n", output_path.c_str());
      }
      state.ResumeTiming();  // And resume timers. They are now counting again.
    }
  }
}

BENCHMARK(BM_CreateTarGzLib)->Arg(10)->Arg(100)->Arg(200);
BENCHMARK_MAIN();
