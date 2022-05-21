#include <memory>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/node_hash_map.h"
#include "benchmark/benchmark.h"

namespace qcraft {
namespace {
void BM_FlatHashMap(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::vector<int>> map;
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      map[i] = std::vector<int>{i};
    }
    for (int i = 10; i < 110; ++i) {
      map[i] = std::vector<int>{i};
      map.erase(i - 10);
    }
  }
}

void BM_FlatHashMapWithPointerValue(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
    }
    for (int i = 10; i < 110; ++i) {
      map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
      map.erase(i - 10);
    }
  }
}

void BM_NodeHashMap(benchmark::State& state) {  // NOLINT
  absl::node_hash_map<int, std::vector<int>> map;
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      map[i] = std::vector<int>{i};
    }
    for (int i = 10; i < 110; ++i) {
      map[i] = std::vector<int>{i};
      map.erase(i - 10);
    }
  }
}

void BM_UnorderedMap(benchmark::State& state) {  // NOLINT
  std::unordered_map<int, std::vector<int>> map;
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      map[i] = std::vector<int>{i};
    }
    for (int i = 10; i < 110; ++i) {
      map[i] = std::vector<int>{i};
      map.erase(i - 10);
    }
  }
}

void BM_FlatHashMap_Rehash(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::vector<int>> map;
  for (int i = 0; i < 100; ++i) {
    map[i] = std::vector<int>{i};
  }
  for (auto _ : state) {
    map.rehash(0);
  }
}

void BM_FlatHashMapWithPointerValue_Rehash(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (int i = 0; i < 100; ++i) {
    map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
  }
  for (auto _ : state) {
    map.rehash(0);
  }
}

void BM_NodeHashMap_Rehash(benchmark::State& state) {  // NOLINT
  absl::node_hash_map<int, std::vector<int>> map;
  for (int i = 0; i < 100; ++i) {
    map[i] = std::vector<int>{i};
  }
  for (auto _ : state) {
    map.rehash(0);
  }
}

void BM_FlatHashMap_PutIfAbsentWithFind(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::vector<int>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::vector<int>{i};
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      if (map.find(i) == map.end()) {
        map[i] = std::vector<int>{i};
      }
    }
  }
}

void BM_FlatHashMap_PutIfAbsentWithCount(benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::vector<int>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::vector<int>{i};
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      if (!map.count(i)) {
        map[i] = std::vector<int>{i};
      }
    }
  }
}

void BM_FlatHashMapWithPointerValue_PutIfAbsentWithFind(
    benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      if (map.find(i) == map.end()) {
        map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
      }
    }
  }
}

void BM_FlatHashMapWithPointerValue_PutIfAbsentWithCount(
    benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      if (!map.count(i)) {
        map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
      }
    }
  }
}

void BM_FlatHashMapWithPointerValue_PutIfAbsentWithEmplace(
    benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      auto [iter, inserted] = map.emplace(i, nullptr);
      if (inserted) {
        iter->second = std::make_unique<std::vector<int>>(std::vector<int>{i});
      }
    }
  }
}

void BM_FlatHashMapWithPointerValue_PutIfAbsentWithTryEmplace(
    benchmark::State& state) {  // NOLINT
  absl::flat_hash_map<int, std::unique_ptr<std::vector<int>>> map;
  for (int i = 0; i < 100; i += 2) {
    map[i] = std::make_unique<std::vector<int>>(std::vector<int>{i});
  }
  for (auto _ : state) {
    for (int i = 1; i < 100; ++i) {
      auto [iter, inserted] = map.try_emplace(i, nullptr);
      if (inserted) {
        iter->second = std::make_unique<std::vector<int>>(std::vector<int>{i});
      }
    }
  }
}

BENCHMARK(BM_FlatHashMap);
BENCHMARK(BM_FlatHashMapWithPointerValue);
BENCHMARK(BM_NodeHashMap);
BENCHMARK(BM_UnorderedMap);
BENCHMARK(BM_FlatHashMap_Rehash);
BENCHMARK(BM_FlatHashMapWithPointerValue_Rehash);
BENCHMARK(BM_NodeHashMap_Rehash);
BENCHMARK(BM_FlatHashMap_PutIfAbsentWithFind);
BENCHMARK(BM_FlatHashMap_PutIfAbsentWithCount);
BENCHMARK(BM_FlatHashMapWithPointerValue_PutIfAbsentWithFind);
BENCHMARK(BM_FlatHashMapWithPointerValue_PutIfAbsentWithCount);
BENCHMARK(BM_FlatHashMapWithPointerValue_PutIfAbsentWithEmplace);
BENCHMARK(BM_FlatHashMapWithPointerValue_PutIfAbsentWithTryEmplace);
}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
