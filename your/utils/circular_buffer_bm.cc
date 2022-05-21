#include <deque>
#include <list>
#include <vector>

#include "benchmark/benchmark.h"
#include "boost/circular_buffer.hpp"

namespace qcraft {
namespace {

void BM_CircularBuffer(benchmark::State& state) {  // NOLINT
  boost::circular_buffer<int> buffer(128);
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      buffer.push_back(i);
    }
    for (int i = 0; i < 100; ++i) {
      buffer.push_back(i);
      buffer.pop_front();
    }
  }
}

void BM_Deque(benchmark::State& state) {  // NOLINT
  std::deque<int> buffer(128);
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      buffer.push_back(i);
    }
    for (int i = 0; i < 100; ++i) {
      buffer.push_back(i);
      buffer.pop_front();
    }
  }
}

void BM_Vector(benchmark::State& state) {  // NOLINT
  std::vector<int> buffer;
  buffer.reserve(128);
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      buffer.push_back(i);
    }
    for (int i = 0; i < 100; ++i) {
      buffer.push_back(i);
      buffer.erase(buffer.begin());
    }
  }
}

void BM_List(benchmark::State& state) {  // NOLINT
  std::list<int> buffer;
  for (auto _ : state) {
    for (int i = 0; i < 10; ++i) {
      buffer.push_back(i);
    }
    for (int i = 0; i < 100; ++i) {
      buffer.push_back(i);
      buffer.pop_front();
    }
  }
}

BENCHMARK(BM_CircularBuffer);
BENCHMARK(BM_Deque);
BENCHMARK(BM_Vector);
BENCHMARK(BM_List);

}  // namespace
}  // namespace qcraft

BENCHMARK_MAIN();
