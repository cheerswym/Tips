#include "onboard/utils/id_generator.h"

#include <atomic>
#include <iostream>
#include <random>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

const int kSessionBits = 14;
const int kOrderBits = 6;

namespace qcraft {

namespace {

std::default_random_engine rand_engine;
std::uniform_int_distribution<uint32_t> int_distrib(0, (1 << kSessionBits) - 1);
}  // namespace

uint32_t GetSessionId() {
  rand_engine.seed(std::chrono::system_clock::now().time_since_epoch().count());
  return int_distrib(rand_engine);
}

uint64_t NextUId() {
  static uint32_t session_id = GetSessionId();
  static std::atomic<uint32_t> order(0);
  uint64_t timestamp = absl::ToUnixMillis(absl::Now());
  uint64_t current_order = order.fetch_add(1);
  uint64_t time_bits = timestamp << (kSessionBits + kOrderBits);
  uint64_t session_bits = session_id << kOrderBits;
  uint64_t order_bits = current_order & ((1 << kOrderBits) - 1);
  return time_bits | session_bits | order_bits;
}

uint64_t GetUId() {
  static std::atomic<uint64_t> last_uid(0);
  auto uid = NextUId();
  uint64_t last_seen_uid = last_uid;
  while (last_seen_uid >= uid ||
         !last_uid.compare_exchange_weak(last_seen_uid, uid)) {
    absl::SleepFor(absl::Milliseconds(1));
    uid = NextUId();
    last_seen_uid = last_uid;
  }
  return uid;
}

uint64_t GetAutoIncrementId(const std::string& bucket) {
  static absl::Mutex mu;
  static std::unordered_map<std::string, uint64_t> ids;
  absl::MutexLock l(&mu);
  uint64_t id = ids[bucket]++;
  return id;
}
}  // namespace qcraft
