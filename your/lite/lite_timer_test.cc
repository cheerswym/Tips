#include "onboard/lite/lite_timer.h"

#include <functional>
#include <memory>
#include <random>
#include <thread>

#include "absl/time/time.h"
#include "folly/Singleton.h"
#include "folly/executors/CPUThreadPoolExecutor.h"
#include "folly/executors/GlobalExecutor.h"
#include "folly/executors/SerialExecutor.h"
#include "folly/executors/ThreadPoolExecutor.h"
#include "folly/init/Init.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/proto/execution_issue.pb.h"

namespace qcraft {

using ::testing::UnorderedElementsAre;

TEST(TimerManagerTest, TestOneShotTimer) {
  std::shared_ptr<SimulateClock> clock =
      std::make_shared<SimulateClock>(absl::Now());
  SetGlobalClock(clock);

  int increase_cnt = 0;
  std::function<void()> callback = [&increase_cnt]() -> void {
    increase_cnt++;
  };
  TimerManager manager;
  manager.AddTimerOrDie("add one", callback, kDisableTimerDurationCheck,
                        absl::Seconds(10), /*one_shot=*/true);
  ASSERT_TRUE(manager.HasTimers());
  EXPECT_EQ(Clock::Now() + absl::Seconds(10), manager.GetNextExpiredTime());
  EXPECT_TRUE(manager.HasTimers());
  manager.ExecuteTimers();

  EXPECT_TRUE(manager.HasTimers());
  EXPECT_EQ(increase_cnt, 0);

  clock->Sleep(absl::Seconds(9));
  ASSERT_TRUE(manager.HasTimers());
  EXPECT_EQ(Clock::Now() + absl::Seconds(1), manager.GetNextExpiredTime());

  manager.ExecuteTimers();

  clock->Sleep(absl::Seconds(1));
  ASSERT_TRUE(manager.HasTimers());
  EXPECT_EQ(Clock::Now(), manager.GetNextExpiredTime());

  manager.ExecuteTimers();
  EXPECT_EQ(increase_cnt, 1);
  EXPECT_FALSE(manager.HasTimers());
}

TEST(TimerManagerTest, TestRepeatedTimer) {
  std::shared_ptr<SimulateClock> clock =
      std::make_shared<SimulateClock>(absl::Now());
  SetGlobalClock(clock);

  int increase_cnt = 0;
  std::function<void()> callback = [&increase_cnt]() -> void {
    increase_cnt++;
  };
  TimerManager manager;
  manager.AddTimerOrDie("repeated_timer", callback, kDisableTimerDurationCheck,
                        absl::Seconds(1), false /*=one_shot*/);
  ASSERT_TRUE(manager.HasTimers());
  EXPECT_EQ(Clock::Now() + absl::Seconds(1), manager.GetNextExpiredTime());
  EXPECT_TRUE(manager.HasTimers());
  manager.ExecuteTimers();

  for (int i = 0; i < 10; i++) {
    clock->Sleep(absl::Seconds(1));
    ASSERT_TRUE(manager.HasTimers());
    EXPECT_EQ(Clock::Now(), manager.GetNextExpiredTime());
    manager.ExecuteTimers();
  }
  EXPECT_EQ(increase_cnt, 10);
  EXPECT_TRUE(manager.HasTimers());
  manager.RemoveTimer("repeated_timer");
  EXPECT_FALSE(manager.HasTimers());
}

TEST(TimerManagerTest, TestJumpOverTimer) {
  std::shared_ptr<SimulateClock> clock =
      std::make_shared<SimulateClock>(absl::Now());
  SetGlobalClock(clock);

  int increase_cnt = 0;
  std::function<void()> callback = [&increase_cnt]() -> void {
    increase_cnt++;
  };
  TimerManager manager;
  manager.AddTimerOrDie("JumpOver", callback, kDisableTimerDurationCheck,
                        absl::Seconds(1), false);
  EXPECT_EQ(Clock::Now() + absl::Seconds(1), manager.GetNextExpiredTime());
  Clock::Sleep(absl::Seconds(3));
  for (int i = 0; i < 5; i++) {
    manager.ExecuteTimers();
    EXPECT_EQ(increase_cnt, 1);
  }
  Clock::Sleep(absl::Seconds(1));
  manager.ExecuteTimers();
  EXPECT_EQ(increase_cnt, 2);
}

TEST(TimerManagerTest, TestDelayTimer) {
  int increase_cnt = 0;
  std::function<void()> callback = [&increase_cnt]() -> void {
    increase_cnt++;
  };
  TimerManager manager;
  manager.AddTimerOrDie("DelayTimer", callback, absl::Seconds(1),
                        absl::Seconds(1), false);
  EXPECT_EQ(Clock::Now() + absl::Seconds(2), manager.GetNextExpiredTime());
  Clock::Sleep(absl::Seconds(1));
  manager.ExecuteTimers();
  EXPECT_EQ(increase_cnt, 0);
  Clock::Sleep(absl::Seconds(1));
  manager.ExecuteTimers();
  EXPECT_EQ(increase_cnt, 1);
  Clock::Sleep(absl::Seconds(1));
  manager.ExecuteTimers();
  EXPECT_EQ(increase_cnt, 2);
}

TEST(CallbackManagerTest, CPUExecutor) {
  std::shared_ptr<SimulateClock> clock =
      std::make_shared<SimulateClock>(absl::Now());
  SetGlobalClock(clock);
  std::unique_ptr<folly::ThreadPoolExecutor> executor =
      std::make_unique<folly::CPUThreadPoolExecutor>(4);
  CallbackManager callback_mgr(executor.get());
  absl::Mutex m;
  std::vector<int> que;
  std::function<void(int x)> cb = [&m, &que](int x) {
    absl::MutexLock l(&m);
    que.push_back(x);
    absl::SleepFor(absl::Milliseconds(10 - x));
  };
  for (int i = 0; i < 10; i++) {
    callback_mgr.ScheduleCallback(std::bind(cb, i));
  }
  EXPECT_EQ(Clock::Now(), callback_mgr.ScheduledTime());
  callback_mgr.RunCallbacks();
  executor->join();
  EXPECT_EQ(10, que.size());
  for (const auto x : que) {
    std::cout << x << std::endl;
  }
}

TEST(CallbackManagerTest, SequenceExecutor) {
  std::shared_ptr<SimulateClock> clock =
      std::make_shared<SimulateClock>(absl::Now());
  SetGlobalClock(clock);
  auto executor = folly::SerialExecutor::create();
  // std::make_unique<folly::SerialExecutor>();
  CallbackManager callback_mgr(executor.get());
  absl::Mutex m;
  std::vector<int> que;
  std::function<void(int x)> cb = [&m, &que](int x) {
    absl::MutexLock l(&m);
    que.push_back(x);
    absl::SleepFor(absl::Milliseconds(10 - x));
  };
  for (int i = 0; i < 10; i++) {
    callback_mgr.ScheduleCallback(std::bind(cb, i));
  }
  EXPECT_EQ(Clock::Now(), callback_mgr.ScheduledTime());
  callback_mgr.RunCallbacks();
  executor.reset();
  ASSERT_EQ(10, que.size());
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(i, que[i]);
  }
}
}  // namespace qcraft

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  folly::SingletonVault::singleton()->registrationComplete();
  return RUN_ALL_TESTS();
}
