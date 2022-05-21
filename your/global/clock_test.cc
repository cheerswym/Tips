#include "onboard/global/clock.h"

#include <thread>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {

TEST(SimulteClockTest, InitializedValue) {
  std::shared_ptr<SimulateClock> clock = std::make_shared<SimulateClock>();
  SetGlobalClock(clock);
  EXPECT_EQ(Clock::Now(), absl::UnixEpoch());
}

TEST(SimulteClockTest, SimulateClockSleep) {
  std::shared_ptr<SimulateClock> clock = std::make_shared<SimulateClock>();
  SetGlobalClock(clock);
  clock->SetTime(absl::Now());
  absl::Time now = Clock::Now();

  now += absl::Seconds(2);
  Clock::Sleep(absl::Seconds(2));
  EXPECT_EQ(Clock::Now(), now);

  Clock::Sleep(absl::ZeroDuration());
  EXPECT_EQ(Clock::Now(), now);
}

TEST(SimulteClockTest, ThreadMultiThreadsSleep) {
  std::shared_ptr<SimulateClock> clock = std::make_shared<SimulateClock>();
  SetGlobalClock(clock);
  const absl::Time now = Clock::Now();

  auto advance_clock = [&clock](absl::Duration duration) {
    clock->Sleep(duration);
  };
  auto advance_global_clock = [](absl::Duration duration) {
    Clock::Sleep(duration);
  };

  std::thread t1(advance_clock, absl::Seconds(1));
  std::thread t2(advance_global_clock, absl::Seconds(2));
  std::thread t3(advance_clock, absl::Seconds(3));
  std::thread t4(advance_global_clock, absl::Seconds(4));

  EXPECT_GE(Clock::Now(), now);

  t1.join();
  t2.join();
  t3.join();
  t4.join();
  EXPECT_EQ(Clock::Now(), now + absl::Seconds(10));
}

TEST(ChronoSystemClockTest, Normal) {
  std::shared_ptr<ChronoSystemClock> clock =
      std::make_shared<ChronoSystemClock>();
  SetGlobalClock(clock);
  const absl::Time now = Clock::Now();
  EXPECT_EQ(absl::ToUnixMillis(absl::Now()), absl::ToUnixMillis(now));
  EXPECT_GE(Clock::Now(), now);
}

TEST(ChronoSteadyClockTest, Normal) {
  std::shared_ptr<ChronoSteadyClock> clock =
      std::make_shared<ChronoSteadyClock>();
  SetGlobalClock(clock);
  const absl::Time now = Clock::Now();
  EXPECT_EQ(absl::ToUnixMillis(absl::Now()), absl::ToUnixMillis(now));
  EXPECT_GE(Clock::Now(), now);
}

}  // namespace qcraft
