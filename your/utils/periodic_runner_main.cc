#include <iostream>

#include "onboard/global/init_qcraft.h"
#include "onboard/utils/periodic_runner.h"

int main(int argc, char* argv[]) {
  qcraft::InitQCraft(&argc, &argv);
  qcraft::PeriodicRunner runner1(absl::Seconds(1), true);
  qcraft::PeriodicRunner runner2(absl::Seconds(1), true);

  runner1.Start([]() {
    LOG(INFO) << "Small";
    absl::SleepFor(absl::Milliseconds(600));
  });
  runner2.Start([]() {
    LOG(INFO) << "Large";
    absl::SleepFor(absl::Seconds(2));
  });
  absl::SleepFor(absl::Seconds(5));
  runner1.Stop();
  runner2.Stop();
}
