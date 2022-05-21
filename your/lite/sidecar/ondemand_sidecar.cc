#include "onboard/lite/sidecar/ondemand_sidecar.h"

#include "onboard/utils/thread_util.h"

namespace qcraft {

OnDemandSideCar::OnDemandSideCar() : queue_(1, "OnDemandSideCarQueue") {}

void OnDemandSideCar::Start(LiteModule *lite_module) {
  thread_ = std::make_unique<std::thread>([this, lite_module] {
    QSetThreadName("OnDemandSideCar");
    while (!stop_notification_.HasBeenNotified()) {
      queue_.take()(lite_module);
    }
  });
}

void OnDemandSideCar::AddJob(const std::function<void(LiteModule *)> &func) {
  queue_.put(func);
}

void OnDemandSideCar::Stop(LiteModule *lite_module) {
  stop_notification_.Notify();
  // add extra job to break the blocking in the queue.
  AddJob([](LiteModule *) {});
  thread_->join();
  LOG(INFO) << "OnDemandSideCar Stopped";
}

SideCar::Type OnDemandSideCar::GetType() { return SideCar::Type::ONDEMAND; }
}  // namespace qcraft
