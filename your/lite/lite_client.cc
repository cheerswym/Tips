#include "onboard/lite/lite_client.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "onboard/global/clock.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport.h"

namespace qcraft {

void LiteClient::Dispatch(absl::Duration duration) {
  QCOUNTER_SPAN("Module_Dispatch");
  const absl::Time deadline = Clock::Now() + duration;
  do {
    absl::Time min_deadline = deadline;
    if (timer_manager_->HasTimers()) {
      const auto next_expired_time = timer_manager_->GetNextExpiredTime();
      if (min_deadline > next_expired_time) {
        min_deadline = next_expired_time;
      }
    }

    do {
      const auto schedule_time = callback_manager_->ScheduledTime();
      if (min_deadline > schedule_time) {
        min_deadline = schedule_time;
      }
      if (!transport_->ReceiveNextMessage(min_deadline - Clock::Now())) {
        break;
      }
    } while (true);

    // ExecuteExpiredTimers.
    timer_manager_->ExecuteTimers();

    // RunScheduledCallbacks.
    callback_manager_->RunCallbacks();
  } while (Clock::Now() < deadline);
}

MessageHub *LiteClient::GetMessageHub() const {
  auto lite_transport = dynamic_cast<LiteTransport *>(transport_.get());
  return lite_transport->GetMessageHub();
}

}  // namespace qcraft
