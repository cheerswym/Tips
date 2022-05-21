#include "onboard/lite/transport.h"

#include <functional>
#include <memory>
#include <thread>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/lite_timer.h"
#include "onboard/lite/lite_transport.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport/message/inner_message_hub.h"

namespace qcraft {

TEST(LiteTransport, RegisterAndDispatch) {
  std::unique_ptr<Transport> transport =
      std::make_unique<LiteTransport>(GlobalInnerMessageHub());
}

}  // namespace qcraft
