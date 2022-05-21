#ifndef ONBOARD_VIS_VANTAGE_FORWARDING_MODULE_H_
#define ONBOARD_VIS_VANTAGE_FORWARDING_MODULE_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/logging/stf_reader.h"
#include "onboard/proto/lidar.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/remote_assist/udp_update_client.h"
#include "onboard/utils/periodic_runner.h"

namespace qcraft {

// A module that forwards Lite messages to Vantage using the vantage clients.
// This module is similar to logger in that it listens to and forwards all Lite
// messages indiscriminately.
class VantageForwardingModule : public LiteModule {
 public:
  explicit VantageForwardingModule(LiteClientBase *lite_client);

  ~VantageForwardingModule();

  void OnInit() override;

  void OnSubscribeChannels() override;

  void OnSetUpTimers() override;

 private:
  void OnLiteMessage(std::shared_ptr<const LiteMsgWrapper> lite_msg);

  void OnShmMessage(std::shared_ptr<const ShmMessage> shm_message);

  void ForwardLiteMessage(const LiteMsgWrapper &lite_msg);

  void ForwardShmMessage(const ShmMessageMetadata &shm_message_metadata,
                         const ShmMessage &shm_message);

 private:
  // Channel names to filer in when running onboard mode.
  std::unordered_set<std::string> onboard_filter_white_list_channels_;
  // Channel names to filer out when running onboard mode.
  std::unordered_set<std::string> onboard_filter_black_list_channels_;

  absl::Mutex mutex_;
  boost::circular_buffer<std::shared_ptr<const LiteMsgWrapper>> lite_msg_queue_
      GUARDED_BY(mutex_);
  boost::circular_buffer<
      std::pair<ShmMessageMetadata, std::shared_ptr<const ShmMessage>>>
      shm_message_queue_ GUARDED_BY(mutex_);
  std::map<std::string, int64> shm_message_cnt_by_type_ GUARDED_BY(&mutex_);
  std::atomic<uint64_t> latest_message_ts_{0};

  // When stop is requested, we will wait for the callback thread to finish.
  absl::Notification stop_notification_;
  std::future<void> lite_publisher_future_;
  std::future<void> shm_publisher_future_;

  bool update_run_params_ = false;
  std::unique_ptr<PeriodicRunner> pull_user_input_runner_;
};

REGISTER_LITE_MODULE(VantageForwardingModule);

}  // namespace qcraft

#endif  // ONBOARD_VIS_VANTAGE_FORWARDING_MODULE_H_
