#ifndef ONBOARD_LITE_LITE_MODULE_H_
#define ONBOARD_LITE_LITE_MODULE_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "boost/circular_buffer.hpp"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/global/car_common.h"
#include "onboard/global/registry.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/params/param_manager.h"
#include "onboard/proto/sensor_scenario_config.pb.h"
#include "onboard/utils/errors.h"

namespace qcraft {
class LiteModule;

// auxiliary runner for the module, will live out side of the main scheduling.
// useful for performance manage runner on onboard mode
// don't use for Non onboard mode.
class SideCar {
 public:
  enum class Type {
    UNKNOWN,
    USER_INPUT,
    PERODIC,
    ONDEMAND,
  };

  static std::string TypeString(Type type) {
    switch (type) {
      case Type::UNKNOWN:
        return "UNKNOWN";
      case Type::USER_INPUT:
        return "USER_INPUT";
      case Type::PERODIC:
        return "PERODIC";
      case Type::ONDEMAND:
        return "ONDEMAND";
    }
  }
  virtual void Start(LiteModule* module) {}
  virtual void Stop(LiteModule* module) {}
  virtual SideCar::Type GetType() = 0;
  virtual void AddJob(const std::function<void(LiteModule*)>& func) {}
  virtual ~SideCar() = default;
};

// deprecated, use the api in qissue_trans.h
#define QISSUE(severity, type, msg)                         \
  LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, 0.1) \
  PublishQIssue(severity, type, QIssueSubType::QIST_UNKNOWN, msg)

class LiteModule {
 public:
  explicit LiteModule(LiteClientBase* lite_client);

  virtual ~LiteModule();

  // Subclass module should override this function.
  // Subclass module should init internal state here.
  virtual void OnInit() = 0;

  // Subclass module should override this function.
  // Subclass module should subscribe channel here.
  virtual void OnSubscribeChannels() = 0;

  // Subclass module should override this function.
  // Subclass module should set up scheduled/long running job here.
  virtual void OnSetUpTimers() = 0;

  // Module is really OK.
  virtual void Ok();

  // Init the Subclass Module
  void Init();

  // Start running module as a seperate thread, with certain duration and stop
  // notification.
  void Start(absl::Duration walking_frequence);

  // notify the module to stop
  void NotifyStop();

  // Wait for the main thread to stop
  void Join();

  // Module name in module_config.
  std::string ModuleName() const;

  LiteModuleName GetModuleName() const;

  // Report an qissue and hold the state for 2 seconds
  void PublishQIssue(QIssueSeverity severity, QIssueType type,
                     QIssueSubType sub_type, const std::string& message,
                     const std::string& args_message = "");

  // Speicalized for lidar frames.
  void SubscribeLidarFrame(
      std::function<void(const LidarFrame&)> lidar_frame_cb,
      const std::string& channel = "lidar_spin");

  void SubscribeEncodedLidarFrame(
      std::function<void(std::shared_ptr<ShmMessage>)> encoded_lidar_frame_cb);

  // Speicalized for decoded images.
  void SubscribeDecodedImage(
      std::function<void(const CameraImage&)> decoded_image_cb);

  // Speicalized for encoded images.
  void SubscribeEncodedImage(
      std::function<void(std::shared_ptr<ShmMessage>)> encoded_image_cb,
      const std::string& channel);

  void SubscribeMsgWithCallback(
      const std::string& channel, const std::string& domain,
      const std::string& field_name,
      std::function<void(std::shared_ptr<const LiteMsgWrapper>)> lite_callback,
      std::function<void(std::shared_ptr<const ShmMessage>)> shm_callback);

  void DisableAutoOk();

  void UpdateRunParams(const UpdateRunParamsProto& run_params_proto);

  void CheckSystemInfo(const SystemInfoProto& system_info_proto);

  QIssueSubType GetQIssueSubType(LiteModuleName lite_module_name);

  QIssueSubType GetQIssueSubType(std::string module_name);

  template <typename T>
  void Subscribe(std::function<void(std::shared_ptr<const T>)> cb) {
    return lite_client_->Subscribe<T>([cb, this](std::shared_ptr<const T> msg) {
      std::shared_ptr<const google::protobuf::Message> lite_msg =
          std::static_pointer_cast<const T>(msg);
      if (FilterSensorMessage(lite_msg)) {
        return;
      }
      cb(msg);
    });
  }

  template <typename T>
  void Subscribe(std::function<void(std::shared_ptr<const T>)> cb,
                 const std::string& channel) {
    return lite_client_->Subscribe<T>(
        [cb, this](std::shared_ptr<const T> msg) {
          std::shared_ptr<const google::protobuf::Message> lite_msg =
              std::static_pointer_cast<const T>(msg);
          if (FilterSensorMessage(lite_msg)) {
            return;
          }
          cb(msg);
        },
        channel);
  }

  template <typename T, typename C>
  void Subscribe(void (C::*pmethod)(std::shared_ptr<const T>), C* pclass) {
    return lite_client_->Subscribe<T>(
        [pmethod, pclass, this](std::shared_ptr<const T> msg) {
          std::shared_ptr<const google::protobuf::Message> lite_msg =
              std::static_pointer_cast<const T>(msg);
          if (FilterSensorMessage(lite_msg)) {
            return;
          }
          (pclass->*pmethod)(msg);
        });
  }

  template <typename T, typename C>
  void Subscribe(void (C::*pmethod)(std::shared_ptr<const T>), C* pclass,
                 const std::string& channel) {
    return lite_client_->Subscribe<T>(
        [pmethod, pclass, this](std::shared_ptr<const T> msg) {
          std::shared_ptr<const google::protobuf::Message> lite_msg =
              std::static_pointer_cast<const T>(msg);
          if (FilterSensorMessage(lite_msg)) {
            return;
          }
          (pclass->*pmethod)(msg);
        },
        channel);
  }

  // Publish the given lite message, and return the header on success.
  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg) {
    return lite_client_->Publish(lite_msg);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(std::unique_ptr<T> lite_msg) {
    return lite_client_->Publish(std::move(lite_msg));
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg,
                                     const std::string& channel) {
    return lite_client_->Publish(lite_msg, channel);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(std::unique_ptr<T> lite_msg,
                                     const std::string& channel) {
    return lite_client_->Publish(std::move(lite_msg), channel);
  }

  template <typename T>
  absl::StatusOr<LiteHeader> Publish(const T& lite_msg,
                                     const std::string& channel,
                                     const std::string& domain) {
    return lite_client_->Publish(lite_msg, channel, domain);
  }

  absl::StatusOr<LiteHeader> PublishShmMsg(ShmMessage* shm_msg) {
    return lite_client_->PublishShmMsg(shm_msg);
  }

  // Share message publisher and subscribers.
  absl::StatusOr<LiteHeader> PublishShmMsg(ShmMessage* shm_msg,
                                           const std::string& channel,
                                           const std::string& domain) {
    return lite_client_->PublishShmMsg(shm_msg, channel, domain);
  }

  // Publish lite wrapper message directly. The header in msg needs to be filled
  // in by yourself
  absl::StatusOr<LiteHeader> ForwardLiteMsg(LiteMsgWrapper* lite_msg) {
    return lite_client_->ForwardLiteMsg(lite_msg);
  }

  // Publish lite wrapper message with different channel and domain name.The
  // header in msg needs to be filled in by yourself
  absl::StatusOr<LiteHeader> ForwardLiteMsg(const std::string& channel,
                                            const std::string& domain,
                                            LiteMsgWrapper* lite_msg) {
    return lite_client_->ForwardLiteMsg(channel, domain, lite_msg);
  }

  absl::StatusOr<LiteHeader> PublishLiteMsgWithMeta(const std::string& channel,
                                                    const std::string& domain,
                                                    LiteMsgWrapper* lite_msg) {
    return lite_client_->PublishLiteMsgWithMeta(channel, domain, lite_msg);
  }

  // Add a timer, die if same timer_name has been added before. If `delay` is
  // greater than absl::ZeroDuration(), timer will start after waiting for
  // `delay`. See TimerManager::ExecuteTimers().
  LiteTimer& AddTimerOrDie(const std::string& timer_name,
                           std::function<void()> cb, absl::Duration delay,
                           absl::Duration period, bool one_shot = true) {
    return lite_client_->AddTimerOrDie(timer_name, std::move(cb), delay, period,
                                       one_shot);
  }

  // Same as above but disables duration check.
  LiteTimer& AddTimerOrDie(const std::string& timer_name,
                           std::function<void()> cb, absl::Duration period,
                           bool one_shot = true) {
    return lite_client_->AddTimerOrDie(timer_name, std::move(cb),
                                       absl::ZeroDuration(), period, one_shot);
  }

  template <typename C>
  LiteTimer& AddTimerOrDie(const std::string& timer_name, void (C::*pmethod)(),
                           C* pclass, absl::Duration delay,
                           absl::Duration period, bool one_shot = true) {
    return lite_client_->AddTimerOrDie(timer_name, pmethod, pclass, delay,
                                       period, one_shot);
  }

  // Same as above but disables duration check.
  template <typename C>
  LiteTimer& AddTimerOrDie(const std::string& timer_name, void (C::*pmethod)(),
                           C* pclass, absl::Duration period,
                           bool one_shot = true) {
    return lite_client_->AddTimerOrDie(timer_name, pmethod, pclass,
                                       absl::ZeroDuration(), period, one_shot);
  }

  void RemoveTimer(const std::string& timer_name) {
    lite_client_->RemoveTimer(timer_name);
  }

  // Schedule Callback
  void Schedule(std::function<void()> cb) { lite_client_->Schedule(cb); }

  void DisableInputOutputChecker() {
    lite_client_->DisableInputOutputChecker();
  }

  void DisableUpdatingMetadata() { lite_client_->DisableUpdatingMetadata(); }

  RunParamsProtoV2 GetRunParams() const {
    RunParamsProtoV2 run_params;
    param_manager_->GetRunParams(&run_params);
    return run_params;
  }

  LiteClientBase* MutableLiteClient() { return lite_client_; }

  void AddSideCar(SideCar::Type type, std::unique_ptr<SideCar> side_car) {
    side_cars_[type] = std::move(side_car);
  }

  int GetRetValue() { return ret_; }

  void SetRetValue(int value) { ret_ = value; }

  const std::vector<LiteModuleConfig>& GetAllModuleConfigs() {
    return all_module_configs_;
  }

  const LiteModuleConfig& GetCurModuleConfig() { return cur_module_config_; }

  void SetSimModuleConf(const SimModuleConf& sim_conf) { sim_conf_ = sim_conf; }

  const SimModuleConf& GetSimModuleConf() { return sim_conf_; }

  const ParamManager& param_manager() const { return *param_manager_; }

  ParamManager* mutable_param_manager() { return param_manager_.get(); }

  VehicleInstallationProto::VehiclePlan ModuleRunContext() {
    return run_context_;
  }

 protected:
  void HandleSystemCommand(std::shared_ptr<const SystemCommandProto> msg);

  void HandleAutonomyState(
      std::shared_ptr<const AutonomyStateProto> autonomy_state);

  void HandleExecutionIssue(
      std::shared_ptr<const ExecutionIssueProto> execution_issue);

  void DoOnDemandSideCarJob(const std::function<void(LiteModule*)>& func);
  struct SensorScenarioItem {
    std::unordered_set<CameraId> cameras;
    std::unordered_set<LidarId> lidars;
    std::unordered_set<RadarId> radars;
  };
  const SensorScenarioItem& GetSensorScenarioConfig() const {
    return sensor_scenario_config_;
  }

  void DisableFilterSensor() { filter_sensor_message_ = false; }

 private:
  // Subscribe system level channel automatcially
  void SubscribeSystemChannels();

  void SubscribeShmMsg(std::function<void(std::shared_ptr<ShmMessage>)> shm_cb,
                       const std::string& channel,
                       const std::string& domain = "");

  void SubscribeMsgWithCallbackInternal(
      const std::string& channel, const std::string& domain,
      const std::string& field_name,
      std::function<void(std::shared_ptr<const LiteMsgWrapper>)> lite_callback,
      std::function<void(std::shared_ptr<const ShmMessage>)> shm_callback);

  void ManageShmMessages(std::shared_ptr<const ShmMessage> shm_message);

  void PublishQTrace(const std::string& message);

  void PublishTrace(const std::string& message);

  void PublishFTrace(const std::string& message);

  bool FilterSensorMessage(std::shared_ptr<const ShmMessage> shm_message);

  bool FilterSensorMessage(
      std::shared_ptr<const google::protobuf::Message> msg);

  void InitSensorScenarioConfig();

  VehiclePose GetVehiclePose(double timestamp) const;

 private:
  LiteClientBase* lite_client_;
  boost::circular_buffer<PoseProto> pose_history_{20};
  std::atomic<VehicleInstallationProto::VehiclePlan> run_context_;
  SensorScenarioItem sensor_scenario_config_;
  LiteModuleConfig cur_module_config_;
  LaunchRunConfig::CpuItem cpu_usage_limit_;
  LaunchRunConfig::MemoryItem memory_usage_limit_;
  LaunchRunConfig::MemoryItem shared_memory_usage_limit_;

 protected:
  std::vector<LiteModuleConfig> all_module_configs_;
  SimModuleConf sim_conf_;
  std::unique_ptr<std::thread> running_thread_;
  sched_param sch_params_;
  std::atomic<bool> initialized_ = false;
  std::map<SideCar::Type, std::unique_ptr<SideCar>> side_cars_;
  absl::Notification main_thread_stop_notification_;
  size_t shm_size_ = 0;
  std::unordered_map<uint64, std::shared_ptr<const ShmMessage>>
      received_shm_messages_;
  int ret_ = 0;

  std::unique_ptr<ParamManager> param_manager_;
  bool sent_ok_ = false;
  bool auto_ok_ = true;

  std::shared_ptr<const AutonomyStateProto> last_autonomy_state_;

  std::map<CameraId, CameraParams> camera_params_;

  bool filter_sensor_message_;
};

#define REGISTER_LITE_MODULE(module_name) \
  REGISTER_SUBCLASS(LiteModule, module_name, LiteClientBase*);

}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_MODULE_H_
