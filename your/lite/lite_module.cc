#include "onboard/lite/lite_module.h"

#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <string>
#include <thread>

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "boost/bimap.hpp"
#include "glog/logging.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/autonomy_state/handlers/q_issue_util.h"
#include "onboard/base/base.h"
#include "onboard/base/base_flags.h"
#include "onboard/global/registry.h"
#include "onboard/global/trace.h"
#include "onboard/lite/lite2_flags.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/lite/scheduler/common/cpu_affinity.h"
#include "onboard/lite/sensor_scenario_config_helper.h"
#include "onboard/lite/transport.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager_factory.h"
#include "onboard/lite/transport/shm/shm_manager.h"
#include "onboard/params/param_manager.h"
#include "onboard/utils/errors.h"
#include "onboard/utils/thread_util.h"

DEFINE_int32(cpu_set_policy, 1,
             "0, 1, 2 cpu affinity set policy. 0: do not set cpu affinity.");
DECLARE_string(lite_run_dir);
DECLARE_string(car_name);
DEFINE_bool(filter_sensor_message, true,
            "Whether to filter sensor messages according to the sensor "
            "scenario config.");

namespace qcraft {
namespace {
constexpr int32 kShmTagNumber = 6;
constexpr int32 kRadarMessageVecTagNumber = 89;  // raw_radar_message_vec_proto
constexpr int32 kRadarObjectsTagNumber = 110;    // raw_radar_objects_proto

const std::vector<boost::bimap<LiteModuleName, QIssueSubType>::value_type>
    kQIssueSubTypesBimapPairs{
        {PLANNER_MODULE, QIssueSubType::QIST_PLANNER},
        {PERCEPTION_MODULE, QIssueSubType::QIST_PERCEPTION},
        {SPIN_PUBLISHER_MODULE, QIssueSubType::QIST_LIDAR},
        {VANTAGE_FORWARDING_MODULE, QIssueSubType::QIST_HMI_VANTAGE},
        {GNSS_IMU_DRIVER_MODULE, QIssueSubType::QIST_IMU},
        {POSITIONING_MODULE, QIssueSubType::QIST_POSITION},
        {VEHICLE_CONTROL_MODULE, QIssueSubType::QIST_CONTROL},
        {IMAGE_PUBLISHER_MODULE, QIssueSubType::QIST_CAMERA_PUBLISHER},
        {CAN_BUS_MODULE, QIssueSubType::QIST_CHASSIS},
        {AUTONOMY_STATE_MODULE, QIssueSubType::QIST_SYSTEM_AUTONOMY_STATE},
        {PREDICTION_MODULE, QIssueSubType::QIST_PREDICTION},
        {REMOTE_ASSIST_MODULE, QIssueSubType::QIST_HMI_QASSIST},
        {VISION_MODULE, QIssueSubType::QIST_PERCEPTION_VISION},
        {ROUTING_MODULE, QIssueSubType::QIST_PLANNER_ROUTING},
        {ROUTE_RECORDER_MODULE, QIssueSubType::QIST_PLANNER_ROUTE_RECORDER},
        {QVIEW_PUBLISHER_MODULE, QIssueSubType::QIST_HMI_QVIEW},
        {AUTONOMY_MODULE, QIssueSubType::QIST_SYSTEM_AUTONOMY},
        {V2X_MODULE, QIssueSubType::QIST_V2X},
        {LOCALIZATION_MODULE, QIssueSubType::QIST_LOCALIZATION},
        {RADAR_PUBLISHER_MODULE, QIssueSubType::QIST_RADAR},
        {QSHOW_PUBLISHER_MODULE, QIssueSubType::QIST_HMI_QSHOW},
        {IMAGE_FORWARDING_MODULE, QIssueSubType::QIST_CAMERA},
        {NODE_MODULE, QIssueSubType::QIST_NODE},
        {NODE_STATE_MODULE, QIssueSubType::QIST_NODE_STATE},
        {IMAGE_SENDER_MODULE, QIssueSubType::QIST_CAMERA_SENDER},
        {RADAR_PROCESS_MODULE, QIssueSubType::QIST_RADAR_PROCESSOR},
        {TRACKER_MODULE, QIssueSubType::QIST_PERCEPTION_TRACKER},
        {AUTONOMY_HMI_MODULE, QIssueSubType::QIST_HMI},
        {LOGGING_MODULE, QIssueSubType::QIST_SYSTEM_LOGGING},
        {OCC_MODULE, QIssueSubType::QIST_OCC},
        {EMERGENCY_BRAKE_MODULE, QIssueSubType::QIST_PLANNER_EMERGENCY_BRAKE}};
const boost::bimap<LiteModuleName, QIssueSubType> kQIssueSubTypesBimap(
    kQIssueSubTypesBimapPairs.begin(), kQIssueSubTypesBimapPairs.end());

std::string GetThreadIdStr(std::thread::id thread_id) {
  std::ostringstream ss;
  ss << thread_id;
  return ss.str();
}

bool IgnoreShmCheck(const LiteModuleName& module_name) {
  if (OnTestBench() && FLAGS_sensor_live_ignore_node_module_memory_check &&
      module_name == NODE_MODULE) {
    return true;
  }
  return false;
}

}  // namespace

LiteModule::LiteModule(LiteClientBase* lite_client)
    : lite_client_(lite_client) {
  QLOG(INFO) << "[LiteModule]     FLAGS_run_conf: " << FLAGS_run_conf;
  QLOG(INFO) << "[LiteModule]     FLAGS_car_name: " << FLAGS_car_name;
  QLOG(INFO) << "[LiteModule] FLAGS_lite_run_dir: " << FLAGS_lite_run_dir;
  QIssueTrans::Instance()->CreateClient(lite_client);
  if (OnTestBenchForRsim()) {
    if (GetModuleName() == NODE_MODULE ||
        GetModuleName() == NODE_STATE_MODULE) {
      param_manager_ = CreateParamManagerFromCarId(FLAGS_rsim_testbench_id);
    } else {
      param_manager_ = CreateParamManagerFromLiteRun(FLAGS_lite_run_dir);
    }
  } else {
    param_manager_ =
        CreateParamManager(FLAGS_run_conf, FLAGS_car_name, FLAGS_lite_run_dir);
    QCHECK(param_manager_ != nullptr);
    if (!FLAGS_specified_vehicle_params_path_in_simulation.empty()) {
      QLOG(INFO) << "FLAGS_specified_vehicle_params_path_in_simulation: "
                 << FLAGS_specified_vehicle_params_path_in_simulation;
      QCHECK_OK(param_manager_->UpdateVehicleParamsFromFile(
          FLAGS_specified_vehicle_params_path_in_simulation));
    }
  }

  all_module_configs_ = LoadModuleConfigsFromCurrentLaunchFile();
  for (int i = 0; i < all_module_configs_.size(); ++i) {
    if (all_module_configs_[i].module_name() == GetModuleName()) {
      cur_module_config_ = all_module_configs_[i];
      break;
    }
  }

  // Module resource init.
  const auto module_resource_configs =
      LoadLaunchConfigFromCurrentLaunchFile().module_resource_configs();
  LaunchRunConfig::ModuleResourceConfig cur_module_resource_config;
  for (int i = 0; i < module_resource_configs.size(); ++i) {
    if (module_resource_configs.Get(i).module_name() == GetModuleName()) {
      cur_module_resource_config = module_resource_configs.Get(i);
      break;
    }
  }

  cpu_usage_limit_ = cur_module_resource_config.cpu_config();
  memory_usage_limit_ = cur_module_resource_config.memory_config();
  shared_memory_usage_limit_ =
      cur_module_resource_config.shared_memory_config();

  if (const auto cpu_core_to_bind =
          cur_module_resource_config.cpu_core_to_bind();
      FLAGS_cpu_set_policy == 1 && !cpu_core_to_bind.empty()) {
    const std::vector<int> cpuset{cpu_core_to_bind.begin(),
                                  cpu_core_to_bind.end()};
    if (!cpuset.empty()) {
      std::string cpu_set_str = ModuleName() + " cpu set[";
      for (int i = 0; i < cpuset.size(); i++) {
        cpu_set_str += std::to_string(cpuset[i]);
        if (i != cpuset.size() - 1) {
          cpu_set_str += ":";
        }
      }
      cpu_set_str += "]";
      QLOG(INFO) << cpu_set_str;
      SetProcessAffinity(cpuset);
    } else {
      DisableProcessAffinity();
    }
  }
}

LiteModule::~LiteModule() { QLOG_NOW(INFO) << ModuleName() << " EXIT"; }

void LiteModule::Ok() {
  if (!sent_ok_) {
    QLOG(INFO) << "Send Ok: " << ModuleName();
    sent_ok_ = true;
    if (FLAGS_lite2_event_based_control) {
      SystemStateProto system_state_proto;
      auto* state = system_state_proto.mutable_module_state();
      state->set_module_name(ModuleName());
      state->set_state(SystemModuleState::OK);
      QLOG_IF_NOT_OK(WARNING, Publish(system_state_proto));
    }
  }
}

void LiteModule::Init() {
  PubSubMessageMap input_channel_domain_pairs;
  PubSubMessageMap output_channel_domain_pairs;

  for (const auto& input : cur_module_config_.inputs()) {
    std::string channel = input.field_name();
    if (!input.channel().empty()) {
      channel = input.channel();
    }
    if (input.min_interval() && input.max_interval()) {
      QCHECK_LT(input.min_interval(), input.max_interval());
    }
    input_channel_domain_pairs[std::make_pair(channel, input.domain())] =
        std::make_pair(absl::Seconds(input.min_interval()),
                       absl::Seconds(input.max_interval()));
  }
  for (const auto& output : cur_module_config_.outputs()) {
    std::string channel = output.field_name();
    if (!output.channel().empty()) {
      channel = output.channel();
    }
    if (output.min_interval() && output.max_interval()) {
      QCHECK_LT(output.min_interval(), output.max_interval());
    }
    output_channel_domain_pairs[std::make_pair(channel, output.domain())] =
        std::make_pair(absl::Seconds(output.min_interval()),
                       absl::Seconds(output.max_interval()));
  }

  if (input_channel_domain_pairs.size() > 0 ||
      output_channel_domain_pairs.size() > 0) {
    lite_client_->SetInputOutputChecker(input_channel_domain_pairs,
                                        output_channel_domain_pairs,
                                        cur_module_config_);
  }

  SetQCraftRunContext(GetRunParams());
  run_context_ = QCraftRunContext();
  QLOG(INFO) << "run_context:"
             << VehicleInstallationProto_VehiclePlan_Name(run_context_.load());
  filter_sensor_message_ = FLAGS_filter_sensor_message;
  // Do not filter sensor message in Q_DBQ_V2 mode
  if (run_context_ == VehicleInstallationProto_VehiclePlan_VP_DBQ_V2) {
    filter_sensor_message_ = false;
  }

  InitSensorScenarioConfig();

  const RunParamsProtoV2 run_params = GetRunParams();
  camera_params_ = ComputeAllCameraParams(run_params.vehicle_params());

  // Init module internal logic
  OnInit();

  // Subscribe system level channel
  SubscribeSystemChannels();

  // Subscribe module channel
  OnSubscribeChannels();

  // Set up module perodic/long running logic.
  OnSetUpTimers();

  // Instantly start the module in simulation.
  if (IsDSimMode() && (!FLAGS_lite2_event_based_control)) {
    Ok();
  }
  initialized_ = true;
}

void LiteModule::Start(absl::Duration walking_frequence) {
  CHECK(initialized_)
      << ModuleName()
      << " is not Initialized, please call Init() before run the module";
  running_thread_ = std::make_unique<std::thread>([&, walking_frequence] {
    QSetThreadName("LiteModRunning");
    // Publish Ready Event along with subscription info
    if (FLAGS_lite2_event_based_control) {
      SystemStateProto system_state_proto;
      auto* state = system_state_proto.mutable_module_state();
      state->set_module_name(ModuleName());
      state->set_state(SystemModuleState::READY);
      auto subs = lite_client_->GetSubDomainChannels();
      if (!subs.empty()) {
        for (auto it = subs.begin(); it != subs.end(); ++it) {
          auto* channels =
              state->mutable_meta()->mutable_subscription()->add_sub_channels();
          channels->set_channel((*it).first);
          channels->set_number((*it).second);
        }
      }

      QLOG_IF_NOT_OK(WARNING, Publish(system_state_proto));
    }

    if (auto_ok_) {
      Ok();
    }

    // enter mainloop
    while (!main_thread_stop_notification_.HasBeenNotified()) {
      ScopedTrace scoped_trace("LiteClientSleep", "module", ModuleName(),
                               false);
      lite_client_->SleepUntil(walking_frequence);
    }
  });
  if (IsOnboardMode()) {
    // start the sidecar
    for (auto& kv : side_cars_) {
      kv.second->Start(this);
    }

    // start the heart beat
    if (FLAGS_lite2_multiprocess) {
      AddTimerOrDie(
          "heart beat",
          [this]() {
            SystemStateProto system_state_proto;
            auto* heart_beat = system_state_proto.mutable_module_heart_beat();
            heart_beat->set_module_name(ModuleName());
            QLOG_IF_NOT_OK(WARNING, Publish(system_state_proto));
          },
          absl::Milliseconds(200), false);
    }
  }
}

void LiteModule::NotifyStop() {
  QLOG(INFO) << "NotifyStop " << ModuleName();
  if (FLAGS_lite2_event_based_control) {
    SystemStateProto system_state_proto;
    auto* state = system_state_proto.mutable_module_state();
    state->set_module_name(ModuleName());
    state->set_state(SystemModuleState::DYING);
    QLOG_IF_NOT_OK(WARNING, Publish(system_state_proto));
  }

  lite_client_->Stop();

  if (IsOnboardMode() && !side_cars_.empty()) {
    for (auto& kv : side_cars_) {
      QLOG(INFO) << "Stopping side car:" << SideCar::TypeString(kv.first);
      kv.second->Stop(this);
    }
  }
  if (!main_thread_stop_notification_.HasBeenNotified()) {
    main_thread_stop_notification_.Notify();
  }
}

void LiteModule::Join() {
  if (running_thread_) {
    running_thread_->join();
  }
}

std::string LiteModule::ModuleName() const {
  if (FLAGS_lite_pressure_test) {
    // Module id is reserved for oneoff usage >= 200000
    if (GetModuleName() >= 200000) {
      return absl::StrCat("MOCK_LITE_PRESSURE_TEST_MODULE_", GetModuleName());
    }
  }
  return LiteModuleName_Name(GetModuleName());
}

LiteModuleName LiteModule::GetModuleName() const {
  return lite_client_->GetModuleName();
}

void LiteModule::PublishQIssue(QIssueSeverity severity, QIssueType type,
                               QIssueSubType sub_type,
                               const std::string& message,
                               const std::string& args_message) {
  ExecutionIssueProto execution_issue;
  auto* issue = execution_issue.mutable_issue();
  issue->set_severity(severity);
  issue->set_type(type);
  issue->set_sub_type(sub_type);
  issue->set_message(message);
  if (!args_message.empty()) {
    issue->set_args_message(args_message);
  }

  QLOG_IF_NOT_OK(WARNING, lite_client_->Publish(execution_issue));
}

void LiteModule::SubscribeLidarFrame(
    std::function<void(const LidarFrame&)> lidar_frame_cb,
    const std::string& channel) {
  SubscribeShmMsg(
      [lidar_frame_cb](std::shared_ptr<ShmMessage> shm_message) {
        const LidarFrame lidar_frame(shm_message);
        lidar_frame_cb(lidar_frame);
      },
      channel);
}

void LiteModule::SubscribeEncodedLidarFrame(
    std::function<void(std::shared_ptr<ShmMessage>)> encoded_lidar_frame_cb) {
  SubscribeShmMsg(
      [encoded_lidar_frame_cb](std::shared_ptr<ShmMessage> shm_message) {
        encoded_lidar_frame_cb(shm_message);
      },
      "encoded_lidar_frame");
}

void LiteModule::SubscribeDecodedImage(
    std::function<void(const CameraImage&)> decoded_image_cb) {
  SubscribeShmMsg(
      [decoded_image_cb, this](std::shared_ptr<ShmMessage> shm_message) {
        const auto camera_id =
            shm_message->shm_msg_metadata()
                .GetExtension(DecodedImageMetadata::decoded_image_meta)
                .camera_id();
        const CameraImage camera_image(std::move(shm_message),
                                       FindOrDie(camera_params_, camera_id));
        decoded_image_cb(camera_image);
      },
      "decoded_image");
}

void LiteModule::SubscribeEncodedImage(
    std::function<void(std::shared_ptr<ShmMessage>)> encoded_image_cb,
    const std::string& channel) {
  const std::set<std::string> kPredefinedEncodedImageChannels{
      "encoded_image", "raw_encoded_image", "downsized_encoded_image"};
  QCHECK(ContainsKey(kPredefinedEncodedImageChannels, channel));
  SubscribeShmMsg(
      [encoded_image_cb](std::shared_ptr<ShmMessage> shm_message) {
        encoded_image_cb(shm_message);
      },
      channel);
}

void LiteModule::SubscribeMsgWithCallback(
    const std::string& channel, const std::string& domain,
    const std::string& field_name,
    std::function<void(std::shared_ptr<const LiteMsgWrapper>)> lite_callback,
    std::function<void(std::shared_ptr<const ShmMessage>)> shm_callback) {
  SubscribeMsgWithCallbackInternal(channel, domain, field_name, lite_callback,
                                   shm_callback);
}

void LiteModule::DisableAutoOk() { auto_ok_ = false; }

void LiteModule::UpdateRunParams(const UpdateRunParamsProto& run_params_proto) {
  QCHECK(param_manager_->UpdateRunParams(run_params_proto.run_params()).ok())
      << "Update run params failed";

  if (!IsOnboardMode()) {
    const RunParamsProtoV2 run_params = GetRunParams();
    camera_params_ = ComputeAllCameraParams(run_params.vehicle_params());
  }
}
void LiteModule::CheckSystemInfo(const SystemInfoProto& system_info_proto) {
  const auto cpu_usage = system_info_proto.cpu_info().usage_amount();
  QCOUNTER("cpu_usage_per_module", cpu_usage);
  if (FLAGS_cpu_usage_check && !OnTestBenchForRsim()) {
    if (cpu_usage >= cpu_usage_limit_.kickout_deadline()) {
      const auto message =
          absl::StrCat("Check module cpu usage: ", ModuleName());
      const auto args_message =
          absl::StrCat("limit value: ", cpu_usage_limit_.kickout_deadline(),
                       "%, actual value: ", cpu_usage, "%");
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_RESOURCE,
                        GetQIssueSubType(GetModuleName()), message,
                        args_message);
    }
  }

  const auto memory_usage_mb = system_info_proto.mem_info().used_kb() >> 10;
  QCOUNTER("memory_usage_per_module", memory_usage_mb);
  if (FLAGS_memory_usage_check && GetModuleName() != RSIM_PLAYBACK_MODULE) {
    if (memory_usage_mb >= memory_usage_limit_.kickout_deadline()) {
      const auto message =
          absl::StrCat("Check module physical memory usage: ", ModuleName());
      const auto args_message =
          absl::StrCat("limit value: ", memory_usage_limit_.kickout_deadline(),
                       "MB, actual value: ", memory_usage_mb, "MB");
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_RESOURCE,
                        GetQIssueSubType(GetModuleName()), message,
                        args_message);
    }
  }
}

QIssueSubType LiteModule::GetQIssueSubType(LiteModuleName lite_module_name) {
  const auto iter = kQIssueSubTypesBimap.left.find(lite_module_name);
  if (iter != kQIssueSubTypesBimap.left.end()) {
    return iter->second;
  } else {
    return QIssueSubType::QIST_UNKNOWN;
  }
}

QIssueSubType LiteModule::GetQIssueSubType(std::string module_name) {
  const auto* litemodule_descriptor = LiteModuleName_descriptor();
  auto module_name_new = module_name;
  if (module_name_new.find("MOCK_LITE_PRESSURE_TEST_MODULE") !=
      std::string::npos) {
    module_name_new = "MOCK_LITE_PRESSURE_TEST_MODULE";
  }
  const auto lite_module_name = static_cast<LiteModuleName>(
      litemodule_descriptor->FindValueByName(module_name_new)->number());
  return GetQIssueSubType(lite_module_name);
}

void LiteModule::HandleSystemCommand(
    std::shared_ptr<const SystemCommandProto> msg) {
  if (msg->has_module_command()) {
    if (msg->module_command().module_name() == ModuleName()) {
      VLOG(5) << "HandleSystemCommand[" << ModuleName() << "] "
              << msg->DebugString() << std::endl;
      if (msg->module_command().command() == SystemCommandProto::STOP) {
        NotifyStop();
      }
    }
  }
}

void LiteModule::HandleAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy_state) {
  do {
    if (last_autonomy_state_ == nullptr) {
      break;
    }
    if (!IS_KICKOUT(last_autonomy_state_->autonomy_state(),
                    autonomy_state->autonomy_state())) {
      break;
    }
    if (!autonomy_state->has_reason()) {
      break;
    }
    const auto& reason = autonomy_state->reason();
    if ((reason.issue_size() == 0) || (reason.failed_issues_size() == 0)) {
      break;
    }
    if (QISSUE_HAS_IGNORE_TRACE_ISSUE(reason.issue(0))) {
      break;
    }
    PublishQTrace(autonomy_state->reason().failed_issues(0));
  } while (false);
  last_autonomy_state_ = autonomy_state;
}

void LiteModule::HandleExecutionIssue(
    std::shared_ptr<const ExecutionIssueProto> execution_issue) {
  do {
    if (last_autonomy_state_ == nullptr) {
      break;
    }
    if (last_autonomy_state_->autonomy_state() == AutonomyStateProto::INIT ||
        last_autonomy_state_->autonomy_state() ==
            AutonomyStateProto::SHUTDOWN) {
      break;
    }
    if (!execution_issue->has_issue()) {
      break;
    }
    const auto& issue = execution_issue->issue();
    if (issue.severity() != QIssueSeverity::QIS_DEBUG) {
      break;
    }
    std::string message("\n ========DEBUG MAIN REASON========\n");
    message = absl::StrCat(message, issue.DebugString());
    message = absl::StrCat(message, "====================================\n");
    PublishQTrace(message);
  } while (false);
}

void LiteModule::DoOnDemandSideCarJob(
    const std::function<void(LiteModule*)>& func) {
  auto side_car = FindOrNull(side_cars_, SideCar::Type::ONDEMAND);
  if (side_car) {
    (*side_car)->AddJob(func);
  } else {
    QLOG(ERROR) << "not on-demand side car";
  }
}

void LiteModule::SubscribeSystemChannels() {
  if (ModuleName() != "AUTONOMY_MODULE") {
    VLOG(5) << "SubscribeSystemChannels:" << ModuleName();
    Subscribe(&LiteModule::HandleSystemCommand, this);
  }
  Subscribe(&LiteModule::HandleAutonomyState, this);
  Subscribe(&LiteModule::HandleExecutionIssue, this);
}

void LiteModule::SubscribeShmMsg(
    std::function<void(std::shared_ptr<ShmMessage>)> shm_cb,
    const std::string& channel, const std::string& domain) {
  lite_client_->SubscribeShmMsg(
      [this,
       shm_cb = std::move(shm_cb)](std::shared_ptr<ShmMessage> shm_message) {
        if (FilterSensorMessage(shm_message)) {
          return;
        }
        ManageShmMessages(shm_message);
        shm_cb(std::move(shm_message));
      },
      channel, domain);
}

void LiteModule::SubscribeMsgWithCallbackInternal(
    const std::string& channel, const std::string& domain,
    const std::string& field_name,
    std::function<void(std::shared_ptr<const LiteMsgWrapper>)> lite_callback,
    std::function<void(std::shared_ptr<const ShmMessage>)> shm_callback) {
  lite_client_->SubscribeMsgWithCallback(
      channel, domain,
      std::make_unique<LiteMsgWrapperCallback>(
          [this, lite_callback = std::move(lite_callback),
           shm_callback = std::move(shm_callback)](
              std::shared_ptr<const LiteMsgWrapper> lite_msg) {
            if (lite_msg->tag_number() == kShmTagNumber) {
              const ShmMessageMetadata& shm_message_metadata =
                  static_cast<const ShmMessageMetadata&>(
                      LiteMsgConverter::Get().GetLiteMsgField(*lite_msg));
              std::shared_ptr<const ShmMessage> shm_message =
                  ShmMessage::CreateToRead(shm_message_metadata);
              ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(
                  shm_message_metadata);
              ManageShmMessages(shm_message);
              if (shm_callback != nullptr) {
                shm_callback(std::move(shm_message));
              }
            } else {
              if (lite_callback != nullptr) {
                lite_callback(std::move(lite_msg));
              }
            }
          },
          field_name));
}

void LiteModule::ManageShmMessages(
    std::shared_ptr<const ShmMessage> shm_message) {
  // Clear unused shmMessages.
  for (auto iter = received_shm_messages_.begin();
       iter != received_shm_messages_.end();) {
    if ((*iter).second.use_count() == 1) {
      shm_size_ -= (*iter).second->buffer_size();
      received_shm_messages_.erase(iter++);
    } else {
      ++iter;
    }
  }

  // The message on a handle is sent multiple times without counting the
  // memory.
  auto handle = shm_message->shm_msg_metadata().cross_proc().handle();
  if (received_shm_messages_.count(handle) != 0) {
    return;
  }
  received_shm_messages_[handle] = std::move(shm_message);

  shm_size_ += received_shm_messages_[handle]->buffer_size();
  const auto shm_size_mb = shm_size_ >> 20;
  if (FLAGS_shared_memory_usage_check && !IgnoreShmCheck(GetModuleName())) {
    if (shm_size_mb > shared_memory_usage_limit_.kickout_deadline()) {
      const auto message =
          absl::StrCat("Check module share memory usage: ", ModuleName());
      const auto args_message = absl::StrCat(
          "limit value: ", shared_memory_usage_limit_.kickout_deadline(),
          "MB, actual value: ", shm_size_mb, "MB");
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_RESOURCE,
                        GetQIssueSubType(GetModuleName()), message,
                        args_message);
    }
  }
  // Megabytes to dashboard view.
  QCOUNTER("shm_size_per_module", shm_size_mb);
}

void LiteModule::PublishQTrace(const std::string& message) {
  if (!Trace::Instance()->ShouldDump()) {
    return;
  }
  DoOnDemandSideCarJob([this, message](LiteModule* lite_module) {
    if (!Trace::Instance()->ShouldDump()) {
      return;
    }
    if (FLAGS_lite2_multiprocess) {
      PublishTrace(message);
      const auto module_name = GetModuleName();
      if (module_name == NODE_STATE_MODULE) {
        PublishFTrace(message);
      }
    } else {
      const auto module_name = GetModuleName();
      if (module_name == NODE_STATE_MODULE) {
        PublishTrace(message);
        PublishFTrace(message);
      }
    }
  });
}

void LiteModule::PublishTrace(const std::string& message) {
  SCOPED_QTRACE_ARG1("MarkTrace", "timestamp", absl::ToUnixMillis(absl::Now()));

  MetaEvent meta_event;
  meta_event.process_id = std::to_string(getpid());
  meta_event.thread_id = GetThreadIdStr(std::this_thread::get_id());
  meta_event.thread_name = GetThreadIdStr(std::this_thread::get_id());
  meta_event.process_name = ModuleName();
  Trace::Instance()->AddMetaEvent(meta_event);

  auto trace_proto = Trace::Instance()->DumpTrace(message);
  if (trace_proto.has_value()) {
    SCOPED_QTRACE("PublishTrace");
    QLOG_IF_NOT_OK(WARNING, Publish(trace_proto.value()));
  }
}

void LiteModule::PublishFTrace(const std::string& message) {
  SCOPED_QTRACE_ARG1("MarkFTrace", "timestamp",
                     absl::ToUnixMillis(absl::Now()));

  auto trace_proto = Trace::Instance()->DumpFTrace(message);
  if (trace_proto.has_value()) {
    SCOPED_QTRACE("PublishFTrace");
    QLOG_IF_NOT_OK(WARNING, Publish(trace_proto.value()));
  }
}

bool LiteModule::FilterSensorMessage(
    std::shared_ptr<const google::protobuf::Message> msg) {
  if (!filter_sensor_message_) {
    return false;
  }
  const auto& converter = qcraft::LiteMsgConverter::Get();
  const auto& header = converter.GetLiteMessageHeader(*(msg.get()));
  const auto tag_number = header.tag_number();
  qcraft::RadarId radar_id;
  if (tag_number == kRadarMessageVecTagNumber) {
    auto radar_msg =
        std::dynamic_pointer_cast<const RawRadarMessageVecProto>(msg);
    radar_id = radar_msg->radar_id();
    if (sensor_scenario_config_.radars.count(radar_id) == 0) {
      QLOG_EVERY_N_SEC(WARNING, 1) << "Filter out RawRadarMessageVecProto:"
                                   << qcraft::RadarId_Name(radar_id);
      return true;
    }
  } else if (tag_number == kRadarObjectsTagNumber) {
    auto radar_msg = std::dynamic_pointer_cast<const RawRadarObjectsProto>(msg);
    radar_id = radar_msg->radar_id();
    if (sensor_scenario_config_.radars.count(radar_id) == 0) {
      QLOG_EVERY_N_SEC(WARNING, 1) << "Filter out RawRadarObjectsProto:"
                                   << qcraft::RadarId_Name(radar_id);
      return true;
    }
  }
  return false;
}

bool LiteModule::FilterSensorMessage(
    std::shared_ptr<const ShmMessage> shm_message) {
  if (!filter_sensor_message_) {
    return false;
  }
  const auto& metadata = shm_message->shm_msg_metadata();
  const auto shm_msg_type = metadata.shm_msg_type();
  if (shm_msg_type == SHM_LIDAR_FRAME) {
    const auto& spin_meta = metadata.GetExtension(SpinMetadata::spin_meta);
    const auto& lidar_id = spin_meta.id();
    if (sensor_scenario_config_.lidars.count(lidar_id) == 0) {
      QLOG_EVERY_N_SEC(WARNING, 1)
          << "Filter out SHM_LIDAR_FRAME:" << qcraft::LidarId_Name(lidar_id);
      return true;
    }
  } else if (shm_msg_type == SHM_ENCODED_IMAGE) {
    const auto& encoded_image_meta =
        metadata.GetExtension(EncodedImageMetadata::encoded_image_meta);
    const auto& camera_id = encoded_image_meta.camera_id();
    if (sensor_scenario_config_.cameras.count(camera_id) == 0) {
      QLOG_EVERY_N_SEC(WARNING, 1) << "Filter out SHM_ENCODED_IMAGE:"
                                   << qcraft::CameraId_Name(camera_id);
      return true;
    }
  } else if (shm_msg_type == SHM_DECODED_IMAGE) {
    const auto& decoded_image_meta =
        metadata.GetExtension(DecodedImageMetadata::decoded_image_meta);
    const auto& camera_id = decoded_image_meta.camera_id();
    if (sensor_scenario_config_.cameras.count(camera_id) == 0) {
      QLOG_EVERY_N_SEC(WARNING, 1) << "Filter out SHM_DECODED_IMAGE:"
                                   << qcraft::CameraId_Name(camera_id);
      return true;
    }
  } else {
    QLOG(FATAL) << "Unexpected shared memory message type.";
  }
  return false;
}

void LiteModule::InitSensorScenarioConfig() {
  const auto& sensor_scenario_config = QCraftSensorScenarioConfig();
  const auto& GetSensorScenario =
      [](const SensorScenario& sensor_scenario) -> auto {
    SensorScenarioItem scenario_item{};
    for (int i = 0; i < sensor_scenario.cameras().size(); i++) {
      scenario_item.cameras.insert(sensor_scenario.cameras(i));
    }
    for (int i = 0; i < sensor_scenario.lidars().size(); i++) {
      scenario_item.lidars.insert(sensor_scenario.lidars(i));
    }
    for (int i = 0; i < sensor_scenario.radars().size(); i++) {
      scenario_item.radars.insert(sensor_scenario.radars(i));
    }
    return scenario_item;
  };

  bool inited = false;
  for (int i = 0; i < sensor_scenario_config.sensor_scenarios().size(); i++) {
    const auto& sensor_scenario = sensor_scenario_config.sensor_scenarios(i);
    if (run_context_ == QCraftRunContext(sensor_scenario.scenario())) {
      sensor_scenario_config_ = GetSensorScenario(sensor_scenario);
      inited = true;
      break;
    }
  }
  auto run_context_str =
      VehicleInstallationProto_VehiclePlan_Name(run_context_.load());
  if (!inited) {
    QLOG(WARNING) << "Not found " << run_context_str
                  << " sensor scenaio config in "
                  << FLAGS_sensor_scenario_config_file;
  }
}
}  // namespace qcraft
