#include "onboard/lite/portal/server/run_event_process.h"

#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace lite {
namespace {

template <typename T>
T *ConstructLiteSubMsg(const uint64_t &timestamp, const int &tag,
                       LiteMsgWrapper *lite_msg) {
  lite_msg->set_tag_number(tag);
  auto descriptor = lite_msg->GetDescriptor();
  auto field_desc = descriptor->FindFieldByNumber(tag);
  auto reflection = lite_msg->GetReflection();
  auto msg = static_cast<T *>(reflection->MutableMessage(lite_msg, field_desc));
  msg->mutable_header()->set_timestamp(timestamp);
  return msg;
}
}  // namespace

RunEventProcess::RunEventProcess() { RegisterProcessFunc(); }

void RunEventProcess::RegisterProcessFunc() {
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_ENGAGE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute engage action";
        auto driver_action = ConstructLiteSubMsg<DriverAction>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kDriverAction, lite_msg);
        driver_action->set_press_engage_button(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_DISENGAGE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute disengage action";
        auto driver_action = ConstructLiteSubMsg<DriverAction>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kDriverAction, lite_msg);
        driver_action->set_press_disengage_button(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_DOOR_OPEN,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute door open";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_door_override()->set_open(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_DOOR_CLOSE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute door close";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_door_override()->set_open(false);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_REQUEST_REMOTE_ASSIST,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " requesting remote assist";
        auto driver_action = ConstructLiteSubMsg<DriverAction>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kDriverAction, lite_msg);
        driver_action->set_request_remote_assist(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_NEXT_STATION,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute next station";
        auto driver_action = ConstructLiteSubMsg<DriverAction>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kDriverAction, lite_msg);
        driver_action->set_press_res_button(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_DRIVE_NOTE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute drive note";
        auto driver_action = ConstructLiteSubMsg<DriverAction>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kDriverAction, lite_msg);
        driver_action->mutable_event_note()->set_note_timestamp(
            absl::ToUnixMicros(Clock::Now()));
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_SEMANTIC_MAP_MODIFIER,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        if (!event.has_semantic_map_modifier_proto()) {
          QLOG(ERROR) << " semantic map modifier event has not map data : "
                      << timestamp;
          return false;
        }
        QLOG(WARNING) << timestamp << " execute semantic map modifier";
        auto map = ConstructLiteSubMsg<SemanticMapModificationProto>(
            timestamp,
            LiteMsgWrapper::LiteMsgCase::kSemanticMapModificationProto,
            lite_msg);
        map->mutable_modifier()->CopyFrom(event.semantic_map_modifier_proto());
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_ENABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute enable ignore traffic light";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()
            ->set_enable_traffic_light_stopping(false);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_DISABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute disable ignore traffic light";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()
            ->set_enable_traffic_light_stopping(true);
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_LEFT,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute lane change left";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_driving_action_request()
            ->mutable_lane_change()
            ->set_direction(LaneChangeRequestProto::LEFT);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_RIGHT,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute lane change right";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_driving_action_request()
            ->mutable_lane_change()
            ->set_direction(LaneChangeRequestProto::RIGHT);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_FORCE_KEEP,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute lane change force keep";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_driving_action_request()
            ->mutable_lane_change()
            ->set_direction(LaneChangeRequestProto::STRAIGHT);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_CANCEL,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute lane change cancel";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_driving_action_request()
            ->mutable_lane_change()
            ->set_direction(LaneChangeRequestProto::CANCEL);
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_ENABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp
                      << " execute enable lane change stationary objects";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()->set_enable_lc_objects(
            true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_DISABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp
                      << " execute disable lane change stationary objects";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()->set_enable_lc_objects(
            false);
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_PULL_OVER_ENABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute enable pull over";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()->set_enable_pull_over(
            true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_PULL_OVER_DISABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute disable pull over";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_enable_feature_override()->set_enable_pull_over(
            false);
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_ON,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_left_blinker_override()->set_on(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_OFF,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_left_blinker_override()->set_on(false);
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_CANCEL,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_left_blinker_override()->Clear();
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_ON,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_right_blinker_override()->set_on(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_OFF,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_right_blinker_override()->set_on(false);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_CANCEL,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_right_blinker_override()->Clear();
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_ON,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_emergency_blinker_override()->set_on(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_OFF,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_emergency_blinker_override()->set_on(false);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_CANCEL,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute turn on left blinker";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_emergency_blinker_override()->Clear();
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_PRODUCT_SETTING,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        if (!event.has_product_setting_proto()) {
          QLOG(ERROR) << " product setting event has not data : " << timestamp;
          return false;
        }
        QLOG(WARNING) << timestamp << " execute product setting";
        auto setting = ConstructLiteSubMsg<QProductSettingProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kQProductSettingProto,
            lite_msg);
        setting->mutable_product_setting_proto()->CopyFrom(
            event.product_setting_proto());
        return true;
      });

  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_STOP,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute emergency stop";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->mutable_stop_vehicle()->set_brake(1);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_ENABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute manual control enable";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->set_use_manual_control_cmd(true);
        return true;
      });
  proc_funcs_.emplace(
      QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_DISABLE,
      [](const uint64_t &timestamp, const QRunEvent &event,
         LiteMsgWrapper *lite_msg) {
        (void)event;
        QLOG(WARNING) << timestamp << " execute manual control disable";
        auto remote_assist = ConstructLiteSubMsg<RemoteAssistToCarProto>(
            timestamp, LiteMsgWrapper::LiteMsgCase::kRemoteAssistToCarProto,
            lite_msg);
        remote_assist->set_use_manual_control_cmd(false);
        return true;
      });
}

void RunEventProcess::OnProcess(const RunEventRequest &request,
                                const LiteMsgProcFunc func) {
  for (const auto &run_event : request.run_events()) {
    LiteMsgWrapper lite_msg;
    auto key = run_event.key();
    if (!ContainsKey(proc_funcs_, key)) {
      QLOG(WARNING) << "proc_funcs_ not contains key : " << key;
      continue;
    }
    if (!FindOrDie(proc_funcs_, key)(request.timestamp(), run_event,
                                     &lite_msg)) {
      continue;
    }
    if (func != nullptr) {
      func(&lite_msg);
    }
  }
}

}  // namespace lite
}  // namespace qcraft
