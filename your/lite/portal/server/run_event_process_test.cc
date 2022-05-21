#include "onboard/lite/portal/server/run_event_process.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace lite {

namespace {
template <typename T>
T *ConstructLiteSubMsg(LiteMsgWrapper *lite_msg) {
  auto descriptor = lite_msg->GetDescriptor();
  auto field_desc = descriptor->FindFieldByNumber(lite_msg->tag_number());
  auto reflection = lite_msg->GetReflection();
  return static_cast<T *>(reflection->MutableMessage(lite_msg, field_desc));
}

}  // namespace

class RunEventProcessTest : public testing::Test {
 protected:
  RunEventProcessTest();

  void SetUp() override {}

  void TearDown() override {}

 private:
  void RegisterCheckFunc();

 protected:
  std::unique_ptr<RunEventProcess> process_ = nullptr;

  using CheckFunc =
      std::function<void(const uint64_t &timestamp, LiteMsgWrapper *lite_msg)>;
  using KeyCheckFuncMap = std::unordered_map<QRunEvent::Key, CheckFunc>;
  KeyCheckFuncMap check_funcs_map_;
};
RunEventProcessTest::RunEventProcessTest() {
  process_ = std::make_unique<RunEventProcess>();

  RegisterCheckFunc();
}
void RunEventProcessTest::RegisterCheckFunc() {
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_ENGAGE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<DriverAction>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->press_engage_button(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_DISENGAGE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<DriverAction>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->press_disengage_button(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_DOOR_OPEN,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->door_override().open(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_DOOR_CLOSE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->door_override().open(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_REQUEST_REMOTE_ASSIST,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<DriverAction>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->request_remote_assist(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_NEXT_STATION,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<DriverAction>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->press_res_button(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_DRIVE_NOTE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<DriverAction>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->event_note().has_note_timestamp(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_DRIVE_MISSION,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<ReroutingRequestProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->has_routing_request(), true);
        EXPECT_EQ(sub_msg->routing_request().id(), "drive mission");
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_SEMANTIC_MAP_MODIFIER,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg =
            ConstructLiteSubMsg<SemanticMapModificationProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->has_modifier(), true);
        EXPECT_EQ(sub_msg->modifier().speed_limit_modifier().max_speed_limit(),
                  60);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_ENABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(
            sub_msg->enable_feature_override().enable_traffic_light_stopping(),
            false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_TRAFFIC_LIGHT_IGNORE_DISABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(
            sub_msg->enable_feature_override().enable_traffic_light_stopping(),
            true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_LEFT,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->driving_action_request().lane_change().direction(),
                  LaneChangeRequestProto::LEFT);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_RIGHT,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->driving_action_request().lane_change().direction(),
                  LaneChangeRequestProto::RIGHT);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_FORCE_KEEP,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->driving_action_request().lane_change().direction(),
                  LaneChangeRequestProto::STRAIGHT);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_CANCEL,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->driving_action_request().lane_change().direction(),
                  LaneChangeRequestProto::CANCEL);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_ENABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->enable_feature_override().enable_lc_objects(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LANE_CHANGE_STATIONARY_OBJECTS_DISABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->enable_feature_override().enable_lc_objects(),
                  false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_PULL_OVER_ENABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->enable_feature_override().enable_pull_over(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_PULL_OVER_DISABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->enable_feature_override().enable_pull_over(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_ON,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->left_blinker_override().on(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_OFF,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->left_blinker_override().on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_LEFT_BLINKER_CANCEL,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->left_blinker_override().has_on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_ON,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->right_blinker_override().on(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_OFF,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->right_blinker_override().on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_RIGHT_BLINKER_CANCEL,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->right_blinker_override().has_on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_ON,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->emergency_blinker_override().on(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_OFF,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->emergency_blinker_override().on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_BLINKER_CANCEL,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->emergency_blinker_override().has_on(), false);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_PRODUCT_SETTING,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<QProductSettingProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->has_product_setting_proto(), true);
        EXPECT_EQ(sub_msg->product_setting_proto()
                      .scene_settings()
                      .scene_settings_size(),
                  1);
        EXPECT_EQ(sub_msg->product_setting_proto()
                      .scene_settings()
                      .scene_settings(0)
                      .scene(),
                  SceneSettingProto::SCENE_TYPE_RIGHT_TURN);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_EMERGENCY_STOP,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->stop_vehicle().brake(), 1.0);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_ENABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->use_manual_control_cmd(), true);
      });
  check_funcs_map_.emplace(
      QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_DISABLE,
      [](const uint64_t &timestamp, LiteMsgWrapper *lite_msg) {
        auto sub_msg = ConstructLiteSubMsg<RemoteAssistToCarProto>(lite_msg);
        EXPECT_EQ(sub_msg->header().timestamp(), timestamp);
        EXPECT_EQ(sub_msg->use_manual_control_cmd(), false);
      });
}
///////////////////////////////////////////////////////////////////////////////

TEST_F(RunEventProcessTest, OnProcess) {
  // set request
  int start = static_cast<int>(QRunEvent::KEY_QCOMAND_ENGAGE);
  int end = static_cast<int>(QRunEvent::KEY_QCOMAND_MANUAL_CONTROL_DISABLE) + 1;
  for (int i = start; i < end; ++i) {
    if (!QRunEvent::Key_IsValid(i)) {
      continue;
    }
    RunEventRequest req;
    req.set_timestamp(123456789);
    auto run_event = req.add_run_events();
    run_event->set_key(static_cast<QRunEvent::Key>(i));
    if (i == QRunEvent::KEY_QCOMAND_DRIVE_MISSION) {
      run_event->mutable_routing_request_proto()->set_id("drive mission");
    } else if (i == QRunEvent::KEY_QCOMAND_SEMANTIC_MAP_MODIFIER) {
      run_event->mutable_semantic_map_modifier_proto()
          ->mutable_speed_limit_modifier()
          ->set_max_speed_limit(60);
    } else if (i == QRunEvent::KEY_QCOMAND_PRODUCT_SETTING) {
      run_event->mutable_product_setting_proto()
          ->mutable_scene_settings()
          ->add_scene_settings()
          ->set_scene(SceneSettingProto::SCENE_TYPE_RIGHT_TURN);
    }

    bool is_check = false;
    LiteMsgWrapper lite_msg;
    auto get_lite_msg = [&lite_msg, &is_check](LiteMsgWrapper *msg) {
      if (msg == nullptr) {
        return;
      }
      lite_msg = *msg;
      is_check = true;
    };
    process_->OnProcess(req, get_lite_msg);

    // check
    if (!is_check) {
      LOG(WARNING) << QRunEvent::Key_Name(run_event->key()) << " not check !!!";
      continue;
    }
    auto event = req.run_events(0);
    auto iter = check_funcs_map_.find(event.key());
    if (iter == check_funcs_map_.end()) {
      LOG(ERROR) << event.key() << " not register to check_funcs_map_";
      continue;
    }
    (*iter).second(req.timestamp(), &lite_msg);
  }
}

}  // namespace lite
}  // namespace qcraft
