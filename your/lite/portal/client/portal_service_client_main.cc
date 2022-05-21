#include <grpc++/grpc++.h>

#include <string>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/global/init_qcraft.h"
#include "onboard/lite/portal/client/portal_service_client.h"
#include "onboard/proto/port.pb.h"

DEFINE_string(server_addr_ip, "0.0.0.0", "Portal Service server address");
DEFINE_int32(server_addr_port, qcraft::PORTAL_SERVICE_PORT,
             "Portal Service server port");

void TestRemoteAssit(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::CommandRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  auto *command = request.mutable_command();
  command->set_type(qcraft::QCommand::TRAFFIC_LIGHT_IGNORE_ENABLE);
  qcraft::lite::CommandResponse response;
  auto status = client->Command(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Remote assit test[OK]: " << response.DebugString();

  } else {
    LOG(ERROR) << "Remote assit test[FAILED]: " << status.error_message();
  }
}

void TestDriveAction(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::CommandRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  auto *command = request.mutable_command();
  command->set_type(qcraft::QCommand::ENGAGE);
  qcraft::lite::CommandResponse response;
  auto status = client->Command(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Drive action test[OK]: " << response.DebugString();

  } else {
    LOG(ERROR) << "Drive action test[FAILED]: " << status.error_message();
  }
}

void TestState(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::StateRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  grpc::ClientContext context;
  auto reader = client->State(&context, request);
  qcraft::lite::StateResponse response;
  while (reader->Read(&response)) {
    LOG(INFO) << "State test[OK]: " << response.DebugString();
  }
  auto status = reader->Finish();
  if (!status.ok()) {
    LOG(ERROR) << "State test[FAILED]: " << status.error_message();
  }
}

void TestInfo(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::InfoRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  qcraft::lite::InfoResponse response;
  auto status = client->Info(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Info test[OK]: " << response.DebugString();

  } else {
    LOG(ERROR) << "Info test[FAILED]: " << status.error_message();
  }
}

void TestDriveRoute(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::CommandRequest request;
  // QR00002
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  auto *command = request.mutable_command();
  command->set_type(qcraft::QCommand::DRIVE_ROUTE);
  auto *drive_route = command->mutable_drive_route();
  drive_route->set_infinite_loop(true);
  drive_route->add_stops()->set_stop_name("changsanjiaoshequ1");
  drive_route->add_stops()->set_stop_name("jiangnandasha");
  drive_route->add_stops()->set_stop_name("yuanrongguangchang");
  drive_route->add_stops()->set_stop_name("ziguangdasha");
  drive_route->add_stops()->set_stop_name("jinkejiudianbei1");
  auto *stops = drive_route->add_stops();
  stops->set_stop_name("qinglongganglulugangjie");
  auto *geo_point = stops->add_via_points()->mutable_geo_point();
  geo_point->set_latitude(2.1055854219467824);
  geo_point->set_longitude(0.5483771079143);
  drive_route->add_stops()->set_stop_name("zhaorundasha");
  drive_route->add_stops()->set_stop_name("changsanjiaoshequ2");
  qcraft::lite::CommandResponse response;
  auto status = client->Command(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Drive route test[OK]: " << response.DebugString();
  } else {
    LOG(ERROR) << "Drive route test[FAILED]: " << status.error_message();
  }
}

void TestSemanticMapOverride(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::CommandRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  auto *command = request.mutable_command();
  command->set_type(qcraft::QCommand::SEMANTIC_MAP_OVERRIDE);
  auto *semantic_map_override = command->mutable_semantic_map_override();
  auto *override_zone = semantic_map_override->mutable_override_zone();
  override_zone->set_limited_speed(50);
  auto *lane = override_zone->add_lanes();
  lane->set_lane(2);
  lane->set_limited_speed(40);
  lane = override_zone->add_lanes();
  lane->set_lane(1);
  lane->set_limited_speed(30);
  auto *region = override_zone->add_regions();
  region->set_limited_speed(40);
  qcraft::lite::CommandResponse response;
  auto status = client->Command(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Semantic map override test[OK]: " << response.DebugString();
  } else {
    LOG(ERROR) << "Semantic map override test[FAILED]: "
               << status.error_message();
  }
}

void TestRunEvent(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::RunEventRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  auto *run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_PURPOSE);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_STRING);
  run_event->set_string_value("weekly-release-testing");
  run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_DRIVER);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_STRING);
  run_event->set_string_value("Other");
  run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_ENGINEER);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_STRING);
  run_event->set_string_value("Other");
  run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_LOCATION);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_STRING);
  run_event->set_string_value("cn.wuhan_dongfeng");

  run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_QCOMAND_ENGAGE);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_NONE);

  qcraft::lite::RunEventResponse response;
  auto status = client->RunEvent(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Run Event Test[OK]: " << response.DebugString();

  } else {
    LOG(ERROR) << "Run Event Test[FAILED]: " << status.error_message();
  }
}
void SetProductSetting(qcraft::ProductSettingProto *setting) {
  using qcraft::SceneSettingProto;
  // turn left
  auto *scene_setting = setting->mutable_scene_settings()->add_scene_settings();
  scene_setting->set_scene(SceneSettingProto::SCENE_TYPE_LEFT_TURN);
  scene_setting->mutable_audio_setting()->set_enable(true);
  scene_setting->mutable_audio_setting()->set_is_play_once(false);
  scene_setting->mutable_audio_setting()->set_audio_file("turn_left_test.wav");
  scene_setting->mutable_text_display_setting()->set_enable(true);

  // v2x bus first
  scene_setting = setting->mutable_scene_settings()->add_scene_settings();
  scene_setting->set_scene(SceneSettingProto::SCENE_TYPE_BUS_FIRST);
  scene_setting->mutable_audio_setting()->set_enable(true);
  scene_setting->mutable_audio_setting()->set_is_play_once(true);
  scene_setting->mutable_audio_setting()->set_audio_file("bus_first_test.wav");
  scene_setting->mutable_text_display_setting()->set_enable(true);

  // sation setting
  setting->mutable_station_setting()->set_audio_enable(true);
}
void TestSceneDynamicCofigure(
    const std::unique_ptr<qcraft::lite::PortalServiceClient> &client) {
  qcraft::lite::RunEventRequest request;
  request.set_timestamp(absl::ToUnixMicros(absl::Now()));
  // configure
  auto *run_event = request.add_run_events();
  run_event->set_key(qcraft::QRunEvent::KEY_QCOMAND_PRODUCT_SETTING);
  run_event->set_value_type(qcraft::QRunEvent::TYPE_MSG);
  SetProductSetting(run_event->mutable_product_setting_proto());

  qcraft::lite::RunEventResponse response;
  auto status = client->RunEvent(request, &response);
  if (status.ok()) {
    LOG(INFO) << "Run Event Test[OK]: " << response.DebugString();

  } else {
    LOG(ERROR) << "Run Event Test[FAILED]: " << status.error_message();
  }
}

int main(int argc, char *argv[]) {
  qcraft::InitQCraft(&argc, &argv);
  const auto server_addr =
      absl::StrCat(FLAGS_server_addr_ip, ":", FLAGS_server_addr_port);
  auto client =
      std::make_unique<qcraft::lite::PortalServiceClient>(server_addr);

  TestRemoteAssit(client);

  TestDriveAction(client);

  TestDriveRoute(client);

  TestSemanticMapOverride(client);

  TestInfo(client);

  TestRunEvent(client);

  TestState(client);

  TestSceneDynamicCofigure(client);

  return 0;
}
