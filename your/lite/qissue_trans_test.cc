#include "onboard/lite/qissue_trans.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/global/clock.h"
#include "onboard/lite/lite_client.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/lite_transport.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport.h"
#include "onboard/lite/transport/message/inner_message_hub.h"

namespace qcraft {

using LiteClientPtr = std::unique_ptr<LiteClient>;
using ExecutionIssueProtoPtr = std::shared_ptr<ExecutionIssueProto>;
using ExecutionIssueProtoPtrVec = std::vector<ExecutionIssueProtoPtr>;

const LiteClientPtr CreateClient(const LiteModuleName& module,
                                 const std::string& domain = "") {
  auto transport = std::make_unique<LiteTransport>(GlobalInnerMessageHub());
  auto client = std::make_unique<LiteClient>(module, std::move(transport),
                                             nullptr, domain);
  return client;
}
void CheckProtoMsg(const QIssueProto& issue, QIssueSeverity severity,
                   QIssueType type, QIssueSubType sub_type,
                   const std::string& message,
                   const std::string& args_message = "") {
  EXPECT_EQ(issue.severity(), severity);
  EXPECT_EQ(issue.type(), type);
  EXPECT_EQ(issue.sub_type(), sub_type);
  EXPECT_EQ(issue.message(), message);
  EXPECT_EQ(issue.args_message(), args_message);
}

// create client
static auto client = CreateClient(PLANNER_MODULE);

TEST(QIssueTransTest, Publish) {
  // reset hub
  GlobalInnerMessageHub()->Reset();

  // add subscriber
  ExecutionIssueProtoPtrVec protos;
  auto callback = [&protos](std::shared_ptr<const ExecutionIssueProto> msg) {
    const auto& header = msg->header();
    const auto& issue = msg->issue();
    LOG(INFO) << header.domain() << "," << header.channel() << ","
              << header.module_id() << "," << issue.severity() << ","
              << issue.type() << "," << issue.sub_type() << ","
              << issue.message() << "," << issue.args_message();
    protos.emplace_back(std::make_shared<ExecutionIssueProto>(*msg));
  };
  client->Subscribe<ExecutionIssueProto>(callback);

  // create qissue client
  QIssueTrans::Instance()->CreateClient(GlobalInnerMessageHub());

  // publish qissue msg
  QIssueTrans::Instance()->Publish(QIS_ERROR, QIT_CLOCK_SYNC,
                                   QIST_LIDAR_STATE_FRAME_DROP, "lidar",
                                   "frame drop");
  // receive qissue msg
  client->SleepUntil(absl::Milliseconds(10));

  // test qissue msg
  EXPECT_EQ(protos.size(), 1UL);
  CheckProtoMsg(protos[0]->issue(), QIS_ERROR, QIT_CLOCK_SYNC,
                QIST_LIDAR_STATE_FRAME_DROP, "lidar", "frame drop");
}

TEST(QIssueTransTest, MacroApi) {
  // reset hub
  GlobalInnerMessageHub()->Reset();

  // add subscriber
  ExecutionIssueProtoPtrVec protos;
  auto callback = [&protos](std::shared_ptr<const ExecutionIssueProto> msg) {
    const auto& header = msg->header();
    const auto& issue = msg->issue();
    LOG(INFO) << header.domain() << "," << header.channel() << ","
              << header.module_id() << "," << issue.severity() << ","
              << issue.type() << "," << issue.sub_type() << ","
              << issue.message() << "," << issue.args_message();
    protos.emplace_back(std::make_shared<ExecutionIssueProto>(*msg));
  };
  client->Subscribe<ExecutionIssueProto>(callback);

  // create qissue client
  QIssueTrans::Instance()->CreateClient(GlobalInnerMessageHub());

  // test macro publish
  QISSUEX(QIS_DEBUG, QIT_PIPELINE, QIST_PLANNER_PROTO_TRAJECTORY_EMPTY,
          "planner");
  // test macro with args publish
  QISSUEX_WITH_ARGS(QIS_ERROR, QIT_CRASH, QIST_PLANNER_PROCESS_TIMEOUT,
                    "planner", "process timeout");

  // receive qissue msg
  client->SleepUntil(absl::Milliseconds(10));

  // test msg
  EXPECT_EQ(protos.size(), 2UL);
  CheckProtoMsg(protos[0]->issue(), QIS_DEBUG, QIT_PIPELINE,
                QIST_PLANNER_PROTO_TRAJECTORY_EMPTY, "planner");
  CheckProtoMsg(protos[1]->issue(), QIS_ERROR, QIT_CRASH,
                QIST_PLANNER_PROCESS_TIMEOUT, "planner", "process timeout");
}

}  // namespace qcraft
