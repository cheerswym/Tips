#include "onboard/node/node_state_module.h"

#include <vector>

#include "absl/strings/str_cat.h"
#include "boost/process.hpp"
#include "onboard/global/clock.h"
#include "onboard/global/system_info.h"
#include "onboard/global/trace.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/node/node_util.h"
#include "onboard/params/param_manager.h"
#include "onboard/proto/vehicle.pb.h"

DEFINE_int32(max_rtt_between_nodes, 150 * 1000, "max rtt between nodes.");
DEFINE_int32(max_drift_between_nodes, 150 * 1000, "max drift between nodes.");
DEFINE_int32(max_interval_between_nodes, 400 * 1000,
             "max disconnect interval between nodes.");

namespace qcraft {

namespace {
constexpr auto kNodeStateProtoTag = 115;
#ifdef __x86_64__
constexpr auto kGPUMaxTemperature = 90;
constexpr auto kGPUMaxAcceptTemperature = 85;
constexpr auto kGPUMinUtilization = 30;
constexpr auto kGPUMaxAcceptClockRatio = 0.4;
#endif
}  // namespace

namespace bp = boost::process;

NodeStateModule::NodeStateModule(LiteClientBase *lite_client)
    : LiteModule(lite_client) {}

NodeStateModule::~NodeStateModule() {
  if (time_sync_runner_) {
    time_sync_runner_->Stop();
  }

  if (system_info_runner_) {
    system_info_runner_->Stop();
  }
}

void NodeStateModule::OnInit() {
  const auto nodes_run_config =
      LoadNodesConfigFromCurrentNodeFile(param_manager());
  for (const auto &node_config : nodes_run_config.nodes()) {
    if (!IsThisNodeNameSpace(node_config)) {
      node_time_points_[GetFullNodeName(node_config)] =
          std::chrono::steady_clock().now();
    }
  }
}

void NodeStateModule::OnSubscribeChannels() {
  if (!IsOnboardMode()) {
    return;
  }

  Subscribe(&NodeStateModule::HandleNodeState, this);
#ifdef __x86_64__
  Subscribe(&NodeStateModule::HandleSystemInfo, this);
#endif
}
void NodeStateModule::OnSetUpTimers() {
  if (!IsOnboardMode()) {
    return;
  }

  AddTimerOrDie(
      "publish_system_info",
      [this]() {
        system_info_runner_ =
            std::make_unique<PeriodicRunner>(absl::Seconds(1));
        system_info_runner_->Start([this]() {
          SystemInfoProto proto =
              qcraft::SystemInfo::Instance()->GetSharedSystemInfoProto();
          QLOG_IF_NOT_OK(WARNING, Publish(proto));
        });
      },
      absl::Seconds(1), true /*one_shot*/);

  AddTimerOrDie(
      "publish_node_state",
      [this]() {
        LiteMsgWrapper lite_msg;
        lite_msg.set_tag_number(kNodeStateProtoTag);
        auto *node_state_proto = lite_msg.mutable_node_state_proto();
        node_state_proto->set_t1(absl::ToUnixMicros(Clock::Now()));
        node_state_proto->mutable_header()->set_timestamp(
            absl::ToUnixMicros(Clock::Now()));
        QLOG_IF_NOT_OK(WARNING, ForwardLiteMsg(&lite_msg));
      },
      absl::Milliseconds(100), false /*one_shot*/);

  AddTimerOrDie(
      "check_node_heartbeat",
      [this]() {
        for (const auto &node_time_point : node_time_points_) {
          const auto interval =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock().now() - node_time_point.second)
                  .count();
          if (interval > FLAGS_max_interval_between_nodes) {
            const auto nodes =
                absl::StrCat(GetFullNodeName(), " -> ", node_time_point.first);
            QISSUEX_WITH_ARGS(
                QIssueSeverity::QIS_ERROR, QIssueType::QIT_NETWORK,
                QIssueSubType::QIST_NODE_STATE_DISCONNECT,
                absl::StrCat("Check node state: DISCONNECT(", nodes, ")"),
                absl::StrCat("interval: ", interval / 1000, " ms"));
          }
        }
      },
      absl::Seconds(1), absl::Milliseconds(400), false /*one_shot*/);
}

void NodeStateModule::HandleNodeState(
    std::shared_ptr<const NodeStateProto> node_state_proto) {
  QCHECK(node_state_proto->has_t1());
  if (!node_state_proto->header().has_full_node_name()) {
    return;
  }

  node_time_points_[node_state_proto->header().full_node_name()] =
      std::chrono::steady_clock().now();
  if (node_state_proto->has_t2()) {
    const auto t3 = Clock::Now();
    const auto rtt = t3 - absl::FromUnixMicros(node_state_proto->t1());
    const auto drift = absl::FromUnixMicros(node_state_proto->t1()) + rtt / 2 -
                       absl::FromUnixMicros(node_state_proto->t2());
    const auto nodes = absl::StrCat(
        GetFullNodeName(), " -> ", node_state_proto->header().full_node_name());
    if (std::abs(absl::ToInt64Microseconds(drift)) >
        FLAGS_max_drift_between_nodes) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_CLOCK_SYNC,
          QIssueSubType::QIST_NODE_STATE_CLOCK_SYNC,
          absl::StrCat("Check node state: ASYNC(", nodes, ")"),
          absl::StrCat("drift: ", absl::ToInt64Milliseconds(drift), " ms"));
      return;
    }

    if (std::abs(absl::ToInt64Microseconds(rtt)) >
        FLAGS_max_rtt_between_nodes) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_NETWORK,
          QIssueSubType::QIST_NODE_STATE_SLOW,
          absl::StrCat("Check node state: SLOW(", nodes, ")"),
          absl::StrCat("rtt: ", absl::ToInt64Milliseconds(rtt), " ms"));
      return;
    }
  } else {
    LiteMsgWrapper lite_msg;
    lite_msg.set_tag_number(kNodeStateProtoTag);
    auto *echo_node_state_proto = lite_msg.mutable_node_state_proto();
    echo_node_state_proto->set_t1(node_state_proto->t1());
    echo_node_state_proto->set_t2(absl::ToUnixMicros(Clock::Now()));
    echo_node_state_proto->mutable_header()->set_timestamp(
        absl::ToUnixMicros(Clock::Now()));
    QLOG_IF_NOT_OK(WARNING, ForwardLiteMsg(&lite_msg));
  }
}

#ifdef __x86_64__
void NodeStateModule::HandleSystemInfo(
    std::shared_ptr<const SystemInfoProto> system_info_proto) {
  for (const auto &gpu_device_info :
       system_info_proto->gpu_info().gpu_device_info()) {
    if (gpu_device_info.temperature() > kGPUMaxTemperature) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_DEVICE,
          QIssueSubType::QIST_GPU_WITH_OVERHEAT,
          absl::StrCat("Check GPU states: OVERHEAT(", gpu_device_info.name(),
                       ")"),
          absl::StrCat("temperature: ", gpu_device_info.temperature(),
                       ", utilization: ", gpu_device_info.gpu_utilization(),
                       ", current_clock: ", gpu_device_info.current_clock()));
    } else if ((gpu_device_info.temperature() > kGPUMaxAcceptTemperature) &&
               (gpu_device_info.gpu_utilization() > kGPUMinUtilization) &&
               (gpu_device_info.current_clock() <
                gpu_device_info.max_clock() * kGPUMaxAcceptClockRatio)) {
      QISSUEX_WITH_ARGS(
          QIssueSeverity::QIS_ERROR, QIssueType::QIT_DEVICE,
          QIssueSubType::QIST_GPU_WITH_UNDERCLOCKED,
          absl::StrCat("Check GPU states: UNDERCLOCKED(",
                       gpu_device_info.name(), ")"),
          absl::StrCat("temperature: ", gpu_device_info.temperature(),
                       ", utilization: ", gpu_device_info.gpu_utilization(),
                       ", current_clock: ", gpu_device_info.current_clock()));
    }
  }
}
#endif

}  // namespace qcraft
