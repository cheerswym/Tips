#include "onboard/node/node_util.h"

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "onboard/global/car_common.h"

DEFINE_string(node_name, "",
              "The name of node, such as main_node、sensor_node");
DEFINE_string(name_space, "", "The name of space, such as omc, xavier");

namespace qcraft {

namespace {
constexpr auto kInteractivePlaybackWithNodeLite2 =
    "onboard/node/test/interactive_playback_with_node_lite2.pb.txt";
}

bool IsThisNode(const NodeConfig &node_config) {
  return node_config.node_name() == FLAGS_node_name &&
         node_config.name_space() == FLAGS_name_space;
}

bool IsThisNode(const NodeConfig &node_config, const std::string &node_name,
                const std::string &name_space) {
  return node_config.node_name() == node_name &&
         node_config.name_space() == name_space;
}

bool IsThisNodeNameSpace(const NodeConfig &node_config) {
  return node_config.name_space() == FLAGS_name_space;
}

bool IsThisNodeMessage(const std::string &channel) {
  return (channel == "system_command_proto" || channel == "system_state_proto");
}

bool IsThisNodeInterface(const NodeConfig &node_config,
                         const std::string &node_ip) {
  return node_config.node_ip() == node_ip;
}

bool IsConnectThisNodeDirectly(const NodeConfig &node_config,
                               const std::string &node_ip) {
  const auto &this_node_ip = node_config.node_ip();
  return this_node_ip.substr(0, this_node_ip.rfind('.')) ==
         node_ip.substr(0, node_ip.rfind('.'));
}

std::string GetFullNodeName(const std::string &node_name,
                            const std::string &name_space) {
  return absl::StrCat("/", name_space, "/", node_name);
}

std::string GetFullNodeName(const NodeConfig &node_config) {
  return GetFullNodeName(node_config.node_name(), node_config.name_space());
}

std::string GetFullNodeName() {
  return GetFullNodeName(FLAGS_node_name, FLAGS_name_space);
}

std::unordered_set<CameraId> GetCameraIdsInThisNode(
    const VehicleParamApi &vehicle_params) {
  std::unordered_set<CameraId> camera_ids;
  if (vehicle_params.has_obc_config(0)) {
    for (int64_t i = 0; i < vehicle_params.obc_size(); ++i) {
      const auto obc_config = vehicle_params.obc(i).installation().obc_config();
      // Only one elements
      const auto &node_name_to_camera_id = obc_config.node_name_to_camera_id(0);
      if (node_name_to_camera_id.node_name() == FLAGS_node_name) {
        for (const auto &camera_id : node_name_to_camera_id.camera_id()) {
          camera_ids.insert(static_cast<CameraId>(camera_id));
        }
        break;
      }
    }
  } else {
    auto cameras = vehicle_params.camera_params();
    for (size_t index = 0; index < cameras.size(); index++) {
      camera_ids.insert(cameras[index].installation().camera_id());
    }
  }
  return camera_ids;
}

CameraCpldTriggerSourceType GetCameraCpldTriggerSourceInThisNode(
    const VehicleParamApi &vehicle_params) {
  for (size_t index = 0; index < vehicle_params.obc_size(); index++) {
    const auto obc = vehicle_params.obc(index);
    if (obc.installation().nodes_run_config().nodes(0).node_name() !=
        FLAGS_node_name) {
      continue;
    }
    return obc.installation().obc_config().camera_cpld_trigger_source();
  }

  return CPLD_TRIGGER_SOURCE_INTERNAL_GPS;
}

NodesRunConfig LoadNodesConfigFromCurrentNodeFile(
    const ParamManager &param_manager) {
  if (IsOnboardMode()) {
    RunParamsProtoV2 run_params;
    param_manager.GetRunParams(&run_params);
    const auto vehicle_param = run_params.vehicle_params();
    return vehicle_param.nodes_run_config();
  } else {
    NodesRunConfig nodes_run_config;
    QCHECK(file_util::TextFileToProto(kInteractivePlaybackWithNodeLite2,
                                      &nodes_run_config));
    return nodes_run_config;
  }
}

}  // namespace qcraft
