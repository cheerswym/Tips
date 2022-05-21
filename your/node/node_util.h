#ifndef ONBOARD_NODE_NODE_UTIL_H_
#define ONBOARD_NODE_NODE_UTIL_H_

#include <string>
#include <unordered_set>

#include "onboard/lite/proto/module_config.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/ads.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

bool IsThisNode(const NodeConfig &node_config);

bool IsThisNode(const NodeConfig &node_config, const std::string &node_name,
                const std::string &name_space);

bool IsThisNodeNameSpace(const NodeConfig &node_config);

bool IsThisNodeMessage(const std::string &channel);

bool IsThisNodeInterface(const NodeConfig &node_config,
                         const std::string &node_ip);

bool IsConnectThisNodeDirectly(const NodeConfig &node_config,
                               const std::string &node_ip);

std::string GetFullNodeName(const NodeConfig &node_config);

std::string GetFullNodeName(const std::string &node_name,
                            const std::string &name_space);

std::string GetFullNodeName();

std::unordered_set<CameraId> GetCameraIdsInThisNode(
    const VehicleParamApi &vehicle_params);

NodesRunConfig LoadNodesConfigFromCurrentNodeFile(
    const ParamManager &param_manager);

CameraCpldTriggerSourceType GetCameraCpldTriggerSourceInThisNode(
    const VehicleParamApi &vehicle_params);

}  // namespace qcraft

#endif  // ONBOARD_NODE_NODE_UTIL_H_
