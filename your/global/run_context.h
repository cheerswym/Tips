#ifndef ONBOARD_GLOBAL_RUN_CONTEXT_H_
#define ONBOARD_GLOBAL_RUN_CONTEXT_H_

#include <string>

#include "gflags/gflags.h"
#include "onboard/base/base_flags.h"
#include "onboard/lite/lite2_flags.h"
#include "onboard/lite/sensor_scenario_config_helper.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"

DECLARE_bool(enable_context_test);

namespace qcraft {

SensorScenarioConfig QCraftSensorScenarioConfig();

VehicleInstallationProto::VehiclePlan QCraftRunContext();

VehicleInstallationProto::VehiclePlan QCraftRunContext(
    const std::string& run_context);

void SetQCraftRunContext(const RunParamsProtoV2& run_params);

bool IsDBQConext(const VehicleInstallationProto::VehiclePlan& run_context);

bool IsDBQConext();

bool IsDBQv4(const VehicleInstallationProto::VehiclePlan& run_context);

bool IsDBQv4();

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_RUN_CONTEXT_H_
