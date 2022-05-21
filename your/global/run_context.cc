#include "onboard/global/run_context.h"

#include <memory>
#include <ostream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(enable_context_test, false, "Only be used in unit test.");

namespace qcraft {

std::atomic<VehicleInstallationProto::VehiclePlan> run_context_ =
    VehicleInstallationProto_VehiclePlan_VP_DBQ_V3;

VehicleInstallationProto::VehiclePlan QCraftRunContext(
    const std::string& run_context) {
  if (run_context == "dbqv2") {
    return VehicleInstallationProto_VehiclePlan_VP_DBQ_V2;
  }
  if (run_context == "dbqv3") {
    return VehicleInstallationProto_VehiclePlan_VP_DBQ_V3;
  }
  if (run_context == "dbqv4") {
    return VehicleInstallationProto_VehiclePlan_VP_DBQ_V4;
  }
  if (run_context == "pbqv1") {
    return VehicleInstallationProto_VehiclePlan_VP_PBQ_V1;
  }
  LOG(FATAL) << "Unknown q_run_context: " << run_context;
}

SensorScenarioConfig QCraftSensorScenarioConfig() {
  static SensorScenarioConfig sensor_scenario_config;
  static std::once_flag flag;

  std::call_once(flag, [] {
    const auto config = LoadSensorScenarioConfigFromCurrentConfigFile();
    QCHECK(config.ok()) << config.status().message();
    sensor_scenario_config = config.value();
  });
  return sensor_scenario_config;
}

VehicleInstallationProto::VehiclePlan QCraftRunContext() {
  if (FLAGS_enable_context_test) {
    return (QCraftRunContext(FLAGS_q_run_context));
  }
  return run_context_.load();
}

static VehicleInstallationProto::VehiclePlan QCraftRunContext(
    const RunParamsProtoV2& run_params) {
  const auto vehicle_params = run_params.vehicle_params().vehicle_params();
  const auto& installation = vehicle_params.installation();
  if (!installation.has_vehicle_plan()) {
    return FLAGS_q_run_conext.empty() ? QCraftRunContext(FLAGS_q_run_context)
                                      : QCraftRunContext(FLAGS_q_run_conext);
  }

  const auto& vehicle_plan = installation.vehicle_plan();

  if (vehicle_plan == VehicleInstallationProto_VehiclePlan_VP_DBQ_V2) {
    return vehicle_plan;
  }
  if (vehicle_plan == VehicleInstallationProto_VehiclePlan_VP_DBQ_V3) {
    return vehicle_plan;
  }
  if (vehicle_plan == VehicleInstallationProto_VehiclePlan_VP_DBQ_V4) {
    if (FLAGS_q_run_conext == "pbqv1" || FLAGS_q_run_context == "pbqv1") {
      return QCraftRunContext(FLAGS_q_run_context);
    }
    return vehicle_plan;
  }
  if (vehicle_plan == VehicleInstallationProto_VehiclePlan_VP_PBQ_V1) {
    return vehicle_plan;
  }

  if (vehicle_plan == VehicleInstallationProto_VehiclePlan_VP_DPC) {
    if (FLAGS_q_run_conext == "dbqv3" || FLAGS_q_run_context == "dbqv3") {
      return QCraftRunContext(FLAGS_q_run_context);
    }
    if (FLAGS_q_run_conext == "pbqv1" || FLAGS_q_run_context == "pbqv1") {
      return QCraftRunContext(FLAGS_q_run_context);
    }
    LOG(FATAL) << "Unsupported q_run_context: " << FLAGS_q_run_context;
  }
  LOG(FATAL) << "Error vehicle_plan: " << vehicle_plan;
}

void SetQCraftRunContext(const RunParamsProtoV2& run_params) {
  run_context_ = QCraftRunContext(run_params);
}

bool IsDBQConext(const VehicleInstallationProto::VehiclePlan& run_context) {
  switch (run_context) {
    case VehicleInstallationProto_VehiclePlan_VP_DBQ_V2:
    case VehicleInstallationProto_VehiclePlan_VP_DBQ_V3:
    case VehicleInstallationProto_VehiclePlan_VP_DBQ_V4:
      return true;
    case VehicleInstallationProto_VehiclePlan_VP_PBQ_V1:
    case VehicleInstallationProto_VehiclePlan_VP_DPC:
    case VehicleInstallationProto_VehiclePlan_VP_DBQ:
    case VehicleInstallationProto_VehiclePlan_VP_PBQ:
      return false;
  }
}

bool IsDBQConext() {
  const auto run_context = QCraftRunContext();
  return IsDBQConext(run_context);
}

bool IsDBQv4(const VehicleInstallationProto::VehiclePlan& run_context) {
  return (QCraftRunContext() == VehicleInstallationProto_VehiclePlan_VP_DBQ_V4);
}

bool IsDBQv4() {
  const auto run_context = QCraftRunContext();
  return IsDBQv4(run_context);
}

}  // namespace qcraft
