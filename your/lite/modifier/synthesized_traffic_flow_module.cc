#include "onboard/lite/modifier/synthesized_traffic_flow_module.h"

#include <memory>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {
namespace {
constexpr double kTLMaxDist = 150.0;

}  // namespace

SynthesizedTrafficFlowModule::SynthesizedTrafficFlowModule(
    LiteClientBase* lite_client)
    : LiteModule(lite_client) {}

void SynthesizedTrafficFlowModule::OnInit() {
  semantic_map_manager_.LoadWholeMap();
}

void SynthesizedTrafficFlowModule::OnSubscribeChannels() {
  Subscribe(&SynthesizedTrafficFlowModule::ProcessPose, this);
  Subscribe(&SynthesizedTrafficFlowModule::UpdateLocalizationTransform, this);
}

void SynthesizedTrafficFlowModule::OnSetUpTimers() {}

void SynthesizedTrafficFlowModule::UpdateLocalizationTransform(
    std::shared_ptr<const LocalizationTransformProto>
        localization_transform_proto) {
  coordinate_converter_.UpdateLocalizationTransform(
      *localization_transform_proto);
}

void SynthesizedTrafficFlowModule::ProcessPose(
    std::shared_ptr<const PoseProto> pose) {
  // TODO(mike): replace with more realistic TL control signals.
  MakeAllIncomingTLsGreen(pose);
}

void SynthesizedTrafficFlowModule::MakeAllIncomingTLsGreen(
    std::shared_ptr<const PoseProto> pose) {
  // Find nearby traffic lights.
  const VehiclePose vehicle_pose(*pose);
  const auto& traffic_lights =
      semantic_map_manager_.semantic_map().traffic_lights();
  std::vector<int64_t> tl_ids;
  for (const auto& traffic_light : traffic_lights) {
    const Vec3d tl_pos = coordinate_converter_.GlobalToSmooth(
        {traffic_light.point().longitude(), traffic_light.point().latitude(),
         0.0});
    // Ignore too far TLs.
    const double dist2 = (tl_pos - vehicle_pose.coord()).squaredNorm();
    if (dist2 > Sqr(kTLMaxDist)) continue;

    // Ignore TLs that are not facing to AV.
    constexpr double kMaxTLAngleDiff = d2r(30.0);
    const Vec2d dir(vehicle_pose.x - tl_pos.x(), vehicle_pose.y - tl_pos.y());
    if (std::abs(NormalizeAngle(dir.Angle() - traffic_light.bb_heading())) >
        kMaxTLAngleDiff) {
      continue;
    }

    tl_ids.push_back(traffic_light.id());
  }
  TrafficLightStatesProto tl_states_proto;
  for (const auto& tl_id : tl_ids) {
    auto* tl_state = tl_states_proto.add_states();
    tl_state->set_traffic_light_id(tl_id);
    tl_state->set_color(TL_GREEN);
  }
  QLOG_IF_NOT_OK(WARNING, Publish(tl_states_proto));
}

}  // namespace qcraft
