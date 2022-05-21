#ifndef ONBOARD_LITE_MODIFIER_SYNTHESIZED_TRAFFIC_FLOW_MODULE_H_
#define ONBOARD_LITE_MODIFIER_SYNTHESIZED_TRAFFIC_FLOW_MODULE_H_

#include <memory>

#include "onboard/lite/lite_module.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

// A module that publishes synthesized traffic flow in simulation, such as
// traffic light controls and virtual agents.
class SynthesizedTrafficFlowModule : public LiteModule {
 public:
  explicit SynthesizedTrafficFlowModule(LiteClientBase* lite_client);
  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  void UpdateLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);
  void ProcessPose(std::shared_ptr<const PoseProto> pose);
  void MakeAllIncomingTLsGreen(std::shared_ptr<const PoseProto> pose);

  CoordinateConverter coordinate_converter_;

  SemanticMapManager semantic_map_manager_;
};

REGISTER_LITE_MODULE(SynthesizedTrafficFlowModule);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_SYNTHESIZED_TRAFFIC_FLOW_MODULE_H_
