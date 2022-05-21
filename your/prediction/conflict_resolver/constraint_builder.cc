#include "onboard/prediction/conflict_resolver/constraint_builder.h"

namespace qcraft {
namespace prediction {
absl::flat_hash_set<mapping::ElementId> GetRedTrafficLightLanes(
    const SemanticMapManager* smm, const TrafficLightManager* tl_mgr) {
  absl::flat_hash_set<mapping::ElementId> lanes;
  // Based on semantic map, iterate through all lane, find traffic light
  // controlled lanes, draw stoplines (halfplane) on lane. Later use the map to
  // mapping stopline to ObjectStMap.
  const auto& tl_map = tl_mgr->GetInferedTlStateMap();
  for (const auto& pair : tl_map) {
    const auto& tl_state_proto = pair.second;
    if (tl_state_proto.has_color() && tl_state_proto.color() == TL_RED) {
      const auto* ptr_tl_proto = smm->FindTrafficLightByIdOrNull(pair.first);
      if (ptr_tl_proto == nullptr) {
        QLOG_EVERY_N_SEC(ERROR, 5.0)
            << "Cannot found, may load faild with map patch, "
               "traffic_light_id: "
            << pair.first;
        QEVENT_EVERY_N_SECONDS(
            "changqing", "prediction_conflict_resolver_tl_id_from_smm_fail", 5,
            [&](QEvent* qevent) {
              qevent->AddField("traffic_light_id", pair.first);
            });
        continue;
      }
      const auto& tl_proto = *ptr_tl_proto;
      for (const auto& lane_id : tl_proto.controlling_lane_ids()) {
        VLOG(3) << absl::StrFormat(
            "Traffic light %d is RED !!  It controlls lane %d", pair.first,
            lane_id);
        lanes.insert(lane_id);
      }
    }
  }
  return lanes;
}
}  // namespace prediction
}  // namespace qcraft
