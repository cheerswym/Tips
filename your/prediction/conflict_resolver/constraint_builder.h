#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONSTRAINT_BUILDER_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONSTRAINT_BUILDER_H_

#include "absl/container/flat_hash_set.h"
#include "onboard/eval/qevent.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

namespace prediction {

absl::flat_hash_set<mapping::ElementId> GetRedTrafficLightLanes(
    const SemanticMapManager* smm, const TrafficLightManager* tl_mgr);
}
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONSTRAINT_BUILDER_H_
