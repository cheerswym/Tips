#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_INPUT_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_INPUT_H_

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::prediction {
// Define input structures for each submodules of conflict resolver.

struct ObjectConflictResolverInput {
  const ObjectConflictManager* obj_con_mgr = nullptr;
  const PredictedTrajectory* predicted_trajectory = nullptr;
  const planner::DiscretizedPath* ego_path = nullptr;
  const planner::SpeedVector* ref_speed = nullptr;
  const std::vector<mapping::ElementId>* possible_lane_path = nullptr;
  const absl::flat_hash_set<mapping::ElementId>* tl_red_lanes = nullptr;
  const ConflictResolverParams* params = nullptr;
  const PredictionContext* context = nullptr;
};
}  // namespace qcraft::prediction
#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_INPUT_H_
