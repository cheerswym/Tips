#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_RESOLVER_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_RESOLVER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_input.h"
#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"
#include "onboard/prediction/conflict_resolver/object_st_map.h"
#include "onboard/prediction/conflict_resolver/object_svt_graph.h"
#include "onboard/prediction/conflict_resolver/svt_cost_provider.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::prediction {
absl::StatusOr<PredictedTrajectory> ResolveObjectConflict(
    const ObjectConflictResolverInput& input, const std::string& traj_id,
    ThreadPool* thread_pool, ConflictResolverDebugProto* debug_proto);
}  // namespace qcraft::prediction
#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_RESOLVER_H_
