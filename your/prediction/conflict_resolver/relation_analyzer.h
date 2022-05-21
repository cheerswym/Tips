#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_RELATION_ANALYZER_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_RELATION_ANALYZER_H_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
enum class AgentRelationType {
  ART_PASS = 1,   // Av passing agent.
  ART_YIELD = 2,  // Av yield to agent.
  ART_NONE = 3,   // No relation.
};

// Object conflict manager will build this input.
struct AgentRelationAnalyzerInput {
  const ObjectProto* object_proto = nullptr;
  const PredictedTrajectory* trajectory = nullptr;
  const SegmentMatcherKdtree* segments = nullptr;
};

AgentRelationType AnalyzeTrajectoryRelation(
    const AgentRelationAnalyzerInput& agent_1,
    const AgentRelationAnalyzerInput& agent_2);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_RELATION_ANALYZER_H_
