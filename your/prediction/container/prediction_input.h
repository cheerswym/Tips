#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/time/time.h"
#include "boost/circular_buffer.hpp"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_state.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::prediction {

struct PredictionInput {
  explicit PredictionInput(int history_size, int history_len, double min_dt) {
    objects_history =
        std::make_unique<ObjectsHistory>(history_size, history_len, min_dt);
    av_context = std::make_unique<AvContext>(history_size);
  }
  // Same as planner input.
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const TrafficLightStatesProto> traffic_light_states;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  absl::Time prediction_init_time;

  // From planner config
  const SemanticMapManager* semantic_map_manager;
  VehicleGeometryParamsProto veh_geom_params;

  // cross-iteration [IN & OUT]
  std::unique_ptr<ObjectsHistory> objects_history;
  std::unique_ptr<AvContext> av_context;

  // Conflict resolver config.
  const ConflictResolverParams* conflict_resolver_params = nullptr;
};

absl::Status FillPredictionState(const PredictionInput& prediction_input,
                                 PredictionState* prediction_state);

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_
