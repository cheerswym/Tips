#ifndef ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_
#define ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_

#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
class AvContext {
 public:
  explicit AvContext(size_t capacity) : av_object_history_(capacity) {}
  void Update(const PoseProto& pose,
              const LocalizationTransformProto& loc_transform,
              const VehicleGeometryParamsProto& veh_geom_params);
  const ObjectHistory& GetAvObjectHistory() const;

 private:
  ObjectHistory av_object_history_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_
