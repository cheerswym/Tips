#include "onboard/prediction/container/av_context.h"

#include <utility>

#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/prediction_defs.h"
namespace qcraft {
namespace prediction {
void AvContext::Update(const PoseProto& pose,
                       const LocalizationTransformProto& loc_transform,
                       const VehicleGeometryParamsProto& veh_geom_params) {
  CoordinateConverter target(loc_transform);
  for (auto& node : *av_object_history_.mutable_buffer()) {
    node.val.TransformCoordinate(target);
  }
  av_object_history_.Push(
      pose.timestamp(), PredictionObject(planner::AvPoseProtoToObjectProto(
                                             kAvObjectId, veh_geom_params, pose,
                                             /*offroad=*/false),
                                         std::move(target)));

  av_object_history_.PopBegin(pose.timestamp() - kHistoryLen);
}

const ObjectHistory& AvContext::GetAvObjectHistory() const {
  return av_object_history_;
}
}  // namespace prediction
}  // namespace qcraft
