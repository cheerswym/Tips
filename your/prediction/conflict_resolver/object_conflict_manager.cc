#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"

#include <string>
#include <vector>

#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::prediction {

namespace {

bool IsObjectTypeNeedSizeFilter(ObjectType type) {
  // Object type needs to do size check.
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
      return false;
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return true;
  }
}

bool IsFilteredBySize(const ObjectProto& object_proto, double min_width,
                      double min_length, double max_ratio) {
  if (IsObjectTypeNeedSizeFilter(object_proto.type())) {
    const auto box = Box2d(object_proto.bounding_box());
    return (box.width() < min_width || box.length() < min_length ||
            box.length() / box.width() > max_ratio);
  }
  return false;
}

bool IgnoredStationaryObjectType(const ObjectType& type) {
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
    // PT_STATIONARY but MOVABLE would be weird.
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
      return false;
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return true;
  }
}
}  // namespace

absl::StatusOr<const ObjectProto*>
ObjectConflictManager::GetObjectProtoByTrajId(std::string traj_id) const {
  const auto* it = FindOrNull(*result_, GetObjectIdFromTrajectoryId(traj_id));
  if (it == nullptr) {
    return absl::InternalError(
        "Cannot find object from prediction raw results! Should not come "
        "here.");
  }
  return &it->perception_object;
}

absl::StatusOr<const PredictedTrajectory* const>
ObjectConflictManager::GetPredictedTrajectoryByTrajId(
    std::string traj_id) const {
  const auto* it = FindOrNull(trajectory_map_, traj_id);
  if (it == nullptr) {
    return absl::InternalError(
        "Cannot find trajectory from trajectory id. Should not come here!");
  }
  return *it;
}

std::vector<std::string> ObjectConflictManager::FindStationaryObjects(
    std::string ego_object_id) const {
  std::vector<std::string> stationary_traj_ids;
  for (const auto& result : *result_) {
    if (result.first == ego_object_id) {
      // ego object, skip.
      continue;
    }
    // TODO(changqing): Maybe need other filters.
    // TODO(changqing): Replace this logic when relation analyzer is in use.
    const auto& object_prediction_result = result.second;
    const auto& object_id = object_prediction_result.id;
    const auto& trajectories = object_prediction_result.trajectories;
    const auto& object_proto = object_prediction_result.perception_object;
    for (const auto& trajectory : trajectories) {
      if (trajectory.type() == PT_STATIONARY &&
          !IgnoredStationaryObjectType(object_proto.type()) &&
          !IsFilteredBySize(object_proto, general_config_->min_width(),
                            general_config_->min_length(),
                            general_config_->max_aspect_ratio())) {
        stationary_traj_ids.push_back(
            MakeTrajectoryId(object_id, trajectory.index()));
      }
    }
  }
  return stationary_traj_ids;
}

absl::StatusOr<std::map<ObjectIDType, ObjectPredictionResult>>
ObjectConflictManager::ModifiedPredictions() const {
  auto mutable_result = *result_;  // Copied.
  for (const auto& [traj_id, modified_trajectory] : modified_trajectory_map_) {
    const auto object_id = GetObjectIdFromTrajectoryId(traj_id);
    auto* mutable_object_result = FindOrNull(mutable_result, object_id);
    if (mutable_object_result == nullptr) {
      // Continue adding other modified trajectories.
      continue;
    }
    auto& mutable_trajectories = mutable_object_result->trajectories;
    for (auto& origin_trajectory : mutable_trajectories) {
      if (origin_trajectory.index() == modified_trajectory.index()) {
        // Trajecoties vector's idxes not necessarily correspond to index() of
        // PredictedTrajectory. Cannot use vector idx to access directly.
        origin_trajectory = modified_trajectory;
        break;
      }
    }
  }
  return mutable_result;
}
}  // namespace qcraft::prediction
