#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_MANAGER_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_MANAGER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"

namespace qcraft::prediction {
// The manager consideres all given object prediction and prediction context,
// build decision constraints (traffic light, stopline, etc.) and which objects
// to consider in conflict resolution for each prediction object.

// Conflict resolution level:
// 0. Consider stationary object and traffic light/stopline infos.
// 1. Add moving objects on current path.

enum class ConflictResolutionLevel {
  CRL_STATIONARY,
  CRL_DYNAMIC,
  CRL_INTERACTIVE,
};
class ObjectConflictManager {
 public:
  explicit ObjectConflictManager(
      const PredictionContext* context,
      const std::map<std::string, ObjectPredictionResult>* scheduler_result,
      const ConflictResolutionConfigProto::ConflictResolverConfig*
          general_config,
      ConflictResolutionLevel resolution_level =
          ConflictResolutionLevel::CRL_STATIONARY)
      : context_(context),
        result_(scheduler_result),
        general_config_(general_config),
        resolution_level_(resolution_level) {
    object_trajs_.reserve(scheduler_result->size());
    for (const auto& pair : *scheduler_result) {
      const auto& prediction_result = pair.second;
      for (const auto& traj : prediction_result.trajectories) {
        if (traj.type() == PT_VOID) {
          continue;
        }
        const auto traj_id = MakeTrajectoryId(pair.first, traj.index());
        auto [it, success] =
            trajectory_map_.emplace(std::make_pair(traj_id, &traj));
        if (!success) {
          continue;  // Insertion the traj_id, PredictedTrajectory map not
                     // successful, now ignore it. But it is weird.
        } else {
          object_trajs_.push_back(traj_id);
          if (traj.type() != PT_STATIONARY) {
            moving_object_trajs_.push_back(traj_id);
          } else {
            stationary_object_trajs_.push_back(traj_id);
          }
        }
      }
    }
    QCHECK_EQ(object_trajs_.size(), trajectory_map_.size());
  }

  std::vector<std::string> FindStationaryObjects(
      std::string ego_object_id) const;

  absl::StatusOr<std::map<ObjectIDType, ObjectPredictionResult>>
  ModifiedPredictions() const;

  void AddModifiedTrajectory(const std::string& traj_id,
                             const PredictedTrajectory& predicted_trajectory) {
    const auto result =
        modified_trajectory_map_.try_emplace(traj_id, predicted_trajectory);
    if (result.second == true) {
      VLOG(5) << "Prediction conflict resolver add modified trajectory for "
              << traj_id << " success.";
    }
  }

  // TODO(changqing): add query functions to get object prediction result and
  // predicted trajectories.

  // Query ObjectProto by TrajId.
  absl::StatusOr<const ObjectProto*> GetObjectProtoByTrajId(
      std::string traj_id) const;
  // Query PredictedTrajectory* by TrajId.
  absl::StatusOr<const PredictedTrajectory* const>
  GetPredictedTrajectoryByTrajId(std::string traj_id) const;

  absl::Span<const std::string> object_traj_ids() const {
    return absl::MakeSpan(object_trajs_);
  }

  const SemanticMapManager* semantic_map_manager() const {
    return context_->semantic_map_manager();
  }

  inline int object_trajs_size() const { return object_trajs_.size(); }
  std::string DebugString() const;

  // A utility function to generate trajectory's id.
  static std::string MakeTrajectoryId(std::string obj_id, int traj_index) {
    return absl::StrFormat("%s-idx%d", obj_id, traj_index);
  }

  // A utility function to extract object's id from trajectory id.
  static std::string GetObjectIdFromTrajectoryId(std::string traj_id) {
    const std::vector<std::string> tokens = absl::StrSplit(traj_id, "-");
    QCHECK(!tokens.empty());
    return tokens.front();
  }

 private:
  const PredictionContext* context_;
  const std::map<std::string, ObjectPredictionResult>* result_;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config_;

  std::map<std::string, const PredictedTrajectory*> trajectory_map_;
  std::map<std::string, PredictedTrajectory> modified_trajectory_map_;
  std::vector<std::string> object_trajs_;  // Object id + traj idx.
  std::vector<std::string>
      moving_object_trajs_;  // Moving trajs possibly need to be resolved.
  std::vector<std::string> stationary_object_trajs_;
  [[maybe_unused]] ConflictResolutionLevel resolution_level_;
};

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_OBJECT_CONFLICT_MANAGER_H_
