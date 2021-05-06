#include "onboard/planner/object/planner_object_manager_builder.h"

#include <memory>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/planner/util/prediction_util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

absl::StatusOr<PlannerObjectManager> PlannerObjectManagerBuilder::Build(
    FilteredTrajectories* filtered_trajs, ThreadPool* thread_pool) {
  SCOPED_QTRACE("PlannerObjectManagerBuilder::Build");

  // Filter trajectories.
  for (auto& object : planner_objects_) {
    auto& trajs = *object.mutable_prediction()->mutable_trajectories();
    int i = 0;
    for (int j = 0, n = trajs.size(); j < n; ++j) {
      auto& traj = trajs[j];
      bool is_traj_filtered = false;
      for (const auto* filter : filters_) {
        const auto reason = filter->Filter(object, traj);
        if (reason != FilterReason::NONE) {
          auto* filtered = filtered_trajs->add_filtered();
          filtered->set_reason(reason);
          filtered->set_id(object.id());
          filtered->set_index(traj.index());
          is_traj_filtered = true;
          break;
        }
      }
      if (!is_traj_filtered) {
        // This is equivalent to erasing the filtered element, but can reduce
        // the number of element moves.
        if (i != j) {
          trajs[i] = std::move(traj);
        }
        ++i;
      }
    }
    trajs.erase(trajs.begin() + i, trajs.end());
  }

  planner_objects_.erase(
      std::remove_if(planner_objects_.begin(), planner_objects_.end(),
                     [](const auto& obj) { return obj.num_trajs() == 0; }),
      planner_objects_.end());

  return PlannerObjectManager(std::move(planner_objects_));
}

ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool) {
  SCOPED_QTRACE("BuildPlannerObjects");

  auto object_info =
      JoinPerceptionAndPrediction(perception, prediction, thread_pool);

  std::vector<ObjectPredictionProto*> object_predictions;
  object_predictions.reserve(object_info.size());
  for (auto& [_, pred] : object_info) {
    object_predictions.push_back(&pred);
  }
  ObjectVector<PlannerObject> planner_objects;
  planner_objects.resize(object_info.size());
  ParallelFor(0, object_predictions.size(), thread_pool, [&](int i) {
    if (align_time.has_value()) {
      AlignPredictionTime(*align_time, object_predictions[i]).IgnoreError();
    }
    planner_objects[ObjectIndex(i)] =
        PlannerObject(prediction::ObjectPrediction(*object_predictions[i]));
  });

  MOVE_DESTROY_CONTAINER_ASYNC(thread_pool, object_info);

  return planner_objects;
}

}  // namespace planner
}  // namespace qcraft
