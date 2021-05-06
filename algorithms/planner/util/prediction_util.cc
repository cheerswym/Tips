#include "onboard/planner/util/prediction_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "google/protobuf/repeated_field.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
absl::Status AlignPredictionTime(double current_time,
                                 ObjectPredictionProto* obj_pred) {
  if (obj_pred->trajectories().empty()) {
    return absl::InvalidArgumentError("The prediction has no trajectory");
  }
  const double object_time = obj_pred->perception_object().timestamp();
  const double shift_time = current_time - object_time;
  if (shift_time < 0.0) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Current time[%f] is earlier than prediction's object time[%f]",
        current_time, object_time));
  }

  RETURN_IF_ERROR(prediction::AlignPerceptionObjectTime(
      current_time, obj_pred->mutable_perception_object()));

  auto& trajectories = *obj_pred->mutable_trajectories();
  if (trajectories.empty()) {
    return absl::NotFoundError(absl::StrCat("The trajectories of object ",
                                            obj_pred->perception_object().id(),
                                            " are all ignored trajectories."));
  }

  constexpr double kDt = prediction::kPredictionTimeStep;
  for (auto& traj : trajectories) {
    auto& points = *traj.mutable_points();
    if (!prediction::IsStationaryTrajectory(traj)) {
      // Remove the history points for moving objects.
      points.erase(points.begin(),
                   std::lower_bound(points.begin(), points.end(), shift_time,
                                    [](const auto& point, double t) {
                                      return point.t() < t;
                                    }));
      if (!points.empty()) {
        // Reset the s and t for the remaining points.
        const double last_s = points[0].s();
        double t = 0.0;
        for (auto& pt : points) {
          pt.set_t(t);
          pt.set_s(pt.s() - last_s);
          t += kDt;
        }
      }
    }
  }
  // Remove empty trajectories.
  trajectories.erase(
      std::remove_if(trajectories.begin(), trajectories.end(),
                     [](const auto& traj) { return traj.points_size() == 0; }),
      trajectories.end());
  if (trajectories.empty()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "All the trajectory points of object %s are filtered "
        "with shift_time=%f. The current time is %f and object time is %f",
        obj_pred->perception_object().id(), shift_time, current_time,
        object_time));
  }
  return absl::OkStatus();
}

absl::flat_hash_map<std::string_view, ObjectPredictionProto>
JoinPerceptionAndPrediction(const ObjectsProto* latest_perceptions,
                            const ObjectsPredictionProto* predictions,
                            ThreadPool* thread_pool) {
  SCOPED_QTRACE_ARG2(
      "JoinPerceptionAndPrediction", "num_objects",
      latest_perceptions == nullptr ? 0 : latest_perceptions->objects_size(),
      "num_predictions",
      predictions == nullptr ? 0 : predictions->objects_size());

  std::vector<std::pair<std::string_view, ObjectPredictionProto>>
      object_info_vec;
  if (predictions != nullptr) {
    object_info_vec.resize(predictions->objects_size());
    ParallelFor(0, predictions->objects_size(), thread_pool, [&](int i) {
      const auto& pred = predictions->objects(i);
      object_info_vec[i] = {pred.perception_object().id(), pred};
    });
  }

  absl::flat_hash_map<std::string_view, ObjectPredictionProto> object_info_map(
      std::make_move_iterator(object_info_vec.begin()),
      std::make_move_iterator(object_info_vec.end()));
  if (latest_perceptions != nullptr) {
    for (const auto& object : latest_perceptions->objects()) {
      const auto& id = object.id();
      if (ObjectPredictionProto* pred = FindOrNull(object_info_map, id)) {
        // Update the object field.
        if (object.timestamp() > pred->perception_object().timestamp()) {
          *pred->mutable_perception_object() = object;
        }
      } else {
        // Add prediction for recently received object.
        auto result = prediction::InstantPredictionForNewObject(object);
        QCHECK(result.ok()) << result.status().message();
        object_info_map.emplace(id, std::move(result).value());
      }
    }
  }
  return object_info_map;
}

}  // namespace planner
}  // namespace qcraft
