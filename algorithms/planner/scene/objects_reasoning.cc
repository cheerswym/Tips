#include "onboard/planner/scene/objects_reasoning.h"

#include <string>
#include <utility>
namespace qcraft::planner {

absl::StatusOr<ObjectsReasoningOutput> RunObjectsReasoning(
    const PlannerObjectManager& object_mgr) {
  // flat_hash_map: object_id, stall_probability.
  absl::flat_hash_map<std::string, double> stall_analysis;
  const auto& objects = object_mgr.planner_objects();
  for (const auto& object : objects) {
    const auto& object_id = object.id();
    const auto object_type = object.type();

    if (!object.is_stationary()) continue;

    if (object_type == ObjectType::OT_UNKNOWN_STATIC ||
        object_type == ObjectType::OT_BARRIER ||
        object_type == ObjectType::OT_CONE ||
        object_type == ObjectType::OT_WARNING_TRIANGLE) {
      stall_analysis[object_id] = 1.0;
      continue;
    }

    if (object.object_proto().has_parked() && object.object_proto().parked()) {
      stall_analysis[object_id] = 1.0;
      continue;
    }
  }

  std::vector<ObjectAnnotationProto> objects_annotation;
  objects_annotation.reserve(stall_analysis.size());
  for (const auto& [object_id, stall_probability] : stall_analysis) {
    ObjectAnnotationProto object_annotation;
    object_annotation.set_object_id(object_id);
    object_annotation.set_stalled_vehicle_likelyhood(stall_probability);
    object_annotation.set_depart_soon_likelyhood(1.0 - stall_probability);
    objects_annotation.emplace_back(std::move(object_annotation));
  }

  return ObjectsReasoningOutput{.objects_annotation =
                                    std::move(objects_annotation)};
}
}  // namespace qcraft::planner
