#include "onboard/planner/scene/occluded_objects_reasoning.h"

#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "onboard/planner/router/route_sections_util.h"
namespace qcraft::planner {

namespace {

// Lidar type sensor fov effective distance.
constexpr double kSensorFovForwardEffectiveDistance = 80.0;  // Meter.

// Along crosswalk heading direction (also call length direction) sample step.
constexpr double kCrosswalkLengthDirectionRawSampleStep = 1.2;  // Meter.

// Along crosswalk heading perpendicular direction (also call width direction)
// sample step.
constexpr double kCrosswalkWidthDirectionRawSampleStep = 1.2;  // Meter.

// Default pedestrian height.
constexpr double kDefaultPedestrianHeight = 1.5;  // Meter.

// Default pedestrian moving velocity.
constexpr double kDefaultPedestrianVelocity = 1.0;  // Meter/second.

// Default pedestrian bounding box length.
constexpr double kDefaultPedestrianBoundingBoxLendth = 0.5;  // Meter.

// Default pedestrian bounding box width.
constexpr double kDefaultPedestrianBoundingBoxWidth = 0.5;  // Meter.
struct CrosswalkManager {
  mapping::CrosswalkInfo crosswalk;
  double start_s_along_route;
  double end_s_along_route;
};

// Along route section from current collect crosswalks.
std::vector<CrosswalkManager> CollectCrosswalkAlongRouteSections(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &route_sections) {
  std::vector<CrosswalkManager> crosswalk_managers;

  const auto clamp_route_sections_or = ClampRouteSectionsBeforeArcLength(
      psmm, route_sections, kSensorFovForwardEffectiveDistance);
  if (!clamp_route_sections_or.ok()) return crosswalk_managers;

  double accumulate_s = 0.0;
  for (int i = 0; i < clamp_route_sections_or->size(); ++i) {
    const auto section_segment =
        clamp_route_sections_or->route_section_segment(i);
    const auto &section_info = psmm.FindSectionInfoOrDie(section_segment.id);

    absl::flat_hash_set<mapping::ElementId> crosswalk_id_set;
    // Collect crosswalks in each sections.
    for (const auto lane_id : section_info.lane_ids) {
      const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
      for (const auto &crosswalk : lane_info.crosswalks) {
        const auto &crosswalk_info = psmm.CrosswalkAt(crosswalk.first);
        // Ignore crosswalks that have been visited.
        if (crosswalk_id_set.contains(crosswalk_info.id)) continue;
        const auto &cw_field = crosswalk.second;
        // Ignore crosswalks that out of route section effective range.
        if (cw_field.x() < section_segment.start_fraction ||
            cw_field.y() > section_segment.end_fraction) {
          continue;
        }
        const double start_s_along_route =
            accumulate_s + lane_info.length() *
                               (cw_field.x() - section_segment.start_fraction);
        const double end_s_along_route =
            accumulate_s + lane_info.length() *
                               (cw_field.y() - section_segment.start_fraction);

        // Collect effective Crosswalks.
        crosswalk_id_set.insert(crosswalk_info.id);
        crosswalk_managers.push_back(
            CrosswalkManager{.crosswalk = crosswalk_info,
                             .start_s_along_route = start_s_along_route,
                             .end_s_along_route = end_s_along_route});
      }
    }
    // Update accumulate_s.
    accumulate_s +=
        (section_segment.end_fraction - section_segment.start_fraction) *
        section_info.average_length;
  }
  return crosswalk_managers;
}

InferredObjectProto CreateInferredObjectProto(const std::string &id,
                                              const Vec2d &unit_dir,
                                              const Vec2d &center) {
  InferredObjectProto proto;
  proto.set_infer_type(InferredObjectProto::OCCLUDED);

  // Set objectproto.
  auto *object_proto = proto.mutable_object_info();
  object_proto->set_id(id);
  object_proto->set_type(ObjectType::OT_PEDESTRIAN);
  Vec2dToProto(center, object_proto->mutable_pos());
  object_proto->set_yaw(unit_dir.Angle());
  object_proto->set_yaw_rate(0.0);
  // Set velocity 1.0 m/s.
  Vec2dToProto(unit_dir * kDefaultPedestrianVelocity,
               object_proto->mutable_vel());
  // Set acceleration 0.0m/s^2.
  Vec2dToProto(Vec2d(), object_proto->mutable_accel());

  // Set countor and boundxing box.
  const auto box =
      Box2d(center, unit_dir.Angle(), kDefaultPedestrianBoundingBoxLendth,
            kDefaultPedestrianBoundingBoxWidth);
  for (const auto &pt : box.GetCornersCounterClockwise()) {
    Vec2dToProto(pt, object_proto->add_contour());
  }
  box.ToProto(object_proto->mutable_bounding_box());

  object_proto->set_parked(false);
  object_proto->set_offroad(false);
  object_proto->set_moving_state(ObjectProto::MS_MOVING);

  // Unset pos_cov,time_stamp,laser_timestamp,bounding_box_source,bwv
  // left_time,min_z,max_z,icp_vel,fen_vel,is_radar_ony_object.

  return proto;
}

// Reasoning occluded objects on each crosswalk.
std::vector<InferredObjectProto> ReasoningOccludedObjectsOnOneCrosswalk(
    const mapping::CrosswalkInfo &cw_info,
    const sensor_fov::SensorFov &sensor_fov) {
  const auto &cw_unit_dir = cw_info.bone_axis_smooth.unit_direction();
  const auto heading = cw_unit_dir.Angle();
  const auto cw_bbox = cw_info.polygon_smooth.BoundingBoxWithHeading(heading);
  const auto sample_origin = cw_bbox.RearCenterPoint();

  // Recalculate sample step along crosswalk length direction.
  const int length_direction_sample_num =
      std::round(cw_bbox.length() / kCrosswalkLengthDirectionRawSampleStep);
  const double length_direction_sample_step =
      cw_bbox.length() / length_direction_sample_num;

  // Recalculate sample step along crosswalk width direction.
  const int width_direction_sample_num =
      std::round(cw_bbox.half_width() / kCrosswalkWidthDirectionRawSampleStep);
  const double width_direction_sample_step =
      cw_bbox.half_width() / width_direction_sample_num;

  // i belong to [1, n]  , j belong to [1, n] or [-1 ,-n].
  const auto calc_sample_center = [&](const int i, const int j) {
    const auto length_direction_offset =
        cw_unit_dir * (i * 1.0 - 0.5) * length_direction_sample_step;
    const auto width_direction_offset =
        cw_unit_dir.Perp() * (j > 0 ? j * 1.0 - 0.5 : j * 1.0 + 0.5) *
        width_direction_sample_step;
    return sample_origin + length_direction_offset + width_direction_offset;
  };

  std::vector<InferredObjectProto> inferred_objects;
  int occluded_object_id = 1;
  // Along length direction sample crosswalk.
  for (int i = 1; i <= length_direction_sample_num; ++i) {
    // Along width direction sample crosswalk.
    for (int j = 1; j <= width_direction_sample_num; ++j) {
      // Bidirectional.
      for (const auto k : {j, -j}) {
        const auto center = calc_sample_center(i, k);
        const auto box = Box2d(center, heading, length_direction_sample_step,
                               width_direction_sample_step);
        const auto is_box_occluded_or =
            sensor_fov.IsOccluded(box, kDefaultPedestrianHeight);
        // Box out of sensor fov detection range or not occluded.
        if (!is_box_occluded_or.ok() || *is_box_occluded_or == false) continue;
        // Create object with heading same as crosswalk heading direction.
        inferred_objects.emplace_back(CreateInferredObjectProto(
            absl::StrCat("occluded-", cw_info.id, "-", occluded_object_id++),
            cw_unit_dir, center));
        // Create object with heading same as crosswalk heading reverse
        // direction.
        inferred_objects.emplace_back(CreateInferredObjectProto(
            absl::StrCat("occluded-", cw_info.id, "-", occluded_object_id++),
            -1.0 * cw_unit_dir, center));
      }
    }
  }
  return inferred_objects;
}

// Reasoning occluded objects on the crosswalk.
std::vector<InferredObjectProto> ReasoningOccludedObjectsOnCrosswalks(
    const OccludedObjectsReasoningInput &input) {
  std::vector<InferredObjectProto> output_inferred_objects;

  const auto cw_mgrs =
      CollectCrosswalkAlongRouteSections(*input.psmm, *input.route_sections);
  for (const auto &cw_mgr : cw_mgrs) {
    auto inferred_objects = ReasoningOccludedObjectsOnOneCrosswalk(
        cw_mgr.crosswalk, *input.sensor_fov);
    for (auto &obj : inferred_objects) {
      output_inferred_objects.emplace_back(std::move(obj));
    }
  }
  return output_inferred_objects;
}

}  // namespace

absl::StatusOr<OccludedObjectsReasoningOutput> RunOccludedObjectsReasoning(
    const OccludedObjectsReasoningInput &input) {
  SCOPED_QTRACE("OccludedObjectsReasoning");
  std::vector<InferredObjectProto> inferred_objects;

  // Reasoning occluded objects on crosswalk.
  auto occluded_objects_on_crosswalk =
      ReasoningOccludedObjectsOnCrosswalks(input);
  inferred_objects.reserve(occluded_objects_on_crosswalk.size());
  for (auto &occluded_object : occluded_objects_on_crosswalk) {
    inferred_objects.emplace_back(std::move(occluded_object));
  }

  // TODO(jiayu): Reasoning occluded objects in other scenarios.
  return OccludedObjectsReasoningOutput{.inferred_objects =
                                            std::move(inferred_objects)};
}
}  // namespace qcraft::planner
