#include "onboard/planner/decision/leading_object.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

using ObjectsOnLane =
    std::vector<std::pair<double, const SpacetimeObjectTrajectory *>>;

constexpr double kObjectDirectionSameWithAVDirectionThresholdAngle =
    M_PI * 3.0 / 4.0;

constexpr double kObjectDirectionSameWithDrivePassageThresholdAngle =
    M_PI / 2.0;
// Used to compare double value.
constexpr double kEpsilon = 0.0001;

// Filter on coming object by drive passage.
absl::StatusOr<bool> IsOncomingObjectJudgeByDrivePassage(
    const DrivePassage &passage, const SecondOrderTrajectoryPoint &obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  constexpr double kSpeedNoiseFilter = 0.1;  // m/s.
  if (angle_diff < kObjectDirectionSameWithDrivePassageThresholdAngle ||
      obj_pose.v() < kSpeedNoiseFilter) {
    return false;
  }
  return true;
}

// Filter on coming object by av heading.
bool IsOncomingObjectJudgeByAVHeading(
    const ApolloTrajectoryPointProto &plan_start_point,
    const SecondOrderTrajectoryPoint &obj_pose) {
  const double av_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = obj_pose.theta();
  const double angle_diff =
      std::abs(NormalizeAngle(av_heading_angle - object_heading_angle));
  // Object moving in the opposite direction of av.
  return angle_diff > kObjectDirectionSameWithAVDirectionThresholdAngle;
}

absl::StatusOr<double> FilterObjectViaDrivePassage(
    const qcraft::VehicleGeometryParamsProto &vehicle_geom,
    const PlannerObject &object, const DrivePassage &passage,
    const PathSlBoundary &sl_boundary, const FrenetBox &av_frenet_box) {
  // Calculate object frenet coordinate.
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  // Filter object which behind AV front edge or beyond the passage length.
  if (object_frenet_box.s_min < av_frenet_box.s_max ||
      object_frenet_box.s_min > passage.end_s()) {
    return absl::OutOfRangeError(absl::StrCat(
        "object:\t", object.id(),
        ", is out of longitude boundary, frenet s min:\t",
        object_frenet_box.s_min, ", av front edge:\t", av_frenet_box.s_max,
        ",passage end s:\t", passage.end_s()));
  }

  // Filter object which deviate from target center lane.
  const double half_vehicle_width = 0.5 * vehicle_geom.width();
  if (object_frenet_box.l_min > half_vehicle_width ||
      object_frenet_box.l_max < -1.0 * half_vehicle_width) {
    return absl::OutOfRangeError(absl::StrCat(
        "object:\t", object.id(), ", is far from  target center lane:\t",
        object_frenet_box.l_min, ",\t", object_frenet_box.l_max,
        ", half vehicle width:\t", half_vehicle_width));
  }
  return object_frenet_box.s_min;
}

bool IsLeadingObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

ObjectsOnLane FindFrontObjectsOnLane(
    const qcraft::VehicleGeometryParamsProto &vehicle_geom,
    const PlannerSemanticMapManager &psmm, const DrivePassage &passage,
    const PathSlBoundary &sl_boundary,
    absl::Span<const SpacetimeObjectTrajectory *const> st_objs,
    const FrenetBox &av_frenet_box) {
  ObjectsOnLane st_objs_on_lane;
  st_objs_on_lane.reserve(st_objs.size());

  for (const auto *st_obj : st_objs) {
    // Filter by object type.
    if (!IsLeadingObjectType(st_obj->planner_object()->type())) {
      continue;
    }

    // Filter oncoming object.
    const auto res =
        IsOncomingObjectJudgeByDrivePassage(passage, st_obj->pose());
    if (!res.ok() || *res == true) {
      continue;
    }

    // Filter by drive passage and av frenet box.
    ASSIGN_OR_CONTINUE(
        const auto object_s,
        FilterObjectViaDrivePassage(vehicle_geom, *st_obj->planner_object(),
                                    passage, sl_boundary, av_frenet_box));
    st_objs_on_lane.emplace_back(object_s, st_obj);
  }

  // Sort by objects arc length on lane path.
  std::sort(st_objs_on_lane.begin(), st_objs_on_lane.end());

  return st_objs_on_lane;
}

// According the first lane id of traffic waiting queue's lane path to match
// current lane path.
absl::flat_hash_set<std::string_view> CollectTrafficWaitingObjectOnCurrentLane(
    const PlannerSemanticMapManager &psmm,
    const SceneOutputProto &scene_reasoning,
    const mapping::LanePath &lane_path) {
  absl::flat_hash_set<std::string_view> traffic_waiting_objects;
  for (const auto &traffic_waiting_queue :
       scene_reasoning.traffic_waiting_queue()) {
    const auto &lane_ids = lane_path.lane_ids();
    if (!traffic_waiting_queue.has_lane_path() ||
        traffic_waiting_queue.lane_path().lane_ids().empty()) {
      continue;
    }
    const auto &first_lane_id = traffic_waiting_queue.lane_path().lane_ids(0);

    if (std::find(lane_ids.begin(), lane_ids.end(), first_lane_id) !=
        lane_ids.end()) {
      for (const auto &object_id : traffic_waiting_queue.object_id()) {
        traffic_waiting_objects.insert(object_id);
      }
    }
  }
  return traffic_waiting_objects;
}

bool IsObjectAvoidableWithSlBoundary(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const FrenetBox &obj_frenet_box, const PathSlBoundary &sl_boundary) {
  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] = sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }

  return std::max(min_left_space, min_right_space) >
         vehicle_geometry_params.width();
}

// Check whether object within traffic light controlled intersection.
bool IsObjectWithinTlControlledIntersection(
    const PlannerSemanticMapManager &psmm, const FrenetBox &obj_frenet_box,
    const mapping::LanePath &lane_path) {
  for (const auto &seg : lane_path) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(seg.lane_id);
    if (lane_info.is_in_intersection == false) continue;
    for (const auto &[idx, frac] : lane_info.intersections) {
      if (!psmm.IntersectionAt(idx).proto->traffic_light_controlled()) continue;
      const double intersection_start_s =
          lane_path.LaneIndexPointToArclength(seg.lane_index, frac.x());
      const double intersection_end_s =
          lane_path.LaneIndexPointToArclength(seg.lane_index, frac.y());
      if (!(obj_frenet_box.s_min > intersection_end_s ||
            obj_frenet_box.s_max < intersection_start_s)) {
        return true;
      }
    }
  }
  return false;
}

bool IsObjectInTheAvFrontCenterArea(const FrenetBox &av_frenet_box,
                                    const FrenetBox &obj_frenet_box) {
  // Check object longitude position.
  if (obj_frenet_box.s_min < av_frenet_box.s_max) return false;

  // Check object lateral position.
  if (obj_frenet_box.l_max < av_frenet_box.l_min ||
      obj_frenet_box.l_min > av_frenet_box.l_max) {
    return false;
  }
  return true;
}

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory &traj, const DrivePassage &passage,
    ConstraintProto::LeadingObjectProto::Reason reason) {
  ConstraintProto::LeadingObjectProto leading_object;
  leading_object.set_traj_id(std::string(traj.traj_id()));
  leading_object.set_reason(reason);

  constexpr double time_interval = 1.0;  // Seconds.
  // Generate ST-constraints based on object current bounding box, for
  // stationary object.
  if (traj.is_stationary()) {
    const auto object_frenet_box =
        passage.QueryFrenetBoxAt(traj.bounding_box());
    if (!object_frenet_box.ok()) return leading_object;
    for (double sample_time = 0.0; sample_time < 10.0;
         sample_time += time_interval) {
      auto *pt = leading_object.add_st_constraints();
      pt->set_s(object_frenet_box->s_min);
      pt->set_t(sample_time);
    }
    return leading_object;
  }

  double sample_time = 0.0;
  const double traj_last_time = traj.states().back().traj_point->t();
  // Generate ST-constraints based on spacetime states, for moving object.
  for (const auto &state : traj.states()) {
    // Sample ST-constraints by 1.0s.
    const auto *traj_point = state.traj_point;
    const double t = traj_point->t();
    // Generate ST-constraints at sample time and trajectory last time.
    if (std::abs(t - sample_time) > kEpsilon &&
        std::abs(t - traj_last_time) > kEpsilon) {
      continue;
    }
    // Filter object state out of passage.
    const auto object_frenet_box = passage.QueryFrenetBoxAt(state.box);
    if (!object_frenet_box.ok()) break;

    // Filter object state by drive passage direction.
    const auto passage_tangent =
        passage.QueryTangentAngleAtS(object_frenet_box->s_min);

    if (!passage_tangent.ok()) break;
    const double angle_diff =
        std::abs(NormalizeAngle(*passage_tangent - traj_point->theta()));
    if (angle_diff > kObjectDirectionSameWithDrivePassageThresholdAngle) {
      break;
    }
    auto *pt = leading_object.add_st_constraints();
    pt->set_s(object_frenet_box->s_min);
    pt->set_t(t);
    sample_time += time_interval;
  }

  return leading_object;
}

}  // namespace

std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerSemanticMapManager &psmm,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const SceneOutputProto &scene_reasoning, const DrivePassage &passage,
    const PathSlBoundary &sl_boundary,
    const std::optional<ClearanceCheckOutput> &lc_clearance_check_output,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const ApolloTrajectoryPointProto &plan_start_point,
    const FrenetBox &av_frenet_box, bool borrow_lane_boundary) {
  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;
  absl::flat_hash_set<std::string_view> leading_object_ids;

  // Collect traffic waiting objects on current lane path.
  const auto traffic_waiting_objects = CollectTrafficWaitingObjectOnCurrentLane(
      psmm, scene_reasoning, passage.lane_path());

  // Find leading objects on current lane.
  const auto st_objs_on_lane = FindFrontObjectsOnLane(
      vehicle_geometry_params, psmm, passage, sl_boundary,
      st_traj_mgr.trajectories(), av_frenet_box);
  leading_objects.reserve(st_objs_on_lane.size());

  // Create forbidden to nudge leading objects.
  for (const auto &[_, traj_ptr] : st_objs_on_lane) {
    const auto obj_traj_id = traj_ptr->traj_id();
    const auto &object = *traj_ptr->planner_object();
    ASSIGN_OR_CONTINUE(const auto obj_frenet_box,
                       passage.QueryFrenetBoxAtContour(object.contour()));

    // Filter objects by AV heading.
    if (IsOncomingObjectJudgeByAVHeading(plan_start_point, traj_ptr->pose())) {
      continue;
    }

    // Stall object is not leading object.
    if (stalled_objects.contains(object.id())) continue;

    // Make leading decision for all non stall objects with borrow lane sl
    // boundary.
    if (borrow_lane_boundary) {
      leading_object_ids.insert(obj_traj_id);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE));
      continue;
    }

    // Traffic waiting object on current lane path is leading object.
    if (traffic_waiting_objects.contains(object.id())) {
      leading_object_ids.insert(obj_traj_id);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE));
      continue;
    }

    // Overtaking is not allowed at the traffic light controlled intersection,
    // when object in the av front center area.
    if (IsObjectInTheAvFrontCenterArea(av_frenet_box, obj_frenet_box) &&
        IsObjectWithinTlControlledIntersection(psmm, obj_frenet_box,
                                               passage.lane_path())) {
      leading_object_ids.insert(obj_traj_id);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE));
      continue;
    }

    // Can not avoid object is leading object.
    if (!IsObjectAvoidableWithSlBoundary(vehicle_geometry_params,
                                         obj_frenet_box, sl_boundary)) {
      leading_object_ids.insert(obj_traj_id);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::FORBIDDEN_TO_NUDGE));
      continue;
    }

    // TODO(jiayu): more strategy to make leading object decision.
  }
  return leading_objects;
}

std::vector<ConstraintProto::LeadingObjectProto> ConvertLaneChangeTargets(
    const DrivePassage &passage, const SpacetimeTrajectoryManager &st_traj_mgr,
    const std::vector<std::string> &lc_targets) {
  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;
  for (const auto &traj_id : lc_targets) {
    const auto *traj = st_traj_mgr.FindTrajectoryById(traj_id);
    if (traj == nullptr) {
      continue;
    }
    leading_objects.push_back(CreateLeadingObject(
        *traj, passage,
        ConstraintProto::LeadingObjectProto::LANE_CHANGE_TARGET));
  }
  return leading_objects;
}

}  // namespace planner
}  // namespace qcraft
