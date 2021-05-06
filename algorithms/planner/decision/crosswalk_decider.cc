#include "onboard/planner/decision/crosswalk_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

struct CrosswalkManager {
  mapping::CrosswalkInfo crosswalk;
  double start_s_on_drive_passage;
  double end_s_on_drive_passage;
  bool tl_controlled;
  const CrosswalkStateProto *last_state = nullptr;
};

// Used to compare double values.
constexpr double kEpsilon = 1e-5;

// This variable depends on the accuracy of the prediction module. For now,
// let's set this value to 3.0s, which means that only predicted trajectories of
// less than 3.0s are considered.
constexpr double kStableObjectPredictedTrajectoryTime = 2.0;  // s

// This variable is used to determine the distance threshold of objects
// associated with crosswalk. If the minimum distance between the object and
// the crosswalk is greater than this value, ignore this object.
constexpr double kCrosswalkAssociationDistance = 1.3;  // m

// If the distance between the object and the vehicle boundary is greater than
// this value and the movement direction deviates from the lane center, ignore
// this object.
constexpr double kIgnoreLateralMoveOffBuffer = 1.0;  // m

// If the distance between the object and the vehicle boundary is greater than
// this value and the movement direction toward the lane center, ignore
// this object.
constexpr double kIgnoreLateralTowardBuffer = 5.0;  // m

// The stand off distance before the crosswalk, when AV decides to stop before
// this crosswalk.
constexpr double kCrossWalkStandoff = 2.0;  // m

// Ignore crosswalk which far away from AV current position
constexpr double kMaxLookAheadDistance = 150;  // m

// The comfortable starting acceleration when AV stop before crosswalk
constexpr double kComfortableAcceleration = 0.5;  // m/s^2

// The two variables are used to distinguish APPROACHING/STOP/PASSED state.
// kStopMinDistance may relative to kCrossWalkStandoff
constexpr double kStopMinDistance = 3.0;  // m
constexpr double kStopMaxVelocity = 0.3;  // m/s

// At the first 5s after av stop before crosswalk, av will yield both static and
// moving objects. 5s later, making stop decision depend on av or pedestrian who
// will reach the crosspoint first.
constexpr double kMaxYieldingTime = 8.0;  // s

constexpr double kEmergencyStopDeceleration = -3.5;  // m/s^2

// The function to calculate comfortable deceleration according vehicle speed
const PiecewiseLinearFunction kVehicleSpeedComfortableBrakePlf(
    std::vector<double>{0.0, 5.0, 10.0}, std::vector<double>{-3.5, -2.5, -1.5});

// The function to calculate time gain when object moving on the crosswalk.
const PiecewiseLinearFunction kObjectMovingTimeGainPlf(
    std::vector<double>{0.1, 0.5}, std::vector<double>{0.0, 4.0});

const CrosswalkStateProto *FindCrosswalkStateOrNull(
    mapping::ElementId crosswalk_id,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
        &crosswalk_states) {
  for (const auto &state : crosswalk_states) {
    if (state.crosswalk_id() == crosswalk_id) return &state;
  }
  return nullptr;
}

std::vector<CrosswalkManager> FindCrosswalks(
    const PlannerSemanticMapManager &psmm, const DrivePassage &passage,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
        &crosswalk_states) {
  std::vector<CrosswalkManager> crosswalk_managers;

  const auto &current_lane_path = passage.lane_path();

  for (auto lane_it = current_lane_path.begin();
       lane_it != current_lane_path.end(); ++lane_it) {
    const auto &lane_info = psmm.FindLaneInfoOrDie((*lane_it).lane_id);

    for (const auto &cw : lane_info.crosswalks) {
      const auto &cw_info = psmm.CrosswalkAt(cw.first);
      const auto &lane_index = lane_it.lane_index();

      const auto &cw_field = cw.second;
      const double start_s_on_ref_path =
          current_lane_path.LaneIndexPointToArclength(lane_index, cw_field.x());
      const double end_s_on_ref_path =
          current_lane_path.LaneIndexPointToArclength(lane_index, cw_field.y());

      // ignore crosswalk which behind AV
      if (end_s_on_ref_path < kEpsilon) continue;

      // ignore crosswalk which far away from AV
      if (end_s_on_ref_path > kMaxLookAheadDistance) continue;

      const bool tl_ctrl_cw =
          cw_info.proto->has_type()
              ? cw_info.proto->type() ==
                    mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED
              : false;
      const bool tl_ctrl_path = !lane_info.traffic_lights.empty();
      const bool tl_controlled = tl_ctrl_cw && tl_ctrl_path;

      constexpr double kQeventSignCrosswalkDistance = 5.0;
      // Record un tl controlled crosswalk.
      if (start_s_on_ref_path < kQeventSignCrosswalkDistance &&
          tl_controlled == false) {
        QEVENT_EVERY_N_SECONDS("jiayukou", "crosswalk_un_tl_controlled",
                               /*every_n_seconds=*/60.0, [&](QEvent *qevent) {
                                 qevent->AddField("cw_id", cw_info.id)
                                     .AddField("cw_type", cw_info.proto->type())
                                     .AddField("start_s", start_s_on_ref_path)
                                     .AddField("end_s", end_s_on_ref_path)
                                     .AddField("tl_control", tl_controlled);
                               });
      }

      crosswalk_managers.emplace_back(CrosswalkManager{
          .crosswalk = cw_info,
          .start_s_on_drive_passage =
              start_s_on_ref_path + passage.lane_path_start_s(),
          .end_s_on_drive_passage =
              end_s_on_ref_path + passage.lane_path_start_s(),
          .tl_controlled = tl_controlled,
          .last_state =
              FindCrosswalkStateOrNull(cw_info.id, crosswalk_states)});
    }
  }

  return crosswalk_managers;
}

bool IsComfortableDecelerationForCrosswalk(double speed, double av_front_edge_s,
                                           double crosswalk_start_s) {
  // Yield to pedestrian only when AV not on the crosswalk
  if (crosswalk_start_s < av_front_edge_s) return false;

  const double comfortable_deceleration_dist =
      0.5 * speed * speed / std::fabs(kVehicleSpeedComfortableBrakePlf(speed));

  return crosswalk_start_s > av_front_edge_s + comfortable_deceleration_dist;
}

// This function return the time gain, When crosswalk type is UNTLCONTROLED.
// Allow more time for pedestrian to crosswalk. Because of the speed of cyclists
// is faster than pedestian ,so deal with this problem separately.
double UnTlControledCrosswalkTimeGain(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
      return 4.0;
    case OT_CYCLIST:
      return 0.6;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.0;
  }
}
bool IsConsideredType(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

bool IsObjectTrajectoryOnCrosswalk(const CrosswalkManager &cw_manager,
                                   const PlannerObject &object) {
  const auto &cw_info = cw_manager.crosswalk;
  const auto &trajs = object.prediction().trajectories();
  for (const auto &pred_traj : trajs) {
    const auto &pred_traj_points = pred_traj.points();
    for (const auto &pt : pred_traj_points) {
      if (pt.t() > kStableObjectPredictedTrajectoryTime) break;

      if (cw_info.polygon_smooth.DistanceSquareTo(pt.pos()) < Sqr(kEpsilon))
        return true;
    }
  }
  return false;
}

bool IsObjectNearCrosswalk(const CrosswalkManager &cw_manager,
                           const PlannerObject &object, double distance) {
  const auto &cw_info = cw_manager.crosswalk;

  return cw_info.polygon_smooth.DistanceSquareTo(object.pose().pos()) <
         Sqr(distance);
}

absl::StatusOr<ConstraintProto::StopLineProto> GenerateCrosswalkConstraints(
    const CrosswalkManager &cw_manager, const DrivePassage &passage) {
  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(
                                         cw_manager.start_s_on_drive_passage));

  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(cw_manager.start_s_on_drive_passage);
  stop_line.set_standoff(kCrossWalkStandoff);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("crosswalk_", cw_manager.crosswalk.id));
  stop_line.mutable_source()->mutable_crosswalk()->set_id(
      cw_manager.crosswalk.id);

  return stop_line;
}

// AV yield to static object, only for must yied type crosswalk
bool CheckCrosswalkTypeValidToYieldObject(mapping::CrosswalkProto::Type type) {
  switch (type) {
    case mapping::CrosswalkProto::MUST_YEILD:
      return true;
    case mapping::CrosswalkProto::UNCONTROLLED:
    case mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED:
      return false;
  }
}

double CalculateAccelerationTime(double dist, double vel_begin, double acc) {
  return (std::sqrt(2 * acc * dist + Sqr(vel_begin)) - vel_begin) / acc;
}

CrosswalkStateProto CreateApproachingState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);
  new_state.set_stop_state(CrosswalkStateProto::APPROACHING);
  return new_state;
}

CrosswalkStateProto CreatePassedState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);
  new_state.set_stop_state(CrosswalkStateProto::PASSED);
  return new_state;
}

CrosswalkStateProto CreateFirstStoppedState(mapping ::ElementId crosswalk_id,
                                            double now_in_seconds) {
  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);
  new_state.set_stop_time(now_in_seconds);
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

// This function will return new state which not contains yield objects from
// last state.
CrosswalkStateProto UpdateCrosswalkState(double av_front_edge_s,
                                         double plan_start_v,
                                         double now_in_seconds,
                                         const CrosswalkManager &cw_manager) {
  // If make decision for this crosswalk for the first time , creat approching
  // state
  const auto &crosswalk_id = cw_manager.crosswalk.id;
  if (cw_manager.last_state == nullptr) {
    return CreateApproachingState(crosswalk_id);
  }

  const auto &last_state = *cw_manager.last_state;
  // If last state is PASSED, just return.
  if (last_state.stop_state() == CrosswalkStateProto::PASSED)
    return CreatePassedState(crosswalk_id);
  // If last state is APPROACHING, update state according to av position and
  // velocity
  if (last_state.stop_state() == CrosswalkStateProto::APPROACHING) {
    if (av_front_edge_s >
            cw_manager.start_s_on_drive_passage - kStopMinDistance &&
        plan_start_v < kStopMaxVelocity) {
      return CreateFirstStoppedState(crosswalk_id, now_in_seconds);
    } else if (av_front_edge_s > cw_manager.end_s_on_drive_passage) {
      return CreatePassedState(crosswalk_id);
    } else {
      return CreateApproachingState(crosswalk_id);
    }
  }

  // STOPPED ==> PASSING meet one of the following condition
  // 1. Exceed max yield time, which is 8.0s now
  // 2. AV has driven out of the crosswalk
  const bool exceed_yield_time =
      now_in_seconds - last_state.stop_time() > kMaxYieldingTime;
  if (exceed_yield_time ||
      av_front_edge_s > cw_manager.end_s_on_drive_passage) {
    return CreatePassedState(crosswalk_id);
  }

  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);
  new_state.set_stop_time(last_state.stop_time());
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

double CalculateObjectAcrossRatio(const PlannerObject &object,
                                  const CrosswalkManager &cw_manager) {
  const auto &cw_segment = cw_manager.crosswalk.bone_axis_smooth;
  const auto &cw_unit_dir = cw_segment.unit_direction();

  const double object_signed_offset =
      (object.pose().pos() - cw_segment.start()).dot(cw_unit_dir);
  double across_ratio =
      std::clamp(object_signed_offset / cw_segment.length(), 0.0, 1.0);

  const Vec2d &object_vel = object.velocity();

  if (object_vel.dot(cw_unit_dir) < kEpsilon) {
    across_ratio = 1.0 - across_ratio;
  }
  return across_ratio;
}

}  // namespace

absl::StatusOr<CrosswalkDeciderOutput> BuildCrosswalkConstraints(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerSemanticMapManager &psmm,
    const ApolloTrajectoryPointProto &plan_start_point,
    const DrivePassage &passage, const PlannerObjectManager &obj_mgr,
    double now_in_seconds,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>
        &last_crosswalk_states) {
  std::vector<ConstraintProto::StopLineProto> cw_stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> cw_speed_regions;
  std::vector<CrosswalkStateProto> cw_states;
  // 1. Find nearest crosswalk
  const auto crosswalk_managers =
      FindCrosswalks(psmm, passage, last_crosswalk_states);

  // 2. Based on the trajectories of related objects, generate stop lines and
  // speed regions for each crosswalk

  const Vec2d plan_start_pos_xy(plan_start_point.path_point().x(),
                                plan_start_point.path_point().y());

  const auto plan_start_pos_sl =
      passage.QueryFrenetCoordinateAt(plan_start_pos_xy);
  if (!plan_start_pos_sl.ok()) {
    return plan_start_pos_sl.status();
  }

  const auto av_front_edge_s =
      vehicle_geometry_params.front_edge_to_center() + plan_start_pos_sl->s;

  // make sure plan_start_v greater than 0.0
  const double plan_start_v = std::max(plan_start_point.v(), kEpsilon);

  const auto av_front_edge_xy =
      passage.QueryPointXYAtSL(av_front_edge_s, plan_start_pos_sl->l);
  if (!av_front_edge_xy.ok()) return av_front_edge_xy.status();

  for (const auto &cw_manager : crosswalk_managers) {
    // Update crosswalk state
    auto cw_state = UpdateCrosswalkState(av_front_edge_s, plan_start_v,
                                         now_in_seconds, cw_manager);
    // Record last yield objects
    absl::flat_hash_set<std::string> last_yield_objects;
    if (cw_manager.last_state != nullptr) {
      const auto &last_state = *cw_manager.last_state;
      const int num_objects = last_state.yield_objects_size();
      for (int i = 0; i < num_objects; ++i) {
        last_yield_objects.insert(last_state.yield_objects(i));
      }
    }

    if (cw_manager.tl_controlled) continue;

    const auto cw_start_xy = passage.QueryPointXYAtSL(
        cw_manager.start_s_on_drive_passage, plan_start_pos_sl->l);
    const auto cw_end_xy = passage.QueryPointXYAtSL(
        cw_manager.end_s_on_drive_passage, plan_start_pos_sl->l);
    if (!cw_start_xy.ok() || !cw_end_xy.ok()) continue;

    VLOG(2) << " * * * * *cw:\t" << cw_manager.crosswalk.id << " |start:\t"
            << cw_manager.start_s_on_drive_passage << " |end:\t"
            << cw_manager.end_s_on_drive_passage << " |tlc:\t"
            << cw_manager.tl_controlled;

    bool has_generate_stop_line = false;
    std::vector<std::string> new_yield_objects;

    for (const auto &object : obj_mgr.planner_objects()) {
      // 2.1 whether filter out this object
      // 2.1.1 Check object type
      if (!IsConsideredType(object.type())) continue;

      const bool object_near_cw = IsObjectNearCrosswalk(
          cw_manager, object, kCrosswalkAssociationDistance);
      // 2.1.2 Check object's trajectory on crosswalk ,in the next 2.0s and
      // Check object's position
      if (!object_near_cw &&
          !IsObjectTrajectoryOnCrosswalk(cw_manager, object)) {
        continue;
      }

      // 2.1.3 Check object moving off the AV path
      const Vec2d cw_axis_unit_dir =
          cw_manager.crosswalk.bone_axis_smooth.unit_direction();

      const Vec2d object_velocity = object.velocity();
      const double object_signed_velocity_proj_on_cw =
          object_velocity.dot(cw_axis_unit_dir);

      const bool av_already_on_crosswalk =
          cw_manager.start_s_on_drive_passage < av_front_edge_s;

      const Vec2d origin =
          av_already_on_crosswalk ? *av_front_edge_xy : *cw_start_xy;
      const double object_signed_offset_to_path =
          (object.pose().pos() - origin).dot(cw_axis_unit_dir);

      const bool same_side =
          object_signed_offset_to_path * object_signed_velocity_proj_on_cw >
          0.0;

      const double av_half_width =
          (vehicle_geometry_params.left_edge_to_center() +
           vehicle_geometry_params.right_edge_to_center()) *
          0.5;

      if (same_side && std::abs(object_signed_offset_to_path) >
                           kIgnoreLateralMoveOffBuffer + av_half_width) {
        continue;
      }

      // 2.1.4 Check object moving parallel to AV path

      if (std::abs(object_velocity.CrossProd(cw_axis_unit_dir)) >
          std::abs(object_signed_velocity_proj_on_cw)) {
        continue;
      }

      // 2.1.5 Check object behind AV front edge,when AV already on the
      // crosswalk
      const Vec2d cw_axis_perp_unit_dir = (*cw_end_xy - *cw_start_xy).Unit();
      const double object_offset_to_cw =
          (object.pose().pos() - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double av_offset_to_cw =
          (*av_front_edge_xy - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double object_lead_av_dist = object_offset_to_cw - av_offset_to_cw;

      if (av_already_on_crosswalk && object_lead_av_dist < kEpsilon) {
        continue;
      }

      // Three condition needed to be check:
      // 1. Object near the crosswalk with the distance to the crosswalk less
      // than 1.0m.
      // 2. The type of crosswalk is MUST_YIELD.
      // 3. It is comfortable to stop before the crosswalk.

      const bool cw_type_valid_yield_object =
          CheckCrosswalkTypeValidToYieldObject(
              cw_manager.crosswalk.proto->type());
      const bool comfortable_deceleration =
          IsComfortableDecelerationForCrosswalk(
              plan_start_v, av_front_edge_s,
              cw_manager.start_s_on_drive_passage);

      if (object_near_cw && cw_type_valid_yield_object &&
          comfortable_deceleration) {
        switch (cw_state.stop_state()) {
          case CrosswalkStateProto::APPROACHING:
          case CrosswalkStateProto::STOPPED:
            // Refrash yield objects every time
            new_yield_objects.push_back(object.id());
            break;
          case CrosswalkStateProto::PASSED:
            // Filter stationary object from last_yield_objects only in PASSED
            // state.
            if (!object.is_stationary() &&
                last_yield_objects.contains(object.id())) {
              new_yield_objects.push_back(object.id());
            }
            break;
        }
      }

      // 2.2 it is necessary to generate stop line for this crosswalk
      // 2.2.1 check obs Ahead of AV cross centerline

      const double distance_to_crosswalk =
          av_already_on_crosswalk
              ? object_lead_av_dist
              : cw_manager.start_s_on_drive_passage - av_front_edge_s;

      double av_t_at_cw = distance_to_crosswalk / plan_start_v;
      double av_t_at_cw_acceleration = CalculateAccelerationTime(
          distance_to_crosswalk, plan_start_v, kComfortableAcceleration);

      // According crosswalk type and object type calculate object_time_gain.
      double object_time_gain = UnTlControledCrosswalkTimeGain(object.type());
      if (object.type() == OT_PEDESTRIAN) {
        object_time_gain += kObjectMovingTimeGainPlf(
            CalculateObjectAcrossRatio(object, cw_manager));
      }

      // Check whether object ahead of av cross the crosswalk with time gain.
      const double object_ahead_of_av_dist_with_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              (av_t_at_cw + object_time_gain) +
          av_half_width - std::abs(object_signed_offset_to_path);
      const bool object_ahead_of_av_with_time_gain =
          object_ahead_of_av_dist_with_time_gain > kEpsilon;

      // Check whether object near av when av accelerate cross the crosswalk.
      const double object_near_av_dist_without_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              av_t_at_cw_acceleration +
          kIgnoreLateralTowardBuffer + av_half_width -
          std::abs(object_signed_offset_to_path);
      const bool object_near_av_without_time_gain =
          object_near_av_dist_without_time_gain > kEpsilon;

      // Check whether av stop comfortablely when yield to object.
      const double deceleration =
          -0.5 * Sqr(plan_start_v) / distance_to_crosswalk;
      const bool av_stop_comfortablely =
          deceleration > kEmergencyStopDeceleration;
      // 2.3 Generate stop lines for this crosswalk
      if (!has_generate_stop_line && object_ahead_of_av_with_time_gain &&
          object_near_av_without_time_gain && av_stop_comfortablely) {
        VLOG(2) << " - + - " << object.id() << ", is ahead of AV.";
        has_generate_stop_line = true;
      }
    }

    // Generate stop line to yield object
    if (has_generate_stop_line || !new_yield_objects.empty()) {
      for (const auto &obj_id : new_yield_objects) {
        cw_state.add_yield_objects(obj_id);
      }
      const auto stop_line = GenerateCrosswalkConstraints(cw_manager, passage);
      if (stop_line.ok()) {
        cw_stop_lines.emplace_back(*stop_line);
        VLOG(2) << " + + + Crosswalk stop line:\t" << cw_manager.crosswalk.id
                << ", generate at:\t" << cw_manager.start_s_on_drive_passage;
      }
    }

    cw_states.push_back(std::move(cw_state));
  }

  return CrosswalkDeciderOutput{.stop_lines = std::move(cw_stop_lines),
                                .speed_regions = std::move(cw_speed_regions),
                                .crosswalk_states = std::move(cw_states)};
}
}  // namespace planner
}  // namespace qcraft
