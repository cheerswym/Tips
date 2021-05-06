#include "onboard/planner/plan/plan_task_helper.h"

#include <limits>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/file_util.h"

DEFINE_bool(
    planner_force_init_uturn_task, false,
    "Force to init one uturn task to test uturn freespace planner offboard.");

DEFINE_string(planner_uturn_reference_lane_path, "",
              "Uturn reference lane path");

namespace qcraft::planner {

namespace {

ReachDestinationCondition CreateReachOffRoadRouteEndCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(0.1);
  condition.mutable_pose_within_radius()->set_radius(1.0);
  condition.mutable_pose_within_radius()->set_heading_error(0.1);
  return condition;
}

ReachDestinationCondition CreateReachOnRoadPoseCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(2.0);
  condition.mutable_pose_within_radius()->set_radius(3.0);
  condition.mutable_pose_within_radius()->set_heading_error(M_PI_4);
  return condition;
}

ReachDestinationCondition CreateReachOnRoadRouteEndCondition() {
  ReachDestinationCondition condition;
  condition.set_speed_error(0.2);
  condition.mutable_pose_within_radius()->set_radius(5.0);
  condition.mutable_pose_within_radius()->set_heading_error(
      std::numeric_limits<double>::infinity());
  return condition;
}

PlanTaskDestinationInfo CreateOffRoadStopTaskInfo(
    const OffRoadDestinationProto &offroad_dest) {
  PlanTaskDestination dest;
  if (offroad_dest.has_parking_spot()) {
    std::vector<mapping::ElementId> parking_spots;
    parking_spots.reserve(
        offroad_dest.parking_spot().specified_parking_spot_ids_size());
    for (const auto id :
         offroad_dest.parking_spot().specified_parking_spot_ids()) {
      parking_spots.push_back(id);
    }
    dest.parking_spots = std::move(parking_spots);
  } else {
    QLOG(FATAL) << "Should not reached here.";
  }

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOffRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOffRoadStopTaskInfoFromEndStrategy(
    const RouteEndStrategyProto &end_proto) {
  QCHECK(end_proto.has_parking_spot());
  PlanTaskDestination dest;
  std::vector<mapping::ElementId> parking_spots;
  parking_spots.reserve(
      end_proto.parking_spot().specified_parking_spot_ids_size());
  for (const auto id : end_proto.parking_spot().specified_parking_spot_ids()) {
    parking_spots.push_back(id);
  }
  dest.parking_spots = std::move(parking_spots);
  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOffRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOffRoadDepartTaskInfo(
    const RouteDepartStategyProto &depart_proto) {
  PlanTaskDestination dest;

  QCHECK(!depart_proto.off_road().specified_onroad_points().empty());

  const auto &destination_proto =
      *depart_proto.off_road().specified_onroad_points().begin();

  if (destination_proto.has_lane_point()) {
    dest.lane_points = {mapping::LanePoint(destination_proto.lane_point())};

  } else {
    QLOG(FATAL) << "Data type not supported for now.";
  }

  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 0.0,
                                 .condition = CreateReachOnRoadPoseCondition()};
}

PlanTaskDestinationInfo CreateOnRoadCruiseTaskInfo(
    const mapping::LanePoint &lane_point) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_point};

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 0.0,
      .condition = CreateReachOnRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateOnRoadCruiseBebofeUturnTaskInfo(
    const mapping::LanePoint &lane_point) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_point};

  return PlanTaskDestinationInfo{
      .dest = std::move(dest),
      .end_speed = 1.0,
      .condition = CreateReachOnRoadRouteEndCondition()};
}

PlanTaskDestinationInfo CreateUturnTaskInfo(
    const SemanticMapManager *smm, const mapping::LanePoint &goal,
    const mapping::LanePath &uturn_lane_path) {
  PlanTaskDestination dest;
  dest.lane_points = {goal};

  const Vec2d goal_pos = goal.ComputePos(*smm);
  const Vec2d uturn_end_pos = uturn_lane_path.back().ComputePos(*smm);

  ReachDestinationCondition condition;
  condition.set_speed_error(std::numeric_limits<double>::infinity());
  condition.mutable_pose_within_radius()->set_radius(
      goal_pos.DistanceTo(uturn_end_pos));
  condition.mutable_pose_within_radius()->set_heading_error(M_PI_4);

  mapping::LanePathProto lp_proto;
  uturn_lane_path.ToProto(&lp_proto);
  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 0.0,
                                 .condition = std::move(condition),
                                 .uturn_ref_lane_path = std::move(lp_proto)};
}

std::pair<Vec2d, double> ConvertDestinationToSmoothPose(
    const CoordinateConverter &cc, const PlanTaskDestination &task_dest,
    const PlannerSemanticMapManager &psmm) {
  if (task_dest.parking_spots.has_value()) {
    // const auto &parking_spot_info =
    //     psmm.semantic_map_manager()->FindParkingSpotByIdOrDie(
    //         task_dest.parking_spots->front());
    return std::make_pair(Vec2d(), 0.0);

  } else if (task_dest.lane_points.has_value()) {
    const auto &lane_point = task_dest.lane_points->front();
    return std::make_pair(
        lane_point.ComputePos(*psmm.semantic_map_manager()),
        lane_point.ComputeLerpTheta(*psmm.semantic_map_manager()));
  } else if (task_dest.global_pose.has_value()) {
    return std::make_pair(
        cc.GlobalToSmooth(Vec2d(task_dest.global_pose->pos.x(),
                                task_dest.global_pose->pos.y())),
        cc.GlobalYawToSmooth(task_dest.global_pose->heading));
  } else {
    QLOG(FATAL) << "Should not reach here.";
  }
}

PlanTaskDestinationInfo CreateUturnTaskFromLanePath(
    const mapping::LanePath &lane_path) {
  PlanTaskDestination dest;
  dest.lane_points = {lane_path.back()};
  mapping::LanePathProto lp_proto;
  lane_path.ToProto(&lp_proto);
  return PlanTaskDestinationInfo{.dest = std::move(dest),
                                 .end_speed = 1.0,
                                 .condition = CreateReachOnRoadPoseCondition(),
                                 .uturn_ref_lane_path = std::move(lp_proto)};
}

absl::StatusOr<std::vector<mapping::LanePath::LaneSegment>>
FindUturnSubLanePathWithinPreview(const PlannerSemanticMapManager &psmm,
                                  const mapping::LanePath &target_lane_path,
                                  double preview_distance) {
  std::vector<mapping::LanePath::LaneSegment> sub_lane_segments;
  for (const auto &lane_seg : target_lane_path) {
    const auto &lane_info = psmm.FindLaneInfoOrDie(lane_seg.lane_id);

    if (lane_info.direction == mapping::LaneProto::UTURN) {
      if (sub_lane_segments.empty()) {
        if (lane_seg.start_s < preview_distance) {
          sub_lane_segments.push_back(lane_seg);
        } else {
          return absl::NotFoundError("beyond preview distance");
        }
      } else if (lane_seg.lane_index ==
                 sub_lane_segments.back().lane_index + 1) {
        sub_lane_segments.push_back(lane_seg);
      } else {
        break;
      }
    }
  }

  if (sub_lane_segments.empty()) {
    return absl::NotFoundError("");
  }

  return sub_lane_segments;
}

}  // namespace

std::deque<PlanTask> CreatePlanTasksQueueFromRoutingResult(
    const RouteManagerOutput &route_output, const SemanticMapManager &smm) {
  const auto &stop_info = route_output.destination_stop;
  std::deque<PlanTask> tasks;

  if (FLAGS_planner_force_init_uturn_task) {
    QCHECK(!FLAGS_planner_uturn_reference_lane_path.empty());
    mapping::LanePathProto lane_path_proto;
    file_util::StringToProto(FLAGS_planner_uturn_reference_lane_path,
                             &lane_path_proto);
    tasks.emplace_back(
        UTURN_PLAN,
        CreateUturnTaskFromLanePath(mapping::LanePath(&smm, lane_path_proto)));
    return tasks;
  }

  if (stop_info.stop_point().has_off_road()) {
    // APA
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadStopTaskInfo(
                                          stop_info.stop_point().off_road()));
    return tasks;
  }

  if (stop_info.has_depart_strategy()) {
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadDepartTaskInfo(
                                          stop_info.depart_strategy()));
  }

  tasks.emplace_back(ON_ROAD_CRUISE_PLAN,
                     CreateOnRoadCruiseTaskInfo(
                         route_output.route_from_current->lane_path().back()));

  if (stop_info.has_stop_strategy() &&
      stop_info.stop_strategy().has_parking_spot()) {
    tasks.emplace_back(OFF_ROAD_PLAN, CreateOffRoadStopTaskInfoFromEndStrategy(
                                          stop_info.stop_strategy()));
  }

  return tasks;
}

bool PlanTaskCompeleted(const PlanTask &task, const CoordinateConverter &cc,
                        Vec2d front_bumper_pos, double ego_heading,
                        double ego_v, const PlannerSemanticMapManager &psmm) {
  const PlanTaskDestinationInfo &dest_info = task.destination_info();

  const ReachDestinationCondition &condition =
      task.destination_info().condition;

  if (condition.has_pose_within_radius()) {
    const auto [pos, heading] =
        ConvertDestinationToSmoothPose(cc, dest_info.dest, psmm);

    if (std::abs(ego_v - dest_info.end_speed) > condition.speed_error()) {
      return false;
    }
    if (std::abs(NormalizeAngle(ego_heading - heading)) >
        condition.pose_within_radius().heading_error()) {
      return false;
    }
    if (pos.DistanceSquareTo(front_bumper_pos) >
        Sqr(condition.pose_within_radius().radius())) {
      return false;
    }
    return true;
  } else {
    QLOG(FATAL) << "Should not reached here.";
  }

  return false;
}

absl::StatusOr<std::vector<PlanTask>> SplitCruiseByUturnTask(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePath &prev_target_lane_path,
    const mapping::LanePoint &route_destination) {
  constexpr double kUturnPreviewDistance = 20.0;  // m.

  ASSIGN_OR_RETURN(const auto uturn_lane_segs,
                   FindUturnSubLanePathWithinPreview(
                       psmm, prev_target_lane_path, kUturnPreviewDistance));

  std::vector<mapping::ElementId> sub_lane_ids;
  sub_lane_ids.reserve(uturn_lane_segs.size());
  for (const auto &lane_seg : uturn_lane_segs) {
    sub_lane_ids.push_back(lane_seg.lane_id);
  }
  const mapping::LanePath uturn_lane_path(
      psmm.semantic_map_manager(), std::move(sub_lane_ids),
      uturn_lane_segs.front().start_fraction,
      uturn_lane_segs.back().end_fraction);

  std::vector<PlanTask> tasks;

  // Create cruise task before uturn.
  if (uturn_lane_segs.front().start_s > kMinCruiseLength) {
    tasks.emplace_back(
        ON_ROAD_CRUISE_PLAN,
        CreateOnRoadCruiseBebofeUturnTaskInfo(uturn_lane_path.front()));
  }

  // Create uturn task.

  // Find uturn task goal point.
  constexpr double kUturnGoalExtendDistance = 20.0;
  mapping::LanePoint uturn_goal = uturn_lane_path.back();
  for (int i = uturn_lane_segs.back().lane_index + 1;
       i < prev_target_lane_path.size(); ++i) {
    const auto &seg = prev_target_lane_path.lane_segment(i);

    if (seg.end_s - uturn_lane_segs.back().end_s > kUturnGoalExtendDistance) {
      const double delta_s =
          seg.end_s - uturn_lane_segs.back().end_s - kUturnGoalExtendDistance;
      const double lane_len = psmm.GetLaneLengthOrDie(seg.lane_id);
      uturn_goal = mapping::LanePoint(seg.lane_id,
                                      seg.end_fraction - delta_s / lane_len);
      break;
    }

    if (i + 1 == prev_target_lane_path.size()) {
      uturn_goal = prev_target_lane_path.back();
      break;
    }
  }

  tasks.emplace_back(
      UTURN_PLAN, CreateUturnTaskInfo(psmm.semantic_map_manager(), uturn_goal,
                                      uturn_lane_path));

  // Create cruise task after uturn.
  if (uturn_goal != prev_target_lane_path.back()) {
    tasks.emplace_back(ON_ROAD_CRUISE_PLAN,
                       CreateOnRoadCruiseTaskInfo(route_destination));
  }

  return tasks;
}

absl::StatusOr<std::vector<PlanTask>> CreateUturnTask(
    const PlannerSemanticMapManager &psmm, const PoseProto &ego_pose,
    const mapping::LanePath &prev_target_lane_path,
    const mapping::LanePoint &route_destination) {
  constexpr double kMaxSpeedForUturnTask = 0.2;  // m/s

  if (ego_pose.vel_body().x() > kMaxSpeedForUturnTask) {
    return absl::UnavailableError("speed too high.");
  }

  constexpr double kUturnPreviewDistance = 20.0;  // m.

  ASSIGN_OR_RETURN(const auto uturn_lane_segs,
                   FindUturnSubLanePathWithinPreview(
                       psmm, prev_target_lane_path, kUturnPreviewDistance));

  if (uturn_lane_segs.front().start_s > 1e-3) {
    return absl::UnavailableError("not reached uturn.");
  }

  std::vector<mapping::ElementId> sub_lane_ids;
  sub_lane_ids.reserve(uturn_lane_segs.size());
  for (const auto &lane_seg : uturn_lane_segs) {
    sub_lane_ids.push_back(lane_seg.lane_id);
  }
  const mapping::LanePath uturn_lane_path(
      psmm.semantic_map_manager(), std::move(sub_lane_ids),
      uturn_lane_segs.front().start_fraction,
      uturn_lane_segs.back().end_fraction);

  std::vector<PlanTask> tasks;

  // Create uturn task.
  // Find uturn task goal point.
  constexpr double kUturnGoalExtendDistance = 20.0;
  mapping::LanePoint uturn_goal = uturn_lane_path.back();
  for (int i = uturn_lane_segs.back().lane_index + 1;
       i < prev_target_lane_path.size(); ++i) {
    const auto &seg = prev_target_lane_path.lane_segment(i);

    if (seg.end_s - uturn_lane_segs.back().end_s > kUturnGoalExtendDistance) {
      const double delta_s =
          seg.end_s - uturn_lane_segs.back().end_s - kUturnGoalExtendDistance;
      const double lane_len = psmm.GetLaneLengthOrDie(seg.lane_id);
      uturn_goal = mapping::LanePoint(seg.lane_id,
                                      seg.end_fraction - delta_s / lane_len);
      break;
    }

    if (i + 1 == prev_target_lane_path.size()) {
      uturn_goal = prev_target_lane_path.back();
      break;
    }
  }

  tasks.emplace_back(
      UTURN_PLAN, CreateUturnTaskInfo(psmm.semantic_map_manager(), uturn_goal,
                                      uturn_lane_path));

  // Create cruise task after uturn.
  if (uturn_goal != prev_target_lane_path.back()) {
    tasks.emplace_back(ON_ROAD_CRUISE_PLAN,
                       CreateOnRoadCruiseTaskInfo(route_destination));
  }
  return tasks;
}

}  // namespace qcraft::planner
