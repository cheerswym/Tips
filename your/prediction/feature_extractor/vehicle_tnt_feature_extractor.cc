#include "onboard/prediction/feature_extractor/vehicle_tnt_feature_extractor.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
// The detection region should be consistent with the classifier.
constexpr float kDetectionHalfWidth = 51.2f;
constexpr float kDetectionRegionFront = 112.0f;
constexpr float kDetectionRegionBehind = 67.2f;
constexpr float kStaticEpsilon = 1e-2;    // m/s.
constexpr double kMaxDistToTarget = 2.0;  // m.
constexpr int kMaxTntContextObjectNum = 31;
DEFINE_bool(smooth_tnt_gt_trajectory, true,
            "Smooth TNT ground truth trajectory.");
inline Box2d GetRegionBox(const Vec2d &pos, const double heading) {
  return qcraft::prediction::GetRegionBox(pos, heading, kDetectionRegionFront,
                                          kDetectionRegionBehind,
                                          kDetectionHalfWidth);
}

VehicleTNTActorsFeature ExtractVehicleTNTActorsFeature(
    Vec2d ref_pos, double rot_rad,
    const ResampledObjectsHistory &objects_history,
    const std::vector<qcraft::ObjectProto> &ego_history) {
  const int objects_num = objects_history.size();
  std::vector<float> trajs(objects_num * kVehicleTNTConfig.history_num *
                           kVehicleTNTConfig.coord_num);
  std::vector<float> speeds(objects_num * kVehicleTNTConfig.history_num);
  std::vector<float> headings(objects_num * kVehicleTNTConfig.history_num *
                              kVehicleTNTConfig.coord_num);
  std::vector<float> cur_poses(objects_num * kVehicleTNTConfig.coord_num);
  SetHistoryStatesAndCurPoses(
      kVehicleTNTConfig.coord_num, kVehicleTNTConfig.history_num,
      kVehicleTNTConfig.history_time_step, ref_pos, rot_rad, objects_history,
      &trajs, &speeds, &headings, &cur_poses);

  std::vector<float> ego_traj(kVehicleTNTConfig.history_num *
                              kVehicleTNTConfig.coord_num);
  std::vector<float> ego_speeds(kVehicleTNTConfig.history_num);
  std::vector<float> ego_headings(kVehicleTNTConfig.history_num *
                                  kVehicleTNTConfig.coord_num);
  std::vector<float> ego_cur_poses(kVehicleTNTConfig.coord_num);
  SetHistoryStatesAndCurPoses(
      kVehicleTNTConfig.coord_num, kVehicleTNTConfig.history_num,
      kVehicleTNTConfig.history_time_step, ref_pos, rot_rad, {ego_history},
      &ego_traj, &ego_speeds, &ego_headings, &ego_cur_poses);

  return VehicleTNTActorsFeature{
      .ego_traj = std::move(ego_traj),
      .ego_speeds = std::move(ego_speeds),
      .ego_headings = std::move(ego_headings),
      .trajs = std::move(trajs),
      .speeds = std::move(speeds),
      .headings = std::move(headings),
      .cur_poses = std::move(cur_poses),
      .types = VotingTypes<PredictionObjectType>(objects_history),
  };
}

void SetLaneLights(absl::Span<const mapping::LaneInfo *const> lane_ptrs,
                   const TrafficLightManager::TLStateHashMap &tl_states,
                   std::vector<float> *lane_lights) {
  lane_lights->clear();
  lane_lights->reserve(lane_ptrs.size() * kVehicleTNTConfig.lane_light_num);
  for (const auto *lane_ptr : lane_ptrs) {
    int num_green = 0;
    int num_yellow = 0;
    int num_red = 0;
    int num_unknown = 0;
    for (const auto *light : lane_ptr->traffic_lights) {
      const auto *light_stat = FindOrNull(tl_states, light->id());
      if (light_stat == nullptr) {
        ++num_unknown;
        continue;
      }
      switch (light_stat->color()) {
        case TL_GREEN:
          ++num_green;
          break;
        case TL_YELLOW:
          ++num_yellow;
          break;
        case TL_RED:
          ++num_red;
          break;
        case TL_UNKNOWN:
          ++num_unknown;
          break;
      }
    }
    lane_lights->push_back(num_green);
    lane_lights->push_back(num_yellow);
    lane_lights->push_back(num_red);
    lane_lights->push_back(num_unknown);
  }
}

VehicleTNTLanesFeature ExtractVehicleTNTLanesFeature(
    Vec2d ref_pos, double rot_rad,
    const std::vector<const mapping::LaneInfo *> lane_ptrs,
    const TrafficLightManager::TLStateHashMap &tl_states) {
  const int lane_size = lane_ptrs.size();
  std::vector<float> lane_centers(lane_size *
                                  kVehicleTNTConfig.max_lane_point_num *
                                  kVehicleTNTConfig.coord_num);
  std::vector<float> lane_segments(lane_size *
                                   kVehicleTNTConfig.max_lane_point_num *
                                   kVehicleTNTConfig.coord_num);
  std::vector<float> lane_controls(lane_size *
                                   kVehicleTNTConfig.lane_control_num);
  for (int i = 0; i < lane_size; ++i) {
    SetLaneCentersAndSegments(kVehicleTNTConfig.max_lane_point_num,
                              kVehicleTNTConfig.coord_num, ref_pos, rot_rad, i,
                              lane_ptrs[i], &lane_centers, &lane_segments);
    SetLaneControls(kVehicleTNTConfig.lane_control_num, i, lane_ptrs[i],
                    &lane_controls);
  }
  std::vector<float> lane_lights;
  SetLaneLights(lane_ptrs, tl_states, &lane_lights);

  return VehicleTNTLanesFeature{
      .lane_centers = std::move(lane_centers),
      .lane_segments = std::move(lane_segments),
      .lane_controls = std::move(lane_controls),
      .lane_lights = std::move(lane_lights),
  };
}

// TODO(shenlong): dedupe targets which are close to each other.
VehicleTNTTargets ExtractVehicleTNTTargets(
    const SemanticMapManager &semantic_map_manager,
    mapping::ElementId intersection_id,
    const std::map<mapping::ElementId, std::vector<Vec2d>>
        &intersection_exits_map,
    Vec2d ref_pos, double rot_rad) {
  const auto *exits_ptr = FindOrNull(intersection_exits_map, intersection_id);
  QCHECK_NOTNULL(exits_ptr);
  std::vector<float> targets;
  VLOG(2) << "num of targets: " << exits_ptr->size();
  for (const auto &exit : *exits_ptr) {
    VLOG(2) << "targets: " << exit.DebugString();
    const auto point = (exit - ref_pos).Rotate(rot_rad);
    targets.push_back(point.x());
    targets.push_back(point.y());
  }
  return VehicleTNTTargets{
      .targets = std::move(targets),
  };
}

std::optional<Vec2d> FindClosestPointWithDistLimit(
    absl::Span<const float> targets, const Vec2d &exit, double dist_limit) {
  Vec2d min_pt;
  double min_dist_sqr = std::numeric_limits<double>::max();
  for (int i = 0; i < targets.size(); i += 2) {
    Vec2d target(targets[i], targets[i + 1]);
    const double dist_sqr = (target - exit).Sqr();
    if (dist_sqr < min_dist_sqr) {
      min_pt = std::move(target);
      min_dist_sqr = dist_sqr;
    }
  }
  if (min_dist_sqr > dist_limit * dist_limit) {
    return std::nullopt;
  }
  return min_pt;
}

VehicleTNTDumpedFeatureProto::LabelsProto ExtractLabels(
    absl::Span<const PredictedTrajectoryPointProto> traj) {
  VehicleTNTDumpedFeatureProto::LabelsProto res;
  if (traj.size() == 0) {
    return res;
  }
  if (traj.front().v() < kStaticEpsilon) {
    res.set_is_static(true);
  } else {
    res.set_is_static(false);
  }
  res.set_speed_diff(traj.back().v() - traj.front().v());
  res.set_turn_angle(
      NormalizeAngle(traj.back().theta() - traj.front().theta()));
  return res;
}

}  // namespace

VehicleTNTFeature ExtractVehicleTNTFeature(
    const ResampledObjectsHistory &objects_history,
    const SemanticMapManager &semantic_map_manager,
    const std::vector<ObjectProto> &predicted_object_history,
    const qcraft::mapping::IntersectionInfo &intersection,
    const std::map<mapping::ElementId, std::vector<Vec2d>>
        &intersection_exits_map,
    const TrafficLightManager::TLStateHashMap &tl_states) {
  VehicleTNTFeature tnt_feature;
  const auto &av_cur_point = predicted_object_history.back();
  tnt_feature.ref_pose = Vec2d(av_cur_point.pos());
  const float heading = av_cur_point.yaw();
  const Box2d region_box = GetRegionBox(tnt_feature.ref_pose, heading);
  ResampledObjectsHistory objects_in_box2d = GetResampledObjectsInBox2d(
      objects_history, region_box, av_cur_point.id(), kMaxTntContextObjectNum);
  tnt_feature.rot_rad = -heading;
  tnt_feature.actors_feature = ExtractVehicleTNTActorsFeature(
      tnt_feature.ref_pose, tnt_feature.rot_rad, objects_in_box2d,
      predicted_object_history);
  const std::vector<const mapping::LaneInfo *> lane_ptrs = GetLanesInBox2d(
      semantic_map_manager, region_box, kVehicleTNTConfig.max_lane_num);
  tnt_feature.lanes_feature = ExtractVehicleTNTLanesFeature(
      tnt_feature.ref_pose, tnt_feature.rot_rad, lane_ptrs, tl_states);
  tnt_feature.ego_id = predicted_object_history.back().id();
  tnt_feature.targets = ExtractVehicleTNTTargets(
      semantic_map_manager, intersection.id, intersection_exits_map,
      tnt_feature.ref_pose, tnt_feature.rot_rad);
  return tnt_feature;
}

void ToVehicleTNTDumpedFeatureProto(
    const VehicleTNTFeature &tnt_feature,
    VehicleTNTDumpedFeatureProto *const feature_proto) {
  auto *mutable_objects_feature = feature_proto->mutable_objects_feature();
  const auto &actors_feature = tnt_feature.actors_feature;
  *mutable_objects_feature->mutable_ego_traj() = {
      actors_feature.ego_traj.begin(), actors_feature.ego_traj.end()};
  *mutable_objects_feature->mutable_ego_speeds() = {
      actors_feature.ego_speeds.begin(), actors_feature.ego_speeds.end()};
  *mutable_objects_feature->mutable_ego_headings() = {
      actors_feature.ego_headings.begin(), actors_feature.ego_headings.end()};
  *mutable_objects_feature->mutable_trajs() = {actors_feature.trajs.begin(),
                                               actors_feature.trajs.end()};
  *mutable_objects_feature->mutable_speeds() = {actors_feature.speeds.begin(),
                                                actors_feature.speeds.end()};
  *mutable_objects_feature->mutable_headings() = {
      actors_feature.headings.begin(), actors_feature.headings.end()};
  *mutable_objects_feature->mutable_cur_poses() = {
      actors_feature.cur_poses.begin(), actors_feature.cur_poses.end()};
  *mutable_objects_feature->mutable_types() = {actors_feature.types.begin(),
                                               actors_feature.types.end()};

  auto *mutable_lanes_feature = feature_proto->mutable_lanes_feature();
  const auto &lanes_feature = tnt_feature.lanes_feature;
  *mutable_lanes_feature->mutable_lane_centers() = {
      lanes_feature.lane_centers.begin(), lanes_feature.lane_centers.end()};
  *mutable_lanes_feature->mutable_lane_segments() = {
      lanes_feature.lane_segments.begin(), lanes_feature.lane_segments.end()};
  *mutable_lanes_feature->mutable_lane_controls() = {
      lanes_feature.lane_controls.begin(), lanes_feature.lane_controls.end()};
  *mutable_lanes_feature->mutable_lane_lights() = {
      lanes_feature.lane_lights.begin(), lanes_feature.lane_lights.end()};
  auto *mutable_targets = feature_proto->mutable_targets();
  const auto &targets = tnt_feature.targets;
  *mutable_targets->mutable_targets() = {targets.targets.begin(),
                                         targets.targets.end()};
}

std::map<std::string, Vec2d> SelectObjectsWithValidGroundTruth(
    const SemanticMapManager &semantic_map_manager,
    const std::map<ObjectIDType, const ObjectPredictionProto *> &log_pred_map,
    std::map<std::string, const mapping::IntersectionInfo *>
        *object_intersection_map) {
  std::map<std::string, Vec2d> obj_exits;
  for (auto it = object_intersection_map->begin(), next = it;
       it != object_intersection_map->end(); it = next) {
    ++next;

    const auto *log_pred = FindPtrOrNull(log_pred_map, it->first);
    if (log_pred == nullptr || log_pred->trajectories_size() == 0) {
      object_intersection_map->erase(it);
      continue;
    }
    const auto &traj_points = log_pred->trajectories(0).points();
    if (traj_points.empty()) {
      object_intersection_map->erase(it);
      continue;
    }

    const auto &polygon = it->second->polygon_smooth;
    bool found_first_in = false;
    bool found_last_in = false;
    Vec2d exit_point;
    for (int index = 0, n = traj_points.size(); index < n; ++index) {
      const bool in = polygon.IsPointIn(Vec2d(traj_points[index].pos()));
      if (!found_first_in && in) {
        found_first_in = true;
      } else if (found_first_in && !in) {
        exit_point = Vec2d(traj_points[index].pos());
        found_last_in = true;
        break;
      }
    }
    if (!found_first_in || !found_last_in) {
      object_intersection_map->erase(it);
    }
    obj_exits[it->first] = exit_point;
  }
  return obj_exits;
}

std::optional<std::pair<VehicleTNTDumpedFeatureProto::GroundTruthProto,
                        VehicleTNTDumpedFeatureProto::LabelsProto>>
ExtractTntGroundTruth(const Vec2d &ref_pos, double rot_rad,
                      const ObjectPredictionProto &object_prediction_proto,
                      absl::Span<const float> targets, const double ts,
                      const std::map<std::string, Vec2d> &obj_exits_map) {
  std::vector<PredictedTrajectoryPointProto> raw_traj;
  constexpr double kDesirableDuration = kPredictionDuration + 1.0;
  const auto oracle_ts =
      object_prediction_proto.perception_object().timestamp();
  const double t_diff = ts - oracle_ts;
  const auto &oracle_traj = object_prediction_proto.trajectories().Get(0);
  // Force initial point to 0,0, skip those ground truth that is out-dated
  auto init_pt = oracle_traj.points(0);
  init_pt.mutable_pos()->set_x(0.0);
  init_pt.mutable_pos()->set_y(0.0);
  init_pt.set_theta(init_pt.theta() + rot_rad);
  init_pt.set_t(t_diff);
  raw_traj.push_back(init_pt);
  for (int i = 0; i < oracle_traj.points_size(); ++i) {
    const auto &raw_pt = oracle_traj.points(i);
    // Skip ground truth that is older than current time.
    if (raw_pt.t() < t_diff + kPredictionTimeStep) {
      continue;
    }
    if (raw_pt.t() > kDesirableDuration) {
      break;
    }
    auto pt = raw_pt;
    auto cur_pos = (Vec2d(pt.pos()) - ref_pos).Rotate(rot_rad);
    pt.mutable_pos()->set_x(cur_pos.x());
    pt.mutable_pos()->set_y(cur_pos.y());
    pt.set_theta(pt.theta() + rot_rad);
    raw_traj.push_back(std::move(pt));
    if (raw_pt.v() < kStaticEpsilon) {
      break;
    }
  }
  if (FLAGS_smooth_tnt_gt_trajectory) {
    raw_traj = SmoothPredictedTrajectory(raw_traj);
  }

  // Cutoff predicted trajectory when object fully stops.
  double cur_t = raw_traj.back().t();
  while (raw_traj.back().v() < kStaticEpsilon &&
         raw_traj.back().t() < kDesirableDuration) {
    auto last_pt = raw_traj.back();
    cur_t += kPredictionTimeStep;
    last_pt.set_t(cur_t);
    raw_traj.push_back(std::move(last_pt));
  }

  auto new_traj = AlignPredictedTrajectoryPoints(
      raw_traj, object_prediction_proto.perception_object().timestamp(), ts,
      kPredictionDuration, kPredictionTimeStep);

  VehicleTNTDumpedFeatureProto::LabelsProto labels = ExtractLabels(new_traj);

  VehicleTNTDumpedFeatureProto::GroundTruthProto gt_proto;
  QCHECK_GE(new_traj.size(), 2);
  *gt_proto.mutable_gt_traj()->mutable_gt_points() = {new_traj.begin() + 1,
                                                      new_traj.end()};
  gt_proto.mutable_gt_traj()->set_object_id(object_prediction_proto.id());
  const auto *exit_ptr =
      FindOrNull(obj_exits_map, object_prediction_proto.id());
  QCHECK_NOTNULL(exit_ptr);
  const auto relative_exit = (*exit_ptr - ref_pos).Rotate(rot_rad);
  auto gt_target_or =
      FindClosestPointWithDistLimit(targets, relative_exit, kMaxDistToTarget);
  if (!gt_target_or.has_value()) {
    return std::nullopt;
  }
  gt_proto.add_target(gt_target_or->x());
  gt_proto.add_target(gt_target_or->y());
  return std::make_pair(gt_proto, labels);
}
}  // namespace prediction
}  // namespace qcraft
