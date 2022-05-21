#include "onboard/prediction/prediction_util.h"

#include <algorithm>
#include <limits>
#include <map>
#include <vector>

#include "absl/strings/str_split.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/util/lane_path_finder.h"
namespace qcraft {
namespace prediction {

namespace {
constexpr double kCurbCollisionThreshold = 0.2;           // m.
constexpr double kTrafficLightCutOffNominalBrake = -3.0;  // m/s^2.
constexpr double kMinSpeedToEnableAlignment = 2.0;        // m/s

// Guess object type.
constexpr double kPedMaxLen = 1.0;
constexpr double kBikeMaxLen = 2.0;
constexpr double kBikeMaxSpeed = 10.0;
constexpr double kPedMaxSpeed = 3.0;

// Used for cutting off prediction traj by curb
constexpr double kCurbCanPassMaxHeight = 0.1;  // m.

bool IsStationaryType(ObjectType type) {
  switch (type) {
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return true;
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
      return false;
  }
}

bool DetermineLightBlockage(
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    const mapping::ElementId tl_id) {
  return (tl_state_map.count(tl_id) &&
          tl_state_map.at(tl_id).color() == TrafficLightColor::TL_RED);
}
bool BlockLaneByTrafficLight(
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    const SemanticMapManager &semantic_map_manager,
    const mapping::ElementId lane_id) {
  const mapping::LaneProto &junction_lane =
      semantic_map_manager.FindLaneByIdOrDie(lane_id);
  for (const auto tl_id :
       junction_lane.startpoint_associated_traffic_lights()) {
    if (DetermineLightBlockage(tl_state_map, tl_id)) {
      return true;
    }
  }
  return false;
}
bool BlockObjectByTrafficLights(
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    const SemanticMapManager &semantic_map_manager,
    mapping::ElementId lane_id) {
  if (BlockLaneByTrafficLight(tl_state_map, semantic_map_manager, lane_id)) {
    return true;
  }
  const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(lane_id);
  const auto *next_idxes_ptr = &lane_info.outgoing_lane_indices;
  if (next_idxes_ptr->empty()) {
    return false;
  }
  for (const auto &next_idx : *next_idxes_ptr) {
    const auto next_id = semantic_map_manager.lane_info()[next_idx].id;
    if (BlockLaneByTrafficLight(tl_state_map, semantic_map_manager, next_id)) {
      return true;
    }
  }
  return false;
}

void CutoffTrajByLightBlockageIndex(int light_blockage_index,
                                    double nominal_brake,
                                    PredictedTrajectory *traj) {
  auto &mutable_points = *traj->mutable_points();
  QCHECK_GT(mutable_points.size(), 0);
  int cut_off_idx = mutable_points.size();
  const double inv_nominal_brake = 1.0 / nominal_brake;
  const double dist2light = mutable_points[light_blockage_index].s();
  const double nominal_brake_dis = std::pow(mutable_points.front().v(), 2) *
                                   0.5 * std::fabs(inv_nominal_brake);
  if (nominal_brake_dis <= dist2light) {
    cut_off_idx = std::min(light_blockage_index, cut_off_idx);
  } else {
    cut_off_idx = std::min(
        std::max(FloorToInt(kEmergencyGuardHorizon / kPredictionTimeStep),
                 light_blockage_index),
        cut_off_idx);
  }
  // Patch up to 10s
  const int final_len =
      std::max(FloorToInt(kPredictionDuration / kPredictionTimeStep),
               light_blockage_index);
  if (cut_off_idx < mutable_points.size()) {
    TruncateTrajectoryBySize(cut_off_idx, &mutable_points);
    const auto &last_pt = mutable_points.back();
    for (int i = cut_off_idx; i < final_len; ++i) {
      auto new_pt = last_pt;
      new_pt.set_t(last_pt.t() + (i - cut_off_idx + 1) * kPredictionTimeStep);
      new_pt.set_v(0.0);
      mutable_points.push_back(std::move(new_pt));
    }
  }
  traj->set_annotation(
      absl::StrCat(traj->annotation(), ", light cutoff at ", cut_off_idx));
}
std::vector<Vec2d> TransformPoints(
    const ::google::protobuf::RepeatedPtrField<Vec2dProto> &proto_points,
    Vec2d origin_center, Vec2d shift, double cos_angle, double sin_angle) {
  std::vector<Vec2d> points;
  points.reserve(proto_points.size());
  const Vec2d new_center = origin_center + shift;
  for (const auto &p : proto_points) {
    const Vec2d point = Vec2dFromProto(p) - origin_center;
    points.emplace_back(point.Rotate(cos_angle, sin_angle) + new_center);
  }
  return points;
}

// Determine object type by heuristic, only for unknown objs.
ObjectType DetermineTypeByHeuristicForUnknown(double len, double cur_v) {
  if (cur_v > kBikeMaxSpeed) {
    return OT_VEHICLE;
  }
  if (len > kBikeMaxLen) {
    return OT_VEHICLE;
  }
  if (len > kPedMaxLen || cur_v > kPedMaxSpeed) {
    return OT_CYCLIST;
  }
  return OT_PEDESTRIAN;
}
}  // namespace

absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto *object) {
  const double time_diff = current_time - object->timestamp();
  constexpr double kMaxTimeDiff = 5.0;  // Seconds.
  if (time_diff < 0.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Provided time[%f] is less than perception's time[%f]",
                        current_time, object->timestamp()));
  } else if (time_diff > kMaxTimeDiff) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Current time[%f] is too different with perception "
                        "time[%f], larger than threshold[%f]",
                        current_time, object->timestamp(), kMaxTimeDiff));
  }

  // set timestamp.
  object->set_timestamp(current_time);
  // NOTE(boqian): decide which yaw to use according to speed.
  // Actually should be done by perception...
  if (object->moving_state() == ObjectProto::MS_STATIC ||
      Vec2dFromProto(object->vel()).squaredNorm() <
          Sqr(kMinSpeedToEnableAlignment)) {
    object->set_yaw(object->bounding_box().heading());
    return absl::OkStatus();
  }

  const Vec2d vel = Vec2dFromProto(object->vel());
  const Vec2d pos = Vec2dFromProto(object->pos());
  const Vec2d accel = Vec2dFromProto(object->accel());

  // Update values assuming const acceleration.
  const Vec2d pos_shift = (vel + 0.5 * accel * time_diff) * time_diff;
  const double yaw_diff = NormalizeAngle(object->yaw_rate() * time_diff);
  const double new_yaw = NormalizeAngle(yaw_diff + object->yaw());
  const Vec2d new_vel = vel + accel * time_diff;
  const Vec2d new_pos = pos_shift + pos;

  // Set pos.
  new_pos.ToProto(object->mutable_pos());

  // Set velocity.
  new_vel.ToProto(object->mutable_vel());

  // Set contour.
  const Vec2d rotation = Vec2d::FastUnitFromAngle(yaw_diff);
  const auto contour_points =
      TransformPoints(object->contour(), /*origin_center=*/pos,
                      /*shift=*/pos_shift, rotation.x(), rotation.y());
  object->clear_contour();
  for (const auto &pt : contour_points) {
    pt.ToProto(object->add_contour());
  }

  // Set bounding box. Angle normalization is done inside transform function.
  object->mutable_bounding_box()->set_x(new_pos.x());
  object->mutable_bounding_box()->set_y(new_pos.y());
  object->mutable_bounding_box()->set_heading(new_yaw);

  // Set yaw.
  object->set_yaw(NormalizeAngle(object->yaw() + yaw_diff));

  return absl::OkStatus();
}

ObjectProto LerpObjectProto(const ObjectProto &a, const ObjectProto &b,
                            double alpha) {
  ObjectProto object = b;
  const double lerped_ts = Lerp(a.timestamp(), b.timestamp(), alpha);
  const Vec2d lerped_pos =
      Lerp(Vec2dFromProto(a.pos()), Vec2dFromProto(b.pos()), alpha);
  const Vec2d lerped_vel =
      Lerp(Vec2dFromProto(a.vel()), Vec2dFromProto(b.vel()), alpha);
  const Vec2d lerped_accel =
      Lerp(Vec2dFromProto(a.accel()), Vec2dFromProto(b.accel()), alpha);
  const double lerped_yaw = NormalizeAngle(LerpAngle(a.yaw(), b.yaw(), alpha));
  const double lerped_yaw_rate =
      NormalizeAngle(LerpAngle(a.yaw_rate(), b.yaw_rate(), alpha));
  object.set_timestamp(lerped_ts);
  lerped_pos.ToProto(object.mutable_pos());
  lerped_vel.ToProto(object.mutable_vel());
  lerped_accel.ToProto(object.mutable_accel());
  object.set_yaw(lerped_yaw);
  object.set_yaw_rate(lerped_yaw_rate);
  return object;
}

ObjectType GuessType(const ObjectHistory &obj) {
  const auto hist_or = obj.GetHistory();
  QCHECK_OK(hist_or.status());
  if (obj.type() == OT_PEDESTRIAN) {
    return OT_PEDESTRIAN;
  }
  if (obj.type() == OT_CYCLIST || obj.type() == OT_MOTORCYCLIST) {
    return OT_CYCLIST;
  }
  if (obj.type() == OT_VEHICLE) {
    return OT_VEHICLE;
  }
  if (obj.type() == OT_UNKNOWN_MOVABLE) {
    return DetermineTypeByHeuristicForUnknown(hist_or->bounding_box().length(),
                                              hist_or->v());
  }
  return obj.type();
}

PredictedTrajectoryPointProto LerpPredictedTrajectoryPointProto(
    const PredictedTrajectoryPointProto &a,
    const PredictedTrajectoryPointProto &b, double alpha) {
  PredictedTrajectoryPointProto pt;
  const Vec2d lerped_pos =
      Lerp(Vec2dFromProto(a.pos()), Vec2dFromProto(b.pos()), alpha);
  lerped_pos.ToProto(pt.mutable_pos());
  pt.set_s(Lerp(a.s(), b.s(), alpha));
  pt.set_theta(NormalizeAngle(LerpAngle(a.theta(), b.theta(), alpha)));
  pt.set_kappa(Lerp(a.kappa(), b.kappa(), alpha));
  pt.set_t(Lerp(a.t(), b.t(), alpha));
  pt.set_v(Lerp(a.v(), b.v(), alpha));
  pt.set_a(Lerp(a.a(), b.a(), alpha));
  return pt;
}

std::vector<ObjectProto> ResampleObjectProtos(
    absl::Span<const ObjectProto> objs, double current_ts, double time_step,
    int max_steps) {
  if (objs.empty()) {
    return {};
  }
  if (objs.size() == 1) {
    ObjectProto object = objs.back();
    const auto align_status = AlignPerceptionObjectTime(current_ts, &object);
    return {object};
  }
  std::vector<double> vect;
  std::vector<ObjectProto> vec_objs;
  vect.reserve(objs.size());
  vec_objs.reserve(objs.size());
  for (const auto &obj : objs) {
    vect.push_back(obj.timestamp() - current_ts);
    vec_objs.push_back(obj);
  }
  class ObjectProtoLerper {
   public:
    ObjectProto operator()(const ObjectProto &a, const ObjectProto &b,
                           double alpha) const {
      return LerpObjectProto(a, b, alpha);
    }
  };
  PiecewiseLinearFunction<ObjectProto, double, ObjectProtoLerper>
      object_proto_plf(vect, vec_objs);
  std::vector<ObjectProto> resampled_objects;
  resampled_objects.reserve(max_steps);
  for (int i = 0; i < max_steps; ++i) {
    const double ts = i * time_step;
    if (ts > vect.back()) {
      break;
    }
    auto resampled_obj = object_proto_plf.EvaluateWithExtrapolation(ts);
    resampled_objects.push_back(std::move(resampled_obj));
  }
  return resampled_objects;
}

PredictedTrajectoryPointProto CreateTrajectoryPointFromStationaryObject(
    const ObjectProto &object) {
  PredictedTrajectoryPointProto point;
  *point.mutable_pos() = object.pos();
  point.set_s(0.0);
  point.set_theta(object.yaw());
  point.set_kappa(0.0);

  point.set_t(0.0);
  point.set_v(0.0);
  point.set_a(0.0);

  return point;
}

double PointToSegmentDistance(const Vec2d query_point, const Vec2d p1,
                              const Vec2d p2, Vec2d *closest_point) {
  const Vec2d segment = p2 - p1;
  const double l_sqr = segment.squaredNorm();
  const Vec2d query_segment = query_point - p1;
  const double projection = query_segment.dot(segment) / l_sqr;
  double spatial_dis = std::numeric_limits<double>::infinity();
  if (projection < 0.0) {
    spatial_dis = p1.DistanceTo(query_point);
    if (closest_point != nullptr) *closest_point = p1;
  } else if (projection > 1.0) {
    spatial_dis = p2.DistanceTo(query_point);
    if (closest_point != nullptr) *closest_point = p2;
  } else {
    spatial_dis = std::abs(query_segment.CrossProd(segment.normalized()));
    if (closest_point != nullptr) *closest_point = Lerp(p1, p2, projection);
  }
  return spatial_dis;
}

Box2d BuildObjectBoundingBox(const ObjectProto &object) {
  if (object.has_bounding_box()) {
    return Box2d(object.bounding_box());
  }

  const auto polygon_or =
      Polygon2d::FromPoints(object.contour(), /*is_convex=*/true);
  QCHECK(polygon_or.has_value())
      << object.ShortDebugString() << " has no contour.";

  return polygon_or->BoundingBoxWithHeading(object.yaw());
}

std::vector<Vec2d> GetObjectContour(const ObjectProto &object_proto) {
  std::vector<Vec2d> contour;
  const auto &contour_proto = object_proto.contour();
  if (!contour_proto.empty()) {
    contour.reserve(contour_proto.size());
    for (auto &vec_2d_proto : contour_proto) {
      contour.emplace_back(vec_2d_proto.x(), vec_2d_proto.y());
    }
  } else {
    QLOG(WARNING) << absl::StrFormat("object %s does not have a contour",
                                     object_proto.id());
  }
  return contour;
}

Vec2d GetObjectFrontCenter(const ObjectProto &object_proto) {
  if (object_proto.has_bounding_box()) {
    const auto &bb = object_proto.bounding_box();
    const Box2d box({bb.x(), bb.y()}, bb.heading(), bb.length(), bb.width());
    const auto corners = box.GetCornersCounterClockwise();
    return (corners[0] + corners[3]) * 0.5;
  } else if (object_proto.has_yaw()) {
    return Vec2dFromProto(object_proto.pos()) +
           Vec2d::FastUnitFromAngle(object_proto.yaw());
  } else {
    return Vec2dFromProto(object_proto.pos());
  }
}

double GetObjectRawHeading(const ObjectProto &object_proto) {
  return object_proto.yaw();
}

// TODO(shenlong) remove this function because we will have multi
// object_prediction instead of a single one for a object.
void RemoveLowProbabilityPrediction(const double prob_thresh,
                                    ObjectPrediction *const object_prediction) {
  CHECK(prob_thresh >= 0.0 && prob_thresh < 1.0);
  const double max_prob =
      std::accumulate(object_prediction->trajectories().begin(),
                      object_prediction->trajectories().end(), 0.0,
                      [](const double max, const auto &traj) {
                        return std::max(max, traj.probability());
                      });
  if (max_prob < prob_thresh) {
    VLOG(2) << absl::StrFormat(
        "All %d traj(s) of %s is(are) below threshold %5.3f (max is %5.3g). I "
        "will do nothing!",
        object_prediction->trajectories().size(), object_prediction->id(),
        prob_thresh, max_prob);
    return;
  }
  auto *trajectories = object_prediction->mutable_trajectories();
  const double prob_sum =
      std::accumulate(object_prediction->trajectories().begin(),
                      object_prediction->trajectories().end(), 0.0,
                      [](const double sum, const auto &traj) {
                        return sum + traj.probability();
                      });
  trajectories->erase(std::remove_if(trajectories->begin(), trajectories->end(),
                                     [prob_thresh](PredictedTrajectory &traj) {
                                       return traj.probability() < prob_thresh;
                                     }),
                      trajectories->end());
  const double prob_sum_after_removing_low_prob =
      std::accumulate(object_prediction->trajectories().begin(),
                      object_prediction->trajectories().end(), 0.0,
                      [](const double sum, const auto &traj) {
                        return sum + traj.probability();
                      });
  const double scale_factor = prob_sum / prob_sum_after_removing_low_prob;
  for (auto &traj : *trajectories) {
    traj.set_probability(traj.probability() * scale_factor);
  }
}

void FillTrajectoryFromRawPoints(const ObjectProto &object_proto,
                                 const std::vector<Vec2d> &raw_points,
                                 const PredictionType traj_type,
                                 const double prediction_step,
                                 PredictedTrajectory *traj) {
  const double inv_pred_step = 1.0 / prediction_step;
  constexpr double kMinThetaValidVel = 0.1;  // m/s
  traj->set_type(traj_type);
  double s = 0.0;
  const Vec2d current_a = Vec2dFromProto(object_proto.accel());
  const Vec2d heading_tangent = Vec2d::FastUnitFromAngle(object_proto.yaw());
  const double accel = current_a.dot(heading_tangent);
  for (int i = 0; i < raw_points.size(); ++i) {
    const double t = i * prediction_step;
    const Vec2d p0 = raw_points[i];
    auto &point = traj->mutable_points()->emplace_back();
    point.set_t(t);
    point.set_pos(p0);
    point.set_s(s);

    if (i + 1 < raw_points.size()) {
      const Vec2d p1 = raw_points[i + 1];
      const Vec2d vec = p1 - p0;
      point.set_v(vec.norm() * inv_pred_step);
      if (i > 0) {
        const auto &prev_point = traj->points()[i - 1];
        point.set_theta(point.v() > kMinThetaValidVel ? vec.FastAngle()
                                                      : prev_point.theta());
        point.set_a((point.v() - prev_point.v()) * inv_pred_step);
        const double v_sum = point.v() + prev_point.v();
        point.set_kappa(v_sum > kMinThetaValidVel
                            ? (point.theta() - prev_point.theta()) /
                                  (v_sum * 0.5 * prediction_step)
                            : 0.0);
      } else {
        point.set_theta(object_proto.yaw());
        point.set_a(accel);
        point.set_kappa(0.0);
      }
      s += vec.norm();
    } else {
      const auto &prev_point = traj->points()[i - 1];
      point.set_v(prev_point.v());
      point.set_theta(prev_point.theta());
      point.set_a(prev_point.a());
      point.set_kappa(prev_point.kappa());
    }
  }
}

void GetMultiTimerStats(const ScopedMultiTimer &timer,
                        MultiTimerStatsProto *stats_proto) {
  stats_proto->Clear();
  absl::Time prev_time = timer.start();
  const double total_duration = absl::ToDoubleSeconds(timer.total_duration());
  for (const auto &timer_mark : timer.marks()) {
    MultiTimerStatsProto::MarkProto *mark = stats_proto->add_marks();
    const double duration = absl::ToDoubleSeconds(timer_mark.time - prev_time);
    mark->set_message(timer_mark.msg);
    mark->set_duration(duration);
    mark->set_ratio_to_total(duration / total_duration);
    prev_time = timer_mark.time;
  }
  stats_proto->set_total_duration(total_duration);
}

void PrintMultiTimerReportStat(const MultiTimerStatsProto &report_proto) {
  std::map<std::string, std::pair<int, double>> accumul_time_map;
  for (const auto &marker : report_proto.marks()) {
    if (accumul_time_map.find(marker.message()) == accumul_time_map.end()) {
      accumul_time_map.emplace(marker.message(),
                               std::make_pair(1, marker.duration()));
    } else {
      accumul_time_map[marker.message()].first++;
      accumul_time_map[marker.message()].second += marker.duration();
    }
  }
  using marker_pair = std::pair<std::string, std::pair<int, double>>;
  auto accumul_time = std::vector<marker_pair>(accumul_time_map.begin(),
                                               accumul_time_map.end());
  std::sort(accumul_time.begin(), accumul_time.end(),
            [](const marker_pair &a, const marker_pair &b) {
              return a.second.second > b.second.second;
            });

  const int name_width =
      std::max_element(accumul_time.begin(), accumul_time.end(),
                       [](const marker_pair &a, const marker_pair &b) {
                         return a.first.size() < b.first.size();
                       })
          ->first.size();
  QLOG(INFO) << absl::StrFormat("\trank\t%-*s\tcount\tduration (ms)\tratio",
                                name_width, "name");
  for (int i = 0; i < accumul_time.size(); ++i) {
    QLOG(INFO) << absl::StrFormat(
        "\t%d\t%-*s\t%d\t%6.3f\t\t%6.2f%%", i, name_width,
        accumul_time[i].first, accumul_time[i].second.first,
        1e3 * accumul_time[i].second.second,
        100 * accumul_time[i].second.second / report_proto.total_duration());
  }
  QLOG(INFO) << absl::StrFormat("\t\t%-*s\t \t%6.3f\t\t100.00%%", name_width,
                                "total", 1e3 * report_proto.total_duration());
}

absl::StatusOr<ObjectPredictionProto> InstantPredictionForNewObject(
    const ObjectProto &object) {
  ObjectPredictionProto proto;
  if (!object.has_id() || !object.has_pos() || !object.has_vel()) {
    return absl::NotFoundError(
        absl::StrFormat("Input Object proto missing one of the fields: "
                        "id[missing=%d], pos[missing=%d], val[missing=%d]",
                        object.has_id(), object.has_pos(), object.has_vel()));
  }

  proto.set_id(object.id());
  *proto.mutable_perception_object() = object;

  if (IsStationaryType(object.type())) {
    auto *traj = proto.add_trajectories();
    traj->set_probability(1.0);
    traj->set_priority(ObjectPredictionPriority::OPP_P3);
    traj->set_type(PredictionType::PT_STATIONARY);
    *traj->add_points() = CreateTrajectoryPointFromStationaryObject(object);
    return proto;
  }

  const double linear_a = 0.0;
  const double yaw_rate = object.yaw_rate();
  double s = 0.0;  // m.
  Vec2d pos = Vec2dFromProto(object.pos());
  const Vec2d initial_v = Vec2dFromProto(object.vel());
  double heading = NormalizeAngle2D(initial_v);
  double speed = initial_v.norm();

  const int prediction_horizon =
      FloorToInt(FLAGS_prediction_duration / kPredictionTimeStep);
  auto *traj_ptr = proto.add_trajectories();
  traj_ptr->mutable_points()->Reserve(prediction_horizon);
  for (int j = 0; j < prediction_horizon; ++j) {
    PredictedTrajectoryPoint point;
    const double t = j * kPredictionTimeStep;
    point.set_t(t);
    point.set_pos(pos);
    const Vec2d v = speed * Vec2d::FastUnitFromAngle(heading);
    pos += v * kPredictionTimeStep;
    point.set_s(s);
    s += speed * kPredictionTimeStep;
    point.set_theta(NormalizeAngle(heading));
    point.set_kappa(speed > 0.0 ? yaw_rate / speed : 0.0);
    point.set_a(linear_a);
    point.set_v(speed);
    heading += yaw_rate * kPredictionTimeStep;
    speed += linear_a * kPredictionTimeStep;
    const auto traj_point_proto_ptr = traj_ptr->add_points();
    point.ToProto(traj_point_proto_ptr);
  }
  traj_ptr->set_probability(1.0);
  if (proto.trajectories_size() == 1) {
    return proto;
  } else {
    return absl::UnknownError(absl::StrFormat(
        "Unknown problem causing result to have trajectories_size != 1."));
  }
}

void Visualize(const ObjectPredictionProto &traj, const std::string &canvas) {
  auto traj_ptr = traj.trajectories();
  vis::Canvas &canvas_pred = vantage_client_man::GetCanvas(canvas);
  const auto current_traj = traj_ptr[0];
  const auto size = current_traj.points_size();
  std::vector<Vec3d> traj_points;
  for (int i = 0; i < size; ++i) {
    const auto current_pt = current_traj.points()[i];
    Vec3d newPt(current_pt.pos().x(), current_pt.pos().y(), 0);
    traj_points.push_back(newPt);
  }
  canvas_pred.DrawPoints(traj_points, vis::Color::kYellow, 5);
}

const double kMinSteering = 1e-4;
const int kMinPoints = 5;
const int kMaxLocalValueNum = 3;
bool IsTrajTwisted(const PredictedTrajectory &traj) {
  const int points_num = traj.points().size();
  if (points_num < 3) return false;
  double accum_steering = 0.0;
  constexpr double kSteeringThreshold = d2r(270.0);
  int local_value_num = 0;
  double prev_steering = 0.0;
  for (int i = 1; i < points_num; ++i) {
    const auto &prev_pt = traj.points()[i - 1];
    const auto &cur_pt = traj.points()[i];
    double cur_steering = NormalizeAngle(cur_pt.theta() - prev_pt.theta());
    if (std::abs(cur_steering) < kMinSteering) {
      cur_steering = 0.0;
    }
    if ((i > kMinPoints) && (i < (points_num - kMinPoints)) &&
        prev_steering != 0.0 && cur_steering != 0.0 &&
        std::signbit(prev_steering) != std::signbit(cur_steering)) {
      local_value_num++;
    }
    prev_steering = cur_steering;
    if (local_value_num > kMaxLocalValueNum) {
      return true;
    }
    accum_steering += std::abs(cur_steering);
    if (accum_steering > kSteeringThreshold) return true;
  }
  return false;
}

std::vector<TrajCollisionInfo> FindTrajCurbCollisionIndex(
    absl::Span<const PredictedTrajectoryPoint> traj,
    const SemanticMapManager &semantic_map_manager, const double object_length,
    qcraft::ObjectType object_type) {
  std::vector<TrajCollisionInfo> collision_infos;
  // TODO(lidong): Get accurate object level.
  const auto level_id = semantic_map_manager.GetLevel();
  for (int i = 1; i < traj.size(); ++i) {
    const Vec2d center = traj[i].pos();
    const double heading = traj[i].theta();
    const double signed_v = traj[i].v();
    // Calculate object augment front/back center for fast rough collision
    // check. Note: the heading of the vehicle is always oriented to the front
    // center.
    const Vec2d check_pos =
        center + 0.5 * std::copysign(object_length, signed_v) *
                     Vec2d::FastUnitFromAngle(heading);
    const Vec2d &prev_pos = traj[i - 1].pos();
    const auto dis_seg = Segment2d(prev_pos, check_pos);
    Segment2d nda_segment;
    std::string boundary_info;
    if (!semantic_map_manager.GetNearestNamedImpassableBoundaryAtLevel(
            level_id, check_pos, &nda_segment, &boundary_info)) {
      continue;
    }
    const double seg_dis = dis_seg.DistanceTo(nda_segment);
    if (seg_dis < kCurbCollisionThreshold) {
      bool can_cross_curb = false;
      // NOTE(yinbao): find curb height, cannot pass if the height is not 0 or
      // ublabeled. OT_PEDESTRIAN can pass all curb.
      if (object_type == qcraft::ObjectType::OT_PEDESTRIAN) {
        can_cross_curb = true;
      } else {
        // NOTE(yinbao): boundary_info is a string, [Type]|[Id]|[idxInSegment].
        std::vector<std::string> boundary_info_ =
            absl::StrSplit(boundary_info, "|");
        if (boundary_info_.size() >= 2 && boundary_info_[0] == "CURB") {
          const auto *lane_boundary_proto =
              semantic_map_manager
                  .FindLaneBoundaryByIdOrDie(std::stoi(boundary_info_[1]))
                  .proto;
          if (lane_boundary_proto->has_height() &&
              lane_boundary_proto->height() < kCurbCanPassMaxHeight) {
            can_cross_curb = true;
          }
        }
      }

      VLOG(3) << absl::StrFormat("traj hits curb at %d", i);
      collision_infos.push_back({i, can_cross_curb});
    }
  }
  return collision_infos;
}

std::optional<int> CutoffTrajByCurb(
    const SemanticMapManager *semantic_map_manager, const double object_length,
    std::vector<PredictedTrajectoryPoint> *mutable_points) {
  if (semantic_map_manager == nullptr || mutable_points == nullptr ||
      mutable_points->empty()) {
    return std::nullopt;
  }
  // NOTE(yinbao): ObjectType here does not affect the logic.
  const auto collision_infos =
      FindTrajCurbCollisionIndex(*mutable_points, *semantic_map_manager,
                                 object_length, qcraft::ObjectType::OT_VEHICLE);
  if (collision_infos.empty()) {
    return std::nullopt;
  }
  const auto &first_idx = collision_infos.front().collision_index;
  QCHECK_GT(first_idx, 0);
  VLOG(3) << absl::StrFormat("traj cutoff at %d", first_idx);
  // Reset (i -> size - 1) points to the i - 1 point
  for (int i = first_idx, n = mutable_points->size(); i < n; ++i) {
    (*mutable_points)[i] = (*mutable_points)[first_idx - 1];
    (*mutable_points)[i].set_t(i * kPredictionTimeStep);
    (*mutable_points)[i].set_v(0.0);
    (*mutable_points)[i].set_a(0.0);
  }
  return first_idx;
}

void CutoffTrajByCurbCollisionIndexes(
    absl::Span<const TrajCollisionInfo> collision_infos, double nominal_brake,
    double kept_horizon, double safe_horizon, PredictedTrajectory *traj) {
  auto *points = traj->mutable_points();
  int cut_off_idx = points->size();
  const double inv_nominal_brake = 1.0 / nominal_brake;
  for (const auto &collision_info : collision_infos) {
    const int collision_index = collision_info.collision_index;
    // Curb cannot be passed through.
    if (!collision_info.can_pass) {
      cut_off_idx = std::min(collision_index, cut_off_idx);
      continue;
    }
    const double dist2curb = points->at(collision_index).s();
    const double nominal_brake_dis =
        std::pow(points->front().v(), 2) * 0.5 * std::fabs(inv_nominal_brake);
    if (nominal_brake_dis <= dist2curb) {
      cut_off_idx =
          std::min(std::max(FloorToInt(kept_horizon / kPredictionTimeStep),
                            collision_index),
                   cut_off_idx);
    } else {
      cut_off_idx =
          std::min(std::max(FloorToInt(safe_horizon / kPredictionTimeStep),
                            collision_index),
                   cut_off_idx);
    }
  }
  if (cut_off_idx < points->size()) {
    TruncateTrajectoryBySize(cut_off_idx, points);
    traj->set_annotation(
        absl::StrCat(traj->annotation(), ", curb cutoff at ", cut_off_idx));
  }
}

bool IsComingToCrossWalk(const SemanticMapManager *semantic_map_manager,
                         const Vec2d &cur_pos, const Vec2d &next_pos) {
  const auto level_id = semantic_map_manager->GetLevel();
  const mapping::CrosswalkInfo *cw_ptr =
      semantic_map_manager->GetNearestCrosswalkInfoAtLevel(level_id, cur_pos);
  if (cw_ptr != nullptr) {
    const double cur_dist = cw_ptr->polygon_smooth.DistanceTo(cur_pos);
    const double next_dist = cw_ptr->polygon_smooth.DistanceTo(next_pos);
    if (cur_dist > 0.0 && next_dist == 0.0) return true;
  }
  return false;
}

void CutoffTrajByTrafficLightStatus(
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    const SemanticMapManager &semantic_map_manager, PredictedTrajectory *traj) {
  const auto &pts = traj->points();
  // Find the enter intersection point of the object.
  int cutoff_index = -1;
  for (int i = 0, n = pts.size(); i < n; ++i) {
    const auto *intersection_ptr =
        semantic_map_manager.GetNearestIntersectionInfoAtLevel(
            semantic_map_manager.GetLevel(), pts[i].pos());
    double dist_to_intersection_polygon = std::numeric_limits<double>::max();
    if (intersection_ptr != nullptr) {
      dist_to_intersection_polygon =
          intersection_ptr->polygon_smooth.DistanceTo(pts[i].pos());
    }
    // Object already in intersection, do not cutoff.
    if (dist_to_intersection_polygon <= 0.0 && i == 0) {
      return;
    }
    if (dist_to_intersection_polygon <= 0.0) {
      cutoff_index = i - 1;
      break;
    }
  }
  // Not inside an intersection.
  if (cutoff_index == -1) {
    return;
  }

  const auto entrance_lane_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          semantic_map_manager, pts[cutoff_index].pos(),
          pts[cutoff_index].theta(), /*boundary_dist_limit=*/0.0,
          /*heading_diff=*/M_PI / 4.0);
  if (entrance_lane_or == std::nullopt) {
    return;
  }
  const auto &entrance_lane = entrance_lane_or.value();
  if (BlockObjectByTrafficLights(tl_state_map, semantic_map_manager,
                                 entrance_lane)) {
    CutoffTrajByLightBlockageIndex(cutoff_index,
                                   kTrafficLightCutOffNominalBrake, traj);
  }
}

}  // namespace prediction
}  // namespace qcraft
