#include "onboard/prediction/feature_extractor/feature_extraction_util.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace prediction {
namespace {
// force the smoothed trajectory always pass the first point.
constexpr double kMaxWeight = 5;
constexpr double kNormalWeight = 1.0;
constexpr int kMaxPolyFitDegree = 7;  // Degree.
}  // namespace
std::vector<PredictedTrajectoryPointProto> AlignPredictedTrajectoryPoints(
    const std::vector<PredictedTrajectoryPointProto> &prev_pts,
    double prev_time, double cur_time, double new_horizon, double dt) {
  if (prev_pts.size() <= 1) {
    return prev_pts;
  }
  class PredictedTrajectoryPointProtoLerper {
   public:
    PredictedTrajectoryPointProto operator()(
        const PredictedTrajectoryPointProto &a,
        const PredictedTrajectoryPointProto &b, double alpha) const {
      return LerpPredictedTrajectoryPointProto(a, b, alpha);
    }
  };
  std::vector<double> vec_t;

  for (const auto &prev_pt : prev_pts) {
    vec_t.push_back(prev_pt.t() + prev_time);
  }
  PiecewiseLinearFunction<PredictedTrajectoryPointProto, double,
                          PredictedTrajectoryPointProtoLerper>
      plf_traj(vec_t, std::vector<PredictedTrajectoryPointProto>(
                          prev_pts.begin(), prev_pts.end()));
  std::vector<PredictedTrajectoryPointProto> res;
  double origin_horizon = prev_pts.back().t() - prev_pts.front().t();
  // ensure at least we have current point & next point.
  const int num = std::max(
      static_cast<int>(std::min(new_horizon, origin_horizon) / dt) + 1, 2);
  res.reserve(num);
  double s = 0.0;
  for (int i = 0; i < num; i++) {
    double cur_t = cur_time + i * dt;
    auto new_pt = plf_traj.EvaluateWithExtrapolation(cur_t);
    new_pt.set_t(i * dt);
    if (i != 0) {
      s += (Vec2d(new_pt.pos()) - Vec2d(res.back().pos())).norm();
      new_pt.set_s(s);
    }
    res.push_back(std::move(new_pt));
  }
  return res;
}
std::vector<PredictedTrajectoryPointProto>
ConvertAVTrajectoryPointsToPredictedTrajectoryPoints(
    absl::Span<const ApolloTrajectoryPointProto> av_pts) {
  std::vector<PredictedTrajectoryPointProto> res;
  res.reserve(av_pts.size());
  for (const auto &pt : av_pts) {
    PredictedTrajectoryPointProto predict_pt;
    auto &pos = *predict_pt.mutable_pos();
    pos.set_x(pt.path_point().x());
    pos.set_y(pt.path_point().y());
    predict_pt.set_v(pt.v());
    predict_pt.set_theta(pt.path_point().theta());
    predict_pt.set_s(pt.path_point().s());
    predict_pt.set_t(pt.relative_time());
    res.push_back(std::move(predict_pt));
  }
  return res;
}

std::vector<const mapping::LaneInfo *> GetLanesInBox2d(
    const SemanticMapManager &semantic_map_manager, const Box2d &region_box,
    const int max_lanes_num) {
  const auto level_id = semantic_map_manager.GetLevel();
  std::vector<const mapping::LaneInfo *> lane_info =
      semantic_map_manager.GetLanesInfoAtLevel(level_id, region_box.center(),
                                               region_box.diagonal() * 0.5);
  std::vector<const mapping::LaneInfo *> intersected_lanes;
  intersected_lanes.reserve(lane_info.size());
  for (const auto *lane_ptr : lane_info) {
    for (const auto &pt : lane_ptr->points_smooth) {
      if (region_box.IsPointIn(pt)) {
        intersected_lanes.push_back(lane_ptr);
        break;
      }
    }
  }
  // Here we hold the stats that max_lanes_num is above 75% lanes num.
  if (intersected_lanes.size() <= max_lanes_num) {
    return intersected_lanes;
  }
  std::vector<std::pair<const mapping::LaneInfo *, double>> lane_dist_pairs;
  lane_dist_pairs.reserve(intersected_lanes.size());
  for (const auto *lane_ptr : intersected_lanes) {
    if (double min_distance = 1000.0;
        semantic_map_manager.GetLaneProjectionAtLevel(
            level_id, region_box.center(), lane_ptr->id, nullptr, nullptr,
            &min_distance)) {
      lane_dist_pairs.push_back({lane_ptr, min_distance});
    }
  }
  std::sort(
      lane_dist_pairs.begin(), lane_dist_pairs.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });
  intersected_lanes.clear();
  for (int i = 0; i < max_lanes_num; ++i) {
    intersected_lanes.push_back(lane_dist_pairs[i].first);
  }
  return intersected_lanes;
}

std::vector<PredictedTrajectoryPointProto> SmoothPredictedTrajectory(
    absl::Span<const PredictedTrajectoryPointProto> raw_traj) {
  if (raw_traj.size() <= 1) {
    return std::vector(raw_traj.begin(), raw_traj.end());
  }
  std::vector<Vec2d> x_series, y_series;
  std::vector<double> weights;
  const double start_t = raw_traj.front().t();
  double sum_s = 0.0;
  for (int i = 0; i < raw_traj.size(); ++i) {
    const auto &raw_pt = raw_traj[i];
    x_series.emplace_back(raw_pt.t() - start_t, raw_pt.pos().x());
    y_series.emplace_back(raw_pt.t() - start_t, raw_pt.pos().y());
    if (i == 0) {
      weights.push_back(kMaxWeight);
    } else {
      weights.push_back(kNormalWeight);
    }
    if (i > 0) {
      sum_s += Vec2d(raw_pt.pos()).DistanceTo(Vec2d(raw_traj[i - 1].pos()));
    }
  }
  int polyfit_degree = std::min<int>(
      static_cast<int>(std::ceil(std::log2(sum_s))), raw_traj.size() - 1);
  polyfit_degree = std::min(polyfit_degree, kMaxPolyFitDegree);
  if (polyfit_degree <= 1) {
    return std::vector(raw_traj.begin(), raw_traj.end());
  }
  PolynomialFitter x_fitter, y_fitter;
  x_fitter.SetDegree(polyfit_degree);
  y_fitter.SetDegree(polyfit_degree);
  x_fitter.LoadData(x_series, weights);
  y_fitter.LoadData(y_series, weights);
  x_fitter.FitData(LS_SOLVER::QR);
  y_fitter.FitData(LS_SOLVER::QR);

  std::vector<PredictedTrajectoryPointProto> new_traj;
  new_traj.reserve(raw_traj.size());
  for (int i = 0; i < raw_traj.size(); ++i) {
    auto new_pt = raw_traj[i];
    new_pt.mutable_pos()->set_x(x_fitter.Evaluate(new_pt.t() - start_t));
    new_pt.mutable_pos()->set_y(y_fitter.Evaluate(new_pt.t() - start_t));
    new_traj.push_back(std::move(new_pt));
  }
  return new_traj;
}

std::vector<const ObjectHistory *> GetObjectsInBox2d(
    absl::Span<const ObjectHistory *const> objects_history,
    const Box2d &region_box) {
  std::vector<const ObjectHistory *> objects_in_box;
  for (const auto *object_history : objects_history) {
    const Vec2d pos = Vec2d(object_history->back().val.pos());
    if (region_box.IsPointIn(pos)) {
      objects_in_box.push_back(object_history);
    }
  }
  return objects_in_box;
}

ResampledObjectsHistory GetResampledObjectsInBox2d(
    const ResampledObjectsHistory &objects_history, const Box2d &region_box,
    const std::string &ego_id, int max_object_num) {
  const auto iter =
      std::find_if(objects_history.begin(), objects_history.end(),
                   [&](const auto &val) { return val.back().id() == ego_id; });
  QCHECK(iter != objects_history.end());
  const Vec2d ego_pos(iter->back().pos());
  struct DistanceInfo {
    const std::vector<ObjectProto> *obj_hist;
    double dist = std::numeric_limits<double>::max();
  };
  std::vector<DistanceInfo> dist_infos;
  dist_infos.reserve(objects_history.size());
  for (const auto &object_history : objects_history) {
    dist_infos.push_back(
        {.obj_hist = &object_history,
         .dist = ego_pos.DistanceSquareTo(Vec2d(object_history.back().pos()))});
  }
  std::sort(dist_infos.begin(), dist_infos.end(),
            [](const DistanceInfo &a, const DistanceInfo &b) {
              return a.dist < b.dist;
            });

  ResampledObjectsHistory objects_in_box;
  objects_in_box.reserve(max_object_num);
  for (const auto &dist_info : dist_infos) {
    if (objects_in_box.size() > max_object_num) {
      break;
    }
    const auto &obj_proto = dist_info.obj_hist->back();
    if (obj_proto.id() == ego_id) {
      continue;
    }
    const Vec2d pos = Vec2d(obj_proto.pos());
    if (!region_box.IsPointIn(pos)) {
      continue;
    }
    objects_in_box.push_back(*dist_info.obj_hist);
  }
  return objects_in_box;
}

Box2d GetRegionBox(const Vec2d &pos, const double heading,
                   double detection_region_front,
                   double detection_region_behind,
                   double detection_half_width) {
  const double half_length =
      (detection_region_front + detection_region_behind) * 0.5;
  const double center_ahead_dis = half_length - detection_region_behind;
  const Vec2d tangent = Vec2d::FastUnitFromAngle(heading);
  const Vec2d center = pos + tangent * center_ahead_dis;
  return Box2d(half_length, detection_half_width, center, heading, tangent);
}

/**
 * @brief Set Objects states features, convert states to ego-based coordinate.
 * @param ref_pos          Transform coordinate origin pos.
 * @param rot_rad          The rad that rotate ego to the east direction.
 * @param objects_history  Resampled 0.1s time step objects hisotry.
 * @param traj_ptrs        Pointer for tramsformed history relative trajs
 * features.
 * @param speed_ptrs       Pointer for tramsformed history speeds features.
 * @param heading_ptrs     Pointer for tramsformed history headings features.
 * @param cur_pos_ptrs     Pointer for transformed objects current poses
 * features.
 */
void SetHistoryStatesAndCurPoses(int coords, int history_num, double time_step,
                                 const Vec2d &ref_pos, double rot_rad,
                                 const ResampledObjectsHistory &objects_history,
                                 std::vector<float> *traj_ptrs,
                                 std::vector<float> *speed_ptrs,
                                 std::vector<float> *heading_ptrs,
                                 std::vector<float> *cur_pos_ptrs) {
  for (int object_idx = 0; object_idx < objects_history.size(); ++object_idx) {
    const auto &object_history = objects_history[object_idx];
    // Tranform pos to ego-based coordinate.
    const Vec2d cur_pos =
        (Vec2dFromProto(object_history.back().pos()) - ref_pos).Rotate(rot_rad);
    (*cur_pos_ptrs)[object_idx * coords] = cur_pos.x();
    (*cur_pos_ptrs)[object_idx * coords + 1] = cur_pos.y();
    Vec2d prev_pos = (Vec2dFromProto(object_history.front().pos()) - ref_pos)
                         .Rotate(rot_rad);
    // Note: Max valid_state_num is history_num + 1.
    const int valid_state_num = object_history.size();
    const int ref_index = history_num - valid_state_num;
    if (ref_index >= 0) {
      const double rot_heading =
          NormalizeAngle(object_history.front().yaw() + rot_rad);
      Vec2d rot_vel =
          (Vec2dFromProto(object_history.front().vel())).Rotate(rot_rad);
      (*traj_ptrs)[(object_idx * history_num + ref_index) * coords] =
          rot_vel.x() * time_step;
      (*traj_ptrs)[(object_idx * history_num + ref_index) * coords + 1] =
          rot_vel.y() * time_step;
      (*speed_ptrs)[object_idx * history_num + ref_index] = rot_vel.norm();
      (*heading_ptrs)[(object_idx * history_num + ref_index) * coords] =
          std::sin(rot_heading);
      (*heading_ptrs)[(object_idx * history_num + ref_index) * coords + 1] =
          std::cos(rot_heading);
    }
    for (int i = 1; i < valid_state_num; ++i) {
      const Vec2d cur_pos =
          (Vec2dFromProto(object_history[i].pos()) - ref_pos).Rotate(rot_rad);
      (*traj_ptrs)[(object_idx * history_num + ref_index + i) * coords] =
          (cur_pos - prev_pos).x();
      (*traj_ptrs)[(object_idx * history_num + ref_index + i) * coords + 1] =
          (cur_pos - prev_pos).y();
      prev_pos = cur_pos;
      const Vec2d rot_vel =
          (Vec2dFromProto(object_history[i].vel())).Rotate(rot_rad);
      const double rot_heading =
          NormalizeAngle(object_history[i].yaw() + rot_rad);
      (*speed_ptrs)[object_idx * history_num + ref_index + i] = rot_vel.norm();
      (*heading_ptrs)[(object_idx * history_num + ref_index + i) * coords] =
          std::sin(rot_heading);
      (*heading_ptrs)[(object_idx * history_num + ref_index + i) * coords + 1] =
          std::cos(rot_heading);
    }
  }
}

void SetLaneCentersAndSegments(int max_lane_point_num, int coord_num,
                               Vec2d ref_pos, double rot_rad, int index,
                               const mapping::LaneInfo *lane_ptr,
                               std::vector<float> *lane_center_ptrs,
                               std::vector<float> *lane_segment_ptrs) {
  const int sample_points_num = max_lane_point_num + 1;
  std::vector<Vec2d> lane_points;
  lane_points.reserve(sample_points_num);
  const double fraction_step = 1.0 / (sample_points_num - 1);
  for (int i = 0; i < sample_points_num; ++i) {
    lane_points.push_back(lane_ptr->LerpPointFromFraction(i * fraction_step));
  }
  Vec2d prev_transformed_lane_point =
      (lane_points[0] - ref_pos).Rotate(rot_rad);
  for (int j = 1; j < sample_points_num; ++j) {
    const Vec2d transformed_lane_point =
        (lane_points[j] - ref_pos).Rotate(rot_rad);
    const Vec2d lane_center =
        (transformed_lane_point + prev_transformed_lane_point) * 0.5f;
    const Vec2d lane_segment =
        transformed_lane_point - prev_transformed_lane_point;
    (*lane_center_ptrs)[(index * max_lane_point_num + j - 1) * coord_num] =
        lane_center.x();
    (*lane_center_ptrs)[(index * max_lane_point_num + j - 1) * coord_num + 1] =
        lane_center.y();
    (*lane_segment_ptrs)[(index * max_lane_point_num + j - 1) * coord_num] =
        lane_segment.x();
    (*lane_segment_ptrs)[(index * max_lane_point_num + j - 1) * coord_num + 1] =
        lane_segment.y();
    prev_transformed_lane_point = transformed_lane_point;
  }
}

std::vector<Vec2d> GetExitsOfIntersection(
    const SemanticMapManager &semantic_map_manager,
    const qcraft::mapping::IntersectionInfo &intersection,
    double reduction_radius) {
  std::vector<Vec2d> targets;
  for (const auto &[lane_index, fractions] : intersection.lanes) {
    // Find lanes that have intersection with the intersection polygon
    if (!(fractions.y() >= fractions.x() && fractions.y() < 1.0)) {
      continue;
    }
    const auto point =
        semantic_map_manager.lane_info()[lane_index].LerpPointFromFraction(
            fractions.y());
    targets.push_back(std::move(point));
  }
  std::sort(targets.begin(), targets.end(),
            [](const auto &a, const auto &b) -> bool { return a.x() < b.x(); });
  return PickPointsByGreedy(targets, reduction_radius);
}

std::vector<Vec2d> PickPointsByGreedy(absl::Span<const Vec2d> points,
                                      double radius) {
  std::vector<bool> to_removed(points.size(), false);
  const auto radius_sqr = radius * radius;
  for (int i = 0; i < points.size(); ++i) {
    if (to_removed[i]) {
      continue;
    }
    const auto &cur_pt = points[i];
    for (int j = i + 1; j < points.size(); ++j) {
      if (cur_pt.DistanceSquareTo(points[j]) < radius_sqr) {
        to_removed[j] = true;
      }
    }
  }
  std::vector<Vec2d> filtered_points;
  filtered_points.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    if (!to_removed[i]) {
      filtered_points.push_back(points[i]);
    }
  }
  return filtered_points;
}
}  // namespace prediction
}  // namespace qcraft
