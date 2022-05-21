#include "onboard/perception/tracker/tracker.h"

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <unordered_map>
#include <utility>

#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "boost/math/distributions/chi_squared.hpp"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/nets/fiery_eye_net_classifier_config.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/obstacle_constants.h"
#include "onboard/perception/perception_util.h"
#include "onboard/perception/tracker/association/metric.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/tracker_constant.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/perception/type_util.h"
#include "onboard/perception/utils/vis_utils.h"

DEFINE_bool(track_association_cvs, false,
            "Whether to draw association results in tracker on canvas");
DEFINE_bool(smooth_bb, false, "Whether to publish smoothed bounding boxes");
DEFINE_bool(measurement_cvs, false,
            "Whether to render measurements with uncertainties.");
DEFINE_bool(track_cvs, false, "Whether to show information about tracks.");
DEFINE_bool(merge_split_track, true,
            "Whether to use track merge and split strategy.");

DEFINE_bool(publish_refined_bbox, true, "Whether to publish refined bbox.");
DEFINE_bool(render_tcn_label_frame_cvs, false,
            "Enable render label frame using canvas.");

namespace qcraft::tracker {

namespace {

// Flip corner type yaw diff threshold.
constexpr double kMinYawDiffThresholdToConsiderCornerFlipping = 0.5 * M_PI;

// NOTE(zheng): The Vel measurement comes from icp or fen,
// they are both not stable nowadays, so we should
// give a larger measurement nosie.
constexpr double kVelMeasurementNoise = Sqr(2.0);  // m/s^2

constexpr TrackState::RefPoint::Type kCornerRefPointTypeTable[] = {
    TrackState::RefPoint::Type::kTopLeftCorner,
    TrackState::RefPoint::Type::kBottomLeftCorner,
    TrackState::RefPoint::Type::kBottomRightCorner,
    TrackState::RefPoint::Type::kTopRightCorner,
};
constexpr TrackState::RefPoint::Type kFaceRefPointTypeTable[] = {
    TrackState::RefPoint::Type::kFrontFaceCenter,
    TrackState::RefPoint::Type::kLeftFaceCenter,
    TrackState::RefPoint::Type::kRearFaceCenter,
    TrackState::RefPoint::Type::kRightFaceCenter,
};

vis::Color GenerateRandomColor(int seed) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<> dis(0.0, 1.0);
  return vis::Color(dis(gen), dis(gen), dis(gen));
}

struct MeasurementProtoPtrComp {
  bool operator()(const MeasurementProto* left,
                  const MeasurementProto* right) const {
    return left->timestamp() < right->timestamp();
  }
};
using MeasurementProtoPtrSet =
    std::set<const MeasurementProto*, MeasurementProtoPtrComp>;

Vec2d FindClosestPoint(const std::vector<Vec2d>& points,
                       const Vec2d& ref_point) {
  QCHECK(!points.empty());
  std::vector<double> dist2(points.size());
  for (int i = 0; i < points.size(); ++i) {
    dist2[i] = (points[i] - ref_point).squaredNorm();
  }
  return points[std::distance(dist2.begin(),
                              std::min_element(dist2.begin(), dist2.end()))];
}

Box2d ComputeMeanBoundingBoxFromHistory(
    const HistoryBuffer<const MeasurementProto*>& measurement_history) {
  int num_bb = 0;
  Vec2d center;
  Vec2d size;
  double heading_sin_sum = 0.0;
  double heading_cos_sum = 0.0;
  std::optional<double> prev_heading;
  double weight_sum = 0.0;
  for (const auto& [_, measurement] : measurement_history) {
    if (measurement->has_laser_measurement() &&
        measurement->laser_measurement().has_detection_bounding_box()) {
      const Box2d bb =
          FLAGS_publish_refined_bbox
              ? Box2d(measurement->laser_measurement().refined_bounding_box())
              : Box2d(
                    measurement->laser_measurement().detection_bounding_box());
      center += bb.center();
      size *= kBoundingBoxSmoothFactor;
      size += Vec2d(bb.length(), bb.width());
      double heading = bb.heading();
      // If the heading is flipped, keep the previous heading.
      if (prev_heading &&
          std::abs(NormalizeAngle(*prev_heading - heading)) > M_PI_2) {
        heading = *prev_heading;
      }
      heading_sin_sum += fast_math::Sin(heading);
      heading_cos_sum += fast_math::Cos(heading);
      prev_heading = heading;
      num_bb++;
      weight_sum = weight_sum * kBoundingBoxSmoothFactor + 1.0;
    }
  }
  QCHECK_GT(num_bb, 0);
  return Box2d(center / num_bb,
               fast_math::Atan2(heading_sin_sum, heading_cos_sum),
               size.x() / weight_sum, size.y() / weight_sum);
}

struct AssociationInfo {
  double ComputeWeight(double gate_scale) const {
    // NOTE(zheng): When last time we update radar or camera measurements,
    // the time diff maybe a very small value, maybe close to zero, this will
    // make a very small gating which will casue lose association issue, so
    // we should keep a min gating threshold when comptue gating threshold.
    const double delta_time = time_diff < 0.1 ? 0.1 : time_diff;
    if (dist2 > Sqr(gate_scale) * Sqr(delta_time) *
                    (onroad ? Sqr(kOnroadMaxMatchDistPerSec)
                            : Sqr(kOffroadMaxMatchDistPerSec))) {
      return 0.0;
    }
    return (0.1 + iou) / (0.1 * (dist2 + 25.0 * mse + 1.0));
  }

  // Squared distance between track centers.
  double dist2;
  // IoU between two tracks.
  double iou;
  // The elapsed time between two tracks.
  double time_diff;
  // Whether the measurement is onroad or offroad.
  bool onroad;
  // ICP match info.
  double mse;
};

double ComputeIoU(const Polygon2d& contour1, const Polygon2d& contour2) {
  Polygon2d intersection;
  if (!contour1.ComputeOverlap(contour2, &intersection)) {
    return 0.0;
  }
  const double intersection_area = intersection.area();
  return intersection_area /
         (contour1.area() + contour2.area() - intersection_area);
}

double ComputePValueFromChiSquareDistr(const double value, const int dof) {
  // To catch non-positive definite covariance matrix.
  QCHECK_GE(value, 0.0) << absl::StrFormat("Illegal chi square value: %f",
                                           value);
  QCHECK(dof == 4 || dof == 2);
  // Construct chi square table of dof 2 and dof 4.
  std::unordered_map<int, std::vector<double>> chi_square_table;
  const std::vector<double> cdf_list = {1.0,   0.995, 0.99,  0.975, 0.95,
                                        0.90,  0.10,  0.05,  0.025, 0.01,
                                        0.005, 0.001, 0.0001};
  chi_square_table[2] =
      std::vector<double>{0.0,   0.010, 0.020, 0.051,  0.103, 0.211, 4.605,
                          5.991, 7.378, 9.210, 10.597, 13.8,  18.5};
  chi_square_table[4] =
      std::vector<double>{0.0,   0.207,  0.297,  0.484,  0.711, 1.064, 7.779,
                          9.488, 11.143, 13.277, 14.860, 18.5,  23.5};
  const PiecewiseLinearFunction plf(chi_square_table[dof], cdf_list);
  return plf.Evaluate(value);
}

void MaybeRenderLabelFrameCvs(const labeling::LabelFrameProto& label_frame) {
  if (!FLAGS_render_tcn_label_frame_cvs) {
    return;
  }
  using namespace labeling;  // NOLINT
  const auto& pose = label_frame.pose();
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/tcn_label_frame_cvs");
  for (const auto& label : label_frame.labels()) {
    canvas.DrawBox({label.x(), label.y(), label.z()}, label.heading(),
                   {label.length(), label.width()}, vis::Color::kWhite, 2);
    canvas.DrawText(absl::StrFormat("[Category] %s",
                                    Label::Category_Name(label.category())),
                    {label.x(), label.y(), label.z()}, 0.0, 0.2,
                    vis::Color::kWhite);
  }
  for (const auto& zone : label_frame.zones()) {
    std::vector<Vec3d> vertices(zone.xs().size());
    QCHECK_EQ(zone.xs().size(), zone.ys().size());
    QCHECK_EQ(zone.xs().size(), zone.zs().size());
    for (int i = 0; i < zone.xs().size(); ++i) {
      vertices[i] = {zone.xs()[i], zone.ys()[i], zone.zs()[i]};
    }
    canvas.DrawPolygon(vertices, GetZoneTypeColor(zone.type()), 2);
    std::vector<Vec2d> vertices_2d(vertices.size());
    for (int i = 0; i < vertices.size(); ++i) {
      vertices_2d[i] = {vertices[i].x(), vertices[i].y()};
    }
    const auto& centroid = Polygon2d(vertices_2d).centroid();
    if (zone.type() != Zone::FULL_SEMANTIC) {
      canvas.DrawText(
          absl::StrFormat("[Type] %s", Zone::ZoneType_Name(zone.type())),
          {centroid.x(), centroid.y(), pose.z() + 0.1}, 0.0, 0.2,
          vis::Color::kWhite);
    }
  }
  canvas.DrawText(
      absl::StrFormat("[LabelFrame] Pose: x %.2f y %.2f z %.2f "
                      "roll %.2f pitch %.2f yaw %.2f, timestamp: %.3f",
                      pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(),
                      pose.yaw(), label_frame.timestamp()),
      {pose.x(), pose.y(), pose.z() + 0.1}, 0.0, 0.2, vis::Color::kWhite);
}

void RenderCameraMeasurement(const CameraMeasurementProto& camera_measurement,
                             const double render_height, vis::Canvas* canvas,
                             const int size = 1,
                             const vis::Color& color = vis::Color::kRed) {
  QCHECK_NOTNULL(canvas);
  Mat2d center_cov;
  Mat2dFromProto(camera_measurement.center_cov(), &center_cov);
  const auto center = Vec2dFromProto(camera_measurement.center());
  // Scale for one sigma, confidence is 0.68,
  // the sigma scale is sqrt(chi-squared(0.68)) = 1.51.
  constexpr double kSqrtOneSigmaScale = 1.51;
  const auto ellipse_points_2d =
      GetCovarianceEllipsePoints(center_cov, kSqrtOneSigmaScale);
  if (!ellipse_points_2d.has_value()) {
    return;
  }
  std::vector<Vec3d> ellipse_points;
  ellipse_points.reserve(ellipse_points_2d->size());
  for (const auto& point_2d : *ellipse_points_2d) {
    ellipse_points.emplace_back(point_2d + Vec2d(center.x(), center.y()),
                                render_height);
  }
  canvas->DrawPolygon(ellipse_points, color, size);
}

void MaybeRenderMeasurements(const MeasurementsProto& measurements_proto) {
  if (!FLAGS_measurement_cvs) return;

  // Draw measurement cov.
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/measurements");

  for (const auto& measurement : measurements_proto.measurements()) {
    // TODO(tao, yu): Visualize all measurements.
    if (!measurement.has_camera_measurement()) {
      continue;
    }
    constexpr double kRenderHeight = 10.;
    RenderCameraMeasurement(measurement.camera_measurement(), kRenderHeight,
                            &canvas);
  }
}

void MaybeRenderTracks(const std::vector<TrackRef>& tracks,
                       const VehiclePose& pose) {
  if (!FLAGS_track_cvs) return;

  vis::Canvas& canvas = vantage_client_man::GetCanvas("perception/tracks");
  for (int i = 0; i < tracks.size(); ++i) {
    const auto& track = *tracks[i];
    // Use a random color to differentiate different tracks.
    const vis::Color color = GenerateRandomColor(i);
    const double render_height = pose.z + 30.;

    tracker_util::RenderContourCvs(track.track_state.contour, render_height,
                                   color, 2, &canvas);
    const std::vector<std::string> track_infos =
        absl::StrSplit(track.DebugString(), '\n', absl::SkipEmpty());

    const auto& pos = track.track_state.ref_point.pos;
    for (int i = 0; i < track_infos.size(); ++i) {
      canvas.DrawText(track_infos[i],
                      {pos.x(), pos.y() - 0.3 * i, render_height}, 0., 0.2,
                      color);
    }
    Mat2d center_cov;
    center_cov = track.track_state.estimator_3d.GetStatePosCov();
    // Scale for one sigma, confidence is 0.68,
    // the sigma scale is sqrt(chi-squared(0.68)) = 1.51.
    constexpr double kSqrtOneSigmaScale = 1.51;
    const auto ellipse_points_2d =
        GetCovarianceEllipsePoints(center_cov, kSqrtOneSigmaScale);
    if (!ellipse_points_2d.has_value()) {
      continue;
    }
    std::vector<Vec3d> ellipse_points;
    ellipse_points.reserve(ellipse_points_2d->size());
    for (const auto& point_2d : *ellipse_points_2d) {
      ellipse_points.emplace_back(point_2d + pos, render_height - 2.);
    }
    canvas.DrawPolygon(ellipse_points, color,
                       track.track_state.estimator_3d.IsCarModel()
                           ? vis::BorderStyleProto::DASHED
                           : vis::BorderStyleProto::DOT_AND_DASH);
  }
}

std::vector<const MeasurementProto*> GetMeasurementsWithType(
    const MeasurementsProto& measurements_proto,
    const MeasurementProto::MeasurementCase& type) {
  const int num_measurements = measurements_proto.measurements_size();
  std::vector<const MeasurementProto*> results;
  results.reserve(num_measurements);
  for (const auto& measurement : measurements_proto.measurements()) {
    if (measurement.Measurement_case() == type) {
      results.push_back(&measurement);
    }
  }
  return results;
}

Polygon2d PredictContour(const double timestamp,
                         const Track<TrackState>& track) {
  // TODO(yu): Consider heading shift as well.
  const auto s = tracker_util::SafePredictPos(track, timestamp);
  if (!s.has_value()) return track.track_state.contour;
  const Vec2d pos_shift =
      s->GetStatePos() - track.track_state.estimator_3d.GetStatePos();
  auto contour_points = track.track_state.contour.points();
  std::for_each(contour_points.begin(), contour_points.end(),
                [&](Vec2d& ele) { ele += pos_shift; });
  return Polygon2d(std::move(contour_points));
}

Box2d PredictBbox(const double timestamp, const Track<TrackState>& track) {
  // TODO(zheng): Consider heading shift as well.
  const auto s = tracker_util::SafePredictPos(track, timestamp);
  if (!s.has_value()) return track.track_state.bounding_box.value();
  const Vec2d pos_shift =
      s->GetStatePos() - track.track_state.estimator_3d.GetStatePos();
  auto box = track.track_state.bounding_box.value();
  box.Shift(pos_shift);
  return box;
}

Box2d PredictRefinedBbox(const double timestamp,
                         const Track<TrackState>& track) {
  // TODO(zheng): Consider heading shift as well.
  const auto s = tracker_util::SafePredictPos(track, timestamp);
  if (!s.has_value()) return track.track_state.refined_bounding_box.value();
  const Vec2d pos_shift =
      s->GetStatePos() - track.track_state.estimator_3d.GetStatePos();
  auto box = track.track_state.refined_bounding_box.value();
  box.Shift(pos_shift);
  return box;
}

void PredictTrackShapeInfo(const double timestamp, Track<TrackState>* track) {
  track->track_state.contour = PredictContour(timestamp, *track);
  if (track->track_state.bounding_box) {
    track->track_state.bounding_box = PredictBbox(timestamp, *track);
    track->track_state.refined_bounding_box =
        PredictRefinedBbox(timestamp, *track);
  }
  // Predict track.track_state.obstacle_infos when prediction is valid.
  const auto s = tracker_util::SafePredictPos(*track, timestamp);
  if (s.has_value()) {
    const Vec2d pos_shift =
        s->GetStatePos() - track->track_state.estimator_3d.GetStatePos();
    track->track_state.anchor_point += pos_shift;
  }
}

void PredictAndUpdateTrack(const double timestamp, Track<TrackState>* track) {
  PredictTrackShapeInfo(timestamp, track);
  track->track_state.state_timestamp = timestamp;
  track->track_state.estimator_3d.Predict(timestamp);
}

bool IsCornerOrFaceCenterRefType(const TrackState::RefPoint::Type ref_type) {
  switch (ref_type) {
    case TrackState::RefPoint::kTopRightCorner:
    case TrackState::RefPoint::kTopLeftCorner:
    case TrackState::RefPoint::kBottomLeftCorner:
    case TrackState::RefPoint::kBottomRightCorner:
    case TrackState::RefPoint::kFrontFaceCenter:
    case TrackState::RefPoint::kLeftFaceCenter:
    case TrackState::RefPoint::kRearFaceCenter:
    case TrackState::RefPoint::kRightFaceCenter:
    case TrackState::RefPoint::kBBCenter:
      return true;
    default:
      return false;
  }
}

TrackState::RefPoint::Type GetFlippedCornerOrFaceType(
    const TrackState::RefPoint::Type& ref_type) {
  QCHECK(IsCornerOrFaceCenterRefType(ref_type));
  switch (ref_type) {
    case TrackState::RefPoint::kTopRightCorner:
      return TrackState::RefPoint::kBottomLeftCorner;
    case TrackState::RefPoint::kTopLeftCorner:
      return TrackState::RefPoint::kBottomRightCorner;
    case TrackState::RefPoint::kBottomLeftCorner:
      return TrackState::RefPoint::kTopRightCorner;
    case TrackState::RefPoint::kBottomRightCorner:
      return TrackState::RefPoint::kTopLeftCorner;
    case TrackState::RefPoint::kFrontFaceCenter:
      return TrackState::RefPoint::kRearFaceCenter;
    case TrackState::RefPoint::kLeftFaceCenter:
      return TrackState::RefPoint::kRightFaceCenter;
    case TrackState::RefPoint::kRearFaceCenter:
      return TrackState::RefPoint::kFrontFaceCenter;
    case TrackState::RefPoint::kRightFaceCenter:
      return TrackState::RefPoint::kLeftFaceCenter;
    default:
      return ref_type;
  }
}

TrackState::RefPoint::Type TransformRefTypeWhenYawFlipped(
    const TrackState::RefPoint::Type& ref_type, double source_heading,
    double target_heading) {
  TrackState::RefPoint::Type transformed_type = ref_type;
  const double angle_diff = NormalizeAngle(source_heading - target_heading);
  if (std::abs(angle_diff) > kMinYawDiffThresholdToConsiderCornerFlipping) {
    transformed_type = GetFlippedCornerOrFaceType(ref_type);
  }
  return transformed_type;
}

MotionFilterParamProto FindMotionFilterParamProtoByType(
    const MotionFilterParamType& type) {
  static const std::unordered_map<MotionFilterParamType, std::string>
      kMotionFilterParamPathTable{
          {MFPT_HIGHWAY_CAR_MODEL_CTRA_CAR,
           "onboard/perception/tracker/motion_filter_2/data/"
           "highway_car_model_ctra_car.pb.txt"},
          {MFPT_HIGHWAY_CAR_MODEL_CTRA_CYC,
           "onboard/perception/tracker/motion_filter_2/data/"
           "highway_car_model_ctra_cyc.pb.txt"},
          {MFPT_URBAN_CAR_MODEL_CTRA_CAR,
           "onboard/perception/tracker/motion_filter_2/data/"
           "urban_car_model_ctra_car.pb.txt"},
          {MFPT_URBAN_CAR_MODEL_CTRA_CYC,
           "onboard/perception/tracker/motion_filter_2/data/"
           "urban_car_model_ctra_cyc.pb.txt"},
          {MFPT_POINT_MODEL_CA,
           "onboard/perception/tracker/motion_filter_2/data/"
           "point_model.pb.txt"},
          {MFPT_POINT_MODEL_CP,
           "onboard/perception/tracker/motion_filter_2/data/"
           "point_model.pb.txt"},
          {MFPT_POINT_MODEL_CA_PED,
           "onboard/perception/tracker/motion_filter_2/data/"
           "point_model_ped.pb.txt"},
          {MFPT_POINT_MODEL_CA_CYC,
           "onboard/perception/tracker/motion_filter_2/data/"
           "point_model_cyc.pb.txt"}};
  MotionFilterParamProto motion_filter_param;
  const std::string param_path = FindOrDie(kMotionFilterParamPathTable, type);
  QCHECK(file_util::TextFileToProto(param_path, &motion_filter_param));
  return motion_filter_param;
}

// Get motion filter by motion filter type.
Estimator CreateEstimator(
    const Track<TrackState>& track, bool should_use_car_model,
    const MotionFilterParamType motion_filter_param_type) {
  const MotionFilterParamProto params =
      FindMotionFilterParamProtoByType(motion_filter_param_type);
  if (should_use_car_model) {
    auto params_ca = params;
    params_ca.set_type(MotionFilterParamProto::CAR_CA);
    auto params_ctrv = params;
    params_ctrv.set_type(MotionFilterParamProto::CAR_CTRV);
    std::vector<MotionFilterParamProto> v{params_ca, params_ctrv};
    return Estimator(ToImmProto(v, {0.5, 0.5}), /*enable_debug_flags=*/true);
  } else {
    if (motion_filter_param_type == MFPT_POINT_MODEL_CP) {
      auto params_cp = params;
      params_cp.set_type(MotionFilterParamProto::POINT_CP);
      std::vector<MotionFilterParamProto> v{params_cp};
      return Estimator(ToImmProto(v, {1.0}), /*enable_debug_flags=*/true);
    } else {
      auto params_cp = params;
      params_cp.set_type(MotionFilterParamProto::POINT_CP);
      auto params_cv = params;
      params_cv.set_type(MotionFilterParamProto::POINT_CV);
      auto params_ca = params;
      params_ca.set_type(MotionFilterParamProto::POINT_CA);
      std::vector<MotionFilterParamProto> v{params_cp, params_cv, params_ca};
      return Estimator(ToImmProto(v, {0.1, 0.45, 0.45}),
                       /*enable_debug_flags=*/true);
    }
  }
}

}  // namespace

MotionFilterParamType Tracker::GetMotionFilterParamType(
    const Track<TrackState>& track, const bool should_use_car_model) {
  const auto type = track.track_state.type;
  if (should_use_car_model && type == MT_VEHICLE) {
    return track.track_state.is_on_highway ? MFPT_HIGHWAY_CAR_MODEL_CTRA_CAR
                                           : MFPT_URBAN_CAR_MODEL_CTRA_CAR;
  } else if (should_use_car_model &&
             (type == MT_CYCLIST || type == MT_MOTORCYCLIST)) {
    return track.track_state.is_on_highway ? MFPT_HIGHWAY_CAR_MODEL_CTRA_CYC
                                           : MFPT_URBAN_CAR_MODEL_CTRA_CYC;
  } else if (type == MT_PEDESTRIAN) {
    return MFPT_POINT_MODEL_CA_PED;
  } else if (type == MT_CYCLIST) {
    return MFPT_POINT_MODEL_CA_CYC;
  } else if (type == MT_VEHICLE || type == MT_FLYING_BIRD ||
             type == MT_MOTORCYCLIST) {
    return MFPT_POINT_MODEL_CA;
  } else if (type == MT_VEGETATION || type == MT_BARRIER || type == MT_CONE ||
             type == MT_FOD || type == MT_ROAD || type == MT_MIST ||
             type == MT_WARNING_TRIANGLE) {
    return MFPT_POINT_MODEL_CP;
  } else if (type == MT_UNKNOWN || type == MT_STATIC_OBJECT) {
    if (ShouldUsePointModelCP(track))
      return MFPT_POINT_MODEL_CP;
    else
      return MFPT_POINT_MODEL_CA;
  } else {
    QLOG(FATAL) << "track type is: " << track.track_state.type
                << " which has no match type in motion filter";
  }
}

void Tracker::SyncMotionFilterInfoToDebugProto(
    const Track<TrackState>& track, TrackerDebugProto* group_debug_proto) {
  auto* motion_filter_proto = group_debug_proto->add_motion_filter_info();
  *motion_filter_proto = track.track_state.estimator_3d.GetMotionFilterInfo();
  motion_filter_proto->set_id(track.track_state.id);
}

void Tracker::SyncSingleTrackInfoToDebugProto(
    const Track<TrackState>& track, TrackerDebugProto* group_debug_proto) {
  auto* track_info_proto = group_debug_proto->add_track_info();
  track_info_proto->mutable_track_contour()->Reserve(
      track.track_state.contour.num_points());
  for (const auto& p : track.track_state.contour.points()) {
    auto* track_contour_info = track_info_proto->add_track_contour();
    track_contour_info->set_x(p.x());
    track_contour_info->set_y(p.y());
  }
  track_info_proto->set_track_id(track.track_state.id);
  track_info_proto->set_track_type(track.track_state.type);
  track_info_proto->set_heading(track.track_state.heading);

  Mat2d center_cov;
  center_cov = track.track_state.estimator_3d.GetStatePosCov();

  auto* pos_cov_proto = track_info_proto->mutable_pos_cov();
  Mat2dToProto(center_cov, pos_cov_proto);

  // Measurement noise cov.
  Mat2d pos_measurement_noise_cov;

  if (track.track_state.estimator_3d.IsCarModel()) {
    const StateData s = track.track_state.estimator_3d.GetStateData();
    const double vel_variance = s.GetVelCov();
    track_info_proto->set_vel_variance(vel_variance);

    // Sync acc variance.
    const double acc_variance = s.GetAccCov();
    track_info_proto->set_acc_variance(acc_variance);

    // Sync heading variance.
    const double heading_variance = s.GetYawCov();
    track_info_proto->set_heading_variance(heading_variance);

    // TODO(jiawei): as discussed with zheng, measurement noise getter will be
    // implemented together with the task of motion filter parameterization
    // next month.
  } else if (track.track_state.estimator_3d.IsPointModel()) {
    // State cov.
    const StateData s = track.track_state.estimator_3d.GetStateData();

    // Sync vel cov.
    const Mat2d vel_cov = s.GetSpeedCov();
    auto* vel_cov_proto = track_info_proto->mutable_vel_cov();
    Mat2dToProto(vel_cov, vel_cov_proto);
    // Sync acc cov.
    const Mat2d acc_cov = s.GetAxAyCov();
    auto* acc_cov_proto = track_info_proto->mutable_acc_cov();
    Mat2dToProto(acc_cov, acc_cov_proto);
  }

  // Sync track history info.
  const bool has_lidar_measurements = TMSTM::HasLidarMeasurements(track);
  const Vec2d pos =
      !has_lidar_measurements
          ? tracker_util::ComputeCentroid(track.track_state.contour.points())
          : track.track_state.ref_point.pos;
  // Sync ref point.
  track_info_proto->mutable_ref_point()->set_x(pos.x());
  track_info_proto->mutable_ref_point()->set_y(pos.y());

  // Sync state pos.
  const auto state_pos = track.track_state.estimator_3d.GetStatePos();
  track_info_proto->mutable_state_pos()->set_x(state_pos.x());
  track_info_proto->mutable_state_pos()->set_y(state_pos.y());

  auto* state_debug_info = track_info_proto->mutable_track_state_debug_info();
  SyncTrackStateToDebugInfo(track, state_debug_info);
}

void Tracker::SyncTrackStateToDebugInfo(
    const Track<TrackState>& track,
    TrackerDebugProto_TrackStateDebugProtoInfo* debug_info) {
  const auto& track_state = track.track_state;
  const auto& track_debug_info = track.debug_info;
  debug_info->set_id(track_state.id);
  debug_info->set_type(track_state.type);
  debug_info->set_ref_point_type(track_state.ref_point.type);
  debug_info->set_last_timestamp(track_state.last_timestamp);
  debug_info->set_offroad(track_state.offroad);
  debug_info->set_vel(track_state.vel);
  debug_info->set_in_parking_area(track_state.in_parking_area);
  debug_info->set_life_state(track_state.life_state);
  debug_info->set_is_merged(track_state.merged_by_other_track);
  debug_info->set_is_splited(track_state.split_from_other_track);
  debug_info->set_motion_filter_type(track_state.motion_filter_type);
  debug_info->set_motion_filter_param_type(
      track_state.motion_filter_param_type);
  debug_info->set_moving_state(
      tracker_util::MovingStateToProto(track_state.moving_state));
  debug_info->set_is_certain_static_track(track_state.is_certain_static_track);
  // TODO(zheng): Add track measurement source type to debug info.
  const bool is_radar_only = track_state.measurement_source_type ==
                             TrackMeasurementSourceType::TMST_RO;
  debug_info->set_is_radar_only(is_radar_only);
  debug_info->set_measurement_source_type(track_state.measurement_source_type);
  debug_info->set_updated_with_camera(track_debug_info.updated_with_camera);
  debug_info->set_updated_with_icp_velocity(
      track_debug_info.updated_with_icp_velocity);
  debug_info->set_updated_with_radar_velocity(
      track_debug_info.updated_with_radar_velocity);
  debug_info->set_updated_with_fen_velocity(
      track_debug_info.updated_with_fen_velocity);
  if (track_debug_info.state_pos_reset_offset) {
    const auto& offset = track_debug_info.state_pos_reset_offset.value();
    auto* offset_proto = debug_info->mutable_reset_offset();
    Vec2dToProto(offset, offset_proto);
  }

  constexpr double kValidTimeDuration = 0.5;  // s
  for (const auto& [ts, m] : track.measurement_history) {
    const double time_diff = track_state.state_timestamp - ts;
    if (time_diff > kValidTimeDuration || m->has_camera_measurement()) {
      continue;
    }
    debug_info->add_m_timestamp(ts);
    debug_info->add_m_type(m->type());
    debug_info->add_m_sensor_type(m->Measurement_case());
    if (m->has_laser_measurement()) {
      debug_info->add_m_id(-1);
      debug_info->add_m_sensor_id(-1);
    } else if (m->has_camera3d_measurement()) {
      debug_info->add_m_id(m->camera3d_measurement().track_id());
      debug_info->add_m_sensor_id(m->camera3d_measurement().camera_id());
    } else if (m->has_radar_measurement()) {
      debug_info->add_m_id(m->radar_measurement().id());
      debug_info->add_m_sensor_id(m->radar_measurement().radar_id());
    } else {
      QLOG(FATAL) << "Unknown measurement type.";
    }
  }

  debug_info->set_observation_state(track_state.observation_state);
}

void Tracker::AddDeletedTrackInfoToDebugProto(const Track<TrackState>& track) {
  const int group_num = tracker_debug_proto_->group_debug_proto_size();
  if (group_num == 0) return;
  // Add delete info to latest debug group proto.
  auto* latest_debug_proto =
      tracker_debug_proto_->mutable_group_debug_proto(group_num - 1);
  auto* track_info_proto = latest_debug_proto->add_deleted_track_info();
  for (const auto& p : track.track_state.contour.points()) {
    auto* track_contour_info = track_info_proto->add_track_contour();
    track_contour_info->set_x(p.x());
    track_contour_info->set_y(p.y());
  }
  track_info_proto->set_merged_by_other_track(
      track.track_state.merged_by_other_track);
}

void Tracker::AddTracksInfoToDebugProto(
    const MeasurementsProto_GroupType group_type, const int group_num,
    const int group_index, const double min_timestamp,
    const double max_timestamp, std::vector<TrackRef>* tracks,
    TrackerDebugProto* debug_proto) {
  SCOPED_QTRACE("Tracker::AddTracksInfoToDebugProto");

  debug_proto->set_group_type(group_type);
  debug_proto->set_group_index(group_index);
  debug_proto->set_group_num(group_num);
  debug_proto->set_group_min_timestamp(min_timestamp);
  debug_proto->set_group_max_timestamp(max_timestamp);

  // Do not publish vegetation or barrier debug info for saving debug proto
  // memory.
  for (auto& track : *tracks) {
    if (track->track_state.type == MT_VEGETATION ||
        track->track_state.type == MT_BARRIER) {
      continue;
    }
    SyncSingleTrackInfoToDebugProto(*track, debug_proto);
    SyncMotionFilterInfoToDebugProto(*track, debug_proto);
    // Reset debug info saved in track.
    track->track_state.estimator_3d.ClearMotionFilterInfo();
    track->debug_info = TrackDebugInfo();
  }
}
void Tracker::RunOutputGuarder() {
  // NOTE(zheng): If the input measurements size is zero, we don't run
  // TrackObjects() func, the tracker debug proto is nullptr, and there
  // is no need to run output guarder.
  if (nullptr == coordinate_converter_ || !tracker_debug_proto_) {
    QLOG(INFO) << "No valid coordinate_converter, the track num is: "
               << tracks_.size();
    return;
  }
  const int group_num = tracker_debug_proto_->group_debug_proto_size();
  std::unordered_map<int, TrackerDebugProto::TrackInfo*> id_to_track_info_ptr;
  if (group_num > 0) {
    auto* latest_debug_proto =
        tracker_debug_proto_->mutable_group_debug_proto(group_num - 1);
    for (int i = 0; i < latest_debug_proto->track_info_size(); ++i) {
      auto* track_info = latest_debug_proto->mutable_track_info(i);
      const int td = track_info->track_id();
      id_to_track_info_ptr[td] = track_info;
    }
  }
  // Run output guarder for all tracks, and sync debug proto.
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int track_index) {
    auto& track = *tracks_[track_index];
    TrackerDebugProto::OutputGuarderDebugInfoProto* output_guarder_debug_info =
        nullptr;
    TrackerDebugProto::TrackInfo* track_info = nullptr;
    if (id_to_track_info_ptr.find(track.track_state.id) !=
        id_to_track_info_ptr.end()) {
      track_info = id_to_track_info_ptr[track.track_state.id];
      output_guarder_debug_info = track_info->mutable_guarder_debug_info();
    }
    track.track_state.moving_state = output_guarder_.Guard(
        pose_, *semantic_map_manager_, *coordinate_converter_, track,
        output_guarder_debug_info);
    if (track_info) {
      // Sync moving state.
      const auto moving_state_proto =
          tracker_util::MovingStateToProto(track.track_state.moving_state);
      track_info->mutable_track_state_debug_info()->set_moving_state(
          moving_state_proto);
    }
  });
}

bool Tracker::FillTrajectory(const Track<TrackState>& track,
                             ObjectProto* object) const {
  if (nullptr == object) return false;
  const double some_future_ts = track.track_state.state_timestamp + 0.1;
  const auto* ldr_box_ckpt =
      tracker_util::FindLatestLidarCheckPointBeforeTimeWithBbox(track,
                                                                some_future_ts);
  const auto* ldr_ckpt = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, some_future_ts, MeasurementsProto::LIDAR);
  const auto* cam_ckpt = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, some_future_ts, MeasurementsProto::CAMERA);
  const auto* rad_ckpt = tracker_util::FindLatestCheckPointBeforeTimeWithType(
      track, some_future_ts, MeasurementsProto::RADAR);
  Vec2d offset_to_object_center = {0.0, 0.0};
  if (ldr_box_ckpt && ldr_box_ckpt->measurement &&
      ldr_box_ckpt->measurement->has_laser_measurement() &&
      ldr_box_ckpt->measurement->laser_measurement()
          .has_detection_bounding_box()) {
    // For LIDAR, we use fen box.
    const auto& bbox =
        ldr_box_ckpt->measurement->laser_measurement().detection_bounding_box();
    offset_to_object_center =
        Vec2d(bbox.x(), bbox.y()) - ldr_box_ckpt->key_point;
  } else if (ldr_ckpt && ldr_ckpt->measurement &&
             ldr_ckpt->measurement->has_laser_measurement() &&
             ldr_ckpt->measurement->laser_measurement()
                 .has_cluster_measurement()) {
    // For LIDAR, we use weighted centroid.
    offset_to_object_center =
        tracker_util::ComputeWeightedObstacleCentroid(
            ldr_ckpt->measurement->laser_measurement().cluster_measurement()) -
        ldr_ckpt->key_point;
  } else if (cam_ckpt && cam_ckpt->measurement &&
             cam_ckpt->measurement->has_camera3d_measurement()) {
    // For CAMERA, we use box center.
    const auto& center = cam_ckpt->measurement->camera3d_measurement().pos();
    offset_to_object_center =
        Vec2d(center.x(), center.y()) - cam_ckpt->key_point;
  } else if (rad_ckpt && rad_ckpt->measurement &&
             rad_ckpt->measurement->has_radar_measurement()) {
    // For RADAR, we use radar pos.
    const auto& radar_pos = rad_ckpt->measurement->radar_measurement().pos();
    offset_to_object_center =
        Vec2d(radar_pos.x(), radar_pos.y()) - rad_ckpt->key_point;
  } else {
    QLOG(FATAL) << "No checkpoint found for moving state history.";
  }

  if (!track.checkpoints.empty()) {
    for (int i = track.checkpoints.size() - 1; i >= 0; --i) {
      const auto& ckpt = track.checkpoints.value(i);
      tracker_util::TrackStateToMotionStateProto(ckpt, offset_to_object_center,
                                                 coordinate_converter_,
                                                 object->add_trajectory());
    }
  } else {
    QLOG(ERROR) << "Track has no checkpoints.";
  }
  return true;
}

bool Tracker::ShouldCreateEstimatorFromCarModel(
    const Track<TrackState>& track) const {
  const auto& track_state = track.track_state;
  // NOTE(zheng): If there is no heading measurement for car model at first,
  // the motion filter heading output maybe not accurate. So we do not create
  // estimator by using car model whatever the track type is if it has no bbox.
  const bool is_automotive_vehicle_type = track_state.type == MT_VEHICLE ||
                                          track_state.type == MT_MOTORCYCLIST ||
                                          track_state.type == MT_CYCLIST;
  return track_state.bounding_box.has_value() && is_automotive_vehicle_type;
}
bool Tracker::ShouldSetEstimatorToCarModelAfterClassification(
    const Track<TrackState>& track) const {
  // We can set estimator to car model if the track is classified to automotive
  // vehicle and the track satisfies one of the following conditions:
  // 1. The track has bbox in current frame, which means we can use the bbox
  // heading to init car model heading state.
  // 2. The track's estimator is car model before, which means the heading state
  // is inited correctly before.
  const auto& track_state = track.track_state;
  const bool has_valid_bbox = track_state.bounding_box.has_value();
  const bool is_car_model_before = track_state.estimator_3d.IsCarModel();
  const bool track_is_automotive_vehicle_type =
      track_state.type == MT_VEHICLE || track_state.type == MT_MOTORCYCLIST ||
      track_state.type == MT_CYCLIST;
  return track_is_automotive_vehicle_type &&
         (has_valid_bbox || is_car_model_before);
}
ObjectsProto Tracker::PublishObjects() {
  SCOPED_QTRACE("Tracker::PublishObjects");
  RunOutputGuarder();
  std::vector<ObjectProto> objects(tracks_.size());
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int track_index) {
    const auto& track = *tracks_[track_index];
    auto& mutable_track = *tracks_[track_index];
    if (track.track_state.life_state != TrackLifeState::kConfirmed) return;
    // Ignore tracks that are in ignorance zone and not under road agents type
    // (vehicle, motorcyclist, cyclist, ped).
    if (coordinate_converter_ &&
        tracker_util::IsInIgnoranceZone(*semantic_map_manager_,
                                        tracker_util::GetContour(track),
                                        *coordinate_converter_) &&
        track.track_state.type != MT_VEHICLE &&
        track.track_state.type != MT_MOTORCYCLIST &&
        track.track_state.type != MT_PEDESTRIAN &&
        track.track_state.type != MT_CYCLIST) {
      return;
    }
    // NOTE(dong): Do not publish road, mist and bird object.
    if (track.track_state.type == MT_ROAD ||
        track.track_state.type == MT_MIST ||
        track.track_state.type == MT_FLYING_BIRD) {
      return;
    }

    ObjectProto& object = objects[track_index];
    const double life_time =
        track.track_state.state_timestamp - track.track_state.first_timestamp;
    const bool is_radar_only = track.track_state.measurement_source_type ==
                               TrackMeasurementSourceType::TMST_RO;
    object.set_is_radar_only_object(is_radar_only);
    object.set_life_time(life_time);
    object.set_id(absl::StrFormat("%d", track.track_state.id));
    object.set_type(type_util::ToObjectType(track.track_state.type));

    object.set_min_z(track.track_state.min_z);
    object.set_max_z(track.track_state.max_z);
    object.set_ground_z(track.track_state.ground_z);
    // Sync icp_vel for prt.
    if (track.track_state.icp_vel) {
      Vec2dToProto(*track.track_state.icp_vel, object.mutable_icp_vel());
      mutable_track.track_state.icp_vel = std::nullopt;
    }
    // Sync fen_vel for prt.
    if (track.track_state.fen_vel) {
      Vec2dToProto(*track.track_state.fen_vel, object.mutable_fen_vel());
      mutable_track.track_state.fen_vel = std::nullopt;
    }

    const StateData s = track.track_state.estimator_3d.GetStateData();
    const double heading = s.GetYaw();
    Vec2d vel{s.GetVx(), s.GetVy()};
    if (track.track_state.moving_state == TrackState::MovingState::kStatic) {
      vel << 0., 0.;
    }

    Vec2dToProto(vel, object.mutable_vel());

    for (const Vec2d& point : track.track_state.contour.points()) {
      Vec2dToProto(point, object.add_contour());
    }
    const auto& track_bbox = FLAGS_publish_refined_bbox
                                 ? track.track_state.refined_bounding_box
                                 : track.track_state.bounding_box;
    const Vec2d object_pos =
        track_bbox
            ? track_bbox->center()
            : tracker_util::ComputeCentroid(track.track_state.contour.points());
    Vec2dToProto(object_pos, object.mutable_pos());
    // TODO(zheng, jingwei): Unify multi ref point to one anchor point, and
    // publish the anchor point state to prediction.
    switch (track.track_state.track_shape_source_type) {
      case TrackShapeSourceType::TSST_LIDAR:
        track_bbox
            ? object.set_pos_source_type(ObjectProto::PST_LIDAR_BBOX)
            : object.set_pos_source_type(ObjectProto::PST_LIDAR_CENTROID);
        break;
      case TrackShapeSourceType::TSST_CAMERA:
        object.set_pos_source_type(ObjectProto::PST_CAMERA_BBOX);
        break;
      case TrackShapeSourceType::TSST_RADAR:
        object.set_pos_source_type(ObjectProto::PST_RADAR_BBOX);
        break;
      default:
        LOG(ERROR) << "Should not be Unknown track shape source type: "
                   << track.track_state.track_shape_source_type;
    }

    object.set_observation_state(track.track_state.observation_state);

    // TODO(zheng) speed direction may not be the same as facing direction
    // (they could be opposite, or they could differ by a little, e.g. when
    // a vehicle is turning).
    object.set_yaw(heading);

    // TODO(zheng) use higher order motion models in tracking to provide
    // higher order measurements such as acceleration and curvature (or yaw
    // rate).
    Vec2d acc{s.GetAx(), s.GetAy()};
    Vec2dToProto(acc, object.mutable_accel());

    object.set_yaw_rate(track.track_state.yaw_rate);

    object.set_timestamp(track.track_state.state_timestamp);
    if (track.track_state.last_laser_timestamp != 0.0) {
      object.set_laser_timestamp(track.track_state.last_laser_timestamp);
    }

    if (track.track_state.type == MT_VEHICLE &&
        track.track_state.in_parking_area &&
        track.track_state.vel <= kParkedCarMaxSpeed) {
      object.set_parked(true);
    }
    if (track.track_state.offroad) {
      object.set_offroad(true);
    }

    // Publish position covariance matrix
    Mat2dProto* pos_cov_out = object.mutable_pos_cov();
    Mat2dToProto(track.track_state.estimator_3d.GetStatePosCov(), pos_cov_out);

    if (track.track_state.bounding_box) {
      auto* bb = object.mutable_bounding_box();
      if (FLAGS_smooth_bb) {
        const Box2d mean_bb =
            ComputeMeanBoundingBoxFromHistory(track.measurement_history);
        if (vel.squaredNorm() < Sqr(1.0)) {
          // Use the history average for parked cars.
          bb->set_x(mean_bb.center_x());
          bb->set_y(mean_bb.center_y());
          bb->set_heading(mean_bb.heading());
        } else {
          bb->set_x(track.track_state.bounding_box->center_x());
          bb->set_y(track.track_state.bounding_box->center_y());
          bb->set_heading(heading);
        }
        bb->set_width(mean_bb.width());
        bb->set_length(mean_bb.length());
      } else {
        if (FLAGS_publish_refined_bbox) {
          bb->set_x(track.track_state.refined_bounding_box->center_x());
          bb->set_y(track.track_state.refined_bounding_box->center_y());
          bb->set_heading(heading);
          bb->set_width(track.track_state.refined_bounding_box->width());
          bb->set_length(track.track_state.refined_bounding_box->length());
        } else {
          bb->set_x(track.track_state.bounding_box->center_x());
          bb->set_y(track.track_state.bounding_box->center_y());
          bb->set_heading(heading);
          bb->set_width(track.track_state.bounding_box->width());
          bb->set_length(track.track_state.bounding_box->length());
        }
      }
      object.set_bounding_box_source(ObjectProto::FIERY_EYE_NET);
    } else {
      constexpr double kMinSpeedForMovingTrack = 1.0;  // m/s

      // If the object has no det bbox, we create a bbox with different
      // method for moving and static object:
      // 1. for static object, we create a min area bbox for it, and set
      //    object's heading to min area bbox heading.
      // 2. for moving object, we create a bbox by using the object's yaw,
      //    and set object's heading to this bbox's heading.
      const auto min_area_bb =
          vel.squaredNorm() < Sqr(kMinSpeedForMovingTrack)
              ? track.track_state.contour.MinAreaBoundingBox()
              : track.track_state.contour.BoundingBoxWithHeading(object.yaw());
      auto* bb = object.mutable_bounding_box();
      bb->set_x(min_area_bb.center_x());
      bb->set_y(min_area_bb.center_y());
      bb->set_heading(min_area_bb.heading());
      bb->set_width(min_area_bb.width());
      bb->set_length(min_area_bb.length());
      object.set_bounding_box_source(ObjectProto::MIN_AREA_BOX);
      object.set_yaw(min_area_bb.heading());
    }

    if (std::isnan(object.yaw())) {
      QLOG(ERROR) << "Found an object with yaw = Nan:\n"
                  << object.DebugString();
      object.set_yaw(0.0);
      object.mutable_vel()->set_x(0.0);
      object.mutable_vel()->set_y(0.0);
    }
    // Note(zheng): When the object's velocity is zero, the acc's estimation
    // has some noise, and  PNC don't use the acc when the velocity is zero,
    // so if the velocity is zero, we set acc to zero.
    const auto& obj_vel = object.vel();
    const double object_speed = Vec2d(obj_vel.x(), obj_vel.y()).norm();
    if (object_speed < DBL_EPSILON) {
      Vec2dToProto(Vec2d::Zero(), object.mutable_accel());
    }
    // Sync moving state.
    const auto& moving_state =
        tracker_util::MovingStateToProto(track.track_state.moving_state);
    object.set_moving_state(moving_state);

    // Set moving state history.
    QCHECK(FillTrajectory(track, &object));
  });

  ObjectsProto objects_proto;
  for (auto& object : objects) {
    if (object.has_id()) {
      *objects_proto.add_objects() = std::move(object);
    }
  }
  objects_proto.set_scope(ObjectsProto::SCOPE_REAL);

  QLOG_IF_NOT_OK(WARNING, lite_module_->Publish(objects_proto));

  // NOTE(zheng): If the input measurements size is zero, we don't run
  // TrackObjects() func, so the tracker debug proto is nullptr.
  // TODO(zheng): Add debug info when the measurements size is zero.
  if (tracker_debug_proto_) {
    QLOG_IF_NOT_OK(WARNING,
                   lite_module_->Publish(std::move(tracker_debug_proto_)));
  }

  return objects_proto;
}

std::vector<const MeasurementProto*> Tracker::GetValidRadarMeasurements(
    const MeasurementsProto& measurements_proto) {
  const int num_measurements = measurements_proto.measurements_size();
  std::vector<const MeasurementProto*> results;
  results.reserve(num_measurements);

  for (const auto& measurement : measurements_proto.measurements()) {
    if (measurement.Measurement_case() == MeasurementProto::kRadarMeasurement) {
      // BANDAID(zheng): To solve radar reflection issues, we filter out the
      // radar measurements which are close to ego car temporary, until we
      // have a better radar reflection filter strategy.
      constexpr double kMaxSqrDistToFilterOut = Sqr(0.0);  // m^2
      const auto radar_m_body_pos =
          Vec2dFromProto(measurement.radar_measurement().body_pos());
      if (radar_m_body_pos.squaredNorm() < kMaxSqrDistToFilterOut) {
        continue;
      }

      // Filter out offroad radar objects.
      const auto& radar_m = measurement.radar_measurement();
      if (nullptr == coordinate_converter_ ||
          (radar_m.has_is_onroad() && !radar_m.is_onroad())) {
        continue;
      }

      // Filter static objects.
      // Filter out UNKNOWN_STATIC radar measurements and low speed radar.
      // We find that radar UNKNOWN_STATIC objects are usually not trustworthy
      // and even has speed.
      constexpr double kMinSquaredSpeedToFilterFaslePositive = Sqr(1.0);
      const auto radar_obj_vel =
          Vec2dFromProto(measurement.radar_measurement().vel());
      if (radar_obj_vel.squaredNorm() < kMinSquaredSpeedToFilterFaslePositive ||
          measurement.type() == MT_STATIC_OBJECT) {
        // Only filter aforementioned radar measurements if it is close to the
        // curb or exceeding covariance threshold.
        if (measurement.radar_measurement().has_vel_cov()) {
          Mat2d radar_obj_vel_cov;
          Mat2dFromProto(measurement.radar_measurement().vel_cov(),
                         &radar_obj_vel_cov);
          const double min_radar_cov_for_low_speed_object =
              tracker_util::IsCornerRadar(
                  measurement.radar_measurement().radar_id())
                  ? 2.0
                  : 1.0;
          if (radar_obj_vel_cov.maxCoeff() >
              Sqr(min_radar_cov_for_low_speed_object)) {
            continue;
          }
        }
        // Note(yan): Now we set the threshold as 200 m as low speed radar are
        // less trustworthy as slow moving objects can be classfied as static
        // objects which leads to safety issues.
        constexpr double kMinDisToCurbThreshold = 2.0;
        const auto radar_m_pos_smooth =
            Vec2dFromProto(measurement.radar_measurement().pos());
        if (!tracker_util::IsInCautiousRegion(pose_, radar_m_pos_smooth) ||
            !(tracker_util::DistanceToCurb(*semantic_map_manager_,
                                           radar_m_pos_smooth) <
              -kMinDisToCurbThreshold)) {
          continue;
        }
      }
      results.push_back(&measurement);
    }
  }
  return results;
}

bool IsTrackOnHighWay(const Track<TrackState>& track,
                      const SemanticMapManager& semantic_map_manager,
                      const CoordinateConverter& coordinate_converter) {
  // Find out if tracks are on high way.
  const auto* lane_ptr = semantic_map_manager.GetNearestLaneInfoAtLevel(
      coordinate_converter.GetLevel(), track.track_state.contour.centroid());
  constexpr double kMinLimitSpeedForHighWay = 70.0 / 3.6;  // m/s
  return lane_ptr && lane_ptr->speed_limit > kMinLimitSpeedForHighWay;
}

void Tracker::TrackObjects(
    const VehiclePose& pose, const CoordinateConverter& coordinate_converter,
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    double timestamp, MeasurementsProto measurements,
    std::shared_ptr<const labeling::LabelFrameProto> label_frame,
    std::shared_ptr<const SensorFovsProto> sensor_fovs_proto) {
  // Skip the empty measurements.
  if (measurements.measurements_size() == 0) {
    return;
  }
  // Find out if tracks are on high way.
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    auto& track = *tracks_[i];
    if (coordinate_converter_) {
      track.track_state.is_on_highway = IsTrackOnHighWay(
          track, *semantic_map_manager_, *coordinate_converter_);
    }
  });

  const auto group_type = measurements.group_type();
  // Add some info for trace debugging, the max allowed string
  // length in trace is 65, so we should save the debug info in
  // two values in case of it beyond the max length.
  const int measurement_num = measurements.measurements_size();

  // Add some info for trace debugging.
  const double header_time = 1e-6 * measurements.header().timestamp();
  const std::string m_type =
      MeasurementsProto::GroupType_Name(measurements.group_type());

  const std::string m_info =
      absl::StrFormat("header_time: %.3f, type: %s, num: %d", header_time,
                      m_type, measurement_num);
  const std::string min_max_timestamp =
      absl::StrFormat("min: %.3f, max: %.3f", measurements.min_timestamp(),
                      measurements.max_timestamp());

  SCOPED_QTRACE_ARG2("Tracker::TrackObjects", "m_info", m_info, "min_max_time",
                     min_max_timestamp);
  latest_label_frame_ = label_frame;

  prev_timestamp_ = timestamp;
  pose_ = pose;
  pose_inv_ = pose_.ToTransform().Inverse();

  // Maybe visualize measurements and tracks.
  MaybeRenderMeasurements(measurements);
  MaybeRenderTracks(tracks_, pose_);
  if (latest_label_frame_) {
    MaybeRenderLabelFrameCvs(*latest_label_frame_);
  }

  // Associate each measurements to current tracks.
  AssociateMeasurementsAndUpdateTracksWithRollBack(
      camera_images, coordinate_converter, timestamp, std::move(measurements));

  // Delete expired tracks.
  RemoveExpiredTracks(group_type, timestamp, sensor_fovs_proto);
}

// Update contour, obstacle centers and bounding box.
void Tracker::UpdateTrackExtents(const LaserMeasurementProto& laser_measurement,
                                 double timestamp, Track<TrackState>* track) {
  if (ShouldUseVisionPredcitedShape(*track, laser_measurement)) {
    PredictTrackShapeInfo(laser_measurement.cluster_measurement().timestamp(),
                          track);
    track->track_state.track_shape_source_type =
        TrackShapeSourceType::TSST_CAMERA;

  } else {
    if (track->track_state.observation_state ==
        ObservationState::OS_PARTIALLY_OBSERVED) {
      const double poly_area =
          geometry_util::ToPolygon2d(laser_measurement.contour()).area();
      constexpr double kMinPolyAreaToUseContourInPoArea = 4.0;
      // NOTE(zheng): Sometimes the big vehicle's heading is not very accurate,
      // when we use bbox as contour in that case, the contour may invade the
      // neighbor lanes which would cause the ego car hard break, so we still
      // publish contour when the vehicle is static and its' contour area is
      // bigger than 4.0 m^2. NOTE(zheng): When we have fine-tune the big
      // vehicle's heading by using L-Shape, we can remove this strategy.
      if (track->track_state.moving_state == TrackState::MovingState::kStatic &&
          poly_area > kMinPolyAreaToUseContourInPoArea) {
        track->track_state.contour =
            geometry_util::ToPolygon2d(laser_measurement.contour());
      } else if (laser_measurement.has_detection_bounding_box()) {
        const auto& bbox_proto = laser_measurement.detection_bounding_box();
        auto bbox =
            track->track_state.estimator_3d.IsCarModel()
                ? Box2d(Vec2d(bbox_proto.x(), bbox_proto.y()),
                        track->track_state.estimator_3d.GetStateData().GetYaw(),
                        bbox_proto.length(), bbox_proto.width())
                : Box2d(bbox_proto);
        track->track_state.contour = Polygon2d(bbox);
      } else {
        track->track_state.contour = PredictContour(timestamp, *track);
      }
    } else {
      track->track_state.contour =
          geometry_util::ToPolygon2d(laser_measurement.contour());
    }
    if (laser_measurement.has_detection_bounding_box()) {
      track->track_state.bounding_box =
          Box2d(laser_measurement.detection_bounding_box());
      track->track_state.refined_bounding_box =
          Box2d(laser_measurement.refined_bounding_box());
    } else {
      track->track_state.bounding_box = std::nullopt;
      track->track_state.refined_bounding_box = std::nullopt;
    }

    track->track_state.track_shape_source_type =
        TrackShapeSourceType::TSST_LIDAR;
  }

  track->track_state.observation_state = laser_measurement.observation_state();
  // Sync min_z, max_z and ground_z value.
  if (laser_measurement.has_cluster_measurement()) {
    const auto& cluster = laser_measurement.cluster_measurement();
    track->track_state.min_z = cluster.min_z();
    track->track_state.max_z = cluster.max_z();
    track->track_state.ground_z = cluster.ground_z();
  }
}

bool Tracker::UpdateAnchorPoint(const MeasurementProto& measurement,
                                Track<TrackState>* track) const {
  if (nullptr == track) return false;

  track->track_state.measurement = &measurement;

  if (measurement.has_laser_measurement()) {
    if (measurement.laser_measurement().has_cluster_measurement()) {
      track->track_state.anchor_point =
          tracker_util::ComputeWeightedObstacleCentroid(
              measurement.laser_measurement().cluster_measurement());
    } else {
      QLOG(ERROR) << "Laser measurement has no cluster measurement.";
    }
  } else if (measurement.has_radar_measurement()) {
    track->track_state.anchor_point =
        Vec2dFromProto(measurement.radar_measurement().pos());
  } else if (measurement.has_camera3d_measurement()) {
    track->track_state.anchor_point =
        Vec2dFromProto(measurement.camera3d_measurement().pos());
  } else {
    QLOG(ERROR) << "Unknown measurement type.";
  }

  return true;
}

void Tracker::CreateNewTracksFromMeasurements(
    const std::vector<const MeasurementProto*>& measurements) {
  SCOPED_QTRACE("Tracker::CreateNewTracksFromMeasurements");
  for (const auto* m : measurements) {
    // We only create new track from radar or laser measurement.
    if ((m->has_camera3d_measurement() &&
         !track_measurement_source_type_manager_.ShouldCreateTrackFromVision(
             *m, pose_, pose_inv_, IsFirstFrame())) ||
        (m->has_radar_measurement() &&
         !TMSTM::ShouldCreateTrackFromRadar(*m, pose_, IsFirstFrame()))) {
      continue;
    }

    tracks_.emplace_back(std::make_unique<Track<TrackState>>());
    auto& track = *tracks_.back();
    track.track_state.id = GenerateNewTrackId();
    track.track_state.first_timestamp = m->timestamp();
    track.track_state.last_timestamp = m->timestamp();
    track.track_state.state_timestamp = m->timestamp();
    track.track_state.last_laser_timestamp = m->timestamp();
    track.track_state.type = m->type();
    track.track_state.life_state = TrackLifeState::kInit;

    UpdateAnchorPoint(*m, &track);

    TrackState::RefPoint ref_point;
    if (m->has_laser_measurement()) {
      const auto& laser_measurement = m->laser_measurement();
      UpdateTrackExtents(laser_measurement, m->timestamp(), &track);
      // Set initial heading if detection exists
      if (laser_measurement.has_detection_bounding_box()) {
        track.track_state.heading =
            laser_measurement.detection_bounding_box().heading();
      }

      // NOTE(zheng): For barrier or vegetation type, we needn't immplement
      // association , and should promote it immediately. But nowadays the
      // llnet may give a false positive on barrier or vegetation type. So, to
      // reduce false positive, we also need to immplement association, and
      // and don't promote it at the first three frames for the barrier or
      // vegetation that comes from llnet. But for vegetation or barrier comes
      // form semantic map, we neen't immplement asociation and should promote
      // it immediately. So we need a is_certain_static_track type to judge
      // the barrier or vegetation type is comes from llnet or semantic map
      // zone.

      if (tracker_util::IsCertainStaticMeasurement(*m)) {
        track.track_state.is_certain_static_track = true;
      }
      ref_point = ChooseAndComputeRefPoint(laser_measurement, track);
      track.track_state.measurement_source_type =
          TrackMeasurementSourceType::TMST_LO;
    } else if (m->has_radar_measurement()) {
      const auto& radar_measurement = m->radar_measurement();
      const Vec2d m_pos = Vec2dFromProto(radar_measurement.pos());
      const double heading =
          NormalizeAngle(Vec2dFromProto(radar_measurement.vel()).FastAngle());
      constexpr double kRadarObjLength = 1.0;  // m
      constexpr double kRadarObjWidth = 1.0;   // m
      Box2d bbox(m_pos, heading, kRadarObjLength, kRadarObjWidth);
      Polygon2d polygon(bbox);
      track.track_state.bounding_box = bbox;
      track.track_state.refined_bounding_box = bbox;
      track.track_state.contour = polygon;
      ref_point.pos = m_pos;
      ref_point.type = TrackState::RefPoint::Type::kContourCentroid;
      track.track_state.type = MT_UNKNOWN;
      track.track_state.measurement_source_type =
          TrackMeasurementSourceType::TMST_RO;

      track.track_state.track_shape_source_type =
          TrackShapeSourceType::TSST_RADAR;
      track.track_state.observation_state = ObservationState::OS_UNKNOWN;
    } else if (m->has_camera3d_measurement()) {
      const auto& camera_measurement = m->camera3d_measurement();
      const Vec2d m_pos = Vec2dFromProto(camera_measurement.pos());
      const double heading = camera_measurement.heading();
      const double length = camera_measurement.length();
      const double width = camera_measurement.width();

      Box2d bbox(m_pos, heading, length, width);
      Polygon2d polygon(bbox);
      track.track_state.bounding_box = bbox;
      track.track_state.refined_bounding_box = bbox;
      track.track_state.contour = polygon;
      ref_point.pos = m_pos;
      ref_point.type = TrackState::RefPoint::Type::kBBCenter;
      track.track_state.measurement_source_type =
          TrackMeasurementSourceType::TMST_VO;
      track.track_state.life_state = TrackLifeState::kInit;

      track.track_state.track_shape_source_type =
          TrackShapeSourceType::TSST_CAMERA;
      track.track_state.observation_state = ObservationState::OS_UNKNOWN;
    }

    const bool should_use_car_model = ShouldCreateEstimatorFromCarModel(track);
    if (coordinate_converter_) {
      track.track_state.is_on_highway = IsTrackOnHighWay(
          track, *semantic_map_manager_, *coordinate_converter_);
    }
    const MotionFilterParamType param_type =
        GetMotionFilterParamType(track, should_use_car_model);
    track.track_state.estimator_3d =
        CreateEstimator(track, should_use_car_model, param_type);
    track.track_state.motion_filter_param_type = param_type;
    track.track_state.motion_filter_type =
        GetMotionFilterTypeFromParamType(param_type);

    // NOTE(zheng): For barrier or vegetation type, we needn't immplement
    // association , and should promote it immediately. But nowadays the llnet
    // may give a false positive on barrier or vegetation type. So, to reduce
    // false positive, we also need to immplement association, and and don't
    // promote it at the first three frames for the barrier or vegetation that
    // comes from llnet. But for vegetation or barrier comes form semantic
    // map, we neen't immplement asociation and should promote it immediately.
    // So we need a is_certain_static_track type to judge the barrier or
    // vegetation type is comes from llnet or semantic map zone.

    // NOTE(jiawei): Object of car model originates from laser; object of
    // point model originates from laser or radar(unknown type).
    if (should_use_car_model) {
      CarState state;
      state.x() = ref_point.pos.x();
      state.y() = ref_point.pos.y();
      if (m->has_laser_measurement()) {
        // QCHECK(m->has_laser_measurement());
        const auto& laser_measurement = m->laser_measurement();
        if (laser_measurement.has_detection_bounding_box()) {
          state.yaw() = track.track_state.bounding_box->heading();
        }
      } else if (track.track_state.measurement_source_type ==
                 TrackMeasurementSourceType::TMST_VO) {
        QCHECK(m->has_camera3d_measurement());
        state.yaw() = m->camera3d_measurement().heading();
        BBoxState bbox_meas;
        bbox_meas.length() = m->camera3d_measurement().length();
        bbox_meas.width() = m->camera3d_measurement().width();
        track.track_state.estimator_3d.InitExtent(bbox_meas);
      }

      track.track_state.estimator_3d.Init(StateData(state), m->timestamp());
      track.track_state.heading_voter.Enable();

    } else {  // point model
      PointState state;
      state.x() = ref_point.pos.x();
      state.y() = ref_point.pos.y();
      if (track.track_state.measurement_source_type ==
          TrackMeasurementSourceType::TMST_RO) {
        QCHECK(m->has_radar_measurement());
        const auto vel = m->radar_measurement().vel();
        state.vx() = vel.x();
        state.vy() = vel.y();
      } else if (track.track_state.measurement_source_type ==
                 TrackMeasurementSourceType::TMST_VO) {
        QCHECK(m->has_camera3d_measurement());
        const auto vel = m->camera3d_measurement().vel();
        state.vx() = vel.x();
        state.vy() = vel.y();
        BBoxState bbox_meas;
        bbox_meas.length() = m->camera3d_measurement().length();
        bbox_meas.width() = m->camera3d_measurement().width();
        track.track_state.estimator_3d.InitExtent(bbox_meas);
      }
      track.track_state.estimator_3d.Init(StateData(state), m->timestamp());
    }

    const StateData s = track.track_state.estimator_3d.GetStateData();
    track.track_state.vel = s.GetVel();
    track.track_state.heading = s.GetYaw();
    track.track_state.ref_point = ref_point;

    track.measurement_history.PushBackAndClearStale(
        m->timestamp(), m, kMaxMeasurementHistoryBufferLength);

    if (track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
            track, pose_, m->timestamp())) {
      track.track_state.life_state = TrackLifeState::kConfirmed;
    }

    // Save checkpoints.
    SaveCheckPoints(&track);
  }
}

void Tracker::AdjustTrackAfterClassification() {
  SCOPED_QTRACE("Tracker::AdjustTrackAfterClassification");

  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    auto& track = *tracks_[i];
    const auto should_use_car_model =
        ShouldSetEstimatorToCarModelAfterClassification(track);
    if (coordinate_converter_) {
      track.track_state.is_on_highway = IsTrackOnHighWay(
          track, *semantic_map_manager_, *coordinate_converter_);
    }
    const auto select_param_type =
        GetMotionFilterParamType(track, should_use_car_model);
    const auto select_motion_filter_type =
        GetMotionFilterTypeFromParamType(select_param_type);
    // NOTE(jiawei): If both are false, they are both point model, we can
    // return either.
    if (select_motion_filter_type == track.track_state.motion_filter_type) {
      return;
    }
    // TODO(jingwei) What's going on here? Is it a reset?
    auto select_estimator =
        CreateEstimator(track, should_use_car_model, select_param_type);
    StateData s = track.track_state.estimator_3d.GetStateData();
    const double origin_heading = s.GetYaw();
    if (track.track_state.estimator_3d.IsPointModel() &&
        select_estimator.IsCarModel()) {
      s.car().state_cov() = select_estimator.GetStateData().car().state_cov();
      s.PointToCar();
      const auto& m_history = track.measurement_history;
      const int num_m = m_history.size();
      const double last_m_timestamp = m_history.back_time();
      constexpr double kDurationAsValidMeasurement = 0.5;  // s
      int last_laser_measurement_ind = -1;
      for (int j = m_history.GetIndexWithTimeAtLeast(
               last_m_timestamp - kDurationAsValidMeasurement);
           j < num_m; ++j) {
        const auto* m = m_history.value(j);
        // Find the latest laser measurement with detection bbox.
        if (m->has_laser_measurement() &&
            m->laser_measurement().has_detection_bounding_box()) {
          last_laser_measurement_ind = j;
        }
      }
      constexpr double kMinVelocityToHaveStableHeading = 2.0;  // m/s
      if (s.GetVel() <= kMinVelocityToHaveStableHeading &&
          last_laser_measurement_ind != -1) {
        const auto* m = m_history.value(last_laser_measurement_ind);
        const auto& laser_m = m->laser_measurement();
        if (laser_m.has_detection_bounding_box()) {
          s.car().state().yaw() = laser_m.detection_bounding_box().heading();
        }
      }

    } else if (track.track_state.estimator_3d.IsCarModel() &&
               select_estimator.IsPointModel()) {
      s.point().state_cov() =
          select_estimator.GetStateData().point().state_cov();
      s.CarToPoint();
    } else if (track.track_state.estimator_3d.IsCarModel() &&
               select_estimator.IsCarModel()) {
      QLOG(FATAL) << "No type switch between car models."
                  << "\noriginal motion filter type: "
                  << track.track_state.motion_filter_type
                  << "\nselect motion filter type: "
                  << select_motion_filter_type;
    }
    select_estimator.InitStateAndCov(s, track.track_state.last_timestamp);
    track.track_state.estimator_3d = select_estimator;
    track.track_state.motion_filter_param_type = select_param_type;
    track.track_state.motion_filter_type = select_motion_filter_type;
    // Unify ref point type to current heading.
    if (IsCornerOrFaceCenterRefType(track.track_state.ref_point.type)) {
      track.track_state.ref_point.type = TransformRefTypeWhenYawFlipped(
          track.track_state.ref_point.type, origin_heading,
          track.track_state.estimator_3d.GetStateData().GetYaw());
    }

    track.track_state.measurement_source_type =
        TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(track);
  });
}

bool Tracker::ShouldSetZeroRefPointResetByRefPointPositionErrorCheck(
    const Vec2d& cur_ref_point_pos, const TrackState& track_state,
    const double cur_laser_timestamp, const Vec2d& track_ref_point_pos) const {
  // If the diff is less than 1 m then do not reset.
  constexpr double kMinRefPointPositionError = 1.0;  // m
  const auto pred_state_date =
      track_state.estimator_3d.ComputePrediction(cur_laser_timestamp);
  const auto offset =
      pred_state_date.GetStatePos() - track_state.estimator_3d.GetStatePos();
  const auto pred_track_ref_point_pos = track_ref_point_pos + offset;
  const auto diff = pred_track_ref_point_pos - cur_ref_point_pos;
  // Only perform this logic for vehicle type.
  return (track_state.type == MeasurementType::MT_VEHICLE &&
          diff.norm() < kMinRefPointPositionError);
}

std::optional<Vec2d> Tracker::ComputeRefPointPosGivenType(
    const LaserMeasurementProto& laser_measurement,
    const TrackState::RefPoint::Type type) const {
  switch (type) {
    case TrackState::RefPoint::kBBCenter:
      if (laser_measurement.has_detection_bounding_box()) {
        const auto& box = laser_measurement.detection_bounding_box();
        return Vec2d(box.x(), box.y());
      } else {
        return std::nullopt;
      }
    case TrackState::RefPoint::kNearestObstacleCenter:
      if (laser_measurement.has_cluster_measurement() &&
          (laser_measurement.cluster_measurement().obstacle_info_size() > 0 ||
           laser_measurement.cluster_measurement()
                   .obstacle_centers_deprecated_size() > 0)) {
        const auto& cluster_measurement =
            laser_measurement.cluster_measurement();
        // TODO(yu): Put vehicle pose into measurements.

        const auto anchor_pos = GetAnchorPosForRefPoint();

        // Find the closest obstacle center as the ref point.
        Vec2d ref_point;
        double min_dist2 = std::numeric_limits<double>::max();
        for (const auto& info : cluster_measurement.obstacle_info()) {
          const Vec2d c = Vec2dFromProto(info.center());
          const double dist2 =
              (c - Vec2d(anchor_pos.x(), anchor_pos.y())).squaredNorm();
          if (dist2 < min_dist2) {
            ref_point = c;
            min_dist2 = dist2;
          }
        }
        for (const auto& obstacle_center :
             cluster_measurement.obstacle_centers_deprecated()) {
          const Vec2d c = Vec2dFromProto(obstacle_center);
          const double dist2 =
              (c - Vec2d(anchor_pos.x(), anchor_pos.y())).squaredNorm();
          if (dist2 < min_dist2) {
            ref_point = c;
            min_dist2 = dist2;
          }
        }
        return ref_point;
      } else {
        return std::nullopt;
      }

    case TrackState::RefPoint::kWeightedObstacleCentroid: {
      // Use the weighted obstacle centroid as the ref point.
      QCHECK(laser_measurement.has_cluster_measurement());
      return tracker_util::ComputeWeightedObstacleCentroid(
          laser_measurement.cluster_measurement());
    }

    case TrackState::RefPoint::kContourCentroid: {
      // Use the contour centroid as the ref point.
      QCHECK(laser_measurement.has_cluster_measurement());
      return tracker_util::ComputeCentroid(
          laser_measurement.cluster_measurement());
    }

    // Compute given type corner ref point.
    case TrackState::RefPoint::kTopRightCorner:
    case TrackState::RefPoint::kTopLeftCorner:
    case TrackState::RefPoint::kBottomLeftCorner:
    case TrackState::RefPoint::kBottomRightCorner:
      return ComputeRefPointPosGivenTypeForCorners(laser_measurement, type);

    // Compute face center as ref point.
    case TrackState::RefPoint::kFrontFaceCenter:
    case TrackState::RefPoint::kLeftFaceCenter:
    case TrackState::RefPoint::kRearFaceCenter:
    case TrackState::RefPoint::kRightFaceCenter:
      return ComputeRefPointPosGivenTypeForFaceCenters(laser_measurement, type);
    default:
      return std::nullopt;
  }
}

std::optional<Vec2d> Tracker::ComputeRefPointOffset(
    const TrackState::RefPoint& ref_point, const Track<TrackState>& track,
    const double& cur_laser_timestamp) const {
  const TrackState::RefPoint::Type ref_point_type = ref_point.type;
  // Return zero offset if ref point type is the same as track's current ref
  // point type.
  if (track.track_state.ref_point.type == ref_point_type) {
    return Vec2d(0., 0.);
  }
  // Find the laser measurement corresponding to the current track ref point.
  const int num_m = track.checkpoints.size();
  for (int i = num_m - 1; i >= 0; --i) {
    const TrackState& track_state = track.checkpoints.value(i);
    const auto* m = track_state.measurement;
    // Skip if the measurement is nullptr or the current measurement
    // is not laser measurement or obsolete.
    if (m == nullptr || !m->has_laser_measurement()) {
      continue;
    }
    const double ts = m->timestamp();
    constexpr double kMaxValidMeasurementTimeDelay = 0.3;  // s
    const auto& laser_m = m->laser_measurement();
    if (track.track_state.state_timestamp - ts >
        kMaxValidMeasurementTimeDelay) {
      break;
    }

    // Flip ref point type when the type is corner and the heading angle
    // between track and measurement great than thershhold.Use the measurement
    // heading as baseline.
    auto flipped_ref_point_type = ref_point_type;
    if (laser_m.has_detection_bounding_box() &&
        IsCornerOrFaceCenterRefType(flipped_ref_point_type)) {
      flipped_ref_point_type = TransformRefTypeWhenYawFlipped(
          flipped_ref_point_type, track.track_state.heading,
          laser_m.detection_bounding_box().heading());
    }

    auto flipped_track_ref_point_type = track.track_state.ref_point.type;
    if (laser_m.has_detection_bounding_box() &&
        IsCornerOrFaceCenterRefType(track.track_state.ref_point.type)) {
      flipped_track_ref_point_type = TransformRefTypeWhenYawFlipped(
          track.track_state.ref_point.type, track.track_state.heading,
          laser_m.detection_bounding_box().heading());
    }
    // Get the ref point position with the current track ref point type.
    const auto pos_type_cur_or =
        ComputeRefPointPosGivenType(laser_m, flipped_track_ref_point_type);
    // Get the ref point position with given ref point type.
    const auto pos_type_given_or =
        ComputeRefPointPosGivenType(laser_m, flipped_ref_point_type);
    if (pos_type_cur_or.has_value() && pos_type_given_or.has_value()) {
      if (ShouldSetZeroRefPointResetByRefPointPositionErrorCheck(
              ref_point.pos, track_state, cur_laser_timestamp,
              *pos_type_cur_or)) {
        return Vec2d(0., 0.);
      } else {
        return *pos_type_given_or - *pos_type_cur_or;
      }
    }
  }
  // Null opt if the current track doesn't have a laser m that could obtain
  // ref point of both types within a certain time range.
  return std::nullopt;
}

TrackState::RefPoint Tracker::ChooseAndComputeRefPoint(
    const LaserMeasurementProto& laser_measurement,
    const Track<TrackState>& track) const {
  // Only use face center or corner for vehicle.
  if (track.track_state.type == MT_VEHICLE &&
      laser_measurement.has_detection_bounding_box()) {
    // Only the measurement is in the right front/behind of AV, we
    // use face center.
    const auto face_center_or = ChooseAndComputeFaceCenters(laser_measurement);
    if (face_center_or) {
      return *face_center_or;
    }
    // Compute and select which corner to use.
    const auto corner_or = ChooseAndComputeCornerRefPoint(laser_measurement);
    if (corner_or) {
      return *corner_or;
    }
  }
  // NOTE(zheng): Bbox center is more stable for cyc.
  if ((track.track_state.type == MT_MOTORCYCLIST ||
       track.track_state.type == MT_CYCLIST) &&
      laser_measurement.has_detection_bounding_box()) {
    const auto center_or = ComputeRefPointPosGivenType(
        laser_measurement, TrackState::RefPoint::kBBCenter);
    if (center_or) {
      return {TrackState::RefPoint::kBBCenter, *center_or};
    }
  }
  const auto nearest_obstacle_center_or = ComputeRefPointPosGivenType(
      laser_measurement, TrackState::RefPoint::kNearestObstacleCenter);
  if (nearest_obstacle_center_or) {
    const auto anchor_pos = GetAnchorPosForRefPoint();
    constexpr double kMinDistanceToUseNearestObstacleCenterRefPoint =
        80.0;  // m.
    if ((*nearest_obstacle_center_or - Vec2d(anchor_pos.x(), anchor_pos.y()))
            .squaredNorm() >
        Sqr(kMinDistanceToUseNearestObstacleCenterRefPoint)) {
      return {TrackState::RefPoint::kNearestObstacleCenter,
              *nearest_obstacle_center_or};
    }
  }

  const auto contour_centroid_or = ComputeRefPointPosGivenType(
      laser_measurement, TrackState::RefPoint::kContourCentroid);
  QCHECK(contour_centroid_or.has_value());
  return {TrackState::RefPoint::kContourCentroid, *contour_centroid_or};
}

std::optional<TrackState::RefPoint> Tracker::ChooseAndComputeCornerRefPoint(
    const LaserMeasurementProto& laser_measurement) const {
  if (!laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  }
  const auto box2d = Box2d(laser_measurement.detection_bounding_box());

  TrackState::RefPoint ref_point;
  // Compute vehicle coord measurement center.
  const auto corners = box2d.GetCornersCounterClockwise();
  // The index of corners we get form GetCornersCounterClockwise function
  // represents, 0: TopLeftCorner, 1: BottomLeftCorner, 2: BottomRightCorner,
  // 3: TopRightCorner
  QCHECK_EQ(corners.size(), 4);
  // Compute nearest corner.
  double min_dist2 = std::numeric_limits<double>::max();
  int min_dist_index = -1;
  const auto anchor_pos = GetAnchorPosForRefPoint();
  for (int i = 0; i < corners.size(); ++i) {
    const Vec2d corner2av_vec = {corners[i].x() - anchor_pos.x(),
                                 corners[i].y() - anchor_pos.y()};
    const double dist2 = corner2av_vec.squaredNorm();
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      ref_point.pos = corners[i];
      min_dist_index = i;
    }
  }
  // Get conrner type.
  QCHECK_LT(min_dist_index, std::size(kCornerRefPointTypeTable));
  ref_point.type = kCornerRefPointTypeTable[min_dist_index];

  return ref_point;
}

std::optional<Vec2d> Tracker::ComputeRefPointPosGivenTypeForCorners(
    const LaserMeasurementProto& laser_measurement,
    const TrackState::RefPoint::Type& ref_type) const {
  if (!laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  }
  // Compute vehicle coord measurement center.
  const auto box2d = Box2d(laser_measurement.detection_bounding_box());

  // The index of corners we get form GetCornersCounterClockwise function
  // represents, 0: TopLeftCorner, 1: BottomLeftCorner, 2: BottomRightCorner,
  // 3: TopRightCorner
  const auto corners = box2d.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  // Compute given type corner.
  const auto corner_index =
      std::distance(std::begin(kCornerRefPointTypeTable),
                    std::find(std::begin(kCornerRefPointTypeTable),
                              std::end(kCornerRefPointTypeTable), ref_type));
  if (corner_index >= 4) {
    QLOG(ERROR) << "Has not found given type corner.";
    return std::nullopt;
  }
  return corners[corner_index];
}

std::optional<TrackState::RefPoint> Tracker::ChooseAndComputeFaceCenters(
    const LaserMeasurementProto& laser_measurement) const {
  if (!laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  }

  const auto box2d = Box2d(laser_measurement.detection_bounding_box());

  const Vec2d pos_av = Vec2d(pose_.x, pose_.y);
  // NOte(zheng): Only the angle diff of measurement and vehicle coord x axis
  // is small than kMaxAllowedYawDiffFromXAxis, we select face center.
  const double m_heading = laser_measurement.detection_bounding_box().heading();
  const Vec2d dir_in_vehcile_coord =
      Vec2d(cos(m_heading), sin(m_heading))
          .FastRotate(static_cast<double>(-pose_.yaw));
  const double yaw_in_vehicle_coord =
      NormalizeAngle(dir_in_vehcile_coord.FastAngle());
  constexpr double kMaxAllowedYawDiffFromXAxis = M_PI / 18.0;
  if (std::abs(yaw_in_vehicle_coord) > kMaxAllowedYawDiffFromXAxis &&
      std::abs(yaw_in_vehicle_coord) < (M_PI - kMaxAllowedYawDiffFromXAxis)) {
    return std::nullopt;
  }

  const Vec2d pos_m = Vec2d(laser_measurement.detection_bounding_box().x(),
                            laser_measurement.detection_bounding_box().y());
  const double dist2 = (pos_av - pos_m).squaredNorm();
  constexpr double kMinAllowedDistanceFromAv = 40.0;
  if (dist2 < Sqr(kMinAllowedDistanceFromAv)) {
    return std::nullopt;
  }
  // Compute vehicle coord measurement center.
  const auto corners = box2d.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  std::vector<Vec2d> face_centers(corners.size());
  // Front face center.
  face_centers[0] = (corners[0] + corners[3]) * 0.5;
  // Left face center.
  face_centers[1] = (corners[0] + corners[1]) * 0.5;
  // Rear face center.
  face_centers[2] = (corners[1] + corners[2]) * 0.5;
  // Right face center.
  face_centers[3] = (corners[2] + corners[3]) * 0.5;
  // Compute if there is a stable face center.
  double min_vehicle_coord_y = std::numeric_limits<double>::max();
  double max_vehicle_coord_y = -std::numeric_limits<double>::max();
  for (auto corner : corners) {
    Vec2d vehicle_coord_corner = (corner - pos_av);
    vehicle_coord_corner =
        vehicle_coord_corner.FastRotate(static_cast<double>(-pose_.yaw));
    min_vehicle_coord_y =
        std::min(min_vehicle_coord_y, vehicle_coord_corner.y());
    max_vehicle_coord_y =
        std::max(max_vehicle_coord_y, vehicle_coord_corner.y());
  }
  // Note(zheng): Only the measurement is at the right behind or right
  // front of the AV, we select the nearest face center.
  if (min_vehicle_coord_y * max_vehicle_coord_y >= 0.0) {
    return std::nullopt;
  }

  // Compute nearest face center.
  int min_dist_index = -1;
  double min_dist = std::numeric_limits<double>::max();
  TrackState::RefPoint ref_point;

  for (size_t i = 0; i < face_centers.size(); ++i) {
    Vec2d corner2av_vec = {face_centers[i].x() - pose_.x,
                           face_centers[i].y() - pose_.y};
    double dist = corner2av_vec.squaredNorm();
    if (dist < min_dist) {
      min_dist = dist;
      ref_point.pos = face_centers[i];
      min_dist_index = i;
    }
  }
  // Get conrner type.
  QCHECK_LT(min_dist_index, std::size(kFaceRefPointTypeTable));
  ref_point.type = kFaceRefPointTypeTable[min_dist_index];

  return ref_point;
}

std::optional<Vec2d> Tracker::ComputeRefPointPosGivenTypeForFaceCenters(
    const LaserMeasurementProto& laser_measurement,
    const TrackState::RefPoint::Type& ref_type) const {
  if (!laser_measurement.has_detection_bounding_box()) {
    return std::nullopt;
  }
  const auto box2d = Box2d(laser_measurement.detection_bounding_box());

  // Compute vehicle coord measurement center.
  const auto corners = box2d.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  std::vector<Vec2d> face_centers(corners.size());
  face_centers[0] = (corners[0] + corners[3]) * 0.5;
  face_centers[1] = (corners[3] + corners[2]) * 0.5;
  face_centers[2] = (corners[2] + corners[1]) * 0.5;
  face_centers[3] = (corners[1] + corners[0]) * 0.5;

  // Compute given type corner.
  const auto face_center_index =
      std::distance(std::begin(kFaceRefPointTypeTable),
                    std::find(std::begin(kFaceRefPointTypeTable),
                              std::end(kFaceRefPointTypeTable), ref_type));
  if (face_center_index >= 4) {
    QLOG(ERROR) << "Has not found given type corner.";
    return std::nullopt;
  }
  return face_centers[face_center_index];
}

void Tracker::UpdateTracksFromMeasurementsWithRollback(
    const std::vector<std::vector<const MeasurementProto*>>&
        matched_measurements_per_track) {
  SCOPED_QTRACE("Tracker::UpdateTracksFromMeasurementsWithRollback");
  QCHECK_EQ(tracks_.size(), matched_measurements_per_track.size());
  double latest_measurement_timestamp = std::numeric_limits<double>::lowest();
  for (const auto& measurements : matched_measurements_per_track) {
    if (measurements.empty()) {
      continue;
    }
    for (const auto* m : measurements) {
      if (latest_measurement_timestamp < m->timestamp()) {
        latest_measurement_timestamp = m->timestamp();
      }
    }
  }

  // If no measurements associated with any track, return.
  if (latest_measurement_timestamp <= 0.) {
    return;
  }

  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    auto& track = *tracks_[i];
    const auto& measurements = matched_measurements_per_track[i];
    if (measurements.empty()) {
      return;
    }
    // If no measurement is associated with the current track and last
    // timestamp of the current track is smaller than the latest timestamp of
    // all of the measurements, we'll predict and update the track to the
    // latest measurement timestamp.
    if (measurements.empty() &&
        track.track_state.state_timestamp < latest_measurement_timestamp) {
      PredictAndUpdateTrack(latest_measurement_timestamp, &track);
      return;
    }
    MeasurementProtoPtrSet measurements_to_process;
    // Note(zheng): A track may associate multiple radar measurements, we
    // select the closest measurement to update the velocity.
    // TODO(zheng): Run with all radar measurements.
    std::map<double, const MeasurementProto*> radar_measurements_to_process;

    for (const auto* measurement : measurements) {
      auto* to_process_m = measurement;
      if (!to_process_m->has_radar_measurement()) {
        measurements_to_process.insert(to_process_m);
      } else {
        double dist2av =
            Vec2dFromProto(to_process_m->radar_measurement().body_pos())
                .squaredNorm();
        radar_measurements_to_process.insert({dist2av, to_process_m});
      }
    }

    if (!radar_measurements_to_process.empty()) {
      const auto radar_m_it = radar_measurements_to_process.begin();
      measurements_to_process.insert(radar_m_it->second);
    }
    const double m_timestamp_min =
        (*measurements_to_process.begin())->timestamp();
    // Needs roll back when track has updated to a state at a timestamp later
    // than the measurement.
    if (track.track_state.last_timestamp > m_timestamp_min) {
      const int checkpoint_ind =
          track.checkpoints.GetIndexWithTimeAtMost(m_timestamp_min);
      // Discard the measurement if it's too old, aka, couldn't find a
      // checkpoint earlier than the current measurement.
      if (checkpoint_ind == -1) {
        return;
      }

      TrackState target_ckpt = track.checkpoints.value(checkpoint_ind);
      const double target_ckpt_ts = target_ckpt.last_timestamp;
      // Keep measurement and checkpoint before checkpoint timestamp.
      while (!track.measurement_history.empty()) {
        const auto& item = track.measurement_history.back();
        if (item.first > target_ckpt_ts) {
          measurements_to_process.insert(item.second);
          track.measurement_history.pop_back();
        } else {
          break;
        }
      }
      while (!track.checkpoints.empty()) {
        const auto& item = track.checkpoints.back();
        if (item.first > target_ckpt_ts) {
          track.checkpoints.pop_back();
          tracker_util::AlignKeyPointsToLatestEstimator(&track);
        } else {
          break;
        }
      }
      track.track_state = std::move(target_ckpt);
    }
    for (const auto* measurement : measurements_to_process) {
      UpdateTrackFromMeasurement(measurement, &track);
    }
  });
}

bool Tracker::IsRadarReflectionMeasurement(
    const MeasurementProto& measurement, const Track<TrackState>& track) const {
  const double radar_heading =
      Vec2dFromProto(measurement.radar_measurement().vel()).FastAngle();
  const double heading_diff =
      std::fabs(NormalizeAngle(radar_heading - track.track_state.heading));
  constexpr double kMaxAllowedHeadingDiff = kOneQuartersPi;
  constexpr double kMinTrackVelForFilterRadarReflections = 1.0;  // m/s
  // NOTE(zheng): When the track's vel is greater than 1.0 m/s and
  // the heading diff is greater than kOneQuartersPi, the associated radar
  // measurement maybe a false positive caused by reflection, so we do not
  // update track from this measurement.
  // TODO(zheng, fanjie): Filter out false positive caused by reflections.
  return heading_diff > kMaxAllowedHeadingDiff &&
         std::fabs(track.track_state.vel) >
             kMinTrackVelForFilterRadarReflections;
}

bool Tracker::IsRadarMeasurementVelocityValid(
    const MeasurementProto& measurement, const Track<TrackState>& track) const {
  // Note(zheng): If the vel angle with vehicle coord x axis is greater than
  // one quarters, it's maybe a shooter, we do not update with this velocy.
  bool is_vel_valid = true;

  // Note(zheng): We should use local coord velocity direction to filter
  // shooters, not use body vel direction, because the vel in local coord may
  // include self av car velocity error, so if we use the body vel, some
  // shooters may not be filtered.
  const double vel_angle_with_av_dir = NormalizeAngle(
      Vec2dFromProto(measurement.radar_measurement().vel()).FastAngle() -
      pose_.yaw);
  if ((fabs(vel_angle_with_av_dir) > kOneQuartersPi &&
       fabs(vel_angle_with_av_dir) < kThreeQuartersPi) ||
      fabs(track.track_state.last_update_radar_velocity_timestamp -
           measurement.timestamp()) < DBL_EPSILON) {
    is_vel_valid = false;
  }
  // Note(zheng): If the radar measurement is right side or left side of the
  // av, the radar measurement velocity is not accurate.
  constexpr double kRadarReliableAreaAngle = kOneQuartersPi;
  const double obj_body_pos_angle = NormalizeAngle(
      Vec2dFromProto(measurement.radar_measurement().radar_coord_pos())
          .FastAngle());

  if ((std::abs(obj_body_pos_angle) > kRadarReliableAreaAngle &&
       std::abs(obj_body_pos_angle) < (M_PI - kRadarReliableAreaAngle))) {
    is_vel_valid = false;
  }
  return is_vel_valid;
}

void Tracker::UpdateRadarOnlyTrackExtents(const MeasurementProto& measurement,
                                          Track<TrackState>* track) {
  const Vec2d m_pos = Vec2dFromProto(measurement.radar_measurement().pos());
  const double heading = NormalizeAngle(
      Vec2dFromProto(measurement.radar_measurement().vel()).FastAngle());
  constexpr double kRadarObjLength = 1.0;  // m
  constexpr double kRadarObjWidth = 1.0;   // m
  Box2d bbox(m_pos, heading, kRadarObjLength, kRadarObjWidth);
  Polygon2d polygon(bbox);
  track->track_state.bounding_box = bbox;
  track->track_state.refined_bounding_box = bbox;
  track->track_state.contour = polygon;
  track->track_state.type = MT_UNKNOWN;
  const StateData s = track->track_state.estimator_3d.GetStateData();
  track->track_state.vel = s.GetVel();
  track->track_state.heading = s.GetYaw();

  track->track_state.track_shape_source_type = TrackShapeSourceType::TSST_RADAR;
  track->track_state.observation_state = ObservationState::OS_UNKNOWN;
}

void Tracker::UpdateTrackFromRadarMeasurement(
    const MeasurementProto* measurement, Track<TrackState>* track) {
  QCHECK_NOTNULL(track);
  QCHECK(measurement->has_radar_measurement());
  // Check if the measurement is a radar reflection.
  if (IsRadarReflectionMeasurement(*measurement, *track)) {
    return;
  }
  // Update measurement history.
  track->measurement_history.PushBackAndClearStale(
      measurement->timestamp(), measurement,
      kMaxMeasurementHistoryBufferLength);

  // Judge if the radar measurement's velocity is valid.
  const bool should_update_vel =
      IsRadarMeasurementVelocityValid(*measurement, *track);

  track->track_state.last_timestamp = measurement->timestamp();
  track->track_state.state_timestamp = measurement->timestamp();
  track->debug_info.updated_with_radar_velocity = should_update_vel;

  const auto obj_pos = measurement->radar_measurement().pos();
  const Vec2d obj_speed =
      Vec2dFromProto(measurement->radar_measurement().vel());

  // We add an extra velocity noise to radar velocity covariance because the
  // covariance from radar is not so accurate, we can adjust it by experience.
  constexpr double kExtraVelocityNoise = 1.0;
  Mat2d m_vel_cov;
  Mat2dFromProto(measurement->radar_measurement().vel_cov(), &m_vel_cov);
  m_vel_cov += kExtraVelocityNoise * Eigen::Matrix2d::Identity();

  auto meas_noise = track->track_state.estimator_3d.GetMeasurementNoise();
  // Construct measurements.
  const PositionMeasurement pos(obj_pos.x(), obj_pos.y());
  const SpeedMeasurement speed(obj_speed(0), obj_speed(1));

  // For non radar only track, it's contour/bbox come from laser measuremnent,
  // we should predict contour/bbox when we associate a radar measurement.
  if (track->track_state.measurement_source_type !=
      TrackMeasurementSourceType::TMST_RO) {
    PredictTrackShapeInfo(measurement->timestamp(), track);
  }
  // Update with measurement.
  track->track_state.estimator_3d.Predict(measurement->timestamp());

  // Only update with position when the track is radar only track.track_state.
  if (track->track_state.measurement_source_type ==
      TrackMeasurementSourceType::TMST_RO) {
    if (track->track_state.estimator_3d.IsCarModel()) {
      const double state_heading =
          track->track_state.estimator_3d.GetStateData().GetYaw();
      const Vec2d state_heading_dir = Vec2d::FastUnitFromAngle(state_heading);
      const PosVelocityMeasurement pos_vel(pos.x(), pos.y(),
                                           obj_speed.dot(state_heading_dir));
      track->track_state.estimator_3d.Update(pos_vel);
    } else {
      const PosSpeedMeasurement pos_speed(pos.x(), pos.y(), speed.vx(),
                                          speed.vy());
      track->track_state.estimator_3d.Update(pos_speed);
    }
    // Update track info.
    UpdateRadarOnlyTrackExtents(*measurement, track);
  } else {
    if (should_update_vel) {
      if (track->track_state.estimator_3d.IsCarModel() ||
          (track->track_state.estimator_3d.IsPointModel() &&
           track->track_state.type != MT_PEDESTRIAN)) {
        meas_noise.set_state_vel_x_measurement_noise_std(sqrt(m_vel_cov(0, 0)));
        meas_noise.set_state_vel_y_measurement_noise_std(sqrt(m_vel_cov(1, 1)));
        track->track_state.estimator_3d.Update(speed, meas_noise);
      }
    }
  }

  if (track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
          *track, pose_, measurement->timestamp())) {
    track->track_state.life_state = TrackLifeState::kConfirmed;
  }

  track->track_state.measurement_source_type =
      TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(*track);
}

void Tracker::SaveCheckPoints(Track<TrackState>* track) {
  track->track_state.coordinate_converter = coordinate_converter_;

  track->track_state.key_point =
      track->track_state.estimator_3d.GetStateData().GetStatePos();
  TrackState checkpoint = track->track_state;

  track->checkpoints.PushBackAndClearStale(checkpoint.last_timestamp,
                                           std::move(checkpoint),
                                           kMaxCheckPointHistoryBufferLength);
}

void Tracker::UpdateTrackFromMeasurement(const MeasurementProto* measurement,
                                         Track<TrackState>* track) {
  if (measurement->has_laser_measurement()) {
    UpdateTrackFromLaserMeasurement(measurement, track);
  } else if (measurement->has_camera3d_measurement()) {
    UpdateTrackFromCameraMeasurement(measurement, track);
  } else if (measurement->has_radar_measurement()) {
    UpdateTrackFromRadarMeasurement(measurement, track);
  }

  UpdateAnchorPoint(*measurement, track);

  // Save checkpoints.
  SaveCheckPoints(track);
}

// TODO(tao, yu): Convert camera measurements to amendment measurement->
// TODO(tao, yu): Implement different measurement update in a more holistic
// way. Update track from camera measurements without state update.
void Tracker::UpdateTrackFromCameraMeasurement(
    const MeasurementProto* measurement, Track<TrackState>* track) {
  QCHECK(measurement->has_camera3d_measurement());
  QCHECK_NOTNULL(track);
  const auto& m = measurement->camera3d_measurement();
  track->measurement_history.PushBackAndClearStale(
      measurement->timestamp(), measurement,
      kMaxMeasurementHistoryBufferLength);
  const auto curr_source_type =
      TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(*track);

  // Skip update if the target is in the LVR area.
  if (TMSTM::HasLidarMeasurements(curr_source_type)) {
    track->debug_info.updated_with_camera = false;
    track->track_state.measurement_source_type = curr_source_type;
    PredictTrackShapeInfo(measurement->timestamp(), track);

    track->track_state.estimator_3d.Predict(measurement->timestamp());
    UpdateTrackState(*measurement, track);
    return;
  }

  track->debug_info.updated_with_camera = true;

  std::optional<double> heading;
  if (track->track_state.estimator_3d.IsCarModel()) {
    heading = m.heading();
    const auto s = track->track_state.estimator_3d.GetStateData();
    const double kMaxDiffAllowHeadingRotation = M_PI_2;
    if (track->track_state.heading_voter.ShouldUseMeasurementHeading(
            s.GetYaw(), *heading, kMaxDiffAllowHeadingRotation)) {
      CarStateData car_state_data;
      const auto pos_cov = s.GetStatePosCov();
      car_state_data.state() = s.car().state();
      car_state_data.state().yaw() = *heading;
      car_state_data.state().vel() = -1.0 * s.GetVel();
      car_state_data.state().acc() = -1.0 * s.GetAcc();
      // Keep each variance in main diagonal.
      car_state_data.state_cov() = s.car().state_cov().diagonal().asDiagonal();
      // Keep 2x2 pos covariance.
      car_state_data.SetPosCov(pos_cov);

      tracker_util::ResetEstimatorStatePos(StateData(car_state_data), track);
      // Disable heading measurement update
      heading = std::nullopt;
    }
    if (heading && (std::abs(NormalizeAngle(*heading - s.GetYaw())) >
                    kMaxDiffAllowHeadingRotation)) {
      heading = std::nullopt;
    }
  }
  track->track_state.estimator_3d.Predict(measurement->timestamp());

  const auto prev_source_type = track->track_state.measurement_source_type;
  track->track_state.measurement_source_type = curr_source_type;

  if (!TMSTM::IsSwitchToNoLidarTrack(prev_source_type, curr_source_type)) {
    Mat3d cov_cam;
    qcraft::Mat3dFromProto(m.mono3d_pos_cov(), &cov_cam);
    const double state_x_meas_std = sqrt(cov_cam(0, 0));
    const double state_y_meas_std = sqrt(cov_cam(1, 1));
    QCHECK(state_x_meas_std > 0.0);
    QCHECK(state_y_meas_std > 0.0);
    const double state_x_y_meas_corr_coe =
        cov_cam(0, 1) / (state_x_meas_std * state_y_meas_std);
    // Normal update
    auto meas_noise = track->track_state.estimator_3d.GetMeasurementNoise();
    meas_noise.set_state_x_measurement_noise_std(state_x_meas_std);
    meas_noise.set_state_y_measurement_noise_std(state_y_meas_std);
    meas_noise.set_state_x_y_measurement_correlation_coefficient(
        state_x_y_meas_corr_coe);
    if (track->track_state.estimator_3d.IsCarModel()) {
      if (heading) {
        const PosHeadingMeasurement pos_yaw(m.pos().x(), m.pos().y(),
                                            m.heading());
        double camera_heading_noise_std = 4.0;  // degree
        if (m.type() == MeasurementType::MT_CYCLIST) {
          camera_heading_noise_std = 8.0;  // degree
        }
        meas_noise.set_state_heading_measurement_noise_std(
            camera_heading_noise_std);
        track->track_state.estimator_3d.Update(pos_yaw, meas_noise);
      } else {
        const PositionMeasurement pos(m.pos().x(), m.pos().y());
        track->track_state.estimator_3d.Update(pos, meas_noise);
      }
    } else {
      const PositionMeasurement pos(m.pos().x(), m.pos().y());
      track->track_state.estimator_3d.Update(pos, meas_noise);
    }
  } else {
    StateData s = track->track_state.estimator_3d.GetStateData();
    s.SetPos(m.pos().x(), m.pos().y());

    tracker_util::ResetEstimatorStatePos(s, track);
  }

  track->track_state.ref_point.type = TrackState::RefPoint::kBBCenter;
  UpdateEstimatorExtent(*measurement, track);
  const auto bbox = ComputeBBoxFromEstimatorExtent(*track);
  track->track_state.bounding_box = bbox;
  track->track_state.refined_bounding_box = bbox;
  Polygon2d polygon(bbox);
  track->track_state.contour = polygon;

  track->track_state.track_shape_source_type =
      TrackShapeSourceType::TSST_CAMERA;
  track->track_state.observation_state = ObservationState::OS_UNKNOWN;
  UpdateTrackState(*measurement, track);
}

void Tracker::UpdateTrackFromLaserMeasurement(
    const MeasurementProto* measurement, Track<TrackState>* track) {
  QCHECK(measurement->has_laser_measurement());
  const auto& laser_measurement = measurement->laser_measurement();
  QCHECK_NOTNULL(track);
  const bool is_in_blind_area = IsInBlindArea(*track);

  // TODO(jingwei): Use the icp vel covariance.
  // Only use icp vel if it's not in blind area, icp exists and is converged.
  bool is_icp_right_matched = false;
  const bool icp_vel_converged = IsIcpVelConverged(*track, laser_measurement);
  if (!is_in_blind_area && icp_vel_converged &&
      laser_measurement.has_icp_measurement()) {
    constexpr double kEpsion = 1e-3;
    const double prev_icp_matched_cluster_timestamp =
        laser_measurement.icp_measurement().matched_prev_cluster_timestamp();
    const int prev_icp_matched_cluster_id =
        laser_measurement.icp_measurement().matched_prev_cluster_id();
    const auto& m_history = track->measurement_history;
    const int num_m = m_history.size();
    constexpr double kDurationAsValidMeasurement = 0.5;  // s
    for (int p = m_history.GetIndexWithTimeAtLeast(
             laser_measurement.cluster_measurement().timestamp() -
             kDurationAsValidMeasurement);
         p < num_m; ++p) {
      const auto* m = m_history.value(p);
      if (m->has_laser_measurement()) {
        QCHECK(m->laser_measurement().has_cluster_measurement());
        double m_cluster_timestamp =
            m->laser_measurement().cluster_measurement().timestamp();
        int m_cluster_id =
            m->laser_measurement().cluster_measurement().cluster_id();
        if (std::abs(m_cluster_timestamp - prev_icp_matched_cluster_timestamp) <
                kEpsion &&
            m_cluster_id == prev_icp_matched_cluster_id) {
          is_icp_right_matched = true;
          break;
        }
      }
    }
  }

  // Update contour, obstacle centers and bounding box.
  UpdateTrackExtents(laser_measurement, measurement->timestamp(), track);
  UpdateEstimatorExtent(*measurement, track);

  const auto& prev_ref_point = track->track_state.ref_point;
  const auto curr_ref_point_origin =
      ChooseAndComputeRefPoint(laser_measurement, *track);
  auto curr_ref_point = curr_ref_point_origin;
  // Flip ref point type when the type is corner and the heading angle
  // between track and measurement great than thershhold.
  if (laser_measurement.has_detection_bounding_box() &&
      IsCornerOrFaceCenterRefType(curr_ref_point.type)) {
    curr_ref_point.type = TransformRefTypeWhenYawFlipped(
        curr_ref_point.type,
        track->track_state.estimator_3d.GetStateData().GetYaw(),
        laser_measurement.detection_bounding_box().heading());
  }

  // The difference between current ref_point_vel and last frame
  // ref_point_vel.
  double ref_point_vel_changment = 0.0;
  // If the ref point type is changed, update the position of both car and
  // point models.
  const bool has_lidar_measurements = TMSTM::HasLidarMeasurements(*track);
  const bool ref_point_changed =
      (curr_ref_point.type != prev_ref_point.type &&
       prev_ref_point.type != TrackState::RefPoint::kNone &&
       has_lidar_measurements);
  // BANDAID(zheng): To solve the shooters caused by ref point changment, we
  // don't update track with ref point when one of the ref point is
  // CornerOrFaceCenter but another is not.
  bool should_only_predict =
      prev_ref_point.type != TrackState::RefPoint::kNone &&
      has_lidar_measurements &&
      (IsCornerOrFaceCenterRefType(prev_ref_point.type) !=
       IsCornerOrFaceCenterRefType(curr_ref_point.type));
  if (ref_point_changed && !should_only_predict) {
    track->track_state.ref_point_vel = std::nullopt;
    const Vec2d curr_state_pos = track->track_state.estimator_3d.GetStatePos();
    // There are 11 kinds of ref-point from 3 kinds of detection as below.
    // Bbox(optional) -> kBBCenter, Corners(4), Face center(4).
    // Contour(required) -> kContourCentroid.
    // Cluster(required) centers -> kNearestObstacleCenter.
    auto ref_point_offset =
        ComputeRefPointOffset(curr_ref_point, *track, measurement->timestamp());
    // Ref_point_offset has value in 3 cases:
    // 1st. (all) Same type ref_point: [bbox->bbox; contour->contour;
    // cluster->cluster] 2nd. (all) Superset(with fen) to subset(no fen):
    // [bbox->contour/cluster] 3rd. (part) Subset(no FEN) to superset(with
    // FEN): [contour/cluster->bbox] Ref_point_offset is nullopt in 1 case:
    // 1st. (part) Subset(no FEN) to superset(with FEN):
    // contour/cluster->bbox;
    if (!ref_point_offset) {
      // Use current laser measurement to estimate ref point offset.
      const auto m_cur_ref_point = ComputeRefPointPosGivenType(
          laser_measurement, curr_ref_point_origin.type);
      const auto m_prev_ref_point =
          ComputeRefPointPosGivenType(laser_measurement, prev_ref_point.type);
      if (m_cur_ref_point && m_prev_ref_point) {
        ref_point_offset = m_cur_ref_point.value() - m_prev_ref_point.value();
      } else {
        // NOTE(zheng): If the latest many frames is radar or camera
        // measurements, it may fail to compute ref_point_offset, in this case
        // we can predict a fake prev ref point.
        const auto s =
            tracker_util::SafePredictPos(*track, measurement->timestamp());
        const Vec2d predicted_pos =
            s.has_value() ? s->GetStatePos()
                          : track->track_state.estimator_3d.GetStatePos();
        const Vec2d curr_state_pos =
            track->track_state.estimator_3d.GetStatePos();
        const Vec2d offset = predicted_pos - curr_state_pos;
        const Vec2d prev_ref_point_pos = curr_ref_point.pos - offset;
        ref_point_offset = prev_ref_point_pos - curr_state_pos;
      }
    }
    // NOTE(jiawei): Any state reset will contaminate the state distribution
    // density which may lead to filter divergence.
    StateData s = track->track_state.estimator_3d.GetStateData();
    const auto pos = curr_state_pos + ref_point_offset.value();
    track->debug_info.state_pos_reset_offset = ref_point_offset;
    s.SetPos(pos.x(), pos.y());
    tracker_util::ResetEstimatorStatePos(s, track);
  }
  if (!ref_point_changed &&
      prev_ref_point.type != TrackState::RefPoint::kNone &&
      has_lidar_measurements) {
    const auto ref_point_movement = curr_ref_point.pos - prev_ref_point.pos;
    const double time_diff =
        measurement->timestamp() - track->track_state.last_laser_timestamp;
    const auto ref_point_vel = ref_point_movement / time_diff;
    if (track->track_state.ref_point_vel) {
      ref_point_vel_changment =
          (ref_point_vel - *(track->track_state.ref_point_vel)).norm();
    }
    track->track_state.ref_point_vel = ref_point_vel;
  }

  // If the track only contains radar, vision measurements, we should reset the
  // state pos.
  if (!has_lidar_measurements) {
    const auto pred_s =
        tracker_util::SafePredictPos(*track, measurement->timestamp());
    const Vec2d predicted_pos =
        pred_s.has_value() ? pred_s->GetStatePos()
                           : track->track_state.estimator_3d.GetStatePos();
    const Vec2d curr_state_pos = track->track_state.estimator_3d.GetStatePos();
    const Vec2d offset = predicted_pos - curr_state_pos;
    // NOTE(zheng): When we update radar only track with a laser measurement
    // we should reset the state to last frame laser measurement
    // ref point, which comes from a predict result.
    // NOTE(jiawei): Any state reset will contaminate the state distribution
    // density which may lead to filter divergence.

    StateData s = track->track_state.estimator_3d.GetStateData();
    const auto pos = curr_ref_point.pos - offset;
    s.SetPos(pos.x(), pos.y());
    tracker_util::ResetEstimatorStatePos(s, track);
  }

  // BANDAID(zheng): To supress big vehicle ref point jump issue, we use
  // the ref_point_vel changment to compute the ref point uncertainty.
  // For objects, the max acc may be 20 m/s^2, if the ref point vel change
  // more than 2 m/s, the ref point may be jump caused by unstable
  // detection, we can give a bigger measurement noise, and reset state pos
  // to current pos.
  double ref_point_noise = 0.5;
  const double kMaxReasonableVelChangment = 2.0;  // m/s
  if (ref_point_vel_changment > kMaxReasonableVelChangment) {
    const double ref_point_uncertainty =
        1.0 * ref_point_vel_changment * ref_point_vel_changment;
    ref_point_noise += ref_point_uncertainty;
  }

  track->track_state.ref_point = curr_ref_point;
  const Vec2d ref_point = curr_ref_point.pos;

  track->measurement_history.PushBackAndClearStale(
      measurement->timestamp(), measurement,
      kMaxMeasurementHistoryBufferLength);
  if (track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
          *track, pose_, measurement->timestamp())) {
    track->track_state.life_state = TrackLifeState::kConfirmed;
  }

  // BANDAID(zheng): supress blind area shooters by enlarge blind area
  // measurement noise.
  if (is_in_blind_area) {
    ref_point_noise *= 2.0;
  }

  const auto prev_source_type = track->track_state.measurement_source_type;
  const auto curr_source_type =
      TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(*track);
  const bool is_switch_to_lidar_track =
      TMSTM::IsSwitchToLidarTrack(prev_source_type, curr_source_type);
  should_only_predict = should_only_predict || is_switch_to_lidar_track;

  std::optional<double> heading;
  if (track->track_state.estimator_3d.IsCarModel() &&
      track->track_state.bounding_box) {
    heading = track->track_state.bounding_box->heading();
    const auto s = track->track_state.estimator_3d.GetStateData();
    const double kMaxDiffAllowHeadingRotation = M_PI_2;
    if (is_switch_to_lidar_track) {
      track->track_state.heading_voter.Reset();
    }
    if (track->track_state.heading_voter.ShouldUseMeasurementHeading(
            s.GetYaw(), *heading, kMaxDiffAllowHeadingRotation)) {
      CarStateData car_state_data;
      const auto pos_cov = s.GetStatePosCov();
      car_state_data.state() = s.car().state();
      car_state_data.state().yaw() = *heading;
      car_state_data.state().vel() = -1.0 * s.GetVel();
      car_state_data.state().acc() = -1.0 * s.GetAcc();
      // Keep each variance in main diagonal.
      car_state_data.state_cov() = s.car().state_cov().diagonal().asDiagonal();
      // Keep 2x2 pos covariance.
      car_state_data.SetPosCov(pos_cov);

      tracker_util::ResetEstimatorStatePos(StateData(car_state_data), track);

      // Disable heading measurement update
      heading = std::nullopt;
    }
    if (heading && (std::abs(NormalizeAngle(*heading - s.GetYaw())) >
                    kMaxDiffAllowHeadingRotation)) {
      heading = std::nullopt;
    }
  }
  if (laser_measurement.has_icp_measurement() &&
      laser_measurement.icp_measurement().has_vel()) {
    track->track_state.icp_vel =
        Vec2dFromProto(laser_measurement.icp_measurement().vel());
  }

  if (laser_measurement.has_fen_velocity()) {
    track->track_state.fen_vel =
        Vec2dFromProto(laser_measurement.fen_velocity());
  }

  // NOTE(zheng): For vel fusion, we first select converged icp vel, if
  // there is no convereged icp vel, we use fen vel. The fen vel is not
  // so accurate when the fen vel is greater the 15 m/s or in the blind
  // area.
  // TODO(zheng): We should select fen or icp vel according to PRT result
  // in different scene.
  [[maybe_unused]] constexpr double kMaxFenVelToUse = 15;  // m / s
  std::optional<Vec2d> velocity_measurement;
  if (is_icp_right_matched) {
    track->debug_info.updated_with_icp_velocity = true;
    velocity_measurement =
        Vec2dFromProto(laser_measurement.icp_measurement().vel());
  }
  // NOTE(zheng): The fen vel is no accurate in highway scene, so we close
  // it until the fen vel has a better result.
  // else if (!is_in_blind_area && laser_measurement.has_fen_velocity()) {
  //   const Vec2d fen_vel = Vec2dFromProto(laser_measurement.fen_velocity());
  //   if (fen_vel.squaredNorm() < Sqr(kMaxFenVelToUse)) {
  //     // TODO(zheng): Use fen vel uncertainty as measurement noise.
  //     track->debug_info.updated_with_fen_velocity = true;
  //     velocity_measurement = fen_vel;
  //   }
  // }

  // Motion filter predict
  const PositionMeasurement pos(ref_point.x(), ref_point.y());
  track->track_state.estimator_3d.Predict(measurement->timestamp());
  const double pos_m_dist =
      track->track_state.estimator_3d.ComputeMahalanobisDistance(pos);
  // NOTE(youjiawei): we only update pos if the measurement pos is inside
  // validation gate in terms of mahalanobis distance. The reason is that it
  // can prevent shooters if motion filter update with abnormal measurement
  // pos.
  bool should_update_pos =
      IsInValidationGateByMahalanobisDistance(pos_m_dist, 2);
  // Close the pos gating until we have accurate gating threshold.
  should_update_pos = true;
  if (should_only_predict) {  // Only predict
    StateData s = track->track_state.estimator_3d.GetStateData();
    s.SetPos(ref_point.x(), ref_point.y());
    tracker_util::ResetEstimatorStatePos(s, track);
  } else {  // Motion filter update
    if (track->track_state.estimator_3d.IsCarModel()) {
      const double state_heading =
          track->track_state.estimator_3d.GetStateData().GetYaw();
      const auto state_heading_dir = Vec2d::FastUnitFromAngle(state_heading);
      if (should_update_pos) {
        auto meas_noise = track->track_state.estimator_3d.GetMeasurementNoise();
        constexpr double kMaxPosMeasurementNoise = 50.0;
        ref_point_noise = std::min(ref_point_noise, kMaxPosMeasurementNoise);
        meas_noise.set_state_x_measurement_noise_std(sqrt(ref_point_noise));
        meas_noise.set_state_y_measurement_noise_std(sqrt(ref_point_noise));
        if (heading && velocity_measurement) {
          auto meas_noise =
              track->track_state.estimator_3d.GetMeasurementNoise();
          const PosHeadingVelocityMeasurement pos_yaw_vel(
              ref_point.x(), ref_point.y(), *heading,
              velocity_measurement->dot(state_heading_dir));
          track->track_state.estimator_3d.Update(pos_yaw_vel, meas_noise);

        } else if (heading && (!velocity_measurement)) {
          const PosHeadingMeasurement pos_yaw(ref_point.x(), ref_point.y(),
                                              *heading);
          track->track_state.estimator_3d.Update(pos_yaw, meas_noise);
        } else if ((!heading) && velocity_measurement) {
          const PosVelocityMeasurement pos_vel(
              ref_point.x(), ref_point.y(),
              velocity_measurement->dot(state_heading_dir));
          track->track_state.estimator_3d.Update(pos_vel);
        } else {
          track->track_state.estimator_3d.Update(pos);
        }
      } else {
        if (heading && velocity_measurement) {
          const HeadingVelocityMeasurement yaw_vel(
              *heading, velocity_measurement->dot(state_heading_dir));
          track->track_state.estimator_3d.Update(yaw_vel);

        } else if (heading && (!velocity_measurement)) {
          const HeadingMeasurement yaw(*heading);
          track->track_state.estimator_3d.Update(yaw);
        } else if ((!heading) && velocity_measurement) {
          const VelocityMeasurement vel(
              velocity_measurement->dot(state_heading_dir));
          track->track_state.estimator_3d.Update(vel);
        }
      }
    } else {  // Point model
      // Update pos with default noise.
      if (should_update_pos) {
        track->track_state.estimator_3d.Update(pos);
      }
      // Update speed with specific noise
      if (velocity_measurement) {
        const SpeedMeasurement speed(velocity_measurement->x(),
                                     velocity_measurement->y());
        auto meas_noise = track->track_state.estimator_3d.GetMeasurementNoise();
        meas_noise.set_state_vel_x_measurement_noise_std(
            sqrt(kVelMeasurementNoise));
        meas_noise.set_state_vel_y_measurement_noise_std(
            sqrt(kVelMeasurementNoise));
        track->track_state.estimator_3d.Update(speed, meas_noise);
      }
    }
  }

  // If car is brake hard and stop, we reset acc to be zero.
  // TODO(zheng, jiawei): Clarify and find hard brake scenario.
  // TODO(jiawei): Check if motion filter 2.0 can handle the above case.
  if (track->track_state.estimator_3d.IsCarModel()) {
    StateData s = track->track_state.estimator_3d.GetStateData();
    if (s.GetVel() < 0.1 && s.GetAcc() < 0.0 &&
        tracker_util::IsSuddenBreakHappening(*track)) {
      VLOG(2) << track->track_state.id << "\n";
      VLOG(2) << "car brake hard!\n\n\n\n";
      s.car().state().acc() = 0.0;
      tracker_util::ResetEstimatorStatePos(s, track);
    }
  }

  // If the motion filter is kCarModel_CTRA, we get yaw rate
  constexpr double kYawRateSmoothScale = 0.7;
  constexpr double kMaxYawRate = d2r(60.0);  // 60 degree/s

  if (track->track_state.estimator_3d.IsCarModel()) {
    const StateData s = track->track_state.estimator_3d.GetStateData();
    track->track_state.yaw_rate = s.GetYawD();
  } else {
    const double prev_vel = track->track_state.vel;
    const StateData s = track->track_state.estimator_3d.GetStateData();
    const double curr_vel = s.GetVel();
    constexpr double kSpeedNoise = 0.25;
    if (curr_vel > kSpeedNoise && prev_vel > kSpeedNoise) {
      const double curr_heading = s.GetYaw();
      const double delta_t =
          measurement->timestamp() - track->track_state.last_timestamp;
      const double curr_yaw_rate =
          (curr_heading - track->track_state.heading) / (delta_t + DBL_EPSILON);
      track->track_state.yaw_rate =
          kYawRateSmoothScale * track->track_state.yaw_rate +
          (1.0 - kYawRateSmoothScale) * curr_yaw_rate;
    }
  }

  const StateData s = track->track_state.estimator_3d.GetStateData();
  track->track_state.vel = s.GetVel();
  track->track_state.heading = s.GetYaw();

  // If the yaw rate is greater than kMaxYawRate or the track type is
  // pedestrian, we set yaw rate to zero.
  if (track->track_state.yaw_rate > kMaxYawRate ||
      track->track_state.type == MT_PEDESTRIAN) {
    track->track_state.yaw_rate = 0.0;
  }

  // Unify ref point type to curr track heading.
  if (laser_measurement.has_detection_bounding_box() &&
      IsCornerOrFaceCenterRefType(track->track_state.ref_point.type)) {
    track->track_state.ref_point.type = TransformRefTypeWhenYawFlipped(
        curr_ref_point_origin.type, track->track_state.heading,
        laser_measurement.detection_bounding_box().heading());
  }

  if (!has_lidar_measurements) {
    // NOTE(jiawei): Any state reset will contaminate the state distribution
    // density which may lead to filter divergence.
    StateData s = track->track_state.estimator_3d.GetStateData();
    const auto pos = curr_ref_point.pos;
    s.SetPos(pos.x(), pos.y());
    tracker_util::ResetEstimatorStatePos(s, track);
  }

  track->track_state.measurement_source_type =
      TMSTM::ComputeTrackMeasurementSourceTypeByMeasurementHistory(*track);

  track->track_state.last_timestamp = measurement->timestamp();
  track->track_state.state_timestamp = measurement->timestamp();
  track->track_state.last_laser_timestamp = measurement->timestamp();
  // Sync min_z, max_z and ground_z value.
  if (measurement->laser_measurement().has_cluster_measurement()) {
    const auto& cluster =
        measurement->laser_measurement().cluster_measurement();
    track->track_state.min_z = cluster.min_z();
    track->track_state.max_z = cluster.max_z();
    track->track_state.ground_z = cluster.ground_z();
  }
}

std::vector<int> Tracker::AssociateCameraMeasurements(
    const double timestamp,
    const std::vector<const MeasurementProto*>& camera_measurements) {
  SCOPED_QTRACE("Tracker::AssociateCameraMeasurements");

  if (camera_measurements.empty() || tracks_.empty()) return {};
  const int num_measurements = camera_measurements.size();
  Eigen::MatrixXd weight_matrix(num_measurements, tracks_.size());
  weight_matrix.setZero();
  ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
    auto& track = *tracks_[i];
    for (int j = 0; j < num_measurements; ++j) {
      const auto* measurement = camera_measurements[j];
      QCHECK(measurement->has_camera_measurement());
      const auto& camera_measurement = measurement->camera_measurement();
      const Vec2d m_center = Vec2dFromProto(camera_measurement.center());
      Mat2d m_center_cov;
      Mat2dFromProto(camera_measurement.center_cov(), &m_center_cov);
      // Use two point-to-dist distance to measure dist-to-dist distance.
      // Use contour centroid as the track center to compute association cost.
      const auto s =
          tracker_util::SafePredictPos(track, measurement->timestamp());

      const Vec2d predicted_offset =
          s.has_value()
              ? s->GetStatePos() - track.track_state.estimator_3d.GetStatePos()
              : Vec2d(0., 0.);
      const Vec2d t_obstacle_center_centroid =
          predicted_offset +
          tracker_util::ComputeCentroid(track.track_state.contour.points());
      Mat2d pos_cov;

      pos_cov = s.has_value() ? s->GetStatePosCov()
                              : track.track_state.estimator_3d.GetStatePosCov();

      constexpr int kMahalanobisDistDof = 2;
      const double track2m_distr_distance_sqr =
          tracker_util::ComputeMahalanobisSquare<kMahalanobisDistDof>(
              t_obstacle_center_centroid, m_center, m_center_cov);
      const double p_value_track2m = ComputePValueFromChiSquareDistr(
          track2m_distr_distance_sqr, kMahalanobisDistDof);
      const double m2track_distr_distance_sqr =
          tracker_util::ComputeMahalanobisSquare<kMahalanobisDistDof>(
              m_center, t_obstacle_center_centroid, pos_cov);
      const double p_value_m2track = ComputePValueFromChiSquareDistr(
          m2track_distr_distance_sqr, kMahalanobisDistDof);
      constexpr double kTrack2CameraMGatingConfidenceLevel = 0.997;
      if (j < 0 || j >= weight_matrix.rows() || i < 0 ||
          i >= weight_matrix.cols()) {
        continue;
      }
      constexpr double kMmToTrackDistrMDistPValueWeight = 0.5;
      weight_matrix(j, i) =
          (1. - p_value_track2m > kTrack2CameraMGatingConfidenceLevel)
              ? -std::numeric_limits<double>::max()
              : p_value_track2m +
                    kMmToTrackDistrMDistPValueWeight * p_value_m2track;
    }
  });
  return tracker_util::ComputeMatches(weight_matrix);
}

void Tracker::AssociateMeasurementsAndUpdateTracksWithRollBack(
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    const CoordinateConverter& coordinate_converter, double timestamp,
    MeasurementsProto measurements_proto) {
  SCOPED_QTRACE("Tracker::AssociateMeasurementsAndUpdateTracksWithRollBack");
  const double min_measurement_timestamp = measurements_proto.min_timestamp();
  const auto group_type = measurements_proto.group_type();
  if (measurements_proto.measurements_size() == 0) {
    return;
  }
  // The association and update pipeline should be rolled back when the
  // incoming measurement's min timestamp is older than the latest
  // measurement's timestamp. It mainly happen in lidar or camera
  // measurements, because it cost a lot time when we generate the lidar or
  // camera measurement. The radar measurement's delay is the smallest, so we
  // don't implement the roll back mechanism for radar measurement group, if
  // the radar measurement group is outdated, we shound deprecate the radar
  // measurements.

  bool should_roll_back = false;
  if (!measurement_history_groups_.empty()) {
    const double latest_measurement_group_timestamp =
        measurement_history_groups_.back().second.min_timestamp();
    if (min_measurement_timestamp < latest_measurement_group_timestamp &&
        latest_measurement_group_timestamp - min_measurement_timestamp > 0.5) {
      // Too old measurement.
      return;
    }
    // Deprecate outdated radar measurements group.
    if (group_type == MeasurementsProto::RADAR &&
        min_measurement_timestamp < latest_measurement_group_timestamp) {
      return;
    }
    // Only if the measurements is lidar or camera measurement and it's
    // min_measurement_timestamp is less than the
    // latest_measurement_group_timestamp we implement the roll back
    // mechanism.
    // TODO(zheng): Open rollback mechanism for camera measurements
    if ((group_type == MeasurementsProto::LIDAR ||
         group_type == MeasurementsProto::CAMERA) &&
        min_measurement_timestamp < latest_measurement_group_timestamp) {
      should_roll_back = true;
    }
  }

  const double buffer_time =
      0.5 + kMaxMeasurementHistoryBufferLength +
      std::max(std::max(kMaxOnroadTrackLifeWithoutUpdate,
                        kMaxOffroadTrackLifeWithoutUpdate),
               kMaxCertainStaticTrackLifeWithoutUpdate);
  // Put measurements in the queue.
  auto cur_pose = pose_;
  auto cur_coordinate_converter = coordinate_converter;
  pose_history_.Insert(measurements_proto.min_timestamp(), std::move(cur_pose));
  pose_history_.ClearOlderThan(buffer_time);

  coordinate_converter_history_.Insert(measurements_proto.min_timestamp(),
                                       std::move(cur_coordinate_converter));
  coordinate_converter_history_.ClearOlderThan(buffer_time);
  coordinate_converter_ = &coordinate_converter_history_.back_value();

  measurement_history_groups_.Insert(measurements_proto.min_timestamp(),
                                     std::move(measurements_proto));
  measurement_history_groups_.ClearOlderThan(buffer_time);
  // Get should roll back measurement frames.
  if (should_roll_back) {
    int start_roll_back_group_index = measurement_history_groups_.size() - 1;
    for (int i = 0; i < measurement_history_groups_.size(); ++i) {
      if (min_measurement_timestamp <=
          measurement_history_groups_.value(i).min_timestamp()) {
        start_roll_back_group_index = i;
        break;
      }
    }

    // Get from whitch time to roll back.
    const double roll_back_checkpoint_time =
        measurement_history_groups_.value(start_roll_back_group_index)
            .min_timestamp();

    // Don't update std::vector<bool> in parallel as it is implemented as a
    // bit-vector. Use std::vector<int> instead.
    std::vector<int> valid_tracks(tracks_.size(), true);

    // Roll back tracks.
    ParallelFor(0, tracks_.size(), thread_pool_, [&](int i) {
      auto& track = *tracks_[i];
      // Needs roll back when track has updated to a state at a timestamp
      // later than the measurement.
      if (track.track_state.last_timestamp >= roll_back_checkpoint_time) {
        int checkpoint_ind =
            track.checkpoints.GetIndexWithTimeAtMost(roll_back_checkpoint_time);
        if (track.track_state.last_timestamp == roll_back_checkpoint_time) {
          checkpoint_ind = checkpoint_ind - 1;
        }
        // Discard the measurement if it's too old, aka, couldn't find a
        // checkpoint earlier than the current measurement.
        if (checkpoint_ind < 0) {
          valid_tracks[i] = false;
          return;
        }
        TrackState target_ckpt = track.checkpoints.value(checkpoint_ind);
        const double target_ckpt_ts = target_ckpt.last_timestamp;
        // Keep measurement and checkpoint before checkpoint timestamp.
        while (!track.measurement_history.empty()) {
          const auto& item = track.measurement_history.back();
          if (item.first > target_ckpt_ts) {
            track.measurement_history.pop_back();
          } else {
            break;
          }
        }
        while (!track.checkpoints.empty()) {
          const auto& item = track.checkpoints.back();
          if (item.first > target_ckpt_ts) {
            track.checkpoints.pop_back();
            tracker_util::AlignKeyPointsToLatestEstimator(&track);
          } else {
            break;
          }
        }
        track.track_state = std::move(target_ckpt);
      }
    });
    // Clear the tracks when the roll back checkpoint time is greater than the
    // first tracked time of track.
    int valid_track_count = 0;
    for (int i = 0; i < valid_tracks.size(); ++i) {
      if (valid_tracks[i]) {
        tracks_[valid_track_count] = std::move(tracks_[i]);
        ++valid_track_count;
      }
    }
    tracks_.resize(valid_track_count);
    // Association and update roll back mechanism.
    const int rollback_group_num =
        measurement_history_groups_.size() - start_roll_back_group_index;
    for (int i = start_roll_back_group_index;
         i < measurement_history_groups_.size(); ++i) {
      pose_ = pose_history_.value(i);
      coordinate_converter_ = &(coordinate_converter_history_.value(i));
      // Associate and update.
      const auto& curr_measurements_group =
          measurement_history_groups_.value(i);

      const auto curr_group_type = curr_measurements_group.group_type();

      auto* group_debug_proto = tracker_debug_proto_->add_group_debug_proto();
      group_debug_proto->set_is_rollback_group(true);
      AssociateMeasurementsAndUpdateTracks(
          camera_images, measurement_history_groups_.value(i).max_timestamp(),
          curr_group_type, measurement_history_groups_.value(i),
          group_debug_proto);
      // Add debug info.
      const int rollback_group_index = i - start_roll_back_group_index + 1;
      AddTracksInfoToDebugProto(
          curr_group_type, rollback_group_num, rollback_group_index,
          curr_measurements_group.min_timestamp(),
          curr_measurements_group.max_timestamp(), &tracks_, group_debug_proto);
    }
  } else {
    const auto& to_precess_group = measurement_history_groups_.back().second;
    auto* group_debug_proto = tracker_debug_proto_->add_group_debug_proto();
    group_debug_proto->set_is_rollback_group(false);
    AssociateMeasurementsAndUpdateTracks(
        camera_images, to_precess_group.max_timestamp(), group_type,
        to_precess_group, group_debug_proto);
    // Add debug info.
    AddTracksInfoToDebugProto(
        to_precess_group.group_type(), 1, 1, to_precess_group.min_timestamp(),
        to_precess_group.max_timestamp(), &tracks_, group_debug_proto);
  }

  // Update latest_tracked_time_.
  for (const auto& track : tracks_) {
    latest_tracked_time_ =
        std::max(latest_tracked_time_, track->track_state.last_timestamp);
  }
}

void Tracker::AssociateMeasurementsAndUpdateTracks(
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    double timestamp, MeasurementsProto::GroupType group_type,
    const MeasurementsProto& measurements_proto,
    TrackerDebugProto* group_debug_proto) {
  SCOPED_QTRACE("Tracker::AssociateMeasurementsAndUpdateTracks");

  const auto laser_measurements = GetMeasurementsWithType(
      measurements_proto, MeasurementProto::kLaserMeasurement);
  const auto camera_measurements = GetMeasurementsWithType(
      measurements_proto, MeasurementProto::kCamera3DMeasurement);
  const auto radar_measurements = GetValidRadarMeasurements(measurements_proto);

  // On the very first iteration when there are no tracks yet, create a
  // track from each measurement.
  // TODO(yu): Init track from camera measurement when necessary.
  if (tracks_.empty()) {
    CreateNewTracksFromMeasurements(laser_measurements);
    CreateNewTracksFromMeasurements(radar_measurements);
    CreateNewTracksFromMeasurements(camera_measurements);
    if (coordinate_converter_) {
      track_classifier_.ClassifyTracks(
          *coordinate_converter_, pose_, camera_images, latest_label_frame_,
          group_debug_proto->mutable_classifier_debug_info(), &tracks_);
    }
    return;
  }

  // Association
  std::vector<int> laser_m_matches;
  if (!laser_measurements.empty()) {
    laser_m_matches = laser_associator_.Association1v1(
        timestamp, laser_measurements, tracks_, coordinate_converter_,
        semantic_map_manager_,
        group_debug_proto->mutable_association_debug_info());
    group_debug_proto->mutable_association_debug_info()->set_sensor_group_type(
        SGT_LIDAR);
    QCHECK_EQ(laser_m_matches.size(), laser_measurements.size());
  }

  std::vector<int> camera_m_matches;
  if (!camera_measurements.empty()) {
    camera_m_matches = camera_associator_.Association1v1(
        timestamp, camera_measurements, tracks_, coordinate_converter_,
        semantic_map_manager_,
        group_debug_proto->mutable_association_debug_info());
    group_debug_proto->mutable_association_debug_info()->set_sensor_group_type(
        SGT_CAMERA);
    QCHECK_EQ(camera_m_matches.size(), camera_measurements.size());
  }

  std::vector<int> radar_m_matches;
  if (!radar_measurements.empty()) {
    radar_m_matches = radar_associator_.AssociationMv1(
        timestamp, radar_measurements, tracks_, coordinate_converter_,
        semantic_map_manager_,
        group_debug_proto->mutable_association_debug_info());
    group_debug_proto->mutable_association_debug_info()->set_sensor_group_type(
        SGT_RADAR);
  }

  if (FLAGS_track_association_cvs) {
    // TODO(zheng): add radar measurements association visualization
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/tracker_association");
    const double render_height = pose_.z + 20.;
    for (int i = 0; i < laser_m_matches.size(); ++i) {
      // Use a random color to differentiate different matched pairs.
      const vis::Color color = GenerateRandomColor(laser_m_matches[i]);

      const auto cluster_contour = geometry_util::ToPolygon2d(
          laser_measurements[i]->laser_measurement().contour());
      tracker_util::RenderContourCvs(cluster_contour, render_height + 0.5,
                                     color, laser_m_matches[i] >= 0 ? 2 : 1,
                                     &canvas, vis::BorderStyleProto::DASHED);
      if (laser_m_matches[i] >= 0) {
        tracker_util::RenderContourCvs(
            tracks_[laser_m_matches[i]]->track_state.contour, render_height,
            color, 2, &canvas);
      }
    }
    for (int i = 0; i < camera_m_matches.size(); ++i) {
      // Use a random color to differentiate different matched pairs.
      const vis::Color color = GenerateRandomColor(camera_m_matches[i]);
      const auto& camera_measurement =
          camera_measurements[i]->camera_measurement();
      RenderCameraMeasurement(camera_measurement, render_height, &canvas,
                              camera_m_matches[i] >= 0 ? 2 : 1, color);
      if (camera_m_matches[i] >= 0) {
        canvas.DrawLine(
            {Vec2dFromProto(camera_measurement.center()), render_height},
            {tracker_util::ComputeCentroid(
                 tracks_[camera_m_matches[i]]->track_state.contour.points()),
             render_height},
            color);
      }
    }
  }

  std::vector<std::vector<const MeasurementProto*>>
      matched_measurements_per_track(tracks_.size());
  std::vector<const MeasurementProto*> unmatched_measurements_laser;
  unmatched_measurements_laser.reserve(laser_m_matches.size());
  for (int i = 0; i < laser_m_matches.size(); ++i) {
    const int track_ind = laser_m_matches[i];
    if (track_ind >= 0) {
      matched_measurements_per_track[track_ind].push_back(
          laser_measurements[i]);
    } else {
      unmatched_measurements_laser.push_back(laser_measurements[i]);
    }
  }

  // Camera measurement
  std::vector<const MeasurementProto*> unmatched_measurements_camera;
  unmatched_measurements_camera.reserve(camera_m_matches.size());
  for (int i = 0; i < camera_m_matches.size(); ++i) {
    const int track_ind = camera_m_matches[i];
    if (track_ind >= 0) {
      matched_measurements_per_track[track_ind].push_back(
          camera_measurements[i]);
    } else {
      unmatched_measurements_camera.push_back(camera_measurements[i]);
    }
  }

  // Radar measurement.
  std::vector<const MeasurementProto*> unmatched_measurements_radar;
  unmatched_measurements_radar.reserve(radar_m_matches.size());
  for (int i = 0; i < radar_m_matches.size(); ++i) {
    const int track_ind = radar_m_matches[i];
    if (track_ind >= 0) {
      matched_measurements_per_track[track_ind].push_back(
          radar_measurements[i]);
    } else {
      unmatched_measurements_radar.push_back(radar_measurements[i]);
    }
  }

  // Update tracks with laser and camera associations.
  UpdateTracksFromMeasurementsWithRollback(matched_measurements_per_track);

  // Create new tracks for laser measurements without associations.
  CreateNewTracksFromMeasurements(unmatched_measurements_laser);
  CreateNewTracksFromMeasurements(unmatched_measurements_radar);
  CreateNewTracksFromMeasurements(unmatched_measurements_camera);

  if (FLAGS_merge_split_track && group_type != MeasurementsProto::RADAR) {
    track_merge_split_manager_.MergeAndSplitTracks(timestamp, &tracks_);
  }
  if (coordinate_converter_) {
    track_classifier_.ClassifyTracks(
        *coordinate_converter_, pose_, camera_images, latest_label_frame_,
        group_debug_proto->mutable_classifier_debug_info(), &tracks_);
  }

  AdjustTrackAfterClassification();
}

void Tracker::RemoveExpiredTracks(
    MeasurementsProto::GroupType group_type, double timestamp,
    std::shared_ptr<const SensorFovsProto> sensor_fovs_proto) {
  SCOPED_QTRACE("Tracker::RemoveExpiredTracks");

  const auto lidar_sensor_fov =
      tracker_util::ComputeLidarSensorFov(sensor_fovs_proto);

  std::set<uint32_t> track_ids_to_delete;
  for (int i = 0; i < tracks_.size(); ++i) {
    auto& track = *tracks_[i];

    const bool is_current_group_contain_latest_measurement_type =
        tracker_util::IsMeasurementGroupContainSpecificMeasurementType(
            group_type,
            track.measurement_history.back_value()->Measurement_case());
    const double life_time_buffer =
        is_current_group_contain_latest_measurement_type
            ? kTrackLifeTimeBufferForSameSensorType
            : kTrackLifeTimeBufferForDifferentSensorType;  // s
    double max_allowed_no_update_time_for_lost_state;
    double max_allowed_no_update_time_for_idle_state;
    bool is_track_in_lidar_occluded_area =
        tracker_util::IsTrackInLidarOccludedArea(lidar_sensor_fov, track,
                                                 timestamp);
    track_life_manager::TrackLifeManager::ComputeMaxAllowedNoUpdatedTime(
        track, life_time_buffer, is_track_in_lidar_occluded_area,
        &max_allowed_no_update_time_for_lost_state,
        &max_allowed_no_update_time_for_idle_state);
    if (track_life_manager::TrackLifeManager::ShouldBeAtIdleState(
            track, pose_.coord().block<2, 1>(0, 0), timestamp,
            max_allowed_no_update_time_for_idle_state)) {
      track.track_state.life_state = TrackLifeState::kIdle;
      track_ids_to_delete.insert(track.track_state.id);
      AddDeletedTrackInfoToDebugProto(track);
    } else if (track_life_manager::TrackLifeManager::ShouldBeAtLostState(
                   track, timestamp,
                   max_allowed_no_update_time_for_lost_state)) {
      track.track_state.life_state = TrackLifeState::kLost;
    }
  }
  // Remove Idle(dead) tracks.
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const auto& track) {
                                 return ContainsKey(track_ids_to_delete,
                                                    track->track_state.id);
                               }),
                tracks_.end());
}

// TODO(zheng): Replace this strategy by using classification result
// after we have more detail classify result.
bool Tracker::ShouldUsePointModelCP(const Track<TrackState>& track) const {
  const Vec2d centroid =
      tracker_util::ComputeCentroid(track.track_state.contour.points());

  // For objects close to lane, we do not use CP model.
  constexpr double kInLaneThreshold = 1.5;  // m
  if (tracker_util::IsInLane(*semantic_map_manager_, centroid,
                             kInLaneThreshold)) {
    return false;
  }

  // TODO(zheng): Also use cp motion model on vision only tracks.
  const bool has_lidar_measurements = TMSTM::HasLidarMeasurements(track);
  if (!has_lidar_measurements) {
    return false;
  }

  const bool is_in_detection_range = tracker_util::IsInFenDetectionRange(
      {centroid.x(), centroid.y(), pose_.z}, pose_);
  if (track.track_state.type == MT_VEHICLE ||
      track.track_state.type == MT_CYCLIST ||
      track.track_state.type == MT_MOTORCYCLIST ||
      track.track_state.type == MT_PEDESTRIAN ||
      track.track_state.type == MT_CYCLIST ||
      track.track_state.type == MT_UNKNOWN || !is_in_detection_range) {
    return false;
  }

  if (track.measurement_history.size() > 1) {
    // Get measurement history info.
    const auto& m_history = track.measurement_history;
    const int num_m = m_history.size();
    const double last_m_timestamp = m_history.back_time();
    constexpr double kDurationAsValidMeasurement = 0.5;  // s

    std::vector<std::pair<double, Vec2d>> icp_vel_vec;
    double max_vel_angular = 0.0;
    double max_acc = 0.0;
    const double kSpeedNoise = 0.25;  // m/s

    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_laser_measurement()) {
        const auto& laser_m = m->laser_measurement();
        // Nowadays, CP mainly handle some shooter issues caused by fence,
        // cone, and other static objects. If the track has lidar detection
        // result, the ref point is stable, rarely produce shooters, so there
        // is no need to apply CP motion model on it.
        if (laser_m.has_detection_bounding_box()) {
          return false;
        }
        if (laser_m.has_icp_measurement() &&
            laser_m.icp_measurement().has_vel()) {
          const auto& icp_m = laser_m.icp_measurement();
          const auto m_vel = Vec2dFromProto(icp_m.vel());
          if (m_vel.norm() > kSpeedNoise) {
            icp_vel_vec.emplace_back(
                std::make_pair(icp_m.matched_curr_cluster_timestamp(), m_vel));
          }
        }
      }
    }

    // Use icp vel in track to judge if the motion is brown moving.
    for (int i = 1; i < icp_vel_vec.size(); ++i) {
      const double vel_angle_diff = icp_vel_vec[i].second.FastAngle() -
                                    icp_vel_vec[i - 1].second.FastAngle();
      const double vel_norm_diff =
          icp_vel_vec[i].second.norm() - icp_vel_vec[i - 1].second.norm();
      const double time_diff = icp_vel_vec[i].first - icp_vel_vec[i - 1].first;
      const double vel_angular =
          std::abs(NormalizeAngle(vel_angle_diff) / (time_diff + DBL_EPSILON));
      const double vel_acc =
          std::abs(vel_norm_diff / (time_diff + DBL_EPSILON));
      max_vel_angular = std::max(vel_angular, max_vel_angular);
      max_acc = std::max(vel_acc, max_acc);
    }
    // Note(zheng): If the max velocity angular and acceleration is greater
    // than the threshold, we treat the unkonwn object as brown moving. The
    // prior is the onroad moving object's velocity angular and acceleration
    // is always lower than a threshold.
    constexpr int kMaxVelAngular = M_PI * 150.0 / 180.0;  // radian
    constexpr int kMaxVelAcc = 50.0;                      // m/s^2
    if (max_acc > kMaxVelAcc || max_vel_angular > kMaxVelAngular) {
      return true;
    } else {
      return false;
    }
  }
  return true;
}

bool Tracker::IsIcpVelConverged(
    const Track<TrackState>& track,
    const LaserMeasurementProto& curr_laser_measurement) const {
  constexpr int kMinLengthMeasurementHistory = 2;
  if (track.measurement_history.size() < kMinLengthMeasurementHistory)
    return false;
  // Get measurement history info.
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  const double last_m_timestamp = m_history.back_time();
  constexpr double kDurationAsValidMeasurement = 0.5;  // s

  std::vector<std::pair<double, Vec2d>> icp_vel_vec;
  double max_vel_angular = 0.0;
  double max_acc = 0.0;
  const double kIcpSpeedNoise = 0.10;  // m/s

  for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                 kDurationAsValidMeasurement);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_icp_measurement() &&
        m->laser_measurement().icp_measurement().has_vel()) {
      const auto& m_icp = m->laser_measurement().icp_measurement();
      const auto m_vel = Vec2dFromProto(m_icp.vel());
      if (m_vel.norm() > kIcpSpeedNoise) {
        icp_vel_vec.emplace_back(
            std::make_pair(m_icp.matched_curr_cluster_timestamp(), m_vel));
      }
    }
  }
  if (icp_vel_vec.size() < kMinLengthMeasurementHistory) {
    return false;
  }

  if (curr_laser_measurement.has_icp_measurement() &&
      curr_laser_measurement.icp_measurement().has_vel()) {
    const auto& m_icp = curr_laser_measurement.icp_measurement();
    const auto m_vel = Vec2dFromProto(m_icp.vel());
    icp_vel_vec.emplace_back(
        std::make_pair(m_icp.matched_curr_cluster_timestamp(), m_vel));
  } else {
    return false;
  }

  // Use max velocity angular and max acc to judge if the icp is converged.
  for (int i = 1; i < icp_vel_vec.size(); ++i) {
    const double time_diff = icp_vel_vec[i].first - icp_vel_vec[i - 1].first;
    if (icp_vel_vec[i].second.norm() > kIcpSpeedNoise &&
        icp_vel_vec[i - 1].second.norm() > kIcpSpeedNoise) {
      const double vel_angle_diff = icp_vel_vec[i].second.FastAngle() -
                                    icp_vel_vec[i - 1].second.FastAngle();
      const double vel_angular =
          std::fabs(NormalizeAngle(vel_angle_diff) / (time_diff + DBL_EPSILON));
      max_vel_angular = std::max(vel_angular, max_vel_angular);
    }

    const double vel_norm_diff =
        icp_vel_vec[i].second.norm() - icp_vel_vec[i - 1].second.norm();

    const double vel_acc = std::fabs(vel_norm_diff / (time_diff + DBL_EPSILON));
    max_acc = std::max(vel_acc, max_acc);
  }
  // Note(zheng): If the max velocity angular and acceleration is greater
  // than the threshold, we treat the icp velocity is not converged. The
  // prior is the onroad moving object's velocity angular and acceleration
  // is always lower than a threshold.
  constexpr int kMaxVelAngular = M_PI * 150.0 / 180.0;  // radian
  constexpr int kMaxVelAcc = 20.0;                      // m/s^2
  constexpr int kMaxPedVelAcc = 20.0;                   // m/s^2
  const double acc_threshlod =
      track.track_state.type == MT_PEDESTRIAN ? kMaxPedVelAcc : kMaxVelAcc;
  if (max_acc > acc_threshlod || max_vel_angular > kMaxVelAngular) {
    return false;
  }

  return true;
}

bool Tracker::IsInBlindArea(const Track<TrackState>& track) const {
  bool is_in_blind_area = false;
  constexpr double kBlindZoneMinX = -6.0;
  constexpr double kBlindZoneMaxX = 12.0;
  constexpr double kBlindZoneMinY = -5.0;
  constexpr double kBlindZoneMaxY = 5.0;
  const auto& min_box = track.track_state.contour.BoundingBoxWithHeading(
      track.track_state.heading);
  const auto& center = pose_inv_.TransformPoint(
      {min_box.center().x(), min_box.center().y(), pose_.z});
  if (center.x() >= kBlindZoneMinX && center.x() <= kBlindZoneMaxX &&
      center.y() >= kBlindZoneMinY && center.y() <= kBlindZoneMaxY) {
    is_in_blind_area = true;
  }
  return is_in_blind_area;
}

Vec3d Tracker::GetAnchorPosForRefPoint() const {
  const Vec3d anchor_pos =
      pose_.ToTransform().TransformPoint(ref_point_anchor_pos_);
  return anchor_pos;
}

void Tracker::UpdateRunParams(const RunParamsProtoV2& run_params) {
  // In jinlv bus we use front two lidar middle pos as anchor pos,
  // while in MKZ, we use the center lidar pos.
  const auto& select_lidar_param =
      perception_util::SelectLidarParams(run_params);
  ref_point_anchor_pos_ =
      Vec3d(select_lidar_param.installation().extrinsics().x(), 0.0,
            select_lidar_param.installation().extrinsics().z());
}

TrackState::MotionFilterType Tracker::GetMotionFilterTypeFromParamType(
    MotionFilterParamType type) {
  if (type == MFPT_HIGHWAY_CAR_MODEL_CTRA_CAR ||
      type == MFPT_URBAN_CAR_MODEL_CTRA_CAR ||
      type == MFPT_HIGHWAY_CAR_MODEL_CTRA_CYC ||
      type == MFPT_URBAN_CAR_MODEL_CTRA_CYC ||
      type == MFPT_HIGHWAY_CAR_MODEL_CTRA_CAR) {
    return TrackState::MotionFilterType::kCarModel;
  } else if (type == MFPT_POINT_MODEL_CP) {
    return TrackState::MotionFilterType::kPointModelCP;
  } else if (type == MFPT_POINT_MODEL_CA || type == MFPT_POINT_MODEL_CA_PED ||
             type == MFPT_POINT_MODEL_CA_CYC) {
    return TrackState::MotionFilterType::kPointModelCA;
  } else {
    QLOG(FATAL) << "track motion filter param type is: " << type
                << " unknown motion filter param type";
  }
}

void Tracker::UpdateEstimatorExtent(const MeasurementProto& measurement,
                                    Track<TrackState>* track) {
  double length = 0.0;
  double width = 0.0;
  auto meas_noise = track->track_state.estimator_3d.GetMeasurementNoise();
  if (measurement.has_camera3d_measurement()) {
    const auto& m = measurement.camera3d_measurement();
    length = m.length();
    width = m.width();
  } else if (measurement.has_laser_measurement()) {
    const auto& m = measurement.laser_measurement();
    if (m.has_detection_bounding_box()) {
      length = m.detection_bounding_box().length();
      width = m.detection_bounding_box().width();
      constexpr double kFENExtentNoise = 0.5;  // m
      meas_noise.set_state_length_measurement_noise_std(kFENExtentNoise);
      meas_noise.set_state_width_measurement_noise_std(kFENExtentNoise);
    } else {
      // Skip extent update if no bounding box
      return;
    }
  }

  if (track->track_state.estimator_3d.extent_is_initialized()) {
    track->track_state.estimator_3d.PredictExtent(measurement.timestamp());
    BBoxMeasurement bbox_meas(length, width, /*height=*/0.0);
    track->track_state.estimator_3d.UpdateExtent(bbox_meas, meas_noise);
  } else {
    BBoxState bbox_state(length, width, /*height=*/0.0);
    track->track_state.estimator_3d.InitExtent(bbox_state);
  }
}

Box2d Tracker::ComputeBBoxFromEstimatorExtent(const Track<TrackState>& track) {
  const auto s = track.track_state.estimator_3d.GetStateData();
  const Vec2d m_pos = s.GetStatePos();
  const auto extent = track.track_state.estimator_3d.GetExtentStateData();
  const double heading = s.GetYaw();
  const double length = extent.state().length();
  const double width = extent.state().width();

  Box2d bbox(m_pos, heading, length, width);

  return bbox;
}

void Tracker::UpdateTrackState(const MeasurementProto& measurement,
                               Track<TrackState>* track) {
  // Tracklet state update
  constexpr double kYawRateSmoothScale = 0.7;
  constexpr double kMaxYawRate = d2r(60.0);  // 60 degree/s

  if (track->track_state.estimator_3d.IsCarModel()) {
    const StateData s = track->track_state.estimator_3d.GetStateData();
    track->track_state.yaw_rate = s.GetYawD();
  } else {
    const double prev_vel = track->track_state.vel;
    const StateData s = track->track_state.estimator_3d.GetStateData();
    const double curr_vel = s.GetVel();
    constexpr double kSpeedNoise = 0.25;
    if (curr_vel > kSpeedNoise && prev_vel > kSpeedNoise) {
      const double curr_heading = s.GetYaw();
      const double delta_t =
          measurement.timestamp() - track->track_state.last_timestamp;
      const double curr_yaw_rate =
          (curr_heading - track->track_state.heading) / (delta_t + DBL_EPSILON);
      track->track_state.yaw_rate =
          kYawRateSmoothScale * track->track_state.yaw_rate +
          (1.0 - kYawRateSmoothScale) * curr_yaw_rate;
    }
  }
  const StateData s = track->track_state.estimator_3d.GetStateData();
  track->track_state.vel = s.GetVel();
  track->track_state.heading = s.GetYaw();

  // If the yaw rate is greater than kMaxYawRate or the track type is
  // pedestrian, we set yaw rate to zero.
  if (track->track_state.yaw_rate > kMaxYawRate ||
      track->track_state.type == MT_PEDESTRIAN) {
    track->track_state.yaw_rate = 0.0;
  }
  if (track_life_manager::TrackLifeManager::ShouldBeAtConfirmedState(
          *track, pose_, measurement.timestamp())) {
    track->track_state.life_state = TrackLifeState::kConfirmed;
  }

  track->track_state.last_timestamp = measurement.timestamp();
  track->track_state.state_timestamp = measurement.timestamp();
}

void Tracker::ResetDebugProto() {
  tracker_debug_proto_ = std::make_unique<TrackerDebugProtoWithRollback>();
}
bool Tracker::ShouldUseVisionPredcitedShape(
    const Track<TrackState>& track,
    const LaserMeasurementProto& laser_measurement) const {
  if (track.checkpoints.empty() || track.measurement_history.empty()) {
    return false;
  }

  constexpr double kMaxSearchDuration = 0.4;  // s
  constexpr int kMinHasBboxLaserMeasurementNum = 3;
  const int checkpoints_num = track.checkpoints.size();
  const int m_num = track.measurement_history.size();
  const double last_measurement_timestamp =
      track.measurement_history.back_time();
  bool has_vision_track_type = false;
  for (int j = track.checkpoints.GetIndexWithTimeAtLeast(
           last_measurement_timestamp - kMaxSearchDuration);
       j < checkpoints_num; ++j) {
    const TrackState& t0 = track.checkpoints.value(j);
    if (t0.measurement_source_type == TrackMeasurementSourceType::TMST_VO ||
        t0.measurement_source_type == TrackMeasurementSourceType::TMST_VR) {
      has_vision_track_type = true;
      break;
    }
  }
  int has_bbox_laser_measurement_count =
      laser_measurement.has_detection_bounding_box() ? 1 : 0;
  for (int j = track.measurement_history.GetIndexWithTimeAtLeast(
           last_measurement_timestamp - kMaxSearchDuration);
       j < m_num; ++j) {
    const auto* m = track.measurement_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_detection_bounding_box()) {
      has_bbox_laser_measurement_count++;
    }
  }

  return has_vision_track_type &&
         has_bbox_laser_measurement_count < kMinHasBboxLaserMeasurementNum;
}

}  // namespace qcraft::tracker
