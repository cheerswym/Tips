#include "onboard/perception/tracker/track_classifier/track_data_storage.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/perception/tracker/track_classifier/label_convert_utils.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/vis/canvas/canvas.h"

DECLARE_bool(track_label_match_cvs);

namespace qcraft::tracker {
namespace {
void MaybeRenderTrackLabelMatch(
    const Track<TrackState>& track,
    const track_classifier_data::TrackData& track_data,
    const int label_object_id, const double contour_overlap_ratio,
    const Polygon2d& intersection) {
  if (ABSL_PREDICT_TRUE(!FLAGS_track_label_match_cvs)) {
    return;
  }
  const auto to_color = [&](const track_classifier_label::Category type) {
    switch (type) {
      case track_classifier_label::VEH_CAR:
      case track_classifier_label::VEH_VAN:
      case track_classifier_label::VEH_TRUCK:
      case track_classifier_label::VEH_BUS:
      case track_classifier_label::VEH_CONSTRUCTION:
        return vis::Color::kMagenta;
      case track_classifier_label::PED_ADULT:
      case track_classifier_label::PED_CHILD:
        return vis::Color::kDarkYellow;
      case track_classifier_label::XCY_BICYCLIST:
      case track_classifier_label::XCY_MOTORCYCLIST:
        return vis::Color::kYellow;
      case track_classifier_label::XCY_TRICYCLIST:
        return vis::Color::kYellow;
      case track_classifier_label::STA_VEGETATION:
        return vis::Color::kGreen;
      case track_classifier_label::STA_CONE:
        return vis::Color::kLightRed;
      case track_classifier_label::OTH_ANIMAL:
      case track_classifier_label::OTH_FOD:
        return vis::Color::kGray;
      default:
        return vis::Color::kWhite;
    }
  };
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/track_classifier");
  const auto category = static_cast<track_classifier_label::Category>(
      (track_data.labels()[track_data.labels_size() - 1]));
  tracker_util::RenderContourCvs(track.track_state.contour, 0.,
                                 to_color(category), 2, &canvas);
  tracker_util::RenderContourCvs(intersection, 0., to_color(category), 5,
                                 &canvas);
  const auto centroid = track.track_state.anchor_point;

  canvas.DrawText(
      absl::StrFormat("Track (%d) label: %s overlapped with label %d (%.3f)",
                      track.track_state.id,
                      track_classifier_label::Category_Name(category),
                      label_object_id, contour_overlap_ratio),
      {centroid, 0.}, 0., 0.2, to_color(category));
}

int CompareFenLabel(
    const Track<TrackState>& track,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
    track_classifier_label::Category* category_from_fen,
    double* max_contour_intersected_ratio, Polygon2d* max_intersection) {
  int label_ind = -1;

  for (int i = 0; i < latest_label_frame->labels_size(); ++i) {
    const auto& label = latest_label_frame->labels(i);
    const Box2d label_bounding_box({label.x(), label.y()}, label.heading(),
                                   label.length(), label.width());

    Polygon2d intersection;
    if (!track.track_state.contour.ComputeOverlap(Polygon2d(label_bounding_box),
                                                  &intersection)) {
      continue;
    }

    const double contour_intersected_ratio =
        intersection.area() / (track.track_state.contour.area() + DBL_EPSILON);

    // Threshold of contour intersection ratio to be considered as matched.
    constexpr double kContourIntersectedRatioThres = 0.1;
    if (contour_intersected_ratio > kContourIntersectedRatioThres &&
        contour_intersected_ratio > *max_contour_intersected_ratio) {
      // Sometimes the contour is too small compared with the label (only one
      // obstacle of a car for example).
      const double label_intersected_ratio =
          intersection.area() / (label_bounding_box.area() + DBL_EPSILON);
      constexpr double kLabelIntersectedRatioThres = 0.01;
      if (label_intersected_ratio < kLabelIntersectedRatioThres) {
        continue;
      }
      *category_from_fen = ConvertToTcnLabel(label.category());
      label_ind = i;
      *max_contour_intersected_ratio = contour_intersected_ratio;
      *max_intersection = intersection;
    }
  }

  return label_ind;
}

int CompareFenZone(
    const Track<TrackState>& track,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
    track_classifier_label::Category* category_from_fen) {
  int zone_ind = -1;
  for (int i = 0; i < latest_label_frame->zones_size(); ++i) {
    const auto& zone = latest_label_frame->zones(i);
    if (zone.xs_size() <= 2 || zone.xs_size() != zone.ys_size()) {
      LOG(WARNING) << "Invalid zones with xs " << zone.xs_size() << " and "
                   << zone.ys_size();
      continue;
    }
    if (zone.type() == labeling::Zone::WHITE ||
        zone.type() == labeling::Zone::FULL_SEMANTIC) {
      continue;
    }

    std::vector<Vec2d> points;

    points.reserve(zone.xs_size());
    for (int i = 0; i < zone.xs_size(); ++i) {
      points.emplace_back(zone.xs(i), zone.ys(i));
    }
    if (track.track_state.contour.HasOverlap(Polygon2d(points))) {
      *category_from_fen = ConvertToTcnLabel(zone.type());
      zone_ind = i;
      break;
    }
  }
  return zone_ind;
}

}  // namespace

std::unique_ptr<TrackDataStorage> TrackDataStorage::Create(
    const std::string& data_db_prefix, const std::string& key_prefix,
    const SemanticMapManager* semantic_map_manager) {
  return std::unique_ptr<TrackDataStorage>(
      new TrackDataStorage(data_db_prefix, key_prefix, semantic_map_manager));
}

TrackDataStorage::TrackDataStorage(
    const std::string& data_db_prefix, const std::string& key_prefix,
    const SemanticMapManager* semantic_map_manager)
    : semantic_map_manager_(semantic_map_manager) {
  data_db_prefix_ = data_db_prefix;
  key_prefix_ = key_prefix;
  InitLevelDB(data_db_prefix_, "_track_db");
}

void TrackDataStorage::SaveData(
    const Track<TrackState>& track,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const VehiclePose& pose, bool use_fen_label,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame) {
  if (use_fen_label) {
    SaveTrackDataByFen(track, track_classifier_info, pose, latest_label_frame);
  } else {
    SaveTrackDataAll(track, track_classifier_info, pose);
  }
}

void TrackDataStorage::SetTrackData(
    const Track<TrackState>& track, const VehiclePoseProto& pose_proto,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const MeasurementProto* measurement_proto,
    track_classifier_label::Category category,
    track_classifier_data::TrackData* track_data) {
  *track_data->add_measurements() = *measurement_proto;
  track_data->add_labels(category);
  track_data->add_heading(track.track_state.heading);
  *track_data->add_track_classifier_info() = track_classifier_info;
  *track_data->add_vehicle_poses() = pose_proto;
  // set track state data
  auto* track_state = track_data->add_track_state();
  const auto s = track.track_state.estimator_3d.GetStateData();
  track_state->set_vx(s.GetVx());
  track_state->set_vy(s.GetVy());
  track_state->set_vel(s.GetVel());
  track_state->set_ax(s.GetAx());
  track_state->set_ay(s.GetAy());
  track_state->set_acc(s.GetAcc());
  track_state->set_yaw(s.GetYaw());
  track_state->set_yaw_rate(s.GetYawD());

  track_state->set_has_moving_radar_measurement(
      HasMovingRadarMeasurement(track));

  constexpr double kInLaneThreshold = kLaneWidth / 2.0 + 0.05;
  track_state->set_is_in_lane(
      tracker_util::IsInLane(*semantic_map_manager_, track, kInLaneThreshold));
}

track_classifier_data::TrackData TrackDataStorage::UpdateTrackData(
    const Track<TrackState>& track, const std::string& db_key,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const VehiclePose& pose, track_classifier_label::Category category) {
  std::string measurement_history_str;
  const auto key_state =
      data_db_->Get(leveldb::ReadOptions(), db_key, &measurement_history_str);
  track_classifier_data::TrackData track_data;
  const auto pose_proto = pose.ToVehiclePoseProto();
  if (key_state.ok()) {
    track_data.ParseFromString(measurement_history_str);
    const int track_len = track_data.measurements_size();
    QCHECK_GE(track_len, 0);
    // Latest timestamp in saved track_data
    const double t1 = track_data.measurements(track_len - 1).timestamp();
    const int num_m = track.measurement_history.size();
    int ind = num_m - 1;
    // Get the first index of new measurement
    while (ind >= 0 && track.measurement_history.value(ind)->timestamp() - t1 >
                           std::numeric_limits<double>::epsilon()) {
      --ind;
    }
    // Add new measurements and labels
    for (int i = ind + 1; i < num_m; ++i) {
      SetTrackData(track, pose_proto, track_classifier_info,
                   track.measurement_history.value(i), category, &track_data);
    }
  } else {
    for (const auto* m : track.measurement_history.value_range()) {
      if (m == nullptr) {
        continue;
      }
      SetTrackData(track, pose_proto, track_classifier_info, m, category,
                   &track_data);
    }
  }
  return track_data;
}

void TrackDataStorage::SaveTrackDataAll(
    const Track<TrackState>& track,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const VehiclePose& pose) {
  const std::string db_key =
      absl::StrFormat("%s/%d", key_prefix_, track.track_state.id);
  // model result has higher priority
  track_classifier_data::TrackData track_data =
      UpdateTrackData(track, db_key, track_classifier_info, pose,
                      ConvertToTcnLabel(track.track_state.type));

  track_data.set_state_timestamp(track.track_state.state_timestamp);
  const auto s = data_db_->Put(leveldb::WriteOptions(), db_key,
                               track_data.SerializeAsString());
  if (!s.ok()) {
    QLOG(FATAL) << s.ToString();
  }
}

void TrackDataStorage::SaveTrackDataByFen(
    const Track<TrackState>& track,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const VehiclePose& pose,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame) {
  if (latest_label_frame == nullptr) {
    return;
  }

  const std::string db_key =
      absl::StrFormat("%s/%d", key_prefix_, track.track_state.id);

  // Check if current track is aligned with the label frame by chekcing if the
  // last measurement timestamp is within a threshold away from label timestamp.
  // Label frame header timestamp is either mid or first scan timestamp of the
  // corresponding lidar frame group.
  constexpr double kMaxTimeDiff = 0.2;  // s
  if (std::fabs(track.track_state.state_timestamp -
                latest_label_frame->header().timestamp() * 1e-6) >
      kMaxTimeDiff) {
    LOG(ERROR) << absl::StrFormat(
        "Track and label is not synced (track (%d): %.3f vs label: %.3f).",
        track.track_state.id, track.track_state.last_timestamp,
        latest_label_frame->header().timestamp() * 1e-6);
    return;
  }

  // Currently, the track would be considered as matched with a certain label
  // if it's current contour is .
  // TODO(yu): Consider assign the label if the track is matched with the
  // label for a period of time.

  track_classifier_label::Category category_from_fen =
      track_classifier_label::OTH_UNKNOWN;
  double max_contour_intersected_ratio = 0;
  Polygon2d max_intersection;
  const int label_ind =
      CompareFenLabel(track, latest_label_frame, &category_from_fen,
                      &max_contour_intersected_ratio, &max_intersection);

  if (label_ind == -1) {
    CompareFenZone(track, latest_label_frame, &category_from_fen);
  }

  track_classifier_data::TrackData track_data = UpdateTrackData(
      track, db_key, track_classifier_info, pose, category_from_fen);
  track_data.set_state_timestamp(track.track_state.state_timestamp);
  const auto s = data_db_->Put(leveldb::WriteOptions(), db_key,
                               track_data.SerializeAsString());

  MaybeRenderTrackLabelMatch(
      track, track_data,
      (label_ind >= 0) ? latest_label_frame->labels(label_ind).object_id() : -1,
      max_contour_intersected_ratio, max_intersection);

  if (!s.ok()) {
    QLOG(FATAL) << s.ToString();
  }
}
}  // namespace qcraft::tracker
