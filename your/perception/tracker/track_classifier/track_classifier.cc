#include "onboard/perception/tracker/track_classifier/track_classifier.h"

#include <algorithm>
#include <limits>
#include <map>
#include <string>

#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "offboard/labeling/proto/track_classifier_label.pb.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/nets/trt/tcn_net.h"
#include "onboard/perception/geometry_util.h"
#include "onboard/perception/tracker/track_classifier/label_convert_utils.h"
#include "onboard/perception/tracker/track_classifier/tcn_constant.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"
#include "onboard/perception/tracker/tracker_util.h"
#include "onboard/vis/canvas/canvas.h"

DEFINE_bool(save_tcn_training_data, false,
            "Whether to save training data for track classifier network.");
DEFINE_bool(use_fen_label, false,
            "Whether to save training data by fen label.");
DEFINE_string(tcn_training_data_db_prefix, "/hosthome/car_data/track_cls/tcn",
              "Path to the training data directory.");
DEFINE_string(
    tcn_key_prefix, "",
    "Key prefix of the tcn training data db, usually run name would suffice.");

DEFINE_bool(track_label_match_cvs, false,
            "Whether to show match result between track and label on canvas.");
DEFINE_bool(enable_tcn_model, true, "Whether to enable tcn model or not");
DEFINE_bool(tcn_requires_image, true,
            "Whether tcn must have corresponding image and point");

namespace qcraft::tracker {

namespace {
void MaybeRenderLabel(
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame) {
  if (ABSL_PREDICT_TRUE(!FLAGS_track_label_match_cvs)) {
    return;
  }
  const auto to_color = [&](const labeling::Label::Category label) {
    switch (label) {
      case labeling::Label::Category::Label_Category_CAR:
      case labeling::Label::Category::Label_Category_TRAILER:
      case labeling::Label::Category::Label_Category_CAR_DOOR:
        return vis::Color::kMagenta;
      case labeling::Label::Category::Label_Category_PED:
      case labeling::Label::Category::Label_Category_CHILD:
        return vis::Color::kDarkYellow;
      case labeling::Label::Category::Label_Category_BICYCLIST:
      case labeling::Label::Category::Label_Category_MOTORCYCLIST:
      case labeling::Label::Category::Label_Category_TRICYCLIST:
        return vis::Color::kYellow;
      case labeling::Label::Category::Label_Category_ANIMAL:
      case labeling::Label::Category::Label_Category_UNKNOWN:
        return vis::Color::kWhite;
      default:
        QLOG(FATAL) << "Should not reach here.";
    }
  };
  for (const auto& label : latest_label_frame->labels()) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/track_classifier");
    canvas.DrawBox({label.x(), label.y(), 0.}, label.heading(),
                   {label.length(), label.width()}, to_color(label.category()));
  }
}
}  // namespace
TrackClassifier::TrackClassifier(const SemanticMapManager* semantic_map_manager,
                                 const ParamManager& param_manager,
                                 ThreadPool* thread_pool)
    : semantic_map_manager_(semantic_map_manager), thread_pool_(thread_pool) {
  if (ABSL_PREDICT_FALSE(FLAGS_save_tcn_training_data)) {
    track_data_storage_ =
        TrackDataStorage::Create(FLAGS_tcn_training_data_db_prefix,
                                 FLAGS_tcn_key_prefix, semantic_map_manager);
  }
  if (FLAGS_enable_tcn_model) {
    QLOG(INFO) << "Init TcnSequenceNetClassifier ...";
    RunParamsProtoV2 run_params;
    param_manager.GetRunParams(&run_params);
    NetParam tcn_net_param;
    CHECK_OK(
        param_manager.GetProtoParam("tcn_sequence_net_param", &tcn_net_param));
    tcn_sequence_net_classifier_ =
        std::make_unique<TcnSequenceNetClassifier>(run_params, tcn_net_param);
    NetParam warning_triangle_net_param;
    CHECK_OK(param_manager.GetProtoParam("warning_triangle_net_param",
                                         &warning_triangle_net_param));
    warning_triangle_classifier_ = std::make_unique<TcnImageNetClassifier>(
        run_params, warning_triangle_net_param);
    QLOG(INFO) << "Init TcnSequenceNetClassifier Done";
  }
}

void TrackClassifier::MaybeSaveTrackData(
    const Track<TrackState>& track,
    const TrackClassifierDebugProto::TrackClassifierInfo& track_classifier_info,
    const VehiclePose& pose,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame) {
  if (ABSL_PREDICT_FALSE(FLAGS_save_tcn_training_data)) {
    track_data_storage_->SaveData(track, track_classifier_info, pose,
                                  FLAGS_use_fen_label, latest_label_frame);
  }
}

bool TrackClassifier::IsFence(const Track<TrackState>& track) {
  constexpr double kMaxDistToLaneForFence = 0.5;  // m
  constexpr double kMaxWidthForFence = 0.8;       // m
  constexpr double kMinLengthWidthRatio = 5.0;

  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  if (num_m == 0) {
    return false;
  }

  // If the measurement history have detecton result, we do not treat it as
  // fence.
  for (int j = m_history.GetIndexWithTimeAtLeast(
           track.track_state.last_timestamp - 0.3);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement() &&
        m->laser_measurement().has_detection_bounding_box()) {
      return false;
    }
  }
  // We use the info of dist to lane_boundary and length / width to judge if the
  // unknown object is fence, most of the fence's length / width is greater than
  // a threshold, and it's pos is close to lane, so we can use these info to
  // classify unknown movable object to fence, this can suppress some shooters.
  const auto ego_level = semantic_map_manager_->GetLevel();
  for (int j = m_history.GetIndexWithTimeAtLeast(
           track.track_state.last_timestamp - 0.3);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement()) {
      const auto measurement_contour =
          geometry_util::ToPolygon2d(m->laser_measurement().contour());
      const auto m_pos = measurement_contour.centroid();
      double min_dist = std::numeric_limits<double>::max();
      Segment2d lane_segment;

      if (const auto* lane_boundary_info =
              semantic_map_manager_->GetNearestLaneBoundaryInfoAtLevel(
                  ego_level, m_pos)) {
        // Compute distance to nearest lane boundary
        for (int k = 0; k < lane_boundary_info->points_smooth.size() - 1; ++k) {
          const Vec2d start = lane_boundary_info->points_smooth[k];
          const Vec2d end = lane_boundary_info->points_smooth[k + 1];
          const Segment2d segment(start, end);
          const double dist = segment.DistanceTo(m_pos);
          if (dist < min_dist) {
            min_dist = dist;
            lane_segment = segment;
          }
        }
      }

      if (min_dist < kMaxDistToLaneForFence) {
        const double lane_heading = lane_segment.heading();
        const auto m_bbox =
            measurement_contour.BoundingBoxWithHeading(lane_heading);
        const double ratio = m_bbox.length() / m_bbox.width();
        if (ratio > kMinLengthWidthRatio &&
            m_bbox.width() < kMaxWidthForFence) {
          return true;
        }
      }
    }
  }
  return false;
}

// Bird logic:
// Normally bird would pass across lane ahead of us causing hard brake. The
// track would be regarded as bird if: it's moving fast, high above ground,
// small in x-y span as well as z span.
bool TrackClassifier::IsFlyingBird(const Track<TrackState>& track) {
  // Only consider those with type MT_UNKNOWN, MT_STATIC_OBJECT or
  // MT_VEGETATION by LLN.
  if (track.track_state.type != MT_UNKNOWN &&
      track.track_state.type != MT_STATIC_OBJECT &&
      track.track_state.type != MT_VEGETATION) {
    return false;
  }

  constexpr double kBirdMaxArea = 0.2;       // m^2.
  constexpr double kBirdMinClearance = 0.2;  // m.
  constexpr double kBirdMaxZSpan = 0.4;      // m.
  constexpr double kMinLikelyBirdRatio = 0.75;
  constexpr double kMinBirdSpeed = 3;  // m/s.

  const auto& m_history = track.measurement_history;
  int num_laser_m = 0;
  int num_laser_m_likely_bird = 0;
  Vec2d first_m_centroid;
  Vec2d last_m_centroid;
  double first_m_timestamp = 0.;
  double last_m_timestamp;
  constexpr double kDurationAsValidMeasurement = 0.5;  // s
  for (int j = m_history.GetIndexWithTimeAtLeast(
           track.track_state.last_timestamp - kDurationAsValidMeasurement);
       j < m_history.size(); ++j) {
    const auto* m = m_history.value(j);
    if (m->has_laser_measurement()) {
      const auto& cluster_m = m->laser_measurement().cluster_measurement();
      const double timestamp = cluster_m.timestamp();
      const auto measurement_contour =
          geometry_util::ToPolygon2d(m->laser_measurement().contour());
      const auto contour_centroid = measurement_contour.centroid();
      const auto contour_area = measurement_contour.area();
      // Compute speed if it's not the first laser measurement.
      if (first_m_timestamp == 0.) {
        first_m_timestamp = timestamp;
        first_m_centroid = contour_centroid;
      }
      // Directly return if there's any measurment that's not sourced from LLN
      // for vegetation track.
      if (track.track_state.type == MT_VEGETATION &&
          m->type() == MT_VEGETATION && m->type_source() != MTS_LL_NET) {
        return false;
      }
      last_m_centroid = contour_centroid;
      last_m_timestamp = timestamp;

      const double z_span = cluster_m.height() - cluster_m.clearance();
      if (contour_area <= kBirdMaxArea &&
          cluster_m.clearance() > kBirdMinClearance && z_span < kBirdMaxZSpan) {
        ++num_laser_m_likely_bird;
      }
      ++num_laser_m;
    }
  }

  // Skip if it consists less than 2 frames of laser measurement.
  if (num_laser_m < 2) {
    return false;
  }

  const double averaged_speed = (last_m_centroid - first_m_centroid).norm() /
                                (std::numeric_limits<double>::epsilon() +
                                 last_m_timestamp - first_m_timestamp);
  const double likely_bird_ratio =
      static_cast<double>(num_laser_m_likely_bird) /
      static_cast<double>(num_laser_m);

  return averaged_speed > kMinBirdSpeed &&
         likely_bird_ratio > kMinLikelyBirdRatio;
}

bool TrackClassifier::IsPossiblyWarningTriangle(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m,
    MeasurementType type) {
  constexpr double NearGroundThreshlod = 0.4;  // m
  constexpr double MaxPossiblyHeight = 0.9;    // m
  if (type != MT_STATIC_OBJECT && type != MT_CONE) {
    return false;
  }
  // Check if the cluster possibly on the ground
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::min();
  for (const auto& info : cluster_m.obstacle_info()) {
    min_z = std::min(min_z, info.min_z());
    max_z = std::max(max_z, info.max_z());
  }
  return (min_z - cluster_m.ground_z() < NearGroundThreshlod) &&
         (max_z - cluster_m.ground_z() < MaxPossiblyHeight);
}

void TrackClassifier::ClassifyWarningTriangle(
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    std::vector<TrackRef>* tracks) {
  constexpr int kImageShortSideMin = 10;
  constexpr int kImageLongSideMax = 300;
  constexpr float kImageAspectRatioMin = 0.25;
  std::vector<int> index;
  std::vector<cv::Mat> images;
  SCOPED_QTRACE("TrackClassifier::ClassifyWarningTriangle");
  index.reserve(kWarningTriangleMaxBatchSize);
  images.reserve(kWarningTriangleMaxBatchSize);
  for (int i = 0; i < tracks->size(); ++i) {
    auto& track = *(*tracks)[i];

    const double last_traggered_measurement_timestamp =
        tracker_util::GetLatestLaserOrCameraMeasurementTimestamp<TrackState>(
            track);
    if (last_traggered_measurement_timestamp -
            track.track_state.classification_timestamp <
        std::numeric_limits<double>::epsilon()) {
      continue;
    }
    const auto& m_history = track.measurement_history;
    if (m_history.empty()) continue;
    const auto* m = m_history.back_value();
    if (m->has_laser_measurement()) {
      QCHECK(m->laser_measurement().has_cluster_measurement());
      MeasurementType type = m->type();
      const auto& cluster_m = m->laser_measurement().cluster_measurement();
      if (!IsPossiblyWarningTriangle(cluster_m, type)) continue;
      const auto& img_patch = GetImagePatchForClusterMeasurement(
          cluster_m, camera_images, kImageShortSideMin, kImageLongSideMax,
          kImageAspectRatioMin);
      if (img_patch.ok()) {
        index.push_back(i);
        images.push_back(*img_patch);
      } else {
        VLOG(2) << img_patch.status();
      }

      if (index.size() == kWarningTriangleMaxBatchSize) {
        RunWarningTriangleClassifyModel(images, index, tracks);
        images.clear();
        index.clear();
      }
    }
  }
  if (!index.empty()) {
    RunWarningTriangleClassifyModel(images, index, tracks);
  }
}

void TrackClassifier::RunWarningTriangleClassifyModel(
    const std::vector<cv::Mat>& images, const std::vector<int>& index,
    std::vector<TrackRef>* tracks) {
  const auto& result =
      warning_triangle_classifier_->ExtractImageFeature(images);
  constexpr double kMinClassConfidenceScore = 0.8;
  constexpr int kWarningTriangleConfidenceIndex = 1;
  constexpr int kConeConfidenceIndex = 2;
  constexpr int kTriangleModelClassNumber = 3;
  QCHECK_EQ(result.size(), index.size() * 3);

  for (int i = 0; i < index.size(); ++i) {
    auto confidence_triangle =
        result[i * kTriangleModelClassNumber + kWarningTriangleConfidenceIndex];
    auto confidence_cone =
        result[i * kTriangleModelClassNumber + kConeConfidenceIndex];
    if (confidence_triangle > kMinClassConfidenceScore) {
      (*tracks)[index[i]]->track_state.type = MT_WARNING_TRIANGLE;
      QEVENT("xuyao", "warning_triangle_detected", [&](QEvent* qevent) {
        qevent->AddField("object", "warning triangle")
            .AddField("confidence", confidence_triangle)
            .AddField("track_id", index[i]);
      });
    } else if (confidence_cone > kMinClassConfidenceScore) {
      (*tracks)[index[i]]->track_state.type = MT_CONE;
      QEVENT("xuyao", "cone_detected", [&](QEvent* qevent) {
        qevent->AddField("object", "cone")
            .AddField("confidence", confidence_cone)
            .AddField("track_id", index[i]);
      });
    }
  }
}

void TrackClassifier::CollectSequenceFeature(
    const HistoryBuffer<const MeasurementProto*>& m_history,
    std::vector<Feature>* feature_seq) {
  const int sequence_length = tcn_sequence_net_classifier_->GetSequenceLength();
  const int feature_length = tcn_sequence_net_classifier_->GetFeatureLength();
  const int num_m = m_history.size();
  // collect sequence feature
  feature_seq->clear();
  Feature feature;
  feature.resize(feature_length);
  for (int j = num_m - 1; j >= 0 && feature_seq->size() < sequence_length;
       --j) {
    const auto* m = m_history.value(j);
    if (!m->has_laser_measurement()) continue;
    const auto& laser_m = m->laser_measurement();
    if (FLAGS_tcn_requires_image) {
      if (laser_m.image_feature_embeddings_size() &&
          laser_m.point_feature_embeddings_size()) {
        memcpy(feature.data(), laser_m.image_feature_embeddings().data(),
               sizeof(float) * feature_length / 2);
        memcpy(feature.data() + feature_length / 2,
               laser_m.point_feature_embeddings().data(),
               sizeof(float) * feature_length / 2);
        feature_seq->push_back(feature);
      }
    } else {
      if (laser_m.point_feature_embeddings_size()) {
        memcpy(feature.data(), laser_m.point_feature_embeddings().data(),
               sizeof(float) * feature_length);
        feature_seq->push_back(feature);
      }
      if (feature_seq->size() == sequence_length) {
        break;
      }
      if (laser_m.image_feature_embeddings_size()) {
        memcpy(feature.data(), laser_m.image_feature_embeddings().data(),
               sizeof(float) * feature_length);
        feature_seq->push_back(feature);
      }
    }
  }
}

void TrackClassifier::ClassifyTracksByModel(
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    TrackClassifierDebugProto* track_classifier_debug_proto,
    std::vector<TrackRef>* tracks) {
  std::vector<std::vector<Feature>> input_feature_batch;
  input_feature_batch.reserve(kMaxBatchSize);
  const int sequence_length = tcn_sequence_net_classifier_->GetSequenceLength();

  std::vector<Feature> feature_seq;
  feature_seq.reserve(sequence_length);
  std::vector<float> out_cls_prob;
  std::vector<int> track_index;
  track_index.reserve(kMaxBatchSize);
  for (int i = 0; i < tracks->size(); ++i) {
    auto& track = *(*tracks)[i];
    // NOTE(zheng): For the radar only track, we classify it to unknown movable.
    // Nowadays we only create radar only track whose velocity is greater than 5
    // m/s and filter out static radar measurements, so the radar only track is
    // movable track.
    // TODO(zheng,zheqi): Add Radar classifiation logic.
    if (track.track_state.measurement_source_type ==
        TrackMeasurementSourceType::TMST_RO) {
      track.track_state.type = MT_UNKNOWN;
      continue;
    }
    // NOTE(zheng): Nowadays, we only trigger tcn pipeline when the
    // lastest measurement is laser(camera3d) measurement. If the The track's
    // latest laser measurement timestamp is equal to track
    // classification_timestamp, it means the track doesn't associate any
    // laser(camera3d) measurement in current frame, so we can skip track
    // classification to save compution time.

    const double last_traggered_measurement_timestamp =
        tracker_util::GetLatestLaserOrCameraMeasurementTimestamp<TrackState>(
            track);
    if (last_traggered_measurement_timestamp -
            track.track_state.classification_timestamp <
        std::numeric_limits<double>::epsilon()) {
      SetDebugInfoFromPreviousResult(
          track,
          track_classifier_debug_proto->mutable_track_classifier_info(i));
      continue;
    }
    CollectSequenceFeature(track.measurement_history, &feature_seq);
    if (!feature_seq.empty()) {
      track_index.push_back(i);
      std::reverse(feature_seq.begin(), feature_seq.end());
      // padding
      while (feature_seq.size() < sequence_length) {
        feature_seq.push_back(feature_seq.back());
      }
      input_feature_batch.push_back(feature_seq);
      if (input_feature_batch.size() == kMaxBatchSize) {
        // run sequence net
        RunSequenceModel(input_feature_batch, track_index, &out_cls_prob,
                         track_classifier_debug_proto, tracks);
        input_feature_batch.clear();
        track_index.clear();
      }
    }
  }
  // For the last batch
  RunSequenceModel(input_feature_batch, track_index, &out_cls_prob,
                   track_classifier_debug_proto, tracks);

  // Check UNKNOWN_STATIC and CONE is warning triangle or not
  ClassifyWarningTriangle(camera_images, tracks);
  return;
}

bool TrackClassifier::IsInTcnScope(const Track<TrackState>& track,
                                   const MeasurementType& tcn_label,
                                   const float max_score) {
  constexpr float kVehicleThreshold = 0.95;
  constexpr float kGeneralThreshold = 0.98;

  if (tcn_label == MT_STATIC_OBJECT) {
    return false;
  }
  // Following cases will use tcn label replace voting label
  // case 1: voting label is UNKNOWN_MOVABLE
  if (track.track_state.type == MT_UNKNOWN) {
    if (max_score > kVehicleThreshold && tcn_label == MT_VEHICLE) {
      return true;
    } else if (max_score > kGeneralThreshold) {
      return true;
    }
  }
  // case 2: UNKNOWN_STATIC in the lane and model type is vehicle
  if (track.track_state.type == MT_STATIC_OBJECT) {
    if (max_score > kGeneralThreshold) {
      if (tcn_label == MT_VEGETATION || tcn_label == MT_BARRIER ||
          tcn_label == MT_CYCLIST || tcn_label == MT_VEHICLE) {
        return true;
      }
    }
  }
  // case3: observation_state is OS_PARTIALLY_OBSERVED
  if (!IsDBQConext() &&
      track.track_state.observation_state == OS_PARTIALLY_OBSERVED) {
    if (max_score > kGeneralThreshold) {
      if (tcn_label == MT_CYCLIST || tcn_label == MT_VEHICLE) {
        return true;
      }
    }
  }
  return false;
}

void TrackClassifier::ApplyTcnLabel(
    const int label, const float max_score,
    TrackClassifierDebugProto::TrackClassifierInfo* track_classifier_info,
    Track<TrackState>* track) {
  const auto& tcn_label =
      TcnTypeToMeasurementType(static_cast<tcn_model::TcnType>(label));
  track_classifier_info->set_model_cls(tcn_label);
  track_classifier_info->set_model_score(max_score);
  track->track_state.classifier_info.tcn_result.type = label;
  track->track_state.classifier_info.tcn_result.score = max_score;
  if (IsInTcnScope(*track, tcn_label, max_score)) {
    track->track_state.type = tcn_label;
    track_classifier_info->set_adopted_method(TrackClassifierDebugProto::MODEL);
    track->track_state.classifier_info.adopted_method =
        TrackState::ClassifierInfo::kTcnModel;
  }
  return;
}

void TrackClassifier::RunSequenceModel(
    const std::vector<std::vector<Feature>>& input_feature,
    const std::vector<int>& track_index, std::vector<float>* out_cls_ptr,
    TrackClassifierDebugProto* track_classifier_debug_proto,
    std::vector<TrackRef>* tracks) {
  if (input_feature.size() == 0) return;
  tcn_sequence_net_classifier_->Classify(input_feature, out_cls_ptr);
  int cur_pos = 0;
  const int num_classes = tcn_sequence_net_classifier_->GetNumClasses();
  for (int i = 0; i < track_index.size(); ++i) {
    const int index = track_index[i];
    int label = 0;
    float max_score = -1;
    for (int j = 0; j < num_classes; ++j) {
      if ((*out_cls_ptr)[cur_pos + j] > max_score) {
        max_score = (*out_cls_ptr)[cur_pos + j];
        label = j;
      }
    }
    TrackClassifierDebugProto::TrackClassifierInfo* track_classifier_info =
        track_classifier_debug_proto->mutable_track_classifier_info(index);
    ApplyTcnLabel(label, max_score, track_classifier_info,
                  (*tracks)[index].get());
    cur_pos += num_classes;
  }
  return;
}

void TrackClassifier::ClassifyTracks(
    const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
    TrackClassifierDebugProto* track_classifier_debug_proto,
    std::vector<TrackRef>* tracks) {
  SCOPED_QTRACE("TrackClassifier::ClassifyTracks");

  track_classifier_debug_proto->Clear();
  for (int i = 0; i < tracks->size(); ++i) {
    auto* tc_info = track_classifier_debug_proto->add_track_classifier_info();

    auto* bbox_proto = tc_info->mutable_bounding_box();
    if ((*tracks)[i]->track_state.refined_bounding_box) {
      const auto& bbox = (*tracks)[i]->track_state.refined_bounding_box;
      bbox_proto->set_x(bbox->center_x());
      bbox_proto->set_y(bbox->center_y());
      bbox_proto->set_heading(bbox->heading());
      bbox_proto->set_width(bbox->width());
      bbox_proto->set_length(bbox->length());
    } else {
      const auto contour_box =
          (*tracks)[i]->track_state.contour.MinAreaBoundingBox();
      bbox_proto->set_x(contour_box.center_x());
      bbox_proto->set_y(contour_box.center_y());
      bbox_proto->set_heading(contour_box.heading());
      bbox_proto->set_width(contour_box.width());
      bbox_proto->set_length(contour_box.length());
    }
  }
  ClassifyTracksByVoting(coordinate_converter, pose, latest_label_frame,
                         track_classifier_debug_proto, tracks);
  if (FLAGS_enable_tcn_model) {
    ClassifyTracksByModel(camera_images, track_classifier_debug_proto, tracks);
  }

  // Sync classification_timestamp.
  double timestamp_sum = 0;
  for (auto& track : *tracks) {
    const double last_traggered_measurement_timestamp =
        tracker_util::GetLatestLaserOrCameraMeasurementTimestamp<TrackState>(
            *track);
    track->track_state.classification_timestamp =
        last_traggered_measurement_timestamp;
    timestamp_sum += track->track_state.state_timestamp;
  }
  ParallelFor(0, tracks->size(), thread_pool_, [&](int i) {
    MaybeSaveTrackData(*(*tracks)[i],
                       track_classifier_debug_proto->track_classifier_info(i),
                       pose, latest_label_frame);
  });
  track_classifier_debug_proto->set_timestamp(
      tracks->size() > 0 ? timestamp_sum / tracks->size() : 0);
}

MeasurementType TrackClassifier::GetTypeForUnknownObjectMeasurement(
    const Track<TrackState>& track, const MeasurementProto& measurement,
    const VehiclePose& pose) {
  const auto s = track.track_state.estimator_3d.GetStateData();
  const double abs_vel = std::abs(s.GetVel());
  // In fen detection range or not
  const Vec2d centroid =
      tracker_util::ComputeCentroid(track.track_state.contour.points());
  const bool is_in_fen_range = tracker_util::IsInFenDetectionRange(
      {centroid.x(), centroid.y(), pose.z}, pose);
  constexpr double kInLaneThresholdForVoting = kLaneWidth / 2.0 + 0.05;
  // If in fen detection range, we set all unknown object as MT_UNKNOWN_STATIC
  // by default which can reduce shooters if unknown object satify the
  // following conditions, we think it is a movable object.

  const auto laser_measurement = measurement.laser_measurement();
  if (is_in_fen_range) {
    // 1. Speed of vehicle should > kStaticObjectMaxSpeed
    if (abs_vel <= kStaticObjectMaxSpeed) {
      return MT_STATIC_OBJECT;
    }
    // 2. Height > kUnknownMovableMinHeight
    QCHECK(laser_measurement.has_cluster_measurement());
    if (laser_measurement.cluster_measurement().height() <
        kUnknownMovableMinHeight) {
      return MT_STATIC_OBJECT;
    }
    // 3. Object in the lane
    if (!tracker_util::IsInLane(*semantic_map_manager_, centroid,
                                kInLaneThresholdForVoting)) {
      return MT_STATIC_OBJECT;
    }
    // 4. STATIC from LLNet
    if (measurement.type_source() == MTS_LL_NET) {
      return MT_STATIC_OBJECT;
    }
    return MT_UNKNOWN;
  }

  return (abs_vel <= kStaticObjectMaxSpeed) ? MT_STATIC_OBJECT : MT_UNKNOWN;
}

void TrackClassifier::SetDebugInfoFromPreviousResult(
    const Track<TrackState>& track,
    TrackClassifierDebugProto::TrackClassifierInfo* track_classifier_info) {
  // type=-1 means not set
  if (track.track_state.classifier_info.voting_type >= 0) {
    track_classifier_info->set_voting_cls(
        track.track_state.classifier_info.voting_type);
  }
  if (track.track_state.classifier_info.tcn_result.type >= 0) {
    track_classifier_info->set_model_cls(
        TcnTypeToMeasurementType(static_cast<tcn_model::TcnType>(
            track.track_state.classifier_info.tcn_result.type)));
    track_classifier_info->set_model_score(
        track.track_state.classifier_info.tcn_result.score);
  }
  if (track.track_state.classifier_info.adopted_method ==
      TrackState::ClassifierInfo::kVoting) {
    track_classifier_info->set_adopted_method(
        TrackClassifierDebugProto::VOTING);
  } else {
    track_classifier_info->set_adopted_method(TrackClassifierDebugProto::MODEL);
  }
}

void TrackClassifier::ClassifyTracksByVoting(
    const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
    std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame,
    TrackClassifierDebugProto* track_classifier_debug_proto,
    std::vector<TrackRef>* tracks) {
  QCHECK_NOTNULL(tracks);
  MaybeRenderLabel(latest_label_frame);
  // TODO(yu): Temporal aggregation on track history measurements in the next
  // change.
  ParallelFor(0, tracks->size(), thread_pool_, [&](int i) {
    auto& track = *(*tracks)[i];
    // NOTE(zheng): For the radar only track, we classify it to unknown movable.
    // Nowadays we only create radar only track whose velocity is greater than 5
    // m/s and filter out static radar measurements, so the radar only track is
    // movable track.track_state.
    // TODO(zheng,zheqi): Add Radar classifiation logic.
    if (track.track_state.measurement_source_type ==
        TrackMeasurementSourceType::TMST_RO) {
      track.track_state.type = MT_UNKNOWN;
      return;
    }

    // NOTE(zheng): There are 2 situations we don't run voting pipeline:
    // 1. Nowadays, we only trigger voting pipeline when the
    // lastest measurement is laser(camera3d) measurement. If the The track's
    // latest laser measurement timestamp is equal to track
    // classification_timestamp, it means the track doesn't associate any
    // laser(camera3d) measurement in current frame, so we can skip track
    // classification to save compution time.
    // 2. The first frame of the splited track, we should keep splitted track
    // original type until new measurements received.
    const double last_traggered_measurement_timestamp =
        tracker_util::GetLatestLaserOrCameraMeasurementTimestamp<TrackState>(
            track);
    if (last_traggered_measurement_timestamp -
                track.track_state.classification_timestamp <
            std::numeric_limits<double>::epsilon() ||
        (track.track_state.split_from_other_track &&
         track.measurement_history.size() == 1)) {
      SetDebugInfoFromPreviousResult(
          track,
          track_classifier_debug_proto->mutable_track_classifier_info(i));
      return;
    }

    const auto& m_history = track.measurement_history;
    std::vector<int> type_votes(MeasurementType_MAX + 1, 0);
    const int num_m = m_history.size();
    const double last_m_timestamp = m_history.back_time();
    constexpr double kDurationAsValidMeasurement = 0.5;  // s
    int last_laser_measurement_ind = -1;
    bool is_voted = false;
    for (int j = m_history.GetIndexWithTimeAtLeast(last_m_timestamp -
                                                   kDurationAsValidMeasurement);
         j < num_m; ++j) {
      const auto* m = m_history.value(j);
      if (m->has_camera_measurement() || m->has_camera3d_measurement()) {
        is_voted = true;
        constexpr int kCameraMeasurementWeight = 2;
        type_votes[static_cast<int>(m->type())] += kCameraMeasurementWeight;
      } else if (m->has_laser_measurement()) {
        is_voted = true;
        QCHECK(m->laser_measurement().has_cluster_measurement());
        constexpr int kLaserMeasurementWeight = 1;
        MeasurementType type = m->type();
        const auto& cluster_m = m->laser_measurement().cluster_measurement();
        // Classify ped/cyc/motorcyc as unknown static if too large.
        // NOTE(yu): For now, use type cyclist for
        // cyclist/motorcyclist/tricyclist.
        const int num_obstacles = cluster_m.has_num_obstacles_deprecated()
                                      ? cluster_m.num_obstacles_deprecated()
                                      : cluster_m.obstacle_info_size();
        if (type == MT_PEDESTRIAN && num_obstacles > kPedMaxObstacles) {
          type = MT_STATIC_OBJECT;
        } else if ((type == MT_CYCLIST || type == MT_MOTORCYCLIST) &&
                   num_obstacles > kCyclistMaxObstacles) {
          type = MT_STATIC_OBJECT;
        }

        if (type == MT_STATIC_OBJECT || type == MT_UNKNOWN) {
          // For unknowns,
          // - treat as static if on the curb (>-1 m)
          // - treat as moveable if has speed, static otherwise.
          // - treat as unknown_static as static whose type comes from llnet.
          // NOTE(zheng): We classify MT_STATIC_OBJECT object whose type_source
          // comes from ll net to static object.
          const bool has_moving_radar_measurement =
              HasMovingRadarMeasurement(track);

          if (cluster_m.near_curb()) {
            type = MT_STATIC_OBJECT;
          } else if (has_moving_radar_measurement) {
            type = MT_UNKNOWN;
          } else {
            // BANDAID(zheqi): Use some rules to reduce shooters and improve the
            // recall of unknown movable
            type = GetTypeForUnknownObjectMeasurement(track, *m, pose);
          }
        }

        // Weight more on road agents.
        int weight_factor = (type == MT_PEDESTRIAN || type == MT_CYCLIST ||
                             type == MT_MOTORCYCLIST || type == MT_VEHICLE)
                                ? 3
                                : 1;

        type_votes[static_cast<int>(type)] +=
            weight_factor * kLaserMeasurementWeight;
        last_laser_measurement_ind = j;
      }
    }
    // If no voting results, we use the type of previous frame
    // Otherwise use the max voted type
    if (!is_voted) return;
    const int max_votes_ind = static_cast<int>(
        std::distance(type_votes.begin(),
                      std::max_element(type_votes.begin(), type_votes.end())));
    track.track_state.type = static_cast<MeasurementType>(max_votes_ind);

    // TODO(yu): Extends offroad label to all objects and increase offroad track
    // life when pedestrian has a better performance.
    // An pedestrian is offroad if the minimum distance to curb is > 0 and not
    // in cross walk.
    if (track.track_state.type == MT_PEDESTRIAN &&
        last_laser_measurement_ind >= 0) {
      track.track_state.offroad =
          m_history.value(last_laser_measurement_ind)
                  ->laser_measurement()
                  .cluster_measurement()
                  .min_dist_to_curb() > 0.0 &&
          !tracker_util::IsInCrosswalk(*semantic_map_manager_, track,
                                       coordinate_converter);
    } else {
      track.track_state.offroad = false;
    }

    if (track.track_state.type == MT_VEHICLE) {
      track.track_state.in_parking_area = tracker_util::IsInParkingArea(
          *semantic_map_manager_, track, coordinate_converter);
    }
    // Sometimes the segmentation of fence is unstable, which may cause
    // some shooter issues, so if we judge an unknown movable object to fence
    // , we should set the type to unknown static.
    if (track.track_state.type == MT_UNKNOWN && IsFence(track)) {
      track.track_state.type = MT_BARRIER;
    }

    if (IsFlyingBird(track)) {
      track.track_state.type = MT_FLYING_BIRD;
    }
    auto* tc_info =
        track_classifier_debug_proto->mutable_track_classifier_info(i);
    tc_info->set_voting_cls(track.track_state.type);
    tc_info->set_adopted_method(TrackClassifierDebugProto::VOTING);
    track.track_state.classifier_info.adopted_method =
        TrackState::ClassifierInfo::kVoting;
    track.track_state.classifier_info.voting_type = track.track_state.type;
  });
}
}  // namespace qcraft::tracker
