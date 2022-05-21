#include "onboard/perception/human_pipeline_tracker.h"

#include <iostream>
#include <unordered_set>

#include "gflags/gflags.h"
#include "onboard/global/trace.h"
#include "onboard/nets/net_util.h"
#include "onboard/perception/tracker/tracker_util.h"

DEFINE_bool(pcn_tracker_log, true, "return tracker debug info");

namespace qcraft::human_tracking {

namespace {
constexpr float kMinIOUThreshold = 0.3f;
constexpr double kMaxUnmatchTime = 0.25f;
constexpr int kMaxIDNumber = 500;
}  // namespace

std::string HumanTracker::GetTrackerInfo(const HumanTrack tracker) {
  if (!FLAGS_pcn_tracker_log) return "";
  std::string info = absl::StrFormat("track:id%d,his:", tracker.id);
  for (int i = 0; i < tracker.type_history.size(); i++)
    info += absl::StrFormat("%d", tracker.type_history[i]);
  return info;
}

std::vector<ImagePatchClassifier::ClassificationResult>
HumanTracker::GetHumanTrackingResult(
    const std::vector<FieryEyeNetClassifier::DetectionBox>& boxes,
    const double detection_timestamp,
    const std::vector<ImagePatchClassifier::ClassificationResult>&
        ped_cyc_box_types,
    std::vector<std::string>* tracker_infos) {
  SCOPED_QTRACE("HumanTracker::GetHumanTrackingResult");
  QCHECK_EQ(boxes.size(), ped_cyc_box_types.size());

  // IOU
  std::unordered_map<int, int> idx2id = {};

  Eigen::MatrixXd iou_matrix(boxes.size(), tracks_.size());
  const auto matches =
      MatchAndAssociate(boxes, detection_timestamp, &idx2id, &iou_matrix);
  const auto track_results =
      UpdateTracks(matches, boxes, detection_timestamp, ped_cyc_box_types,
                   idx2id, iou_matrix, tracker_infos);
  return track_results;
}

ImagePatchClassifier::ClassificationResult HumanTracker::GetTrackerResult(
    const boost::circular_buffer<ImagePatchClassifier::ClassificationResult>&
        type_history) {
  int ped_num = 0, cyc_num = 0, unknown_num = 0;
  for (int i = 0; i < type_history.size(); i++) {
    if (type_history[i] == ImagePatchClassifier::kPedestrian)
      ped_num++;
    else if (type_history[i] == ImagePatchClassifier::kCyclist)
      cyc_num++;
    else
      unknown_num++;
  }
  // once model output ped more than 3 frame,this box must be ped
  int human_num = ped_num + cyc_num;
  if (human_num >= unknown_num) {
    if (ped_num >= cyc_num)
      return ImagePatchClassifier::kPedestrian;
    else
      return ImagePatchClassifier::kCyclist;
  } else {
    return ImagePatchClassifier::kUnknown;
  }
}

std::vector<ImagePatchClassifier::ClassificationResult>
HumanTracker::UpdateTracks(
    const std::vector<int>& matches,
    const std::vector<FieryEyeNetClassifier::DetectionBox>& boxes,
    const double detection_timestamp,
    const std::vector<ImagePatchClassifier::ClassificationResult>&
        ped_cyc_box_types,
    const std::unordered_map<int, int>& idx2id,
    const Eigen::MatrixXd& iou_matrix,
    std::vector<std::string>* tracker_infos) {
  std::vector<ImagePatchClassifier::ClassificationResult> track_results = {};
  std::unordered_set<int> matches_trackers = {};
  for (int row = 0; row < matches.size(); row++) {
    // Matched track predictions and boxes.
    if (matches[row] > -1 &&
        iou_matrix(row, matches[row]) >= kMinIOUThreshold) {
      const auto obj_id = idx2id.at(matches[row]);
      tracks_[obj_id].box = boxes[row];
      tracks_[obj_id].last_timestamp = detection_timestamp;
      matches_trackers.insert(matches[row]);
      tracks_[obj_id].type_history.push_back(ped_cyc_box_types[row]);
      track_results.push_back(GetTrackerResult(tracks_[obj_id].type_history));
      (*tracker_infos)[row] = GetTrackerInfo(tracks_[obj_id]);
    } else {
      int obj_id = BuildNewTrack(boxes[row], ped_cyc_box_types[row],
                                 detection_timestamp);
      track_results.push_back(ped_cyc_box_types[row]);
      (*tracker_infos)[row] = GetTrackerInfo(tracks_[obj_id]);
    }
  }
  // Unmatched tracks.
  for (int col = 0; col < idx2id.size(); ++col) {
    if (matches_trackers.find(col) == matches_trackers.end()) {
      int obj_id = idx2id.at(col);
      if (detection_timestamp - tracks_[obj_id].last_timestamp <
          kMaxUnmatchTime) {
        tracks_[obj_id].box.box =
            PredictBox(tracks_[obj_id], detection_timestamp);
      } else {
        tracks_.erase(obj_id);
      }
    }
  }
  return track_results;
}

int HumanTracker::BuildNewTrack(
    const FieryEyeNetClassifier::DetectionBox& box,
    const ImagePatchClassifier::ClassificationResult& type,
    const double detection_timestamp) {
  HumanTrack track;
  track.box = box;
  track.type_history.push_back(type);
  track.last_timestamp = detection_timestamp;
  current_id_++;
  current_id_ = current_id_ % kMaxIDNumber;
  track.id = current_id_;
  tracks_[current_id_] = track;
  return current_id_;
}

// Match and association by IOU.
std::vector<int> HumanTracker::MatchAndAssociate(
    const std::vector<FieryEyeNetClassifier::DetectionBox>& detections,
    const double detection_timestamp, std::unordered_map<int, int>* idx2id,
    Eigen::MatrixXd* iou_matrix) {
  std::vector<Box2d> predictions = {};
  iou_matrix->setZero();
  int predictions_index = 0;
  idx2id->clear();
  for (const auto& track : tracks_) {
    const Box2d pre_box_2d = PredictBox(track.second, detection_timestamp);
    predictions.push_back(pre_box_2d);
    (*idx2id)[predictions_index] = track.first;
    predictions_index += 1;
  }
  for (int i = 0; i < detections.size(); ++i) {
    for (int j = 0; j < predictions.size(); ++j) {
      (*iou_matrix)(i, j) = net_util::ComputeIoU(Polygon2d(detections[i].box),
                                                 Polygon2d(predictions[j]));
    }
  }

  const auto matches_result =
      tracker::tracker_util::ComputeMatches(*iou_matrix);
  return matches_result;
}

Box2d HumanTracker::PredictBox(const HumanTrack& track,
                               const double timestamp) {
  const double time_interval = timestamp - track.last_timestamp;
  const auto bbox_box = track.box.box;
  const auto bbox_velocity = track.box.velocity;
  // Get prediction box position by velocity
  const float box_x_preciction =
      bbox_box.center_x() + bbox_velocity.x() * time_interval;
  const float box_y_preciction =
      bbox_box.center_y() + bbox_velocity.y() * time_interval;
  Box2d prediction_box =
      Box2d({box_x_preciction, box_y_preciction}, bbox_box.heading(),
            bbox_box.length(), bbox_box.width());
  return prediction_box;
}
}  // namespace qcraft::human_tracking
