#ifndef ONBOARD_PERCEPTION_HUMAN_PIPELINE_TRACKER_H_
#define ONBOARD_PERCEPTION_HUMAN_PIPELINE_TRACKER_H_

#include <algorithm>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <queue>
#include <regex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/nets/fiery_eye_net_classifier.h"
#include "onboard/nets/image_patch_classifier.h"

namespace qcraft::human_tracking {

struct HumanTrack {
  int id;
  double last_timestamp;
  boost::circular_buffer<ImagePatchClassifier::ClassificationResult>
      type_history{10};
  FieryEyeNetClassifier::DetectionBox box;
};

class HumanTracker {
 public:
  HumanTracker() = default;

  std::vector<ImagePatchClassifier::ClassificationResult>
  GetHumanTrackingResult(
      const std::vector<FieryEyeNetClassifier::DetectionBox>& boxes,
      const double detection_timestamp,
      const std::vector<ImagePatchClassifier::ClassificationResult>&
          ped_cyc_box_types,
      std::vector<std::string>* tracker_infos);

 private:
  Box2d GetNextBoxPrediction(const Box2d& unmatched_tracker_box,
                             const Box2d& unmatched_prediction_box);

  ImagePatchClassifier::ClassificationResult GetTrackerResult(
      const boost::circular_buffer<ImagePatchClassifier::ClassificationResult>&
          history_results);

  std::string GetTrackerInfo(const HumanTrack tracker);

  std::vector<ImagePatchClassifier::ClassificationResult> UpdateTracks(
      const std::vector<int>& matches,
      const std::vector<FieryEyeNetClassifier::DetectionBox>& boxes,
      const double detection_timestamp,
      const std::vector<ImagePatchClassifier::ClassificationResult>&
          ped_cyc_box_types,
      const std::unordered_map<int, int>& idx2id,
      const Eigen::MatrixXd& iou_matrix,
      std::vector<std::string>* tracker_infos);

  int BuildNewTrack(const FieryEyeNetClassifier::DetectionBox& box,
                    const ImagePatchClassifier::ClassificationResult& type,
                    const double detection_timestamp);

  std::vector<int> MatchAndAssociate(
      const std::vector<FieryEyeNetClassifier::DetectionBox>& detections,
      const double detection_timestamp, std::unordered_map<int, int>* idx2id,
      Eigen::MatrixXd* iou_matrix);

  Box2d PredictBox(const HumanTrack& track, const double timestamp);

  int current_id_ = 0;
  std::unordered_map<int, HumanTrack> tracks_;

  DISALLOW_COPY_AND_ASSIGN(HumanTracker);
};
};      // namespace qcraft::human_tracking
#endif  // ONBOARD_PERCEPTION_HUMAN_PIPELINE_TRACKER_H_
