#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_DATA_STORAGE_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_DATA_STORAGE_H_
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "offboard/labeling/proto/label_frame.pb.h"
#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "offboard/labeling/proto/track_classifier_label.pb.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/perception/tracker/track_classifier/data_storage.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {
class TrackDataStorage : public DataStorage {
 public:
  static std::unique_ptr<TrackDataStorage> Create(
      const std::string& data_db_prefix, const std::string& key_prefix,
      const SemanticMapManager* semantic_map_manager);
  void SaveData(
      const Track<TrackState>& track,
      const TrackClassifierDebugProto::TrackClassifierInfo&
          track_classifier_info,
      const VehiclePose& pose, bool use_fen_label,
      std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame);

 private:
  TrackDataStorage(const std::string& data_db_prefix,
                   const std::string& key_prefix,
                   const SemanticMapManager* semantic_map_manager);

  void SaveTrackDataByFen(
      const Track<TrackState>& track,
      const TrackClassifierDebugProto::TrackClassifierInfo&
          track_classifier_info,
      const VehiclePose& pose,
      std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame);
  void SaveTrackDataAll(const Track<TrackState>& track,
                        const TrackClassifierDebugProto::TrackClassifierInfo&
                            track_classifier_info,
                        const VehiclePose& pose);
  track_classifier_data::TrackData UpdateTrackData(
      const Track<TrackState>& track, const std::string& db_key,
      const TrackClassifierDebugProto::TrackClassifierInfo&
          track_classifier_info,
      const VehiclePose& pose, track_classifier_label::Category category);
  void SetTrackData(const Track<TrackState>& track,
                    const VehiclePoseProto& pose_proto,
                    const TrackClassifierDebugProto::TrackClassifierInfo&
                        track_classifier_info,
                    const MeasurementProto* measurement_proto,
                    track_classifier_label::Category category,
                    track_classifier_data::TrackData* track_data);
  const SemanticMapManager* const semantic_map_manager_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_DATA_STORAGE_H_
