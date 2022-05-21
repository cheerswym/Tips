#ifndef ONBOARD_PERCEPTION_TRACKER_TRACKER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACKER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "boost/circular_buffer.hpp"
#include "offboard/labeling/label_frame_util.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/perception/segmentation/segmenter.h"
#include "onboard/perception/tracker/association/onboard_associator.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/perception/tracker/track_classifier/track_classifier.h"
#include "onboard/perception/tracker/track_life_manager.h"
#include "onboard/perception/tracker/track_measurement_source_type_manager.h"
#include "onboard/perception/tracker/track_merge_split_manager.h"
#include "onboard/perception/tracker/tracker_output_guarder.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::tracker {

using TMSTM =
    track_measurement_source_type_manager::TrackMeasurementSourceTypeManager;

class Tracker {
 public:
  explicit Tracker(LiteModule* lite_client,
                   const SemanticMapManager* semantic_map_manager,
                   ThreadPool* thread_pool, const ParamManager& param_manager)
      : lite_module_(lite_client),
        semantic_map_manager_(semantic_map_manager),
        thread_pool_(thread_pool),
        track_classifier_(semantic_map_manager_, param_manager, thread_pool),
        laser_associator_(thread_pool),
        radar_associator_(thread_pool),
        camera_associator_(thread_pool),
        track_merge_split_manager_(thread_pool) {
    RunParamsProtoV2 run_params;
    param_manager.GetRunParams(&run_params);
    track_measurement_source_type_manager_ = TMSTM(run_params);
    laser_associator_.Init(
        "onboard/perception/tracker/association/config/"
        "dbq_laser_associator_assemble_config.pb.txt",
        "DBQ_LIDAR");
    radar_associator_.Init(
        "onboard/perception/tracker/association/config/"
        "dbq_radar_associator_assemble_config.pb.txt",
        "DBQ_RADAR");
    camera_associator_.Init(
        "onboard/perception/tracker/association/config/"
        "dbq_camera_associator_assemble_config.pb.txt",
        "DBQ_CAMERA");
  }

  Tracker(const Tracker&) = delete;
  Tracker& operator=(const Tracker&) const = delete;

  void TrackObjects(
      const VehiclePose& pose, const CoordinateConverter& coordinate_converter,
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      double timestamp, MeasurementsProto measurements,
      std::shared_ptr<const labeling::LabelFrameProto> label_frame,
      std::shared_ptr<const SensorFovsProto> sensor_fovs_proto);

  void UpdateRunParams(const RunParamsProtoV2& run_params);
  ObjectsProto PublishObjects();

  void ResetDebugProto();

 private:
  bool FillTrajectory(const Track<TrackState>& track,
                      ObjectProto* object) const;

  bool UpdateAnchorPoint(const MeasurementProto& measurement,
                         Track<TrackState>* track) const;

  void AssociateMeasurementsAndUpdateTracks(
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      double timestamp, MeasurementsProto::GroupType group_type,
      const MeasurementsProto& measurements_proto,
      TrackerDebugProto* group_debug_proto);
  void AssociateMeasurementsAndUpdateTracksWithRollBack(
      const std::map<CameraId, CameraImageWithTransform>& camera_images,
      const CoordinateConverter& coordinate_converter, double timestamp,
      MeasurementsProto measurements_proto);
  std::vector<int> AssociateCameraMeasurements(
      const double timestamp,
      const std::vector<const MeasurementProto*>& camera_measurements);
  std::vector<int> AssociateRadarMeasurements(
      const double timestamp,
      const std::vector<const MeasurementProto*>& radar_measurements,
      TrackerDebugProto* tracker_debug_proto);
  void RemoveExpiredTracks(
      MeasurementsProto::GroupType group_type, double timestamp,
      std::shared_ptr<const SensorFovsProto> sensor_fovs_proto);

  uint32_t GenerateNewTrackId() { return id_counter_++; }

  // Create a new track from the input measurement, and append it to tracks_.
  void CreateNewTracksFromMeasurements(
      const std::vector<const MeasurementProto*>& measurements);

  void UpdateTracksFromMeasurementsWithRollback(
      const std::vector<std::vector<const MeasurementProto*>>&
          matched_measurements_per_track);

  // Update the given track from the input laser measurement.
  void UpdateTrackFromMeasurement(const MeasurementProto* measurement,
                                  Track<TrackState>* track);

  // Update the given track from the input laser measurement.
  void UpdateTrackFromLaserMeasurement(const MeasurementProto* measurement,
                                       Track<TrackState>* track);

  // Update the given track from the input camera measurement.
  void UpdateTrackFromCameraMeasurement(const MeasurementProto* measurement,
                                        Track<TrackState>* track);

  // Update the given track from the input radar measurement.
  void UpdateTrackFromRadarMeasurement(const MeasurementProto* measurement,
                                       Track<TrackState>* track);

  // Compute the position offset from the ref point the track is currently using
  // to the given new ref_point_type on the most recent valid measurement of the
  // track.
  std::optional<Vec2d> ComputeRefPointOffset(
      const TrackState::RefPoint& ref_point, const Track<TrackState>& track,
      const double& cur_laser_timestamp) const;

  // Cpmpute the distance diff between two ref points (one current and one
  // prediction from past) and decide whether to return zero ref point reset
  bool ShouldSetZeroRefPointResetByRefPointPositionErrorCheck(
      const Vec2d& cur_ref_point_pos, const TrackState& track_state,
      const double timestamp, const Vec2d& track_ref_point_pos) const;

  // Pick and compute the ref point for the given measurement.
  TrackState::RefPoint ChooseAndComputeRefPoint(
      const LaserMeasurementProto& measurement,
      const Track<TrackState>& track) const;

  // Compute ref point position given a certain type.
  std::optional<Vec2d> ComputeRefPointPosGivenType(
      const LaserMeasurementProto& laser_measurement,
      const TrackState::RefPoint::Type type) const;

  // Adjust car model yaw state when first use car model.
  void AdjustTrackAfterClassification();

  // Select corner ref point.
  std::optional<TrackState::RefPoint> ChooseAndComputeCornerRefPoint(
      const LaserMeasurementProto& laser_measurement) const;

  std::optional<Vec2d> ComputeRefPointPosGivenTypeForCorners(
      const LaserMeasurementProto& laser_measurement,
      const TrackState::RefPoint::Type& ref_type) const;

  // Select face center ref point.
  std::optional<Vec2d> ComputeRefPointPosGivenTypeForFaceCenters(
      const LaserMeasurementProto& laser_measurement,
      const TrackState::RefPoint::Type& ref_type) const;
  std::optional<TrackState::RefPoint> ChooseAndComputeFaceCenters(
      const LaserMeasurementProto& laser_measurement) const;

  void SyncMotionFilterInfoToDebugProto(const Track<TrackState>& track,
                                        TrackerDebugProto* group_debug_proto);
  void SyncSingleTrackInfoToDebugProto(const Track<TrackState>& track,
                                       TrackerDebugProto* group_debug_proto);
  void SyncTrackStateToDebugInfo(
      const Track<TrackState>& track,
      TrackerDebugProto_TrackStateDebugProtoInfo* debug_info);

  void AddTracksInfoToDebugProto(const MeasurementsProto_GroupType group_type,
                                 const int group_num, const int group_index,
                                 const double min_timestamp,
                                 const double max_timestamp,
                                 std::vector<TrackRef>* tracks,
                                 TrackerDebugProto* debug_proto);

  void AddDeletedTrackInfoToDebugProto(const Track<TrackState>& track);

  bool ShouldUsePointModelCP(const Track<TrackState>& track) const;

  // Judge if icp vel is converged, we update motion filter by icp only when
  // it's converged.
  bool IsIcpVelConverged(
      const Track<TrackState>& track,
      const LaserMeasurementProto& curr_laser_measurement) const;
  // Judge if the track is in the blind area.
  bool IsInBlindArea(const Track<TrackState>& track) const;

  // Get anchor point for computing ref point.
  Vec3d GetAnchorPosForRefPoint() const;
  std::vector<const MeasurementProto*> GetValidRadarMeasurements(
      const MeasurementsProto& measurements_proto);

  // Update contour, obstacle centers and bounding box.
  void UpdateTrackExtents(const LaserMeasurementProto& laser_measurement,
                          double timestamp, Track<TrackState>* track);
  void SaveCheckPoints(Track<TrackState>* track);

  bool IsRadarReflectionMeasurement(const MeasurementProto& measurement,
                                    const Track<TrackState>& track) const;

  bool IsRadarMeasurementVelocityValid(const MeasurementProto& measurement,
                                       const Track<TrackState>& track) const;

  void UpdateRadarOnlyTrackExtents(const MeasurementProto& measurement,
                                   Track<TrackState>* track);

  MotionFilterParamType GetMotionFilterParamType(
      const Track<TrackState>& track, const bool should_use_car_model);
  TrackState::MotionFilterType GetMotionFilterTypeFromParamType(
      MotionFilterParamType type);
  void RunOutputGuarder();
  bool ShouldCreateEstimatorFromCarModel(const Track<TrackState>& track) const;
  bool ShouldSetEstimatorToCarModelAfterClassification(
      const Track<TrackState>& track) const;

  bool IsFirstFrame() { return latest_tracked_time_ == 0.0; }

  void UpdateEstimatorExtent(const MeasurementProto& measurement,
                             Track<TrackState>* track);

  Box2d ComputeBBoxFromEstimatorExtent(const Track<TrackState>& track);
  void UpdateTrackState(const MeasurementProto& measurement,
                        Track<TrackState>* track);
  // BANDAID(zheng): For pbq mode, if the track switch from VR to LVR,
  // the object contour and pos we published would change a lot
  // because of the the unstable detection on M1 lidar border. So
  // when switch from VR to LVR, we publish vision predict result
  // until we have enough fen detections to keep the contour and pos smooth
  // in sequential.
  bool ShouldUseVisionPredcitedShape(
      const Track<TrackState>& track,
      const LaserMeasurementProto& laser_measurement) const;

  VehiclePose pose_;
  AffineTransformation pose_inv_;
  CoordinateConverter* coordinate_converter_ = nullptr;
  LiteModule* const lite_module_;
  std::vector<TrackRef> tracks_;
  uint32_t id_counter_ = 0;

  double prev_timestamp_ = 0.0;

  const SemanticMapManager* semantic_map_manager_;
  ThreadPool* const thread_pool_;

  TrackClassifier track_classifier_;

  std::unique_ptr<TrackerDebugProtoWithRollback> tracker_debug_proto_;

  HistoryBuffer<MeasurementsProto> measurement_history_groups_;
  // Save pose history and coordinate converter history for roll back.
  HistoryBuffer<VehiclePose> pose_history_;
  HistoryBuffer<CoordinateConverter> coordinate_converter_history_;
  // Record latest tracked measurement timestamp for roll back mechanism.
  double latest_tracked_time_ = 0.0;

  // Latest corresponding label frame.
  std::shared_ptr<const labeling::LabelFrameProto> latest_label_frame_;

  // The anchor pos for selecting ref point, in vehicle coord.
  Vec3d ref_point_anchor_pos_;

  association::OnboardAssociator<Track<TrackState>, MeasurementProto>
      laser_associator_;
  association::OnboardAssociator<Track<TrackState>, MeasurementProto>
      radar_associator_;
  association::OnboardAssociator<Track<TrackState>, MeasurementProto>
      camera_associator_;
  TrackMergeSplitManager track_merge_split_manager_;

  tracker_output_guarder::TrackerOutputGuarder output_guarder_;

  TMSTM track_measurement_source_type_manager_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACKER_H_
