#ifndef ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_ENGINE_H_
#define ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_ENGINE_H_

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "onboard/lite/lite_module.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/multi_camera_fusion/single_camera_tracker.h"
#include "onboard/perception/multi_camera_fusion/track.h"
#include "onboard/perception/tracker/association/association_util.h"
#include "onboard/perception/tracker/association/onboard_associator.h"
namespace qcraft::multi_camera_fusion {

// Sometimes we may copy some track from origin track vector,
// ao we need to save the track's raw index in the origin track
// vector, the pair's first vector saves the raw index list,
// the pair's second vector saves the copied tracks.
using CorrespondingTrackIndexList =
    std::pair<std::vector<int>, std::vector<TrackRef>>;

// The struct saves the fused track infos.
struct FusedTrack {
  // The new id we generated after we fused the tracks.
  int fused_id;
  // Ids that we fused.
  std::unordered_set<int32_t> associated_id_list;
  double latest_timestamp;

  // The output fused track.
  tracker::Track<MultiCameraTrackState> output_track;
};

class MultiCameraFusionEngine {
 public:
  explicit MultiCameraFusionEngine(
      LiteModule* lite_module, ThreadPool* thread_pool,
      const SemanticMapManager* semantic_map_manager,
      const std::set<CameraId>& to_processed_cameras,
      const RunParamsProtoV2& run_params);
  // Track single camera's object.
  void TrackSingleCameraObjects(
      const CameraId camera_id, const VehiclePose& pose, const double timestamp,
      std::shared_ptr<const MeasurementsProto> measurement_group);
  void TrackMultiCameraObjects(const std::vector<CameraId>& processed_cameras,
                               const CoordinateConverter& coordinate_converter,
                               const double timestamp, const VehiclePose& pose);

 private:
  void AssociateAndFusedOverlapedAreaTracks(
      const CameraId source_camera_id, const CameraId target_camera_id,
      std::unordered_map<CameraId, std::unordered_set<int>>*
          tracks_seen_by_other_camera,
      std::unordered_map<int32_t,
                         std::vector<tracker::Track<MultiCameraTrackState>>>*
          should_fused_tracks);
  // Compute which track is in the overlaped area.
  void GetOverlapedAreaTracks(
      const CameraId source_camera_id, const CameraId target_camera_id,
      CorrespondingTrackIndexList* source_overlaped_tracks,
      CorrespondingTrackIndexList* target_overlaped_tracks);

  // Fuse the overlaped area tracks.
  void MergeTheAssociatedTracks(
      const std::vector<TrackRef>& source_tracks,
      const std::vector<TrackRef>& target_tracks,
      const std::vector<int>& source_to_target_matching_idxs,
      std::unordered_map<int32_t,
                         std::vector<tracker::Track<MultiCameraTrackState>>>*
          should_fused_tracks);

  void AssociateTracksBetweenTwoCameras(
      const CameraId source_camera_id, const CameraId target_camera_id,
      const CorrespondingTrackIndexList& source_overlaped_tracks,
      const CorrespondingTrackIndexList& target_overlaped_tracks,
      const double timestamp, std::vector<int>* source_to_target_matching_idxs);

  // TODO(zheng): Implement epipolar constrian.
  bool IsSatisfingEpipolarConstrain(
      const Vec3d& source_far_point, const Vec3d& source_near_point,
      const BoundingBox2dProto& target_bbox,
      const CameraParametersProto& target_camera_params);

  uint32_t GenerateNewTrackId() { return id_counter_++; }

  // Fuse multi camera tracks after the images has beed processed one cycle.
  void FuseMultiCameraTracks(const std::vector<CameraId>& processed_cameras,
                             const double timestamp, const VehiclePose& pose,
                             MeasurementsProto* measurements_proto);

  void RemoveExpiredTracks(double timestamp);

  void PublishSingleCameraTrackerMeasurements();

  double ComputeImageXAxisProjectionBuffer(const double range,
                                           const double range_std_error,
                                           const double fx);
  // Return select index.
  int SelectTrackFromShouldFusedTracks(
      const std::vector<tracker::Track<MultiCameraTrackState>>&
          should_fused_tracks,
      const tracker::Track<MultiCameraTrackState>& last_fused_track);
  Polygon2d PredictContour(const double timestamp,
                           const tracker::Track<MultiCameraTrackState>& track);
  // TODO(zheng): Put this func into associator.
  bool TangentialDistanceGating(
      const tracker::Track<MultiCameraTrackState>& source_track,
      const tracker::Track<MultiCameraTrackState>& target_track,
      const AffineTransformation& smooth_to_source_camera_transform,
      const double timestamp);
  // For test.
  friend class MultiCameraFusionEngineFriend;

 private:
  LiteModule* lite_module_;
  ThreadPool* thread_pool_;

  std::map<CameraId, std::unique_ptr<SingleCameraTracker>>
      multi_camera_fusion_engine_;
  CameraParamsMap camera_params_;
  std::map<CameraId, AffineTransformation> vehicle_to_camera_per_camera_;
  std::map<CameraId, AffineTransformation> camera_to_vehicle_per_camera_;
  tracker::association::OnboardAssociator<tracker::Track<MultiCameraTrackState>,
                                          MeasurementProto>
      camera_associator_;
  // The key is fused track id, value is the fused track infos.
  std::unordered_map<int32, FusedTrack> fused_tracks_;
  // The key is the single camera track id, the value is the corresponding
  // fused track id.
  std::unordered_map<int32_t, int32_t> associated_to_fused_id_map_;

  static constexpr double kMaxAllowedNoMeasurementUpdateTimeForFusedTrack =
      0.3;  // s
  uint32_t id_counter_ = 0;
  double latest_tracked_timestamp_ = 0.0;

  const SemanticMapManager* semantic_map_manager_;

  // Localization transform between smooth and [local or global]
  CoordinateConverter coordinate_converter_;

  constexpr static double kRangeStdErrorRatio = 0.1;  // 10% percentage.
  constexpr static double kExtraRangeError = 5.0;     // m
};

}  // namespace qcraft::multi_camera_fusion

#endif  // ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_ENGINE_H_
