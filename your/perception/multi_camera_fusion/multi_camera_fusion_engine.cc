#include "onboard/perception/multi_camera_fusion/multi_camera_fusion_engine.h"

#include <algorithm>
#include <limits>
#include <string>

#include "onboard/perception/multi_camera_fusion/tracker_util.h"
#include "onboard/perception/projection_util.h"

namespace qcraft::multi_camera_fusion {

MultiCameraFusionEngine::MultiCameraFusionEngine(
    LiteModule* lite_module, ThreadPool* thread_pool,
    const SemanticMapManager* semantic_map_manager,
    const std::set<CameraId>& to_processed_cameras,
    const RunParamsProtoV2& run_params)
    : lite_module_(lite_module),
      thread_pool_(thread_pool),
      camera_associator_(thread_pool),
      semantic_map_manager_(semantic_map_manager) {
  // Init associator for multi camera association.
  camera_associator_.Init(
      "onboard/perception/tracker/association/config/"
      "pbq_multi_camera_associator_assemble_config.pb.txt",
      "multi_camera_association");

  // Init the fusion engine.
  for (const auto id : to_processed_cameras) {
    multi_camera_fusion_engine_[id] =
        std::make_unique<SingleCameraTracker>(id, thread_pool_);
  }
  // Get the camera params.

  for (const auto& camera_params :
       run_params.vehicle_params().camera_params()) {
    const auto camera_id = camera_params.installation().camera_id();
    if (to_processed_cameras.find(camera_id) == to_processed_cameras.end()) {
      continue;
    }
    const LidarParametersProto* ref_lidar_params = nullptr;
    const auto all_lidar_params = run_params.vehicle_params().lidar_params();
    for (const auto& lidar_params : all_lidar_params) {
      if (lidar_params.installation().lidar_id() ==
          camera_params.installation().ref_lidar_id()) {
        ref_lidar_params = &lidar_params;
        break;
      }
    }

    camera_params_.emplace(camera_id,
                           CameraParams(camera_params, ref_lidar_params));
  }

  // Get vehicle to camera trans.
  for (const auto& [camera_id, camera_params] : camera_params_) {
    const auto vehicle_to_camera =
        camera_params.camera_to_vehicle_extrinsics().ToTransform().Inverse();
    vehicle_to_camera_per_camera_.emplace(camera_id, vehicle_to_camera);
    camera_to_vehicle_per_camera_.emplace(
        camera_id, camera_params.camera_to_vehicle_extrinsics().ToTransform());
  }

  // Set the init value of the id_counter.
  id_counter_ =
      kMaxTrackNumForOneCamera * static_cast<int>(CameraId::CAM_FUNCTION_MAX);
}

void MultiCameraFusionEngine::TrackSingleCameraObjects(
    const CameraId camera_id, const VehiclePose& pose, const double timestamp,
    std::shared_ptr<const MeasurementsProto> measurement_group) {
  SCOPED_QTRACE_ARG2("MultiCameraFusionEngine::TrackSingleCameraObjects",
                     "camera: ", CameraId_Name(camera_id),
                     " timestamp: ", std::to_string(timestamp));
  latest_tracked_timestamp_ = std::max(timestamp, latest_tracked_timestamp_);
  multi_camera_fusion_engine_[camera_id]->TrackObjects(timestamp, pose,
                                                       measurement_group);
}

void MultiCameraFusionEngine::TrackMultiCameraObjects(
    const std::vector<CameraId>& processed_cameras,
    const CoordinateConverter& coordinate_converter, const double timestamp,
    const VehiclePose& pose) {
  SCOPED_QTRACE("MultiCameraFusionEngine::TrackMultiCameraObjects");
  // Set the track life state to kLost if the camera is lost in this process
  // cycle.
  coordinate_converter_ = coordinate_converter;
  std::unordered_set<CameraId> tracked_cameras;
  for (const auto camera_id : processed_cameras) {
    tracked_cameras.insert(camera_id);
  }
  for (auto& [camera_id, tracker] : multi_camera_fusion_engine_) {
    if (tracked_cameras.find(camera_id) == tracked_cameras.end()) {
      tracker->ResetLifeStateToLost();
    }
  }

  // Publish single camera tracker measurements for debug.
  PublishSingleCameraTrackerMeasurements();
  std::unique_ptr<MeasurementsProto> measurements_proto =
      std::make_unique<MeasurementsProto>();
  FuseMultiCameraTracks(processed_cameras, timestamp, pose,
                        measurements_proto.get());

  RemoveExpiredTracks(latest_tracked_timestamp_);
  // Publish measurements proto.
  QLOG_IF_NOT_OK(WARNING, lite_module_->Publish(std::move(measurements_proto),
                                                "camera_measurements"));
}

inline double GetLaneHeading(const mapping::LaneInfo& lane_info,
                             const Vec2d& smooth_coord) {
  double heading = 0.0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i + 1 < lane_info.points_smooth.size(); ++i) {
    Segment2d segment(lane_info.points_smooth[i],
                      lane_info.points_smooth[i + 1]);
    const double curr_dist = segment.DistanceTo(smooth_coord);
    if (min_dist > curr_dist) {
      min_dist = curr_dist;
      heading = segment.heading();
    }
  }
  return heading;
}

void MultiCameraFusionEngine::FuseMultiCameraTracks(
    const std::vector<CameraId>& processed_cameras, const double timestamp,
    const VehiclePose& pose, MeasurementsProto* measurements_proto) {
  SCOPED_QTRACE("MultiCameraFusionEngine::FuseMultiCameraTracks");
  // If the track is seen by other camera, we save its id for fusion.
  std::unordered_map<CameraId, std::unordered_set<int>>
      tracks_seen_by_other_camera;
  std::unordered_map<int32_t,
                     std::vector<tracker::Track<MultiCameraTrackState>>>
      should_fused_tracks;
  // Fuse overlaped area tracks.
  for (int source_idx = 0; source_idx < processed_cameras.size();
       ++source_idx) {
    const int target_idx = (source_idx + 1) % processed_cameras.size();

    auto source_camera_id = processed_cameras[source_idx];
    auto target_camera_id = processed_cameras[target_idx];
    VLOG(1) << "Fuse cameras: " << CameraId_Name(source_camera_id) << " "
            << CameraId_Name(target_camera_id);

    const auto& source_tracks =
        multi_camera_fusion_engine_[source_camera_id]->GetConfirmedTracks();
    const auto& target_tracks =
        multi_camera_fusion_engine_[target_camera_id]->GetConfirmedTracks();
    if (source_tracks.size() == 0 || target_tracks.size() == 0) {
      continue;
    }

    AssociateAndFusedOverlapedAreaTracks(source_camera_id, target_camera_id,
                                         &tracks_seen_by_other_camera,
                                         &should_fused_tracks);
  }

  // Convert unoverlaped area track measurements.
  for (const auto cam_id : processed_cameras) {
    const auto& tracks =
        multi_camera_fusion_engine_[cam_id]->GetConfirmedTracks();
    for (int i = 0; i < tracks.size(); ++i) {
      if (tracks_seen_by_other_camera[cam_id].find(i) !=
          tracks_seen_by_other_camera[cam_id].end()) {
        continue;
      }
      // NOTE(zheng): If the track has fused before, we reserve the
      // fused track id to keep the output track id is consecutive.
      const auto track_id = tracks[i]->track_state.id;
      auto m = TrackToMeasurementProto(*tracks[i]);
      if (associated_to_fused_id_map_.find(track_id) !=
          associated_to_fused_id_map_.end()) {
        const auto fused_track_id = associated_to_fused_id_map_[track_id];
        const auto last_fused_track =
            fused_tracks_[fused_track_id].output_track;
        const auto& source_camera_pose =
            multi_camera_fusion_engine_[last_fused_track.track_state.camera_id]
                ->pose();

        // Smooth to source camera transform.
        const auto smooth_to_source_camera_transform =
            vehicle_to_camera_per_camera_[last_fused_track.track_state
                                              .camera_id] *
            source_camera_pose.ToTransform().Inverse();
        // Clean wrong association by using gating strategy.
        if (TangentialDistanceGating(last_fused_track, *tracks[i],
                                     smooth_to_source_camera_transform,
                                     timestamp)) {
          fused_tracks_[fused_track_id].associated_id_list.erase(
              tracks[i]->track_state.id);
          associated_to_fused_id_map_.erase(tracks[i]->track_state.id);
          *measurements_proto->add_measurements() = std::move(m);
        } else {
          should_fused_tracks[fused_track_id].emplace_back(*tracks[i]);
        }
      } else {
        *measurements_proto->add_measurements() = std::move(m);
      }
    }
  }
  // TODO(zheng): Add more smart fusion logic.
  for (auto& [fused_track_id, tracks] : should_fused_tracks) {
    const int select_id = SelectTrackFromShouldFusedTracks(
        tracks, fused_tracks_[fused_track_id].output_track);
    fused_tracks_[fused_track_id].output_track = tracks[select_id];
    auto m = TrackToMeasurementProto(tracks[select_id]);
    m.mutable_camera3d_measurement()->set_track_id(fused_track_id);
    fused_tracks_[fused_track_id].latest_timestamp =
        std::max(m.timestamp(), fused_tracks_[fused_track_id].latest_timestamp);
    *measurements_proto->add_measurements() = std::move(m);
  }
  // We unify the timestamp of all measurements to hack the measurement
  // group overlap area issue in tracker rollback strategy.
  for (auto& m : *measurements_proto->mutable_measurements()) {
    m.set_timestamp(timestamp);
    m.mutable_camera3d_measurement()->set_timestamp(timestamp);
    // Sync fused track timestamp.
    if (fused_tracks_.find(m.camera3d_measurement().track_id()) !=
        fused_tracks_.end()) {
      fused_tracks_[m.camera3d_measurement().track_id()].latest_timestamp =
          std::max(timestamp, fused_tracks_[m.camera3d_measurement().track_id()]
                                  .latest_timestamp);
    }
  }
  measurements_proto->set_min_timestamp(timestamp);
  measurements_proto->set_max_timestamp(timestamp);
  measurements_proto->set_group_type(MeasurementsProto::CAMERA);

  // NOTE: We fuse HDMap measurement to get better estimation.
  const auto level_id = coordinate_converter_.GetLevel();

  const auto* ego_nearest_intersection =
      semantic_map_manager_->GetNearestIntersectionInfoAtLevel(level_id,
                                                               pose.coord2d());
  if (nullptr == ego_nearest_intersection) return;
  const bool ego_in_intersection =
      ego_nearest_intersection->polygon_smooth.IsPointIn(pose.coord2d());
  if (ego_in_intersection) return;

  const auto ego_lane_ptrs = semantic_map_manager_->GetLanesInfoAtLevel(
      level_id, pose.coord2d(), /*radius*/ 2.0);
  if (ego_lane_ptrs.empty()) return;

  const mapping::LaneInfo* ego_lane_ptr = nullptr;
  for (const auto* lane : ego_lane_ptrs) {
    if (lane->direction == mapping::LaneProto::STRAIGHT) {
      ego_lane_ptr = lane;
      break;
    }
  }
  if (ego_lane_ptr == nullptr) return;

  const double ego_lane_yaw = GetLaneHeading(*ego_lane_ptr, pose.coord2d());

  for (int i = 0; i < measurements_proto->measurements_size(); ++i) {
    auto m = measurements_proto->mutable_measurements(i)
                 ->mutable_camera3d_measurement();
    const auto m_pos = Vec3dFromProto(m->pos());

    const auto* obj_nearest_intersection =
        semantic_map_manager_->GetNearestIntersectionInfoAtLevel(
            level_id, Vec2d(m_pos.x(), m_pos.y()));
    if (nullptr == obj_nearest_intersection) continue;
    const bool obj_in_intersection =
        obj_nearest_intersection->polygon_smooth.IsPointIn(
            Vec2d(m_pos.x(), m_pos.y()));
    if (obj_in_intersection) continue;

    const auto obj_lane_ptrs = semantic_map_manager_->GetLanesInfoAtLevel(
        level_id, Vec2d(m_pos.x(), m_pos.y()), /*radius*/ 2.0);
    if (obj_lane_ptrs.empty()) continue;

    const mapping::LaneInfo* obj_lane_ptr = nullptr;
    for (const auto* lane : obj_lane_ptrs) {
      if (lane->direction == mapping::LaneProto::STRAIGHT) {
        obj_lane_ptr = lane;
        break;
      }
    }
    if (obj_lane_ptr == nullptr) continue;
    const double obj_lane_yaw =
        GetLaneHeading(*obj_lane_ptr, Vec2d(m_pos.x(), m_pos.y()));

    constexpr double kAngle150 = d2r(150.0);
    double angle_diff = std::abs(obj_lane_yaw - ego_lane_yaw);
    if (angle_diff > M_PI) {
      angle_diff = 2 * M_PI - angle_diff;
    }
    if (angle_diff > kAngle150) {
      // Project speed to lane direction.
      const Vec2d obj_lane_dir_vec =
          Vec2d(cos(obj_lane_yaw), sin(obj_lane_yaw));
      const Vec2d proj_vel =
          obj_lane_dir_vec * Vec2dFromProto(m->vel()).dot(obj_lane_dir_vec);
      m->set_heading(obj_lane_yaw);
      Vec2dToProto(proj_vel, m->mutable_vel());
    }
  }
}

void MultiCameraFusionEngine::AssociateAndFusedOverlapedAreaTracks(
    const CameraId source_camera_id, const CameraId target_camera_id,
    std::unordered_map<CameraId, std::unordered_set<int>>*
        tracks_seen_by_other_camera,
    std::unordered_map<int32_t,
                       std::vector<tracker::Track<MultiCameraTrackState>>>*
        should_fused_tracks) {
  SCOPED_QTRACE(
      "MultiCameraFusionEngine::AssociateAndFusedOverlapedAreaTracks");
  const auto& source_tracks =
      multi_camera_fusion_engine_[source_camera_id]->GetConfirmedTracks();
  const auto& target_tracks =
      multi_camera_fusion_engine_[target_camera_id]->GetConfirmedTracks();
  std::vector<int> source_to_target_matching_idxs(source_tracks.size(), -1);

  CorrespondingTrackIndexList source_overlaped_tracks;
  CorrespondingTrackIndexList target_overlaped_tracks;
  // We only associate tracks which are in overlaped area.
  GetOverlapedAreaTracks(source_camera_id, target_camera_id,
                         &source_overlaped_tracks, &target_overlaped_tracks);

  const double target_camera_timestamp =
      multi_camera_fusion_engine_[target_camera_id]->latest_tracked_timestamp();
  AssociateTracksBetweenTwoCameras(
      source_camera_id, target_camera_id, source_overlaped_tracks,
      target_overlaped_tracks, target_camera_timestamp,
      &source_to_target_matching_idxs);

  for (int i = 0; i < source_to_target_matching_idxs.size(); ++i) {
    if (source_to_target_matching_idxs[i] >= 0) {
      (*tracks_seen_by_other_camera)[source_camera_id].insert(i);
      const int target_track_index = source_to_target_matching_idxs[i];
      (*tracks_seen_by_other_camera)[target_camera_id].insert(
          target_track_index);
    }
  }
  // Merge the associated tracks to measurments.
  MergeTheAssociatedTracks(source_tracks, target_tracks,
                           source_to_target_matching_idxs, should_fused_tracks);
}

void MultiCameraFusionEngine::AssociateTracksBetweenTwoCameras(
    const CameraId source_camera_id, const CameraId target_camera_id,
    const CorrespondingTrackIndexList& source_overlaped_tracks,
    const CorrespondingTrackIndexList& target_overlaped_tracks,
    const double timestamp, std::vector<int>* source_to_target_matching_idxs) {
  SCOPED_QTRACE("MultiCameraFusionEngine::AssociateTracksBetweenTwoCameras");

  const auto& source_tracks =
      multi_camera_fusion_engine_[source_camera_id]->GetConfirmedTracks();
  const auto& target_tracks =
      multi_camera_fusion_engine_[target_camera_id]->GetConfirmedTracks();

  const auto& source_camera_pose =
      multi_camera_fusion_engine_[source_camera_id]->pose();

  // Smooth to source camera transform.
  const auto smooth_to_source_camera_transform =
      vehicle_to_camera_per_camera_[source_camera_id] *
      source_camera_pose.ToTransform().Inverse();

  std::vector<const MeasurementProto*> target_measurements;
  for (const auto& track_ref : target_overlaped_tracks.second) {
    const MeasurementProto* m = track_ref->measurement_history.back_value();
    target_measurements.push_back(m);
  }
  // TODO(zheng): Add debug proto.
  TrackerDebugProto group_debug_proto;
  const auto& tracks = source_overlaped_tracks.second;
  // TODO(zheng, jingwei): Use track state checkpoint in pbq association.
  std::vector<int> camera_m_matches = camera_associator_.Association1v1(
      timestamp, target_measurements, tracks, nullptr, nullptr,
      group_debug_proto.mutable_association_debug_info());
  for (int i = 0; i < camera_m_matches.size(); ++i) {
    const int track_ind = camera_m_matches[i];
    if (track_ind >= 0) {
      const int source_track_index = source_overlaped_tracks.first[track_ind];
      const int target_track_index = target_overlaped_tracks.first[i];
      const auto& source_track = source_tracks[source_track_index];
      const auto& target_track = target_tracks[target_track_index];
      const bool should_prune = TangentialDistanceGating(
          *source_track, *target_track, smooth_to_source_camera_transform,
          latest_tracked_timestamp_);

      if (should_prune) {
        continue;
      } else {
        (*source_to_target_matching_idxs)[source_track_index] =
            target_track_index;
      }
    }
  }
}

// TODO(zheng): Use a manager to merge tracks.
void MultiCameraFusionEngine::MergeTheAssociatedTracks(
    const std::vector<TrackRef>& source_tracks,
    const std::vector<TrackRef>& target_tracks,
    const std::vector<int>& source_to_target_matching_idxs,
    std::unordered_map<int32_t,
                       std::vector<tracker::Track<MultiCameraTrackState>>>*
        should_fused_tracks) {
  SCOPED_QTRACE("MultiCameraFusionEngine::MergeTheAssociatedTracks");
  for (int i = 0; i < source_tracks.size(); ++i) {
    const auto& source_track = *source_tracks[i];
    const int target_idx = source_to_target_matching_idxs[i];
    if (target_idx < 0) {
      continue;
    }
    QCHECK_GE(target_idx, 0);
    const auto& target_track = *target_tracks[target_idx];
    const int source_track_id = source_track.track_state.id;
    const int target_track_id = target_track.track_state.id;
    int fused_track_id = -1;
    // Save the id relationships.
    if (associated_to_fused_id_map_.find(source_track_id) !=
        associated_to_fused_id_map_.end()) {
      fused_track_id = associated_to_fused_id_map_[source_track_id];
      QCHECK(fused_tracks_.find(fused_track_id) != fused_tracks_.end());

      if (associated_to_fused_id_map_.find(target_track_id) ==
          associated_to_fused_id_map_.end()) {
        associated_to_fused_id_map_.insert({target_track_id, fused_track_id});
        // Record the fusion info.
        auto& fused_track_info = fused_tracks_[fused_track_id];
        fused_track_info.associated_id_list.insert(target_track_id);
        fused_track_info.latest_timestamp =
            std::max(fused_track_info.latest_timestamp,
                     target_track.track_state.state_timestamp);
      }
    } else if (associated_to_fused_id_map_.find(target_track_id) !=
               associated_to_fused_id_map_.end()) {
      fused_track_id = associated_to_fused_id_map_[target_track_id];
      QCHECK(fused_tracks_.find(fused_track_id) != fused_tracks_.end());
      // Record the fusion info.
      auto& fused_track_info = fused_tracks_[fused_track_id];
      fused_track_info.associated_id_list.insert(source_track_id);
      fused_track_info.latest_timestamp =
          std::max(fused_track_info.latest_timestamp,
                   source_track.track_state.state_timestamp);
      associated_to_fused_id_map_.insert({source_track_id, fused_track_id});
    } else {
      // Select the longest track id as the fused id.
      const double source_track_life_time =
          source_track.track_state.state_timestamp -
          source_track.track_state.first_timestamp;
      const double target_track_life_time =
          target_track.track_state.state_timestamp -
          target_track.track_state.first_timestamp;
      fused_track_id = source_track_life_time > target_track_life_time
                           ? source_track.track_state.id
                           : target_track.track_state.id;

      // Record the fusion info.
      associated_to_fused_id_map_.insert({source_track_id, fused_track_id});
      associated_to_fused_id_map_.insert({target_track_id, fused_track_id});
      FusedTrack new_fused_track;
      new_fused_track.fused_id = fused_track_id;
      new_fused_track.associated_id_list.insert(source_track_id);
      new_fused_track.associated_id_list.insert(target_track_id);
      new_fused_track.latest_timestamp =
          std::max(source_track.track_state.state_timestamp,
                   target_track.track_state.state_timestamp);
      new_fused_track.output_track =
          source_track_life_time > target_track_life_time ? source_track
                                                          : target_track;
      fused_tracks_.insert({fused_track_id, new_fused_track});
    }
    QCHECK(fused_track_id >= 0);

    // Save the fused tracks to measurement.
    (*should_fused_tracks)[fused_track_id].push_back(source_track);
    (*should_fused_tracks)[fused_track_id].push_back(target_track);
  }
}

void MultiCameraFusionEngine::GetOverlapedAreaTracks(
    const CameraId source_camera_id, const CameraId target_camera_id,
    CorrespondingTrackIndexList* source_overlaped_tracks,
    CorrespondingTrackIndexList* target_overlaped_tracks) {
  SCOPED_QTRACE("MultiCameraFusionEngine::GetOverlapedAreaTracks");
  const auto& source_tracks =
      multi_camera_fusion_engine_[source_camera_id]->GetConfirmedTracks();
  const auto& target_tracks =
      multi_camera_fusion_engine_[target_camera_id]->GetConfirmedTracks();

  const auto& source_camera_params = camera_params_[source_camera_id];
  const auto& target_camera_params = camera_params_[target_camera_id];

  const auto& source_camera_pose =
      multi_camera_fusion_engine_[source_camera_id]->pose();
  const auto& target_camera_pose =
      multi_camera_fusion_engine_[target_camera_id]->pose();

  const auto source_smooth_to_camera_trans =
      (source_camera_pose.ToTransform() *
       source_camera_params.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();
  const auto target_smooth_to_camera_trans =
      (target_camera_pose.ToTransform() *
       target_camera_params.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();

  const double source_camera_fx = source_camera_params.camera_matrix().fx();
  const double target_camera_fx = target_camera_params.camera_matrix().fx();

  for (int i = 0; i < source_tracks.size(); ++i) {
    const tracker::Track<MultiCameraTrackState>& track = *source_tracks[i];
    // Add pos uncertainty when we project the pos.
    const auto pos_3d = track.track_state.pos;
    const Vec3d target_camera_pos_in_smooth =
        target_camera_pose.ToTransform().TransformPoint(
            camera_to_vehicle_per_camera_[target_camera_id].GetTranslation());
    const double half_length =
        track.track_state.estimator_3d.GetExtentStateData().state().length() /
        2.0;
    double range = (pos_3d - target_camera_pos_in_smooth).norm();

    // To process the truncated measurement, we consider the lenght buffer.
    range =
        std::max(std::numeric_limits<double>::epsilon(), range - half_length);
    const double range_std_error =
        range * kRangeStdErrorRatio + kExtraRangeError;
    const double image_x_axis_buffer = ComputeImageXAxisProjectionBuffer(
        range, range_std_error, target_camera_fx);
    VLOG(1) << absl::StrFormat(
        "Image x axis projection buffer: range: %.2f, fx: %.2f, "
        "buffer: %.2f",
        range, target_camera_fx, image_x_axis_buffer);
    const auto target_projection_pos = projection_util::SmoothPointToImagePos(
        pos_3d, target_smooth_to_camera_trans, target_camera_params, true);
    // Check if the pos is in overlap area.
    if (!target_projection_pos.has_value() ||
        target_projection_pos->x() < -image_x_axis_buffer ||
        target_projection_pos->x() >
            target_camera_params.width() + image_x_axis_buffer ||
        target_projection_pos->y() < 0 ||
        target_projection_pos->y() > target_camera_params.height()) {
      continue;
    }
    source_overlaped_tracks->first.push_back(i);
    source_overlaped_tracks->second.push_back(source_tracks[i]);
  }

  for (int i = 0; i < target_tracks.size(); ++i) {
    const tracker::Track<MultiCameraTrackState>& track = *target_tracks[i];
    // Add pos uncertainty when we project the pos.
    const auto pos_3d = track.track_state.pos;
    const Vec3d source_camera_pos_in_smooth =
        source_camera_pose.ToTransform().TransformPoint(
            camera_to_vehicle_per_camera_[source_camera_id].GetTranslation());
    const double half_length =
        track.track_state.estimator_3d.GetExtentStateData().state().length() /
        2.0;
    double range = (pos_3d - source_camera_pos_in_smooth).norm();
    // To process the truncated measurement, we consider the lenght buffer.
    range =
        std::max(std::numeric_limits<double>::epsilon(), range - half_length);
    const double range_std_error =
        range * kRangeStdErrorRatio + kExtraRangeError;
    const double image_x_axis_buffer = ComputeImageXAxisProjectionBuffer(
        range, range_std_error, source_camera_fx);
    VLOG(1) << absl::StrFormat(
        "Image x axis projection buffer: range: %.2f, fx: %.2f, "
        "buffer: %.2f",
        range, target_camera_fx, image_x_axis_buffer);

    const auto source_projection_pos = projection_util::SmoothPointToImagePos(
        pos_3d, source_smooth_to_camera_trans, source_camera_params, true);

    // Check if the pos is in overlap area.
    if (!source_projection_pos.has_value() ||
        source_projection_pos->x() < -image_x_axis_buffer ||
        source_projection_pos->x() >
            source_camera_params.width() + image_x_axis_buffer ||
        source_projection_pos->y() < 0 ||
        source_projection_pos->y() > source_camera_params.height()) {
      continue;
    }

    target_overlaped_tracks->first.push_back(i);
    target_overlaped_tracks->second.push_back(target_tracks[i]);
  }
}

void MultiCameraFusionEngine::RemoveExpiredTracks(double timestamp) {
  SCOPED_QTRACE("MultiCameraFusionEngine::RemoveExpiredTracks");
  // In case some of the camera is sudden broken, we also should
  // clear single camera expired tracks here.
  for (auto& [_, cam_tracker] : multi_camera_fusion_engine_) {
    cam_tracker->RemoveExpiredTracks(timestamp);
  }

  // Remove expired fused tracks.
  auto iter = fused_tracks_.begin();
  while (iter != fused_tracks_.end()) {
    if (timestamp - iter->second.latest_timestamp >
        kMaxAllowedNoMeasurementUpdateTimeForFusedTrack) {
      for (const auto& id : iter->second.associated_id_list) {
        auto single_camera_id_iter = associated_to_fused_id_map_.find(id);
        QCHECK(single_camera_id_iter != associated_to_fused_id_map_.end());
        associated_to_fused_id_map_.erase(single_camera_id_iter);
      }
      iter = fused_tracks_.erase(iter);
    } else {
      iter++;
    }
  }
}

void MultiCameraFusionEngine::PublishSingleCameraTrackerMeasurements() {
  SCOPED_QTRACE(
      "MultiCameraFusionEngine::PublishSingleCameraTrackerMeasurements");
  std::unique_ptr<MeasurementsProto> measurements_proto =
      std::make_unique<MeasurementsProto>();
  for (const auto& [camera_id, cam_tracker] : multi_camera_fusion_engine_) {
    const auto& tracks = cam_tracker->GetConfirmedTracks();
    for (const auto& track : tracks) {
      auto m = TrackToMeasurementProto(*track, false);
      *measurements_proto->add_measurements() = std::move(m);
    }
  }
  QLOG_IF_NOT_OK(WARNING,
                 lite_module_->Publish(std::move(measurements_proto),
                                       "single_camera_tracker_measurements"));
}

double MultiCameraFusionEngine::ComputeImageXAxisProjectionBuffer(
    const double range, const double range_std_error, const double fx) {
  return range_std_error * fx /
         (range + std::numeric_limits<double>::epsilon());
}

int MultiCameraFusionEngine::SelectTrackFromShouldFusedTracks(
    const std::vector<tracker::Track<MultiCameraTrackState>>&
        should_fused_tracks,
    const tracker::Track<MultiCameraTrackState>& last_fused_track) {
  QCHECK(!should_fused_tracks.empty());
  const auto track_pos = last_fused_track.track_state.pos.head<2>();
  const auto level_id = coordinate_converter_.GetLevel();
  const auto* lane_ptr = semantic_map_manager_->GetNearestLaneInfoAtLevel(
      level_id, Vec2d(track_pos.x(), track_pos.y()));

  int selected_id = 0;
  int max_patch_size = std::numeric_limits<int>::lowest();
  double min_angle_diff = std::numeric_limits<double>::max();
  // NOTE(zheng): If he nearest lane is straight, we select the measurement
  // whose heading is close to the nearest lane's heading, else we select the
  // measurement whose image patch is bigger.
  if (lane_ptr && mapping::LaneProto::STRAIGHT == lane_ptr->direction) {
    const double lane_heading =
        semantic_map_manager_->GetPointHeadingWithLaneAtLevel(
            level_id, Vec2d(track_pos.x(), track_pos.y()));
    for (int i = 0; i < should_fused_tracks.size(); ++i) {
      const auto m = TrackToMeasurementProto(should_fused_tracks[i]);
      const double m_heading = m.camera3d_measurement().heading();
      double angle_diff = std::abs(NormalizeAngle(m_heading - lane_heading));
      // Ingore 180 degree flip issue.
      if (angle_diff > M_PI_2) {
        angle_diff = M_PI - angle_diff;
      }
      VLOG(1) << "Track id: " << m.camera3d_measurement().track_id()
              << ", angle diff: " << angle_diff;
      if (angle_diff < min_angle_diff) {
        selected_id = i;
        min_angle_diff = angle_diff;
      }
    }
  } else {
    // Add patch size selection.
    for (int i = 0; i < should_fused_tracks.size(); ++i) {
      const auto m = TrackToMeasurementProto(should_fused_tracks[i]);
      const auto& patch = m.camera3d_measurement().bbox_2d();
      const int patch_size = patch.width() * patch.height();
      if (patch_size > max_patch_size) {
        max_patch_size = patch_size;
        selected_id = i;
      }
    }
  }
  return selected_id;
}

// TODO(zheng): Put the PredictContour func in a util file.
Polygon2d MultiCameraFusionEngine::PredictContour(
    const double timestamp,
    const tracker::Track<MultiCameraTrackState>& track) {
  const auto s = tracker::tracker_util::SafePredictPos(track, timestamp);
  if (!s.has_value()) return track.track_state.contour;
  const Vec2d pos_shift =
      s->GetStatePos() - track.track_state.estimator_3d.GetStatePos();
  auto contour_points = track.track_state.contour.points();
  std::for_each(contour_points.begin(), contour_points.end(),
                [&](Vec2d& ele) { ele += pos_shift; });
  return Polygon2d(std::move(contour_points));
}
bool MultiCameraFusionEngine::TangentialDistanceGating(
    const tracker::Track<MultiCameraTrackState>& source_track,
    const tracker::Track<MultiCameraTrackState>& target_track,
    const AffineTransformation& smooth_to_source_camera_transform,
    const double timestamp) {
  // NOTE(zheng): The mono3d pos in tangential direction is accurate, so if
  // the distance in tangential direction is greater than threshold, and the
  // two bbox iop is small than threshold, we
  // should not associate the two tracks.
  constexpr double kTangentialDistanceThreshold = 1.0;
  constexpr double kMinIopThreshold = 0.2;
  const auto source_pos =
      source_track.track_state.estimator_3d.ComputePrediction(timestamp);
  const auto source_track_pos_in_source_camera_coord =
      smooth_to_source_camera_transform.TransformPoint(Vec3d(
          source_pos.x(), source_pos.y(), source_track.track_state.pos.z()));
  const auto target_pos =
      target_track.track_state.estimator_3d.ComputePrediction(timestamp);
  const auto target_track_pos_in_source_camera_coord =
      smooth_to_source_camera_transform.TransformPoint(Vec3d(
          target_pos.x(), target_pos.y(), target_track.track_state.pos.z()));
  // Compute distance in tangential direction.
  const Vec2d longitudinal_direction =
      source_track_pos_in_source_camera_coord.head<2>();
  const Vec2d tangential_direction =
      longitudinal_direction.Rotate(M_PI_2) /
      (longitudinal_direction.norm() + std::numeric_limits<double>::epsilon());
  const double distance_in_tangential_direction =
      std::abs((target_track_pos_in_source_camera_coord.head<2>() -
                source_track_pos_in_source_camera_coord.head<2>())
                   .dot(tangential_direction));
  // Compute two track contour iop.
  const double iop =
      tracker::association_util::IoP(PredictContour(timestamp, source_track),
                                     PredictContour(timestamp, target_track));

  // Compute distance in nearest lane tangential direction.
  const auto level_id = coordinate_converter_.GetLevel();
  const auto* lane_ptr = semantic_map_manager_->GetNearestLaneInfoAtLevel(
      level_id, Vec2d(source_pos.x(), source_pos.y()));
  double distance_in_lane_tangential_direction = 0.0;
  if (lane_ptr && mapping::LaneProto::STRAIGHT == lane_ptr->direction) {
    const double lane_heading =
        semantic_map_manager_->GetPointHeadingWithLaneAtLevel(
            level_id, Vec2d(source_pos.x(), source_pos.y()));

    const Vec2d lane_tangential_direction =
        Vec2d::FastUnitFromAngle(lane_heading).Rotate(M_PI_2);
    distance_in_lane_tangential_direction =
        std::abs((source_pos.GetStatePos() - target_pos.GetStatePos())
                     .dot(lane_tangential_direction));
  }

  VLOG(1) << "distance_in_tangential_direction: " << source_track.track_state.id
          << " " << target_track.track_state.id << " "
          << distance_in_tangential_direction << " " << iop << " "
          << distance_in_lane_tangential_direction;
  // TODO(zheng): Add iou gating in config.
  return (distance_in_tangential_direction > kTangentialDistanceThreshold ||
          distance_in_lane_tangential_direction >
              kTangentialDistanceThreshold ||
          iop < std::numeric_limits<double>::epsilon()) &&
         iop < kMinIopThreshold;
}

}  // namespace qcraft::multi_camera_fusion
