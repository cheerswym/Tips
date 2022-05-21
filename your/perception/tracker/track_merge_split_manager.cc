#include "onboard/perception/tracker/track_merge_split_manager.h"

#include <algorithm>
#include <limits>
#include <map>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/tracker/tracker_util.h"

namespace qcraft::tracker {

bool TrackMergeSplitManager::ShouldMergeTracks(
    double timestamp, const Track<TrackState>& source_track,
    const Polygon2d& source_predicted_contour,
    const Track<TrackState>& target_track,
    const Polygon2d& target_predicted_contour) {
  // Check if two tracks should be merged, the condition used to judge is as
  // follows:
  // 1. The two tracks should be onroad tracks.
  // 2. Both tracks should be not initiated at the current frame.
  // 3. The two tracks should not have laser measurement at the same
  //    time of the moment of timestamp.
  // 4. The two tracks should be vehicle or unknowns.
  // 5. The two tracks' contour overlap larger than threshold.
  if (source_track.track_state.offroad || target_track.track_state.offroad ||
      (source_track.track_state.type != MT_VEHICLE &&
       source_track.track_state.type != MT_STATIC_OBJECT &&
       source_track.track_state.type != MT_UNKNOWN) ||
      (target_track.track_state.type != MT_VEHICLE &&
       target_track.track_state.type != MT_STATIC_OBJECT &&
       target_track.track_state.type != MT_UNKNOWN) ||
      source_track.measurement_history.size() == 1 ||
      target_track.measurement_history.size() == 1) {
    return false;
  }

  const double kSameFrameTimeDiffThreshold = 0.05;  // 50 ms
  if (std::abs(source_track.track_state.last_laser_timestamp -
               target_track.track_state.last_laser_timestamp) <
      kSameFrameTimeDiffThreshold) {
    return false;
  }

  const bool has_overlap =
      source_predicted_contour.HasOverlap(target_predicted_contour);
  if (!has_overlap) {
    return false;
  }

  Polygon2d overlap;
  source_predicted_contour.ComputeOverlap(target_predicted_contour, &overlap);
  const double soruce_area = source_predicted_contour.area();
  const double target_area = target_predicted_contour.area();
  const double overlap_ratio =
      overlap.area() / (std::min(soruce_area, target_area) +
                        std::numeric_limits<double>::epsilon());
  VLOG(2) << "Track merge overlap ratio: " << source_track.track_state.id << " "
          << target_track.track_state.id << " " << overlap_ratio;

  // Overlap threshold for merging tracks.
  const double kOverlapThresholdForMergeTracks = 0.5;
  if (overlap_ratio < kOverlapThresholdForMergeTracks) {
    return false;
  }

  return true;
}

void TrackMergeSplitManager::MergeTracks(
    double timestamp, const std::vector<Polygon2d>& predicted_contours,
    std::vector<TrackRef>* output_tracks) {
  auto& tracks = *QCHECK_NOTNULL(output_tracks);
  const int num_tracks = tracks.size();
  SCOPED_QTRACE_ARG1("TrackMergeSplitManager::MergeTracks", "num_tracks",
                     num_tracks);

  // Use union-find algorithm to find merged tracks groups.
  std::vector<int> merged_table(num_tracks);
  std::iota(std::begin(merged_table), std::end(merged_table), 0);
  for (int i = 0; i < num_tracks; ++i) {
    for (int j = i + 1; j < num_tracks; ++j) {
      if (ShouldMergeTracks(timestamp, *tracks[i], predicted_contours[i],
                            *tracks[j], predicted_contours[j])) {
        merged_table[j] = i;
      }
    }
  }
  // Find connected domains.
  std::map<int, std::vector<int>> track_id_to_merge_id_group;
  std::vector<int> group;
  for (int i = 0; i < num_tracks; ++i) {
    int p = i;
    group.clear();
    // Find a children to root connected group.
    while (p != merged_table[p]) {
      group.push_back(p);
      p = merged_table[p];
    }
    if (group.empty()) {
      continue;
    }
    // Merge groups which have a common root.
    auto& merge_id_group = track_id_to_merge_id_group[p];
    merge_id_group.insert(merge_id_group.end(), group.begin(), group.end());
    // Push root index.
    merge_id_group.push_back(p);
  }

  for (const auto& [_, merge_id_group] : track_id_to_merge_id_group) {
    // If the connected group only have root node, it needn't to merge.
    if (merge_id_group.size() < 2) {
      continue;
    }
    // Note(zheng): After track merge, we compute the kept track's id, state,
    // measurement history by using the following rules.
    // 1. State: Inherit the longest track's state.
    // 2. Track id: Inherit from newest track, because if we reserve the longest
    // track's id, after merge, the lonest track's pos may jump, which may cause
    // trouble for prediction.
    // 3. Measurement history: Only inherit form the track that has newest laser
    // measurement, becasue if we inherit the other track's measurement history
    // it may cause some shooters when roll back.

    // We select the track who has the newest laser measurement and the type is
    // unknow as the reserved track. If all the track is unknown type, we select
    // the newest track as the reserved track.
    int reserved_track_index = merge_id_group[0];
    bool has_known_type = false;
    for (const auto& idx : merge_id_group) {
      if (tracks[idx]->track_state.type != MT_STATIC_OBJECT &&
          tracks[idx]->track_state.type != MT_UNKNOWN) {
        has_known_type = true;
        break;
      }
    }
    for (const auto& idx : merge_id_group) {
      if (has_known_type &&
          (tracks[idx]->track_state.type == MT_STATIC_OBJECT ||
           tracks[idx]->track_state.type == MT_UNKNOWN)) {
        continue;
      }
      reserved_track_index =
          (tracks[reserved_track_index]->track_state.last_laser_timestamp <
           tracks[idx]->track_state.last_laser_timestamp)
              ? idx
              : reserved_track_index;
    }
    // When the track merge happens continuous multiple times,
    // the two track's exist time may be equal, we will fail to
    // select the longest track in this case, so we should select
    // a confirmed track as the default longest track index.
    int longest_track_index = merge_id_group[0];
    for (int index : merge_id_group) {
      if (tracks[index]->track_state.life_state == TrackLifeState::kConfirmed) {
        longest_track_index = index;
        break;
      }
    }
    // Select the longest track.
    double max_exist_time =
        tracks[merge_id_group[0]]->track_state.last_laser_timestamp -
        tracks[merge_id_group[0]]->track_state.first_timestamp;
    for (int index : merge_id_group) {
      const double exist_time =
          tracks[index]->track_state.last_laser_timestamp -
          tracks[index]->track_state.first_timestamp;
      if (exist_time > max_exist_time) {
        max_exist_time = exist_time;
        longest_track_index = index;
      }
    }

    // If thed the reserved track is not confirmed and its state uncertainty is
    // greater than the longest track, we sync velocity/yaw/acc/yawd state and
    // promote the reserved trac immediately.
    const auto& longest_track = *tracks[longest_track_index];
    auto& reserved_track = *tracks[reserved_track_index];
    if (reserved_track_index != longest_track_index &&
        longest_track.track_state.life_state == TrackLifeState::kConfirmed &&
        reserved_track.track_state.life_state != TrackLifeState::kConfirmed) {
      reserved_track.track_state.life_state = TrackLifeState::kConfirmed;
      reserved_track.track_state.moving_state =
          longest_track.track_state.moving_state;
      SyncTrackStateFromSourceToTarget(longest_track.track_state,
                                       &reserved_track.track_state);
    }
    for (int i = 0; i < merge_id_group.size(); ++i) {
      if (merge_id_group[i] == reserved_track_index) {
        continue;
      }
      VLOG(2) << "Merge tracks: "
              << tracks[reserved_track_index]->track_state.id << " "
              << tracks[merge_id_group[i]]->track_state.id;
      tracks[merge_id_group[i]]->track_state.merged_by_other_track = true;
    }
  }
}

void TrackMergeSplitManager::SplitTracks(
    double timestamp, const std::vector<Polygon2d>& predicted_contours,
    std::vector<TrackRef>* output_tracks) {
  SCOPED_QTRACE("TrackMergeSplitManager::SplitTracks");
  auto& tracks = *QCHECK_NOTNULL(output_tracks);

  // The track split mechanism mainly handle one track become two tracks issue,
  // we can use the track split mechanism to promote the other
  // new track immediately, and make the new track inherit the other track's
  // velocity/acc/yaw/yaw rate .

  // Split function: Judge if the source track should be splited and execute
  // split logic.
  const auto split_track = [&](const Track<TrackState>& source_track,
                               const Polygon2d& source_predicted_contour,
                               Track<TrackState>& target_track,
                               const Polygon2d& target_predicted_contour) {
    const int checkpoints_num = source_track.checkpoints.size();
    if (checkpoints_num < 2 ||
        source_track.track_state.life_state != TrackLifeState::kConfirmed ||
        target_track.track_state.life_state == TrackLifeState::kConfirmed) {
      return false;
    }
    // Compute overlap ratio, all predict to current timestamp.
    const double kOverlapThresholdForSplitTracks = 0.5;
    // NOTE(zheng): When the vehicle is over-segmented, an unknown object
    // with small area contour would appear besides of the vehicle. To solve
    // this kind of case, we don't immplement track split strategy wthen one
    // of the contour is too small.
    constexpr double kMaxContourRatio = 10;  // max / min
    const double predict_source_area = source_predicted_contour.area();
    const double predict_target_area = target_predicted_contour.area();
    const double contour_ratio =
        std::max(predict_source_area, predict_target_area) /
        std::min(predict_source_area, predict_target_area);
    if (contour_ratio > kMaxContourRatio) {
      return false;
    }

    // Use last track checkpoint to check if the two track should be split.
    const auto& last_source_track_state =
        source_track.checkpoints.value(checkpoints_num - 2);
    const auto predict_last_source_contour =
        tracker_util::PredictContour(timestamp, last_source_track_state);
    const bool has_overlap_with_curr_source_track =
        predict_last_source_contour.HasOverlap(source_predicted_contour);
    const bool has_overlap_with_curr_target_track =
        predict_last_source_contour.HasOverlap(target_predicted_contour);
    if (has_overlap_with_curr_source_track &&
        has_overlap_with_curr_target_track) {
      const double source_area = source_predicted_contour.area();
      const double target_area = target_predicted_contour.area();
      const double last_source_area = predict_last_source_contour.area();

      Polygon2d overlap_with_current_source_track;
      predict_last_source_contour.ComputeOverlap(
          source_predicted_contour, &overlap_with_current_source_track);
      const double overlap_ratio_with_current_source_track =
          overlap_with_current_source_track.area() /
          std::max(DBL_EPSILON, std::min(last_source_area, source_area));

      Polygon2d overlap_with_current_target_track;
      predict_last_source_contour.ComputeOverlap(
          target_predicted_contour, &overlap_with_current_target_track);
      const double overlap_ratio_with_current_target_track =
          overlap_with_current_target_track.area() /
          std::max(DBL_EPSILON, std::min(last_source_area, target_area));
      if (overlap_ratio_with_current_source_track >
              kOverlapThresholdForSplitTracks &&
          overlap_ratio_with_current_target_track >
              kOverlapThresholdForSplitTracks) {
        // Promote immediately, inherit source track's moving state and
        // type.
        target_track.track_state.life_state = TrackLifeState::kConfirmed;
        target_track.track_state.moving_state =
            source_track.track_state.moving_state;
        target_track.track_state.type = source_track.track_state.type;
        SyncEstimatorFromSourceToTargetAfterSplit(source_track.track_state,
                                                  &target_track.track_state);

        return true;
      }
    }
    return false;
  };

  // We only implement the track split mechanism when one track is confirmed and
  // the other one is not confirmed, because the track split mechanism mainly
  // handle one track become two tracks issue, by using track split mechanism we
  // can promote the other new track immediately, so we needn't to consider
  // other situations.
  for (int i = 0; i < tracks.size(); ++i) {
    auto& track0 = *tracks[i];
    for (int j = i + 1; j < tracks.size(); ++j) {
      auto& track1 = *tracks[j];
      if (!(track0.track_state.life_state == TrackLifeState::kConfirmed &&
            track1.measurement_history.size() == 1) &&
          !(track1.track_state.life_state == TrackLifeState::kConfirmed &&
            track0.measurement_history.size() == 1)) {
        continue;
      }

      // The track split mechanlism is only suitable for the situation that
      // the two tracks both have laser measurement in current frame.
      constexpr double kSameFrameTimeDiffThreshold = 0.05;  // s
      const double laser_measurement_time_diff =
          std::fabs(track0.track_state.last_laser_timestamp -
                    track1.track_state.last_laser_timestamp);
      if (laser_measurement_time_diff > kSameFrameTimeDiffThreshold) {
        continue;
      }

      if (track0.track_state.offroad || track1.track_state.offroad ||
          (track0.track_state.type != MT_VEHICLE &&
           track0.track_state.type != MT_STATIC_OBJECT &&
           track0.track_state.type != MT_UNKNOWN) ||
          (track1.track_state.type != MT_VEHICLE &&
           track1.track_state.type != MT_STATIC_OBJECT &&
           track1.track_state.type != MT_UNKNOWN)) {
        continue;
      }
      // Split tracks.
      const int src_index =
          track0.track_state.life_state == TrackLifeState::kConfirmed ? i : j;
      const int tgt_index =
          track0.track_state.life_state == TrackLifeState::kConfirmed ? j : i;
      if (split_track(*tracks[src_index], predicted_contours[src_index],
                      *tracks[tgt_index], predicted_contours[tgt_index])) {
        tracks[tgt_index]->track_state.split_from_other_track = true;
      }
    }
  }
}
void TrackMergeSplitManager::SyncTrackStateFromSourceToTarget(
    const TrackState& source_track_state, TrackState* target_track_state) {
  QCHECK(target_track_state != nullptr);
  // Sync velocity/acc/yaw/yaw rate to target track
  const tracker::StateData source_s =
      source_track_state.estimator_3d.GetStateData();
  const auto source_track_state_vel = source_s.GetVel();
  const auto source_acc = source_s.GetAcc();
  const auto source_yaw = source_s.GetYaw();
  const auto source_yaw_rate = source_s.GetYawD();
  // NOTE(jiawei): Any state reset will contaminate the state
  // distribution density which may lead to filter divergence.
  tracker::StateData target_s = target_track_state->estimator_3d.GetStateData();
  target_s.car().state().vel() = source_track_state_vel;
  target_s.car().state().yawd() = source_yaw_rate;
  target_s.car().state().yaw() = source_yaw;
  target_s.car().state().acc() = source_acc;
  // Also sync to point state.
  target_s.CarToPoint();
  // TODO(zheng, jiawei): Enlarge state convariance.
  target_track_state->estimator_3d.SetStateData(target_s);
  target_track_state->vel = target_s.GetVel();
  target_track_state->heading = target_s.GetYaw();
}

void TrackMergeSplitManager::SyncEstimatorFromSourceToTargetAfterSplit(
    const TrackState& source_track_state, TrackState* target_track_state) {
  QCHECK(target_track_state != nullptr);
  const Vec2d ori_target_state_pos =
      target_track_state->estimator_3d.GetStatePos();
  const auto ori_target_state_pos_cov =
      target_track_state->estimator_3d.GetStatePosCov();
  const double ori_target_prev_timestamp =
      target_track_state->estimator_3d.prev_timestamp();
  // Sync estimator_3d.
  target_track_state->estimator_3d = source_track_state.estimator_3d;
  target_track_state->motion_filter_param_type =
      source_track_state.motion_filter_param_type;
  target_track_state->motion_filter_type =
      source_track_state.motion_filter_type;
  // Use origin pos, pos_cov, prev_timestamp.
  // NOTE(jiawei): Any state reset will contaminate the state
  // distribution density which may lead to filter divergence.
  auto target_s = target_track_state->estimator_3d.GetStateData();
  target_s.car().state().x() = ori_target_state_pos.x();
  target_s.car().state().y() = ori_target_state_pos.y();
  target_s.SetPosCov(ori_target_state_pos_cov);
  // TODO(zheng, jiawei): Enlarge state convariance.
  target_track_state->estimator_3d.InitStateAndCov(target_s,
                                                   ori_target_prev_timestamp);
  // TODO(zheng): Remove track vel and track heading variable.
  target_track_state->vel = target_s.GetVel();
  target_track_state->heading = target_s.GetYaw();
}

void TrackMergeSplitManager::ResetMergeSplitState(
    std::vector<TrackRef>* tracks) {
  // Reset merge/split state.
  ParallelFor(0, tracks->size(), thread_pool_, [&](int i) {
    (*tracks)[i]->track_state.merged_by_other_track = false;
    (*tracks)[i]->track_state.split_from_other_track = false;
  });
}
void TrackMergeSplitManager::MergeAndSplitTracks(
    double timestamp, std::vector<TrackRef>* tracks) {
  QCHECK(tracks != nullptr);
  ResetMergeSplitState(tracks);
  // Compute the predicted contours for all tracks.
  const int num_tracks = tracks->size();
  std::vector<Polygon2d> predicted_contours(num_tracks);
  ParallelFor(0, num_tracks, thread_pool_, [&](int i) {
    predicted_contours[i] =
        tracker_util::PredictContour(timestamp, (*tracks)[i]->track_state);
  });

  MergeTracks(timestamp, predicted_contours, tracks);
  SplitTracks(timestamp, predicted_contours, tracks);
}

}  // namespace qcraft::tracker
