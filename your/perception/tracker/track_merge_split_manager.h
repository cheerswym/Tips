#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_MERGE_SPLIT_MANAGER_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_MERGE_SPLIT_MANAGER_H_

#include <string>
#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/perception/tracker/track.h"

namespace qcraft::tracker {

class TrackMergeSplitManager {
 public:
  explicit TrackMergeSplitManager(ThreadPool* thread_pool)
      : thread_pool_(thread_pool) {}

  void MergeAndSplitTracks(double timestamp, std::vector<TrackRef>* tracks);

 private:
  bool ShouldMergeTracks(double timestamp,
                         const Track<TrackState>& source_track_state,
                         const Polygon2d& source_predicted_contour,
                         const Track<TrackState>& target_track_state,
                         const Polygon2d& target_predicted_contour);

  void MergeTracks(double timestamp,
                   const std::vector<Polygon2d>& predicted_contours,
                   std::vector<TrackRef>* output_tracks);

  void SplitTracks(double timestamp,
                   const std::vector<Polygon2d>& predicted_contours,
                   std::vector<TrackRef>* output_tracks);
  void SyncTrackStateFromSourceToTarget(const TrackState& source_track_state,
                                        TrackState* target_track_state);
  void SyncEstimatorFromSourceToTargetAfterSplit(
      const TrackState& source_track_state, TrackState* target_track_state);
  void ResetMergeSplitState(std::vector<TrackRef>* tracks);

 private:
  ThreadPool* thread_pool_;
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_MERGE_SPLIT_MANAGER_H_
