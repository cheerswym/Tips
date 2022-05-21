#include "onboard/perception/tracker/voter.h"

#include "onboard/math/util.h"
#include "onboard/perception/tracker/tracker_constant.h"

namespace qcraft::tracker {

// We only use voting in the first kMaxHitNumForVoting frames including bbox.
bool HeadingVoter::ShouldUseVoting() {
  return counting_num_ < kMaxHitNumForVoting;
}

void HeadingVoter::Enable() { Reset(); }

void HeadingVoter::Reset() {
  counting_num_ = 1;
  track_heading_hits_num_ = 1;
}

void HeadingVoter::Disnable() { counting_num_ = kMaxHitNumForVoting; }

bool HeadingVoter::ShouldUseMeasurementHeading(double track_heading,
                                               double fen_heading,
                                               double angle_range_thres) {
  if (std::abs(NormalizeAngle(track_heading - fen_heading)) >
      angle_range_thres) {
    --track_heading_hits_num_;
  } else {
    ++track_heading_hits_num_;
  }
  ++counting_num_;
  track_heading_hits_num_ = std::min(kMaxCountVal, track_heading_hits_num_);

  if (track_heading_hits_num_ <= 0) {
    track_heading_hits_num_ = 1;
    return true;
  }
  return false;
}

}  // namespace qcraft::tracker