#ifndef ONBOARD_PERCEPTION_TRACKER_VOTER_H_
#define ONBOARD_PERCEPTION_TRACKER_VOTER_H_

namespace qcraft::tracker {

// NOTE(youjiawei): We only use this function in the beginning of tracking a new
// object to deal with Fen heading flipping issues.
class HeadingVoter {
 public:
  static constexpr int kMaxHitNumForVoting = 5;
  static constexpr int kMaxCountVal = 3;

  HeadingVoter() : counting_num_(0), track_heading_hits_num_(0) {}
  void Enable();
  void Disnable();
  void Reset();

  bool ShouldUseVoting();

  bool ShouldUseMeasurementHeading(double track_heading, double fen_heading,
                                   double angle_range_thres);

 private:
  int counting_num_;
  int track_heading_hits_num_;
};
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_VOTER_H_
