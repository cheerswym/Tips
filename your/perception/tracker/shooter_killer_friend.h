#ifndef ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_FRIEND_
#define ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_FRIEND_

#include "onboard/perception/tracker/shooter_killer.h"

namespace qcraft::tracker::shooter_killer {

class ShooterKillerFriend {
 public:
  ShooterKillerFriend(ShooterKiller* ptr) : shooter_killer_ptr_(ptr) {}
  ~ShooterKillerFriend() = default;

  inline ShooterKiller::ShooterDebugResult IsShooter(
      const VehiclePose& pose, const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooter(pose, track);
  }
  inline bool IsShooterByMaxAccCheck(const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooterByMaxAccCheck(track);
  }
  inline bool IsShooterByHugeUnknownCheck(
      const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooterByHugeUnknownCheck(track);
  }
  inline bool IsShooterByBboxCenterAverageMovingSantityCheck(
      const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooterByBboxCenterAverageMovingSantityCheck(
        track);
  }
  inline bool IsShooterByDetBboxCenterMovingConsistencyCheck(
      const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooterByDetBboxCenterMovingConsistencyCheck(
        track);
  }
  inline bool IsShooterByHugeSpeed(const Track<TrackState>& track) const {
    return shooter_killer_ptr_->IsShooterByHugeSpeed(track);
  }

 private:
  ShooterKiller* shooter_killer_ptr_;
};

}  // namespace qcraft::tracker::shooter_killer

#endif  // ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_FRIEND_
