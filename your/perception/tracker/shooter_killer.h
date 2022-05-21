#ifndef ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_
#define ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_

#include <string>
#include <unordered_map>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/tracker/track.h"

namespace qcraft::tracker::shooter_killer {

class ShooterKiller {
 public:
  enum ShooterType {
    kIsShooterByHugeSpeed = 0,
    kIsShooterByMaxAccCheck = 1,
    kIsShooterByHugeUnknownCheck = 2,
    kIsShooterByBboxCenterAverageMovingSantityCheck = 3,
    kIsShooterByDetBboxCenterMovingConsistencyCheck = 4,
    kNoEvidenceAsShooter = 5,
    kNotInShootRangeOfShooterKiller = 6,
    kNoJudge = 7,
  };
  using ShooterDebugResult = std::pair<bool, ShooterType>;

  static std::string DebugToString(const ShooterType& debug) {
    // TODO(jingwei) Use vector instead.
    const static std::unordered_map<ShooterType, std::string> kDict{
        {kIsShooterByHugeSpeed, "kIsShooterByHugeSpeed"},
        {kIsShooterByMaxAccCheck, "kIsShooterByMaxAccCheck"},
        {kIsShooterByHugeUnknownCheck, "kIsShooterByHugeUnknownCheck"},
        {kIsShooterByBboxCenterAverageMovingSantityCheck,
         "kIsShooterByBboxCenterAverageMovingSantityCheck"},
        {kIsShooterByDetBboxCenterMovingConsistencyCheck,
         "kIsShooterByDetBboxCenterMovingConsistencyCheck"},
        {kNoEvidenceAsShooter, "kNoEvidenceAsShooter"},
        {kNotInShootRangeOfShooterKiller, "kNotInShootRangeOfShooterKiller"},
        {kNoJudge, "kNoJudge"},
    };
    const auto itr = kDict.find(debug);
    if (itr != kDict.end())
      return itr->second;
    else
      return "IllegalDebugState";
  }

  ShooterKiller() = default;
  ~ShooterKiller() = default;

  ShooterDebugResult IsShooter(const VehiclePose& pose,
                               const Track<TrackState>& track) const;

  friend class ShooterKillerFriend;

 private:
  bool IsShooterByMaxAccCheck(const Track<TrackState>& track) const;
  bool IsShooterByHugeUnknownCheck(const Track<TrackState>& track) const;
  bool IsShooterByBboxCenterAverageMovingSantityCheck(
      const Track<TrackState>& track) const;
  bool IsShooterByDetBboxCenterMovingConsistencyCheck(
      const Track<TrackState>& track) const;
  bool IsShooterByHugeSpeed(const Track<TrackState>& track) const;
};

}  // namespace qcraft::tracker::shooter_killer

#endif  // ONBOARD_PERCEPTION_TRACKER_SHOOTER_KILLER_
