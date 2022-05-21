#ifndef ONBOARD_PERCEPTION_TRACKER_MOTION_STATIC_CLASSIFIER_
#define ONBOARD_PERCEPTION_TRACKER_MOTION_STATIC_CLASSIFIER_

#include <string>
#include <unordered_map>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/perception/tracker/track.h"

namespace qcraft::tracker::motion_static_classifier {

enum MovingState {
  MOVING = 0,
  STATIC = 1,
  UNKNOWN = 2,
};

class MotionStaticClassifier {
 public:
  enum DebugState {
    kIsStaticByNaiveTooSlowIsStaticPhilosophy = 0,
    kIsMovingByNaiveRadarMeasurementa = 1,
    kIsStaticByNaiveFenVelocity = 2,
    kIsStaticByStaticObjectZone = 3,
    kIsStaticByType = 4,
    kIsUnknownByTrackIsAtBegining = 5,
    kIsUnknownByDoNotHaveEnoughInformation = 6,
    kNoJudge = 7,
  };
  using MovingStateDebug = std::pair<MovingState, DebugState>;

  static std::string DebugToString(const DebugState& debug) {
    // TODO(jingwei) Use vector instead.
    const static std::unordered_map<DebugState, std::string> kDict{
        {kIsStaticByNaiveTooSlowIsStaticPhilosophy,
         "kIsStaticByNaiveTooSlowIsStaticPhilosophy"},
        {kIsMovingByNaiveRadarMeasurementa,
         "kIsMovingByNaiveRadarMeasurementa"},
        {kIsStaticByNaiveFenVelocity, "kIsStaticByNaiveFenVelocity"},
        {kIsStaticByStaticObjectZone, "kIsStaticByStaticObjectZone"},
        {kIsStaticByType, "kIsStaticByType"},
        {kIsUnknownByTrackIsAtBegining, "kIsUnknownByTrackIsAtBegining"},
        {kIsUnknownByDoNotHaveEnoughInformation,
         "kIsUnknownByDoNotHaveEnoughInformation"},
        {kNoJudge, "kNoJudge"},
    };
    const auto itr = kDict.find(debug);
    if (itr != kDict.end())
      return itr->second;
    else
      return "IllegalDebugState";
  }

  MotionStaticClassifier() = default;
  ~MotionStaticClassifier() = default;

  MovingStateDebug MovingOrStatic(
      const VehiclePose& pose,
      const mapping::SemanticMapManager& semantic_map_manager,
      const CoordinateConverter& coordinate_converter,
      const Track<TrackState>& track) const;

 private:
  bool IsStaticByNaiveTooSlowIsStaticPhilosophy(
      const Track<TrackState>& track) const;

  bool IsMovingByNaiveRadarMeasurement(const VehiclePose& pose,
                                       const Track<TrackState>& track) const;

  // Remove this function a.s.a.p.
  bool IsStaticByNaiveFenVelocity(const Track<TrackState>& track) const;

  bool IsStaticByStaticObjectZone(
      const Track<TrackState>& track,
      const mapping::SemanticMapManager& semantic_map_manager,
      const CoordinateConverter& coordinate_converter) const;

  bool IsStaticByType(const Track<TrackState>& track) const;
};

}  // namespace qcraft::tracker::motion_static_classifier

#endif  // ONBOARD_PERCEPTION_TRACKER_MOTION_STATIC_CLASSIFIER_
