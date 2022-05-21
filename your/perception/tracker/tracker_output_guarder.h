#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_OUTPUT_GUARDER_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_OUTPUT_GUARDER_

#include <string>
#include <unordered_map>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/tracker/motion_static_classifier.h"
#include "onboard/perception/tracker/shooter_killer.h"
#include "onboard/perception/tracker/track.h"

namespace qcraft::tracker::tracker_output_guarder {

class TrackerOutputGuarder {
 public:
  TrackerOutputGuarder() = default;
  ~TrackerOutputGuarder() = default;

  TrackState::MovingState Guard(
      const VehiclePose& pose,
      const mapping::SemanticMapManager& semantic_map_manager,
      const CoordinateConverter& coordinate_converter,
      const Track<TrackState>& track,
      TrackerDebugProto::OutputGuarderDebugInfoProto* guarder_debug_proto)
      const;

 private:
  motion_static_classifier::MotionStaticClassifier motion_static_classifier_;
  shooter_killer::ShooterKiller shooter_killer_;
};

}  // namespace qcraft::tracker::tracker_output_guarder

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_OUTPUT_GUARDER_
