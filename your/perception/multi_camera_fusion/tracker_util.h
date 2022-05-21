#ifndef ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACKER_UTIL_H_
#define ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACKER_UTIL_H_
#include "onboard/perception/multi_camera_fusion/track.h"

namespace qcraft::multi_camera_fusion {

MeasurementProto TrackToMeasurementProto(
    const tracker::Track<MultiCameraTrackState>& track,
    bool sync_reid_feature = true);

}

#endif  // ONBOARD_PERCEPTION_MULTI_CAMERA_FUSION_TRACKER_UTIL_H_
