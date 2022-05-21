#ifndef ONBOARD_PERCEPTION_LANE_DETECTOR_LANE_DETECTOR_ADAPTER_H_
#define ONBOARD_PERCEPTION_LANE_DETECTOR_LANE_DETECTOR_ADAPTER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/camera/utils/camera_image.h"
#include "onboard/nets/lane_camera_perception.h"
#include "opencv2/core.hpp"

namespace qcraft {

class LaneDetectorAdapter {
 public:
  explicit LaneDetectorAdapter(const NetParam& net_param);

  MultiCameraLanesProto GetLanes(const CameraParams& camera_param,
                                 const CameraImage& image);

  MultiCameraLanesProto GetLanes(CameraId camera_id,
                                 const CameraParams& camera_param,
                                 cv::Mat image, double camera_center_timestamp);

 private:
  std::unique_ptr<LaneCameraPerception> lane_detector_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_LANE_DETECTOR_LANE_DETECTOR_ADAPTER_H_
