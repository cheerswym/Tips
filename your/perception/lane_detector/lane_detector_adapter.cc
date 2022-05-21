#include "onboard/perception/lane_detector/lane_detector_adapter.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/lite/logging.h"
#include "onboard/nets/net_util.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft {

LaneDetectorAdapter::LaneDetectorAdapter(const NetParam& net_param)
    : lane_detector_(std::make_unique<LaneCameraPerception>(net_param)) {}

MultiCameraLanesProto LaneDetectorAdapter::GetLanes(
    const CameraParams& camera_param, const CameraImage& image) {
  return GetLanes(image.camera_id(), camera_param, image.ToMat(),
                  image.center_timestamp());
}

MultiCameraLanesProto LaneDetectorAdapter::GetLanes(
    CameraId camera_id, const CameraParams& camera_param, cv::Mat image,
    double camera_center_timestamp) {
  SCOPED_QTRACE_ARG1("LaneDetectorAdapter::GetLanes", "camera_id",
                     CameraId_Name(camera_id));
  MultiCameraLanesProto multi_camera_lanes;

  // We solve just one image now
  std::vector<std::vector<LaneLine>> multi_camera_lane_lines;
  if (!lane_detector_->DetectImages({image}, &multi_camera_lane_lines)) {
    QLOG(ERROR) << "Lane detector: failed to get outputs";
    return multi_camera_lanes;
  }
  // Logic to update message
  // Solve every image's res
  for (int id = 0; id < multi_camera_lane_lines.size(); ++id) {
    const auto& singe_camera_lane_lines = multi_camera_lane_lines[id];
    auto* single_camera_lanes = multi_camera_lanes.add_single_camera_lanes();
    single_camera_lanes->set_camera_id(camera_id);
    single_camera_lanes->set_camera_center_timestamp(camera_center_timestamp);
    // Solve every lane's res
    for (int i = 0; i < singe_camera_lane_lines.size(); ++i) {
      const auto& lane_line = singe_camera_lane_lines[i].curve_image_point_set;
      auto* single_lane = single_camera_lanes->add_single_lane();
      // Solve every point's res
      for (int j = 0; j < lane_line.size(); ++j) {
        const auto& point = lane_line[j];
        auto* lane_point = single_lane->add_lane_points();
        lane_point->set_x(point.x);
        lane_point->set_y(point.y);
      }
    }
  }

  return multi_camera_lanes;
}

}  // namespace qcraft
