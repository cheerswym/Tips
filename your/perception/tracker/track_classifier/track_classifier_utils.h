#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_UTILS_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_UTILS_H_
#include <map>
#include <optional>
#include <string>

#include "leveldb/db.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/perception/tracker/track.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
namespace qcraft::tracker {
absl::StatusOr<cv::Mat> GetImagePatchForClusterMeasurement(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m,
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    int image_short_side_min = 10, int image_long_side_max = 1000,
    float image_aspect_ratio_min = 0);
bool IsInImageRange(const CameraImage& image, int x, int y);
absl::StatusOr<leveldb::DB*> OpenLevelDB(const std::string& data_db_prefix,
                                         const std::string& db_name);
bool HasMovingRadarMeasurement(const Track<TrackState>& track);
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_TRACK_CLASSIFIER_UTILS_H_
