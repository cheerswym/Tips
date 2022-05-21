#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_DATA_STORAGE_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_DATA_STORAGE_H_
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "offboard/labeling/proto/track_classifier_data.pb.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/tracker/track_classifier/data_storage.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::tracker {
struct ImagePatch {
  cv::Mat image;
  cv::Rect roi;
  std::unordered_map<int, Vec2i> project_points;
  double time_diff = -100.0;
};

class LaserDataStorage : public DataStorage {
 public:
  static std::unique_ptr<LaserDataStorage> Create(
      const std::string& data_db_prefix, const std::string& key_prefix);
  void SaveData(const SegmentedCluster& cluster,
                const std::vector<const LaserPoint*>& cluster_points,
                const ImagePatch& image_patch, const VehiclePose& pose,
                float real_ground_z);

 private:
  LaserDataStorage(const std::string& data_db_prefix,
                   const std::string& key_prefix);
};

}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LASER_DATA_STORAGE_H_
