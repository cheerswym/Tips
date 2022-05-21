#include "onboard/perception/tracker/track_classifier/laser_data_storage.h"

#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/lite/logging.h"
#include "onboard/perception/laser_point.h"
#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"

namespace qcraft::tracker {
std::unique_ptr<LaserDataStorage> LaserDataStorage::Create(
    const std::string& data_db_prefix, const std::string& key_prefix) {
  return std::unique_ptr<LaserDataStorage>(
      new LaserDataStorage(data_db_prefix, key_prefix));
}

LaserDataStorage::LaserDataStorage(const std::string& data_db_prefix,
                                   const std::string& key_prefix) {
  data_db_prefix_ = data_db_prefix;
  key_prefix_ = key_prefix;
  InitLevelDB(data_db_prefix_, "_laser_embedding_db");
}

void LaserDataStorage::SaveData(
    const SegmentedCluster& cluster,
    const std::vector<const LaserPoint*>& cluster_points,
    const ImagePatch& image_patch, const VehiclePose& pose,
    float real_ground_z) {
  track_classifier_data::LaserMeasurementRawData laser_m_raw_data;
  for (int i = 0; i < cluster_points.size(); ++i) {
    const auto& point = cluster_points[i];
    auto* raw_point = laser_m_raw_data.add_points();
    const float height = point->z - real_ground_z;
    // tcn input feature && pcn input feature
    raw_point->set_x(point->x);
    raw_point->set_y(point->y);
    raw_point->set_z(point->z);
    raw_point->set_intensity(point->normalized_intensity());
    raw_point->set_has_returns_behind(point->has_return_behind);
    raw_point->set_lidar_id(point->lidar_id);
    raw_point->set_lidar_type(static_cast<LidarType>(point->lidar_type));
    const auto& pos_coord_vehicle =
        pose.ToTransform().Inverse().TransformPoint(point->coord());
    // Point range in x-y plane.
    const float range_xy = Hypot(pos_coord_vehicle.x(), pos_coord_vehicle.y());
    raw_point->set_range_xy(range_xy);
    raw_point->set_height(height);
    if (const auto* res = FindOrNull(image_patch.project_points, i)) {
      raw_point->set_proj_x(res->x());
      raw_point->set_proj_y(res->y());
    }
  }
  laser_m_raw_data.set_cluster_id(cluster.id());
  laser_m_raw_data.set_timestamp(cluster.timestamp());
  if (!image_patch.image.empty()) {
    // Encode image and save
    std::vector<uchar> encoded_image;
    std::vector<int> compression_params;
    track_classifier_data::ImagePatch* image_patch_proto =
        laser_m_raw_data.mutable_image_patch();
    track_classifier_data::Roi* roi_proto = image_patch_proto->mutable_roi();
    roi_proto->set_x(image_patch.roi.x);
    roi_proto->set_y(image_patch.roi.y);
    roi_proto->set_width(image_patch.roi.width);
    roi_proto->set_height(image_patch.roi.height);
    image_patch_proto->set_time_diff(image_patch.time_diff);
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    // Compression level: 0-9, default is 1
    // A higher value means a smaller size and longer compression time
    compression_params.push_back(5);
    cv::imencode(".png", image_patch.image, encoded_image, compression_params);
    image_patch_proto->set_image_data(
        std::string(encoded_image.begin(), encoded_image.end()));
  }
  const std::string db_key = absl::StrFormat("%s/%d/%.3f", key_prefix_,
                                             cluster.id(), cluster.timestamp());

  VLOG(2) << absl::StrFormat("Writing with key %s and value %s", db_key,
                             laser_m_raw_data.DebugString());
  const auto s = data_db_->Put(leveldb::WriteOptions(), db_key,
                               laser_m_raw_data.SerializeAsString());
  if (!s.ok()) {
    QLOG(FATAL) << s.ToString();
  }
}

}  // namespace qcraft::tracker
