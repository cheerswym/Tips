#include "onboard/perception/tracker/track_classifier/track_classifier_utils.h"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "leveldb/db.h"
#include "leveldb/options.h"
#include "onboard/camera/utils/camera_image.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/perception/projection_util.h"
#include "onboard/utils/filesystem.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::tracker {
namespace {
constexpr double kMaxTimeDiff = 0.155;  // s
constexpr double kValidImagePosRatioThres = 0.4f;
absl::StatusOr<cv::Mat> ProcessOneCamera(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m,
    const CameraImageWithTransform& image_with_transform,
    int image_short_side_min, const int image_long_side_max,
    const float image_aspect_ratio_min, double* max_image_ratio) {
  const auto& image = std::get<CameraImage>(image_with_transform);
  const auto& camera_param = image.params();
  const auto smooth_to_camera_transform =
      (image_with_transform.first.pose().ToTransform() *
       camera_param.camera_to_vehicle_extrinsics().ToTransform())
          .Inverse();
  int start_x = image.width();
  int end_x = -1;
  int start_y = image.height();
  int end_y = -1;
  int valid_image_pos_number = 0;
  const int total_image_pos = cluster_m.obstacle_info_size() * 2;

  for (const auto& info : cluster_m.obstacle_info()) {
    for (const auto z : {info.min_z(), info.max_z()}) {
      const auto image_pos = projection_util::SmoothPointToImagePos(
          Vec3d(info.center().x(), info.center().y(), z),
          smooth_to_camera_transform, camera_param);
      if (image_pos) {
        start_x = std::min(start_x, image_pos->x());
        start_y = std::min(start_y, image_pos->y());
        end_x = std::max(end_x, image_pos->x());
        end_y = std::max(end_y, image_pos->y());
        ++valid_image_pos_number;
      }
    }
  }
  const double valid_image_pos_ratio =
      static_cast<double>(valid_image_pos_number) /
      static_cast<double>(total_image_pos);
  if (valid_image_pos_ratio < kValidImagePosRatioThres ||
      valid_image_pos_ratio < *max_image_ratio) {
    return absl::FailedPreconditionError("image_pos_ratio not valid");
  }
  // Check if start_x/y, end_x/y are valid, skip if not.
  if (!IsInImageRange(image, start_x, start_y) ||
      !IsInImageRange(image, end_x, end_y) || end_x <= start_x ||
      end_y <= start_y) {
    return absl::FailedPreconditionError(
        "(start_x, start_y) or (end_x, end_y ) not valid");
  }
  const float image_height = static_cast<float>(end_y - start_y);
  const float image_width = static_cast<float>(end_x - start_x);
  if (image_height / image_width < image_aspect_ratio_min ||
      image_width / image_height < image_aspect_ratio_min) {
    return absl::FailedPreconditionError("Invalid image ratio");
  }
  // Padding 10%
  const int kWidthPadding = (end_x - start_x) * 0.1;
  const int kHeightPadding = (end_y - start_y) * 0.1;
  start_x = std::max(1, start_x - kWidthPadding / 2);
  end_x = std::min(image.width() - 1, end_x + kWidthPadding / 2);
  start_y = std::max(1, start_y - kHeightPadding / 2);
  end_y = std::min(image.height() - 1, end_y + kHeightPadding / 2);
  if (std::min(end_x - start_x, end_y - start_y) < image_short_side_min ||
      std::max(end_x - start_x, end_y - start_y) > image_long_side_max) {
    return absl::FailedPreconditionError(
        absl::StrCat("Image size not valid short side: ",
                     std::min(end_x - start_x, end_y - start_y),
                     " vs min short", image_short_side_min));
  }
  *max_image_ratio = valid_image_pos_ratio;
  const cv::Rect rect(cv::Point(start_x, start_y), cv::Point(end_x, end_y));
  return image.ToMat()(rect);
}
}  // namespace

bool IsInImageRange(const CameraImage& image, int x, int y) {
  return x >= 0 && x < image.width() && y >= 0 && y < image.height();
}

absl::StatusOr<cv::Mat> GetImagePatchForClusterMeasurement(
    const LaserMeasurementProto::ClusterMeasurement& cluster_m,
    const std::map<CameraId, CameraImageWithTransform>& camera_images,
    const int image_short_side_min, const int image_long_side_max,
    const float image_aspect_ratio_min) {
  const double cluster_ts = cluster_m.timestamp();
  double max_image_ratio = 0;
  CameraId selected_id = CAM_UNKNOWN;
  cv::Mat image_patch;
  std::vector<CameraId> auto_cameras = GetAutoCameraIds();
  for (const auto& [camera_id, image_with_transform] : camera_images) {
    if (std::find(auto_cameras.begin(), auto_cameras.end(), camera_id) ==
        auto_cameras.end()) {
      continue;
    }
    const auto& image = std::get<CameraImage>(image_with_transform);
    const double sensor_time_diff =
        std::abs(cluster_ts - image.center_timestamp());
    if (sensor_time_diff > kMaxTimeDiff) {
      QLOG(WARNING) << absl::StrFormat(
          "[TrackClassifierUtils] Can't find corresponding camera image for "
          "cluster timestamp: %.3f, The one found is with center "
          "timestamp lidar time: %.3f and trigger timestamp: %.3f "
          "sensor_time_diff: %.3f",
          cluster_ts, image.center_timestamp(), image.trigger_timestamp(),
          sensor_time_diff);
      continue;
    }
    const auto result = ProcessOneCamera(
        cluster_m, image_with_transform, image_short_side_min,
        image_long_side_max, image_aspect_ratio_min, &max_image_ratio);
    if (result.ok()) {
      image_patch = *result;
      selected_id = camera_id;
    }
  }
  if (selected_id == CAM_UNKNOWN) {
    return absl::FailedPreconditionError("Can't get correspoding image");
  } else {
    cv::Mat img;
    cv::cvtColor(image_patch, img, cv::COLOR_RGB2BGR);
    return img;
  }
}

absl::StatusOr<leveldb::DB*> OpenLevelDB(const std::string& data_db_prefix,
                                         const std::string& db_name) {
  // options.error_if_exists = true;
  const auto& output_dir = filesystem::path(data_db_prefix).parent_path();
  if (!filesystem::is_directory(output_dir)) {
    if (!filesystem::create_directories(output_dir)) {
      return absl::InternalError(
          absl::StrCat("Create ", output_dir.string(), " failed"));
    }
  }
  const std::string training_data_db = data_db_prefix + db_name;
  // Remove old data
  if (filesystem::is_directory(training_data_db)) {
    constexpr int kMaxAttempt = 5;
    int attempt = 0;
    int status = 0;
    while (!status && attempt < kMaxAttempt) {
      try {
        status = filesystem::remove_all(training_data_db);
      } catch (...) {
        attempt += 1;
        QLOG(INFO) << "Retry remove " << training_data_db << " " << attempt
                   << "times";
        sleep(1);
      }
    }
    if (!status) {
      return absl::InternalError(
          absl::StrCat("Remove ", training_data_db, " failed"));
    }
  }
  leveldb::DB* dbptr = nullptr;
  leveldb::Options options;
  options.create_if_missing = true;
  leveldb::Status open_db_status =
      leveldb::DB::Open(options, training_data_db, &dbptr);
  if (!open_db_status.ok()) {
    return absl::InternalError(open_db_status.ToString());
  }
  return dbptr;
}

bool HasMovingRadarMeasurement(const Track<TrackState>& track) {
  constexpr double kMinTrueMovingSpeedSqr = Sqr(2.0);  // m/s^2
  const auto& m_history = track.measurement_history;
  const int num_m = m_history.size();
  if (num_m == 0) {
    return false;
  }

  // Find if the track has moving radar measurements.
  for (int j = m_history.GetIndexWithTimeAtLeast(
           track.track_state.last_timestamp - 0.5);
       j < num_m; ++j) {
    const auto* m = m_history.value(j);
    if (m->has_radar_measurement()) {
      const double radar_speed_sqr =
          Vec2dFromProto(m->radar_measurement().vel()).squaredNorm();
      if (radar_speed_sqr > kMinTrueMovingSpeedSqr) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace qcraft::tracker
