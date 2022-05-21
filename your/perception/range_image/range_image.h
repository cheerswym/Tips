#ifndef ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_H_
#define ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/perception/projection_util.h"
#include "onboard/perception/semantic_segmentation_result.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/core.hpp"

namespace qcraft {

constexpr int kMaxNumPointsPerPixel = 6;

struct RangeImageRegion {
  uint16_t min_row = std::numeric_limits<uint16_t>::max();
  uint16_t max_row = std::numeric_limits<uint16_t>::min();
  uint16_t min_col = std::numeric_limits<uint16_t>::max();
  uint16_t max_col = std::numeric_limits<uint16_t>::min();
};

class RangeImage {
 public:
  RangeImage(const LidarParametersProto& lidar_param,
             const LidarFrame& lidar_frame,
             const SemanticSegmentationResults& semseg_results);
  RangeImage() = delete;

  static constexpr float kMaxRange = 200.f;

  static RangeImageProto RangeImageToProto(const RangeImage& range_image);

  const cv::Mat& range_image() const { return range_image_; }
  const cv::Mat& intensity_image() const { return intensity_image_; }
  const cv::Mat& semantic_image() const { return semantic_image_; }
  const cv::Mat& instance_image() const { return instance_image_; }

  LidarId lidar_id() const { return lidar_frame_.lidar_id(); }
  LidarModel lidar_type() const { return lidar_frame_.lidar_type(); }

  int width() const { return width_; }
  int height() const { return height_; }

  int min_row() const { return range_image_region_.min_row; }
  int max_row() const { return range_image_region_.max_row; }
  int min_col() const { return range_image_region_.min_col; }
  int max_col() const { return range_image_region_.max_col; }

  bool is_spin() const { return lidar_frame_.is_spin(); }
  bool is_point_cloud() const { return lidar_frame_.is_point_cloud(); }

  const VehiclePose& StartPose() const { return lidar_frame_.StartPose(); }
  const VehiclePose& MidPose() const { return lidar_frame_.MidPose(); }
  const VehiclePose& EndPose() const { return lidar_frame_.EndPose(); }

  std::vector<Vec3d> PointsAt(const cv::Point& point) const {
    std::vector<Vec3d> points;
    if (lidar_frame_.is_spin()) {
      if (const auto point_return = CalibratedReturnAt(point.x, point.y)) {
        points.emplace_back(point_return->x, point_return->y, point_return->z);
      }
    } else {
      const auto pointcloud_points = PointCloudPointsAt(point.y, point.x);
      for (int i = 0; i < pointcloud_points.size(); i++) {
        points.emplace_back(pointcloud_points[i].x, pointcloud_points[i].y,
                            pointcloud_points[i].z);
      }
    }

    return points;
  }

  std::optional<Vec3d> PointAt(const cv::Point& point) const {
    if (lidar_frame_.is_spin()) {
      if (const auto point_return = CalibratedReturnAt(point.y, point.x)) {
        return Vec3d(point_return->x, point_return->y, point_return->z);
      }
    } else {
      if (const auto p = PointCloudPointAt(point.y, point.x)) {
        return Vec3d(p->x, p->y, p->z);
      }
    }

    return std::nullopt;
  }

  std::optional<float> RangeAt(const cv::Point& point) const {
    if (lidar_frame_.is_spin()) {
      if (const auto point_return = CalibratedReturnAt(point.y, point.x)) {
        return point_return->range;
      }
    } else {
      if (const auto p = PointCloudPointAt(point.y, point.x)) {
        return p->range();
      }
    }

    return std::nullopt;
  }

  // NOTE(dong): Get image pos from scan/beam index.
  std::pair<uint16_t, uint16_t> ImagePosAt(int scan_index,
                                           int beam_index) const {
    if (lidar_frame_.is_spin()) {
      const auto index = scan_index * height_ + beam_index;
      DCHECK_LT(index, index_to_rc_.size());
      return index_to_rc_[index];
    } else {
      DCHECK_LT(scan_index, index_to_rc_.size());
      return index_to_rc_[scan_index];
    }
  }

 private:
  void InitWithSpin(const Spin& spin);

  void InitWithPointCloud(const PointCloud& point_cloud);

  void InitializeAllImages(int height, int width);

  void UpdateSemanticImageUsingSemsegResult(const Vec3d& point,
                                            const int semseg_result_index,
                                            const int row, const int col);

  std::optional<PointCloud::Point> PointCloudPointAt(const int row,
                                                     const int col) const {
    const auto points = PointCloudPointsAt(row, col);
    if (points.empty()) return std::nullopt;
    return *std::max_element(points.begin(), points.end(),
                             [](const auto& lhs, const auto& rhs) {
                               return lhs.intensity < rhs.intensity;
                             });
  }

  std::optional<CalibratedReturn> CalibratedReturnAt(int row, int col) const {
    const auto index = row * width_ + col;
    DCHECK_LT(index, spin_rc_to_index_.size());
    const auto return_index = spin_rc_to_index_[index];
    if (!return_index.is_valid) {
      return std::nullopt;
    }
    const auto& shot = lidar_frame_.spin()->ShotAt(return_index.scan_index,
                                                   return_index.beam_index);
    if (shot.num_returns == 0) return std::nullopt;
    return shot.calibrated_returns[return_index.return_index];
  }

  std::vector<PointCloud::Point> PointCloudPointsAt(const int row,
                                                    const int col) const {
    const auto index = row * width_ + col;
    DCHECK_LT(index, pointcloud_rc_to_index_.size());
    const auto indices = pointcloud_rc_to_index_[index];
    std::vector<PointCloud::Point> points;
    points.reserve(indices.size());
    for (const auto& index : indices) {
      points.push_back(lidar_frame_.point_cloud()->point(index));
    }
    return points;
  }

  // Note: This function post-process range image with a dense CRF architecture
  void PostProcessRangeImage();

  LidarParametersProto lidar_param_;

  LidarFrame lidar_frame_;
  SemanticSegmentationResults semseg_results_;

  cv::Mat range_image_;
  cv::Mat intensity_image_;
  cv::Mat semantic_image_;
  cv::Mat instance_image_;
  // Width of the range image corresponds to num of scans of the given spinning
  // lidar or azimuth bins of the given solid lidar.
  int width_ = 0;
  // Height of the range image corresponds to the beam size of the spinning
  // lidar or elevation bins of the given solid lidar.
  int height_ = 0;
  // Valid region
  RangeImageRegion range_image_region_;
  // the minimum and maximum column index of the range image to be covered with
  // semantic image results
  RangeImageRegion semantic_image_region_;
  // Map pixel row/col to scan/beam index.
  struct ReturnIndex {
    ReturnIndex() : is_valid(false) {}
    ReturnIndex(int scan_index, int beam_index, int return_index)
        : scan_index(scan_index),
          beam_index(beam_index),
          return_index(return_index),
          is_valid(true) {}
    int scan_index : 16;
    int beam_index : 13;
    int return_index : 2;
    bool is_valid : 1;
  };
  static_assert(sizeof(ReturnIndex) == 4);
  // Distinguish between spin and pointcloud with different data structure
  std::vector<ReturnIndex> spin_rc_to_index_;
  using IndicesPerPixel = std::array<int, kMaxNumPointsPerPixel>;
  std::vector<IndicesPerPixel> pointcloud_rc_to_index_;

  using RowColVector = std::vector<std::pair<uint16_t, uint16_t>>;
  RowColVector index_to_rc_;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_H_
