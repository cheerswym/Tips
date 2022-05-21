#include "onboard/perception/range_image/range_image.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/camera/utils/camera_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/math/util.h"
#include "onboard/perception/range_image/range_image_util.h"
#include "onboard/proto/lidar.pb.h"
#include "opencv2/core/types.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"

DEFINE_bool(enable_crf, false, "Enable range image optimization with DenseCRF");

namespace qcraft {

namespace {

constexpr double kResolutionAzimuth = 0.2;      // degree
constexpr double kResolutionElevation = 0.2;    // degree
constexpr double kMaxAzimuthOfM1 = 90.0;        // degree
constexpr double kMinAzimuthOfM1 = -90.0;       // degree
constexpr double kMaxElevationOfM1 = 18.0;      // degree
constexpr double kMinElevationOfM1 = -18.0;     // degree
constexpr double kCameraHFovBuffer = d2r(10.);  // radian
constexpr double kResolutionAzimuthInv = 1.0 / kResolutionAzimuth;
constexpr double kResolutionElevationInv = 1.0 / kResolutionElevation;

// CRF related parameters.
constexpr float kXYDev = 1.0f;         // xy deviation in smoothness kernel
constexpr float kWeightSmooth = 1.0f;  // weight for smoothness kernel
constexpr float kXYDevApp = 2.0f;      // xy deviation in appearance kernel
constexpr float kRIDevApp =
    2.0f;  // Range&Intensity deviation in appearance kernel
constexpr float kWeightApp = 10.0f;  // weight for appearance kernel
constexpr int kNumIterations = 2;    // number of iterations for CRF

using SemsegIndicesPerDegree = std::array<int8_t, 360>;

int NormalizeCol(const int col, const int width) {
  const int rem_col = col % width;
  return rem_col < 0 ? rem_col + width
                     : (rem_col >= width ? rem_col - width : rem_col);
}
// Wrap angle to [0, 360.0)
double WrapAngleInDegree(const double angle) {
  return angle < 0.0 ? angle + 360.0 : angle >= 360.0 ? angle - 360.0 : angle;
}

double ComputeCameraHfovHalf(
    const CameraIntrinsicsMatrix& camera_intrinsics_matrix) {
  return fast_math::Atan2(camera_intrinsics_matrix.cx(),
                          camera_intrinsics_matrix.fx()) +
         kCameraHFovBuffer;
}

std::vector<double> ComputeCameraHfovHalves(
    const std::vector<CameraIntrinsicsMatrix>& camera_intrinsics_matrixs) {
  const int size = camera_intrinsics_matrixs.size();
  std::vector<double> hfov_halfves(camera_intrinsics_matrixs.size());
  for (int i = 0; i < size; i++) {
    hfov_halfves[i] = ComputeCameraHfovHalf(camera_intrinsics_matrixs[i]);
  }

  return hfov_halfves;
}

std::pair<uint16_t, uint16_t> ConvertAnglesToRC(const double elevation,
                                                const double azimuth) {
  const int row = (-elevation - kMinElevationOfM1) * kResolutionElevationInv;
  const int col = (-azimuth - kMinAzimuthOfM1) * kResolutionAzimuthInv;
  return {row, col};
}

SemsegIndicesPerDegree ComputeSemsegIndicesPerDegree(
    const LidarParametersProto& lidar_param,
    const SemanticSegmentationResults& semseg_results) {
  SemsegIndicesPerDegree semseg_indices_per_degree({0});
  const auto& lidar_extrinsics = lidar_param.installation().extrinsics();
  std::vector<CameraIntrinsicsMatrix> camera_intrinsics_matrixs;
  camera_intrinsics_matrixs.reserve(semseg_results.size());
  for (const auto& semseg_res : semseg_results) {
    camera_intrinsics_matrixs.emplace_back(
        semseg_res->camera_param().camera_matrix());
  }
  const std::vector<double> hfov_halves =
      ComputeCameraHfovHalves(camera_intrinsics_matrixs);
  for (int i = 0; i < semseg_indices_per_degree.size(); i++) {
    semseg_indices_per_degree[i] = -1;
    double min_yaw_diff = std::numeric_limits<double>::max();
    for (int j = 0; j < semseg_results.size(); ++j) {
      const double camera_to_vehicle_yaw =
          semseg_results[j]->camera_param().camera_to_vehicle_extrinsics().yaw;
      const double scan_yaw =
          M_PI - d2r(static_cast<double>(i)) + lidar_extrinsics.yaw();
      const double yaw_diff =
          std::abs(NormalizeAngle(scan_yaw - camera_to_vehicle_yaw));
      if (yaw_diff < hfov_halves[j] && yaw_diff < min_yaw_diff) {
        semseg_indices_per_degree[i] = j;
        min_yaw_diff = yaw_diff;
      }
    }
  }
  return semseg_indices_per_degree;
}

}  // namespace

RangeImageProto RangeImage::RangeImageToProto(const RangeImage& range_image) {
  RangeImageProto range_image_proto;

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  // Compression level: 0-9, default is 1
  // A higher value means a smaller size and longer compression time
  compression_params.push_back(5);

  std::vector<uchar> encoded_intensity_image;
  cv::imencode(".png", range_image.intensity_image(), encoded_intensity_image,
               compression_params);
  range_image_proto.set_intensity_image(
      reinterpret_cast<void*>(encoded_intensity_image.data()),
      encoded_intensity_image.size());

  std::vector<uchar> encoded_range_image;
  cv::imencode(".png", range_image.range_image(), encoded_range_image,
               compression_params);
  range_image_proto.set_range_image(
      reinterpret_cast<void*>(encoded_range_image.data()),
      encoded_range_image.size());

  std::vector<uchar> encoded_semantic_image;
  cv::imencode(".png", range_image.semantic_image(), encoded_semantic_image,
               compression_params);
  range_image_proto.set_semantic_image(
      reinterpret_cast<void*>(encoded_semantic_image.data()),
      encoded_semantic_image.size());

  std::vector<uchar> encoded_instance_image;
  cv::imencode(".png", range_image.instance_image(), encoded_instance_image,
               compression_params);
  range_image_proto.set_instance_image(
      reinterpret_cast<void*>(encoded_instance_image.data()),
      encoded_instance_image.size());

  range_image_proto.set_lidar_id(range_image.lidar_id());
  range_image_proto.set_max_range_in_meter(RangeImage::kMaxRange);

  return range_image_proto;
}

RangeImage::RangeImage(const LidarParametersProto& lidar_param,
                       const LidarFrame& lidar_frame,
                       const SemanticSegmentationResults& semseg_results)
    : lidar_param_(lidar_param),
      lidar_frame_(lidar_frame),
      semseg_results_(semseg_results) {
  if (lidar_frame_.is_spin()) {
    InitWithSpin(*lidar_frame_.spin());
  } else if (lidar_frame_.is_point_cloud()) {
    InitWithPointCloud(*lidar_frame_.point_cloud());
  } else {
    QLOG(FATAL) << "Should not reach here.";
  }

  // DenseCRF only enables LDR_FRONT for DBQV4 and PBQ. LDR_FRONT_LEFT and
  // LDR_FRONT_RIGHT for other DBQ configurations
  if (FLAGS_enable_crf) {
    const auto run_context = QCraftRunContext();
    if (!IsDBQConext(run_context) || IsDBQv4(run_context)) {
      if (lidar_frame_.lidar_id() == LidarId::LDR_FRONT) {
        PostProcessRangeImage();
      }
    } else {
      const bool lidar_process =
          (lidar_frame_.lidar_id() == LidarId::LDR_FRONT_LEFT ||
           lidar_frame_.lidar_id() == LidarId::LDR_FRONT_RIGHT);
      if (lidar_process) PostProcessRangeImage();
    }
  }
}

void RangeImage::InitializeAllImages(const int height, const int width) {
  range_image_.create(height, width, CV_8UC1);
  std::memset(range_image_.data, 255, height * width);

  intensity_image_.create(height, width, CV_8UC1);
  std::memset(intensity_image_.data, 0, height * width);

  semantic_image_.create(height, width, CV_8UC2);
  std::memset(semantic_image_.data, 0, height * width * 2);

  instance_image_.create(height_, width_, CV_8UC1);
  memset(instance_image_.data, 0, height_ * width_);
}

void RangeImage::UpdateSemanticImageUsingSemsegResult(
    const Vec3d& point, const int semseg_result_index, const int row,
    const int col) {
  const auto& semseg_result = semseg_results_[semseg_result_index];

  // comment out below DCHECK temporarily for compatible with old data
  // TODO(haijun): recover the check after use new data for PRT

  // const int sem_seg_output_scale =
  //     static_cast<int>(semseg_result->sem_seg_output_scale());
  constexpr int kSemSegOutputScale = 8;
  // DCHECK_EQ(sem_seg_output_scale, kSemSegOutputScale);
  if (const auto img_pos = projection_util::SmoothPointToImagePos(
          point, semseg_result->smooth_to_camera_transform(),
          semseg_result->camera_param())) {
    const cv::Mat& semantic_mask = semseg_result->semantic_mat();
    const cv::Mat& uncertainty_mask = semseg_result->uncertainty_mat();
    const cv::Mat& instance_mask = semseg_result->instance_mat();
    const int index = (img_pos->y() / kSemSegOutputScale) * semantic_mask.cols +
                      (img_pos->x() / kSemSegOutputScale);
    semantic_image_.at<cv::Vec2b>(row, col) = {semantic_mask.data[index],
                                               uncertainty_mask.data[index]};
    instance_image_.at<uchar>(row, col) = instance_mask.data[index];
  }
}

void RangeImage::InitWithPointCloud(const PointCloud& pointcloud) {
  SCOPED_QTRACE_ARG1("RangeImage::InitWithPointCloud", "lidar_id",
                     LidarId_Name(pointcloud.lidar_id()));
  height_ = static_cast<int>((kMaxElevationOfM1 - kMinElevationOfM1) *
                             kResolutionElevationInv);
  width_ = static_cast<int>((kMaxAzimuthOfM1 - kMinAzimuthOfM1) *
                            kResolutionAzimuthInv);

  InitializeAllImages(height_, width_);

  const int num_points = pointcloud.num_points();
  pointcloud_rc_to_index_.resize(height_ * width_);
  index_to_rc_.resize(num_points);
  std::vector<uint8_t> num_points_at_rc(height_ * width_, 0);
  int num_points_out_of_rc_to_index = 0;

  // For each 1-degree azimuth, calculate the index of the best fit
  // segmentation.
  const SemsegIndicesPerDegree semseg_indices_per_degree =
      ComputeSemsegIndicesPerDegree(lidar_param_, semseg_results_);

  for (int i = 0; i < num_points; i++) {
    const auto& point = pointcloud.point(i);
    const double range = point.range();  // m
    if (range == 0) {
      continue;
    }
    const auto [row, col] =
        ConvertAnglesToRC(point.elevation_deg(), point.azimuth_deg());
    DCHECK(row >= 0 && row < height_ && col >= 0 && col < width_)
        << "col : " << col << " row " << row << " height_ " << height_
        << " width_ " << width_;
    range_image_region_.min_row = std::min(range_image_region_.min_row, row);
    range_image_region_.max_row = std::max(range_image_region_.max_row, row);
    range_image_region_.min_col = std::min(range_image_region_.min_col, col);
    range_image_region_.max_col = std::max(range_image_region_.max_col, col);

    const int index = row * width_ + col;

    if (num_points_at_rc[index] < kMaxNumPointsPerPixel) {
      pointcloud_rc_to_index_[index][num_points_at_rc[index]] = i;
      num_points_at_rc[index]++;
    } else {
      num_points_out_of_rc_to_index++;
    }
    index_to_rc_[i] = {row, col};

    range_image_.data[index] =
        std::clamp(RoundToInt(range * (1.f / kMaxRange) * 255.f), 0, 255);
    intensity_image_.data[index] = point.intensity;

    // The azimuth is the inclination between x-axis and the direction of point,
    // and the azimuth is negative on the right and positive on the left. Here
    // we compute the index of the nearest camera is consistent with the spin.
    // So we calculate the angle between the negative x-axis direction with the
    // direction of point.
    const double azimuth = WrapAngleInDegree(180.0 - point.azimuth_deg());
    // Compute sementic image
    // Check if the scan match the given camera.
    const int in_view_semseg_result_index =
        semseg_indices_per_degree[FloorToInt(azimuth)];
    if (in_view_semseg_result_index >= 0) {
      semantic_image_region_.max_col =
          std::max(semantic_image_region_.max_col, col);
      semantic_image_region_.min_col =
          std::min(semantic_image_region_.min_col, col);
      UpdateSemanticImageUsingSemsegResult(
          {point.x, point.y, point.z}, in_view_semseg_result_index, row, col);
    }
  }

  constexpr int kMaxNumOutOfRcToIndexPoints = 500;

  if (num_points_out_of_rc_to_index > kMaxNumOutOfRcToIndexPoints) {
    QEVENT("zhangtao", "num_points_out_of_rc_to_index", [&](QEvent* qevent) {
      qevent
          ->AddField("num_points_out_of_rc_to_index",
                     num_points_out_of_rc_to_index)
          .AddField("num of points", num_points);
    });
  }
}

void RangeImage::InitWithSpin(const Spin& spin) {
  SCOPED_QTRACE_ARG1("RangeImage::InitWithSpin", "lidar_id",
                     LidarId_Name(spin.lidar_id()));
  QCHECK(lidar_param_.inherent().has_intrinsics());
  const auto& intrinsics = lidar_param_.inherent().intrinsics();

  height_ = spin.num_beams();
  width_ = spin.num_scans();

  InitializeAllImages(height_, width_);

  spin_rc_to_index_.resize(height_ * width_);
  index_to_rc_.resize(height_ * width_);

  // For each 1-degree azimuth, calculate the index of the best fit
  // segmentation.
  const SemsegIndicesPerDegree semseg_indices_per_degree =
      ComputeSemsegIndicesPerDegree(lidar_param_, semseg_results_);

  const auto azimuth_offsets_protobuf = intrinsics.azimuth_offsets();
  const std::vector<double> azimuth_offsets(azimuth_offsets_protobuf.begin(),
                                            azimuth_offsets_protobuf.end());
  std::vector<double> start_frames;
  if (lidar_frame_.lidar_type() == LIDAR_PANDAR_AT128) {
    const auto start_frames_protobuf =
        lidar_param_.inherent().intrinsics().start_frames();
    start_frames.assign(start_frames_protobuf.begin(),
                        start_frames_protobuf.end());
  }

  std::vector<int> col_offsets(spin.num_beams());
  for (int i = 0; i < spin.num_beams(); ++i) {
    col_offsets[i] =
        RoundToInt(azimuth_offsets[i] * (1.0 / 360.0) * spin.num_scans());
  }

  // TODO(dong, yu): parallelize across beams.
  for (int i = 0; i < spin.num_scans(); ++i) {
    const auto& scan = spin.scan(i);
    for (int j = 0; j < scan.num_shots; ++j) {
      const auto& shot = *(scan.shots + j);
      if (shot.num_returns == 0) {
        continue;
      }

      const uint16_t row = j;
      const uint16_t col = NormalizeCol(i + col_offsets[j], spin.num_scans());
      DCHECK(col >= 0 && col < spin.num_scans()) << col;
      range_image_region_.min_row = std::min(range_image_region_.min_row, row);
      range_image_region_.max_row = std::max(range_image_region_.max_row, row);
      range_image_region_.min_col = std::min(range_image_region_.min_col, col);
      range_image_region_.max_col = std::max(range_image_region_.max_col, col);
      // Choose the shot with max intensity.
      int return_index_with_max_intensity = 0;
      uint8_t max_intensity = shot.calibrated_returns[0].intensity;
      for (int k = 1; k < shot.num_returns; ++k) {
        const auto& calib_return = shot.calibrated_returns[k];
        const uint8_t intensity = calib_return.intensity;
        if (max_intensity < intensity) {
          max_intensity = intensity;
          return_index_with_max_intensity = k;
        }
      }
      const auto& selected_return =
          shot.calibrated_returns[return_index_with_max_intensity];

      spin_rc_to_index_[row * width_ + col] =
          ReturnIndex(i, j, return_index_with_max_intensity);
      index_to_rc_[i * height_ + j] = {row, col};

      const int index = row * width_ + col;
      range_image_.data[index] = std::clamp(
          RoundToInt(selected_return.range * (1.f / kMaxRange * 255.f)), 0,
          255);
      intensity_image_.data[index] = selected_return.intensity;

      double azimuth = range_image_util::ComputeSpinAzimuth(
          lidar_frame_.lidar_type(), j, scan.azimuth_in_degree, azimuth_offsets,
          start_frames);

      azimuth = WrapAngleInDegree(azimuth);

      // Compute semantic image
      // Check if the scan match the given camera.
      const int in_view_semseg_result_index =
          semseg_indices_per_degree[FloorToInt(azimuth)];

      if (in_view_semseg_result_index >= 0) {
        semantic_image_region_.max_col =
            std::max(semantic_image_region_.max_col, col);
        semantic_image_region_.min_col =
            std::min(semantic_image_region_.min_col, col);
        UpdateSemanticImageUsingSemsegResult(
            {selected_return.x, selected_return.y, selected_return.z},
            in_view_semseg_result_index, row, col);
      }
    }
  }
}

void RangeImage::PostProcessRangeImage() {
  // Data validity check
  if (semantic_image_region_.max_col <= semantic_image_region_.min_col) {
    QLOG(INFO) << "NO semantics results have been projected in the range "
                  "image. Skipping post processing.";
    return;
  }

  SCOPED_QTRACE_ARG1(
      "RangeImage::PostProcessRangeImage", "num_cols",
      semantic_image_region_.max_col - semantic_image_region_.min_col + 1);
  // Use Dense CRF to process the result
  cv::Mat updated_semantic_image = range_image_util::DenseCRFProcessRangeImage(
      semantic_image_, range_image_, intensity_image_,
      semantic_image_region_.min_col, semantic_image_region_.max_col,
      kNumIterations, kWeightSmooth, kXYDev, kWeightApp, kXYDevApp, kRIDevApp);

  // Assign the new semantic labels to the class member variable
  semantic_image_ = updated_semantic_image;

  return;
}

}  // namespace qcraft
