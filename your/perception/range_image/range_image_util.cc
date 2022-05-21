#include "onboard/perception/range_image/range_image_util.h"

#include <algorithm>
#include <chrono>
#include <tuple>
#include <utility>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lidar/lidar_util.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/proto/perception.pb.h"

#ifndef Q_CPU_ONLY
#include "onboard/perception/range_image/range_image_dense_crf_gpu.h"
#else
#include "onboard/perception/range_image/range_image_dense_crf_cpu.h"
#endif

namespace qcraft::range_image_util {

namespace {

bool ContainsCamera(const SemanticSegmentationResults &semseg_results,
                    const CameraId camera_id) {
  return std::find_if(semseg_results.begin(), semseg_results.end(),
                      [=](const auto &result) {
                        return result->camera_id() == camera_id;
                      }) != semseg_results.end();
}

cv::Mat UpdateSemanticResult(const std::vector<int16_t> map,
                             const std::vector<float> prob, const int min_col,
                             const int max_col, const cv::Mat &semantic_image) {
  // Data availibility check
  cv::Mat updated_semantic = semantic_image.clone();
  if (semantic_image.empty() || map.size() <= 0 || prob.size() <= 0) {
    QLOG(INFO) << "Updating has issues in input data (empty semantic or absent "
                  "CRF result), keep original semantic result";
    return updated_semantic;
  }

  // This is not perfect and hard-coded, may think a better way to do this
  // (simple way is getting the result from RangeImageDenseCRF class)
  const int num_class = SegmentationType_ARRAYSIZE - 1;
  // Update semantic result, be aware of the semantic image size is equal
  // or larger than map and prob value
  const float min_prob = 1 / static_cast<float>(num_class);
  const int cols = max_col - min_col + 1;

  for (int i = 0; i < semantic_image.rows; i++) {
    for (int j = 0; j < cols; j++) {
      const uchar label = static_cast<uchar>(map[i * cols + j]);
      QCHECK(label >= 0 && label < num_class);

      // Calculate the uncertainty value from prob distribution
      float unormalize_uncertainty = 0;
      const size_t start_index = (i * cols + j) * num_class;
      std::for_each(
          prob.begin() + start_index, prob.begin() + start_index + num_class,
          [&](const float &value) { unormalize_uncertainty += value * value; });
      QCHECK(unormalize_uncertainty >= min_prob && unormalize_uncertainty <= 1);
      const uchar uncertainty =
          static_cast<uchar>(unormalize_uncertainty * 255);
      // Update the semantic image directly with updated label and uncertainty
      updated_semantic.at<cv::Vec2b>(i, j + min_col) = {label, uncertainty};
    }
  }

  return updated_semantic;
}

// Crop regions for CRF to process, only crop it on the column not row
std::tuple<cv::Mat, cv::Mat, cv::Mat> CropEffectiveCRFRegion(
    const cv::Mat semantic_image, const cv::Mat range_image,
    const cv::Mat intensity_image, const int min_col, const int max_col) {
  // Data availibility check
  if (semantic_image.empty() || range_image.empty() ||
      intensity_image.empty()) {
    QLOG(INFO) << "Empty data for CRF post processing";
    return std::make_tuple(semantic_image, range_image, intensity_image);
  }

  const cv::Rect2i sub_region(min_col, 0, max_col - min_col + 1,
                              semantic_image.rows);
  cv::Mat cropped_semantic_image = semantic_image(sub_region);
  cv::Mat cropped_range_image = range_image(sub_region);
  cv::Mat cropped_intensity_image = intensity_image(sub_region);
  return std::make_tuple(cropped_semantic_image, cropped_range_image,
                         cropped_intensity_image);
}

}  // namespace

SemanticSegmentationResults CollectSemanticSegmentationResultsForCurrentLidar(
    const LidarId lidar_id, const SemanticSegmentationResults &semseg_results) {
  SemanticSegmentationResults selected_semseg_results;
  selected_semseg_results.reserve(semseg_results.size());
  for (const auto &result : semseg_results) {
    // Prefer rotated camera in the same view.
    if (result->camera_id() == CAM_R_FRONT &&
        ContainsCamera(semseg_results, CAM_L_FRONT)) {
      continue;
    }
    selected_semseg_results.emplace_back(result);
  }

  return selected_semseg_results;
}

double ComputeAT128Azimuth(const int beam_index, const double azimuth,
                           const std::vector<double> &azimuth_offsets,
                           const std::vector<double> &start_frames) {
  const int frame_id = ComputeAT128FrameId(start_frames, azimuth);
  DCHECK_GE(frame_id, 0) << azimuth;

  constexpr double kTwoThirdsPi = M_PI_2 + M_PI;

  const double beam_azimuth = (azimuth - start_frames[frame_id]) * 2.0 -
                              azimuth_offsets[beam_index] - r2d(kTwoThirdsPi);

  return beam_azimuth;
}

double ComputeSpinAzimuth(const LidarModel lidar_type, const int beam_index,
                          const double azimuth,
                          const std::vector<double> &azimuth_offsets,
                          const std::vector<double> &start_frames) {
  switch (lidar_type) {
    case LIDAR_PANDAR_AT128:
      return ComputeAT128Azimuth(beam_index, azimuth, azimuth_offsets,
                                 start_frames);
    default:
      return azimuth + azimuth_offsets[beam_index];
  }
}

cv::Mat DenseCRFProcessRangeImage(
    const cv::Mat semantic_image, const cv::Mat range_iamge,
    const cv::Mat intensity_image, const int min_col, const int max_col,
    const int num_iterations, const float weight_smoothness,
    const float smoothness_xy_dev, const float weight_app,
    const float app_xy_dev, const float app_ri_dev) {
  // Crop the effective CRF process region first
  const int mid_col = (min_col + max_col) / 2;
  const int half_size = std::max((max_col - min_col + 1) / 8, 1);
  const int min_col_update = mid_col - half_size;
  const int max_col_update = mid_col + half_size;
  auto crop_image =
      CropEffectiveCRFRegion(semantic_image, range_iamge, intensity_image,
                             min_col_update, max_col_update);

  cv::Mat semantic_image_crop = std::get<0>(crop_image);
  cv::Mat range_image_crop = std::get<1>(crop_image);
  cv::Mat intensity_image_crop = std::get<2>(crop_image);

#ifndef Q_CPU_ONLY
  // Run CRF GPU inference
  range_image_dense_crf::RangeImageDenseCRFGPU crf_gpu(
      semantic_image_crop, range_image_crop, intensity_image_crop);
  auto crf_result = crf_gpu.PostProcessRangeImageDenseCRF(
      num_iterations, weight_smoothness, smoothness_xy_dev, weight_app,
      app_xy_dev, app_ri_dev);
  // Update original semantic result
  if (crf_result.first.size() > 0 && crf_result.second.size() > 0) {
    cv::Mat updated_semantic_image =
        UpdateSemanticResult(crf_result.first, crf_result.second,
                             min_col_update, max_col_update, semantic_image);
    return updated_semantic_image;
  } else {
    return semantic_image;
  }

#else
  // Run CRF CPU inference
  range_image_dense_crf::RangeImageDenseCRFCPU crf_cpu(
      semantic_image_crop, range_image_crop, intensity_image_crop);
  auto crf_result = crf_cpu.PostProcessRangeImageDenseCRF(
      num_iterations, weight_smoothness, smoothness_xy_dev, weight_app,
      app_xy_dev, app_ri_dev);
  // Update original semantic result
  if (crf_result.first.size() > 0 && crf_result.second.size() > 0) {
    cv::Mat updated_semantic_image =
        UpdateSemanticResult(crf_result.first, crf_result.second,
                             min_col_update, max_col_update, semantic_image);

    return updated_semantic_image;
  } else {
    return semantic_image;
  }
#endif
}

}  // namespace qcraft::range_image_util
