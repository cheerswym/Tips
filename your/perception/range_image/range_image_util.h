#ifndef ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_UTIL_H_
#define ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_UTIL_H_

#include <vector>

#include "onboard/perception/semantic_segmentation_result.h"
#include "opencv2/core.hpp"

namespace qcraft::range_image_util {

SemanticSegmentationResults CollectSemanticSegmentationResultsForCurrentLidar(
    const LidarId lidar_id, const SemanticSegmentationResults &semseg_results);

double ComputeAT128Azimuth(const int beam_index, const double azimuth,
                           const std::vector<double> &azimuth_offsets,
                           const int num_beams,
                           const std::vector<double> &start_frames);

double ComputeSpinAzimuth(const LidarModel lidar_type, const int beam_index,
                          const double azimuth,
                          const std::vector<double> &azimuth_offsets,
                          const std::vector<double> &start_frames);

cv::Mat DenseCRFProcessRangeImage(
    const cv::Mat semantic_image, const cv::Mat range_iamge,
    const cv::Mat intensity_image, const int min_col, const int max_col,
    const int num_iterations = 10, const float weight_smoothness = 1.0,
    const float smoothness_xy_dev = 1.0, const float weight_app = 10.0,
    const float app_xy_dev = 2.0, const float app_ri_dev = 2.0);
}  // namespace qcraft::range_image_util

#endif  // ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_UTIL_H_
