#ifndef ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_BASE_H_
#define ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_BASE_H_

#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "opencv2/core.hpp"

namespace qcraft::range_image_dense_crf {

class RangeImageDenseCRFBase {
 public:
  RangeImageDenseCRFBase(const cv::Mat& semantic_image,
                         const cv::Mat& range_image,
                         const cv::Mat& intensity_image)
      : semantic_image_(semantic_image),
        range_image_(range_image),
        intensity_image_(intensity_image) {}
  RangeImageDenseCRFBase() = delete;
  ~RangeImageDenseCRFBase() = default;

  // Run the dense CRF algorithm to refine range image
  virtual std::pair<std::vector<int16_t>, std::vector<float>>
  PostProcessRangeImageDenseCRF(const int num_iterations,
                                const float weight_smoothness,
                                const float smoothness_xy_dev,
                                const float weight_app, const float app_xy_dev,
                                const float app_ri_dev) = 0;

 public:
  // Range image, itnensity and semantic image, semantic image has two channels
  // while the first channel is its semantic label while the second channel is
  // its uncertainty value
  cv::Mat semantic_image_;
  cv::Mat range_image_;
  cv::Mat intensity_image_;

  // Get unary potential from semantic and uncertainty
  std::vector<float> GetUnaryFromUncertainty(const cv::Mat& img_semantic,
                                             const int num_class);
  // Get range/intenisty from range image and intensity image
  std::vector<float> ReadRangeIntensity(const cv::Mat& img_range,
                                        const cv::Mat& img_intensity);

  // Generate a log map to reduce unary term calculation
  absl::flat_hash_map<uchar, std::pair<float, float>> GenerateLogMap(
      const int num_class);
};

}  // namespace qcraft::range_image_dense_crf

#endif  // ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_BASE_H_
