#ifndef ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_GPU_H_
#define ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_GPU_H_

#include <utility>
#include <vector>

#include "onboard/perception/range_image/range_image_dense_crf_base.h"

namespace qcraft::range_image_dense_crf {

class RangeImageDenseCRFGPU : public RangeImageDenseCRFBase {
 public:
  RangeImageDenseCRFGPU(const cv::Mat& semantic_image,
                        const cv::Mat& range_image,
                        const cv::Mat& intensity_image)
      : RangeImageDenseCRFBase(semantic_image, range_image, intensity_image) {}

  RangeImageDenseCRFGPU() = delete;
  ~RangeImageDenseCRFGPU() = default;

  // Run the dense CRF algorithm to refine range image
  std::pair<std::vector<int16_t>, std::vector<float>>
  PostProcessRangeImageDenseCRF(const int num_iterations,
                                const float weight_smoothness,
                                const float smoothness_xy_dev,
                                const float weight_app, const float app_xy_dev,
                                const float app_ri_dev) final;
};

}  // namespace qcraft::range_image_dense_crf

#endif  // ONBOARD_PERCEPTION_RANGE_IMAGE_RANGE_IMAGE_DENSE_CRF_GPU_H_
