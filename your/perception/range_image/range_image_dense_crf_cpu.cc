#include "onboard/perception/range_image/range_image_dense_crf_cpu.h"

#include <algorithm>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "third_party/dense_crf/densecrf_cpu.h"
#include "third_party/dense_crf/pairwise_cpu.h"
namespace qcraft::range_image_dense_crf {

std::pair<std::vector<int16_t>, std::vector<float>>
RangeImageDenseCRFCPU::PostProcessRangeImageDenseCRF(
    const int num_iterations, const float weight_smoothness,
    const float smoothness_xy_dev, const float weight_app,
    const float app_xy_dev, const float app_ri_dev) {
  // We must give the constexpr value to the number of classes for template
  constexpr int num_class = SegmentationType_ARRAYSIZE - 1;

  // Initalize the result of map and prob
  std::vector<int16_t> map;
  std::vector<float> prob;
  // Get the unary potentials calculated from uncertainty score
  const int w = semantic_image_.cols;
  const int h = semantic_image_.rows;
  auto unary_energy = GetUnaryFromUncertainty(semantic_image_, num_class);

  if (unary_energy.size() != w * h * num_class) {
    QLOG(ERROR) << "Failed to generate correct unary potentials! Post "
                   "Processing of range image will be terminated.";
    return std::make_pair(map, prob);
  }

  // Get range and intensity data together
  auto ri_feat = ReadRangeIntensity(range_image_, intensity_image_);
  if (ri_feat.size() != w * h * 2) {
    QLOG(WARNING) << "Failed to load range or intensity image! Post Processing "
                     "of range image will be terminated.";
    return std::make_pair(map, prob);
  }

  // Initialize the output of CRF model
  map.resize(w * h, 0);
  prob.resize(w * h * num_class, 0.0f);

  // Initalize the CPU version of DenseCRF and add two indenepdent pairwise
  // terms
  dcrf_cuda::DenseCRFCPU<num_class> crf(w * h);
  crf.setUnaryEnergy(unary_energy.data());

  auto* smoothness_pairwise =
      dcrf_cuda::PottsPotentialCPU<num_class, 2>::FromImage<>(
          w, h, weight_smoothness, smoothness_xy_dev);
  crf.addPairwiseEnergy(smoothness_pairwise);

  auto* appearance_pairwise =
      dcrf_cuda::PottsPotentialCPU<num_class, 4>::FromImage<float>(
          w, h, weight_app, app_xy_dev, ri_feat.data(), app_ri_dev);
  crf.addPairwiseEnergy(appearance_pairwise);

  // Do map inference and get probability
  crf.inference(num_iterations, true);
  int16_t* map_cpu = crf.getMap();
  float* prob_cpu = crf.getProbability();

  // Project the result to the output vector
  std::copy(map_cpu, map_cpu + w * h, map.begin());
  std::copy(prob_cpu, prob_cpu + w * h * num_class, prob.begin());

  // return the value
  return std::make_pair(std::move(map), std::move(prob));
}

}  // namespace qcraft::range_image_dense_crf
