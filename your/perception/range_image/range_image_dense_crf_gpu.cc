#include "onboard/perception/range_image/range_image_dense_crf_gpu.h"

#include <algorithm>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/proto/perception.pb.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "third_party/dense_crf/densecrf_gpu.h"
#include "third_party/dense_crf/pairwise_gpu.h"

DEFINE_int32(dense_crf_gpu_id, 1,
             "The id of gpu to run dense crf on range image");

namespace qcraft::range_image_dense_crf {

std::pair<std::vector<int16_t>, std::vector<float>>
RangeImageDenseCRFGPU::PostProcessRangeImageDenseCRF(
    const int num_iterations, const float weight_smoothness,
    const float smoothness_xy_dev, const float weight_app,
    const float app_xy_dev, const float app_ri_dev) {
  // We must give the constexpr value to the number of classes for template
  constexpr int num_class = SegmentationType_ARRAYSIZE - 1;

  // Set GPU ID first to avoid the GPU conflict with FEN.
  if (cudaSetDevice(FLAGS_dense_crf_gpu_id) != cudaSuccess) {
    auto cuda_error = cudaGetLastError();
    std::string cuda_error_msg(cudaGetErrorString(cuda_error));
    QLOG(WARNING) << "CUDA error message is:" << cuda_error_msg
                  << absl::StreamFormat(
                         ". GPU id %d is not valid, fallback to gpu 0.",
                         FLAGS_dense_crf_gpu_id);
    QCHECK_EQ(cudaSetDevice(0), cudaSuccess);
  }

  // Initialize the result of map and prob
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

  // Copy the unary potentials to GPU
  float* unary_gpu = nullptr;
  cudaMalloc(reinterpret_cast<void**>(&unary_gpu),
             sizeof(float) * w * h * num_class);
  cudaMemcpy(unary_gpu, unary_energy.data(), sizeof(float) * w * h * num_class,
             cudaMemcpyHostToDevice);

  // Get range and intensity data together
  auto ri_feat = ReadRangeIntensity(range_image_, intensity_image_);

  if (ri_feat.size() != w * h * 2) {
    QLOG(WARNING) << "Failed to load range or intensity image! Post Processing "
                     "of range image will be terminated.";
    return std::make_pair(map, prob);
  }

  // Copy the range and intensity data to GPU
  float* ri_feat_gpu = nullptr;
  cudaMalloc(reinterpret_cast<void**>(&ri_feat_gpu), sizeof(float) * w * h * 2);
  cudaMemcpy(ri_feat_gpu, ri_feat.data(), sizeof(float) * w * h * 2,
             cudaMemcpyHostToDevice);

  // Initialize the output of CRF model
  map.resize(w * h, 0);
  prob.resize(w * h * num_class, 0.0f);

  // Run the CRF model
  dcrf_cuda::DenseCRFGPU<num_class> crf(w * h);
  crf.setUnaryEnergy(unary_gpu);

  auto* smoothness_pairwise =
      dcrf_cuda::PottsPotentialGPU<num_class, 2>::FromImage<>(
          w, h, weight_smoothness, smoothness_xy_dev);
  crf.addPairwiseEnergy(smoothness_pairwise);

  auto* appearance_pairwise =
      dcrf_cuda::PottsPotentialGPU<num_class, 4>::FromImage<float>(
          w, h, weight_app, app_xy_dev, ri_feat_gpu, app_ri_dev);
  crf.addPairwiseEnergy(appearance_pairwise);

  // Do map inference and get probability
  crf.inference(num_iterations, true);

  // Copy the result from GPU to CPU
  int16_t* map_gpu = crf.getMap();
  cudaMemcpy(reinterpret_cast<void*>(map.data()),
             reinterpret_cast<void*>(map_gpu), sizeof(int16_t) * w * h,
             cudaMemcpyDeviceToHost);
  float* prob_gpu = crf.getProbability();
  cudaMemcpy(reinterpret_cast<void*>(prob.data()),
             reinterpret_cast<void*>(prob_gpu),
             sizeof(float) * w * h * num_class, cudaMemcpyDeviceToHost);

  // Clean the GPU memory
  cudaFree(unary_gpu);
  cudaFree(ri_feat_gpu);

  // Return the value
  return std::make_pair(std::move(map), std::move(prob));
}

}  // namespace qcraft::range_image_dense_crf
