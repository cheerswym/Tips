#include "onboard/perception/range_image/range_image_dense_crf_base.h"

#include <algorithm>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qcraft::range_image_dense_crf {

std::vector<float> RangeImageDenseCRFBase::GetUnaryFromUncertainty(
    const cv::Mat& img_semantic, const int num_class) {
  // Validity check
  std::vector<float> unary_potential;
  if (num_class <= 0) return unary_potential;
  if (img_semantic.empty()) return unary_potential;
  if (img_semantic.type() != CV_8UC2) return unary_potential;

  // Get log map once
  static auto log_map = GenerateLogMap(num_class);
  const int w = img_semantic.cols;
  const int h = img_semantic.rows;
  const uchar min_confi = static_cast<uchar>(255 / num_class);
  // Resize the vector of unary potentials
  unary_potential.resize(w * h * num_class, 0.0f);

  for (size_t i = 0; i < h; i++) {
    for (size_t j = 0; j < w; j++) {
      uchar confi = img_semantic.at<cv::Vec2b>(i, j)[1];
      const size_t label =
          static_cast<size_t>(img_semantic.at<cv::Vec2b>(i, j)[0]);
      if (label >= num_class || confi < 0) {
        QLOG_EVERY_N_SEC(ERROR, 1) << "Invalid label or confidence value!";
        continue;
      }
      if (confi < min_confi) confi = 127;
      const size_t start_index = (i * w + j) * num_class;
      // Get the beginning index of the unary potential
      const auto log_value = FindOrDie(log_map, confi);
      std::fill_n(unary_potential.begin() + start_index, num_class,
                  log_value.second);
      unary_potential[start_index + label] = log_value.first;
    }
  }

  return unary_potential;
}

std::vector<float> RangeImageDenseCRFBase::ReadRangeIntensity(
    const cv::Mat& img_range, const cv::Mat& img_intensity) {
  // Validity check
  std::vector<float> ri_feat;
  if (img_range.empty() || img_intensity.empty()) return ri_feat;
  if (img_range.cols != img_intensity.cols ||
      img_range.rows != img_intensity.rows) {
    QLOG(ERROR) << "The range and intensity images are not the same size";
    return ri_feat;
  }

  // Rename the column and witdth of image with a simpler naming convention
  const int w = img_range.cols;
  const int h = img_range.rows;

  // Allocate memory for the range/intensity feature
  ri_feat.resize(w * h * 2, 0.0f);

  // Read the range and intensity features
  for (size_t i = 0; i < h; i++) {
    for (size_t j = 0, k = 0; j < 2 * w; j += 2, k++) {
      ri_feat[i * 2 * w + j] =
          static_cast<float>(img_range.data[i * img_range.step1() + k]);
      ri_feat[i * 2 * w + j + 1] =
          static_cast<float>(img_intensity.data[i * img_intensity.step1() + k]);
    }
  }

  return ri_feat;
}

absl::flat_hash_map<uchar, std::pair<float, float>>
RangeImageDenseCRFBase::GenerateLogMap(const int num_class) {
  // Generate an unordered  map for the log function
  absl::flat_hash_map<uchar, std::pair<float, float>> log_map;
  const int min_value = 255 / num_class;
  const int byte_max = 256;

  for (int i = min_value; i < byte_max; i++) {
    const float confi = i / 255.0f;
    const float rest_confi = (1.0f - confi) / (num_class - 1);
    const float unary_potential_other = -std::log(rest_confi);
    const float unary_potential = -std::log(confi);
    InsertOrDie(&log_map, static_cast<uchar>(i),
                std::make_pair(unary_potential, unary_potential_other));
  }

  return log_map;
}
}  // namespace qcraft::range_image_dense_crf
