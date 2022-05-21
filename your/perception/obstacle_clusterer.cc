#include "onboard/perception/obstacle_clusterer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stack>
#include <tuple>
#include <utility>
#include <vector>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

DEFINE_bool(obstacle_clusterer_cvs, false, "Render obstacle clusterer results");
DEFINE_bool(blured_obstacles_cvs, false, "Render gaussian blur results.");

namespace qcraft {
namespace {

constexpr float kGaussianBlurFarDist = 80.f;  // m

void MaybeRenderClustersCvs(const ClusterVector& clusters, double pose_z) {
  if (!FLAGS_obstacle_clusterer_cvs) return;
  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_clusterer_cvs");
  for (const auto& cluster : clusters) {
    const auto cluster_contour =
        cluster_util::ComputeContourWithZ(cluster, pose_z);
    canvas.DrawPolygon(cluster_contour, vis::Color::kDarkYellow, 1);
  }
}

void MaybeRenderObstaclesAfterGaussianBlurCvs(
    const cv::Mat& obstacle_image, const cv::Mat& threshold_image,
    const ObstaclePtrs& obstacles_in_grid,
    const ObstacleRCCoordConverter& rc_coord_converter,
    const VehiclePose& pose) {
  if (!FLAGS_blured_obstacles_cvs) return;

  vis::Canvas& canvas =
      vantage_client_man::GetCanvas("perception/obstacle_clusterer_cvs");
  for (int row = 0; row < obstacle_image.rows; ++row) {
    for (int col = 0; col < obstacle_image.cols; ++col) {
      const uchar value = obstacle_image.at<uchar>(row, col);
      if (value <= threshold_image.at<uchar>(row, col)) {
        continue;
      }
      vis::Color color(value * (1 / 255.f), value * (1 / 255.f),
                       value * (1 / 255.f));
      color.a() = 0.7;
      const Vec2d coord = rc_coord_converter.RCToCoord({row, col});
      canvas.DrawBox(Vec3d(coord, pose.z), 0.0,
                     {Obstacle::kDiameter, Obstacle::kDiameter},
                     vis::Color(0.0, 0.0, 0.0, 0.0), color);
      canvas.DrawText(absl::StrFormat("%d", value), Vec3d(coord, pose.z + 0.1),
                      1.0, 0.03, vis::Color::kSkyBlue);
    }
  }
}

}  // namespace

ObstacleClusterer::ObstacleClusterer(int width, int height,
                                     ThreadPool* thread_pool)
    : width_(width),
      height_(height),
      obstacles_in_grid_(width * height),
      obstacle_image_(width, height, CV_8UC1, cv::Scalar(0)),
      threshold_image_(width, height, CV_8UC1, cv::Scalar(0)),
      thread_pool_(thread_pool) {
  for (int i = 0; i < height; ++i) {
    uchar* row_data = threshold_image_.ptr<uchar>(i);
    for (int j = 0; j < width; ++j) {
      const float dist = Hypot<float>(i - height / 2, j - width / 2) *
                         static_cast<float>(Obstacle::kDiameter);
      const float threshold = dist < kGaussianBlurFarDist
                                  ? std::clamp(15.0f - dist * 0.3f, 5.0f, 15.0f)
                                  : std::clamp(13.0f - dist * 0.1f, 1.0f, 5.0f);
      row_data[j] = RoundToInt(threshold);
    }
  }
}

ClusterVector ObstacleClusterer::ClusterObstacles(
    const ObstaclePtrs& obstacles,
    const ObstacleRCCoordConverter& rc_coord_converter, const VehiclePose& pose,
    const FloodFilling flood_filling) {
  SCOPED_QTRACE_ARG1("ObstacleClusterer::ClusterObstacles", "num_obstacles",
                     obstacles.size());
  // Wait for the resetting done in the last iteration.
  WaitForFuture(reset_future_);

  const int width = width_;
  const int height = height_;

  // Create an image and set the pixel for each obstacle to 255.
  auto* obstacle_image_ptr = obstacle_image_.data;

  // Find the subregion that include all obstacles.
  bool contains_onroad_obstacle = false;
  uint16_t min_row = std::numeric_limits<uint16_t>::max();
  uint16_t max_row = std::numeric_limits<uint16_t>::min();
  uint16_t min_col = std::numeric_limits<uint16_t>::max();
  uint16_t max_col = std::numeric_limits<uint16_t>::min();
  for (const auto* obstacle : obstacles) {
    // Don't use ignored and offroad obstacles.
    if (obstacle->row < height && obstacle->col < width &&
        obstacle->type != ObstacleProto::IGNORED &&
        obstacle->type != ObstacleProto::OFFROAD) {
      // Note the padding on obstacle_image.
      const int row = obstacle->row;
      const int col = obstacle->col;
      const int index = row * width + col;
      *(obstacle_image_ptr + index) = 0xFF;
      obstacles_in_grid_[index] = obstacle;
      contains_onroad_obstacle = true;
    }
    min_row = std::min(min_row, obstacle->row);
    max_row = std::max(max_row, obstacle->row);
    min_col = std::min(min_col, obstacle->col);
    max_col = std::max(max_col, obstacle->col);
  }

  if (!contains_onroad_obstacle) return {};

  {
    // Gaussian-blur the obstacle image.
    SCOPED_QTRACE("ObstacleClusterer::GaussianBlur");
    cv::Mat obstacle_image_subregion = obstacle_image_(cv::Rect(
        min_col, min_row, max_col - min_col + 1, max_row - min_row + 1));
    cv::GaussianBlur(obstacle_image_subregion, obstacle_image_subregion, {7, 7},
                     0.0);
    MaybeRenderObstaclesAfterGaussianBlurCvs(obstacle_image_, threshold_image_,
                                             obstacles_in_grid_,
                                             rc_coord_converter, pose);
  }

  ParallelFor(0, height, thread_pool_, [&](int i) {
    uchar* row_data = obstacle_image_.ptr<uchar>(i);
    const uchar* thres_row_data = threshold_image_.ptr<uchar>(i);

    int j = 0;
#if defined(__SSE2__)
    for (; j + 16 < width; j += 16) {
      const auto data_vec =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(row_data + j));
      const auto thres_vec =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(thres_row_data + j));
      const auto max_vec = _mm_max_epu8(data_vec, thres_vec);
      const auto mask_vec = _mm_cmpeq_epi8(data_vec, max_vec);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(row_data + j), mask_vec);
    }
    // __SSE2__
#elif defined(__ARM_NEON)
    for (; j + 16 < width; j += 16) {
      // Loads 16 vector-elements
      const uint8x16_t data_vec = vld1q_u8(row_data + j);
      const uint8x16_t thres_vec = vld1q_u8(thres_row_data + j);
      // Compare each element in data_vec to the one in thres_vec.
      // If data_vec[i] is greater than or equal to thres_vec[i],
      //   set mask_vec[i] 0xFF, otherwise set it 0x00.
      // NOTE(sweif): Use vcgeq_u8 (greater than or equal to) to match the SSE
      // impl above, rather than vcgtq_u8 (greater than) as the non-simd impl
      // below.
      const uint8x16_t mask_vec = vcgeq_u8(data_vec, thres_vec);
      vst1q_u8(row_data + j, mask_vec);
    }
#endif  // __ARM_NEON
    for (; j < width; ++j) {
      uchar& pixel = row_data[j];
      pixel = pixel > thres_row_data[j] ? 255 : 0;
    }
  });

  ClusterVector clusters;
  if (flood_filling == FloodFilling::kScanline) {
    ScanlineFloodFilling(&clusters);
  } else {
    NaiveFloodFilling(obstacles, &clusters);
  }

  MaybeRenderClustersCvs(clusters, pose.z);

  reset_future_ = ScheduleFuture(ThreadPool::DisposalPool(), [&] {
    SCOPED_QTRACE("ObstacleClusterer::ClusterObstacles_reset");
    obstacles_in_grid_.assign(obstacles_in_grid_.size(), nullptr);
    memset(obstacle_image_.data, 0, width_ * height_);
  });

  return clusters;
}

void ObstacleClusterer::NaiveFloodFilling(const ObstaclePtrs& obstacles,
                                          ClusterVector* clusters) const {
  SCOPED_QTRACE("ClustererLoop::NaiveFloodFilling");

  const auto height = height_;
  const auto width = width_;

  auto* obstacle_image_ptr = obstacle_image_.data;
  std::vector<std::pair<int, int>> queue;
  for (const auto* obstacle : obstacles) {
    const int row = obstacle->row;
    const int col = obstacle->col;
    const int index = row * width + col;
    // NOTE(dong): Should skip OFFROAD or IGNORED obstacles.
    if (obstacles_in_grid_[index] == nullptr) continue;
    if (std::exchange(obstacle_image_ptr[index], 0) == 0) continue;

    ObstaclePtrs obstacles_in_cluster;
    obstacles_in_cluster.push_back(obstacle);

    // Find connected component.
    queue.clear();
    queue.emplace_back(row, col);
    while (!queue.empty()) {
      int r, c;
      std::tie(r, c) = queue.back();
      queue.pop_back();

      for (int y = std::max(0, r - 1); y <= std::min(height - 1, r + 1); ++y) {
        for (int x = std::max(0, c - 1); x <= std::min(width - 1, c + 1); ++x) {
          if (std::exchange(obstacle_image_ptr[y * width + x], 0) == 0) {
            continue;
          }
          queue.emplace_back(y, x);
          if (const auto* obstacle = obstacles_in_grid_[y * width + x]) {
            obstacles_in_cluster.push_back(obstacle);
          }
        }
      }
    }

    clusters->emplace_back(std::move(obstacles_in_cluster));
  }
}

void ObstacleClusterer::ScanlineFloodFilling(ClusterVector* clusters) const {
  SCOPED_QTRACE("ClustererLoop::ScanlineFloodFilling");

  struct ScanLine {
    int16_t min_x;
    int16_t max_x;
    int16_t y;
    bool span_up : 1;
    bool span_down : 1;
    bool span_left : 1;
    bool span_right : 1;
  };
  static_assert(sizeof(ScanLine) == 8);

  const auto height = height_;
  const auto width = width_;

  auto* obstacle_image_ptr = obstacle_image_.data;
  std::vector<ScanLine> positions_stack;
  positions_stack.reserve(width);
  for (int i = 0; i < height; ++i) {
    const int offset = i * width;
    for (int j = 0; j < width; ++j) {
      if (*(obstacle_image_ptr + offset + j) == 0) continue;

      positions_stack.clear();
      ObstaclePtrs obstacles_in_cluster;
      if (const auto* obstacle = obstacles_in_grid_[offset + j]) {
        obstacles_in_cluster.push_back(obstacle);
      }
      *(obstacle_image_ptr + offset + j) = 0;

      positions_stack.push_back(
          {static_cast<int16_t>(j), static_cast<int16_t>(j),
           static_cast<int16_t>(i), false, false, true, true});
      while (!positions_stack.empty()) {
        const auto [init_min_x, init_max_x, y, span_up, span_down, span_left,
                    span_right] = positions_stack.back();
        positions_stack.pop_back();

        auto min_x = init_min_x;
        auto max_x = init_max_x;
        if (span_left) {
          while (min_x > 0 &&
                 *(obstacle_image_ptr + y * width + min_x - 1) > 0) {
            --min_x;
            *(obstacle_image_ptr + y * width + min_x) = 0;
            if (const auto* obstacle = obstacles_in_grid_[y * width + min_x]) {
              obstacles_in_cluster.push_back(obstacle);
            }
          }
        }
        if (span_right) {
          while (max_x < width - 1 &&
                 *(obstacle_image_ptr + y * width + max_x + 1) > 0) {
            ++max_x;
            *(obstacle_image_ptr + y * width + max_x) = 0;
            if (const auto* obstacle = obstacles_in_grid_[y * width + max_x]) {
              obstacles_in_cluster.push_back(obstacle);
            }
          }
        }

        if (min_x > 0) --min_x;
        if (max_x < width - 1) ++max_x;

        const auto add_next_line = [&](const int16_t y, const bool is_next,
                                       const bool is_down,
                                       const int16_t init_min_x,
                                       const int16_t init_max_x,
                                       const int16_t min_x, const int16_t max_x,
                                       const int width) {
          int16_t cur_min_x = min_x;
          bool in_range = false;
          int16_t x = min_x;
          for (; x <= max_x; ++x) {
            const int index = y * width + x;
            const bool empty = (is_next || x < init_min_x || x > init_max_x) &&
                               *(obstacle_image_ptr + index);
            if (!in_range && empty) {
              cur_min_x = x;
              in_range = true;
            } else if (in_range && !empty) {
              positions_stack.push_back({cur_min_x, static_cast<int16_t>(x - 1),
                                         y, !is_down, is_down,
                                         min_x == cur_min_x, false});
              in_range = false;
            }
            if (in_range) {
              *(obstacle_image_ptr + index) = 0;
              if (const auto* obstacle = obstacles_in_grid_[index]) {
                obstacles_in_cluster.push_back(obstacle);
              }
            }
            if (!is_next && x == init_min_x) {
              x = init_max_x;
            }
          }
          if (in_range) {
            positions_stack.push_back({cur_min_x, static_cast<int16_t>(x - 1),
                                       y, !is_down, is_down, min_x == cur_min_x,
                                       true});
          }
        };

        if (y < height - 1) {
          add_next_line(y + 1, !span_up, true, init_min_x, init_max_x, min_x,
                        max_x, width);
        }
        if (y > 0) {
          add_next_line(y - 1, !span_down, false, init_min_x, init_max_x, min_x,
                        max_x, width);
        }
      }

      if (!obstacles_in_cluster.empty()) {
        clusters->emplace_back(std::move(obstacles_in_cluster));
      }
    }
  }
}

}  // namespace qcraft
