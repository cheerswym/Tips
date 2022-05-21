#include "onboard/perception/ground_removal/gradient_based_ground_remover.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_rsqrt.h"
#include "onboard/math/util.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"

DEFINE_bool(dump_gradient_image, false, "Whether to dump gradient images.");
DEFINE_bool(gradient_based_ground_remover_cvs, false,
            "Whether to render vis results.");

namespace qcraft::ground_removal {

namespace {

// Debug
constexpr char kDefaultDebugImageDir[] = "/hosthome/ground_removal";

class GradientImage {
 public:
  GradientImage(const Spin& spin, const int width, const int height)
      : spin_(spin), width_(width), height_(height), nodes_(width * height) {}

  struct Node {
    struct Gradient {
      // Get angle of gradient in radians.
      float theta() const { return std::atan2(delta_z, std::sqrt(dist_sqr)); }

      std::string DebugString() const {
        return absl::StrFormat("dz: %.2f dr: %.2f ds: %.2f grad: %.2f", delta_z,
                               delta_r, dist_sqr, gradient);
      }

      float delta_z = 0.f;
      float delta_r = 0.f;
      float dist_sqr = 0.f;
      float gradient = 0.f;  // Could be NAN if dist_sqr is 0.
    };

    constexpr explicit operator bool() const noexcept {
      return shot != nullptr;
    }

    const CalibratedReturn& last_return() const {
      QCHECK_NOTNULL(shot);
      QCHECK_NE(shot->num_returns, 0);
      return shot->calibrated_returns[shot->num_returns - 1];
    }

    int num_returns() const { return QCHECK_NOTNULL(shot)->num_returns; }

    const LaserShot* shot = nullptr;
    std::optional<Gradient> up;
    std::optional<Gradient> down;
    std::optional<Gradient> prev;
    std::optional<Gradient> next;
    uint16_t scan_index = 0;
    uint16_t beam_index = 0;
    bool is_ground = false;
  };

  const Node& operator()(int row, int col) const {
    QCHECK(row * width_ + col < nodes_.size()) << row << " " << col;
    return nodes_[row * width_ + col];
  }
  Node& operator()(int row, int col) {
    QCHECK(row * width_ + col < nodes_.size()) << row << " " << col;
    return nodes_[row * width_ + col];
  }

  int width() const { return width_; }
  int height() const { return height_; }

  LidarId lidar_id() const { return spin_.lidar_id(); }
  LidarModel lidar_type() const { return spin_.lidar_type(); }

 private:
  const Spin& spin_;
  const int width_;
  const int height_;
  std::vector<Node> nodes_;
};

bool IsTargetLidarId(const LidarId lidar_id) {
  switch (lidar_id) {
    case LDR_CENTER:
    case LDR_FRONT_LEFT:
    case LDR_FRONT_RIGHT:
      return true;
    case LDR_UNKNOWN:
    case LDR_FRONT_BLIND:
    case LDR_LEFT_BLIND:
    case LDR_RIGHT_BLIND:
    case LDR_REAR_BLIND:
    case LDR_FRONT_LEFT_BLIND:
    case LDR_FRONT_RIGHT_BLIND:
    case LDR_FRONT:
    case LDR_REAR:
    case LDR_REAR_LEFT:
    case LDR_REAR_RIGHT:
      return false;
  }
}

int NormalizeCol(const int col, const int width) {
  const int rem_col = col % width;
  return rem_col < 0        ? rem_col + width
         : rem_col >= width ? rem_col - width
                            : rem_col;
}

void DumpGradientImage(const GradientImage& gradient_image) {
  static int counter = 0;
  {
    cv::Mat up_gradient_image(gradient_image.height(), gradient_image.width(),
                              CV_8UC1);
    std::pair<float, float> up_angle_range = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::lowest()};
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node || !node.up) continue;
        up_angle_range.first = std::min(up_angle_range.first, node.up->theta());
        up_angle_range.second =
            std::max(up_angle_range.second, node.up->theta());
      }
    }
    const auto normalize_angle_to_pixel = [=](const float angle) {
      return RoundToInt((angle - up_angle_range.first) /
                        (up_angle_range.second - up_angle_range.first) * 255);
    };
    auto* image_data = up_gradient_image.data;
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node || !node.up) continue;
        *(image_data + row * gradient_image.width() + col) =
            normalize_angle_to_pixel(node.up->theta());
      }
    }
    cv::imwrite(
        absl::StrFormat("%s/%d_%s_%s_up.jpg", kDefaultDebugImageDir, counter,
                        LidarId_Name(gradient_image.lidar_id()),
                        LidarModel_Name(gradient_image.lidar_type())),
        up_gradient_image);
  }
  {
    cv::Mat next_gradient_image(gradient_image.height(), gradient_image.width(),
                                CV_8UC1);
    std::pair<float, float> next_angle_range = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::lowest()};
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node || !node.next) continue;
        next_angle_range.first =
            std::min(next_angle_range.first, node.next->theta());
        next_angle_range.second =
            std::max(next_angle_range.second, node.next->theta());
      }
    }
    const auto normalize_angle_to_pixel = [=](const float angle) {
      return RoundToInt((angle - next_angle_range.first) /
                        (next_angle_range.second - next_angle_range.first) *
                        255);
    };
    auto* image_data = next_gradient_image.data;
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node || !node.next) continue;
        *(image_data + row * gradient_image.width() + col) =
            normalize_angle_to_pixel(node.next->theta());
      }
    }
    cv::imwrite(
        absl::StrFormat("%s/%d_%s_%s_next.jpg", kDefaultDebugImageDir, counter,
                        LidarId_Name(gradient_image.lidar_id()),
                        LidarModel_Name(gradient_image.lidar_type())),
        next_gradient_image);
  }
  {
    cv::Mat intensity_image(gradient_image.height(), gradient_image.width(),
                            CV_8UC1);
    auto* image_data = intensity_image.data;
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node) continue;
        *(image_data + row * gradient_image.width() + col) =
            node.last_return().intensity;
      }
    }
    cv::imwrite(
        absl::StrFormat("%s/%d_%s_%s_intensity.jpg", kDefaultDebugImageDir,
                        counter, LidarId_Name(gradient_image.lidar_id()),
                        LidarModel_Name(gradient_image.lidar_type())),
        intensity_image);
  }
  {
    cv::Mat range_image(gradient_image.height(), gradient_image.width(),
                        CV_8UC1);
    auto* image_data = range_image.data;
    for (int row = 0; row < gradient_image.height(); ++row) {
      for (int col = 0; col < gradient_image.width(); ++col) {
        const auto& node = gradient_image(row, col);
        if (!node) continue;
        constexpr float kMaxRange = 200.f;
        *(image_data + row * gradient_image.width() + col) = std::clamp(
            RoundToInt(node.last_return().range * (1.f / kMaxRange) * 255.f), 0,
            255);
      }
    }
    cv::imwrite(
        absl::StrFormat("%s/%d_%s_%s_range.jpg", kDefaultDebugImageDir, counter,
                        LidarId_Name(gradient_image.lidar_id()),
                        LidarModel_Name(gradient_image.lidar_type())),
        range_image);
  }
  ++counter;
}

GradientImage::Node::Gradient ComputeNodeGradient(
    const GradientImage::Node& node, const GradientImage::Node& other) {
  const float delta_z = std::abs(other.last_return().z - node.last_return().z);
  const float delta_r = other.last_return().range - node.last_return().range;
  const float dist_sqr = Sqr(other.last_return().x - node.last_return().x) +
                         Sqr(other.last_return().y - node.last_return().y);
  const float gradient =
      delta_z *
      FastRsqrtEstimate(dist_sqr + std::numeric_limits<float>::epsilon());
  return {delta_z, delta_r, dist_sqr, gradient};
}

GradientImage InitGradientImage(const Spin& spin,
                                const LidarParametersProto& lidar_param) {
  SCOPED_QTRACE_ARG1("InitGradientImage", "lidar_id",
                     LidarId_Name(spin.lidar_id()));

  const auto& intrinsics = lidar_param.inherent().intrinsics();
  std::vector<int> col_offsets(spin.num_beams());
  for (int i = 0; i < spin.num_beams(); ++i) {
    col_offsets[i] = RoundToInt(intrinsics.azimuth_offsets(i) * (1.0 / 360.0) *
                                spin.num_scans());
  }
  // Initialize gradient image
  GradientImage image(spin, spin.num_scans(), spin.num_beams());
  for (int i = 0; i < spin.num_scans(); ++i) {
    const auto& scan = spin.scan(i);
    for (int j = 0; j < scan.num_shots; ++j) {
      const auto& shot = *(scan.shots + j);
      if (shot.num_returns == 0) {
        continue;
      }
      const int row = j;
      const int col = NormalizeCol(i + col_offsets[j], spin.num_scans());

      auto& node = image(row, col);
      node.shot = &shot;
      node.scan_index = i;
      node.beam_index = j;
    }
  }
  // Compute node gradient attributes
  for (int row = 0; row < image.height(); ++row) {
    for (int col = 0; col < image.width(); ++col) {
      auto& node = image(row, col);
      if (!node) continue;
      auto* prev_node = &image(row, NormalizeCol(col - 1, image.width()));
      prev_node = prev_node->shot ? prev_node : nullptr;
      if (prev_node) {
        node.prev = prev_node->next = ComputeNodeGradient(node, *prev_node);
      }
      auto* up_node = row - 1 >= 0 ? &image(row - 1, col) : nullptr;
      up_node = up_node && up_node->shot ? up_node : nullptr;
      if (up_node) {
        node.up = up_node->down = ComputeNodeGradient(node, *up_node);
      }
    }
  }

  if (FLAGS_dump_gradient_image) {
    DumpGradientImage(image);
  }

  return image;
}

std::vector<float> ComputeAdjacentScanMaxDeltaZ(const int num_beams,
                                                const bool is_seed) {
  const float kMaxNearDeltaZ = is_seed ? 0.02 : 0.03;  // m
  const float kMaxFarDeltaZ = is_seed ? 0.04 : 0.06;   // m
  std::vector<float> adjacent_scan_max_delta_z(num_beams);
  for (int i = 0; i < num_beams; ++i) {
    const float max_delta_z =
        kMaxFarDeltaZ - i * (kMaxFarDeltaZ - kMaxNearDeltaZ) / (num_beams - 1);
    adjacent_scan_max_delta_z[i] = max_delta_z;
  }
  return adjacent_scan_max_delta_z;
}

/*

Beam n-1   *        *        *        *        *
                 prev_up    up     next_up

Beam n     *        *       (*)       *        *
                  prev     node     next

Beam n+1   *        *        *        *        *
                           down

*/
void ComputeGroundSeedPoints(GradientImage* gradient_image) {
  SCOPED_QTRACE_ARG1("ComputeGroundSeedPoints", "lidar_id",
                     LidarId_Name(QCHECK_NOTNULL(gradient_image)->lidar_id()));
  auto& image = *gradient_image;
  // The larger the beam index, the closer to the vehicle.
  // Iterate from near to far in the vehicle coordinate.
  const auto adjacent_scan_max_delta_z =
      ComputeAdjacentScanMaxDeltaZ(image.height(), /*is_seed*/ true);
  constexpr float kMaxAdjacentBeamGradient = 0.10;  // theta about 6.0°
  constexpr float kMaxAdjacentScanGradient = 0.10;  // theta about 6.0°
  const float kMinAdjacentBeamDeltaR =
      IsBlindLidar(image.lidar_id()) ? 0.03 : 0.20;  // m
  for (int row = image.height() - 1; row > image.height() / 2; --row) {
    for (int col = 0; col <= image.width(); ++col) {
      auto& node = image(row, NormalizeCol(col, image.width()));
      if (!node || node.is_ground) continue;
      auto& prev = image(row, NormalizeCol(col - 1, image.width()));
      auto& next = image(row, NormalizeCol(col + 1, image.width()));
      if (!prev || !next) continue;
      if (!node.up || !prev.up || !next.up) continue;
      if (node.prev->gradient > kMaxAdjacentScanGradient ||
          node.prev->delta_z > adjacent_scan_max_delta_z[row] ||
          node.next->gradient > kMaxAdjacentScanGradient ||
          node.next->delta_z > adjacent_scan_max_delta_z[row]) {
        continue;
      }
      if (node.up->delta_r < kMinAdjacentBeamDeltaR ||
          node.up->gradient > kMaxAdjacentBeamGradient ||
          prev.up->delta_r < kMinAdjacentBeamDeltaR ||
          prev.up->gradient > kMaxAdjacentBeamGradient ||
          next.up->delta_r < kMinAdjacentBeamDeltaR ||
          next.up->gradient > kMaxAdjacentBeamGradient) {
        continue;
      }
      constexpr float kMaxAdjacentBeamSecondDerivative = 0.10;
      if (node.down && (node.down->delta_r < 0 ||
                        std::abs(node.up->gradient - node.down->gradient) >
                            kMaxAdjacentBeamSecondDerivative)) {
        continue;
      }
      node.is_ground = true;
      prev.is_ground = true;
      next.is_ground = true;
    }
  }
}

int ComputeColOffset(const int col, const int prev_col, const int width) {
  return prev_col - col > width / 2   ? col + width - prev_col
         : col - prev_col > width / 2 ? col - width - prev_col
                                      : col - prev_col;
}

[[maybe_unused]] void PropagateGroundSeedPoints(GradientImage* gradient_image) {
  SCOPED_QTRACE_ARG1("PropagateGroundSeedPoints", "lidar_id",
                     LidarId_Name(QCHECK_NOTNULL(gradient_image)->lidar_id()));
  auto& image = *gradient_image;
  // The larger the beam index, the closer to the vehicle.
  // Iterate from near to far in the vehicle coordinate.
  const auto adjacent_scan_max_delta_z =
      ComputeAdjacentScanMaxDeltaZ(image.height(), /*is_seed*/ false);
  constexpr float kMaxAdjacentBeamGradient = 0.50;  // theta about 26.5°
  constexpr float kMaxAdjacentScanGradient = 0.20;  // theta about 11.5°
  const float kMinAdjacentBeamDeltaR =
      IsBlindLidar(image.lidar_id()) ? 0.03 : 0.20;  // m
  const auto propagate = [&](const int row, const int col, int& prev_col) {
    auto& node = image(row, col);
    if (!node) {
      return;
    }
    if (prev_col == -1 && !node.is_ground) {
      return;
    }
    if (prev_col == -1 && node.is_ground) {
      prev_col = col;
      return;
    }
    constexpr int kMaxPrevNodeColOffset = 4;
    if (std::abs(ComputeColOffset(col, prev_col, image.width())) >
        kMaxPrevNodeColOffset) {
      prev_col = node.is_ground ? col : -1;
      return;
    }
    const auto& prev = image(row, prev_col);
    QCHECK(prev && node && prev.is_ground);
    if (prev.is_ground && !node.is_ground) {
      // Propagate to the current node.
      const int col_offset = ComputeColOffset(col, prev_col, image.width());
      const auto node_gradient = col_offset == 1 ? *node.prev
                                 : col_offset == -1
                                     ? *node.next
                                     : ComputeNodeGradient(node, prev);
      if (node_gradient.gradient > kMaxAdjacentScanGradient &&
          node_gradient.delta_z > adjacent_scan_max_delta_z[row]) {
        return;
      }
      if (node.down) {
        const auto& down = image(row + 1, col);
        if (node.down->delta_r < kMinAdjacentBeamDeltaR) {
          return;
        }
        if (down.is_ground && node.down->gradient > kMaxAdjacentBeamGradient) {
          return;
        }
      }
      if (node.up && node.up->gradient > kMaxAdjacentBeamGradient) {
        return;
      }
      node.is_ground = true;
      prev_col = col;
    }
    if (node.is_ground) {
      // Propagate to up beam.
      if (!node.up) return;
      auto& up = image(row - 1, col);
      if (up.is_ground) return;
      constexpr float kMaxAdjacentBeamGradientToPropagate = 0.18;  // 10.0°
      if (node.up->gradient < kMaxAdjacentBeamGradientToPropagate &&
          node.up->delta_r > kMinAdjacentBeamDeltaR) {
        up.is_ground = true;
        return;
      }
      if (!node.down) return;
      const auto& down = image(row + 1, col);
      if (!down.is_ground) return;
      constexpr float kMaxAdjacentBeamSecondDerivative = 0.10;
      if (std::abs(node.down->gradient - node.up->gradient) <
              kMaxAdjacentBeamSecondDerivative &&
          node.up->gradient < kMaxAdjacentBeamGradient &&
          node.up->delta_r > kMinAdjacentBeamDeltaR) {
        up.is_ground = true;
        return;
      }
    }
  };
  for (int row = image.height() - 1; row >= 0; --row) {
    int prev_col = -1;
    for (int col = 0; col < image.width(); ++col) {
      propagate(row, NormalizeCol(col, image.width()), prev_col);
    }
  }
  for (int row = image.height() - 1; row >= 0; --row) {
    int prev_col = -1;
    for (int col = image.width() - 1; col >= 0; --col) {
      propagate(row, NormalizeCol(col, image.width()), prev_col);
    }
  }
}

GroundPointTable GenerateGroundPointTable(const GradientImage& image) {
  SCOPED_QTRACE_ARG1("UpdateNonLastReturnAttributes", "lidar_id",
                     LidarId_Name(image.lidar_id()));
  GroundPointTable ground_point_table(image.width(), image.height());
  for (int row = 0; row < image.height(); ++row) {
    for (int col = 0; col < image.width(); ++col) {
      const auto& node = image(row, col);
      if (!node || !node.is_ground) {
        continue;
      }
      QCHECK_NE(node.num_returns(), 0);
      if (node.num_returns() == 1) {
        ground_point_table(node.scan_index, node.beam_index, 0) = true;
        continue;
      }
      ground_point_table(node.scan_index, node.beam_index, 1) = true;
      constexpr float kMaxDeltaZOfTwoReturns = 0.05;  // m
      if (std::abs(node.shot->calibrated_returns[0].z -
                   node.shot->calibrated_returns[1].z) <
          kMaxDeltaZOfTwoReturns) {
        ground_point_table(node.scan_index, node.beam_index, 0) = true;
      }
    }
  }
  return ground_point_table;
}

GroundPointTable ComputeGroundPointTable(
    const Spin& spin, const LidarParametersProto& lidar_param) {
  SCOPED_QTRACE_ARG1("ComputeGroundPointTable", "lidar_id",
                     LidarId_Name(spin.lidar_id()));
  auto gradient_image = InitGradientImage(spin, lidar_param);
  ComputeGroundSeedPoints(&gradient_image);
  PropagateGroundSeedPoints(&gradient_image);
  return GenerateGroundPointTable(gradient_image);
}

}  // namespace

void GradientBasedGroundRemover::Compute() {
  SCOPED_QTRACE("GradientBasedGroundRemover::Compute");
  for (const auto& [lidar_id, lidar_frame] : lidar_frames_) {
    if (!lidar_frame.is_spin() || !IsTargetLidarId(lidar_id)) {
      continue;
    }
    ground_point_tables_[lidar_id] = ComputeGroundPointTable(
        *lidar_frame.spin(), FindOrDie(lidar_params_, lidar_id));
  }
}

}  // namespace qcraft::ground_removal
