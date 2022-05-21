#include "onboard/perception/obstacle_semantic_manager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/math/geometry/box2d.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"

DEFINE_bool(dump_obstacle_semantic_grid_image, false,
            "Whether to dump obstacle semantic grid debug image.");
DEFINE_bool(obstacle_semantic_manager_cvs, false,
            "Whether to turn on cvs to show semantic results.");
DEFINE_bool(obstacle_semantic_manager_debug, false,
            "Whether to enable obstacle semantic manager debug mode.");

namespace qcraft {

namespace {

using SemanticTypeArray = std::array<ObstacleProto::Type, kNumSemanticTypes>;

// Probability ranges in the open interval (0, 1)
// Common value mapping table
// Probability   Log Odds
//     0.00       -inf
//     0.0001     -9.21024
//     0.001      -6.90675
//     0.01       -4.59512
//     0.05       -2.94444
//     0.10       -2.19722
//     0.15       -1.7346
//     0.20       -1.38629
//     0.25       -1.09861
//     0.30       -0.847298
//     0.35       -0.619039
//     0.40       -0.405465
//     0.45       -0.200671
//     0.50        0
//     0.55        0.200671
//     0.60        0.405465
//     0.65        0.619039
//     0.70        0.847298
//     0.75        1.09861
//     0.80        1.38629
//     0.85        1.7346
//     0.90        2.19722
//     0.95        2.94444
//     0.99        4.59512
//     0.999       6.90677
//     0.9999      9.21007
//     1.00        inf
float ProbToLogOdds(const float probability) {
  return std::log(probability / (1.f - probability));
}
float LogOddsToProb(const float log_odds) {
  return 1.f - 1.f / (std::exp(log_odds) + 1.f);
}
// Semantic grid roi
constexpr float kSemanticGridForward = 80.f;   // m
constexpr float kSemanticGridBackward = 40.f;  // m
constexpr float kSemanticGridLength =
    kSemanticGridForward + kSemanticGridBackward;
constexpr float kSemanticGridWidth = 80.f;  // m
// Probability for semantic map measurements.
constexpr float kVegetationProbIfHitBySemanticMap = 0.8f;
constexpr float kBarrierProbIfHitBySemanticMap = 0.8f;
// Probability if miss a measurement.
constexpr float kVegetationProbIfMissed = 0.3f;
constexpr float kBarrierProbIfMissed = 0.2f;
// Probability for classify obstacles.
constexpr float kMinVegetationProbToBeClassified = 0.9f;
constexpr float kMinBarrierProbToBeClassified = 0.9f;
// Min probability of occupied obstacle.
constexpr float kMinOccupiedProb = 0.05f;
// Min log odds of occupied obstacle.
static const float kMinOccupiedLogOdds = ProbToLogOdds(kMinOccupiedProb);
// Min probability of occupied obstacle that will be promoted.
constexpr float kMinOccupiedProbToBePromoted = 0.9f;
// Min log odds of occupied obstacle that will be promoted.
static const float kMinOccupiedLogOddsToBePromoted =
    ProbToLogOdds(kMinOccupiedProbToBePromoted);
//  Occupied probability if miss a measurement.
constexpr float kOccupiedProbIfMissed = 0.25f;
//  Occupied log odds if miss a measurement.
static const float kOccupiedLogOddsIfMissed =
    ProbToLogOdds(kOccupiedProbIfMissed);
// Debug
constexpr char kDefaultDebugImageDir[] = "/hosthome/osm";

// NOTE(dong): Binary bayes filter
//                      p(x|z(t))
//  l(t) = l(t-1) + log----------- - l(0)
//                     1-p(x|z(t))
//
// Usually, l(0) = log(0.5/0.5) = 0
//
// For more information:
// https://en.wikipedia.org/wiki/Occupancy_grid_mapping
// https://www.cs.cmu.edu/~16831-f14/notes/F14/16831_lecture06_agiri_dmcconac_kumarsha_nbhakta.pdf
float ComputeLogOdds(const float prev_log_odds, const float measurement) {
  return prev_log_odds + measurement;
}

std::array<int, ObstacleProto::Type_ARRAYSIZE> GetChannelMap() {
  std::array<int, ObstacleProto::Type_ARRAYSIZE> channel_map;
  std::fill(channel_map.begin(), channel_map.end(), -1);
  // Set obstacle type channels
  channel_map[ObstacleProto::VEGETATION] = 0;
  channel_map[ObstacleProto::BARRIER] = 1;
  //
  QCHECK(std::all_of(
      channel_map.begin(), channel_map.end(), [](const int channel) {
        return (channel == -1) || (channel >= 0 && channel < kNumSemanticTypes);
      }));
  return channel_map;
}

int ObstacleTypeToChannel(const ObstacleProto::Type type) {
  static auto channel_map = GetChannelMap();
  return channel_map[type];
}

SemanticTypeArray GetObstacleTypeMap() {
  SemanticTypeArray obstacle_type_map;
  const auto channel_map = GetChannelMap();
  for (int i = 0; i < channel_map.size(); ++i) {
    if (channel_map[i] == -1) continue;
    obstacle_type_map[channel_map[i]] = static_cast<ObstacleProto::Type>(i);
  }
  return obstacle_type_map;
}

ObstacleProto::Type ChannelToObstacleType(const int channel) {
  static auto obstacle_type_map = GetObstacleTypeMap();
  return obstacle_type_map[channel];
}

bool IsTargetObstacleType(const ObstacleProto::Type type) {
  return ObstacleTypeToChannel(type) >= 0;
}

enum ObstacleSemanticSource {
  kSemanticMap = 0,
  kSemanticSegmentationResults = 1,
  kPillarSemanticResults = 2,
};

LogOddsArray GetHitLogOddsMap(const ObstacleSemanticSource source) {
  LogOddsArray log_odds;
  std::fill(log_odds.begin(), log_odds.end(), 0.f);
  switch (source) {
    case kSemanticMap: {
      log_odds[ObstacleTypeToChannel(ObstacleProto::VEGETATION)] =
          ProbToLogOdds(kVegetationProbIfHitBySemanticMap);
      log_odds[ObstacleTypeToChannel(ObstacleProto::BARRIER)] =
          ProbToLogOdds(kBarrierProbIfHitBySemanticMap);
      break;
    }
    case kSemanticSegmentationResults:
    case kPillarSemanticResults:
      QLOG(FATAL) << "Should not reach here.";
  }
  return log_odds;
}
// NOTE(dong): Only support semantic map measurements by now.
float ChannelToHitLogOdds(const int channel) {
  static auto log_odds_map = GetHitLogOddsMap(kSemanticMap);
  return log_odds_map[channel];
}

float ChannelToMissLogOdds(const int channel) {
  static auto log_odds_map = [] {
    LogOddsArray log_odds;
    std::fill(log_odds.begin(), log_odds.end(), 0.f);
    log_odds[ObstacleTypeToChannel(ObstacleProto::VEGETATION)] =
        ProbToLogOdds(kVegetationProbIfMissed);
    log_odds[ObstacleTypeToChannel(ObstacleProto::BARRIER)] =
        ProbToLogOdds(kBarrierProbIfMissed);
    return log_odds;
  }();
  return log_odds_map[channel];
}

float ChannelToMinClassifyLogOdds(const int channel) {
  static auto log_odds_map = [] {
    LogOddsArray log_odds;
    std::fill(log_odds.begin(), log_odds.end(),
              std::numeric_limits<float>::max());
    log_odds[ObstacleTypeToChannel(ObstacleProto::VEGETATION)] =
        ProbToLogOdds(kMinVegetationProbToBeClassified);
    log_odds[ObstacleTypeToChannel(ObstacleProto::BARRIER)] =
        ProbToLogOdds(kMinBarrierProbToBeClassified);
    return log_odds;
  }();
  return log_odds_map[channel];
}

Box2d GetSemanticGridRoi(const VehiclePose& pose) {
  Box2d region({pose.x, pose.y}, pose.yaw, kSemanticGridLength,
               kSemanticGridWidth);
  region.Shift(Vec2d((kSemanticGridForward - kSemanticGridBackward) * 0.5, 0.0)
                   .FastRotate(pose.yaw));
  return region;
}

Vec2i ComputeRCOffset(const VehiclePose& prev_pose,
                      const VehiclePose& curr_pose) {
  return {RoundToInt(prev_pose.y * (1.f / Obstacle::kDiameter)) -
              RoundToInt(curr_pose.y * (1.f / Obstacle::kDiameter)),
          RoundToInt(prev_pose.x * (1.f / Obstacle::kDiameter)) -
              RoundToInt(curr_pose.x * (1.f / Obstacle::kDiameter))};
}

void CheckSemanticGridIsEmpty(const ObstacleSemanticGrid& semantic_grid) {
  for (int row = 0; row < semantic_grid.height(); ++row) {
    for (int col = 0; col < semantic_grid.width(); ++col) {
      const auto& info = semantic_grid(row, col);
      for (const auto& log_odds : info) {
        QCHECK_EQ(log_odds, 0.f);
      }
    }
  }
}

cv::Mat ConvertGridToCvMat(const ObstacleSemanticGrid& semantic_grid,
                           const int channel) {
  cv::Mat image(semantic_grid.height(), semantic_grid.width(), CV_8UC1);
  uint8_t* image_data = image.data;
  for (int row = 0; row < semantic_grid.height(); ++row) {
    for (int col = 0; col < semantic_grid.width(); ++col) {
      const float log_odds = semantic_grid(row, col).log_odds(channel);
      const float prob = LogOddsToProb(log_odds);
      QCHECK_LE(prob, 1.f) << log_odds;
      QCHECK_GT(prob, 0.f) << log_odds;
      *(image_data + row * semantic_grid.width() + col) =
          static_cast<uint8_t>(prob * 255.f);
    }
  }
  return image;
}

std::pair<int, float> GetMaxProbChannel(
    const ObstacleSemanticGrid::GridInfo& info) {
  std::pair<int, float> max_prob_channel = {0, info.log_odds(0)};
  for (int i = 1; i < kNumSemanticTypes; ++i) {
    if (info.log_odds(i) > max_prob_channel.second) {
      max_prob_channel = {i, info.log_odds(i)};
    }
  }
  return max_prob_channel;
}

}  // namespace

ObstacleSemanticManager::ObstacleSemanticManager(const int width,
                                                 const int height)
    : prev_grid_(width, height), curr_grid_(width, height) {}

void ObstacleSemanticManager::ComputeCurrentGrid(
    const VehiclePose& pose, const double timestamp,
    const ObstaclePtrs& obstacles) {
  SCOPED_QTRACE("ObstacleSemanticManager::ComputeCurrentGrid");

  curr_grid_.set_pose(pose);
  curr_grid_.set_timestamp(timestamp);

  UpdateCurrentGridWithSemanticMap(obstacles);
}

void ObstacleSemanticManager::UpdateCurrentGridWithSemanticMap(
    const ObstaclePtrs& obstacles) {
  SCOPED_QTRACE_ARG1(
      "ObstacleSemanticManager::UpdateCurrentGridWithSemanticMap",
      "num_obstacles", obstacles.size());

  const auto roi = GetSemanticGridRoi(curr_grid_.pose());
  for (const auto* obstacle : obstacles) {
    if (!IsTargetObstacleType(obstacle->type)) continue;
    if (!roi.IsPointIn(obstacle->coord())) continue;
    auto& info = curr_grid_(obstacle->row, obstacle->col);
    const int channel = ObstacleTypeToChannel(obstacle->type);
    info.set_log_odds(channel, ComputeLogOdds(info.log_odds(channel),
                                              ChannelToHitLogOdds(channel)));
    info.set_occupied_log_odds(
        ComputeLogOdds(info.occupied_log_odds(), ChannelToHitLogOdds(channel)));
    QCHECK(curr_grid_.add_observed_info(obstacle->row, obstacle->col));
  }

  if (FLAGS_dump_obstacle_semantic_grid_image) {
    for (int i = 0; i < kNumSemanticTypes; ++i) {
      const std::string type_name =
          ObstacleProto::Type_Name(ChannelToObstacleType(i));
      const cv::Mat debug_img = ConvertGridToCvMat(curr_grid_, i);
      static int indices[kNumSemanticTypes] = {};
      cv::imwrite(
          absl::StrFormat("%s/%d_%s_semantic_map.jpg", kDefaultDebugImageDir,
                          indices[i]++, type_name),
          debug_img);
    }
  }
}

void ObstacleSemanticManager::PredictAndUpdate(
    const ObstacleManager& obstacle_manager) {
  SCOPED_QTRACE_ARG2(
      "ObstacleSemanticManager::PredictAndUpdate", "num_current_observed_infos",
      curr_grid_.observed_infos().size(), "num_previous_observed_infos",
      prev_grid_.observed_infos().size());
  // Check timestamp rollback
  if (curr_grid_.timestamp() < prev_grid_.timestamp()) {
    QLOG(ERROR) << absl::StrFormat(
        "Current obstacles has timestamp %.4f earlier than the previous one "
        "%.4f. Obstacle semantic history discarded.",
        curr_grid_.timestamp(), prev_grid_.timestamp());
    return;
  }
  // Predict previous grid and update current grid.
  const auto roi = GetSemanticGridRoi(curr_grid_.pose());
  const auto rc_offset = ComputeRCOffset(prev_grid_.pose(), curr_grid_.pose());
  for (const auto& prev_rc : prev_grid_.observed_infos()) {
    const Vec2i curr_rc = prev_rc + rc_offset;
    if (!roi.IsPointIn(obstacle_manager.RCToCoord(curr_rc))) {
      continue;
    }
    const auto& prev_info = prev_grid_(prev_rc);
    auto& curr_info = curr_grid_(curr_rc);
    // Update occupied log odds.
    // Set a minimum occupied log odds to prevent reducing an occluded grid's
    // probability to too small.
    const bool is_occupied = curr_grid_(curr_rc).is_occupied();
    QCHECK_EQ(curr_grid_.is_observed(curr_rc), is_occupied);
    const float log_odds = is_occupied
                               ? ComputeLogOdds(prev_info.occupied_log_odds(),
                                                curr_info.occupied_log_odds())
                               : ComputeLogOdds(prev_info.occupied_log_odds(),
                                                kOccupiedLogOddsIfMissed);
    curr_info.set_occupied_log_odds(std::max(log_odds, kMinOccupiedLogOdds));
    // Update log odds for each semantic type.
    for (int i = 0; i < kNumSemanticTypes; ++i) {
      if (curr_info.has_measurement(i)) {
        curr_info.set_log_odds(
            i, ComputeLogOdds(prev_info.log_odds(i), curr_info.log_odds(i)));
      } else {
        curr_info.set_log_odds(
            i, ComputeLogOdds(prev_info.log_odds(i), ChannelToMissLogOdds(i)));
      }
    }
    // Collect missed grid.
    curr_grid_.add_observed_info(curr_rc);

    if (FLAGS_obstacle_semantic_manager_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/obstacle_semantic_manager");
      const auto coord = obstacle_manager.RCToCoord(curr_rc);
      canvas.DrawBox({coord, curr_grid_.pose().z + 0.4}, 0., {0.2, 0.2},
                     vis::Color::kSkyBlue);
      canvas.DrawText(
          absl::StrFormat("Cr Occupied LO %.2f Pb %.2f",
                          curr_info.occupied_log_odds(),
                          LogOddsToProb(curr_info.occupied_log_odds())),
          {coord + Vec2d(-0.1, 0.06), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("Cr %s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(0)),
                          curr_info.log_odds(0),
                          LogOddsToProb(curr_info.log_odds(0))),
          {coord + Vec2d(-0.1, 0.04), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("Cr %s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(1)),
                          curr_info.log_odds(1),
                          LogOddsToProb(curr_info.log_odds(1))),
          {coord + Vec2d(-0.1, 0.02), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("Pr Occupied LO %.2f Pb %.2f",
                          prev_info.occupied_log_odds(),
                          LogOddsToProb(prev_info.occupied_log_odds())),
          {coord + Vec2d(-0.1, -0.02), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("Pr %s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(0)),
                          prev_info.log_odds(0),
                          LogOddsToProb(prev_info.log_odds(0))),
          {coord + Vec2d(-0.1, -0.04), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("Pr %s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(1)),
                          prev_info.log_odds(1),
                          LogOddsToProb(prev_info.log_odds(1))),
          {coord + Vec2d(-0.1, -0.06), curr_grid_.pose().z + 0.4}, 0., 0.012,
          vis::Color::kWhite);
    }
  }

  if (FLAGS_dump_obstacle_semantic_grid_image) {
    for (int i = 0; i < kNumSemanticTypes; ++i) {
      const std::string type_name =
          ObstacleProto::Type_Name(ChannelToObstacleType(i));
      const cv::Mat debug_img = ConvertGridToCvMat(curr_grid_, i);
      static int indices[kNumSemanticTypes] = {};
      cv::imwrite(absl::StrFormat("%s/%d_%s_updated.jpg", kDefaultDebugImageDir,
                                  indices[i]++, type_name),
                  debug_img);
    }
  }
}

void ObstacleSemanticManager::ClassifyBySemanticGrid(
    ObstacleManager* obstacle_manager) {
  SCOPED_QTRACE_ARG1("ObstacleSemanticManager::ClassifyBySemanticGrid",
                     "num_current_observed_infos",
                     curr_grid_.observed_infos().size());
  // For now, we only classify barrier and vegetation obstacles.
  // TODO(dong, yu): Change this to a more general solution.
  for (const auto& rc : curr_grid_.observed_infos()) {
    const auto [row, col] = std::make_tuple(rc.x(), rc.y());
    auto* obstacle = obstacle_manager->mutable_obstacle(row, col);
    if (!obstacle || obstacle->type == ObstacleProto::OFFROAD) {
      continue;
    }
    // Classify and promote obstacle.
    const auto& info = curr_grid_(row, col);
    const auto [channel, log_odds] = GetMaxProbChannel(info);
    const float min_log_odds_to_be_classified =
        ChannelToMinClassifyLogOdds(channel);
    if (log_odds >= min_log_odds_to_be_classified) {
      obstacle->type = ChannelToObstacleType(channel);
      obstacle->type_source = ObstacleProto::OBSTACLE_SEMANTIC_MANAGER;
    } else if (IsTargetObstacleType(obstacle->type) &&
               info.occupied_log_odds() < kMinOccupiedLogOddsToBePromoted) {
      obstacle->type = ObstacleProto::IGNORED;
      obstacle->type_source = ObstacleProto::OBSTACLE_SEMANTIC_MANAGER;
    }

    if (FLAGS_obstacle_semantic_manager_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/obstacle_semantic_manager");
      canvas.DrawBox({obstacle->coord(), obstacle->ground_z}, 0., {0.2, 0.2},
                     vis::Color::kTransparent, vis::Color(0.0, 0.0, 0.5, 0.2));
      canvas.DrawText(
          absl::StrFormat("%s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(0)),
                          info.log_odds(0), LogOddsToProb(info.log_odds(0))),
          {obstacle->x - 0.1, obstacle->y + 0.02, obstacle->ground_z}, 0.,
          0.012, vis::Color::kWhite);
      canvas.DrawText(
          absl::StrFormat("%s LO %.2f Pb %.2f",
                          ObstacleProto::Type_Name(ChannelToObstacleType(1)),
                          info.log_odds(1), LogOddsToProb(info.log_odds(1))),
          {obstacle->x - 0.1, obstacle->y - 0.02, obstacle->ground_z}, 0.,
          0.012, vis::Color::kWhite);
    }
  }
}

void ObstacleSemanticManager::CleanUpAndSwap() {
  SCOPED_QTRACE("ObstacleSemanticManager::CleanUpAndSwap");
  curr_grid_.swap(prev_grid_);
  curr_grid_.clear();

  if (FLAGS_obstacle_semantic_manager_debug) {
    CheckSemanticGridIsEmpty(curr_grid_);
  }
}

void ObstacleSemanticManager::ClassifyObstacles(
    const VehiclePose& pose, const double timestamp,
    ObstacleManager* obstacle_manager) {
  SCOPED_QTRACE_ARG1("ObstacleSemanticManager::ClassifyObstacles",
                     "num_obstacles",
                     QCHECK_NOTNULL(obstacle_manager)->obstacle_ptrs().size());
  const auto& obstacles = obstacle_manager->obstacle_ptrs();
  // Init current grid
  ComputeCurrentGrid(pose, timestamp, obstacles);
  // Temporal fusion
  PredictAndUpdate(*obstacle_manager);
  // Classify obstacles
  ClassifyBySemanticGrid(obstacle_manager);
  // Clean up and swap curr & prev resources
  CleanUpAndSwap();
}

}  // namespace qcraft
