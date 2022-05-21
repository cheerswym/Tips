#ifndef ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_COVARIANCE_H_
#define ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_COVARIANCE_H_

#include <map>
#include <string>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/cluster.h"
#include "onboard/perception/registration/point_matcher_structs.h"
#include "pcl/point_types.h"

namespace qcraft {

enum IcpCovMatchExit {
  INVALID = 0,
  FAIL_TOO_FEW_POINTS = 1,
  FAIL_NOT_ENOUGH_CORRESPONDENCES = 2,
  FAIL_MSE_DIVERGE = 3,
  SUCCESS_MSE_CONVERGE = 4,
  FAIL_ITERATION_REACH_LIMIT = 5,
  SUCCESS_DIFF_TRANSFORM_CONVERGE = 6,
  FAIL_NUMERAL_CALCULATIONS_FAIL = 7,
};

struct IcpCovMatchResult {
  std::string DebugString() const {
    std::map<IcpCovMatchExit, std::string> kExitDict{
        {INVALID, "Invalid"},
        {FAIL_TOO_FEW_POINTS, "[FAIL] Too few points"},
        {FAIL_NOT_ENOUGH_CORRESPONDENCES, "[FAIL] Not enough correspondences"},
        {FAIL_MSE_DIVERGE, "[FAIL] Mse diverge"},
        {SUCCESS_MSE_CONVERGE, "[SUCCESS] Mse converge"},
        {FAIL_ITERATION_REACH_LIMIT, "[FAIL] Iteration reach limit"},
        {SUCCESS_DIFF_TRANSFORM_CONVERGE,
         "[SUCCESS] Diff transformation converge"}};
    return absl::StrFormat(
        "success: %s, exit: %s, mse: %.4f, pre_mse: %.4f, "
        "num_matched_points: "
        "%d, matched_ratio: %.2f, "
        "num_iteration: %d, ",
        success ? "YES" : "NO", kExitDict[exit], mse, pre_mse,
        num_matched_points,
        static_cast<float>(num_matched_points) /
            static_cast<float>(src_num_points),
        num_iteration);
  }

  double mse = 0.0;
  double pre_mse = 0.0;
  int num_matched_points = 0;
  int src_num_points = 0;
  AffineTransformation transform;
  int num_iteration = 0;
  Eigen::Matrix6d cov = Eigen::Matrix6d::Identity();
  bool success = false;
  IcpCovMatchExit exit = INVALID;
};

class IcpWithCovariance {
 public:
  IcpWithCovariance() = default;

  IcpCovMatchResult MatchPoints(
      const pcl::PointCloud<pcl::PointNormal>::ConstPtr& ref_points,
      const pcl::PointCloud<pcl::PointNormal>::ConstPtr& src_points,
      const PointMatcherOptions& options) const;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_ICP_WITH_COVARIANCE_H_
