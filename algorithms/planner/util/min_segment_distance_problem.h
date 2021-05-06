#ifndef ONBOARD_PLANNER_UTIL_MIN_SEGMENT_DISTANCE_PROBLEM_H_
#define ONBOARD_PLANNER_UTIL_MIN_SEGMENT_DISTANCE_PROBLEM_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/util/qtfm_segment_matcher_v2.h"

namespace qcraft::planner {

// The segment distance from a point to a segment is defined by
// qcraft::Segment2d::DistanceTo, denoted as seg_dist(seg, pt).
// The min segment distance is simply the min value of the seg_dist from
// a point to a batch of segments:
//
// min_seg_dist({seg_1,seg_2,seg_3,...seg_n}, pt) =
// min(seg_dist(seg_1, pt),seg_dist(seg_2,pt),... seg_dist(seg_n,pt)).
//
// The following class defines the problem such that:
// answers the value of min_seg_dist for a given point, with
// fixed setting of segments input.
// WARNING: when seg_dist(seg, pt)> cutoff_distance, seg will be skipped.
// Please make sure provide a reasonable cutoff_distance.
class MinSegmentDistanceProblem final {
 public:
  static constexpr double kEpsilon = 1e-10;

  struct SecondOrderDerivativeType {
    double df_dx = 0.0;
    double df_dy = 0.0;
    double d2f_dx_dx = 0.0;
    double d2f_dx_dy = 0.0;
    double d2f_dy_dy = 0.0;
  };

  // TODO(huaiyuan): Use separate vector for segment and string.
  // WARNING: When use_qtfm == true, cutoff_distance can't be too large!
  // Current threshold is 100.0.
  MinSegmentDistanceProblem(
      std::vector<std::pair<std::string, Segment2d>> named_segments,
      bool use_qtfm, double cutoff_distance,
      const std::optional<std::vector<int>>& optional_start_segment_ids =
          std::nullopt)
      : named_segments_(std::move(named_segments)),
        use_qtfm_(use_qtfm),
        cutoff_distance_(cutoff_distance) {
    QCHECK_GT(cutoff_distance_, 0);
    QCHECK(!named_segments_.empty());
    if (use_qtfm_) {
      constexpr double kMaxCutoffDistance = 100.0;
      QCHECK_LE(cutoff_distance_, kMaxCutoffDistance);

      constexpr double kResolution = 0.5;
      constexpr int kDepth = 5;
      double x_min = +std::numeric_limits<double>::infinity();
      double y_min = +std::numeric_limits<double>::infinity();
      double x_max = -std::numeric_limits<double>::infinity();
      double y_max = -std::numeric_limits<double>::infinity();

      std::vector<Segment2d> segments;
      segments.reserve(named_segments_.size());
      for (const auto& named_segment : named_segments_) {
        const Segment2d& segment = named_segment.second;
        segments.push_back(segment);

        x_max = std::max(segment.max_x(), x_max);
        x_min = std::min(segment.min_x(), x_min);
        y_max = std::max(segment.max_y(), y_max);
        y_min = std::min(segment.min_y(), y_min);
      }
      QCHECK_GE(x_max, x_min);
      QCHECK_GE(y_max, y_min);

      QtfmSegmentMatcherV2::Config config{.x_min = x_min - cutoff_distance_,
                                          .x_max = x_max + cutoff_distance_,
                                          .y_min = y_min - cutoff_distance_,
                                          .y_max = y_max + cutoff_distance_,
                                          .resolution = kResolution,
                                          .qtfm_depth = kDepth,
                                          .cutoff_num_range = 3,
                                          .cutoff_max_num_candidate = 16,
                                          .cutoff_distance = cutoff_distance_};
      QCHECK(optional_start_segment_ids);
      qtfm_segment_matcher_.emplace(std::move(config), std::move(segments),
                                    *optional_start_segment_ids);
    } else {
      segment_matcher_.emplace(named_segments_);
    }
  }

  const Segment2d* GetNearestSegment(const Vec2d& point) const;

  double Evaluate(const Vec2d& point) const;

  // Segment id is unique for each segment.
  double EvaluateWithNearestSegmentId(const Vec2d& point,
                                      std::string* segment_id) const;

  // 2nd order derivative is infinite/undefined for certain points,
  // for such condition, it will use a neighbor point near the singular point.
  double EvaluateWithSecondOrderDerivatives(
      const Vec2d& point, SecondOrderDerivativeType* derivative_output) const;

  MinSegmentDistanceProblem(const MinSegmentDistanceProblem& other)
      : named_segments_(other.named_segments_),
        use_qtfm_(other.use_qtfm_),
        cutoff_distance_(other.cutoff_distance_) {
    QCHECK_GT(cutoff_distance_, 0);
    QCHECK(!named_segments_.empty());
    if (use_qtfm_) {
      QCHECK(other.qtfm_segment_matcher_);
      qtfm_segment_matcher_.emplace(*other.qtfm_segment_matcher_);
    } else {
      segment_matcher_.emplace(named_segments_);
    }
  }

  double cutoff_distance() const { return cutoff_distance_; }

  void DrawQtfmSegmentMatcher(vis::Canvas* canvas) {
    if (canvas == nullptr) {
      return;
    }

    if (qtfm_segment_matcher_) {
      qtfm_segment_matcher_->DrawFieldMap(canvas);
      qtfm_segment_matcher_->DrawSegments(canvas);
    }
  }

 private:
  double DistanceToCenterWithDerivative(
      const Vec2d& point, const Vec2d& center,
      SecondOrderDerivativeType* derivative) const;

  double DistanceToLineWithDerivative(
      const Vec2d& point, const Segment2d& segment_on_line,
      SecondOrderDerivativeType* derivative) const;

  std::vector<std::pair<std::string, Segment2d>> named_segments_;
  bool use_qtfm_ = false;

  double cutoff_distance_ = 0.0;

  std::optional<SegmentMatcherKdtree> segment_matcher_;
  std::optional<QtfmSegmentMatcherV2> qtfm_segment_matcher_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_MIN_SEGMENT_DISTANCE_PROBLEM_H_
