#ifndef ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE_H_
#define ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/aabox_kdtree2d.h"
#include "onboard/math/segment_matcher/segment_matcher.h"

namespace qcraft {

class SegmentMatcherKdtree : public SegmentMatcher {
 public:
  explicit SegmentMatcherKdtree(std::vector<Vec2d> points);

  explicit SegmentMatcherKdtree(std::vector<Segment2d> segments);

  explicit SegmentMatcherKdtree(
      std::vector<std::pair<std::string, Segment2d>> named_segments);

  bool GetNearestSegmentId(double x, double y,
                           std::string* const id) const override;

  std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                double r) const override;

  bool GetNearestSegmentIndex(double x, double y,
                              int* const index) const override;

  std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                           double r) const override;

  const Segment2d* GetNearestSegment(double x, double y) const override;

  std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                   double r) const override;

  bool GetNearestNamedSegment(double x, double y, Segment2d* const seg,
                              std::string* const id) const override;

  std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double r) const override;

 private:
  void InitAAboxKdtree();

 private:
  std::unique_ptr<AABoxKDTree2d<AABoxInfo>> segments_tree_;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE_H_
