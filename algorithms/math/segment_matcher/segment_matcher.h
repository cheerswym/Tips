#ifndef ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_
#define ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/aabox_info.h"

#include "onboard/lite/check.h"

namespace qcraft {

/**
 * @class SegmentMatcher
 * @brief The base class of segment matcher, you can use different search
 *        engine to inherit it, such as SegmentMatcherKdtree.
 */
class SegmentMatcher {
 public:
  /**
   * @brief Constructor which takes a vector of Vec2d.
   * @param segments The points to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(const std::vector<Vec2d>& points);

  /**
   * @brief Constructor which takes a vector of segments.
   * @param segments The segments to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(std::vector<Segment2d> segments);

  /**
   * @brief Constructor which takes a vector of Segments with id.
   * @param named_segments The named_segments to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(
      const std::vector<std::pair<std::string, Segment2d>>& named_segments);

  virtual ~SegmentMatcher() = default;

  const Segment2d* GetSegmentByIndex(int index) const;

  const Segment2d* GetSegmentById(const std::string& id) const;

  const std::vector<Segment2d>& segments() const { return segment_; }

  /**
   * @brief Convert xy coordinates to sl coordinates. Do not use this function
   *        when constructing with discretized segments
   * @param is_clamp If is_clamp is true, do not support linear extension
   *        interpolation.
   * @param accumulated_s The longitudinal accumulated distance.
   * @param lateral The lateral distance.
   * @return Whether the conversion is successful.
   */
  bool GetProjection(double x, double y, bool is_clamp,
                     double* const accumulated_s, double* const lateral) const;

  /**
   * @brief Convert xy coordinates to sl coordinates. Do not use this function
   *        when constructing with discretized segments and do not
   *        support linear extension interpolation.
   * @param nearest_point The nearest point on the segment to the input point.
   * @param accumulated_s The longitudinal accumulated distance.
   * @param min_dist The shortest distance from points on the segment
   *         to the input point.
   * @param segment The segment on the lane.
   * @return Whether the conversion is successful.
   */
  bool GetProjection(double x, double y, Vec2d* const nearest_point = nullptr,
                     double* const accumulated_s = nullptr,
                     double* const min_dist = nullptr,
                     Segment2d* const segment = nullptr) const;

  /**
   * @brief Get the nearest segment index with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @param nearest_index The index of nearest segment.
   * @return Whether the search is successful.
   */
  bool GetNearestSegmentIndexWithHeading(double x, double y, double theta,
                                         double max_radius,
                                         double max_heading_diff,
                                         int* const nearest_index) const;

  /**
   * @brief Get the nearest segment id with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @param nearest_id The id of nearest segment.
   * @return Whether the search is successful.
   */
  bool GetNearestSegmentIdWithHeading(double x, double y, double theta,
                                      double max_radius,
                                      double max_heading_diff,
                                      std::string* const nearest_id) const;

  /**
   * @brief Get segments in search radius with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @return The segments in search radius.
   */
  std::vector<std::string> GetSegmentIdInRadiusWithHeading(
      double x, double y, double theta, double max_radius,
      double max_heading_diff) const;

  /********************** Get segment id ***********************/
  virtual bool GetNearestSegmentId(double x, double y,
                                   std::string* const id) const = 0;

  virtual std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                        double r) const = 0;

  /********************** Get segment index ***********************/
  virtual bool GetNearestSegmentIndex(double x, double y,
                                      int* const index) const = 0;

  virtual std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                                   double r) const = 0;

  /********************** Get segment ***********************/
  virtual const Segment2d* GetNearestSegment(double x, double y) const = 0;

  virtual std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                           double r) const = 0;

  /*********************** Get named segment *****************/
  virtual bool GetNearestNamedSegment(double x, double y, Segment2d* const seg,
                                      std::string* const id) const = 0;

  virtual std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double radius) const = 0;

 protected:
  std::vector<AABoxInfo> aa_boxes_info_;
  std::unordered_map<std::string, const Segment2d*> segment_map_;
  std::vector<Segment2d> segment_;
  bool index_flag_ = false;
  bool named_flag_ = false;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_
