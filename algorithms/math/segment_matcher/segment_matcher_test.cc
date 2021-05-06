#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"

#include <algorithm>
#include <limits>
#include <random>

#include "gtest/gtest.h"

#include "onboard/lite/check.h"

namespace qcraft {

TEST(SegmentMatcherTest, Kdtree) {
  const double range = 300.0;
  const int seg_num = 100;
  const int num_query = 100;
  std::vector<std::pair<std::string, Segment2d>> segments_with_id;
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> x_random(-range, range);
  std::uniform_real_distribution<double> y_random(-range, range);

  // generate segments
  for (int i = 0; i < seg_num; ++i) {
    const auto id = std::to_string(i);
    double cx = x_random(mt);
    double cy = y_random(mt);
    segments_with_id.emplace_back(
        id, Segment2d({cx - 5.0, cy - 5.0}, {cx + 5.0, cy + 5.0}));
  }

  // Construct a segment matcher
  std::unique_ptr<SegmentMatcherKdtree> kdtree_matcher =
      std::make_unique<SegmentMatcherKdtree>(segments_with_id);

  // search nearest segment
  for (int i = 0; i < num_query; ++i) {
    const Vec2d point(x_random(mt), y_random(mt));
    std::string expected_id;
    double expected_distance = std::numeric_limits<double>::infinity();
    for (int j = 0; j < segments_with_id.size(); ++j) {
      const double dis = segments_with_id[j].second.DistanceTo(point);
      if (dis < expected_distance) {
        expected_distance = dis;
        expected_id = segments_with_id[j].first;
      }
    }
    auto kdtree_object =
        kdtree_matcher->GetNearestSegment(point.x(), point.y());
    EXPECT_DOUBLE_EQ(kdtree_object->DistanceTo(point), expected_distance);
    std::string kdtree_id;
    Segment2d seg;
    std::string kdtree_named_id;
    EXPECT_TRUE(
        kdtree_matcher->GetNearestSegmentId(point.x(), point.y(), &kdtree_id));
    EXPECT_TRUE(kdtree_matcher->GetNearestNamedSegment(point.x(), point.y(),
                                                       &seg, &kdtree_named_id));

    EXPECT_EQ(kdtree_id, expected_id);
    EXPECT_EQ(kdtree_id, kdtree_named_id);
  }

  // search segments in radius
  constexpr double kRadius = 10.0;
  for (int i = 0; i < num_query; ++i) {
    Vec2d point(x_random(mt), y_random(mt));

    std::vector<std::string> ids;
    for (int j = 0; j < segments_with_id.size(); ++j) {
      if (segments_with_id[j].second.DistanceTo(point) > kRadius) {
        continue;
      }
      ids.push_back(segments_with_id[j].first);
    }

    auto kdtree_objects =
        kdtree_matcher->GetSegmentInRadius(point.x(), point.y(), kRadius);
    auto kdtree_ids =
        kdtree_matcher->GetSegmentIdInRadius(point.x(), point.y(), kRadius);
    auto kdtree_named_seg =
        kdtree_matcher->GetNamedSegmentsInRadius(point.x(), point.y(), kRadius);

    EXPECT_EQ(kdtree_objects.size(), ids.size());
    EXPECT_EQ(kdtree_ids.size(), ids.size());
    EXPECT_EQ(kdtree_named_seg.size(), ids.size());
  }
}

TEST(SegmentMatcherTest, KdtreeEmptyTests) {
  const double x = 0;
  const double y = 0;
  const double theta = 0;
  const double max_radius = 10.0;
  const double max_heading_diff = M_PI_2;
  double s = 0.0;
  double l = 0.0;
  int nearest_index = 0;

  std::vector<Segment2d> segments;
  std::unique_ptr<SegmentMatcher> matcher =
      std::make_unique<SegmentMatcherKdtree>(segments);

  bool success = matcher->GetNearestSegmentIndexWithHeading(
      x, y, theta, max_radius, max_heading_diff, &nearest_index);
  EXPECT_TRUE(!success);
  success = matcher->GetNearestSegmentIndex(x, y, &nearest_index);
  EXPECT_TRUE(!success);
  success = matcher->GetProjection(x, y, true, &s, &l);
  EXPECT_TRUE(!success);
  EXPECT_EQ(matcher->GetSegmentByIndex(0), nullptr);
  EXPECT_TRUE(matcher->GetSegmentIndexInRadius(x, y, max_radius).empty());
  EXPECT_EQ(matcher->GetNearestSegment(x, y), nullptr);
  EXPECT_TRUE(matcher->GetSegmentInRadius(x, y, max_radius).empty());

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  std::unique_ptr<SegmentMatcher> id_matcher =
      std::make_unique<SegmentMatcherKdtree>(named_segments);

  std::string nearest_id;
  success = id_matcher->GetNearestSegmentIdWithHeading(
      x, y, theta, max_radius, max_heading_diff, &nearest_id);
  EXPECT_TRUE(!success);
  success = id_matcher->GetNearestSegmentId(x, y, &nearest_id);
  EXPECT_TRUE(!success);
  EXPECT_EQ(id_matcher->GetSegmentById(nearest_id), nullptr);
  EXPECT_TRUE(id_matcher->GetSegmentIdInRadius(x, y, max_radius).empty());
  EXPECT_EQ(id_matcher->GetNearestSegment(x, y), nullptr);
  EXPECT_TRUE(id_matcher->GetSegmentInRadius(x, y, max_radius).empty());
}

}  // namespace qcraft
