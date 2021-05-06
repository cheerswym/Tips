#include "onboard/math/geometry/polyline2d.h"

#include <algorithm>
#include <utility>

#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {

// Polyline2d
Polyline2d::Polyline2d(std::vector<Vec2d> points)
    : points_(std::move(points)), aabb_(points_.front(), points_.back()) {
  QCHECK(!points_.empty());
  QCHECK_GT(points_.size(), 1);

  point_s_.reserve(points_.size() + 1);
  point_s_.push_back(0);
  for (int i = 0; i + 1 < points_.size(); ++i) {
    point_s_.push_back(points_[i].DistanceTo(points_[i + 1]) + point_s_.back());
  }
}

Polyline2d::Polyline2d(std::vector<Vec2d> points, std::vector<double> point_s)
    : points_(std::move(points)),
      point_s_(std::move(point_s)),
      aabb_(points_.front(), points_.back()) {
  QCHECK(!points_.empty());
  QCHECK_GT(points_.size(), 1);
  QCHECK_EQ(point_s_.size(), points_.size());
  QCHECK_EQ(point_s_[0], 0.0);
}

Vec2d Polyline2d::Sample(double s) const {
  const int i = GetSegmentIndexFromS(s);
  const double alpha = (s - point_s_[i]) / (point_s_[i + 1] - point_s_[i]);
  QCHECK_GE(alpha, 0.0);
  QCHECK_LE(alpha, 1.0);

  return Lerp(points_[i], points_[i + 1], alpha);
}

std::vector<Vec2d> Polyline2d::Sample(const std::vector<double>& s) const {
  // Sanity checks.
  QCHECK(!s.empty());
  for (int i = 0; i < s.size(); ++i) {
    QCHECK_GE(s[i], 0.0);
    QCHECK_LE(s[i], point_s_.back());
    if (i > 0) QCHECK_GT(s[i], s[i - 1]);
  }

  std::vector<Vec2d> res;
  res.reserve(s.size());
  int index = 0;
  for (int i = 0; i < s.size(); ++i) {
    if (i == 0) {
      index = GetSegmentIndexFromS(s[i]);
    } else {
      while (s[i] >= point_s_[index + 1] && index + 1 < point_s_.size()) {
        index++;
      }
    }
    const double alpha =
        (s[i] - point_s_[index]) / (point_s_[index + 1] - point_s_[index]);
    res.push_back(Lerp(points_[index], points_[index + 1], alpha));
  }

  return res;
}

Vec2d Polyline2d::SampleTangent(double s) const {
  const int i = GetSegmentIndexFromS(s);

  return (points_[i + 1] - points_[i]).normalized();
}
std::vector<Vec2d> Polyline2d::SampleTangent(
    const std::vector<double>& s) const {
  // Sanity checks.
  QCHECK(!s.empty());
  for (int i = 0; i < s.size(); ++i) {
    QCHECK_GE(s[i], 0.0);
    QCHECK_LE(s[i], point_s_.back());
    if (i > 0) QCHECK_GT(s[i], s[i - 1]);
  }

  std::vector<Vec2d> res;
  res.reserve(s.size());
  int index = 0;
  for (int i = 0; i < s.size(); ++i) {
    if (i == 0) {
      index = GetSegmentIndexFromS(s[i]);
    } else {
      while (s[i] >= point_s_[index + 1] && index + 1 < point_s_.size()) {
        index++;
      }
    }
    res.push_back((points_[i + 1] - points_[i]).normalized());
  }

  return res;
}

int Polyline2d::GetSegmentIndexFromS(double s) const {
  QCHECK_GE(s, 0.0);
  QCHECK_LE(s, point_s_.back());

  int index = std::upper_bound(point_s_.begin(), point_s_.end(), s) -
              point_s_.begin() - 1;
  if (index == point_s_.size() - 1) {
    QCHECK_EQ(s, point_s_.back());
    index -= 1;
  }

  return index;
}

// SampledPolyline2d
SampledPolyline2d::SampledPolyline2d(std::vector<Vec2d> points, double interval)
    : Polyline2d(std::move(points)), interval_(interval) {
  QCHECK_GT(interval_, 0);

  BuildSamples();
}

SampledPolyline2d::SampledPolyline2d(std::vector<Vec2d> points,
                                     std::vector<double> point_s,
                                     double interval)
    : Polyline2d(std::move(points), std::move(point_s)), interval_(interval) {
  QCHECK_GT(interval_, 0);

  BuildSamples();
}

int SampledPolyline2d::GetSampleSegmentIndexFromS(double s) const {
  QCHECK_GE(s, 0.0);
  QCHECK_LE(s, sample_s_.back());

  int index = std::upper_bound(sample_s_.begin(), sample_s_.end(), s) -
              sample_s_.begin() - 1;
  if (index == sample_s_.size() - 1) {
    QCHECK_EQ(s, sample_s_.back());
    index -= 1;
  }

  return index;
}

void SampledPolyline2d::BuildSamples() {
  const int estimated_num_samples =
      static_cast<int>(point_s_.back() / interval_);
  samples_.reserve(estimated_num_samples);
  sample_s_.reserve(estimated_num_samples);
  tangents_.reserve(estimated_num_samples);

  int index = 0;
  double s = 0.0;
  while (true) {
    while (index + 1 < point_s_.size() && s > point_s_[index + 1]) {
      ++index;
    }
    if (index + 1 >= point_s_.size()) break;
    const double alpha =
        (s - point_s_[index]) / (point_s_[index + 1] - point_s_[index]);
    samples_.push_back(Lerp(points_[index], points_[index + 1], alpha));
    sample_s_.push_back(s);
    tangents_.push_back(
        Vec2d(points_[index + 1] - points_[index]).normalized());
    constexpr double kEpsilon = 0.01;
    if (s < point_s_.back() - kEpsilon && s + interval_ > point_s_.back()) {
      s = point_s_.back();
    } else {
      s += interval_;
    }
  }
  samples_.back() = points_.back();
  sample_s_.back() = point_s_.back();
}

}  // namespace qcraft
