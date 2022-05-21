#include "onboard/planner/speed/st_boundary.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "onboard/lite/logging.h"

namespace qcraft::planner {

namespace {

void RemoveRedundantPoints(
    std::vector<std::pair<StPoint, StPoint>>* point_pairs) {
  if (!point_pairs || point_pairs->size() <= 2) return;

  constexpr double kMaxDist = 0.01;  // m.
  int i = 0;
  int j = 1;

  const auto is_point_near = [](const auto& seg, Vec2d point, double max_dist) {
    return seg.DistanceSquareTo(point) < Sqr(max_dist);
  };

  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    const Segment2d lower_seg(ToVec2d((*point_pairs)[i].first),
                              ToVec2d((*point_pairs)[j + 1].first));
    const Segment2d upper_seg(ToVec2d((*point_pairs)[i].second),
                              ToVec2d((*point_pairs)[j + 1].second));
    if (!is_point_near(lower_seg, ToVec2d((*point_pairs)[j].first), kMaxDist) ||
        !is_point_near(upper_seg, ToVec2d((*point_pairs)[j].second),
                       kMaxDist)) {
      ++i;
      if (i != j) {
        (*point_pairs)[i] = (*point_pairs)[j];
      }
    }
    ++j;
  }
  (*point_pairs)[++i] = point_pairs->back();
  point_pairs->resize(i + 1);
}

}  // namespace

StBoundary::StBoundary(std::vector<std::pair<StPoint, StPoint>> point_pairs,
                       std::vector<VtPoint> speed_points,
                       std::vector<OverlapInfo> overlap_infos,
                       StBoundaryProto::ObjectType object_type, std::string id,
                       double probability, bool is_stationary,
                       std::optional<StOverlapMetaProto> overlap_meta)
    : object_type_(object_type),
      source_type_(ObjectTypeToSourceType(object_type)),
      speed_points_(std::move(speed_points)),
      overlap_infos_(std::move(overlap_infos)),
      id_(std::move(id)),
      traj_id_(RecoverTrajId(id_, source_type_)),
      object_id_(RecoverObjectId(id_, source_type_)),
      probability_(probability),
      is_stationary_(is_stationary),
      overlap_meta_(std::move(overlap_meta)) {
  Init(std::move(point_pairs));
}

void StBoundary::Init(std::vector<std::pair<StPoint, StPoint>> point_pairs) {
  RemoveRedundantPoints(&point_pairs);

  lower_points_.clear();
  upper_points_.clear();
  lower_points_.reserve(point_pairs.size());
  upper_points_.reserve(point_pairs.size());
  points_.clear();
  points_.reserve(point_pairs.size() * 2);

  for (const auto& item : point_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (const auto& point : lower_points_) {
    points_.emplace_back(point.t(), point.s());
  }

  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
  }

  QCHECK(lower_points_.size() == upper_points().size());
  BuildFromPoints();

  for (const auto& point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }

  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}

bool StBoundary::IsValid(
    const std::vector<std::pair<StPoint, StPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    QLOG(WARNING)
        << "point_pairs.size() must > 2. current point_pairs.size() = "
        << point_pairs.size();
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  for (int i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      QLOG(WARNING) << "s is not increasing";
      return false;
    }

    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      QLOG(WARNING) << "t diff is larger in each StPoint pair";
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        QLOG(WARNING) << "t is not increasing";
        QLOG(WARNING) << " curr_lower: " << curr_lower.DebugString();
        QLOG(WARNING) << " curr_upper: " << curr_upper.DebugString();
        QLOG(WARNING) << " next_lower: " << next_lower.DebugString();
        QLOG(WARNING) << " next_upper: " << next_upper.DebugString();
        return false;
      }
    }
  }
  return true;
}

bool StBoundary::IsPointInBoundary(const StPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  int left = 0;
  int right = 0;
  if (!GetLowerPointsIndexRange(st_point.t(), &left, &right)) {
    QLOG(WARNING) << "failed to get index range.";
    return false;
  }
  QCHECK_NE(left, right);

  const auto cross_prod = [](const Vec2d& start_point, const Vec2d& end_point_1,
                             const Vec2d& end_point_2) {
    return Vec2d(end_point_1 - start_point)
        .CrossProd(Vec2d(end_point_2 - start_point));
  };

  const double check_upper =
      cross_prod(ToVec2d(st_point), ToVec2d(upper_points_[left]),
                 ToVec2d(upper_points_[right]));
  const double check_lower =
      cross_prod(ToVec2d(st_point), ToVec2d(lower_points_[left]),
                 ToVec2d(lower_points_[right]));

  return (check_upper * check_lower < 0);
}

void StBoundary::ExpandByT(double left, double right) {
  QCHECK_GE(left, 0.0);
  QCHECK_GE(right, 0.0);
  constexpr double kEps = 1e-6;
  if (left < kEps && right < kEps) {
    return;
  }

  std::vector<std::pair<StPoint, StPoint>> point_pairs;
  point_pairs.reserve(lower_points_.size() + 2);
  std::vector<VtPoint> speed_points;
  speed_points.reserve(speed_points_.size() + 2);

  if (left > 0.0 && lower_points_.front().t() > 0.0) {
    const double left_expanded_t =
        std::max(0.0, lower_points_.front().t() - left);
    const StPoint left_expanded_lower_point(lower_points_.front().s(),
                                            left_expanded_t);
    const StPoint left_expanded_upper_point(upper_points_.front().s(),
                                            left_expanded_t);
    const VtPoint left_expanded_speed_point(speed_points_.front().v(),
                                            left_expanded_t);
    point_pairs.emplace_back(left_expanded_lower_point,
                             left_expanded_upper_point);
    speed_points.push_back(left_expanded_speed_point);
  }

  // Note: Size of speed points are not necessarily equal to lower points and
  // upper points.
  for (int i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }
  for (int i = 0; i < speed_points_.size(); ++i) {
    speed_points.push_back(speed_points_[i]);
  }

  if (right > 0.0) {
    const double right_expanded_t = lower_points_.back().t() + right;
    const StPoint right_expanded_lower_point(lower_points_.back().s(),
                                             right_expanded_t);
    const StPoint right_expanded_upper_point(upper_points_.back().s(),
                                             right_expanded_t);
    const VtPoint right_expanded_speed_point(speed_points_.back().v(),
                                             right_expanded_t);
    point_pairs.emplace_back(right_expanded_lower_point,
                             right_expanded_upper_point);
    speed_points.push_back(right_expanded_speed_point);
  }

  set_speed_points(std::move(speed_points));
  Init(std::move(point_pairs));
}

StBoundary::SourceType StBoundary::ObjectTypeToSourceType(
    StBoundaryProto::ObjectType object_type) {
  switch (object_type) {
    case StBoundaryProto::UNKNOWN_OBJECT:
      return SourceType::UNKNOWN;
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN:
    case StBoundaryProto::STATIC:
    case StBoundaryProto::IGNORABLE:
      return SourceType::ST_OBJECT;
    case StBoundaryProto::VIRTUAL:
      return SourceType::VIRTUAL;
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
      return SourceType::IMPASSABLE_BOUNDARY;
  }
}

std::string StBoundary::SourceTypeName(StBoundary::SourceType source_type) {
  switch (source_type) {
    case StBoundary::SourceType::IMPASSABLE_BOUNDARY:
      return "IMPASSABLE_BOUNDARY";
    case StBoundary::SourceType::ST_OBJECT:
      return "ST_OBJECT";
    case StBoundary::SourceType::UNKNOWN:
      return "ST_OBJECT";
    case StBoundary::SourceType::VIRTUAL:
      return "VIRTUAL";
  }
}

std::optional<std::pair<double, double>> StBoundary::GetBoundarySRange(
    double curr_time) const {
  if (curr_time < min_t_ || curr_time > max_t_) {
    return std::nullopt;
  }

  int left = 0;
  int right = 0;
  if (!GetLowerPointsIndexRange(curr_time, &left, &right)) {
    QLOG(WARNING) << "Fail to get index range.";
    return std::nullopt;
  }
  QCHECK_NE(left, right);

  const double alpha = (curr_time - upper_points_[left].t()) /
                       (upper_points_[right].t() - upper_points_[left].t());

  return std::make_pair(
      upper_points_[left].s() +
          alpha * (upper_points_[right].s() - upper_points_[left].s()),
      lower_points_[left].s() +
          alpha * (lower_points_[right].s() - lower_points_[left].s()));
}

std::optional<double> StBoundary::GetStBoundarySpeedAtT(double t) const {
  if (t < min_t_ || t > max_t_) return std::nullopt;

  int left = 0;
  int right = 0;
  if (!GetSpeedPointsIndexRange(t, &left, &right)) {
    QLOG(WARNING) << "Fail to get index range.";
    return std::nullopt;
  }
  QCHECK_NE(left, right);

  const double alpha =
      LerpFactor(speed_points_[left].t(), speed_points_[right].t(), t);

  return Lerp(speed_points_[left].v(), speed_points_[right].v(), alpha);
}

StBoundaryRef StBoundary::CreateInstance(
    const StBoundaryPoints& st_boundary_points,
    StBoundaryProto::ObjectType object_type, std::string id, double probability,
    bool is_stationary) {
  QCHECK_EQ(st_boundary_points.lower_points.size(),
            st_boundary_points.upper_points.size());
  QCHECK_GE(st_boundary_points.lower_points.size(), 2);
  std::vector<std::pair<StPoint, StPoint>> point_pairs;
  point_pairs.reserve(st_boundary_points.lower_points.size());
  for (int i = 0; i < st_boundary_points.lower_points.size(); ++i) {
    point_pairs.emplace_back(st_boundary_points.lower_points[i],
                             st_boundary_points.upper_points[i]);
  }
  return StBoundaryRef(
      new StBoundary(std::move(point_pairs), st_boundary_points.speed_points,
                     st_boundary_points.overlap_infos, object_type,
                     std::move(id), probability, is_stationary,
                     /*overlap_meta=*/std::nullopt));
}

StBoundaryRef StBoundary::CopyInstance(const StBoundary& st_boundary) {
  const auto& lower_points = st_boundary.lower_points();
  const auto& upper_points = st_boundary.upper_points();

  std::vector<std::pair<StPoint, StPoint>> point_pairs;
  point_pairs.reserve(lower_points.size());
  for (int i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(lower_points[i], upper_points[i]);
  }

  return StBoundaryRef(
      new StBoundary(std::move(point_pairs), st_boundary.speed_points(),
                     st_boundary.overlap_infos(), st_boundary.object_type(),
                     st_boundary.id(), st_boundary.probability(),
                     st_boundary.is_stationary(), st_boundary.overlap_meta()));
}

std::string StBoundary::DebugString() const {
  if (IsEmpty()) {
    return "St boundary is empty.";
  }
  std::string ret = absl::StrFormat(
      "St boundary ( id : %s, min_s : %f, max_s : %f, min_t : %f, max_t : "
      "%f "
      ")\n",
      id_, min_s(), max_s(), min_t(), max_t());
  ret = absl::StrCat(ret, "upper_points: ");
  for (const auto& up : upper_points_) {
    ret = absl::StrCat(ret, up.DebugString(), ", ");
  }
  ret = absl::StrCat(ret, "\nlower_points: ");
  for (const auto& lp : lower_points_) {
    ret = absl::StrCat(ret, lp.DebugString(), ", ");
  }
  return ret;
}

void StBoundary::set_id(const std::string& id) {
  id_ = id;
  traj_id_ = RecoverTrajId(id, source_type_);
  object_id_ = RecoverObjectId(id, source_type_);
}

std::optional<std::string> StBoundary::RecoverObjectId(
    const std::string& st_boundary_id, SourceType source_type) {
  if (source_type != SourceType::ST_OBJECT) {
    return std::nullopt;
  }
  if (const auto found = st_boundary_id.find("-idx");
      found == std::string::npos) {
    QLOG(ERROR) << "St_boundary_id of st_object [" << st_boundary_id
                << "] should contain \"-idx\" but not!";
    return std::nullopt;
  } else {
    return st_boundary_id.substr(0, found);
  }
}

std::optional<std::string> StBoundary::RecoverTrajId(
    const std::string& st_boundary_id, StBoundary::SourceType source_type) {
  if (source_type != StBoundary::SourceType::ST_OBJECT) {
    return std::nullopt;
  }
  const auto it = st_boundary_id.find("|");
  if (it == std::string::npos) {
    return st_boundary_id;
  }
  return st_boundary_id.substr(0, it);
}

bool StBoundary::GetLowerPointsIndexRange(double t, int* left,
                                          int* right) const {
  return QueryIndexRange(lower_points_, t, left, right);
}

bool StBoundary::GetUpperPointsIndexRange(double t, int* left,
                                          int* right) const {
  return QueryIndexRange(upper_points_, t, left, right);
}
bool StBoundary::GetSpeedPointsIndexRange(double t, int* left,
                                          int* right) const {
  return QueryIndexRange(speed_points_, t, left, right);
}

std::string GetStBoundaryIntegrationId(const StBoundary& st_boundary) {
  if (st_boundary.source_type() == StBoundary::SourceType::ST_OBJECT) {
    const auto& object_id = st_boundary.object_id();
    QCHECK(object_id.has_value());
    return *object_id;
  }
  return st_boundary.id();
}

}  // namespace qcraft::planner
