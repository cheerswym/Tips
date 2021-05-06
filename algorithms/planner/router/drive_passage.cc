#include "onboard/planner/router/drive_passage.h"

#include <algorithm>
#include <limits>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/status_macros.h"
DEFINE_bool(planner_draw_drive_passage, false, "Draw drive passage.");

namespace qcraft::planner {
// Station
absl::StatusOr<std::pair<double, double>> Station::QueryCurbOffsetAt(
    double signed_lat) const {
  const double right_offset = boundaries_.front().lat_offset;
  const double left_offset = boundaries_.back().lat_offset;
  if (signed_lat < right_offset || signed_lat > left_offset) {
    return absl::OutOfRangeError(
        absl::StrFormat("Lateral offset %.2f out of range [%.2f, %.2f]!",
                        signed_lat, right_offset, left_offset));
  }

  return std::make_pair(right_offset - signed_lat, left_offset - signed_lat);
}

absl::StatusOr<BoundaryQueryResponse> Station::QueryEnclosingLaneBoundariesAt(
    double signed_lat) const {
  const double right_offset = boundaries_.front().lat_offset;
  const double left_offset = boundaries_.back().lat_offset;
  if (signed_lat < right_offset || signed_lat > left_offset) {
    return absl::OutOfRangeError(
        absl::StrFormat("Lateral offset %.2f out of range [%.2f, %.2f]!",
                        signed_lat, right_offset, left_offset));
  }

  const auto it = std::upper_bound(
      boundaries_.begin(), boundaries_.end(), signed_lat,
      [](double val, const auto& it) { return val < it.lat_offset; });

  OptionalBoundary right_boundary = *std::prev(it);
  right_boundary->lat_offset -= signed_lat;
  OptionalBoundary left_boundary =
      it == boundaries_.end()
          ? std::nullopt  // Only happens if signed_lat == left curb's offset.
          : OptionalBoundary(
                {.type = it->type, .lat_offset = it->lat_offset - signed_lat});

  return std::make_pair(std::move(right_boundary), std::move(left_boundary));
}

//~~~~~~~~~~~~~~~~~~ Drive Passage ~~~~~~~~~~~~~~~~

DrivePassage::DrivePassage(
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    StationVector<Station> stations, mapping::LanePath lane_path,
    mapping::LanePath extend_lane_path, double lane_path_start_s)
    : planner_semantic_map_manager_(planner_semantic_map_manager),
      stations_(std::move(stations)),
      lane_path_(std::move(lane_path)),
      extend_lane_path_(std::move(extend_lane_path)),
      beyond_lane_path_(lane_path_.back() != extend_lane_path_.back()),
      lane_path_start_s_(lane_path_start_s) {
  center_seg_inv_len_.reserve(stations_.size() - 1);
  segments_.reserve(stations_.size() - 1);
  for (const auto index : stations_.index_from(1)) {
    const StationIndex prev_index(index.value() - 1);
    center_seg_inv_len_.emplace_back(
        1.0 / stations_[prev_index].xy().DistanceTo(stations_[index].xy()));
    segments_.emplace_back(stations_[prev_index].xy(), stations_[index].xy());
  }
  segment_matcher_ = std::make_unique<SegmentMatcherKdtree>(segments_);
}

// ########## query operations ##########
absl::StatusOr<double> DrivePassage::QuerySpeedLimitAt(Vec2d point) const {
  return FindNearestStation(point).speed_limit();
}

absl::StatusOr<std::pair<double, double>> DrivePassage::QueryCurbOffsetAt(
    Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];

  return station.QueryCurbOffsetAt(projection.signed_l);
}

absl::StatusOr<std::pair<double, double>>
DrivePassage::QueryNearestBoundaryLateralOffset(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto& near_station = stations_[result.near_station_index];
  double right_offset = std::numeric_limits<double>::lowest();
  double left_offset = std::numeric_limits<double>::max();
  for (const auto& bound : near_station.boundaries()) {
    if (bound.lat_offset < 0) {
      right_offset = std::max(right_offset, bound.lat_offset);
    }
    if (bound.lat_offset > 0) {
      left_offset = std::min(left_offset, bound.lat_offset);
    }
  }
  return std::make_pair(right_offset, left_offset);
}

absl::StatusOr<std::pair<double, double>> DrivePassage::QueryCurbOffsetAtS(
    double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);
  return stations_[result.near_station_index].QueryCurbOffsetAt(
      /*signed_lat=*/0.0);
}

absl::StatusOr<std::pair<Vec2d, Vec2d>> DrivePassage::QueryCurbPointAt(
    Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];
  ASSIGN_OR_RETURN(const auto offsets,
                   station.QueryCurbOffsetAt(projection.signed_l));
  const Vec2d normal = station.tangent().Perp();

  return std::make_pair(point + normal * offsets.first,
                        point + normal * offsets.second);
}

absl::StatusOr<std::pair<Vec2d, Vec2d>> DrivePassage::QueryCurbPointAtS(
    double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto& near_station = stations_[result.near_station_index];
  ASSIGN_OR_RETURN(const auto offsets, near_station.QueryCurbOffsetAt(0.0));
  const auto& prev_station = stations_[result.station_index_1];
  const auto& succ_station = stations_[result.station_index_2];
  const double t =
      result.ds / (succ_station.accumulated_s() - prev_station.accumulated_s());
  const auto normal =
      Vec2d::FastUnitFromAngle(LerpAngle(prev_station.tangent().FastAngle(),
                                         succ_station.tangent().FastAngle(), t))
          .Perp();
  const auto center = Lerp(prev_station.xy(), succ_station.xy(), t);
  return std::make_pair(center + normal * offsets.first,
                        center + normal * offsets.second);
}

absl::StatusOr<BoundaryQueryResponse>
DrivePassage::QueryEnclosingLaneBoundariesAt(Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }
  const auto& station = stations_[projection.near_station_index];

  return station.QueryEnclosingLaneBoundariesAt(projection.signed_l);
}

BoundaryQueryResponse DrivePassage::QueryEnclosingLaneBoundariesAtS(
    double s) const {
  const auto station_index = FindNearestStationIndexAtS(s);

  // Guaranteed not to exceed lateral range.
  return stations_[station_index]
      .QueryEnclosingLaneBoundariesAt(/*signed_lat=*/0.0)
      .value();
}

absl::StatusOr<Vec2d> DrivePassage::QueryLaterallyUnboundedTangentAt(
    Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  return stations_[projection.station_index_1].tangent();
}

absl::StatusOr<Vec2d> DrivePassage::QueryTangentAt(Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }

  return stations_[projection.station_index_1].tangent();
}

absl::StatusOr<Vec2d> DrivePassage::QueryTangentAtS(double s) const {
  ASSIGN_OR_RETURN(const auto angle, QueryTangentAngleAtS(s));
  return Vec2d::FastUnitFromAngle(angle);
}

absl::StatusOr<double> DrivePassage::QueryTangentAngleAtS(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);
  const auto& prev_station = stations_[result.station_index_1];
  const auto& succ_station = stations_[result.station_index_2];

  const double t =
      result.ds / (succ_station.accumulated_s() - prev_station.accumulated_s());
  return NormalizeAngle(LerpAngle(prev_station.tangent().FastAngle(),
                                  succ_station.tangent().FastAngle(), t));
}

// frenet queries
absl::StatusOr<Vec2d> DrivePassage::QueryPointXYAtS(double s) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }

  const auto result = BinarySearchForNearStation(s);

  return stations_[result.station_index_1].xy() +
         stations_[result.station_index_1].tangent() * result.ds;
}

absl::StatusOr<Vec2d> DrivePassage::QueryPointXYAtSL(double s, double l) const {
  if (s < front_s() || s > end_s()) {
    return absl::OutOfRangeError(
        absl::StrFormat("%.2f is out of accumulated_s range [%.2f, %.2f].", s,
                        front_s(), end_s()));
  }
  const auto result = BinarySearchForNearStation(s);
  const auto ref_tangent = stations_[result.station_index_1].tangent();
  const auto ref_xy = stations_[result.station_index_1].xy() +
                      stations_[result.station_index_1].tangent() * result.ds;

  const Vec2d normal = ref_tangent.Perp();

  return ref_xy + normal * l;
}

absl::StatusOr<StationWaypoint> DrivePassage::QueryFrenetLonOffsetAt(
    Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }

  return StationWaypoint{
      .station_index = projection.near_station_index,
      .lon_offset = projection.accum_s -
                    stations_[projection.near_station_index].accumulated_s(),
      .accum_s = projection.accum_s};
}

absl::StatusOr<double> DrivePassage::QueryFrenetLatOffsetAt(Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }

  return projection.signed_l;
}

absl::StatusOr<FrenetCoordinate>
DrivePassage::QueryLaterallyUnboundedFrenetCoordinateAt(Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  return FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l};
}

absl::StatusOr<FrenetCoordinate> DrivePassage::QueryUnboundedFrenetCoordinateAt(
    Vec2d point) const {
  int nearest_index = -1;
  if (segment_matcher_ == nullptr) {
    return absl::InternalError(
        "Failed to project point to stations because of null segment_matcher_ "
        "pointer.");
  }
  if (!segment_matcher_->GetNearestSegmentIndex(point.x(), point.y(),
                                                &nearest_index)) {
    return absl::NotFoundError(
        "Fail to get nearest segment index with segment_matcher.");
  }
  const auto& nearest_seg = segments_[nearest_index];
  const double prod = nearest_seg.ProductOntoUnit(point);
  const double proj =
      nearest_seg.ProjectOntoUnit(point) * center_seg_inv_len_[nearest_index];
  const auto& prev_station = stations_[StationIndex(nearest_index)];
  const auto& station = stations_[StationIndex(nearest_index + 1)];

  return FrenetCoordinate{
      .s = Lerp(prev_station.accumulated_s(), station.accumulated_s(), proj),
      .l = prod};
}

absl::StatusOr<FrenetCoordinate> DrivePassage::QueryFrenetCoordinateAt(
    Vec2d point) const {
  ASSIGN_OR_RETURN(const auto projection, ProjectPointToStations(point));
  const auto status = IsProjectionResultOnDrivePassage(projection);
  if (!status.ok()) {
    return status;
  }
  return FrenetCoordinate{.s = projection.accum_s, .l = projection.signed_l};
}

absl::StatusOr<FrenetBox> DrivePassage::QueryFrenetBoxAt(
    const Box2d& box) const {
  return QueryFrenetBoxAtContour(Polygon2d(box));
}

absl::StatusOr<FrenetBox> DrivePassage::QueryFrenetBoxAtContour(
    const Polygon2d& contour) const {
  std::vector<FrenetCoordinate> coords;
  bool beyond_front_s = false, behind_end_s = false, left_of_right_curb = false,
       right_of_left_curb = false;
  coords.reserve(contour.points().size());
  for (const auto& pt : contour.points()) {
    auto res = QueryUnboundedFrenetCoordinateAt(pt);
    if (!res.ok()) {
      return absl::NotFoundError("No frenet conversion is found.");
    }

    if (res->s > front_s()) beyond_front_s = true;
    if (res->s < end_s()) behind_end_s = true;
    res->s = std::clamp(res->s, front_s(), end_s());

    const auto [right_curb, left_curb] = QueryCurbOffsetAtS(res->s).value();
    if (res->l > right_curb) left_of_right_curb = true;
    if (res->l < left_curb) right_of_left_curb = true;
    res->l = std::clamp(res->l, right_curb, left_curb);
    coords.push_back(*res);
  }

  if (!(beyond_front_s && behind_end_s && left_of_right_curb &&
        right_of_left_curb)) {
    return absl::NotFoundError("Box has no overlap with drive passage.");
  }
  FrenetBox frenet_box{
      .s_max = -std::numeric_limits<double>::infinity(),
      .s_min = std::numeric_limits<double>::infinity(),
      .l_max = -std::numeric_limits<double>::infinity(),
      .l_min = std::numeric_limits<double>::infinity(),
  };
  for (const auto& coord : coords) {
    frenet_box.s_max = std::max(frenet_box.s_max, coord.s);
    frenet_box.s_min = std::min(frenet_box.s_min, coord.s);
    frenet_box.l_max = std::max(frenet_box.l_max, coord.l);
    frenet_box.l_min = std::min(frenet_box.l_min, coord.l);
  }

  return frenet_box;
}

absl::StatusOr<std::vector<FrenetCoordinate>>
DrivePassage::BatchQueryFrenetCoordinates(
    absl::Span<const Vec2d> points) const {
  std::vector<FrenetCoordinate> frenet_points;
  frenet_points.reserve(points.size());
  for (const auto& pt : points) {
    ASSIGN_OR_RETURN(auto frenet_pt, QueryFrenetCoordinateAt(pt));
    frenet_points.push_back(std::move(frenet_pt));
  }
  return frenet_points;
}

// ######## query operations end ########

StationIndex DrivePassage::FindNearestStationIndex(Vec2d point) const {
  auto min_dis = std::numeric_limits<double>::max();
  StationIndex min_dis_index;
  for (const auto i : stations_.index_range()) {
    const auto& station = stations_[i];
    const auto dis = point.DistanceSquareTo(station.xy());
    if (dis < min_dis) {
      min_dis = dis;
      min_dis_index = i;
    }
  }
  return min_dis_index;
}

StationIndex DrivePassage::FindNearestStationIndexAtS(double s) const {
  const auto it = std::upper_bound(stations_.begin(), stations_.end(), s,
                                   [](double val, const Station& station) {
                                     return val < station.accumulated_s();
                                   });
  if (it == stations_.begin()) return StationIndex(0);
  if (it == stations_.end()) return StationIndex(stations_.size() - 1);

  return std::fabs(it->accumulated_s() - s) <
                 std::fabs(std::prev(it)->accumulated_s() - s)
             ? StationIndex(it - stations_.begin())
             : StationIndex(it - stations_.begin() - 1);
}

absl::StatusOr<DrivePassage::ProjectionResult>
DrivePassage::ProjectPointToStations(Vec2d point) const {
  int nearest_index = -1;
  if (segment_matcher_ == nullptr) {
    return absl::InternalError(
        "Failed to project point to stations because of null segment_matcher_ "
        "pointer.");
  }
  if (!segment_matcher_->GetNearestSegmentIndex(point.x(), point.y(),
                                                &nearest_index)) {
    return absl::NotFoundError(
        "Fail to get nearest segment index with segment_matcher.");
  }
  const auto& nearest_seg = segments_[nearest_index];
  const double prod = nearest_seg.ProductOntoUnit(point);
  const double proj =
      nearest_seg.ProjectOntoUnit(point) * center_seg_inv_len_[nearest_index];
  double s, l, lerp_factor;
  const auto& prev_station = stations_[StationIndex(nearest_index)];
  const auto& station = stations_[StationIndex(nearest_index + 1)];
  if (proj < 0.0 && nearest_index > 1) {
    l = std::copysign(nearest_seg.start().DistanceTo(point), prod);
    s = prev_station.accumulated_s();
    lerp_factor = 0.0;
  } else if (proj > 1.0 && nearest_index + 1 < stations_.size()) {
    l = std::copysign(nearest_seg.end().DistanceTo(point), prod);
    s = station.accumulated_s();
    lerp_factor = 1.0;
  } else {
    l = prod;
    lerp_factor = proj;
    s = Lerp(prev_station.accumulated_s(), station.accumulated_s(),
             lerp_factor);
  }
  // Projection that is out of drive passage's range is invalid.
  constexpr double kEpsilon = 0.1;  // m.
  if (s < front_s() - kEpsilon || s > end_s() + kEpsilon) {
    return absl::NotFoundError("No valid projection is found.");
  }
  s = std::clamp(s, front_s(), end_s());
  ProjectionResult res;
  res.station_index_1 = StationIndex(nearest_index);
  res.station_index_2 = StationIndex(nearest_index + 1);
  res.accum_s = s;
  res.signed_l = l;
  res.lerp_factor = lerp_factor;
  res.near_station_index =
      res.lerp_factor < 0.5 ? res.station_index_1 : res.station_index_2;
  return res;
}

absl::Status DrivePassage::IsProjectionResultOnDrivePassage(
    const ProjectionResult& res) const {
  const auto& near_station = stations_[res.near_station_index];
  if (res.signed_l < 0.0) {
    if (res.signed_l < near_station.boundaries().front().lat_offset) {
      return absl::OutOfRangeError(
          absl::StrFormat("%f is out of right lateral range %f.", res.signed_l,
                          near_station.boundaries().front().lat_offset));
    }
  } else {
    if (res.signed_l > near_station.boundaries().back().lat_offset) {
      return absl::OutOfRangeError(
          absl::StrFormat("%f is out of left lateral range %f.", res.signed_l,
                          near_station.boundaries().back().lat_offset));
    }
  }

  return absl::OkStatus();
}

DrivePassage::BinarySeachResult DrivePassage::BinarySearchForNearStation(
    double s) const {
  const auto it = std::upper_bound(stations_.begin(), stations_.end(), s,
                                   [](double val, const Station& station) {
                                     return val < station.accumulated_s();
                                   });
  StationIndex prev_index;
  StationIndex next_index;
  if (it == stations_.begin()) {
    prev_index = StationIndex(0);
    next_index = StationIndex(1);
  } else if (it == stations_.end()) {
    prev_index = StationIndex(stations_.size() - 2);
    next_index = StationIndex(stations_.size() - 1);
  } else {
    prev_index = StationIndex(it - stations_.begin() - 1);
    next_index = StationIndex(it - stations_.begin());
  }

  const double ds1 = s - stations_[prev_index].accumulated_s();
  const double ds2 = stations_[next_index].accumulated_s() - s;

  return BinarySeachResult{.station_index_1 = prev_index,
                           .station_index_2 = next_index,
                           .near_station_index = std::abs(ds1) < std::abs(ds2)
                                                     ? prev_index
                                                     : next_index,
                           .ds = ds1};
}

}  // namespace qcraft::planner
