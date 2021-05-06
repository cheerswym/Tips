#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_

#include <memory>
#include <utility>
#include <vector>

#include "onboard/container/strong_vector.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

DECLARE_STRONG_VECTOR(Station);

struct StationCenter {
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  double fraction = 0.0;
  Vec2d xy;
  Vec2d tangent;
  double accum_s;
  double speed_limit;
  bool is_virtual;
  bool is_merging;
  bool is_in_intersection;
  mapping::LaneProto::Direction direction;

  mapping::LanePoint GetLanePoint() const {
    return mapping::LanePoint(lane_id, fraction);
  }

  Vec2d lat_point(double signed_offset) const {
    return xy + tangent.Perp() * signed_offset;
  }
  Vec2d lon_point(double signed_offset) const {
    return xy + tangent * signed_offset;
  }
  double lat_offset(Vec2d v) const { return tangent.CrossProd(v - xy); }
  double lon_offset(Vec2d v) const { return tangent.Dot(v - xy); }
};

enum StationBoundaryType {
  BROKEN_WHITE = 0,
  SOLID_WHITE = 1,
  BROKEN_YELLOW = 2,
  SOLID_YELLOW = 3,
  SOLID_DOUBLE_YELLOW = 4,
  CURB = 5,
  VIRTUAL_CURB = 6,

  UNKNOWN_TYPE = 99
};

struct StationBoundary {
  StationBoundaryType type;
  double lat_offset;

  bool IsSolid() const {
    switch (type) {
      case StationBoundaryType::SOLID_WHITE:
      case StationBoundaryType::SOLID_YELLOW:
      case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      case StationBoundaryType::CURB:
      case StationBoundaryType::VIRTUAL_CURB:
        return true;
      case StationBoundaryType::UNKNOWN_TYPE:
      case StationBoundaryType::BROKEN_WHITE:
      case StationBoundaryType::BROKEN_YELLOW:
        return false;
    }
  }
};

using OptionalBoundary = std::optional<StationBoundary>;
using BoundaryQueryResponse = std::pair<OptionalBoundary, OptionalBoundary>;

class Station {
 public:
  explicit Station(StationCenter center, std::vector<StationBoundary> bounds)
      : center_(std::move(center)), boundaries_(std::move(bounds)) {}

  const Vec2d& xy() const { return center_.xy; }
  const Vec2d& tangent() const { return center_.tangent; }
  double accumulated_s() const { return center_.accum_s; }
  double speed_limit() const { return center_.speed_limit; }
  bool is_virtual() const { return center_.is_virtual; }

  bool is_merging() const { return center_.is_merging; }
  bool is_in_intersection() const { return center_.is_in_intersection; }
  mapping::LaneProto::Direction direction() const { return center_.direction; }

  Vec2d lat_point(double signed_offset) const {
    return center_.lat_point(signed_offset);
  }
  Vec2d lon_point(double signed_offset) const {
    return center_.lon_point(signed_offset);
  }
  double lat_offset(Vec2d v) const { return center_.lat_offset(v); }
  double lon_offset(Vec2d v) const { return center_.lon_offset(v); }

  absl::Span<const StationBoundary> boundaries() const { return boundaries_; }

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      double signed_lat) const;
  // Including curb.
  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      double signed_lat) const;

  mapping::LanePoint GetLanePoint() const { return center_.GetLanePoint(); }

 private:
  // Stations are sampled on the target lane path.
  StationCenter center_;
  // Ordered by offset from right to left
  std::vector<StationBoundary> boundaries_;
};

struct StationWaypoint {
  StationIndex station_index;
  double lon_offset;
  double accum_s;
};

class DrivePassage {
 public:
  DrivePassage() : planner_semantic_map_manager_(nullptr) {}
  DrivePassage(const PlannerSemanticMapManager* planner_semantic_map_manager,
               StationVector<Station> stations, mapping::LanePath lane_path,
               mapping::LanePath extend_lane_path, double lane_path_start_s);

  DrivePassage(DrivePassage const& o)
      : planner_semantic_map_manager_(o.planner_semantic_map_manager_),
        stations_(o.stations_),
        center_seg_inv_len_(o.center_seg_inv_len_),
        lane_path_(o.lane_path_),
        extend_lane_path_(o.extend_lane_path_),
        beyond_lane_path_(o.beyond_lane_path_),
        lane_path_start_s_(o.lane_path_start_s_),
        segments_(o.segments_) {
    segment_matcher_ = std::make_unique<SegmentMatcherKdtree>(segments_);
  }

  DrivePassage& operator=(DrivePassage const& o) {
    planner_semantic_map_manager_ = o.planner_semantic_map_manager_;
    stations_ = o.stations_;
    center_seg_inv_len_ = o.center_seg_inv_len_;
    lane_path_ = o.lane_path_;
    extend_lane_path_ = o.extend_lane_path_;
    beyond_lane_path_ = o.beyond_lane_path_;
    lane_path_start_s_ = o.lane_path_start_s_;
    segments_ = o.segments_;
    segment_matcher_ = std::make_unique<SegmentMatcherKdtree>(segments_);
    return *this;
  }

  DrivePassage& operator=(DrivePassage&& o) {
    if (this != &o) {
      planner_semantic_map_manager_ =
          std::move(o.planner_semantic_map_manager_);
      stations_ = std::move(o.stations_);
      center_seg_inv_len_ = std::move(o.center_seg_inv_len_);
      lane_path_ = std::move(o.lane_path_);
      extend_lane_path_ = std::move(o.extend_lane_path_);
      beyond_lane_path_ = o.beyond_lane_path_;
      lane_path_start_s_ = o.lane_path_start_s_;
      segments_ = std::move(o.segments_);
      segment_matcher_ = std::move(o.segment_matcher_);
    }
    return *this;
  }

  // ########## query operations ##########
  // NOTE (boqian): each pair.first represents the right side and .second the
  // left side, with all lateral offsets on the right side always smaller than 0
  absl::StatusOr<double> QuerySpeedLimitAt(Vec2d point) const;

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      Vec2d point) const;
  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryNearestBoundaryLateralOffset(
      double s) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAt(Vec2d point) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAtS(double s) const;

  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      Vec2d point) const;
  BoundaryQueryResponse QueryEnclosingLaneBoundariesAtS(double s) const;

  absl::StatusOr<Vec2d> QueryLaterallyUnboundedTangentAt(Vec2d point) const;

  absl::StatusOr<Vec2d> QueryTangentAt(Vec2d point) const;
  absl::StatusOr<Vec2d> QueryTangentAtS(double s) const;
  absl::StatusOr<double> QueryTangentAngleAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtSL(double s, double l) const;

  absl::StatusOr<StationWaypoint> QueryFrenetLonOffsetAt(Vec2d point) const;

  absl::StatusOr<double> QueryFrenetLatOffsetAt(Vec2d point) const;

  absl::StatusOr<FrenetCoordinate> QueryFrenetCoordinateAt(Vec2d point) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour) const;

  // Returns projection that is not bounded to the drive passage when object is
  // not near drive passage.
  absl::StatusOr<FrenetCoordinate> QueryLaterallyUnboundedFrenetCoordinateAt(
      Vec2d point) const;

  absl::StatusOr<FrenetCoordinate> QueryUnboundedFrenetCoordinateAt(
      Vec2d point) const;

  // Query a set of points
  absl::StatusOr<std::vector<FrenetCoordinate>> BatchQueryFrenetCoordinates(
      absl::Span<const Vec2d> points) const;

  // ######## query operations end ########

  // Find the nearest station from a point.
  StationIndex FindNearestStationIndex(Vec2d point) const;
  const Station& FindNearestStation(Vec2d point) const {
    return stations_[FindNearestStationIndex(point)];
  }
  // Find the nearest station from s.
  StationIndex FindNearestStationIndexAtS(double s) const;
  const Station& FindNearestStationAtS(double s) const {
    return stations_[FindNearestStationIndexAtS(s)];
  }

  double end_s() const { return stations_.back().accumulated_s(); }
  double front_s() const { return stations_.front().accumulated_s(); }
  double lane_path_start_s() const { return lane_path_start_s_; }
  bool beyond_lane_path() const { return beyond_lane_path_; }

  bool empty() const { return stations_.size() == 0; }
  int size() const { return stations_.size(); }

  const Station& station(StationIndex index) const { return stations_[index]; }
  const StationVector<Station>& stations() const { return stations_; }

  // Based on which the drive passage is built.
  // extend_lane_path has more segments on both sides than lane path. drive
  // passage's length is equal to extend_lane_path length. extend_lane_path is
  // also used to calculate traffic lights info on route as we want to
  // investigate the stoplines behind us.
  const mapping::LanePath& extend_lane_path() const {
    return extend_lane_path_;
  }

  // lane_path starts from plan_start_state and ends at route end or horizon
  // end. Route end stopline is calculated based on lane path.
  const mapping::LanePath& lane_path() const { return lane_path_; }

  mapping::SemanticLevelId AvLevel() const {
    return planner_semantic_map_manager_->GetLevel();
  }

  const PlannerSemanticMapManager* planner_semantic_map_manager() const {
    return planner_semantic_map_manager_;
  }

  const SemanticMapManager* semantic_map_manager() const {
    return planner_semantic_map_manager_->semantic_map_manager();
  }

 private:
  struct ProjectionResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double accum_s;
    double signed_l;
    double lerp_factor;
  };

  absl::StatusOr<ProjectionResult> ProjectPointToStations(Vec2d point) const;
  absl::Status IsProjectionResultOnDrivePassage(
      const ProjectionResult& res) const;

  struct BinarySeachResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double ds;  // s - prev_s
  };

  BinarySeachResult BinarySearchForNearStation(double s) const;

  const PlannerSemanticMapManager* planner_semantic_map_manager_;
  StationVector<Station> stations_;
  std::vector<double> center_seg_inv_len_;  // One less than stations.
  mapping::LanePath lane_path_;
  mapping::LanePath extend_lane_path_;
  bool beyond_lane_path_;
  double lane_path_start_s_;
  std::vector<Segment2d> segments_;
  std::unique_ptr<SegmentMatcherKdtree> segment_matcher_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_
