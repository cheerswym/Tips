#ifndef ONBOARD_PLANNER_COMPOSITE_LANE_PATH_H_
#define ONBOARD_PLANNER_COMPOSITE_LANE_PATH_H_

#include <algorithm>
#include <deque>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/util.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

// A CompositeLanePath is a sequence of LanePaths, with overlaps at transition
// points.
//
//                      logical transition point
//                                 |
//                                 v
//
// o------lane_path0---------------------->o
//                         o------------------------lane_path1----->o
//
//                         ^               ^
//                         |               |
//                         +--- overlap ---+
//
// There is a logical transition point within the overlap region. In terms of
// arclength parameterized motion along the sequence of lane paths in this
// composite lane path, this logical transition point is considered the point at
// which the motion abruptly jumps from one lane path to the next. In general
// the overlap portions between the two consecutive lane paths should be roughly
// parallel and aligned to each other (usually ensured by lane neighbor
// relations), so that this jump is approximately lateral only.
//
class CompositeLanePath {
 public:
  using LanePath = mapping::LanePath;
  using LanePoint = mapping::LanePoint;

  struct TransitionInfo {
    static constexpr double kDefaultOverlapLength = 50.0;  // m.
    static constexpr double kDefaultTransitionPointFraction = 0.5;

    // Arclength of the overlap region in meters.
    double overlap_length = kDefaultOverlapLength;

    // Logical transition point in terms of fraction along the overlap region.
    double transition_point_fraction = kDefaultTransitionPointFraction;

    // Flag optimized
    bool optimized = false;

    // Lane change direction
    bool lc_left;

    // Lc section id. If two adjacent transtions share the same section id, we
    // will operate consecutive lane changes
    int64_t lc_section_id = mapping::kInvalidSectionId;

    bool IsValid() const {
      return overlap_length >= 0 &&
             InRange(transition_point_fraction, 0.0, 1.0);
    }
  };

  static constexpr TransitionInfo kDefaultTransition{
      .overlap_length = TransitionInfo::kDefaultOverlapLength,
      .transition_point_fraction =
          TransitionInfo::kDefaultTransitionPointFraction,
      .optimized = false,
      .lc_left = true,
      .lc_section_id = mapping::kInvalidSectionId};

 public:
  CompositeLanePath() {}

  CompositeLanePath(std::vector<LanePath> lane_paths,
                    std::vector<TransitionInfo> transitions)
      : lane_paths_(std::move(lane_paths)),
        transitions_(std::move(transitions)) {
    Build();
  }

  CompositeLanePath(std::vector<LanePath> lane_paths,
                    const TransitionInfo &transition)
      : lane_paths_(std::move(lane_paths)) {
    transitions_.assign(lane_paths_.size() - 1, transition);
    Build();
  }

  explicit CompositeLanePath(LanePath lane_path)
      : lane_paths_({std::move(lane_path)}) {
    Build();
  }

  CompositeLanePath(const SemanticMapManager *semantic_map_manager,
                    LanePoint lane_point)
      : CompositeLanePath(mapping::LanePath(semantic_map_manager, lane_point)) {
  }

  CompositeLanePath(const SemanticMapManager *semantic_map_manager,
                    const CompositeLanePathProto &proto) {
    FromProto(proto, semantic_map_manager);
  }

  CompositeLanePath(const SemanticMapManager *semantic_map_manager,
                    const mapping::LanePathProto &proto) {
    FromLanePathProto(proto, semantic_map_manager);
  }

  CompositeLanePath(const SemanticMapManager *semantic_map_manager,
                    const RouteProto &proto) {
    FromRouteProto(proto, semantic_map_manager);
  }

  CompositeLanePath Rebuild() const {
    CompositeLanePath path = *this;
    path.Build();
    for (auto &lane_path : path.lane_paths_) {
      lane_path.Rebuild();
    }
    return path;
  }

  // Number of lane segments (occurrence of lanes) along the path. This is the
  // sum of the sizes of all individual lane paths.
  int size() const { return lane_ids_.size(); }

  // Number of lane paths.
  int num_lane_paths() const { return lane_paths_.size(); }

  bool IsEmpty() const { return lane_paths_.empty(); }
  bool IsEqual(const CompositeLanePath &other) const {
    if (num_lane_paths() != other.num_lane_paths()) return false;
    for (int i = 0; i < num_lane_paths(); ++i) {
      if (lane_paths_[i] != other.lane_paths_[i]) return false;
    }
    if (transitions_.size() != other.transitions_.size()) return false;
    for (int i = 0; i < transitions_.size(); ++i) {
      if (transitions_[i].overlap_length !=
          other.transitions_[i].overlap_length) {
        return false;
      }
      if (transitions_[i].transition_point_fraction !=
          other.transitions_[i].transition_point_fraction) {
        return false;
      }
    }
    return true;
  }
  bool operator==(const CompositeLanePath &other) const {
    return IsEqual(other);
  }
  bool operator!=(const CompositeLanePath &other) const {
    return !IsEqual(other);
  }

  const std::vector<LanePath> &lane_paths() const { return lane_paths_; }
  const std::vector<TransitionInfo> &transitions() const {
    return transitions_;
  }
  const LanePath &lane_path(int i) const { return lane_paths_[i]; }
  // std::vector<mapping::ElementId> &lane_ids() const { return lane_ids_; }

  double GetLanePathLengthBeforeTransition(int lane_path_index) const {
    if (lane_path_index == 0) return 0.0;
    const auto &t = transitions_[lane_path_index - 1];
    return t.overlap_length * t.transition_point_fraction;
  }

  double GetLanePathLengthAfterTransition(int lane_path_index) const {
    if (lane_path_index + 1 == num_lane_paths()) return 0.0;
    const auto &t = transitions_[lane_path_index];
    return t.overlap_length * (1.0 - t.transition_point_fraction);
  }
  LanePath::LaneIndexPoint LanePathEntryIndexPoint(int lane_path_index) const {
    return lane_path_transition_points_[lane_path_index].first;
  }
  LanePath::LaneIndexPoint LanePathExitIndexPoint(int lane_path_index) const {
    return lane_path_transition_points_[lane_path_index].second;
  }

  LanePoint front() const {
    QCHECK(!IsEmpty());
    return lane_paths_.front().front();
  }
  LanePoint back() const {
    QCHECK(!IsEmpty());
    return lane_paths_.back().back();
  }

  bool IsConnectedTo(const CompositeLanePath &subsequent_path) const;
  bool IsConnectedTo(const LanePath &subsequent_path) const;
  CompositeLanePath Connect(const CompositeLanePath &subsequent_path) const;
  CompositeLanePath Compose(
      const CompositeLanePath &subsequent_path,
      const TransitionInfo &transition = kDefaultTransition) const;

  bool IsValid() const {
    return lane_paths_.size() == transitions_.size() + 1 &&
           std::all_of(lane_paths_.begin(), lane_paths_.end(),
                       [](const LanePath &p) { return p.IsValid(); }) &&
           std::all_of(transitions_.begin(), transitions_.end(),
                       [](const TransitionInfo &t) { return t.IsValid(); });
  }

  double length() const {
    QCHECK(!lane_path_end_s_.empty());
    return lane_path_end_s_.back();
  }

  // Two ways to identify a lane in a composite lane path:
  // - the overall index (a single int)
  // - the composite index (a {lane path index, lane index in lane path} pair).
  // The transformation from the former to the latter involves a binary search,
  // while the other way around is a simple arithmetic operation. Therefore the
  // canonical way for referencing a lane in this interface is the second way.
  struct CompositeIndex {
    int lane_path_index;
    int lane_index;

    CompositeIndex() : lane_path_index(0), lane_index(0) {}
    CompositeIndex(int lane_path_index, int lane_index)
        : lane_path_index(lane_path_index), lane_index(lane_index) {}
    explicit CompositeIndex(
        const CompositeLanePathProto::CompositeIndexProto &proto) {
      FromProto(proto);
    }

    bool operator==(const CompositeIndex &other) const {
      return lane_path_index == other.lane_path_index &&
             lane_index == other.lane_index;
    }
    bool operator!=(const CompositeIndex &other) const {
      return !(*this == other);
    }
    bool operator<(const CompositeIndex &other) const {
      return lane_path_index < other.lane_path_index ||
             (lane_path_index == other.lane_path_index &&
              lane_index < other.lane_index);
    }
    void FromProto(const CompositeLanePathProto::CompositeIndexProto &proto) {
      lane_path_index = proto.lane_path_index();
      lane_index = proto.lane_index();
    }
    void ToProto(CompositeLanePathProto::CompositeIndexProto *proto) const {
      proto->set_lane_path_index(lane_path_index);
      proto->set_lane_index(lane_index);
    }

    template <typename H>
    friend H AbslHashValue(H h, const CompositeIndex &index) {
      return H::combine(std::move(h), index.lane_path_index, index.lane_index);
    }

    std::string DebugString() const {
      return absl::StrCat("(", lane_path_index, ", ", lane_index, ")");
    }
  };

  CompositeIndex OverallIndexToCompositeIndex(int overall_index) const;
  int CompositeIndexToOverallIndex(CompositeIndex composite_index) const;

  CompositeIndex IncrementCompositeIndex(CompositeIndex composite_index,
                                         int inc) const;
  CompositeIndex DecrementCompositeIndex(CompositeIndex composite_index,
                                         int dec) const;

  // Signed distance from index0 to index1 (positive if index1 > index0).
  int CompositeIndexDist(CompositeIndex index0, CompositeIndex index1) const;

  CompositeIndex composite_index_begin() const { return {0, 0}; }
  CompositeIndex composite_index_end() const { return {num_lane_paths(), 0}; }

  // Access to individual lanes. Note that this iteration does not visit the
  // extra lanes in the overlap regions (i.e. lanes after or before the logical
  // transition points).
  class LaneSegmentIterator
      : public std::iterator<std::bidirectional_iterator_tag,
                             LanePath::LaneSegment> {
   public:
    explicit LaneSegmentIterator(const CompositeLanePath *composite_lane_path,
                                 CompositeIndex composite_index)
        : composite_lane_path_(*QCHECK_NOTNULL(composite_lane_path)),
          composite_index_(composite_index) {}

    const CompositeLanePath &composite_lane_path() const {
      return composite_lane_path_;
    }
    const LanePath &lane_path() const {
      return composite_lane_path_.lane_path(composite_index_.lane_path_index);
    }
    int lane_path_index() const { return composite_index_.lane_path_index; }
    int lane_index() const { return composite_index_.lane_index; }
    CompositeIndex composite_index() const { return composite_index_; }
    int overall_index() const {
      return composite_lane_path_.CompositeIndexToOverallIndex(
          composite_index_);
    }

    LanePath::LaneSegment operator*() const {
      return composite_lane_path_.lane_segment_in_composite_lane_path(
          composite_index_);
    }

    LaneSegmentIterator &operator=(const LaneSegmentIterator &other) {
      DCHECK(composite_lane_path_ == other.composite_lane_path_);
      composite_index_ = other.composite_index_;
      return *this;
    }

    bool operator==(const LaneSegmentIterator &other) const {
      DCHECK(composite_lane_path_ == other.composite_lane_path_);
      return composite_index_ == other.composite_index_;
    }
    bool operator!=(const LaneSegmentIterator &other) const {
      DCHECK(composite_lane_path_ == other.composite_lane_path_);
      return !(*this == other);
    }
    bool operator<(const LaneSegmentIterator &other) const {
      DCHECK(composite_lane_path_ == other.composite_lane_path_);
      return composite_index_ < other.composite_index_;
    }

    LaneSegmentIterator operator+(int inc) const {
      return LaneSegmentIterator(
          &composite_lane_path_,
          composite_lane_path_.IncrementCompositeIndex(composite_index_, inc));
    }
    LaneSegmentIterator operator-(int dec) const {
      return LaneSegmentIterator(
          &composite_lane_path_,
          composite_lane_path_.DecrementCompositeIndex(composite_index_, dec));
    }
    int operator-(const LaneSegmentIterator &other) const {
      DCHECK(composite_lane_path_ == other.composite_lane_path_);
      return composite_lane_path_.CompositeIndexDist(other.composite_index_,
                                                     composite_index_);
    }

    LaneSegmentIterator operator++() {
      const LaneSegmentIterator old_it = *this;
      composite_index_ =
          composite_lane_path_.IncrementCompositeIndex(composite_index_, 1);
      return old_it;
    }
    const LaneSegmentIterator &operator++(int) {
      composite_index_ =
          composite_lane_path_.IncrementCompositeIndex(composite_index_, 1);
      return *this;
    }
    LaneSegmentIterator operator--() {
      const LaneSegmentIterator old_it = *this;
      composite_index_ =
          composite_lane_path_.DecrementCompositeIndex(composite_index_, 1);
      return old_it;
    }
    const LaneSegmentIterator &operator--(int) {
      composite_index_ =
          composite_lane_path_.DecrementCompositeIndex(composite_index_, 1);
      return *this;
    }

   private:
    const CompositeLanePath &composite_lane_path_;
    CompositeIndex composite_index_;
  };

  LaneSegmentIterator begin() const {
    return LaneSegmentIterator(this, composite_index_begin());
  }
  LaneSegmentIterator end() const {
    return LaneSegmentIterator(this, composite_index_end());
  }

  LanePath::LaneSegment lane_segment_in_lane_path(
      CompositeIndex composite_index) const {
    DCHECK_GE(composite_index.lane_path_index, 0);
    DCHECK_LT(composite_index.lane_path_index, num_lane_paths());
    return lane_path(composite_index.lane_path_index)
        .lane_segment(composite_index.lane_index);
  }

  LanePath::LaneSegment lane_segment_in_lane_path(int overall_index) const {
    return lane_segment_in_lane_path(
        OverallIndexToCompositeIndex(overall_index));
  }

  LanePath::LaneSegment lane_segment_in_composite_lane_path(
      CompositeIndex composite_index) const;

  LanePath::LaneSegment lane_segment_in_composite_lane_path(
      int overall_index) const {
    return lane_segment_in_composite_lane_path(
        OverallIndexToCompositeIndex(overall_index));
  }

  class LaneIndexPoint {
   public:
    LaneIndexPoint() = default;
    LaneIndexPoint(const CompositeIndex &index, double lane_fraction)
        : index_(index), lane_fraction_(lane_fraction) {}

    explicit LaneIndexPoint(
        const CompositeLanePathProto::LaneIndexPointProto &proto) {
      FromProto(proto);
    }
    void set_index(const CompositeIndex &index) { index_ = index; }
    void set_lane_fraction(double lane_fraction) {
      lane_fraction_ = lane_fraction;
    }
    const CompositeIndex &index() const { return index_; }
    double lane_fraction() const { return lane_fraction_; }
    void ToProto(CompositeLanePathProto::LaneIndexPointProto *proto) const;
    void FromProto(const CompositeLanePathProto::LaneIndexPointProto &proto);

    bool operator==(const LaneIndexPoint &o) const {
      return index() == o.index() && lane_fraction() == o.lane_fraction();
    }

    template <typename H>
    friend H AbslHashValue(H h, const LaneIndexPoint &lip) {
      return H::combine(std::move(h), lip.index(), lip.lane_fraction());
    }

   private:
    CompositeIndex index_;
    double lane_fraction_;
  };

  class LanePathPoint {
   public:
    LanePathPoint() : LanePathPoint(-1, 0) {}
    LanePathPoint(int lane_path_index, double s)
        : index_(lane_path_index), s_(s) {}
    int index() const { return index_; }
    double s() const { return s_; }

   private:
    // Lane path index.
    int index_;
    // s in lane path.
    double s_;
  };

  int ArclengthToLanePathIndex(double s) const;
  LanePathPoint ArclengthToLanePathPoint(double s) const;
  LaneIndexPoint ArclengthToLaneIndexPoint(double s) const;
  double LanePathIndexToArclength(int lane_path_index) const;
  double LanePathPointToArclength(LanePathPoint lane_path_point) const;
  double LaneIndexPointToArclength(
      const LaneIndexPoint &lane_index_point) const;

  LanePoint ArclengthToLanePoint(double s) const;
  double FirstOccurrenceOfLanePointToArclength(LanePoint point) const;

  bool ContainsLanePoint(
      LanePoint point,
      LaneIndexPoint *first_encounter_of_lane_index_point = nullptr,
      bool forward_search = true) const;

  CompositeLanePath BeforeFirstOccurrenceOfLanePoint(LanePoint point) const;
  CompositeLanePath BeforeLastOccurrenceOfLanePoint(LanePoint point) const;
  CompositeLanePath AfterFirstOccurrenceOfLanePoint(LanePoint point) const;
  CompositeLanePath BeforeArclength(double s) const;
  CompositeLanePath AfterArclength(double s) const;
  /// @brief If both composite_index composite_indexthe and lane point are
  /// provided, no need to use route_s__)
  CompositeLanePath AfterCompositeIndexAndLanePoint(
      const SemanticMapManager *semantic_map_manager,
      const CompositeIndex &composite_index,
      const mapping::LanePoint &LanePoint) const;
  CompositeLanePath BeforeLanePathPoint(LanePathPoint lane_path_point) const;
  CompositeLanePath AfterLanePathPoint(LanePathPoint lane_path_point) const;
  CompositeLanePath BeforeLaneIndexPoint(LaneIndexPoint lane_index_point) const;

  void ToProto(CompositeLanePathProto *proto) const;
  void FromProto(const CompositeLanePathProto &proto,
                 const SemanticMapManager *semantic_map_manager);

  void FromLanePathProto(const mapping::LanePathProto &proto,
                         const SemanticMapManager *semantic_map_manager);
  void FromRouteProto(const RouteProto &route_proto,
                      const SemanticMapManager *semantic_map_manager);

  std::string DebugString() const;
  std::string ShortDebugString() const;

  void Clear();

 protected:
  void Build();

 private:
  LaneIndexPoint FirstOccurrenceOfLanePointToLaneIndexPoint(
      LanePoint point) const;
  LaneIndexPoint LastOccurrenceOfLanePointToLaneIndexPoint(
      LanePoint point) const;
  LanePathPoint FirstOccurrenceOfLanePointToLanePathPoint(
      LanePoint point) const;
  LanePathPoint LastOccurrenceOfLanePointToLanePathPoint(LanePoint point) const;

  std::vector<LanePath> lane_paths_;

  // One for each transtion between consecutive lane paths. One element shorter
  // than lane_paths_.
  std::vector<TransitionInfo> transitions_;

  // Arclength at startpoint of each lane path. One longer than lane_paths_.
  // First element is always 0.
  // The endpoint of a lane path other than the last one in lane_paths_ is
  // defined to be the logical transition point. Therefore thie end s of one
  // lane path is the same as the start s of the next lane path.
  std::vector<double> lane_path_end_s_;

  // Transition (entry and exit) points on each lane path. Same size as
  // lane_paths_.
  std::vector<std::pair<LanePath::LaneIndexPoint, LanePath::LaneIndexPoint>>
      lane_path_transition_points_;

  std::vector<mapping::ElementId> lane_ids_;
  std::vector<int> cumulative_lane_path_sizes_;  // One longer than lane_paths_.
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMPOSITE_LANE_PATH_H_
