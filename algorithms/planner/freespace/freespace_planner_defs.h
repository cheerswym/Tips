#ifndef ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_

#include <string>
#include <vector>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"

namespace qcraft {
namespace planner {

struct FreespaceBoundary {
  std::string id;
  FreespaceMapProto::BoundaryType type;
  Segment2d segment;
  bool near_parking_spot;
};

enum class SpecialBoundaryType {
  GEAR_REVERSE_STOPPER,
  SOFT_PARKING_SPOT_LINE,
  CROSSABLE_LANE_LINE  // Consider in local smoother but not in ha*.
};

struct SpecialBoundary {
  std::string id;
  Segment2d segment;
  SpecialBoundaryType type;
};

struct FreespaceMap {
  // Freespace region is defined as an AAbox whose center is final goal.
  AABox2d region;
  // Map boundaries.
  std::vector<FreespaceBoundary> boundaries;
  // Special boundaries.
  std::vector<SpecialBoundary> special_boundaries;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_
