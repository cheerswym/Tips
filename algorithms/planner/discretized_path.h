#ifndef ONBOARD_PLANNER_DISCRETIZED_PATH_H_
#define ONBOARD_PLANNER_DISCRETIZED_PATH_H_

#include <vector>

#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

// The path points to construct a discretized-path should meet the conditions
// that 1) the s is monotonically increasing and 2) the start s is zero.
class DiscretizedPath : public std::vector<PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);

  double length() const {
    if (empty()) {
      return 0.0;
    }
    return back().s();
  }

  FrenetCoordinate XYToSL(Vec2d pos) const;

  PathPoint Evaluate(double path_s) const;

  PathPoint EvaluateReverse(double path_s) const;

 protected:
  std::vector<PathPoint>::const_iterator QueryLowerBound(double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(double path_s) const;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DISCRETIZED_PATH_H_
