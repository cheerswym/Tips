#ifndef ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_point.h"

namespace qcraft::planner {

class SpeedVector : public std::vector<SpeedPoint> {
 public:
  SpeedVector() = default;

  virtual ~SpeedVector() = default;

  explicit SpeedVector(std::vector<SpeedPoint> speed_points);

  std::optional<SpeedPoint> EvaluateByTime(double t) const;

  std::optional<SpeedPoint> EvaluateByS(double s) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  // Use container as a template to make it compatible with protobuf's repeated
  // field and vector.
  template <template <class...> class Container>
  void FromProto(const Container<SpeedPointProto>& speed_points) {
    reserve(speed_points.size());
    for (const auto& speed_point : speed_points) {
      SpeedPoint sp;
      sp.FromProto(speed_point);
      push_back(std::move(sp));
    }
    std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
      return p1.t() < p2.t();
    });
  }

  void ToProto(SpeedPointsProto* speed_points) const {
    speed_points->Clear();
    for (auto it = begin(); it != end(); ++it) {
      it->ToProto(speed_points->add_speed_points());
    }
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
