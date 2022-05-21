
#ifndef ONBOARD_PREDICTION_UTIL_MOTION_MODEL_H_
#define ONBOARD_PREDICTION_UTIL_MOTION_MODEL_H_

#include <functional>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/proto/perception.pb.h"
namespace qcraft {
namespace prediction {
struct UniCycleState {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double heading = 0.0;
  double yaw_rate = 0.0;
  double acc = 0.0;
};
constexpr int kUniCycleStateNum = 6;

inline UniCycleState ObjectProtoToUniCycleState(const ObjectProto& obj_proto) {
  UniCycleState state = {.x = obj_proto.pos().x(),
                         .y = obj_proto.pos().y(),
                         .v = Vec2dFromProto(obj_proto.vel()).norm(),
                         .heading = NormalizeAngle(obj_proto.yaw()),
                         .yaw_rate = obj_proto.yaw_rate()};
  const auto accel = Vec2d(obj_proto.accel());
  const auto speed = Vec2d(obj_proto.vel());
  if (accel.dot(speed) < 0) {
    state.acc = -accel.norm();
  } else {
    state.acc = accel.norm();
  }
  return state;
}

constexpr int kBicycleModelStateNum = 8;

struct BicycleModelState {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double heading = 0.0;
  double acc = 0.0;
  double front_wheel_angle = 0.0;

  std::string DebugString() const {
    return absl::StrCat("x: ", x, " y: ", y, " v: ", v, " heading:", heading,
                        " acc:", acc,
                        " front_wheel_angle: ", front_wheel_angle);
  }
};

BicycleModelState SimulateBicycleModel(const BicycleModelState& prev_state,
                                       double lf, double lr, double dt);

// x[]: state (including state and control)
std::vector<double> BicycleModel(absl::Span<const double> x);

UniCycleState SimulateUniCycleModel(const UniCycleState& prev_state, double dt);

// x[]: state (including state and control)
std::vector<double> UniCycleModel(absl::Span<const double> x);

std::vector<double> RungeKutta4(
    absl::Span<const double> u0, double dt,
    std::function<std::vector<double>(absl::Span<const double>)> model);

}  // namespace prediction
}  // namespace qcraft

#endif
