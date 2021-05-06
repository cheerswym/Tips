#include "onboard/planner/trajectory_util.h"

#include <algorithm>

namespace qcraft {
namespace planner {

void LerpTrajPoint(const TrajectoryPointProto &p0,
                   const TrajectoryPointProto &p1, double alpha,
                   TrajectoryPointProto *p) {
  p->mutable_pos()->set_x(Lerp(p0.pos().x(), p1.pos().x(), alpha));
  p->mutable_pos()->set_y(Lerp(p0.pos().y(), p1.pos().y(), alpha));
#define LERP_FIELD(field, lerp) \
  p->set_##field(lerp(p0.field(), p1.field(), alpha));
  LERP_FIELD(s, Lerp)
  LERP_FIELD(theta, LerpAngle)
  LERP_FIELD(kappa, Lerp)
  LERP_FIELD(t, Lerp)
  LERP_FIELD(v, Lerp)
  LERP_FIELD(a, Lerp)
  LERP_FIELD(j, Lerp)
  LERP_FIELD(psi, Lerp)
  LERP_FIELD(chi, Lerp)
#undef LERP_FIELD
}

void LerpTrajPoint(const TrajectoryPoint &p0, const TrajectoryPoint &p1,
                   double alpha, TrajectoryPoint *p) {
  p->set_pos(Vec2d(Lerp(p0.pos().x(), p1.pos().x(), alpha),
                   Lerp(p0.pos().y(), p1.pos().y(), alpha)));
#define LERP_FIELD(field, lerp) \
  p->set_##field(lerp(p0.field(), p1.field(), alpha));
  LERP_FIELD(s, Lerp)
  LERP_FIELD(theta, LerpAngle)
  LERP_FIELD(kappa, Lerp)
  LERP_FIELD(t, Lerp)
  LERP_FIELD(v, Lerp)
  LERP_FIELD(a, Lerp)
  LERP_FIELD(j, Lerp)
  LERP_FIELD(psi, Lerp)
  LERP_FIELD(chi, Lerp)
#undef LERP_FIELD
}

void LerpTrajPoint(const SecondOrderTrajectoryPoint &p0,
                   const SecondOrderTrajectoryPoint &p1, double alpha,
                   SecondOrderTrajectoryPoint *p) {
  p->set_pos(Vec2d(Lerp(p0.pos().x(), p1.pos().x(), alpha),
                   Lerp(p0.pos().y(), p1.pos().y(), alpha)));
#define LERP_FIELD(field, lerp) \
  p->set_##field(lerp(p0.field(), p1.field(), alpha));
  LERP_FIELD(s, Lerp)
  LERP_FIELD(theta, LerpAngle)
  LERP_FIELD(kappa, Lerp)
  LERP_FIELD(t, Lerp)
  LERP_FIELD(v, Lerp)
  LERP_FIELD(a, Lerp)
#undef LERP_FIELD
}

std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const TrajectoryPointProto> traj) {
  std::vector<ApolloTrajectoryPointProto> res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i) {
    res.push_back(ToApolloTrajectoryPointProto(traj[i]));
  }

  return res;
}

std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const prediction::PredictedTrajectoryPoint> traj) {
  std::vector<ApolloTrajectoryPointProto> res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i) {
    TrajectoryPointProto pt;
    traj[i].SecondOrderTrajectoryPoint::ToProto(&pt);
    res.push_back(ToApolloTrajectoryPointProto(pt));
  }
  return res;
}

std::vector<TrajectoryPointProto> ToTrajectoryPointProto(
    absl::Span<const ApolloTrajectoryPointProto> traj) {
  std::vector<TrajectoryPointProto> res;
  if (traj.empty()) return res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i) {
    res.push_back(ToTrajectoryPointProto(traj[i]));
  }
  // Fill fourth-order state by finite difference up to the last but one point
  // and assume that the last point has identical one as the last but one
  // point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_chi((res[i + 1].psi() - res[i].psi()) /
                   (res[i + 1].t() - res[i].t()));
  }
  if (traj.size() > 1) {
    res.back().set_chi((res.rbegin() + 1)->chi());
  }

  return res;
}

std::vector<TrajectoryPointProto> ToTrajectoryPointProtoFromSecondOrderApollo(
    absl::Span<const ApolloTrajectoryPointProto> traj) {
  std::vector<TrajectoryPointProto> res;
  if (traj.empty()) return res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i) {
    res.push_back(ToTrajectoryPointProtoFromSecondOrderApollo(traj[i]));
  }
  // Fill third-order states by finite difference up to the last but one point
  // and assume that the last point has identical ones as the last but one
  // point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_j((traj[i + 1].a() - traj[i].a()) /
                 (traj[i + 1].relative_time() - traj[i].relative_time()));
    res[i].set_psi(
        (traj[i + 1].path_point().kappa() - traj[i].path_point().kappa()) /
        (traj[i + 1].relative_time() - traj[i].relative_time()));
  }
  if (traj.size() > 1) {
    res.back().set_j((res.rbegin() + 1)->j());
    res.back().set_psi((res.rbegin() + 1)->psi());
  }
  // Fill fourth-order state by finite difference up to the last but one point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_chi((res[i + 1].psi() - res[i].psi()) /
                   (res[i + 1].t() - res[i].t()));
  }
  res.back().set_chi(0.0);

  return res;
}

std::vector<TrajectoryPointProto> ToTrajectoryPointProto(
    absl::Span<const TrajectoryPoint> traj) {
  std::vector<TrajectoryPointProto> res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i)
    traj[i].ToProto(&res.emplace_back());

  return res;
}

std::vector<TrajectoryPoint> ToTrajectoryPoint(
    absl::Span<const TrajectoryPointProto> traj) {
  std::vector<TrajectoryPoint> res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i) res.emplace_back(traj[i]);

  return res;
}

std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const TrajectoryPoint> traj) {
  std::vector<ApolloTrajectoryPointProto> res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i)
    traj[i].ToProto(&res.emplace_back());

  return res;
}

std::vector<TrajectoryPoint> ToTrajectoryPoint(
    absl::Span<const ApolloTrajectoryPointProto> traj) {
  std::vector<TrajectoryPoint> res;
  if (traj.empty()) return res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i)
    res.emplace_back(ToTrajectoryPointProto(traj[i]));
  // Fill fourth-order state by finite difference up to the last but one point
  // and assume that the last point has identical one as the last but one
  // point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_chi((res[i + 1].psi() - res[i].psi()) /
                   (res[i + 1].t() - res[i].t()));
  }
  if (traj.size() > 1) {
    res.back().set_chi((res.rbegin() + 1)->chi());
  }

  return res;
}

std::vector<TrajectoryPoint> ToTrajectoryPointFromSecondOrderApollo(
    absl::Span<const ApolloTrajectoryPointProto> traj) {
  std::vector<TrajectoryPoint> res;
  if (traj.empty()) return res;
  res.reserve(traj.size());
  for (int i = 0, n = traj.size(); i < n; ++i)
    res.emplace_back(ToTrajectoryPointProtoFromSecondOrderApollo(traj[i]));
  // Fill third-order states by finite difference up to the last but one point
  // and assume that the last point has identical ones as the last but one
  // point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_j((traj[i + 1].a() - traj[i].a()) /
                 (traj[i + 1].relative_time() - traj[i].relative_time()));
    res[i].set_psi(
        (traj[i + 1].path_point().kappa() - traj[i].path_point().kappa()) /
        (traj[i + 1].relative_time() - traj[i].relative_time()));
  }
  if (traj.size() > 1) {
    res.back().set_j((res.rbegin() + 1)->j());
    res.back().set_psi((res.rbegin() + 1)->psi());
  }
  // Fill fourth-order state by finite difference up to the last but one point.
  for (int i = 0, n = traj.size() - 1; i < n; ++i) {
    res[i].set_chi((res[i + 1].psi() - res[i].psi()) /
                   (res[i + 1].t() - res[i].t()));
  }
  res.back().set_chi(0.0);

  return res;
}

std::vector<ApolloTrajectoryPointProto> ShiftTrajectoryByTime(
    double shift_time,
    const std::vector<ApolloTrajectoryPointProto> &trajectory_points,
    double max_decel_jerk, double max_acc_jerk) {
  if (trajectory_points.empty()) {
    return {};
  }
  const double trajectory_time_step = trajectory_points[1].relative_time() -
                                      trajectory_points[0].relative_time();
  const int kShiftIndex = RoundToInt(shift_time / trajectory_time_step);
  const int trajectory_size = trajectory_points.size();
  if (kShiftIndex < 0 || kShiftIndex >= trajectory_size) {
    return {};
  }
  if (kShiftIndex == 0) {
    return trajectory_points;
  }
  const ApolloTrajectoryPointProto &start_point =
      trajectory_points[kShiftIndex];
  const double start_s = start_point.path_point().s();
  const double start_t = start_point.relative_time();

  std::vector<ApolloTrajectoryPointProto> res_traj;
  res_traj.reserve(trajectory_size);

  for (int i = kShiftIndex; i < trajectory_points.size(); ++i) {
    res_traj.push_back(trajectory_points[i]);
    auto &point = res_traj.back();
    point.set_relative_time(point.relative_time() - start_t);
    point.mutable_path_point()->set_s(point.path_point().s() - start_s);
  }

  constexpr double kMinSpeed = 1e-6;
  constexpr double kMinDist = 1e-6;
  while (res_traj.size() < trajectory_size) {
    // Extrapolate past end of trajectory using a simple const-acceleration
    // model to fill out the remainder of points.
    auto &prev_point = res_traj.back();
    const double dist = std::max(
        kMinDist, prev_point.v() * trajectory_time_step +
                      0.5 * prev_point.a() * Sqr(trajectory_time_step));
    auto point = prev_point;
    point.set_relative_time(prev_point.relative_time() + trajectory_time_step);
    *point.mutable_path_point() =
        GetPathPointAlongCircle(point.path_point(), dist);
    // Make sure this and next step speed is not below zero.
    point.set_v(std::max(
        kMinSpeed, prev_point.v() + prev_point.a() * trajectory_time_step));
    point.set_a(
        std::max(prev_point.a(), -1.0 * point.v() / trajectory_time_step));
    prev_point.set_j(
        std::clamp((point.a() - prev_point.a()) / trajectory_time_step,
                   max_decel_jerk, max_acc_jerk));
    res_traj.push_back(point);  // Don't use prev_point after this point.
  }
  res_traj.back().set_j((res_traj.rbegin() + 1)->j());
  QCHECK_EQ(res_traj.size(), trajectory_size);

  return res_traj;
}

}  // namespace planner
}  // namespace qcraft
