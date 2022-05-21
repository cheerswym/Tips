#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_THIRD_ORDER_BICYCLE_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_THIRD_ORDER_BICYCLE_H_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/fast_math.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/cost_helper.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// The "third order" here refers to the control being the third derivatives of
// position in both lateral and longitudinal directions:
//   psi = dkappa/dt = d(dtheta/ds)/dt = d(||d(dx/ds))/ds||)/dt
//   j = da/dt = d(dv/dt)/dt = d(d(ds/dt)/dt)/dt
//
// The Euler-style time discretization in the implementation below is second
// order.
// The system dynamics differentiation implemented below is second order
// (or first order if --ddp_tob_use_f_second_derivative=false).
template <int STEPS>
class ThirdOrderBicycle {
 public:
  struct ClampInfo {
    std::vector<int> indice;
    std::vector<std::pair<double, double>> values;
  };

 public:
  ThirdOrderBicycle(const MotionConstraintParamsProto *motion_constraint_params,
                    const VehicleGeometryParamsProto *veh_geo_params,
                    const VehicleDriveParamsProto *veh_drive_params,
                    double trajectory_time_step,
                    bool enable_post_process = true)
      : motion_constraint_params_(motion_constraint_params),
        veh_geo_params_(veh_geo_params),
        veh_drive_params_(veh_drive_params),
        dt_(trajectory_time_step),
        enable_post_process_(enable_post_process) {}

  static constexpr char kProblemPrefix[] = "Tob";

  static constexpr int kHorizon = STEPS;
  // State: x, y, theta, v, kappa, a, s
  static constexpr int kStateSize = 7;
  static constexpr int kStateXIndex = 0;
  static constexpr int kStateYIndex = 1;
  static constexpr int kStateThetaIndex = 2;
  static constexpr int kStateVIndex = 3;
  static constexpr int kStateKappaIndex = 4;
  static constexpr int kStateAIndex = 5;
  static constexpr int kStateSIndex = 6;

  // Control: psi, j.
  static constexpr int kControlSize = 2;
  static constexpr int kControlPsiIndex = 0;
  static constexpr int kControlJIndex = 1;

  static constexpr double kMinSpeed = 1.0e-6;
  static constexpr double kMaxSpeed = std::numeric_limits<double>::infinity();

  // Dynamics (mixed second order time integration):
  //   a^{k+1} = a^k + j dt                                              (exact)
  //   kappa^{k+1} = kappa^k + psi dt                                    (exact)
  //   v^{k+1} = v^k + a^{k+0.5} dt
  //           = v^k + a^k dt + 0.5 j dt^2                               (exact)
  //   theta^{k+1} = theta^k + v^{k+0.5} kappa^{k+0.5} dt
  //               = theta^k + (v^k + 0.5 a^k dt + 0.25 j dt^2) *
  //                     (kappa^k + 0.5 psi dt) dt                   (2nd order)
  //   x^{k+1} = x^k + v^{k+0.5} cos(theta^{k+0.5}) dt
  //           = x^k + (v^k + 0.5 a dt + 0.25 j dt^2) dt *
  //                 cos(theta^k + 0.5 (v^k + 0.5 a^k dt + 0.25 j dt^2) *
  //                         (kappa^k + 0.5 psi dt) dt)              (2nd order)
  //   y^{k+1} = y^k + v^{k+0.5} sin(theta^{k+0.5}) dt
  //           = y^k + (v^k + 0.5 a dt + 0.25 j dt^2) dt *
  //                 sin(theta^k + 0.5 (v^k + 0.5 a^k dt + 0.25 j dt^2) *
  //                         (kappa^k + 0.5 psi dt) dt)              (2nd order)
  //   s^{k+1} = s^k + v^k dt + 0.5 * a^k dt^2 + j^k * dt^3 / 6.0        (exact)

  // Types.
  using StateType = Eigen::Matrix<double, kStateSize, 1>;
  using ControlType = Eigen::Matrix<double, kControlSize, 1>;
  using StatesType = Eigen::Matrix<double, kStateSize * kHorizon, 1>;
  using ControlsType = Eigen::Matrix<double, kControlSize * kHorizon, 1>;

  using GType = double;
  using DGDxType = Eigen::Matrix<double, 1, kStateSize>;
  using DGDuType = Eigen::Matrix<double, 1, kControlSize>;
  using DDGDxDxType = Eigen::Matrix<double, kStateSize, kStateSize>;
  using DDGDxDuType = Eigen::Matrix<double, kStateSize, kControlSize>;
  using DDGDuDxType = Eigen::Matrix<double, kControlSize, kStateSize>;
  using DDGDuDuType = Eigen::Matrix<double, kControlSize, kControlSize>;

  using FType = StateType;
  using DFDxType = Eigen::Matrix<double, kStateSize, kStateSize>;
  using DFDuType = Eigen::Matrix<double, kStateSize, kControlSize>;
  // Outmost dimension of the F hessians below is the intrinsic dimension of f.
  // After a dot product with a type compatible with the G gradient (such as
  // \nabla J), they become a type compatible with the G hessian.
  using DDFDxDxType = std::array<DDGDxDxType, kStateSize>;
  using DDFDxDuType = std::array<DDGDxDuType, kStateSize>;
  using DDFDuDxType = std::array<DDGDuDxType, kStateSize>;
  using DDFDuDuType = std::array<DDGDuDuType, kStateSize>;

  // State and control accessors.
  static StateType MakeState(double pos_x, double pos_y, double theta, double v,
                             double kappa, double a, double s) {
    StateType x;
    StateSetX(pos_x, &x);
    StateSetY(pos_y, &x);
    StateSetTheta(theta, &x);
    StateSetV(v, &x);
    StateSetKappa(kappa, &x);
    StateSetA(a, &x);
    StateSetS(s, &x);
    return x;
  }

  static StateType TestState() {
    return MakeState(1.0, 0.5, 0.0, 1.0, 0.1, 0.1, 0.4);
  }

  static ControlType MakeControl(double psi, double j) {
    ControlType u;
    ControlSetPsi(psi, &u);
    ControlSetJ(j, &u);
    return u;
  }

  static ControlType TestControl() { return MakeControl(0.01, 0.05); }

  static double StateGetX(const StateType &x) { return x[kStateXIndex]; }
  static double StateGetY(const StateType &x) { return x[kStateYIndex]; }
  static Vec2d StateGetPos(const StateType &x) {
    return Vec2d(x[kStateXIndex], x[kStateYIndex]);
  }
  static double StateGetTheta(const StateType &x) {
    return x[kStateThetaIndex];
  }
  static double StateGetV(const StateType &x) { return x[kStateVIndex]; }
  static double StateGetKappa(const StateType &x) {
    return x[kStateKappaIndex];
  }
  static double StateGetA(const StateType &x) { return x[kStateAIndex]; }
  static double StateGetS(const StateType &x) { return x[kStateSIndex]; }
  static void StateSetX(double pos_x, StateType *x) {
    (*x)[kStateXIndex] = pos_x;
  }
  static void StateSetY(double pos_y, StateType *x) {
    (*x)[kStateYIndex] = pos_y;
  }
  static void StateSetPos(Vec2d pos, StateType *x) {
    (*x)[kStateXIndex] = pos.x();
    (*x)[kStateYIndex] = pos.y();
  }
  static void StateSetTheta(double theta, StateType *x) {
    (*x)[kStateThetaIndex] = theta;
  }
  static void StateSetV(double v, StateType *x) { (*x)[kStateVIndex] = v; }
  static void StateSetKappa(double kappa, StateType *x) {
    (*x)[kStateKappaIndex] = kappa;
  }
  static void StateSetA(double a, StateType *x) { (*x)[kStateAIndex] = a; }
  static void StateSetS(double s, StateType *x) { (*x)[kStateSIndex] = s; }

  static double ControlGetPsi(const ControlType &u) {
    return u[kControlPsiIndex];
  }
  static double ControlGetJ(const ControlType &u) { return u[kControlJIndex]; }
  static void ControlSetPsi(double psi, ControlType *u) {
    (*u)[kControlPsiIndex] = psi;
  }
  static void ControlSetJ(double j, ControlType *u) {
    (*u)[kControlJIndex] = j;
  }

  static StateType GetStateAtStep(const StatesType &xs, int k) {
    return xs.template segment<kStateSize>(k * kStateSize);
  }
  static Eigen::Map<const StateType> GetStateMapAtStep(const StatesType &xs,
                                                       int k) {
    return Eigen::Map<const StateType>(xs.data() + k * kStateSize, kStateSize);
  }
  static ControlType GetControlAtStep(const ControlsType &us, int k) {
    return us.template segment<kControlSize>(k * kControlSize);
  }
  static Eigen::Map<const ControlsType> GetControlMapAtStep(
      const ControlsType &us, int k) {
    return Eigen::Map<const ControlsType>(us.data() + k * kControlSize,
                                          kControlSize);
  }
  static void SetStateAtStep(const StateType &x, int k, StatesType *xs) {
    xs->template segment<kStateSize>(k * kStateSize) = x;
  }
  static void SetControlAtStep(const ControlType &u, int k, ControlsType *us) {
    us->template segment<kControlSize>(k * kControlSize) = u;
  }

  static double x(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateXIndex];
  }
  static double y(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateYIndex];
  }
  static Vec2d pos(const StatesType &xs, int k) {
    return Vec2d(xs(k * kStateSize + kStateXIndex),
                 xs(k * kStateSize + kStateYIndex));
  }
  static double theta(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateThetaIndex];
  }
  static double v(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateVIndex];
  }
  static double kappa(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateKappaIndex];
  }
  static double a(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateAIndex];
  }
  static double s(const StatesType &xs, int k) {
    return xs[k * kStateSize + kStateSIndex];
  }
  static double psi(const ControlsType &us, int k) {
    return us[k * kControlSize + kControlPsiIndex];
  }
  static double j(const ControlsType &us, int k) {
    return us[k * kControlSize + kControlJIndex];
  }

  static void set_x(double x, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateXIndex] = x;
  }
  static void set_y(double y, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateYIndex] = y;
  }
  static void set_pos(Vec2d pos, int k, StatesType *xs) {
    (*xs)(k * kStateSize + kStateXIndex) = pos.x();
    (*xs)(k * kStateSize + kStateYIndex) = pos.y();
  }
  static void set_theta(double theta, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateThetaIndex] = theta;
  }
  static void set_v(double v, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateVIndex] = v;
  }
  static void set_kappa(double kappa, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateKappaIndex] = kappa;
  }
  static void set_a(double a, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateAIndex] = a;
  }
  static void set_s(double s, int k, StatesType *xs) {
    (*xs)[k * kStateSize + kStateSIndex] = s;
  }
  static void set_psi(double psi, int k, ControlsType *us) {
    (*us)[k * kControlSize + kControlPsiIndex] = psi;
  }
  static void set_j(double j, int k, ControlsType *us) {
    (*us)[k * kControlSize + kControlJIndex] = j;
  }

  double dt() const { return dt_; }

  bool enable_post_process() const { return enable_post_process_; }

  // F and derivatives.
  struct FDerivatives {
    FType value;
    DFDxType dfdx;
    DFDuType dfdu;
    DDFDxDxType ddfdxdx;
    DDFDuDxType ddfdudx;
    DDFDuDuType ddfdudu;
  };

  void EvaluateFDerivatives(int k, const StateType &x, const ControlType &u,
                            FDerivatives *fd) const {
    const double kSquareDt = Sqr(dt_);
    const double kCubicDt = Cube(dt_);

    const double pos_x = StateGetX(x);
    const double pos_y = StateGetY(x);
    const double theta = StateGetTheta(x);
    const double v = StateGetV(x);
    const double kappa = StateGetKappa(x);
    const double a = StateGetA(x);
    const double s = StateGetS(x);
    const double psi = ControlGetPsi(u);
    const double j = ControlGetJ(u);

    const double half_da = 0.5 * j * dt_;
    const double halfway_a = a + half_da;
    const double half_dv = 0.5 * halfway_a * dt_;
    const double halfway_v = v + half_dv;
    const double half_dkappa = 0.5 * psi * dt_;
    const double halfway_kappa = kappa + half_dkappa;
    const double half_dtheta = 0.5 * halfway_v * halfway_kappa * dt_;
    const double halfway_theta = theta + half_dtheta;
    const double cos_halfway_theta = std::cos(halfway_theta);
    const double sin_halfway_theta = std::sin(halfway_theta);

    const double next_pos_x = pos_x + halfway_v * cos_halfway_theta * dt_;
    const double next_pos_y = pos_y + halfway_v * sin_halfway_theta * dt_;
    const double next_theta = halfway_theta + half_dtheta;
    const double next_v = halfway_v + half_dv;
    const double next_kappa = halfway_kappa + half_dkappa;
    const double next_a = halfway_a + half_da;
    const double next_s =
        s + v * dt_ + 0.5 * a * kSquareDt + j * kCubicDt / 6.0;

    StateSetX(next_pos_x, &fd->value);
    StateSetY(next_pos_y, &fd->value);
    StateSetTheta(next_theta, &fd->value);
    StateSetV(next_v, &fd->value);
    StateSetKappa(next_kappa, &fd->value);
    StateSetA(next_a, &fd->value);
    StateSetS(next_s, &fd->value);

    DFDxType &dfdx = fd->dfdx;
    dfdx = DFDxType::Zero();
    dfdx(kStateXIndex, kStateXIndex) = 1.0;
    dfdx(kStateXIndex, kStateThetaIndex) = -halfway_v * sin_halfway_theta * dt_;
    dfdx(kStateXIndex, kStateVIndex) =
        cos_halfway_theta * dt_ -
        halfway_v * sin_halfway_theta * 0.5 * halfway_kappa * kSquareDt;
    dfdx(kStateXIndex, kStateKappaIndex) =
        -halfway_v * sin_halfway_theta * 0.5 * halfway_v * kSquareDt;
    dfdx(kStateXIndex, kStateAIndex) =
        cos_halfway_theta * 0.5 * kSquareDt -
        halfway_v * sin_halfway_theta * 0.25 * halfway_kappa * kCubicDt;
    dfdx(kStateYIndex, kStateYIndex) = 1.0;
    dfdx(kStateYIndex, kStateThetaIndex) = halfway_v * cos_halfway_theta * dt_;
    dfdx(kStateYIndex, kStateVIndex) =
        sin_halfway_theta * dt_ +
        halfway_v * cos_halfway_theta * 0.5 * halfway_kappa * kSquareDt;
    dfdx(kStateYIndex, kStateKappaIndex) =
        halfway_v * cos_halfway_theta * 0.5 * halfway_v * kSquareDt;
    dfdx(kStateYIndex, kStateAIndex) =
        sin_halfway_theta * 0.5 * kSquareDt +
        halfway_v * cos_halfway_theta * 0.25 * halfway_kappa * kCubicDt;
    dfdx(kStateThetaIndex, kStateThetaIndex) = 1.0;
    dfdx(kStateThetaIndex, kStateVIndex) = halfway_kappa * dt_;
    dfdx(kStateThetaIndex, kStateKappaIndex) = halfway_v * dt_;
    dfdx(kStateThetaIndex, kStateAIndex) = 0.5 * halfway_kappa * kSquareDt;
    dfdx(kStateVIndex, kStateVIndex) = 1.0;
    dfdx(kStateVIndex, kStateAIndex) = dt_;
    dfdx(kStateKappaIndex, kStateKappaIndex) = 1.0;
    dfdx(kStateAIndex, kStateAIndex) = 1.0;
    dfdx(kStateSIndex, kStateVIndex) = dt_;
    dfdx(kStateSIndex, kStateAIndex) = 0.5 * kSquareDt;
    dfdx(kStateSIndex, kStateSIndex) = 1.0;

    DFDuType &dfdu = fd->dfdu;
    dfdu = DFDuType::Zero();
    dfdu(kStateXIndex, kControlPsiIndex) =
        -halfway_v * sin_halfway_theta * 0.25 * halfway_v * kCubicDt;
    dfdu(kStateXIndex, kControlJIndex) =
        cos_halfway_theta * 0.25 * kCubicDt -
        halfway_v * sin_halfway_theta * 0.125 * halfway_kappa * Sqr(kSquareDt);
    dfdu(kStateYIndex, kControlPsiIndex) =
        halfway_v * cos_halfway_theta * 0.25 * halfway_v * kCubicDt;
    dfdu(kStateYIndex, kControlJIndex) =
        sin_halfway_theta * 0.25 * kCubicDt +
        halfway_v * cos_halfway_theta * 0.125 * halfway_kappa * Sqr(kSquareDt);
    dfdu(kStateThetaIndex, kControlPsiIndex) = 0.5 * halfway_v * kSquareDt;
    dfdu(kStateThetaIndex, kControlJIndex) = 0.25 * halfway_kappa * kCubicDt;
    dfdu(kStateVIndex, kControlJIndex) = 0.5 * kSquareDt;
    dfdu(kStateKappaIndex, kControlPsiIndex) = dt_;
    dfdu(kStateAIndex, kControlJIndex) = dt_;
    dfdu(kStateSIndex, kControlJIndex) = kCubicDt / 6.0;

    DDFDxDxType &ddfdxdx = fd->ddfdxdx;
    for (int i = 0; i < kStateSize; ++i) ddfdxdx[i] = DDGDxDxType::Zero();
    DDFDuDxType &ddfdudx = fd->ddfdudx;
    for (int i = 0; i < kStateSize; ++i) ddfdudx[i] = DDGDuDxType::Zero();
    DDFDuDuType &ddfdudu = fd->ddfdudu;
    for (int i = 0; i < kStateSize; ++i) ddfdudu[i] = DDGDuDuType::Zero();

    // Currently we don't need 2rd derivatives, besides model with 2rd
    // derivatives can't pass unit test.
    if (true) return;
    ddfdxdx[kStateXIndex](kStateThetaIndex, kStateThetaIndex) =
        -halfway_v * cos_halfway_theta * dt_;
    ddfdxdx[kStateXIndex](kStateThetaIndex, kStateVIndex) =
        -sin_halfway_theta * dt_ -
        halfway_v * cos_halfway_theta * 0.5 * halfway_kappa * kSquareDt;
    ddfdxdx[kStateXIndex](kStateVIndex, kStateThetaIndex) =
        ddfdxdx[kStateXIndex](kStateThetaIndex, kStateVIndex);
    ddfdxdx[kStateXIndex](kStateVIndex, kStateVIndex) =
        -sin_halfway_theta * halfway_kappa * kSquareDt -
        halfway_v * cos_halfway_theta * 0.25 * Sqr(halfway_kappa) * kCubicDt;
    ddfdxdx[kStateXIndex](kStateAIndex, kStateThetaIndex) =
        -0.5 * sin_halfway_theta * kSquareDt -
        halfway_v * cos_halfway_theta * 0.25 * halfway_kappa * kCubicDt;
    ddfdxdx[kStateXIndex](kStateAIndex, kStateVIndex) =
        -sin_halfway_theta * 0.5 * halfway_kappa * kCubicDt -
        halfway_v * cos_halfway_theta * 0.5 * 0.25 * Sqr(halfway_kappa) *
            Sqr(kSquareDt);
    ddfdxdx[kStateXIndex](kStateAIndex, kStateAIndex) =
        -sin_halfway_theta * 0.25 * halfway_kappa * Sqr(kSquareDt) -
        halfway_v * cos_halfway_theta * Sqr(0.25 * halfway_kappa) * kSquareDt *
            kCubicDt;

    ddfdxdx[kStateYIndex](kStateThetaIndex, kStateThetaIndex) =
        -halfway_v * sin_halfway_theta * dt_;
    ddfdxdx[kStateYIndex](kStateThetaIndex, kStateVIndex) =
        cos_halfway_theta * dt_ -
        halfway_v * sin_halfway_theta * 0.5 * halfway_kappa * kSquareDt;
    ddfdxdx[kStateYIndex](kStateVIndex, kStateThetaIndex) =
        ddfdxdx[kStateYIndex](kStateThetaIndex, kStateVIndex);
    ddfdxdx[kStateYIndex](kStateVIndex, kStateVIndex) =
        cos_halfway_theta * halfway_kappa * kSquareDt -
        halfway_v * sin_halfway_theta * 0.25 * Sqr(halfway_kappa) * kCubicDt;
    ddfdxdx[kStateYIndex](kStateAIndex, kStateThetaIndex) =
        0.5 * cos_halfway_theta * kSquareDt -
        halfway_v * sin_halfway_theta * 0.25 * halfway_kappa * kCubicDt;
    ddfdxdx[kStateYIndex](kStateAIndex, kStateVIndex) =
        cos_halfway_theta * 0.5 * halfway_kappa * kCubicDt -
        halfway_v * sin_halfway_theta * 0.5 * 0.25 * Sqr(halfway_kappa) *
            Sqr(kSquareDt);
    ddfdxdx[kStateYIndex](kStateAIndex, kStateAIndex) =
        cos_halfway_theta * 0.25 * halfway_kappa * Sqr(kSquareDt) -
        halfway_v * sin_halfway_theta * Sqr(0.25 * halfway_kappa) * kSquareDt *
            kCubicDt;

    ddfdxdx[kStateXIndex](kStateKappaIndex, kStateThetaIndex) =
        -halfway_v * cos_halfway_theta * 0.5 * halfway_v * kSquareDt;
    ddfdxdx[kStateXIndex](kStateKappaIndex, kStateVIndex) =
        -sin_halfway_theta * halfway_v * kSquareDt -
        halfway_v * cos_halfway_theta * 0.25 * halfway_kappa * halfway_v *
            kCubicDt;
    ddfdxdx[kStateXIndex](kStateKappaIndex, kStateAIndex) =
        -halfway_v * sin_halfway_theta * 0.5 * kCubicDt -
        halfway_v * cos_halfway_theta * 0.125 * halfway_v * halfway_kappa *
            Sqr(kSquareDt);
    ddfdxdx[kStateYIndex](kStateKappaIndex, kStateThetaIndex) =
        -halfway_v * sin_halfway_theta * 0.5 * halfway_v * kSquareDt;
    ddfdxdx[kStateYIndex](kStateKappaIndex, kStateVIndex) =
        cos_halfway_theta * halfway_v * kSquareDt -
        halfway_v * sin_halfway_theta * 0.25 * halfway_kappa * halfway_v *
            kCubicDt;
    ddfdxdx[kStateYIndex](kStateKappaIndex, kStateAIndex) =
        halfway_v * cos_halfway_theta * 0.5 * kCubicDt -
        halfway_v * sin_halfway_theta * 0.125 * halfway_v * halfway_kappa *
            Sqr(kSquareDt);
    ddfdxdx[kStateThetaIndex](kStateKappaIndex, kStateVIndex) = dt_;
    ddfdxdx[kStateThetaIndex](kStateKappaIndex, kStateAIndex) = 0.5 * kSquareDt;

    ddfdxdx[kStateXIndex](kStateKappaIndex, kStateKappaIndex) =
        -halfway_v * cos_halfway_theta * Sqr(0.5 * halfway_v) * kCubicDt;
    ddfdxdx[kStateYIndex](kStateKappaIndex, kStateKappaIndex) =
        -halfway_v * sin_halfway_theta * Sqr(0.5 * halfway_v) * kCubicDt;
  }

  void EvaluateFDerivativesForAllSteps(
      const StatesType &xs, const ControlsType &us,
      std::array<FDerivatives, kHorizon> *fds) const {
    for (int k = 0; k < kHorizon; ++k) {
      EvaluateFDerivatives(k, GetStateAtStep(xs, k), GetControlAtStep(us, k),
                           &(*fds)[k]);
    }
  }

  static FType EvaluateF(int k, const StateType &x, const ControlType &u,
                         double dt) {
    const double pos_x = StateGetX(x);
    const double pos_y = StateGetY(x);
    const double theta = StateGetTheta(x);
    const double v = StateGetV(x);
    const double kappa = StateGetKappa(x);
    const double a = StateGetA(x);
    const double s = StateGetS(x);
    const double psi = ControlGetPsi(u);
    const double j = ControlGetJ(u);

    const double half_da = 0.5 * j * dt;
    const double halfway_a = a + half_da;
    const double half_dv = 0.5 * halfway_a * dt;
    const double halfway_v = v + half_dv;
    const double half_dkappa = 0.5 * psi * dt;
    const double halfway_kappa = kappa + half_dkappa;
    const double half_dtheta = 0.5 * halfway_v * halfway_kappa * dt;
    const double halfway_theta = theta + half_dtheta;
    const double cos_halfway_theta = std::cos(halfway_theta);
    const double sin_halfway_theta = std::sin(halfway_theta);

    constexpr double kMaxValue = 1e50;

    const double next_pos_x = std::clamp(
        pos_x + halfway_v * cos_halfway_theta * dt, -kMaxValue, kMaxValue);
    const double next_pos_y = std::clamp(
        pos_y + halfway_v * sin_halfway_theta * dt, -kMaxValue, kMaxValue);
    const double next_theta =
        std::clamp(halfway_theta + half_dtheta, -kMaxValue, kMaxValue);
    const double next_v =
        std::clamp(halfway_v + half_dv, -kMaxValue, kMaxValue);
    const double next_kappa =
        std::clamp(halfway_kappa + half_dkappa, -kMaxValue, kMaxValue);
    const double next_a =
        std::clamp(halfway_a + half_da, -kMaxValue, kMaxValue);
    const double next_s =
        std::clamp(s + v * dt + 0.5 * a * Sqr(dt) + j * Cube(dt) / 6.0,
                   -kMaxValue, kMaxValue);

    FType next_x = FType::Zero();
    StateSetX(next_pos_x, &next_x);
    StateSetY(next_pos_y, &next_x);
    StateSetTheta(next_theta, &next_x);
    StateSetV(next_v, &next_x);
    StateSetKappa(next_kappa, &next_x);
    StateSetA(next_a, &next_x);
    StateSetS(next_s, &next_x);
    return next_x;
  }

  FType EvaluateF(int k, const StateType &x, const ControlType &u) const {
    return EvaluateF(k, x, u, dt_);
  }

  // G and derivatives.
  struct GDerivatives {
    GType value;
    DGDxType dgdx;
    DGDuType dgdu;
    DDGDxDxType ddgdxdx;
    DDGDuDxType ddgdudx;
    DDGDuDuType ddgdudu;
  };

  void EvaluateGDerivatives(int k, const StateType &x, const ControlType &u,
                            GDerivatives *gd) const {
    gd->value = 0.0;
    gd->dgdx = DGDxType::Zero();
    gd->dgdu = DGDuType::Zero();
    gd->ddgdxdx = DDGDxDxType::Zero();
    gd->ddgdudx = DDGDuDxType::Zero();
    gd->ddgdudu = DDGDuDuType::Zero();
    for (const auto &cost : costs_) {
      gd->value += cost->EvaluateG(k, x, u);
      gd->dgdx += cost->EvaluateDGDx(k, x, u);
      gd->dgdu += cost->EvaluateDGDu(k, x, u);
      gd->ddgdxdx += cost->EvaluateDDGDxDx(k, x, u);
      gd->ddgdudx += cost->EvaluateDDGDuDx(k, x, u);
      gd->ddgdudu += cost->EvaluateDDGDuDu(k, x, u);
    }
  }

  void EvaluateGDerivativesForAllSteps(
      const StatesType &xs, const ControlsType &us,
      std::array<GDerivatives, kHorizon> *gds) const {
    for (int k = 0; k < kHorizon; ++k) {
      EvaluateGDerivatives(k, GetStateAtStep(xs, k), GetControlAtStep(us, k),
                           &(*gds)[k]);
    }
  }

  void AddGDerivatives(int k, const StateType &x, const ControlType &u,
                       GDerivatives *gd) const {
    gd->value = 0.0;
    gd->dgdx = DGDxType::Zero();
    gd->dgdu = DGDuType::Zero();
    gd->ddgdxdx = DDGDxDxType::Zero();
    gd->ddgdudx = DDGDuDxType::Zero();
    gd->ddgdudu = DDGDuDuType::Zero();
    for (const auto &cost : costs_) {
      gd->value += cost->EvaluateG(k, x, u);
      cost->AddDGDx(k, x, u, &gd->dgdx);
      cost->AddDGDu(k, x, u, &gd->dgdu);
      cost->AddDDGDxDx(k, x, u, &gd->ddgdxdx);
      cost->AddDDGDuDx(k, x, u, &gd->ddgdudx);
      cost->AddDDGDuDu(k, x, u, &gd->ddgdudu);
    }
  }

  void AddGDerivativesForAllSteps(
      const StatesType &xs, const ControlsType &us,
      std::array<GDerivatives, kHorizon> *gds) const {
    for (int k = 0; k < kHorizon; ++k) {
      AddGDerivatives(k, GetStateAtStep(xs, k), GetControlAtStep(us, k),
                      &(*gds)[k]);
    }
  }

  // Only post-process longitudinal control, try to keep a_postprocess_k+1 =
  // a_origin_k+1
  ControlType PostProcessLonU(ControlType u, const StateType &x,
                              const StateType &x_next,
                              ClampInfo *control_clamp_info,
                              bool forward) const {
    if (!enable_post_process_) return u;
    // TODO(renjie): read the parameters from function arguments.
    double min_speed = kMinSpeed;
    double max_speed = kMaxSpeed;
    if (!forward) {
      min_speed = -kMaxSpeed;
      max_speed = -kMinSpeed;
    }
    const ControlType u_origin = u;
    static const double max_accel =
        motion_constraint_params_->max_acceleration();
    static const double max_decel =
        motion_constraint_params_->max_deceleration();
    static const double max_accel_jerk =
        motion_constraint_params_->max_accel_jerk();
    static const double max_decel_jerk =
        motion_constraint_params_->max_decel_jerk();

    const double j_fit = (StateGetA(x_next) - StateGetA(x)) / dt_;
    if (j_fit <= max_accel_jerk && j_fit >= max_decel_jerk &&
        StateGetA(x_next) <= max_accel && StateGetA(x_next) >= max_decel)
      ControlSetJ(j_fit, &u);

    ControlSetJ(std::clamp(ControlGetJ(u), (max_decel - StateGetA(x)) / dt_,
                           (max_accel - StateGetA(x)) / dt_),
                &u);
    ControlSetJ(
        std::clamp(
            ControlGetJ(u),
            ((min_speed - StateGetV(x)) / dt_ - StateGetA(x)) / (dt_ * 0.5),
            ((max_speed - StateGetV(x)) / dt_ - StateGetA(x)) / (dt_ * 0.5)),
        &u);
    ControlSetJ(std::clamp(ControlGetJ(u), max_decel_jerk, max_accel_jerk), &u);
    constexpr double kEps = 1e-7;
    if (std::abs(u_origin[kControlJIndex] - u[kControlJIndex]) > kEps) {
      control_clamp_info->indice.emplace_back(kControlJIndex);
      control_clamp_info->values.emplace_back(u_origin[kControlJIndex],
                                              u[kControlJIndex]);
    }
    return u;
  }

  // Only Post-process longitudinal state.
  StateType PostProcessLonX(StateType x, const ControlType &u,
                            ClampInfo *state_clamp_info) const {
    if (!enable_post_process_) return x;
    // TODO(renjie): read the parameters from function arguments.
    const StateType x_origin = x;
    static const double max_accel =
        motion_constraint_params_->max_acceleration();
    static const double max_decel =
        motion_constraint_params_->max_deceleration();
    static const double max_accel_jerk =
        motion_constraint_params_->max_accel_jerk();
    static const double max_decel_jerk =
        motion_constraint_params_->max_decel_jerk();

    StateSetV(std::clamp(StateGetV(x), kMinSpeed, kMaxSpeed), &x);
    StateSetA(
        std::clamp(
            StateGetA(x),
            (kMinSpeed - StateGetV(x)) / dt_ - max_accel_jerk * dt_ / 2.0,
            (kMaxSpeed - StateGetV(x)) / dt_ - max_decel_jerk * dt_ / 2.0),
        &x);
    StateSetA(std::clamp(StateGetA(x), max_decel, max_accel), &x);
    constexpr double kEps = 1e-7;
    for (const int index : {kStateVIndex, kStateAIndex}) {
      if (std::abs(x_origin[index] - x[index]) > kEps) {
        state_clamp_info->indice.emplace_back(index);
        state_clamp_info->values.emplace_back(x_origin[index], x[index]);
      }
    }
    return x;
  }

  // Set full stop trajectory from from k_fixed_start. v, a, psi and j
  // should be zero.
  void SetFullStopTrajectoryFromStep(int k_fixed_start,
                                     const StateType &x_fixed, StatesType *xs,
                                     ControlsType *us) const {
    if (!enable_post_process_) return;
    for (int k = k_fixed_start; k < kHorizon; ++k) {
      SetStateAtStep(x_fixed, k, xs);
      set_v(0.0, k, xs);
      set_a(0.0, k, xs);
      set_j(0.0, k, us);
      set_psi(0.0, k, us);
    }
  }

  bool CheckDu(const ControlsType &dus, const std::string &prefix) const {
    constexpr double kDuLimit = 1e6;
    for (int k = 0; k < dus.size(); ++k) {
      const double du = dus[k];
      if (du > kDuLimit || du < -kDuLimit) {
        QEVENT_EVERY_N_SECONDS("runbing", prefix + "_check_u", 0.2,
                               [&](QEvent *qevent) {
                                 qevent->AddField("index", k)
                                     .AddField("value", du)
                                     .AddField("limit", kDuLimit);
                               });
        return false;
      }
    }
    return true;
  }

  // Conversion between states and trajectories.
  static StateType FitInitialState(
      const std::vector<TrajectoryPoint> &init_traj_points) {
    CHECK_GE(init_traj_points.size(), 2);
    const TrajectoryPoint &p0 = init_traj_points[0];
    StateType x;
    StateSetX(p0.pos().x(), &x);
    StateSetY(p0.pos().y(), &x);
    StateSetTheta(p0.theta(), &x);
    StateSetV(p0.v(), &x);
    StateSetKappa(p0.kappa(), &x);
    StateSetA(p0.a(), &x);
    StateSetS(p0.s(), &x);
    return x;
  }

  static StatesType FitState(
      const std::vector<TrajectoryPoint> &init_traj_points) {
    StatesType xs;
    for (int k = 0; k < kHorizon; ++k) {
      const TrajectoryPoint &p = init_traj_points[k];
      StateType x;
      StateSetX(p.pos().x(), &x);
      StateSetY(p.pos().y(), &x);
      StateSetTheta(p.theta(), &x);
      StateSetV(p.v(), &x);
      StateSetKappa(p.kappa(), &x);
      StateSetA(p.a(), &x);
      StateSetS(p.s(), &x);
      SetStateAtStep(x, k, &xs);
    }
    return xs;
  }

  // Deduce control from traj points. If the v and theta of the traj points are
  // accurate, the control a and kappa computed here are exact. However if the
  // trajectory's v and theta come from ComputeTrajectoryDerivatives(), they're
  // not guaranteed to be the same as those that correspond to the roll out done
  // by ThirdOrderBicycle.
  static ControlsType FitControl(
      const std::vector<TrajectoryPoint> &traj_points, const StateType &x0) {
    CHECK_EQ(traj_points.size(), kHorizon);
    const double dt = (traj_points.back().t() - traj_points.front().t()) /
                      static_cast<double>(kHorizon - 1);
    ControlsType us = ControlsType::Zero();
    for (int k = 0; k + 1 < kHorizon; ++k) {
      const TrajectoryPoint &p0 = traj_points[k];
      const TrajectoryPoint &p1 = traj_points[k + 1];
      const double psi = (p1.kappa() - p0.kappa()) / dt;
      const double j = (p1.a() - p0.a()) / dt;
      set_psi(psi, k, &us);
      set_j(j, k, &us);
    }
    if (kHorizon > 1) {
      set_psi(psi(us, kHorizon - 2), kHorizon - 1, &us);
      set_j(j(us, kHorizon - 2), kHorizon - 1, &us);
    }
    return us;
  }

  // Roll out the control sequence from an initial state.
  static StatesType RollOutControl(const StateType &x0,
                                   const ControlsType &us) {
    StatesType xs;
    SetStateAtStep(x0, 0, &xs);
    for (int k = 0; k + 1 < kHorizon; ++k) {
      SetStateAtStep(
          EvaluateF(k, GetStateAtStep(xs, k), GetControlAtStep(us, k)), k + 1,
          &xs);
    }
    return xs;
  }

  static void ExtractTrajectoryPoint(int k, const StateType &x,
                                     const ControlType &u, double dt,
                                     TrajectoryPoint *traj_point) {
    traj_point->set_t(dt * k);
    traj_point->set_pos(StateGetPos(x));
    traj_point->set_theta(StateGetTheta(x));
    traj_point->set_v(StateGetV(x));
    traj_point->set_kappa(StateGetKappa(x));
    traj_point->set_a(StateGetA(x));
    traj_point->set_s(StateGetS(x));
    traj_point->set_psi(ControlGetPsi(u));
    traj_point->set_j(ControlGetJ(u));
  }

  void ExtractTrajectoryPoint(int k, const StateType &x, const ControlType &u,
                              TrajectoryPoint *traj_point) const {
    ExtractTrajectoryPoint(k, x, u, dt_, traj_point);
  }

  using CostType = Cost<ThirdOrderBicycle>;
  void AddCost(std::unique_ptr<CostType> cost) {
    costs_.push_back(std::move(cost));
  }

  const std::vector<std::unique_ptr<CostType>> &costs() const { return costs_; }

  using CostHelperType = CostHelper<ThirdOrderBicycle>;
  void AddCostHelper(std::unique_ptr<CostHelperType> helper) {
    cost_helpers_.push_back(std::move(helper));
  }
  const std::vector<std::unique_ptr<CostHelperType>> &cost_helpers() const {
    return cost_helpers_;
  }

  // Normalization scales: a rough scale to bring the different state and
  // control DoFs to comparable magnitudes, e.g. for regularization purposes.
  static const std::array<double, kStateSize> &state_scales() {
    static const std::array<double, kStateSize> scales = {1.0, 1.0,  5.0,
                                                          0.2, 10.0, 0.4};
    return scales;
  }

  static const std::array<double, kControlSize> &control_scales() {
    static const std::array<double, kControlSize> scales = {20.0, 1.0};
    return scales;
  }

  // Control step toward a pure pursuit target.
  ControlType PurePursuitController(const StateType &x,
                                    const StateType &longitudinal_target,
                                    const StateType &lateral_target,
                                    int longitudinal_look_ahead_steps,
                                    double lateral_look_ahead_dist) {
    const Vec2d pos = StateGetPos(x);

    const double v_target = (StateGetS(longitudinal_target) - StateGetS(x)) /
                            (longitudinal_look_ahead_steps * dt_);
    const double a = (v_target - StateGetV(x)) / dt_;
    const double j = (a - StateGetA(x)) / dt_;

    const Vec2d lateral_target_pos = StateGetPos(lateral_target);
    const double theta = StateGetTheta(x);
    const double alpha = Vec2d(lateral_target_pos - pos).FastAngle() - theta;
    const double kappa = 2.0 * fast_math::Sin(alpha) / lateral_look_ahead_dist;
    const double psi = (kappa - StateGetKappa(x)) / dt_;

    ControlType u;
    ControlSetPsi(psi, &u);
    ControlSetJ(j, &u);
    return u;
  }

  static std::string ControlIndexToName(int index) {
    switch (index) {
      case kControlPsiIndex:
        return "psi";
      case kControlJIndex:
        return "j";
      default:
        QLOG(FATAL) << "Not Control Index: [" << index << "]";
        return "";
    }
  }

  static std::string StateIndexToName(int index) {
    switch (index) {
      case kStateXIndex:
        return "x";
      case kStateYIndex:
        return "y";
      case kStateThetaIndex:
        return "theta";
      case kStateVIndex:
        return "v";
      case kStateKappaIndex:
        return "kappa";
      case kStateAIndex:
        return "a";
      case kStateSIndex:
        return "s";
      default:
        QLOG(FATAL) << "Not State Index: [" << index << "]";
        return "";
    }
  }

 private:
  std::vector<std::unique_ptr<CostType>> costs_;
  std::vector<std::unique_ptr<CostHelperType>> cost_helpers_;
  const MotionConstraintParamsProto *motion_constraint_params_;
  const VehicleGeometryParamsProto *veh_geo_params_;
  const VehicleDriveParamsProto *veh_drive_params_;
  double dt_;
  bool enable_post_process_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_THIRD_ORDER_BICYCLE_H_
