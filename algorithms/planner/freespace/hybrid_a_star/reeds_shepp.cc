#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"

#include <cmath>
#include <limits>

#include "glog/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

std::pair<double, double> CalcTauOmega(double u, double v, double xi,
                                       double eta, double phi) {
  const double sin_u = fast_math::Sin(u);
  const double cos_u = fast_math::Cos(u);
  const double delta = NormalizeAngle(u - v);
  const double sin_delta = fast_math::SinNormalized(delta);
  const double cos_delta = fast_math::CosNormalized(delta);

  const double A = sin_u - sin_delta;
  const double B = cos_u - cos_delta - 1.0;
  const double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  const double t2 = 2.0 * (cos_delta - fast_math::Cos(v) - cos_u) + 3.0;
  double tau = 0.0;
  if (t2 < 0.0) {
    tau = NormalizeAngle(t1 + M_PI);
  } else {
    tau = NormalizeAngle(t1);
  }
  const double omega = NormalizeAngle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  const double r = std::sqrt(x * x + y * y);
  const double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

// Please refer to paper "Optimal paths for a car that goes both forwards and
// backwards" for the following formulas.

// Formula 8.1
void LSL(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x - sin_phi, y - 1.0 + cos_phi);
  const double u = polar.first;
  const double t = polar.second;
  if (t >= -kZero) {
    const double v = NormalizeAngle(phi - t);
    if (v >= -kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.2
void LSR(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x + sin_phi, y - 1.0 - cos_phi);
  const double u1 = polar.first * polar.first;
  const double t1 = polar.second;
  if (u1 >= 4.0) {
    const double u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, u);
    const double t = NormalizeAngle(t1 + theta);
    const double v = NormalizeAngle(t - phi);
    if (t >= -kZero && v >= -kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.3 / 8.4
void LRL(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x - sin_phi, y - 1.0 + cos_phi);
  const double u1 = polar.first;
  const double t1 = polar.second;
  if (u1 <= 4.0) {
    const double u = -2.0 * std::asin(0.25 * u1);
    const double t = NormalizeAngle(t1 + 0.5 * u + M_PI);
    const double v = NormalizeAngle(phi - t + u);
    if (t >= -kZero && u <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Additional motion not included in paper.
void SLS(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const double phi_mod = NormalizeAngle(phi);
  const double tan_phi_mod = std::tan(phi_mod);
  const double tan_half_phi_mod = std::tan(phi_mod / 2.0);
  constexpr double epsilon = 1e-1;
  if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    const double xd = -y / tan_phi_mod + x;
    const double t = xd - tan_half_phi_mod;
    const double u = phi_mod;
    const double v = std::sqrt((x - xd) * (x - xd) + y * y) - tan_half_phi_mod;
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    const double xd = -y / tan_phi_mod + x;
    const double t = xd - tan_half_phi_mod;
    const double u = phi_mod;
    const double v = -std::sqrt((x - xd) * (x - xd) + y * y) - tan_half_phi_mod;
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  }
}

// Formula 8.7
void LRLRn(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    const double u = std::acos(rho);
    const auto tau_omega = CalcTauOmega(u, -u, xi, eta, phi);
    if (tau_omega.first >= -kZero && tau_omega.second <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = tau_omega.first;
      param->v = tau_omega.second;
    }
  }
}

// Formula 8.8
void LRLRp(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho <= 1.0 && rho >= 0.0) {
    const double u = -std::acos(rho);
    if (u >= -0.5 * M_PI) {
      const auto tau_omega = CalcTauOmega(u, u, xi, eta, phi);
      if (tau_omega.first >= -kZero && tau_omega.second >= -kZero) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

// Formula 8.9
void LRSL(double x, double y, double phi, double sin_phi, double cos_phi,
          RSPParam* param) {
  const double xi = x - sin_phi;
  const double eta = y - 1.0 + cos_phi;
  const auto polar = Cartesian2Polar(xi, eta);
  const double rho = polar.first;
  const double theta = polar.second;
  if (rho >= 2.0) {
    const double r = std::sqrt(rho * rho - 4.0);
    const double u = 2.0 - r;
    const double t = NormalizeAngle(theta + std::atan2(r, -2.0));
    const double v = NormalizeAngle(phi - 0.5 * M_PI - t);
    if (t >= -kZero && u <= kZero && v <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.10
void LRSR(double x, double y, double phi, double sin_phi, double cos_phi,
          RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const auto polar = Cartesian2Polar(-eta, xi);
  const double rho = polar.first;
  const double theta = polar.second;
  if (rho >= 2.0) {
    const double t = theta;
    const double u = 2.0 - rho;
    const double v = NormalizeAngle(t + 0.5 * M_PI - phi);
    if (t >= -kZero && u <= kZero && v <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.11
void LRSLR(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const auto polar = Cartesian2Polar(xi, eta);
  const double rho = polar.first;
  if (rho >= 2.0) {
    const double u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= kZero) {
      const double t = NormalizeAngle(
          atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      const double v = NormalizeAngle(t - phi);
      if (t >= -kZero && v >= -kZero) {
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
      }
    }
  }
}

bool SetRSP(int size, const double* lengths, const char* types,
            std::vector<ReedSheppPath>* all_possible_paths) {
  double total_length = 0.0;
  for (int i = 0; i < size; ++i) {
    total_length += std::abs(lengths[i]);
  }
  if (total_length == 0.0) {
    return false;
  }
  ReedSheppPath path;
  std::vector<double> length_vec(lengths, lengths + size);
  std::vector<char> type_vec(types, types + size);
  path.segs_lengths = std::move(length_vec);
  path.segs_types = std::move(type_vec);
  path.total_length = total_length;
  all_possible_paths->push_back(std::move(path));
  return true;
}

bool SCS(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam SLS_param;
  SLS(x, y, phi, sin_phi, cos_phi, &SLS_param);
  const double SLS_lengths[3] = {SLS_param.t, SLS_param.u, SLS_param.v};
  const char SLS_types[] = "SLS";
  if (SLS_param.flag &&
      !SetRSP(3, SLS_lengths, SLS_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with SLS_param";
    return false;
  }

  RSPParam SRS_param;
  SLS(x, -y, -phi, -sin_phi, cos_phi, &SRS_param);
  const double SRS_lengths[3] = {SRS_param.t, SRS_param.u, SRS_param.v};
  const char SRS_types[] = "SRS";
  if (SRS_param.flag &&
      !SetRSP(3, SRS_lengths, SRS_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with SRS_param";
    return false;
  }
  return true;
}

bool CSC(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LSL1_param;
  LSL(x, y, phi, sin_phi, cos_phi, &LSL1_param);
  const double LSL1_lengths[3] = {LSL1_param.t, LSL1_param.u, LSL1_param.v};
  const char LSL1_types[] = "LSL";
  if (LSL1_param.flag &&
      !SetRSP(3, LSL1_lengths, LSL1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL_param";
    return false;
  }

  RSPParam LSL2_param;
  LSL(-x, y, -phi, -sin_phi, cos_phi, &LSL2_param);
  const double LSL2_lengths[3] = {-LSL2_param.t, -LSL2_param.u, -LSL2_param.v};
  const char LSL2_types[] = "LSL";
  if (LSL2_param.flag &&
      !SetRSP(3, LSL2_lengths, LSL2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL2_param";
    return false;
  }

  RSPParam LSL3_param;
  LSL(x, -y, -phi, -sin_phi, cos_phi, &LSL3_param);
  const double LSL3_lengths[3] = {LSL3_param.t, LSL3_param.u, LSL3_param.v};
  const char LSL3_types[] = "RSR";
  if (LSL3_param.flag &&
      !SetRSP(3, LSL3_lengths, LSL3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL3_param";
    return false;
  }

  RSPParam LSL4_param;
  LSL(-x, -y, phi, sin_phi, cos_phi, &LSL4_param);
  const double LSL4_lengths[3] = {-LSL4_param.t, -LSL4_param.u, -LSL4_param.v};
  const char LSL4_types[] = "RSR";
  if (LSL4_param.flag &&
      !SetRSP(3, LSL4_lengths, LSL4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL4_param";
    return false;
  }

  RSPParam LSR1_param;
  LSR(x, y, phi, sin_phi, cos_phi, &LSR1_param);
  const double LSR1_lengths[3] = {LSR1_param.t, LSR1_param.u, LSR1_param.v};
  const char LSR1_types[] = "LSR";
  if (LSR1_param.flag &&
      !SetRSP(3, LSR1_lengths, LSR1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR1_param";
    return false;
  }

  RSPParam LSR2_param;
  LSR(-x, y, -phi, -sin_phi, cos_phi, &LSR2_param);
  const double LSR2_lengths[3] = {-LSR2_param.t, -LSR2_param.u, -LSR2_param.v};
  const char LSR2_types[] = "LSR";
  if (LSR2_param.flag &&
      !SetRSP(3, LSR2_lengths, LSR2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR2_param";
    return false;
  }

  RSPParam LSR3_param;
  LSR(x, -y, -phi, -sin_phi, cos_phi, &LSR3_param);
  const double LSR3_lengths[3] = {LSR3_param.t, LSR3_param.u, LSR3_param.v};
  const char LSR3_types[] = "RSL";
  if (LSR3_param.flag &&
      !SetRSP(3, LSR3_lengths, LSR3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR3_param";
    return false;
  }

  RSPParam LSR4_param;
  LSR(-x, -y, phi, sin_phi, cos_phi, &LSR4_param);
  const double LSR4_lengths[3] = {-LSR4_param.t, -LSR4_param.u, -LSR4_param.v};
  const char LSR4_types[] = "RSL";
  if (LSR4_param.flag &&
      !SetRSP(3, LSR4_lengths, LSR4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR4_param";
    return false;
  }
  return true;
}

bool CCC(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRL1_param;
  LRL(x, y, phi, sin_phi, cos_phi, &LRL1_param);
  const double LRL1_lengths[3] = {LRL1_param.t, LRL1_param.u, LRL1_param.v};
  const char LRL1_types[] = "LRL";
  if (LRL1_param.flag &&
      !SetRSP(3, LRL1_lengths, LRL1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL_param";
    return false;
  }

  RSPParam LRL2_param;
  LRL(-x, y, -phi, -sin_phi, cos_phi, &LRL2_param);
  const double LRL2_lengths[3] = {-LRL2_param.t, -LRL2_param.u, -LRL2_param.v};
  const char LRL2_types[] = "LRL";
  if (LRL2_param.flag &&
      !SetRSP(3, LRL2_lengths, LRL2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL2_param";
    return false;
  }

  RSPParam LRL3_param;
  LRL(x, -y, -phi, -sin_phi, cos_phi, &LRL3_param);
  const double LRL3_lengths[3] = {LRL3_param.t, LRL3_param.u, LRL3_param.v};
  const char LRL3_types[] = "RLR";
  if (LRL3_param.flag &&
      !SetRSP(3, LRL3_lengths, LRL3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL3_param";
    return false;
  }

  RSPParam LRL4_param;
  LRL(-x, -y, phi, sin_phi, cos_phi, &LRL4_param);
  const double LRL4_lengths[3] = {-LRL4_param.t, -LRL4_param.u, -LRL4_param.v};
  const char LRL4_types[] = "RLR";
  if (LRL4_param.flag &&
      !SetRSP(3, LRL4_lengths, LRL4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL4_param";
    return false;
  }

  // backward
  const double xb = x * cos_phi + y * sin_phi;
  const double yb = x * sin_phi - y * cos_phi;

  RSPParam LRL5_param;
  LRL(xb, yb, phi, sin_phi, cos_phi, &LRL5_param);
  const double LRL5_lengths[3] = {LRL5_param.v, LRL5_param.u, LRL5_param.t};
  const char LRL5_types[] = "LRL";
  if (LRL5_param.flag &&
      !SetRSP(3, LRL5_lengths, LRL5_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL5_param";
    return false;
  }

  RSPParam LRL6_param;
  LRL(-xb, yb, -phi, -sin_phi, cos_phi, &LRL6_param);
  const double LRL6_lengths[3] = {-LRL6_param.v, -LRL6_param.u, -LRL6_param.t};
  const char LRL6_types[] = "LRL";
  if (LRL6_param.flag &&
      !SetRSP(3, LRL6_lengths, LRL6_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL6_param";
    return false;
  }

  RSPParam LRL7_param;
  LRL(xb, -yb, -phi, -sin_phi, cos_phi, &LRL7_param);
  const double LRL7_lengths[3] = {LRL7_param.v, LRL7_param.u, LRL7_param.t};
  const char LRL7_types[] = "RLR";
  if (LRL7_param.flag &&
      !SetRSP(3, LRL7_lengths, LRL7_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL7_param";
    return false;
  }

  RSPParam LRL8_param;
  LRL(-xb, -yb, phi, sin_phi, cos_phi, &LRL8_param);
  const double LRL8_lengths[3] = {-LRL8_param.v, -LRL8_param.u, -LRL8_param.t};
  const char LRL8_types[] = "RLR";
  if (LRL8_param.flag &&
      !SetRSP(3, LRL8_lengths, LRL8_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL8_param";
    return false;
  }
  return true;
}

bool CCCC(double x, double y, double phi, double sin_phi, double cos_phi,
          std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRLRn1_param;
  LRLRn(x, y, phi, sin_phi, cos_phi, &LRLRn1_param);
  const double LRLRn1_lengths[4] = {LRLRn1_param.t, LRLRn1_param.u,
                                    -LRLRn1_param.u, LRLRn1_param.v};
  const char LRLRn1_types[] = "LRLR";
  if (LRLRn1_param.flag &&
      !SetRSP(4, LRLRn1_lengths, LRLRn1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRLRn2_param;
  LRLRn(-x, y, -phi, -sin_phi, cos_phi, &LRLRn2_param);
  const double LRLRn2_lengths[4] = {-LRLRn2_param.t, -LRLRn2_param.u,
                                    LRLRn2_param.u, -LRLRn2_param.v};
  const char LRLRn2_types[] = "LRLR";
  if (LRLRn2_param.flag &&
      !SetRSP(4, LRLRn2_lengths, LRLRn2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn2_param";
    return false;
  }

  RSPParam LRLRn3_param;
  LRLRn(x, -y, -phi, -sin_phi, cos_phi, &LRLRn3_param);
  const double LRLRn3_lengths[4] = {LRLRn3_param.t, LRLRn3_param.u,
                                    -LRLRn3_param.u, LRLRn3_param.v};
  const char LRLRn3_types[] = "RLRL";
  if (LRLRn3_param.flag &&
      !SetRSP(4, LRLRn3_lengths, LRLRn3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn3_param";
    return false;
  }

  RSPParam LRLRn4_param;
  LRLRn(-x, -y, phi, sin_phi, cos_phi, &LRLRn4_param);
  const double LRLRn4_lengths[4] = {-LRLRn4_param.t, -LRLRn4_param.u,
                                    LRLRn4_param.u, -LRLRn4_param.v};
  const char LRLRn4_types[] = "RLRL";
  if (LRLRn4_param.flag &&
      !SetRSP(4, LRLRn4_lengths, LRLRn4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn4_param";
    return false;
  }

  RSPParam LRLRp1_param;
  LRLRp(x, y, phi, sin_phi, cos_phi, &LRLRp1_param);
  const double LRLRp1_lengths[4] = {LRLRp1_param.t, LRLRp1_param.u,
                                    LRLRp1_param.u, LRLRp1_param.v};
  const char LRLRp1_types[] = "LRLR";
  if (LRLRp1_param.flag &&
      !SetRSP(4, LRLRp1_lengths, LRLRp1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp1_param";
    return false;
  }

  RSPParam LRLRp2_param;
  LRLRp(-x, y, -phi, -sin_phi, cos_phi, &LRLRp2_param);
  const double LRLRp2_lengths[4] = {-LRLRp2_param.t, -LRLRp2_param.u,
                                    -LRLRp2_param.u, -LRLRp2_param.v};
  const char LRLRp2_types[] = "LRLR";
  if (LRLRp2_param.flag &&
      !SetRSP(4, LRLRp2_lengths, LRLRp2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp2_param";
    return false;
  }

  RSPParam LRLRp3_param;
  LRLRp(x, -y, -phi, -sin_phi, cos_phi, &LRLRp3_param);
  const double LRLRp3_lengths[4] = {LRLRp3_param.t, LRLRp3_param.u,
                                    LRLRp3_param.u, LRLRp3_param.v};
  const char LRLRp3_types[] = "RLRL";
  if (LRLRp3_param.flag &&
      !SetRSP(4, LRLRp3_lengths, LRLRp3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp3_param";
    return false;
  }

  RSPParam LRLRp4_param;
  LRLRp(-x, -y, phi, sin_phi, cos_phi, &LRLRp4_param);
  const double LRLRp4_lengths[4] = {-LRLRp4_param.t, -LRLRp4_param.u,
                                    -LRLRp4_param.u, -LRLRp4_param.v};
  const char LRLRp4_types[] = "RLRL";
  if (LRLRp4_param.flag &&
      !SetRSP(4, LRLRp4_lengths, LRLRp4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp4_param";
    return false;
  }
  return true;
}

bool CCSC(double x, double y, double phi, double sin_phi, double cos_phi,
          std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSL1_param;
  LRSL(x, y, phi, sin_phi, cos_phi, &LRSL1_param);
  const double LRSL1_lengths[4] = {LRSL1_param.t, -0.5 * M_PI, LRSL1_param.u,
                                   LRSL1_param.v};
  const char LRSL1_types[] = "LRSL";
  if (LRSL1_param.flag &&
      !SetRSP(4, LRSL1_lengths, LRSL1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL1_param";
    return false;
  }

  RSPParam LRSL2_param;
  LRSL(-x, y, -phi, -sin_phi, cos_phi, &LRSL2_param);
  const double LRSL2_lengths[4] = {-LRSL2_param.t, 0.5 * M_PI, -LRSL2_param.u,
                                   -LRSL2_param.v};
  const char LRSL2_types[] = "LRSL";
  if (LRSL2_param.flag &&
      !SetRSP(4, LRSL2_lengths, LRSL2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL2_param";
    return false;
  }

  RSPParam LRSL3_param;
  LRSL(x, -y, -phi, -sin_phi, cos_phi, &LRSL3_param);
  const double LRSL3_lengths[4] = {LRSL3_param.t, -0.5 * M_PI, LRSL3_param.u,
                                   LRSL3_param.v};
  const char LRSL3_types[] = "RLSR";
  if (LRSL3_param.flag &&
      !SetRSP(4, LRSL3_lengths, LRSL3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL3_param";
    return false;
  }

  RSPParam LRSL4_param;
  LRSL(-x, -y, phi, sin_phi, cos_phi, &LRSL4_param);
  const double LRSL4_lengths[4] = {-LRSL4_param.t, 0.5 * M_PI, -LRSL4_param.u,
                                   -LRSL4_param.v};
  const char LRSL4_types[] = "RLSR";
  if (LRSL4_param.flag &&
      !SetRSP(4, LRSL4_lengths, LRSL4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL4_param";
    return false;
  }

  RSPParam LRSR1_param;
  LRSR(x, y, phi, sin_phi, cos_phi, &LRSR1_param);
  const double LRSR1_lengths[4] = {LRSR1_param.t, -0.5 * M_PI, LRSR1_param.u,
                                   LRSR1_param.v};
  const char LRSR1_types[] = "LRSR";
  if (LRSR1_param.flag &&
      !SetRSP(4, LRSR1_lengths, LRSR1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR1_param";
    return false;
  }

  RSPParam LRSR2_param;
  LRSR(-x, y, -phi, -sin_phi, cos_phi, &LRSR2_param);
  const double LRSR2_lengths[4] = {-LRSR2_param.t, 0.5 * M_PI, -LRSR2_param.u,
                                   -LRSR2_param.v};
  const char LRSR2_types[] = "LRSR";
  if (LRSR2_param.flag &&
      !SetRSP(4, LRSR2_lengths, LRSR2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR2_param";
    return false;
  }

  RSPParam LRSR3_param;
  LRSR(x, -y, -phi, -sin_phi, cos_phi, &LRSR3_param);
  const double LRSR3_lengths[4] = {LRSR3_param.t, -0.5 * M_PI, LRSR3_param.u,
                                   LRSR3_param.v};
  const char LRSR3_types[] = "RLSL";
  if (LRSR3_param.flag &&
      !SetRSP(4, LRSR3_lengths, LRSR3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR3_param";
    return false;
  }

  RSPParam LRSR4_param;
  LRSR(-x, -y, phi, sin_phi, cos_phi, &LRSR4_param);
  const double LRSR4_lengths[4] = {-LRSR4_param.t, 0.5 * M_PI, -LRSR4_param.u,
                                   -LRSR4_param.v};
  const char LRSR4_types[] = "RLSL";
  if (LRSR4_param.flag &&
      !SetRSP(4, LRSR4_lengths, LRSR4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR4_param";
    return false;
  }

  // backward
  const double xb = x * cos_phi + y * sin_phi;
  const double yb = x * sin_phi - y * cos_phi;

  RSPParam LRSL5_param;
  LRSL(xb, yb, phi, sin_phi, cos_phi, &LRSL5_param);
  const double LRSL5_lengths[4] = {LRSL5_param.v, LRSL5_param.u, -0.5 * M_PI,
                                   LRSL5_param.t};
  const char LRSL5_types[] = "LSRL";
  if (LRSL5_param.flag &&
      !SetRSP(4, LRSL5_lengths, LRSL5_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRSL6_param;
  LRSL(-xb, yb, -phi, -sin_phi, cos_phi, &LRSL6_param);
  const double LRSL6_lengths[4] = {-LRSL6_param.v, -LRSL6_param.u, 0.5 * M_PI,
                                   -LRSL6_param.t};
  const char LRSL6_types[] = "LSRL";
  if (LRSL6_param.flag &&
      !SetRSP(4, LRSL6_lengths, LRSL6_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL6_param";
    return false;
  }

  RSPParam LRSL7_param;
  LRSL(xb, -yb, -phi, -sin_phi, cos_phi, &LRSL7_param);
  const double LRSL7_lengths[4] = {LRSL7_param.v, LRSL7_param.u, -0.5 * M_PI,
                                   LRSL7_param.t};
  const char LRSL7_types[] = "RSLR";
  if (LRSL7_param.flag &&
      !SetRSP(4, LRSL7_lengths, LRSL7_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL7_param";
    return false;
  }

  RSPParam LRSL8_param;
  LRSL(-xb, -yb, phi, sin_phi, cos_phi, &LRSL8_param);
  const double LRSL8_lengths[4] = {-LRSL8_param.v, -LRSL8_param.u, 0.5 * M_PI,
                                   -LRSL8_param.t};
  const char LRSL8_types[] = "RSLR";
  if (LRSL8_param.flag &&
      !SetRSP(4, LRSL8_lengths, LRSL8_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL8_param";
    return false;
  }

  RSPParam LRSR5_param;
  LRSR(xb, yb, phi, sin_phi, cos_phi, &LRSR5_param);
  const double LRSR5_lengths[4] = {LRSR5_param.v, LRSR5_param.u, -0.5 * M_PI,
                                   LRSR5_param.t};
  const char LRSR5_types[] = "RSRL";
  if (LRSR5_param.flag &&
      !SetRSP(4, LRSR5_lengths, LRSR5_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR5_param";
    return false;
  }

  RSPParam LRSR6_param;
  LRSR(-xb, yb, -phi, -sin_phi, cos_phi, &LRSR6_param);
  const double LRSR6_lengths[4] = {-LRSR6_param.v, -LRSR6_param.u, 0.5 * M_PI,
                                   -LRSR6_param.t};
  const char LRSR6_types[] = "RSRL";
  if (LRSR6_param.flag &&
      !SetRSP(4, LRSR6_lengths, LRSR6_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR6_param";
    return false;
  }

  RSPParam LRSR7_param;
  LRSR(xb, -yb, -phi, -sin_phi, cos_phi, &LRSR7_param);
  const double LRSR7_lengths[4] = {LRSR7_param.v, LRSR7_param.u, -0.5 * M_PI,
                                   LRSR7_param.t};
  const char LRSR7_types[] = "LSLR";
  if (LRSR7_param.flag &&
      !SetRSP(4, LRSR7_lengths, LRSR7_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR7_param";
    return false;
  }

  RSPParam LRSR8_param;
  LRSR(-xb, -yb, phi, sin_phi, cos_phi, &LRSR8_param);
  const double LRSR8_lengths[4] = {-LRSR8_param.v, -LRSR8_param.u, 0.5 * M_PI,
                                   -LRSR8_param.t};
  const char LRSR8_types[] = "LSLR";
  if (LRSR8_param.flag &&
      !SetRSP(4, LRSR8_lengths, LRSR8_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR8_param";
    return false;
  }
  return true;
}

bool CCSCC(double x, double y, double phi, double sin_phi, double cos_phi,
           std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSLR1_param;
  LRSLR(x, y, phi, sin_phi, cos_phi, &LRSLR1_param);
  const double LRSLR1_lengths[5] = {LRSLR1_param.t, -0.5 * M_PI, LRSLR1_param.u,
                                    -0.5 * M_PI, LRSLR1_param.v};
  const char LRSLR1_types[] = "LRSLR";
  if (LRSLR1_param.flag &&
      !SetRSP(5, LRSLR1_lengths, LRSLR1_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR1_param";
    return false;
  }

  RSPParam LRSLR2_param;
  LRSLR(-x, y, -phi, -sin_phi, cos_phi, &LRSLR2_param);
  const double LRSLR2_lengths[5] = {-LRSLR2_param.t, 0.5 * M_PI,
                                    -LRSLR2_param.u, 0.5 * M_PI,
                                    -LRSLR2_param.v};
  const char LRSLR2_types[] = "LRSLR";
  if (LRSLR2_param.flag &&
      !SetRSP(5, LRSLR2_lengths, LRSLR2_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR2_param";
    return false;
  }

  RSPParam LRSLR3_param;
  LRSLR(x, -y, -phi, -sin_phi, cos_phi, &LRSLR3_param);
  const double LRSLR3_lengths[5] = {LRSLR3_param.t, -0.5 * M_PI, LRSLR3_param.u,
                                    -0.5 * M_PI, LRSLR3_param.v};
  const char LRSLR3_types[] = "RLSRL";
  if (LRSLR3_param.flag &&
      !SetRSP(5, LRSLR3_lengths, LRSLR3_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR3_param";
    return false;
  }

  RSPParam LRSLR4_param;
  LRSLR(-x, -y, phi, sin_phi, cos_phi, &LRSLR4_param);
  const double LRSLR4_lengths[5] = {-LRSLR4_param.t, 0.5 * M_PI,
                                    -LRSLR4_param.u, 0.5 * M_PI,
                                    -LRSLR4_param.v};
  const char LRSLR4_types[] = "RLSRL";
  if (LRSLR4_param.flag &&
      !SetRSP(5, LRSLR4_lengths, LRSLR4_types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR4_param";
    return false;
  }
  return true;
}

bool GenerateRSP(const Node3d& start_node, const Node3d& end_node,
                 double max_kappa,
                 std::vector<ReedSheppPath>* all_possible_paths) {
  const double dx = end_node.x() - start_node.x();
  const double dy = end_node.y() - start_node.y();
  const double dphi = end_node.theta() - start_node.theta();
  const double sin_dphi = fast_math::Sin(dphi);
  const double cos_dphi = fast_math::Cos(dphi);
  const double c = start_node.cos_theta();
  const double s = start_node.sin_theta();
  // Normalize the initial point to (0,0,0).
  const double x = (c * dx + s * dy) * max_kappa;
  const double y = (-s * dx + c * dy) * max_kappa;
  if (!SCS(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at SCS";
  }
  if (!CSC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CSC";
  }
  if (!CCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCC";
  }
  if (!CCCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCCC";
  }
  if (!CCSC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCSC";
  }
  if (!CCSCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCSCC";
  }
  if (all_possible_paths->empty()) {
    VLOG(3) << "No path generated by certain two configurations";
    return false;
  }
  return true;
}

absl::StatusOr<ReedSheppPath> GetShortestReedsShepp(const Node3d& start_node,
                                                    const Node3d& end_node,
                                                    double max_kappa) {
  QCHECK_GT(max_kappa, 0.0);
  std::vector<ReedSheppPath> all_possible_paths;
  if (!GenerateRSP(start_node, end_node, max_kappa, &all_possible_paths)) {
    return absl::UnavailableError("Fail to generate Reeds Shepp curve!");
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  int optimal_path_index = 0;
  for (int i = 0; i < all_possible_paths.size(); ++i) {
    if (all_possible_paths[i].total_length > 0.0 &&
        all_possible_paths[i].total_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = all_possible_paths[i].total_length;
    }
  }

  ReedSheppPath res(std::move(all_possible_paths[optimal_path_index]));
  const double scale = 1.0 / max_kappa;
  for (double& segs_length : res.segs_lengths) {
    segs_length *= scale;
  }
  res.total_length *= scale;

  // The answer maybe incorrect and need to check the end point.
  return res;
}

struct Motion {
  double delta_x;
  double delta_y;
  double delta_theta;
  double kappa;
  double s;
  bool forward;
};

std::vector<ReedSheppPoint> GetSampledShortestReedsShepp(
    const Node3d& start_node, const Node3d& end_node, double max_kappa,
    double resolution) {
  QCHECK_GT(resolution, 0.0);
  std::vector<ReedSheppPoint> res;
  const auto rs_path = GetShortestReedsShepp(start_node, end_node, max_kappa);
  if (!rs_path.ok()) return res;

  const auto compute_motion = [&](double length, char type) -> Motion {
    const bool forward = (length > 0.0);
    const double sign = (forward ? 1.0 : -1.0);
    switch (type) {
      case 'L': {
        const double delta_theta = max_kappa * length;
        const double sin_theta = fast_math::Sin(delta_theta * 0.5);
        const double cos_theta = fast_math::Cos(delta_theta * 0.5);
        const double delta_x = 2.0 * sin_theta * cos_theta / max_kappa;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_kappa;
        return {delta_x,          delta_y,          delta_theta,
                sign * max_kappa, std::abs(length), forward};
      }
      case 'R': {
        const double delta_theta = -max_kappa * length;
        const double sin_theta = fast_math::Sin(delta_theta * 0.5);
        const double cos_theta = fast_math::Cos(delta_theta * 0.5);
        const double delta_x = 2.0 * sin_theta * cos_theta / max_kappa;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_kappa;
        return {-delta_x,         -delta_y,         delta_theta,
                sign * max_kappa, std::abs(length), forward};
      }
      case 'S':
        return {sign * std::abs(length), 0.0,    0.0, 0.0,
                std::abs(length),        forward};
    }
    return {0.0, 0.0, 0.0, 0.0, 0.0, false};
  };

  QCHECK_EQ(rs_path->segs_lengths.size(), rs_path->segs_types.size());
  // Record the last node.
  ReedSheppPoint last_point = {start_node.x(), start_node.y(),
                               start_node.theta(), start_node.forward()};
  for (int i = 0; i < rs_path->segs_lengths.size(); ++i) {
    const double length = rs_path->segs_lengths[i];
    const char type = rs_path->segs_types[i];
    Motion motion = compute_motion(std::copysign(resolution, length), type);
    double s = 0.0;
    constexpr double kEpsilon = 0.01;
    while (s + kEpsilon < std::abs(length)) {
      double s_increment = resolution;
      if (s + s_increment >= std::abs(length)) {
        s_increment = std::abs(length) - s;
        motion = compute_motion(std::copysign(s_increment, length), type);
      }
      s += s_increment;
      const double next_x = last_point.x +
                            motion.delta_x * fast_math::Cos(last_point.theta) -
                            motion.delta_y * fast_math::Sin(last_point.theta);
      const double next_y = last_point.y +
                            motion.delta_x * fast_math::Sin(last_point.theta) +
                            motion.delta_y * fast_math::Cos(last_point.theta);
      const double next_theta =
          NormalizeAngle(last_point.theta + motion.delta_theta);
      res.push_back({next_x, next_y, next_theta, motion.forward});
      last_point = res.back();
    }
  }
  // Check if the last point is close to end.
  constexpr double kMaxXYError = 0.05;
  constexpr double kMaxThetaError = 0.01;
  const double xy_error =
      Hypot(res.back().x - end_node.x(), res.back().y - end_node.y());
  const double theta_error =
      std::abs(NormalizeAngle(res.back().theta - end_node.theta()));
  if (xy_error > kMaxXYError || theta_error > kMaxThetaError) {
    res.clear();
    VLOG(2) << "RS extension has a big error, xy error = " << xy_error
            << ", theta error = " << theta_error;
  }
  return res;
}

}  // namespace planner
}  // namespace qcraft
