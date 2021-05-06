#ifndef ONBOARD_MATH_QP_QPOASES_SOLVER_H_
#define ONBOARD_MATH_QP_QPOASES_SOLVER_H_

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#include "qpOASES/qpOASES.hpp"
#pragma clang diagnostic pop

#include "onboard/math/qp/dense_qp_solver.h"

namespace qcraft {

// Wrapper for the QPOASES active set solver.
class QpoasesSolver : public DenseQpSolver {
 public:
  QpoasesSolver(const MatXd &A, const VecXd &b, const MatXd &G, const VecXd &g,
                const MatXd &H, const VecXd &h);
  virtual ~QpoasesSolver() = default;

  bool Solve() override;
  bool Solve(const qpOASES::Options &options);
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_QP_QPOASES_SOLVER_H_
