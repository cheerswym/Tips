#ifndef ONBOARD_MATH_QP_OSQP_SOLVER_H_
#define ONBOARD_MATH_QP_OSQP_SOLVER_H_

#include "osqp/osqp.h"

#include "onboard/math/qp/sparse_qp_solver.h"

namespace qcraft {

// Wrapper for the OSQP sparse solver.
class OsqpSolver : public SparseQpSolver {
 public:
  OsqpSolver(const SMatXd &A, const VecXd &b, const SMatXd &G, const VecXd &g,
             const SMatXd &H, const VecXd &h);
  virtual ~OsqpSolver() = default;

  bool Solve() override;
  bool Solve(const OSQPSettings &settings);
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_QP_OSQP_SOLVER_H_
