#include "onboard/planner/optimization/ddp/ddp_optimizer.h"

DEFINE_int32(planner_dopt_canvas_level, 0,
             "DdpOptimizer canvas verbosity level.");
DEFINE_bool(
    planner_dopt_symmetrize_j_hessian, true,
    "Whether to manually symmetrize ddJdxdx at every backward pass step.");
DEFINE_bool(enable_stepsize_after_line_search, true,
            "Whether to try step_size_adjustment if line search failed.");

namespace qcraft {
namespace planner {}  // namespace planner
}  // namespace qcraft
