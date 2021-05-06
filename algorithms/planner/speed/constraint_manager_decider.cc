#include "onboard/planner/speed/constraint_manager_decider.h"

#include "onboard/lite/logging.h"

namespace qcraft {
namespace planner {
namespace {

void MakeConstraintDecisionForStBoundary(
    const ConstraintManager& constraint_mgr,
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  const auto& st_boundary = *st_boundary_with_decision->raw_st_boundary();
  if (st_boundary.source_type() != StBoundary::SourceType::ST_OBJECT) {
    return;
  }
  QCHECK(st_boundary.traj_id().has_value());
  if (constraint_mgr.IsLeadingObject(*st_boundary.traj_id())) {
    st_boundary_with_decision->set_decision_type(StBoundaryProto::FOLLOW);
    st_boundary_with_decision->set_decision_reason(
        StBoundaryProto::CONSTRAINT_MGR_DECIDER);
  }
}

}  // namespace

void MakeConstraintDecisionForStBoundaries(
    const ConstraintManager& constraint_mgr,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd) {
  QCHECK_NOTNULL(st_boundaries_wd);
  for (auto& st_boundary_wd : *st_boundaries_wd) {
    MakeConstraintDecisionForStBoundary(constraint_mgr, &st_boundary_wd);
  }
}

}  // namespace planner
}  // namespace qcraft
