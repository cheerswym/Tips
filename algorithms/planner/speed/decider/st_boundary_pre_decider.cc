#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEpsilon = 0.01;

void MakeHitCurrentAvPositionIgnoreDecision(
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  // If a prediction trajectory
  const StBoundary* st_boundary = st_boundary_with_decision->st_boundary();
  if (st_boundary->bottom_left_point().s() <= kEpsilon &&
      st_boundary->bottom_left_point().t() >= 0.0) {
    st_boundary_with_decision->set_decision_type(StBoundaryProto::IGNORE);
    st_boundary_with_decision->set_decision_reason(
        StBoundaryProto::IGNORE_TRACKER_DECIDER);
    st_boundary_with_decision->set_decision_info(
        "ignore hit current Av position");
    return;
  }
}

// Currently only for freespace planner
void MakeIgnoreDecisionForDistantObjects(
    StBoundaryWithDecision* st_boundary_with_decision) {
  // Do not make decision if already have one.
  if (st_boundary_with_decision->decision_type() != StBoundaryProto::UNKNOWN) {
    return;
  }
  // If a prediction trajectory && too far (ttc > 5s) from current pose
  if (st_boundary_with_decision->st_boundary()->bottom_left_point().t() > 5.0) {
    st_boundary_with_decision->set_decision_type(StBoundaryProto::IGNORE);
    st_boundary_with_decision->set_decision_reason(
        StBoundaryProto::IGNORE_TRACKER_DECIDER);
    st_boundary_with_decision->set_decision_info("ignore too distant objects");
  }
}

}  // namespace

void MakeFreespaceStBoundaryPreDecision(
    const FreespaceStBoundaryPreDecisionInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  QCHECK_NOTNULL(st_boundaries_with_decision);
  for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
    if (st_boundary_with_decision.st_boundary()->source_type() ==
        StBoundary::SourceType::ST_OBJECT) {
      MakeHitCurrentAvPositionIgnoreDecision(&st_boundary_with_decision);
      MakeIgnoreDecisionForDistantObjects(&st_boundary_with_decision);
    }
  }
}

}  // namespace planner
}  // namespace qcraft
